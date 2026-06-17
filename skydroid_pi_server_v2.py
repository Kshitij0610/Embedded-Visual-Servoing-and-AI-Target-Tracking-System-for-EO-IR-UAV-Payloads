import cv2
import socket
import threading
import time
import struct
import json
import numpy as np

try:
    from ultralytics import YOLO
    yolo_model = YOLO('yolov8n.onnx', task='detect')
    has_yolo = True
except ImportError:
    yolo_model = None
    has_yolo = False
except Exception as e:
    print(f"[Warning] Failed to load YOLO ONNX model: {e}")
    yolo_model = None
    has_yolo = False

# --- Target Resolution for processing (360p) ---
TARGET_W = 640
TARGET_H = 360

class VideoStream:
    """Threaded video stream reader — identical pattern to the proven v1 code."""
    def __init__(self, src):
        self.src = src
        self.cap = cv2.VideoCapture(src, cv2.CAP_FFMPEG)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.ret, self.frame = self.cap.read()
        if self.ret and self.frame is not None:
            self.frame = cv2.resize(self.frame, (TARGET_W, TARGET_H))
        self.running = True
        self.thread = threading.Thread(target=self.update, args=())
        self.thread.daemon = True
        self.thread.start()

    def update(self):
        while self.running:
            if self.cap.isOpened():
                ret, frame = self.cap.read()
                if ret:
                    self.ret = ret
                    self.frame = cv2.resize(frame, (TARGET_W, TARGET_H))
            else:
                time.sleep(0.01)

    def read(self):
        return self.ret, self.frame

    def stop(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=2.0)
        self.cap.release()

class LowPassFilter:
    def __init__(self, alpha=0.5):
        self.alpha = alpha
        self.val = 0.0
    def reset(self):
        self.val = 0.0
    def update(self, new_val):
        self.val = self.alpha * new_val + (1.0 - self.alpha) * self.val
        return self.val

class PIDController:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, max_out=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()

    def update(self, error):
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0.0:
            dt = 1e-4

        self.integral += error * dt
        integral_max = self.max_out / (self.ki + 1e-6)
        self.integral = max(min(self.integral, integral_max), -integral_max)

        derivative = (error - self.prev_error) / dt
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        self.prev_error = error
        self.last_time = current_time

        return max(min(output, self.max_out), -self.max_out)

class SkydroidGimbal:
    def __init__(self, ip="192.168.144.108"):
        self.ip = ip
        self.ports = [9002, 5000, 1030]
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        print("[Gimbal] Connecting and enabling manual attitude control...")
        self._send_command('#TPUG2wGAA01')
        time.sleep(0.5)

    def get_crc_command(self, cmd_str: str) -> bytes:
        total = sum(cmd_str.encode('utf-8'))
        crc_hex = f"{total & 0xFF:02X}"
        return (cmd_str + crc_hex).encode('utf-8')

    def _send_command(self, cmd_str: str):
        cmd_bytes = self.get_crc_command(cmd_str)
        for port in self.ports:
            try:
                self.sock.sendto(cmd_bytes, (self.ip, port))
            except Exception:
                pass

    def to_hex_byte(self, val: int) -> str:
        val = max(min(val, 100), -100)
        if val < 0:
            val = 256 + val
        return f"{val & 0xFF:02X}"

    def set_speed(self, yaw_speed: float, pitch_speed: float):
        y_val = int(yaw_speed * 100)
        p_val = int(pitch_speed * 100)
        cmd = f"#TPUG4wGSM{self.to_hex_byte(y_val)}{self.to_hex_byte(p_val)}"
        self._send_command(cmd)

    def stop(self):
        self._send_command('#TPUG2wPTZ00')

    def center(self):
        self._send_command('#TPUG2wPTZ05')


# Global State
state_lock = threading.Lock()
shared_frame = None
shared_metadata = {}

tracker = None          # Legacy MIL Tracker
mode = 'NORMAL'         # 'NORMAL' or 'YOLO'
yolo_locked_id = None
yolo_missing_frames = 0
current_stream_idx = 1  # 1 = RGB, 2 = Thermal

# PID Controllers and LPF
pid_yaw = PIDController()
pid_pitch = PIDController()
lpf_x = LowPassFilter(alpha=0.8)  # Higher alpha = faster gimbal response
lpf_y = LowPassFilter(alpha=0.8)

gimbal = None
video = None

# Default Tuning
inv_x = 1.0
inv_y = -1.0
global_kd = 0.0

def get_kd_scaled():
    # If in YOLO mode, drastically reduce Kd to prevent jitter from inference dead-time
    if mode == 'YOLO':
        return global_kd * 0.1
    return global_kd

def switch_camera_stream():
    """Switch camera stream in a separate thread to avoid blocking the UDP listener."""
    global video, current_stream_idx
    current_stream_idx = 2 if current_stream_idx == 1 else 1
    # RGB = port 554, stream=1 | Thermal = port 555, stream=2
    if current_stream_idx == 1:
        new_url = "rtsp://192.168.144.108:554/stream=1"
    else:
        new_url = "rtsp://192.168.144.108:555/stream=2"
    print(f"[Camera] Switching to {'Thermal' if current_stream_idx == 2 else 'RGB'} ({new_url})")
    old_video = video
    # Create new stream first, then stop old one
    new_video = VideoStream(new_url)
    video = new_video
    if old_video:
        old_video.stop()
    print(f"[Camera] Stream switch complete.")

def udp_command_listener():
    global tracker, inv_x, inv_y, mode, yolo_locked_id, global_kd
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', 5005))
    print("[UDP Server] Listening for commands on port 5005...")
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            msg = json.loads(data.decode('utf-8'))
            action = msg.get('action')

            with state_lock:
                if action == 'tune':
                    pid_yaw.kp = msg['kp']; pid_pitch.kp = msg['kp']
                    pid_yaw.ki = msg['ki']; pid_pitch.ki = msg['ki']
                    global_kd = msg['kd']

                    kd_scaled = get_kd_scaled()
                    pid_yaw.kd = kd_scaled; pid_pitch.kd = kd_scaled

                    pid_yaw.max_out = msg['max_spd']; pid_pitch.max_out = msg['max_spd']
                    inv_x = -1.0 if msg['inv_x'] else 1.0
                    inv_y = -1.0 if msg['inv_y'] else 1.0

                elif action == 'mode_switch':
                    new_mode = msg.get('mode', 'NORMAL')
                    print(f"[State] Switching from {mode} to {new_mode}")
                    mode = new_mode
                    tracker = None
                    yolo_locked_id = None
                    kd_scaled = get_kd_scaled()
                    pid_yaw.kd = kd_scaled; pid_pitch.kd = kd_scaled
                    pid_yaw.reset(); pid_pitch.reset()
                    lpf_x.reset(); lpf_y.reset()
                    if gimbal: gimbal.stop()

                elif action == 'camera_mode':
                    pass  # handled outside lock below

                elif action == 'track':
                    if mode == 'NORMAL' and shared_frame is not None:
                        bbox = msg['bbox']
                        tracker = cv2.TrackerMIL_create()
                        tracker.init(shared_frame, tuple(bbox))
                        pid_yaw.reset(); pid_pitch.reset()
                        lpf_x.reset(); lpf_y.reset()
                        print(f"[Tracker] Legacy MIL Initialized with bbox {bbox}")

                elif action == 'lock_id':
                    if mode == 'YOLO':
                        yolo_locked_id = msg.get('id')
                        pid_yaw.reset(); pid_pitch.reset()
                        lpf_x.reset(); lpf_y.reset()
                        print(f"[Tracker] YOLO Locked onto ID {yolo_locked_id}")

                elif action == 'stop':
                    tracker = None
                    yolo_locked_id = None
                    if gimbal:
                        gimbal.stop()
                        gimbal.center()
                        print("[Tracker] Stopped & Centered.")

            # Camera switch must happen outside state_lock to avoid deadlock
            if action == 'camera_mode':
                threading.Thread(target=switch_camera_stream, daemon=True).start()

        except Exception as e:
            pass  # ignore malformed packets

def tcp_video_server():
    """Sends metadata + JPEG frames to the laptop client over TCP."""
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('0.0.0.0', 5000))
    server_socket.listen(1)
    print("[TCP Server] Waiting for laptop on port 5000...")

    while True:
        conn, addr = server_socket.accept()
        print(f"[TCP Server] Client connected from {addr}")
        try:
            while True:
                with state_lock:
                    frame = shared_frame.copy() if shared_frame is not None else None
                    metadata = shared_metadata.copy()

                if frame is not None:
                    ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
                    if ret:
                        img_data = buffer.tobytes()
                        meta_data = json.dumps(metadata).encode('utf-8')

                        # Protocol: [4B meta_len] [meta_data] [4B img_len] [img_data]
                        packet = struct.pack("<L", len(meta_data)) + meta_data + \
                                 struct.pack("<L", len(img_data)) + img_data

                        conn.sendall(packet)
                time.sleep(0.03)  # Cap to ~30 fps
        except Exception as e:
            print(f"[TCP Server] Client disconnected: {e}")
            conn.close()

def main():
    global shared_frame, shared_metadata, tracker, gimbal, video, mode, yolo_locked_id, yolo_missing_frames

    gimbal = SkydroidGimbal()
    video = VideoStream(f"rtsp://192.168.144.108:554/stream={current_stream_idx}")

    threading.Thread(target=udp_command_listener, daemon=True).start()
    threading.Thread(target=tcp_video_server, daemon=True).start()

    print("[Main] Server processing loop started.")

    # Target classes: person(0), car(2) for YOLOv8 default COCO
    ALLOWED_CLASSES = [0, 2]

    try:
        while True:
            ret, frame = video.read()
            if not ret or frame is None:
                # Generate a "NO SIGNAL" frame so the client doesn't timeout
                dummy = np.zeros((TARGET_H, TARGET_W, 3), dtype=np.uint8)
                cv2.putText(dummy, "NO SIGNAL", (180, 190), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)
                with state_lock:
                    shared_metadata = {"mode": mode, "state": "NO_SIGNAL", "bboxes": [], "locked_id": None}
                    shared_frame = dummy
                time.sleep(0.1)
                continue

            h_img, w_img = frame.shape[:2]
            center_x, center_y = w_img // 2, h_img // 2

            metadata = {
                "mode": mode,
                "state": "IDLE",
                "bboxes": [],
                "locked_id": None
            }

            with state_lock:
                current_mode = mode
                curr_tracker = tracker
                curr_yolo_locked_id = yolo_locked_id

            disp_frame = frame.copy()

            if current_mode == 'NORMAL':
                if curr_tracker is not None:
                    metadata["state"] = "TRACKING"
                    success, box = curr_tracker.update(frame)
                    if success:
                        x, y, w, h = [int(v) for v in box]
                        cv2.rectangle(disp_frame, (x, y), (x+w, y+h), (255,0,0), 2)
                        metadata["bboxes"].append({"id": 0, "bbox": [x,y,w,h], "cls": -1})

                        obj_cx = x + w // 2
                        obj_cy = y + h // 2
                        cv2.line(disp_frame, (center_x, center_y), (obj_cx, obj_cy), (0,0,255), 2)

                        err_x = lpf_x.update(obj_cx - center_x)
                        err_y = lpf_y.update(obj_cy - center_y)

                        yaw_cmd = pid_yaw.update(err_x) * inv_x
                        pitch_cmd = pid_pitch.update(err_y) * inv_y
                        gimbal.set_speed(yaw_cmd, pitch_cmd)

                        cv2.putText(disp_frame, f"Err: ({int(err_x)}, {int(err_y)})", (15, h_img - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
                        cv2.putText(disp_frame, f"Cmd: ({yaw_cmd:.2f}, {pitch_cmd:.2f})", (15, h_img - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
                    else:
                        with state_lock: tracker = None
                        gimbal.stop()
                else:
                    cv2.drawMarker(disp_frame, (center_x, center_y), (0, 255, 255), cv2.MARKER_CROSS, 20, 2)

            elif current_mode == 'YOLO' and has_yolo:
                # Run YOLO ByteTrack
                results = yolo_model.track(frame, persist=True, classes=ALLOWED_CLASSES, verbose=False, imgsz=320)

                boxes = results[0].boxes
                tracks = []

                if boxes is not None and boxes.id is not None:
                    for box, track_id, cls_id in zip(boxes.xyxy, boxes.id, boxes.cls):
                        x1, y1, x2, y2 = box.tolist()
                        tid = int(track_id.item())
                        cid = int(cls_id.item())

                        w = int(x2 - x1)
                        h = int(y2 - y1)
                        x = int(x1)
                        y = int(y1)

                        tracks.append({"id": tid, "bbox": [x,y,w,h], "cls": cid})
                        metadata["bboxes"].append({"id": tid, "bbox": [x,y,w,h], "cls": cid})

                        # Draw unselected boxes
                        if tid != curr_yolo_locked_id:
                            cv2.rectangle(disp_frame, (x, y), (x+w, y+h), (0,255,0), 2)
                            cv2.putText(disp_frame, f"ID:{tid}", (x, max(0, y-5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

                if curr_yolo_locked_id is None:
                    if len(tracks) > 1:
                        metadata["state"] = "WAITING_FOR_SELECTION"
                        gimbal.stop()
                        cv2.putText(disp_frame, "MULTIPLE TARGETS - PLEASE SELECT", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,165,255), 2)
                    elif len(tracks) == 1:
                        # Auto-lock if only 1 target
                        with state_lock:
                            yolo_locked_id = tracks[0]["id"]
                            yolo_missing_frames = 0
                else:
                    metadata["state"] = "TRACKING"
                    metadata["locked_id"] = curr_yolo_locked_id

                    target = next((t for t in tracks if t["id"] == curr_yolo_locked_id), None)
                    if target:
                        yolo_missing_frames = 0
                        x, y, w, h = target["bbox"]

                        cv2.rectangle(disp_frame, (x, y), (x+w, y+h), (255,0,255), 3)
                        cv2.putText(disp_frame, f"LOCKED ID:{curr_yolo_locked_id}", (x, max(0, y-10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,255), 2)

                        obj_cx = x + w // 2
                        obj_cy = y + h // 2
                        cv2.line(disp_frame, (center_x, center_y), (obj_cx, obj_cy), (0,0,255), 2)

                        err_x = lpf_x.update(obj_cx - center_x)
                        err_y = lpf_y.update(obj_cy - center_y)

                        yaw_cmd = pid_yaw.update(err_x) * inv_x
                        pitch_cmd = pid_pitch.update(err_y) * inv_y
                        gimbal.set_speed(yaw_cmd, pitch_cmd)
                    else:
                        yolo_missing_frames += 1
                        if yolo_missing_frames > 30:
                            print(f"[Tracker] Lost YOLO ID {curr_yolo_locked_id} for 30 frames. Reverting to Selection.")
                            with state_lock:
                                yolo_locked_id = None
                            gimbal.stop()
                        else:
                            gimbal.stop()
                            cv2.putText(disp_frame, f"TARGET LOST ({yolo_missing_frames}/30)", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)

            with state_lock:
                shared_metadata = metadata
                shared_frame = disp_frame

            time.sleep(0.01)

    except KeyboardInterrupt:
        pass
    finally:
        if gimbal:
            gimbal.center()
            time.sleep(0.5)
            gimbal.stop()
        if video:
            video.stop()
        print("[Main] Server shutdown.")

if __name__ == '__main__':
    main()

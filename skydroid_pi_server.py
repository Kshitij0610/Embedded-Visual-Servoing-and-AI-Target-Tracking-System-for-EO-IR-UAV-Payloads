import cv2
import socket
import threading
import time
import struct
import json
import numpy as np

class VideoStream:
    def __init__(self, src):
        self.cap = cv2.VideoCapture(src, cv2.CAP_FFMPEG)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.ret, self.frame = self.cap.read()
        self.running = True
        self.thread = threading.Thread(target=self.update, args=())
        self.thread.daemon = True
        self.thread.start()
        
    def update(self):
        while self.running:
            if self.cap.isOpened():
                ret, frame = self.cap.read()
                if ret:
                    self.ret, self.frame = ret, frame
            else:
                time.sleep(0.01)
                
    def read(self):
        return self.ret, self.frame
        
    def stop(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join()
        self.cap.release()

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
tracker = None
pid_yaw = PIDController()
pid_pitch = PIDController()
gimbal = None

# Default Tuning
inv_x = 1.0
inv_y = -1.0 

def udp_command_listener():
    global tracker, inv_x, inv_y
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', 5005))
    print("[UDP Server] Listening for commands from Laptop on port 5005...")
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            msg = json.loads(data.decode('utf-8'))
            action = msg.get('action')
            
            if action == 'tune':
                with state_lock:
                    pid_yaw.kp = msg['kp']; pid_pitch.kp = msg['kp']
                    pid_yaw.ki = msg['ki']; pid_pitch.ki = msg['ki']
                    pid_yaw.kd = msg['kd']; pid_pitch.kd = msg['kd']
                    pid_yaw.max_out = msg['max_spd']; pid_pitch.max_out = msg['max_spd']
                    inv_x = -1.0 if msg['inv_x'] else 1.0
                    inv_y = -1.0 if msg['inv_y'] else 1.0
                    
            elif action == 'track':
                bbox = msg['bbox']
                with state_lock:
                    if shared_frame is not None:
                        tracker = cv2.TrackerMIL_create()
                        tracker.init(shared_frame, tuple(bbox))
                        pid_yaw.reset()
                        pid_pitch.reset()
                        print(f"[Tracker] Initialized with bbox {bbox}")
                        
            elif action == 'stop':
                with state_lock:
                    tracker = None
                    if gimbal:
                        gimbal.stop()
                        gimbal.center()
                        print("[Tracker] Stopped & Centered by Laptop.")
                        
        except Exception as e:
            print(f"UDP Error: {e}")

def tcp_video_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('0.0.0.0', 5000))
    server_socket.listen(1)
    print("[TCP Server] Waiting for laptop to connect for video feed on port 5000...")
    
    while True:
        conn, addr = server_socket.accept()
        print(f"[TCP Server] Client connected from {addr}")
        try:
            while True:
                with state_lock:
                    frame = shared_frame.copy() if shared_frame is not None else None
                
                if frame is not None:
                    # Compress frame to JPEG
                    ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
                    if ret:
                        data = buffer.tobytes()
                        # Send 4-byte little-endian size, followed by JPEG data
                        conn.sendall(struct.pack("<L", len(data)) + data)
                time.sleep(0.03) # Cap to ~30 fps
        except Exception as e:
            print(f"[TCP Server] Client disconnected: {e}")
            conn.close()

def main():
    global shared_frame, tracker, gimbal, inv_x, inv_y
    gimbal = SkydroidGimbal()
    
    # RPi reads directly from the ethernet-connected camera
    video = VideoStream("rtsp://192.168.144.108:554/stream=1") 
    
    # Start network threads
    threading.Thread(target=udp_command_listener, daemon=True).start()
    threading.Thread(target=tcp_video_server, daemon=True).start()
    
    print("[Main] Server processing loop started. Running Headlessly.")
    
    try:
        while True:
            ret, frame = video.read()
            if not ret or frame is None:
                continue
                
            h_img, w_img = frame.shape[:2]
            center_x, center_y = w_img // 2, h_img // 2
            
            with state_lock:
                disp_frame = frame.copy()
                if tracker is not None:
                    success, box = tracker.update(frame)
                    if success:
                        x, y, w, h = [int(v) for v in box]
                        cv2.rectangle(disp_frame, (x, y), (x+w, y+h), (255,0,0), 2)
                        
                        obj_cx = x + w // 2
                        obj_cy = y + h // 2
                        cv2.line(disp_frame, (center_x, center_y), (obj_cx, obj_cy), (0,0,255), 2)
                        
                        err_x = obj_cx - center_x
                        err_y = obj_cy - center_y
                        
                        yaw_cmd = pid_yaw.update(err_x) * inv_x
                        pitch_cmd = pid_pitch.update(err_y) * inv_y
                        
                        gimbal.set_speed(yaw_cmd, pitch_cmd)
                        
                        cv2.putText(disp_frame, f"Err: ({err_x}, {err_y})", (15, h_img - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                        cv2.putText(disp_frame, f"Cmd: ({yaw_cmd:.2f}, {pitch_cmd:.2f})", (15, h_img - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                    else:
                        tracker = None
                        gimbal.stop()
                        cv2.putText(disp_frame, "Tracking Lost", (15, h_img - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
                else:
                    cv2.drawMarker(disp_frame, (center_x, center_y), (0, 255, 255), cv2.MARKER_CROSS, 20, 2)
                    
                shared_frame = disp_frame
            
            time.sleep(0.01) # Small sleep to prevent CPU pegging
            
    except KeyboardInterrupt:
        pass
    finally:
        gimbal.center()
        time.sleep(0.5)
        gimbal.stop()
        video.stop()
        print("[Main] Server shutdown.")

if __name__ == '__main__':
    main()

import cv2
import socket
import threading
import time

class VideoStream:
    """Multithreaded RTSP video stream fetcher to ensure zero lag."""
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
    """Discrete PID controller with anti-windup."""
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
    """UDP network controller for the Skydroid C12 Gimbal."""
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

# UI Variables
drawing = False
start_pt = (-1, -1)
current_pt = (-1, -1)
new_bbox = None

def mouse_callback(event, x, y, flags, param):
    global drawing, start_pt, current_pt, new_bbox
    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        start_pt = (x, y)
        current_pt = (x, y)
        new_bbox = None
    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing:
            current_pt = (x, y)
    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        current_pt = (x, y)
        x_min = min(start_pt[0], current_pt[0])
        y_min = min(start_pt[1], current_pt[1])
        w = abs(start_pt[0] - current_pt[0])
        h = abs(start_pt[1] - current_pt[1])
        if w > 20 and h > 20:
            new_bbox = (x_min, y_min, w, h)

def main():
    global new_bbox, drawing, start_pt, current_pt
    
    gimbal = SkydroidGimbal()
    video = VideoStream("rtsp://192.168.144.108:554/stream=1")
    
    print("Waiting for video stream...")
    time.sleep(2)
    ret, frame = video.read()
    if not ret or frame is None:
        print("Failed to capture video. Check network connection.")
        video.stop()
        return

    tracker = None
    pid_yaw = PIDController()
    pid_pitch = PIDController()
    
    cv2.namedWindow('Skydroid Tracker FPV', cv2.WINDOW_NORMAL)
    cv2.setMouseCallback('Skydroid Tracker FPV', mouse_callback)
    
    def nothing(x): pass
    
    # --- UI Sliders Setup ---
    # Kp increased back to 15 now that inversion prevents runaway crashing
    cv2.createTrackbar('Kp (Speed)', 'Skydroid Tracker FPV', 15, 100, nothing)    
    cv2.createTrackbar('Ki (Steady)', 'Skydroid Tracker FPV', 0, 100, nothing)
    cv2.createTrackbar('Kd (Dampen)', 'Skydroid Tracker FPV', 1, 100, nothing)
    cv2.createTrackbar('MaxSpd %', 'Skydroid Tracker FPV', 50, 100, nothing)     
    
    # Default 'Inv PITCH' set to 1 to fix the runaway pitch
    cv2.createTrackbar('Inv YAW', 'Skydroid Tracker FPV', 0, 1, nothing)
    cv2.createTrackbar('Inv PITCH', 'Skydroid Tracker FPV', 1, 1, nothing)     
    
    try:
        while True:
            ret, frame = video.read()
            if not ret or frame is None:
                continue
                
            frame_disp = frame.copy()
            h_img, w_img = frame_disp.shape[:2]
            center_x, center_y = w_img // 2, h_img // 2
            
            # Read trackbars and map to PID
            raw_kp = cv2.getTrackbarPos('Kp (Speed)', 'Skydroid Tracker FPV')
            raw_ki = cv2.getTrackbarPos('Ki (Steady)', 'Skydroid Tracker FPV')
            raw_kd = cv2.getTrackbarPos('Kd (Dampen)', 'Skydroid Tracker FPV')
            raw_max_speed = cv2.getTrackbarPos('MaxSpd %', 'Skydroid Tracker FPV')
            
            kp = raw_kp / 10000.0
            ki = raw_ki / 10000.0
            kd = raw_kd / 10000.0
            max_speed = raw_max_speed / 100.0
            
            inv_x = -1.0 if cv2.getTrackbarPos('Inv YAW', 'Skydroid Tracker FPV') else 1.0
            inv_y = -1.0 if cv2.getTrackbarPos('Inv PITCH', 'Skydroid Tracker FPV') else 1.0
            
            pid_yaw.kp = kp; pid_pitch.kp = kp
            pid_yaw.ki = ki; pid_pitch.ki = ki
            pid_yaw.kd = kd; pid_pitch.kd = kd
            pid_yaw.max_out = max_speed; pid_pitch.max_out = max_speed

            # Display on-screen instructions
            instructions = [
                "Controls & Status",
                "-----------------",
                "Drag Mouse: Select Target",
                "ESC: Exit & Auto-Recenter",
                "C: Stop & Auto-Recenter",
                "-----------------",
                f"Slider 1 [Kp Speed] : {raw_kp}",
                f"Slider 2 [Ki Steady]: {raw_ki}",
                f"Slider 3 [Kd Dampen]: {raw_kd}",
                f"Slider 4 [MaxSpd %] : {raw_max_speed}%",
                f"Slider 5 [Inv YAW]  : {'YES' if inv_x < 0 else 'NO'}",
                f"Slider 6 [Inv PITCH]: {'YES' if inv_y < 0 else 'NO'}"
            ]
            
            y_offset = 30
            for text in instructions:
                cv2.putText(frame_disp, text, (15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                y_offset += 25

            # Initialize Tracker on mouse release
            if new_bbox is not None:
                tracker = cv2.TrackerMIL_create()
                tracker.init(frame, new_bbox)
                pid_yaw.reset()
                pid_pitch.reset()
                new_bbox = None
                print("[Tracker] Locked on target!")

            if drawing:
                gimbal.stop()
                cv2.rectangle(frame_disp, start_pt, current_pt, (0, 255, 0), 2)
                
            elif tracker is not None:
                success, box = tracker.update(frame)
                if success:
                    x, y, w, h = [int(v) for v in box]
                    cv2.rectangle(frame_disp, (x, y), (x + w, y + h), (255, 0, 0), 2)
                    
                    obj_cx = x + w // 2
                    obj_cy = y + h // 2
                    
                    cv2.line(frame_disp, (center_x, center_y), (obj_cx, obj_cy), (0, 0, 255), 2)
                    
                    # Calculate Pixel Error (Vector from Center to Target)
                    err_x = obj_cx - center_x
                    err_y = obj_cy - center_y
                    
                    # Kinematics: Calculate velocity correction
                    yaw_cmd = pid_yaw.update(err_x) * inv_x
                    pitch_cmd = pid_pitch.update(err_y) * inv_y
                    
                    # Actuation
                    gimbal.set_speed(yaw_cmd, pitch_cmd)
                    
                    # Display error & command stats
                    cv2.putText(frame_disp, f"Err: ({err_x}, {err_y})", (15, y_offset+10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                    cv2.putText(frame_disp, f"Cmd: ({yaw_cmd:.2f}, {pitch_cmd:.2f})", (15, y_offset+35), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                else:
                    cv2.putText(frame_disp, "Tracking Lost", (15, y_offset+10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    gimbal.stop()
                    
            else:
                # Idle State
                cv2.drawMarker(frame_disp, (center_x, center_y), (0, 255, 255), cv2.MARKER_CROSS, 20, 2)

            cv2.imshow('Skydroid Tracker FPV', frame_disp)
            
            key = cv2.waitKey(1) & 0xFF
            if key == 27: # ESC
                print("[Tracker] Exiting and recentering...")
                gimbal.center()
                time.sleep(0.5)
                break
            elif key == ord('c'):
                tracker = None
                print("[Tracker] Cleared. Recentering...")
                gimbal.center()

    except KeyboardInterrupt:
        print("[Tracker] Interrupted. Recentering...")
        gimbal.center()
        time.sleep(0.5)
    finally:
        gimbal.stop()
        video.stop()
        cv2.destroyAllWindows()
        print("Shutdown complete.")

if __name__ == '__main__':
    main()

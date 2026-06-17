import cv2
import socket
import struct
import json
import threading
import time
import numpy as np

# Network Config
PI_IP = "rpi5.local"
TCP_PORT = 5000
UDP_PORT = 5005

udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# UI Variables
drawing = False
start_pt = (-1, -1)
current_pt = (-1, -1)
frame_disp = None
frame_lock = threading.Lock()

def send_command(msg_dict):
    """Sends a JSON command to the Raspberry Pi over UDP."""
    try:
        data = json.dumps(msg_dict).encode('utf-8')
        udp_sock.sendto(data, (PI_IP, UDP_PORT))
    except Exception as e:
        print(f"UDP send error: {e}")

def mouse_callback(event, x, y, flags, param):
    """Handles drawing the bounding box on the laptop GUI."""
    global drawing, start_pt, current_pt
    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        start_pt = (x, y)
        current_pt = (x, y)
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
            # Send the selected bounding box to the Pi
            send_command({"action": "track", "bbox": [x_min, y_min, w, h]})
            print(f"[Client] Sent tracking box to Pi: {(x_min, y_min, w, h)}")

def tcp_video_client():
    """Connects to the Pi to receive the processed MJPEG video stream."""
    global frame_disp
    while True:
        try:
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.settimeout(5.0)
            client_socket.connect((PI_IP, TCP_PORT))
            print("[Client] Connected to Pi video stream.")
            
            data = b""
            payload_size = struct.calcsize("<L")
            
            while True:
                # Read 4-byte header containing the JPEG size
                while len(data) < payload_size:
                    packet = client_socket.recv(4096)
                    if not packet: break
                    data += packet
                if len(data) < payload_size:
                    break
                    
                packed_msg_size = data[:payload_size]
                data = data[payload_size:]
                msg_size = struct.unpack("<L", packed_msg_size)[0]
                
                # Read the full JPEG payload
                while len(data) < msg_size:
                    packet = client_socket.recv(4096)
                    if not packet: break
                    data += packet
                if len(data) < msg_size:
                    break
                    
                frame_data = data[:msg_size]
                data = data[msg_size:]
                
                # Decode JPEG to Numpy Array
                frame_np = cv2.imdecode(np.frombuffer(frame_data, np.uint8), cv2.IMREAD_COLOR)
                if frame_np is not None:
                    with frame_lock:
                        frame_disp = frame_np
                        
        except Exception as e:
            print(f"[Client] Connection dropped: {e}. Retrying in 2 seconds...")
            time.sleep(2)

def main():
    # Start the TCP Video thread
    threading.Thread(target=tcp_video_client, daemon=True).start()
    
    cv2.namedWindow('Skydroid Remote GCS', cv2.WINDOW_NORMAL)
    cv2.setMouseCallback('Skydroid Remote GCS', mouse_callback)
    
    def nothing(x): pass
    
    # Sliders
    cv2.createTrackbar('Kp (Speed)', 'Skydroid Remote GCS', 15, 100, nothing)    
    cv2.createTrackbar('Ki (Steady)', 'Skydroid Remote GCS', 0, 100, nothing)
    cv2.createTrackbar('Kd (Dampen)', 'Skydroid Remote GCS', 1, 100, nothing)
    cv2.createTrackbar('MaxSpd %', 'Skydroid Remote GCS', 50, 100, nothing)     
    
    cv2.createTrackbar('Inv YAW', 'Skydroid Remote GCS', 0, 1, nothing)
    cv2.createTrackbar('Inv PITCH', 'Skydroid Remote GCS', 1, 1, nothing)     
    
    print("\n--- Remote Ground Control Station Started ---")
    print(f"Targeting Pi at {PI_IP}")
    print("Waiting for video frames from Pi...")
    
    last_tune_time = 0
    
    try:
        while True:
            # Periodically send tune params to the Pi (every 0.5s)
            current_time = time.time()
            if current_time - last_tune_time > 0.5:
                tune_cmd = {
                    "action": "tune",
                    "kp": cv2.getTrackbarPos('Kp (Speed)', 'Skydroid Remote GCS') / 10000.0,
                    "ki": cv2.getTrackbarPos('Ki (Steady)', 'Skydroid Remote GCS') / 10000.0,
                    "kd": cv2.getTrackbarPos('Kd (Dampen)', 'Skydroid Remote GCS') / 10000.0,
                    "max_spd": cv2.getTrackbarPos('MaxSpd %', 'Skydroid Remote GCS') / 100.0,
                    "inv_x": cv2.getTrackbarPos('Inv YAW', 'Skydroid Remote GCS') == 1,
                    "inv_y": cv2.getTrackbarPos('Inv PITCH', 'Skydroid Remote GCS') == 1
                }
                send_command(tune_cmd)
                last_tune_time = current_time

            with frame_lock:
                display = frame_disp.copy() if frame_disp is not None else None
                    
            if display is not None:
                # If user is currently drawing a box on the laptop, show the preview
                if drawing:
                    cv2.rectangle(display, start_pt, current_pt, (0, 255, 0), 2)
                    
                raw_kp = cv2.getTrackbarPos('Kp (Speed)', 'Skydroid Remote GCS')
                raw_ki = cv2.getTrackbarPos('Ki (Steady)', 'Skydroid Remote GCS')
                raw_kd = cv2.getTrackbarPos('Kd (Dampen)', 'Skydroid Remote GCS')
                raw_max_speed = cv2.getTrackbarPos('MaxSpd %', 'Skydroid Remote GCS')
                inv_yaw = cv2.getTrackbarPos('Inv YAW', 'Skydroid Remote GCS')
                inv_pitch = cv2.getTrackbarPos('Inv PITCH', 'Skydroid Remote GCS')
                    
                instructions = [
                    "Client Controls",
                    "-----------------",
                    "Drag Mouse: Select Target",
                    "ESC: Exit App",
                    "C: Stop Tracking & Recenter",
                    "-----------------",
                    f"Slider 1 [Kp Speed] : {raw_kp}",
                    f"Slider 2 [Ki Steady]: {raw_ki}",
                    f"Slider 3 [Kd Dampen]: {raw_kd}",
                    f"Slider 4 [MaxSpd %] : {raw_max_speed}%",
                    f"Slider 5 [Inv YAW]  : {'YES' if inv_yaw else 'NO'}",
                    f"Slider 6 [Inv PITCH]: {'YES' if inv_pitch else 'NO'}",
                    "-----------------",
                    f"Target IP: {PI_IP}"
                ]
                
                y_offset = 30
                for text in instructions:
                    cv2.putText(display, text, (15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    y_offset += 25
                    
                cv2.imshow('Skydroid Remote GCS', display)
                
            key = cv2.waitKey(20) & 0xFF
            if key == 27:
                send_command({"action": "stop"})
                break
            elif key == ord('c'):
                send_command({"action": "stop"})
                print("[Client] Sent Stop/Recenter command.")
                
    except KeyboardInterrupt:
        pass
    finally:
        send_command({"action": "stop"})
        cv2.destroyAllWindows()
        print("Shutdown.")

if __name__ == '__main__':
    main()

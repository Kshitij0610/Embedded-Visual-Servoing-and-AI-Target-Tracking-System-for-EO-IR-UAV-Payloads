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
frame_metadata = None
frame_lock = threading.Lock()
current_mode = "NORMAL"

def send_command(msg_dict):
    """Sends a JSON command to the Raspberry Pi over UDP."""
    try:
        data = json.dumps(msg_dict).encode('utf-8')
        udp_sock.sendto(data, (PI_IP, UDP_PORT))
    except Exception as e:
        print(f"UDP send error: {e}")

def point_in_rect(pt, rect):
    x, y, w, h = rect
    return x <= pt[0] <= x + w and y <= pt[1] <= y + h

def mouse_callback(event, x, y, flags, param):
    """Handles drawing the bounding box and YOLO selection on the laptop GUI."""
    global drawing, start_pt, current_pt, current_mode, frame_metadata

    if current_mode == "NORMAL":
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
                send_command({"action": "track", "bbox": [x_min, y_min, w, h]})
                print(f"[Client] Sent NORMAL tracking box to Pi: {(x_min, y_min, w, h)}")

    elif current_mode == "YOLO":
        if event == cv2.EVENT_LBUTTONDOWN:
            # Check if click is inside any YOLO bbox
            if frame_metadata is not None and "bboxes" in frame_metadata:
                for target in frame_metadata["bboxes"]:
                    if point_in_rect((x, y), target["bbox"]):
                        target_id = target["id"]
                        send_command({"action": "lock_id", "id": target_id})
                        print(f"[Client] Locked onto YOLO ID: {target_id}")
                        break

def tcp_video_client():
    """Connects to the Pi to receive the processed metadata + MJPEG video stream."""
    global frame_disp, frame_metadata
    while True:
        try:
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.settimeout(5.0)
            client_socket.connect((PI_IP, TCP_PORT))
            print("[Client] Connected to Pi video stream.")

            data = b""
            size_fmt = "<L"
            header_size = struct.calcsize(size_fmt)

            while True:
                # Read 4-byte JSON Metadata size
                while len(data) < header_size:
                    packet = client_socket.recv(4096)
                    if not packet: raise ConnectionError("Server closed")
                    data += packet

                meta_size = struct.unpack(size_fmt, data[:header_size])[0]
                data = data[header_size:]

                # Read JSON Metadata
                while len(data) < meta_size:
                    packet = client_socket.recv(4096)
                    if not packet: raise ConnectionError("Server closed")
                    data += packet

                meta_data_bytes = data[:meta_size]
                data = data[meta_size:]

                try:
                    metadata = json.loads(meta_data_bytes.decode('utf-8'))
                except json.JSONDecodeError:
                    metadata = {}

                # Read 4-byte JPEG Image size
                while len(data) < header_size:
                    packet = client_socket.recv(4096)
                    if not packet: raise ConnectionError("Server closed")
                    data += packet

                img_size = struct.unpack(size_fmt, data[:header_size])[0]
                data = data[header_size:]

                # Read the full JPEG payload
                while len(data) < img_size:
                    packet = client_socket.recv(65536)
                    if not packet: raise ConnectionError("Server closed")
                    data += packet

                frame_data = data[:img_size]
                data = data[img_size:]

                # Decode JPEG to Numpy Array
                frame_np = cv2.imdecode(np.frombuffer(frame_data, np.uint8), cv2.IMREAD_COLOR)
                if frame_np is not None:
                    with frame_lock:
                        frame_disp = frame_np
                        frame_metadata = metadata

        except Exception as e:
            print(f"[Client] Connection dropped: {e}. Retrying in 2 seconds...")
            time.sleep(2)

def main():
    global current_mode
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
                meta = frame_metadata.copy() if frame_metadata is not None else None

            if display is not None:
                if meta:
                    server_mode = meta.get("mode", "NORMAL")
                    server_state = meta.get("state", "IDLE")

                    if server_mode != current_mode:
                        current_mode = server_mode

                    if current_mode == "YOLO" and server_state == "WAITING_FOR_SELECTION":
                        # Draw banner warning for the user
                        cv2.rectangle(display, (0, 0), (display.shape[1], 60), (0, 165, 255), -1)
                        cv2.putText(display, "MULTIPLE TARGETS DETECTED", (30, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
                        cv2.putText(display, "Click on a target to lock", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

                if current_mode == "NORMAL" and drawing:
                    cv2.rectangle(display, start_pt, current_pt, (0, 255, 0), 2)

                # Read slider values for display
                raw_kp = cv2.getTrackbarPos('Kp (Speed)', 'Skydroid Remote GCS')
                raw_ki = cv2.getTrackbarPos('Ki (Steady)', 'Skydroid Remote GCS')
                raw_kd = cv2.getTrackbarPos('Kd (Dampen)', 'Skydroid Remote GCS')
                raw_max_speed = cv2.getTrackbarPos('MaxSpd %', 'Skydroid Remote GCS')
                inv_yaw = cv2.getTrackbarPos('Inv YAW', 'Skydroid Remote GCS')
                inv_pitch = cv2.getTrackbarPos('Inv PITCH', 'Skydroid Remote GCS')

                # Build instructions — placed at top-right corner
                instructions = [
                    f"Mode: {current_mode}",
                    "-----------------",
                    "[y]: Toggle Mode",
                    "[t]: Thermal Toggle",
                    "[c]: Stop & Recenter",
                    "[esc]: Quit",
                    "-----------------",
                    f"Kp [Speed]  : {raw_kp}",
                    f"Ki [Steady] : {raw_ki}",
                    f"Kd [Dampen] : {raw_kd}",
                    f"MaxSpd      : {raw_max_speed}%",
                    f"Inv YAW     : {'YES' if inv_yaw else 'NO'}",
                    f"Inv PITCH   : {'YES' if inv_pitch else 'NO'}",
                ]

                # Calculate text position — top-right corner
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.45
                thickness = 1
                line_h = 18
                max_text_w = max(cv2.getTextSize(t, font, font_scale, thickness)[0][0] for t in instructions)
                text_x = display.shape[1] - max_text_w - 15

                # Draw semi-transparent background
                overlay = display.copy()
                bg_h = len(instructions) * line_h + 10
                cv2.rectangle(overlay, (text_x - 8, 2), (display.shape[1] - 2, bg_h + 2), (0, 0, 0), -1)
                cv2.addWeighted(overlay, 0.5, display, 0.5, 0, display)

                y_offset = 20
                for text in instructions:
                    cv2.putText(display, text, (text_x, y_offset), font, font_scale, (0, 255, 255), thickness)
                    y_offset += line_h

                cv2.imshow('Skydroid Remote GCS', display)

            key = cv2.waitKey(20) & 0xFF
            if key == 27:
                send_command({"action": "stop"})
                break
            elif key == ord('c'):
                send_command({"action": "stop"})
                print("[Client] Sent Stop/Recenter command.")
            elif key == ord('y'):
                new_mode = "YOLO" if current_mode == "NORMAL" else "NORMAL"
                send_command({"action": "mode_switch", "mode": new_mode})
                print(f"[Client] Requested Mode Switch to: {new_mode}")
            elif key == ord('t'):
                send_command({"action": "camera_mode"})
                print("[Client] Requested Camera Stream Toggle (RGB/Thermal)")

    except KeyboardInterrupt:
        pass
    finally:
        send_command({"action": "stop"})
        cv2.destroyAllWindows()
        print("Shutdown.")

if __name__ == '__main__':
    main()

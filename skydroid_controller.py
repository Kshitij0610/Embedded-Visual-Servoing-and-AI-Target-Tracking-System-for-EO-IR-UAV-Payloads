import cv2
import socket
import sys
import time

# Skydroid C12 Camera Settings
CAMERA_IP = "192.168.144.108"
RTSP_URL_RGB = f"rtsp://{CAMERA_IP}:554/stream=1"
RTSP_URL_THERMAL = f"rtsp://{CAMERA_IP}:555/stream=2"

# Ports used by Skydroid for gimbal control over UDP
UDP_PORTS = [9002, 5000, 1030]

# Skydroid Gimbal Commands
COMMANDS = {
    'UP': '#TPUG2wPTZ01',
    'DOWN': '#TPUG2wPTZ02',
    'LEFT': '#TPUG2wPTZ03',
    'RIGHT': '#TPUG2wPTZ04',
    
    # Skydroid maps X/Y/Z fine-tuning adjustments. Z-axis is Roll!
    'ROLL_LEFT': '#TPUG2wPTZ13',  # Z_SUBTRACT
    'ROLL_RIGHT': '#TPUG2wPTZ12', # Z_ADD
    
    'CENTER': '#TPUG2wPTZ05',
    'ZOOM_IN': '#TPUM2wZMC01',
    'ZOOM_OUT': '#TPUM2wZMC02',
    'ZOOM_STOP': '#TPUM2wZMC00',
    
    'RECORD_START': '#TPUD2wREC01',
    'RECORD_STOP': '#TPUD2wREC00',
    'TAKE_PHOTO': '#TPUD2wCAP01',
    
    # Gimbal Operation Modes (Unlock Yaw/Roll)
    'MODE_FOLLOW': '#TPUG2wPTZ06',
    'MODE_LOCK_HEAD': '#TPUG2wPTZ07',
    'MODE_FOLLOW_SWITCH': '#TPUG2wPTZ08',
    
    # Enable manual attitude control (Push Attitude)
    'ENABLE_MANUAL_CONTROL': '#TPUG2wGAA01',
    
    # Master STOP command for all PTZ movements
    'STOP': '#TPUG2wPTZ00'
}

def get_crc_command(cmd_str: str) -> bytes:
    """Calculates the CRC checksum used by Skydroid protocols."""
    total = sum(cmd_str.encode('utf-8'))
    crc_hex = f"{total & 0xFF:02X}"
    return (cmd_str + crc_hex).encode('utf-8')

def send_command(sock: socket.socket, cmd_name: str):
    if cmd_name in COMMANDS:
        cmd_bytes = get_crc_command(COMMANDS[cmd_name])
        for port in UDP_PORTS:
            try:
                sock.sendto(cmd_bytes, (CAMERA_IP, port))
            except Exception:
                pass
        print(f"Sent {cmd_name}: {cmd_bytes.decode('utf-8')} (to ports {UDP_PORTS})")

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # Enable Manual Control immediately on startup
    print("Enabling manual control mode on the Gimbal...")
    send_command(sock, 'ENABLE_MANUAL_CONTROL')
    time.sleep(0.5)
    
    current_mode = 'RGB'
    current_url = RTSP_URL_RGB
    target_dimensions = None
    
    print("\n--- Skydroid C12 Gimbal Control ---")
    print("Use the following keys to control the gimbal (make sure the video window is focused):")
    print("  W : UP")
    print("  S : DOWN")
    print("  A : LEFT (Yaw)")
    print("  D : RIGHT (Yaw)")
    print("  Q : ROLL LEFT")
    print("  E : ROLL RIGHT")
    print("  C : RE-CENTER")
    print("  I : ZOOM IN")
    print("  O : ZOOM OUT")
    print("  P : TAKE PHOTO")
    print("  R : START RECORDING")
    print("  T : STOP RECORDING")
    print("  M : SWITCH RGB/THERMAL MODE")
    print("\n--- Gimbal Modes (Use these if Yaw/Roll are locked) ---")
    print("  1 : MODE: FOLLOW")
    print("  2 : MODE: LOCK HEAD")
    print("  3 : MODE: FOLLOW SWITCH")
    print("  ESC : QUIT\n")

    def open_camera(url):
        print(f"Connecting to {current_mode} video stream at {url}...")
        cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not cap.isOpened():
            print(f"Error: Could not open {current_mode} stream.")
        return cap
        
    cap = open_camera(current_url)
    
    # Variables for movement stepping
    stop_command_time = 0

    try:
        while True:
            ret = False
            if cap.isOpened():
                ret, frame = cap.read()
                
            if not ret:
                print("Failed to grab frame. Reconnecting...")
                cap.release()
                time.sleep(1)
                cap = open_camera(current_url)
                continue
                
            if current_mode == 'RGB' and target_dimensions is None:
                height, width = frame.shape[:2]
                if width > 0 and height > 0:
                    target_dimensions = (width, height)
                    
            if current_mode == 'THERMAL' and target_dimensions is not None:
                frame = cv2.resize(frame, target_dimensions)
                
            cv2.imshow('Skydroid C12 FPV', frame)

            current_time = time.time()
            if stop_command_time > 0 and current_time >= stop_command_time:
                send_command(sock, 'STOP')
                stop_command_time = 0

            key = cv2.waitKey(1) & 0xFF
            
            movement_keys = {
                ord('w'): 'UP',
                ord('s'): 'DOWN',
                ord('a'): 'LEFT',
                ord('d'): 'RIGHT',
                ord('q'): 'ROLL_LEFT',
                ord('e'): 'ROLL_RIGHT'
            }
            
            if key in movement_keys:
                if stop_command_time == 0:
                    send_command(sock, movement_keys[key])
                    # Move exactly for 150ms
                    stop_command_time = time.time() + 0.15
            elif key == ord('c'):
                send_command(sock, 'CENTER')
            elif key == ord('i'):
                if stop_command_time == 0:
                    send_command(sock, 'ZOOM_IN')
                    stop_command_time = time.time() + 0.15
            elif key == ord('o'):
                if stop_command_time == 0:
                    send_command(sock, 'ZOOM_OUT')
                    stop_command_time = time.time() + 0.15
            elif key == ord('p'):
                send_command(sock, 'TAKE_PHOTO')
            elif key == ord('r'):
                send_command(sock, 'RECORD_START')
            elif key == ord('t'):
                send_command(sock, 'RECORD_STOP')
            elif key == ord('m'):
                if current_mode == 'RGB':
                    current_mode = 'THERMAL'
                    current_url = RTSP_URL_THERMAL
                else:
                    current_mode = 'RGB'
                    current_url = RTSP_URL_RGB
                cap.release()
                cap = open_camera(current_url)
            elif key == ord('1'):
                send_command(sock, 'MODE_FOLLOW')
            elif key == ord('2'):
                send_command(sock, 'MODE_LOCK_HEAD')
            elif key == ord('3'):
                send_command(sock, 'MODE_FOLLOW_SWITCH')
            elif key == 27: # ESC
                print("Exiting...")
                break
    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    print("Cleaning up resources...")
    if cap:
        cap.release()
    cv2.destroyAllWindows()
    sock.close()

if __name__ == "__main__":
    main()

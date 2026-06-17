# Embedded Visual Servoing and AI Target Tracking System for EO/IR UAV Payloads

An open-source embedded control framework for the Skydroid C12 dual-sensor gimbal, enabling real-time visual servoing, AI-based target detection and tracking, and remote ground control — designed for researchers and developers working with UAV-mounted EO/IR payloads.

<p align="center">
  <img src="https://img.shields.io/badge/Python-3.8+-3776AB?style=flat-square&logo=python&logoColor=white" />
  <img src="https://img.shields.io/badge/OpenCV-4.x-5C3EE8?style=flat-square&logo=opencv&logoColor=white" />
  <img src="https://img.shields.io/badge/YOLOv8-Ultralytics-00FFFF?style=flat-square&logo=yolo&logoColor=black" />
  <img src="https://img.shields.io/badge/Raspberry%20Pi%205-A22846?style=flat-square&logo=raspberrypi&logoColor=white" />
  <img src="https://img.shields.io/badge/RTSP-Streaming-FF6F00?style=flat-square" />
</p>

---

## Table of Contents

- [Motivation](#motivation)
- [What This Framework Enables](#what-this-framework-enables)
- [System Architecture](#system-architecture)
- [Repository Structure](#repository-structure)
- [Command Protocol Reference](#command-protocol-reference)
- [Software Modules](#software-modules)
- [Network Protocols](#network-protocols)
- [Installation](#installation)
- [Usage Guide](#usage-guide)
- [Challenges & Resolutions](#challenges--resolutions)
- [Customization Guide](#customization-guide)
- [License](#license)

---

## Motivation

Commercial UAV gimbal systems like the Skydroid C12 offer excellent hardware — dual-channel EO/IR sensors, 3-axis stabilization, and Ethernet-based video output — but lack programmatic interfaces for researchers and integrators. The vendor ecosystem is centered around a single Android FPV application, which limits the ability to:

- Integrate gimbal control into custom UAV ground control stations
- Deploy autonomous tracking algorithms on embedded companion computers
- Interface with computer vision pipelines for real-time target acquisition
- Switch between EO and IR streams programmatically during missions

This project addresses that gap by providing a fully documented, modular Python framework that exposes every gimbal function through clean APIs, along with a complete visual servoing pipeline that demonstrates how to build intelligent tracking applications on top of it.

---

## What This Framework Enables

| Capability | Description |
|:---|:---|
| **Full Gimbal Control API** | Velocity-based pan/tilt/roll, mode switching, re-centering, zoom, photo/video capture — all accessible through simple Python function calls |
| **Real-Time Visual Servoing** | Closed-loop PID control that drives gimbal axes to keep a selected target centered in the camera frame |
| **AI Target Detection** | YOLOv8 integration for automatic person and vehicle detection with persistent multi-target tracking (ByteTrack) |
| **Dual-Stream EO/IR Support** | Programmatic switching between RGB and thermal video streams at runtime |
| **Remote Ground Control Station** | Laptop-based client with live video, HUD overlays, real-time PID tuning, and target selection |
| **Headless Embedded Deployment** | Designed to run on Raspberry Pi 5 without a display — all processing on the edge |
| **Custom Protocol for Video + Metadata** | A lightweight binary multiplexing protocol that streams annotated video with JSON metadata over TCP |

### Modes of Operation

- **Normal Mode** — Operator manually selects a target region; the system locks on using OpenCV MIL tracking and servo-controls the gimbal to follow
- **YOLO Mode** — AI detects all persons and vehicles in the scene; the operator clicks to lock onto a specific target, or the system auto-locks when only one target is present

---

## System Architecture

```
 +------------------+        Wi-Fi          +----------------------+
 |   Laptop GCS     |<--------------------->|   Raspberry Pi 5     |
 |                  |   TCP: Video + Meta   |  (Companion Computer)|
 |  - OpenCV GUI    |<--------------------->|  - RTSP Capture      |
 |  - PID Sliders   |   UDP: Commands       |  - AI Detection      |
 |  - Target Select |                       |  - PID Controller    |
 |  - HUD Overlay   |                       |  - Gimbal Interface  |
 +------------------+                       +-----------+----------+
                                                      |
                                            Ethernet  |
                                            192.168.144.x
                                                      |
                                             +--------+--------+
                                             |  Skydroid C12   |
                                             |  EO/IR Gimbal   |
                                             |                 |
                                             | - RGB Camera    |
                                             | - Thermal Cam   |
                                             | - 3-Axis Stab   |
                                             +-----------------+
```

**Data Flow:**

```
C12 Camera (RTSP) --> Pi Video Thread --> AI Detection / MIL Tracking
                                              |
                                              v
                                        PID Error Compute
                                              |
                                              v
                                   UDP Gimbal Speed Commands --> C12
                                              |
                                   JPEG Encode + JSON Metadata
                                              |
                                              v
                                   TCP Stream --> Laptop GCS
```

All I/O operations (RTSP capture, UDP command listener, TCP video server) run in independent daemon threads so that network latency or client disconnections never stall the tracking pipeline.

---

## Repository Structure

```
skydroid/
|
+-- server/                          # Raspberry Pi (headless)
|   +-- pi_server_v1.py              # Basic MIL tracking + PID control
|   +-- pi_server_v2.py              # + YOLOv8, low-pass filter, metadata protocol
|   +-- yolov8n.onnx                 # YOLOv8 Nano model (auto-downloaded if absent)
|
+-- client/                          # Laptop Ground Control Station
|   +-- laptop_client_v1.py          # Basic video display + PID sliders
|   +-- laptop_client_v2.py          # + YOLO target selection, EO/IR toggle, HUD
|
+-- utils/
|   +-- manual_controller.py         # Keyboard-based gimbal tester (WASD control)
|   +-- local_tracker.py             # Standalone Pi tracker (for bench testing with display)
|
+-- reference/
|   +-- protocol_notes.md            # Documented command structure and port mappings
|   +-- jadx_src/                    # Decompiled vendor source for reference
|
+-- README.md
```

---

## Command Protocol Reference

The C12 gimbal accepts commands over UDP on ports `9002`, `5000`, and `1030` simultaneously. All commands follow a consistent frame structure:

```
#TPUG<length>w<module><payload><CRC>
```

| Module | Purpose | Example |
|:---|:---|:---|
| `GAA` | Attitude control enable | `#TPUG2wGAA01` (required before movement) |
| `PTZ` | Discrete pan/tilt/roll | `#TPUG2wPTZ01` (tilt up), `PTZ03` (pan left) |
| `GSM` | Proportional speed control | `#TPUG4wGSM32E2` (yaw +50%, pitch -30%) |
| `ZMC` | Zoom control | `#TPUM2wZMC01` (zoom in), `ZMC00` (stop) |
| `REC` | Video recording | `#TPUD2wREC01` (start), `REC00` (stop) |
| `CAP` | Still capture | `#TPUD2wCAP01` |

### CRC Calculation

```python
def compute_crc(cmd_str: str) -> bytes:
    """Simple checksum: sum of all bytes, masked to 8 bits."""
    total = sum(cmd_str.encode('utf-8'))
    crc = f"{total & 0xFF:02X}"
    return (cmd_str + crc).encode('utf-8')
```

### Speed Encoding

Yaw and pitch velocities are encoded as signed hex bytes in the range [-100, +100]:

```python
def to_hex_byte(val: int) -> str:
    val = max(min(val, 100), -100)
    if val < 0:
        val = 256 + val          # two's complement
    return f"{val & 0xFF:02X}"
```

### RTSP Stream Endpoints

| Sensor | URL | Port |
|:---|:---|:---|
| EO (RGB) | `rtsp://192.168.144.108:554/stream=1` | 554 |
| IR (Thermal) | `rtsp://192.168.144.108:555/stream=2` | 555 |

> **Note:** The camera boots with a fixed IP of `192.168.144.108` on its Ethernet interface. No DHCP is available — the companion computer must be configured on the `192.168.144.x` subnet.

---

## Software Modules

### Server (Raspberry Pi)

#### Version 1 — Foundation
- Threaded RTSP capture with single-frame buffer (zero-lag)
- OpenCV MIL tracker for manual ROI-based tracking
- Dual PID controllers (yaw / pitch) with configurable gains and anti-windup
- TCP video streaming (`[4B size][JPEG]`)
- UDP command listener for `tune`, `track`, `stop`

#### Version 2 — Enhanced *(Recommended)*
All v1 features, plus:
- **YOLOv8 Integration** — Real-time person (class 0) and vehicle (class 2) detection at 320x320 resolution
- **ByteTrack Persistent IDs** — Multi-target tracking with stable IDs across frames
- **Dual Tracking Modes** — `NORMAL` (manual MIL) and `YOLO` (AI detection with auto or manual lock)
- **Low-Pass Filtering** — EMA filter (`alpha = 0.8`) on the error signal to suppress bounding-box jitter before PID
- **Adaptive Gain Scaling** — In YOLO mode, derivative gain is automatically reduced to 10% to account for inference latency (~50ms gaps between detections)
- **Metadata Streaming** — Each TCP frame carries JSON metadata: mode, state, all bounding boxes, and locked target ID
- **EO/IR Switching** — Toggle between RGB and thermal streams without restarting
- **Target Loss Handling** — Graceful timeout after 30 consecutive lost frames; reverts to selection mode
- **No-Signal Fallback** — Synthetic frame generated when RTSP is unavailable so the client remains connected

#### Server State Machine

```
                    +------------+
                    |    IDLE    |
                    | (standby)  |
                    +-----+------+
                          |
          +---------------+---------------+
          | NORMAL mode   |   YOLO mode   |
          v               |               v
   +------------+        |      +------------------+
   | MIL Track  |        |      | YOLO Detection   |
   | (manual)   |        |      |                  |
   | success    |        |      | 1 target -> auto |
   | -> PID     |        |      | N targets -> wait|
   | failure    |        |      | 0 targets -> idle|
   | -> IDLE    |        |      +--------+---------+
   +------------+        |               | lock_id
                         |               v
                         |      +------------------+
                         |      | YOLO LOCKED      |
                         |      | (track by ID)    |
                         |      |                  |
                         |      | found -> PID     |
                         |      | lost 30f -> IDLE |
                         |      +------------------+
          +--------------+--------------+
          |         STOP command         |
          |    -> re-center -> IDLE      |
          +------------------------------+
```

### Client (Laptop GCS)

#### Version 2 — Full-Featured Ground Control Station
- Decodes multiplexed TCP stream (metadata + video)
- Click-to-select target in YOLO mode (sends `lock_id` command)
- Real-time PID tuning sliders (`Kp`, `Ki`, `Kd`, `max_speed`, axis inversion)
- Semi-transparent HUD overlay (mode, state, shortcuts)
- Multi-target warning banner when selection is required
- Keyboard shortcuts: `Y` (mode toggle), `T` (EO/IR toggle), `C` (stop & center), `ESC` (quit)

### Utility Scripts

| Script | Purpose |
|:---|:---|
| `manual_controller.py` | Standalone keyboard gimbal tester (WASD pan/tilt, QE roll, IO zoom, C center) — useful for verifying hardware connectivity |
| `local_tracker.py` | Single-script tracker that runs on Pi with display (bench testing without the network client) |

---

## Network Protocols

### TCP Video Stream (Pi -> Laptop, Port 5000)

**Version 1:**
```
+-------------+-------------+
| 4B JPEG len | JPEG data   |
| (uint32 LE) |             |
+-------------+-------------+
```

**Version 2:**
```
+-------------+-------------+-------------+-------------+
| 4B meta len | JSON metadata| 4B JPEG len | JPEG data   |
| (uint32 LE) | (UTF-8)     | (uint32 LE) |             |
+-------------+-------------+-------------+-------------+
```

**Example JSON metadata:**
```json
{
  "mode": "YOLO",
  "state": "TRACKING",
  "locked_id": 3,
  "bboxes": [
    {"id": 3, "bbox": [120, 80, 60, 90], "cls": 0},
    {"id": 7, "bbox": [400, 200, 55, 70], "cls": 2}
  ]
}
```

### UDP Commands (Laptop -> Pi, Port 5005)

All commands are JSON-encoded datagrams:

```json
// Tune PID parameters
{"action": "tune", "kp": 0.0015, "ki": 0.0, "kd": 0.0001, "max_spd": 0.5, "inv_x": false, "inv_y": true}

// Start MIL tracking with bounding box
{"action": "track", "bbox": [120, 80, 60, 90]}

// Lock onto a specific YOLO target ID
{"action": "lock_id", "id": 3}

// Switch tracking mode
{"action": "mode_switch", "mode": "YOLO"}

// Toggle EO / IR stream
{"action": "camera_mode"}

// Stop and re-center
{"action": "stop"}
```

---

## Installation

### Hardware

- Skydroid C12 gimbal camera with Ethernet output
- Raspberry Pi 5 (or Pi 4) with Ubuntu 24.04 / Raspberry Pi OS
- Ethernet cable (camera to Pi)
- 3S-4S LiPo battery (camera power)
- Laptop on the same Wi-Fi network as the Pi

### Raspberry Pi Setup

**1. Network configuration:**

The C12 camera uses a fixed IP (`192.168.144.108`). Configure the Pi's Ethernet port accordingly:

```bash
sudo ip addr add 192.168.144.200/24 dev eth0
sudo ip link set eth0 up

# Verify
ping 192.168.144.108
```

To persist across reboots:
```bash
sudo nmcli con add type ethernet con-name skydroid-cam ifname eth0 \
  ipv4.addresses 192.168.144.200/24 ipv4.method manual
```

**2. Python dependencies:**

```bash
python3 -m venv cv_env
source cv_env/bin/activate

# Core
pip install opencv-python-headless numpy

# For YOLO mode (v2)
pip install ultralytics onnxruntime
```

> Use `opencv-python-headless` on the Pi. The `yolov8n.onnx` model auto-downloads on first run, or place it manually.

### Laptop Setup

```bash
pip install opencv-python numpy
```

---

## Usage Guide

**1. Power the C12 camera.** Wait 30-45 seconds for the embedded system to boot and the Ethernet link to establish.

**2. Start the server on the Pi:**
```bash
python3 pi_server_v2.py
```

Expected output:
```
[Gimbal] Connecting and enabling manual attitude control...
[UDP Server] Listening for commands on port 5005...
[TCP Server] Waiting for laptop on port 5000...
[Main] Server processing loop started.
```

**3. Start the client on the laptop:**
```bash
python3 laptop_client_v2.py
```

**4. Operation:**

| Key | Action |
|:---:|:---|
| `Y` | Toggle between NORMAL and YOLO tracking mode |
| `T` | Toggle between EO (RGB) and IR (thermal) camera |
| Click + drag | In NORMAL mode, define a tracking region |
| Click on box | In YOLO mode, select which target to lock |
| `C` | Stop tracking and re-center gimbal |
| `ESC` | Quit |

---

## Challenges & Resolutions

During development, several practical challenges emerged that are worth documenting for anyone extending this work:

### 1. Undocumented Control Interface

**Challenge:** The C12 has no published API or SDK. All programmatic interfaces had to be derived from analyzing the vendor's Android application.

**Resolution:** The APK was decompiled using [JADX](https://github.com/skylot/jadx) and the control path was traced through `SkydroidGimbal` -> `SkydroidControl` -> `CommonPayload` classes. This revealed the complete command structure, CRC algorithm, UDP port scheme, and RTSP endpoints, which are now fully documented in this repository for community use.

### 2. Multi-Port UDP Reliability

**Challenge:** Gimbal commands were intermittently dropped, suggesting the camera firmware may listen on different ports depending on version or state.

**Resolution:** Commands are now broadcast to all three discovered ports (`9002`, `5000`, `1030`) simultaneously. This redundant transmission ensures reliable delivery across firmware variants without modification.

### 3. Bounding-Box Jitter in PID Control

**Challenge:** Frame-to-frame noise in both MIL tracker output and YOLO detection coordinates caused the gimbal to oscillate, even when the target was stationary.

**Resolution:** An Exponential Moving Average (EMA) low-pass filter (`alpha = 0.8`) was applied to the pixel error signal before it reaches the PID controllers. This preserves responsive tracking while eliminating high-frequency jitter.

### 4. YOLO Inference Latency Causing Overshoot

**Challenge:** The ~50ms gap between YOLO detections (inference latency) meant the derivative term in the PID would spike when a new detection arrived, causing temporary overshoot.

**Resolution:** In YOLO mode, the derivative gain `Kd` is automatically scaled to 10% of its configured value. This accommodates the dead-time in the detection loop while maintaining smooth actuation.

### 5. Pi Power Instability Under AI Load

**Challenge:** Running YOLOv8 inference alongside video encoding and network I/O caused the Raspberry Pi 5 to brown out, dropping SSH connections and corrupting the tracking loop.

**Resolution:** A dedicated 5V/5A USB-C power supply (official Pi 5 PSU) is required for stable operation. For lower-power deployments, the v1 server (MIL tracking only) runs reliably on standard 3A supplies.

### 6. RTSP Stream Unavailability Handling

**Challenge:** When the camera is powered off or the Ethernet cable is disconnected, the RTSP reader blocks indefinitely, freezing the entire pipeline.

**Resolution:** A timeout mechanism was added to the RTSP capture thread. If no frame arrives within a defined window, a synthetic "NO SIGNAL" frame is generated and the TCP client remains connected. The system automatically resumes when the stream restores.

### 7. Thermal Stream Discovery

**Challenge:** The thermal camera RTSP endpoint was not exposed in the primary APK control flow and had to be located through additional analysis.

**Resolution:** The thermal stream runs on port `555` with path `/stream=2`, as documented in the protocol reference above. Both streams can be switched at runtime via the `camera_mode` UDP command without restarting the server.

---

## Customization Guide

This framework is intentionally modular so researchers can adapt it to their specific use cases:

| Use Case | What to Modify |
|:---|:---|
| **Different tracking algorithm** | Replace the MIL tracker in `pi_server_v1.py` or the YOLO integration in `pi_server_v2.py` with your own detector |
| **Different object classes** | Update the YOLO class filter in v2 (currently `cls 0` for person, `cls 2` for vehicle) or use a custom-trained model |
| **Custom GCS interface** | The TCP video protocol is fully documented — build your own client in any language that can parse `[4B meta][JSON][4B img][JPEG]` |
| **Different companion computer** | The server uses standard Python sockets and OpenCV — it runs on any Linux SBC with Ethernet (tested on Pi 4 and Pi 5) |
| **UART/MAVLink integration** | The `SkydroidGimbal` class is self-contained — replace its UDP output with MAVLink `MOUNT_CONTROL` messages for autopilot integration |
| **Add recording / telemetry logging** | The metadata JSON structure carries all tracking state — log it to disk or stream it to a telemetry server alongside the video |

---

## License

This project is released for educational and research purposes. The Skydroid brand, firmware, and associated applications are the property of their respective owners. Protocol documentation was derived through interoperability analysis in accordance with applicable fair use provisions.

---

<p align="center">
  <i>Developed to advance open embedded vision research for UAV payloads.</i>
</p>

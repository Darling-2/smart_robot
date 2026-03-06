Raspberry Pi 5 (ROS2 Jazzy) runtime

Prerequisites
- Ubuntu 24.04 + ROS2 Jazzy
- Python packages: ultralytics, opencv-python-headless, pyzmq
- This folder includes a self-contained ROS2 workspace

Install deps
- Run: scripts/install_deps.sh

Build workspace
- Run: scripts/build_ws.sh

Network
- Set static IP and confirm OrangePi is reachable.
- Example: RPi 192.168.50.3, OrangePi 192.168.50.2

CycloneDDS
- Copy config: config/cyclonedds.xml -> ~/cyclonedds.xml
- Edit peer IPs in the file.

Run
- One-key (recommended): scripts/one_key_inspection.sh
- Manual: scripts/start_rpi_jazzy.sh

Notes
- Default YOLO model: yolov8n.pt
- ZMQ goal addr default: tcp://192.168.50.2:5555
- ZMQ status addr default: tcp://192.168.50.2:5556

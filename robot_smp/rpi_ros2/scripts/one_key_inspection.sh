#!/usr/bin/env bash
set -e

# One-key inspection entry on RPi5
# This script can optionally start OrangePi ROS1 over SSH,
# then starts ROS2 inspection locally.

ORANGEPI_HOST="192.168.50.2"
ORANGEPI_USER="orangepi"
ORANGEPI_PORT="22"
USE_SSH_START="true"
ORANGEPI_ROBOT_SMP_DIR="/home/kafka/robot_smp"

ROBOT_SMP_DIR="/home/kafka/robot_smp"

if [ ! -d "$ROBOT_SMP_DIR" ]; then
  echo "[ERROR] robot_smp not found at $ROBOT_SMP_DIR"
  exit 1
fi

if [ ! -f "$ROBOT_SMP_DIR/rpi_ros2/scripts/start_rpi_jazzy.sh" ]; then
  echo "[ERROR] start script not found: $ROBOT_SMP_DIR/rpi_ros2/scripts/start_rpi_jazzy.sh"
  exit 1
fi

if [ ! -f "$HOME/cyclonedds.xml" ]; then
  echo "[ERROR] $HOME/cyclonedds.xml not found. Copy from robot_smp/rpi_ros2/config/cyclonedds.xml"
  exit 1
fi

if ! ping -c 1 -W 1 "$ORANGEPI_HOST" >/dev/null 2>&1; then
  echo "[WARN] OrangePi ($ORANGEPI_HOST) unreachable. You can still run local ROS2 vision."
fi

if [ "$USE_SSH_START" = "true" ]; then
  echo "[INFO] Starting OrangePi ROS1 over SSH..."
  ssh -p "$ORANGEPI_PORT" "$ORANGEPI_USER@$ORANGEPI_HOST" \
    "bash -lc '$ORANGEPI_ROBOT_SMP_DIR/orange_pi_ros1/scripts/one_key_orangepi.sh'" \
    || echo "[WARN] SSH start failed. Start OrangePi manually."
else
  echo "[INFO] SSH start disabled. Start OrangePi manually."
fi

sleep 2

bash "$ROBOT_SMP_DIR/rpi_ros2/scripts/start_rpi_jazzy.sh"

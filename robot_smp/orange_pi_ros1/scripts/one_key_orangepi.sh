#!/usr/bin/env bash
set -e

# One-key startup on OrangePi (ROS1)

export BASE_TYPE=NanoRobot
export LIDAR_TYPE=ydlidar
ROS1_WS="${ROS1_WS:-$HOME/bingda_ros1_noetic-nano}"

source /opt/ros/noetic/setup.bash

# Update to your ROS1 workspace path on OrangePi
if [ ! -f "$ROS1_WS/devel/setup.bash" ]; then
  echo "[ERROR] ROS1 workspace not found: $ROS1_WS/devel/setup.bash"
  echo "        Please set ROS1_WS env or run catkin_make first."
  exit 1
fi
source "$ROS1_WS/devel/setup.bash"

LOG_DIR="$HOME/robot_smp/logs"
mkdir -p "$LOG_DIR"

# Avoid duplicate launch if script is called repeatedly
pkill -f "roslaunch robot_navigation robot_navigation.launch" 2>/dev/null || true
pkill -f "roslaunch robot_navigation zmq_bridge.launch" 2>/dev/null || true

sleep 1

# Navigation stack (map + amcl + move_base)
nohup roslaunch robot_navigation robot_navigation.launch \
  > "$LOG_DIR/robot_navigation.log" 2>&1 &

sleep 2

# ZMQ bridge
nohup roslaunch robot_navigation zmq_bridge.launch \
  > "$LOG_DIR/zmq_bridge.log" 2>&1 &

echo "[OK] OrangePi ROS1 started. Logs: $LOG_DIR"

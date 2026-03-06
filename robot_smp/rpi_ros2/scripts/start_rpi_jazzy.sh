#!/usr/bin/env bash
set -e

# Env
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export BASE_TYPE=NanoRobot

if [ -f "$HOME/cyclonedds.xml" ]; then
  export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
else
  echo "[ERROR] ~/cyclonedds.xml not found. Copy from robot_smp/rpi_ros2/config."
  exit 1
fi

# Source ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# Update to your workspace path on RPi
source ~/robot_smp/rpi_ros2/ros2_ws/install/setup.bash

# Launch inspection (YOLO + logging + patrol + ZMQ bridge)
ros2 launch robot_vision_ros2 inspection.launch.py \
  use_zmq_bridge:=true \
  zmq_goal_addr:=tcp://192.168.50.2:5555 \
  zmq_status_addr:=tcp://192.168.50.2:5556 \
  model_path:=yolov8n.pt \
  max_fps:=5.0

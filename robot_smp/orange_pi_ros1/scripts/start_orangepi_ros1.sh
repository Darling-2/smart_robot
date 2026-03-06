#!/usr/bin/env bash
set -e

# Example startup sequence (adjust to your actual launch files)
# 1) Base control + sensors
# roslaunch base_control base_startup.launch
# 2) Lidar
# roslaunch robot_navigation lidar.launch
# 3) Navigation
# roslaunch robot_navigation robot_navigation.launch
# 4) ZMQ bridge
roslaunch robot_navigation zmq_bridge.launch

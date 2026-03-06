#!/usr/bin/env bash
set -e

source /opt/ros/jazzy/setup.bash

cd "$HOME/robot_smp/rpi_ros2/ros2_ws"

# Install dependencies (if rosdep is set up on the RPi)
if command -v rosdep >/dev/null 2>&1; then
  rosdep update
  rosdep install --from-paths src --ignore-src -r -y
fi

colcon build --symlink-install

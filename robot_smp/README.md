Robot SMP (RPi ROS2 + OrangePi ROS1)

This folder is the deployment guide and runtime scripts for the dual-board setup:
- OrangePi (ROS1 Noetic): lidar, base control, navigation, move_base
- Raspberry Pi 5 (ROS2 Jazzy): camera, YOLOv8, inspection manager, waypoint patrol
- Inter-board link: Ethernet + ZeroMQ (goal push + result pub)

Layout
- rpi_ros2/: scripts and configs for Raspberry Pi 5
- orange_pi_ros1/: scripts for OrangePi ROS1 side

Quick start (summary)
1) Configure static IPs and confirm ping works.
2) Copy robot_smp to both boards at the same path (e.g. /home/kafka/robot_smp).
3) OrangePi: compile ROS1 workspace once after update (catkin_make).
4) OrangePi: run orange_pi_ros1/scripts/one_key_orangepi.sh (or start from RPi via SSH).
5) RPi5: run rpi_ros2/scripts/one_key_inspection.sh.

Important
- If copied from Windows to Linux, run chmod once on both boards:
	chmod +x /home/kafka/robot_smp/rpi_ros2/scripts/*.sh
	chmod +x /home/kafka/robot_smp/orange_pi_ros1/scripts/*.sh

See the subfolder READMEs for details.

OrangePi (ROS1 Noetic) runtime

Prerequisites
- ROS1 Noetic + your existing navigation stack
- Python package: pyzmq
- ROS1 workspace built at least once (catkin_make)

Install deps
- pip3 install pyzmq

Run
1) One-key (recommended): scripts/one_key_orangepi.sh
2) Optional: set custom workspace path before run
   export ROS1_WS=/home/your_user/bingda_ros1_noetic-nano

Default ports
- Goal (RPi -> OrangePi): tcp://0.0.0.0:5555
- Status (OrangePi -> RPi): tcp://0.0.0.0:5556

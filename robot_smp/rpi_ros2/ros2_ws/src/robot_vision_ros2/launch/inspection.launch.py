#!/usr/bin/env python3
"""
完整巡检 launch (树莓派端)
  相机 + YOLO 检测 + 巡检记录 + 巡检点巡航

注意：ROS1 move_base 在香橙派启动，本 launch 不启动导航。
waypoint_patrol 通过 ZMQ 跨网发送导航目标。
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ---- 参数 ----
    model_path = LaunchConfiguration('model_path', default='yolov8n.pt')
    max_fps = LaunchConfiguration('max_fps', default='5.0')
    save_dir = LaunchConfiguration('save_dir', default='/tmp/inspection')
    loop = LaunchConfiguration('loop', default='false')
    wait_at_point = LaunchConfiguration('wait_at_point', default='5.0')
    status_topic = LaunchConfiguration('status_topic', default='/inspection/status')
    use_zmq_bridge = LaunchConfiguration('use_zmq_bridge', default='true')
    zmq_goal_addr = LaunchConfiguration('zmq_goal_addr', default='tcp://192.168.50.2:5555')
    zmq_status_addr = LaunchConfiguration('zmq_status_addr', default='tcp://192.168.50.2:5556')

    vision_pkg = get_package_share_directory('robot_vision_ros2')

    waypoints_file = os.path.join(vision_pkg, 'config', 'waypoints.yaml')
    yolo_launch = os.path.join(vision_pkg, 'launch', 'yolo.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('model_path', default_value=model_path),
        DeclareLaunchArgument('max_fps', default_value=max_fps),
        DeclareLaunchArgument('save_dir', default_value=save_dir),
        DeclareLaunchArgument('loop', default_value=loop),
        DeclareLaunchArgument('wait_at_point', default_value=wait_at_point),
        DeclareLaunchArgument('status_topic', default_value=status_topic),
        DeclareLaunchArgument('use_zmq_bridge', default_value=use_zmq_bridge),
        DeclareLaunchArgument('zmq_goal_addr', default_value=zmq_goal_addr),
        DeclareLaunchArgument('zmq_status_addr', default_value=zmq_status_addr),

        # 1) 视觉检测 (相机 + YOLO + 巡检记录)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(yolo_launch),
            launch_arguments={
                'model_path': model_path,
                'max_fps': max_fps,
                'save_dir': save_dir,
                'status_topic': status_topic,
            }.items(),
        ),

        # 2) 巡检点巡航 (通过 DDS 跨网调用香橙派上的 Nav2)
        Node(
            package='robot_vision_ros2',
            executable='waypoint_patrol',
            name='waypoint_patrol',
            output='screen',
            parameters=[{
                'waypoints_file': waypoints_file,
                'loop': loop,
                'wait_at_point': wait_at_point,
                'status_topic': status_topic,
                'use_zmq_bridge': use_zmq_bridge,
                'zmq_goal_addr': zmq_goal_addr,
                'zmq_status_addr': zmq_status_addr,
            }],
        ),
    ])

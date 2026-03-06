#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    image_topic = LaunchConfiguration('image_topic', default='/image_raw')
    # RPi5 CPU 推理优化：
    #   - yolov8n: nano 版模型，比 yolov8s 快 3-5x，适合嵌入式
    #   - max_fps=5: RPi5 CPU 推理 yolov8n@640 约 200-400ms/帧，5fps 足够
    #   - conf=0.4: 略微提高置信度阈值，减少误检
    model_path = LaunchConfiguration('model_path', default='yolov8n.pt')
    conf = LaunchConfiguration('conf', default='0.4')
    iou = LaunchConfiguration('iou', default='0.45')
    device = LaunchConfiguration('device', default='cpu')
    publish_annotated = LaunchConfiguration('publish_annotated', default='true')
    max_fps = LaunchConfiguration('max_fps', default='5.0')

    vision_pkg_dir = get_package_share_directory('robot_vision_ros2')
    camera_launch = os.path.join(vision_pkg_dir, 'launch', 'robot_camera.launch.py')
    save_dir = LaunchConfiguration('save_dir', default='/tmp/inspection')
    min_conf = LaunchConfiguration('min_conf', default='0.5')
    save_images = LaunchConfiguration('save_images', default='true')
    status_topic = LaunchConfiguration('status_topic', default='/inspection/status')

    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value=image_topic),
        DeclareLaunchArgument('model_path', default_value=model_path),
        DeclareLaunchArgument('conf', default_value=conf),
        DeclareLaunchArgument('iou', default_value=iou),
        DeclareLaunchArgument('device', default_value=device),
        DeclareLaunchArgument('publish_annotated', default_value=publish_annotated),
        DeclareLaunchArgument('max_fps', default_value=max_fps),
        DeclareLaunchArgument('save_dir', default_value=save_dir),
        DeclareLaunchArgument('min_conf', default_value=min_conf),
        DeclareLaunchArgument('save_images', default_value=save_images,
                              description='Whether to save annotated images'),
        DeclareLaunchArgument('status_topic', default_value=status_topic,
                              description='Inspection status topic for patrol linkage'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch),
        ),

        Node(
            package='robot_vision_ros2',
            executable='yolo_detector',
            name='yolo_detector',
            output='screen',
            parameters=[{
                'image_topic': image_topic,
                'model_path': model_path,
                'conf': conf,
                'iou': iou,
                'device': device,
                'publish_annotated': publish_annotated,
                'max_fps': max_fps,
            }],
        ),

        Node(
            package='robot_vision_ros2',
            executable='inspection_manager',
            name='inspection_manager',
            output='screen',
            parameters=[{
                'detections_topic': '/yolo/detections',
                'save_dir': save_dir,
                'min_conf': min_conf,
                'save_images': save_images,
                'status_topic': status_topic,
                'image_save_cooldown': 5.0,
                'max_saved_images': 500,
            }],
        ),
    ])

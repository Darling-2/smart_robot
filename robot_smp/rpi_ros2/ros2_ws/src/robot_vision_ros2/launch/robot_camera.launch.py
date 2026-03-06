#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    video_device = LaunchConfiguration('video_device', default='/dev/video0')
    pixel_format = LaunchConfiguration('pixel_format', default='YUYV')
    output_encoding = LaunchConfiguration('output_encoding', default='bgr8')

    # ======== 相机分辨率设置 ========
    # 720p 相机默认降采样到 640×480：
    #   - YOLO 推理内部 resize 到 640×640，采集更大分辨率浪费 CPU & 带宽
    #   - 640×480 在 RPi5 CPU 推理下更高效(帧率更高)
    #   - 如需更高分辨率(如远距离目标检测)，改为 [1280, 720]
    IMAGE_SIZE = [640, 480]

    return LaunchDescription([

        DeclareLaunchArgument(
            'video_device',
            default_value=video_device,
            description='Specifying device name'),

        DeclareLaunchArgument(
            'pixel_format',
            default_value=pixel_format,
            description='Specifying pixel format'),

        DeclareLaunchArgument(
            'output_encoding',
            default_value=output_encoding,
            description='Specifying output encoding type'),

        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera',
            parameters=[{
                'video_device': video_device,
                'pixel_format': pixel_format,
                'output_encoding': output_encoding,
                'image_size': IMAGE_SIZE,
            }],
            output='screen'),
    ])


import os
from glob import glob
from setuptools import setup

package_name = 'robot_vision_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
        # Config files (waypoints, DDS)
        (os.path.join('share', package_name, 'config'), glob('config/*.*')),
        # Scripts
        (os.path.join('share', package_name, 'script'), glob('script/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bingda',
    maintainer_email='bingda@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_test = robot_vision_ros2.vision_test:main',
            'cv_bridge_test = robot_vision_ros2.cv_bridge_test:main',
            'yolo_detector = robot_vision_ros2.yolo_detector:main',
            'inspection_manager = robot_vision_ros2.inspection_manager:main',
            'waypoint_patrol = robot_vision_ros2.waypoint_patrol:main',
            'report_generator = robot_vision_ros2.report_generator:main'
        ],
    },
)

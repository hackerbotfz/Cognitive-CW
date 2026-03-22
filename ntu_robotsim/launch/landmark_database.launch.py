#!/usr/bin/env python3
"""
Launch file for the Landmark Database system.

Starts:
  - yolo_detector          – runs YOLO on the RGBD camera image stream and
                             publishes detections on /yolo_detections.
  - landmark_database_node – fuses detections with depth data, maintains the
                             landmark database, and exposes it via topics and
                             ROS2 services.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    yolo_detector_node = Node(
        package='ntu_robotsim',
        executable='yolo_detector.py',
        name='yolo_detector',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    landmark_database_node = Node(
        package='ntu_robotsim',
        executable='landmark_database_node.py',
        name='landmark_database_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        yolo_detector_node,
        landmark_database_node,
    ])

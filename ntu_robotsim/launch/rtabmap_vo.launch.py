#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_dir = get_package_share_directory('ntu_robotsim')
    params_file = os.path.join(pkg_dir, 'config', 'rtabmap_vo_params.yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo simulation clock')

    spawn_x_arg = DeclareLaunchArgument(
        'spawn_x', default_value='-3.0',
        description='Robot spawn X in map frame (must match single_robot_sim)')

    spawn_y_arg = DeclareLaunchArgument(
        'spawn_y', default_value='2.5',
        description='Robot spawn Y in map frame (must match single_robot_sim)')

    spawn_z_arg = DeclareLaunchArgument(
        'spawn_z', default_value='0.1',
        description='Robot spawn Z in map frame (must match single_robot_sim)')

    use_sim_time = LaunchConfiguration('use_sim_time')
    spawn_x      = LaunchConfiguration('spawn_x')
    spawn_y      = LaunchConfiguration('spawn_y')
    spawn_z      = LaunchConfiguration('spawn_z')

    map_to_vo_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_vo_odom_publisher',
        arguments=[
            spawn_x, spawn_y, spawn_z,
            '0', '0', '0', '1',          # qx qy qz qw (identity rotation)
            'map', 'vo_odom'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_publisher',
        arguments=[
            '0.1', '0', '0.1',           # camera is ~10 cm forward and up
            '0', '0', '0', '1',
            'atlas/base_link', 'atlas/realsense'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    rgbd_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rtabmap_vo',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('rgb/image',       '/atlas/rgbd_camera/image'),
            ('rgb/camera_info', '/atlas/rgbd_camera/camera_info'),
            ('depth/image',     '/atlas/rgbd_camera/depth/image_raw'),
            ('odom',            '/atlas/visual_odom'),
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )

    return LaunchDescription([
        use_sim_time_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        map_to_vo_odom,
        base_to_camera,
        rgbd_odometry,
    ])

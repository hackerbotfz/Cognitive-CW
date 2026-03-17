#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    ntu_pkg = FindPackageShare('ntu_robotsim')
    odom_tf_pkg = FindPackageShare('odom_to_tf_ros2')
    nav2_pkg = FindPackageShare('nav2_bringup')

    map_file = PathJoinSubstitution([
        ntu_pkg,
        'maps',
        'cwmaze_map.yaml'
    ])

    nav2_params = PathJoinSubstitution([
        ntu_pkg,
        'config',
        'nav2_params.yaml'
    ])

    rviz_config = PathJoinSubstitution([
        ntu_pkg,
        'config',
        'single_robot.rviz'
    ])

    robot_gazebo_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ntu_robotsim'),
                'launch',
                'single_robot_sim.launch.py'
            ])
        )
    )

    cwmaze_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ntu_robotsim'),
                'launch',
                'cwmaze.launch.py'
            ])
        )
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d',
            PathJoinSubstitution([
                FindPackageShare('ntu_robotsim'),
                'config',
                'single_robot.rviz'
            ])
        ]
    )

    odom_to_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('odom_to_tf_ros2'),
                'launch',
                'atlas_odom_to_tf.launch.py'
            ])
        )
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                nav2_pkg,
                'launch',
                'bringup_launch.py'
            ])
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'True',
            'params_file': nav2_params
        }.items()
    )

    map_transformer = Node(
	package = "tf2_ros",
	executable = "static_transform_publisher",
	arguments = ["0", "0", "0", "0", "0", "0", "map", "odom"]
    )

    octomap_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('octomap_server2'),
                'launch',
                'octomap_server_launch.py'
            ])
        )
    )

    teleop = Node(
	package='teleop_twist_keyboard',
	executable='teleop_twist_keyboard',
	name='teleop_twist_keyboard',
	prefix='xterm -e',
	output='screen',
	remappings=[('cmd_vel', 'atlas/cmd_vel')]
    )

    return LaunchDescription([
	cwmaze_launch,
	map_transformer,
	teleop,
	rviz_node,
        robot_gazebo_rviz,
        odom_to_tf,
        octomap_server,
	nav2_bringup
    ])

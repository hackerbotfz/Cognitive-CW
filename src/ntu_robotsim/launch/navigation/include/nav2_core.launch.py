"""
Nav2 core navigation component launch file.

This component handles path planning, control, and behaviour coordination.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for Nav2 core stack."""

    # Get package directory
    pkg_dir = get_package_share_directory('ntu_robotsim')

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Nav2 parameters file
    nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2', 'nav2_params.yaml')

    # Controller server
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            nav2_params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # Planner server
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            nav2_params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # Behaviour server
    behaviour_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[
            nav2_params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # BT Navigator
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            nav2_params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # Lifecycle manager for navigation
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'autostart': True},
            {'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator'
            ]}
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        controller_server_node,
        planner_server_node,
        behaviour_server_node,
        bt_navigator_node,
        lifecycle_manager_node
    ])

"""
Main navigation launch file for ntu_robotsim with Nav2.

This launch file coordinates:
- Gazebo simulation
- Robot spawning and ROS2 bridges
- AMCL localisation
- Nav2 navigation stack
- RViz visualisation
"""

import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate the complete navigation launch description."""

    # Get package directories
    pkg_dir = get_package_share_directory('ntu_robotsim')

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )

    # Include Gazebo maze world launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'maze.launch.py')
        )
    )

    # Include robot spawning and bridges launch
    robot_spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'single_robot_sim.launch.py')
        )
    )

    # Include AMCL localisation component
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'navigation', 'include', 'amcl.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    # Include Nav2 core navigation component
    nav2_core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'navigation', 'include', 'nav2_core.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    # RViz node with Nav2 configuration
    rviz_config_file = os.path.join(pkg_dir, 'config', 'nav2', 'nav2_view.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=launch.conditions.IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        use_rviz_arg,

        # Simulation environment
        gazebo_launch,
        robot_spawn_launch,

        # Navigation components
        amcl_launch,
        nav2_core_launch,

        # Visualisation
        rviz_node
    ])

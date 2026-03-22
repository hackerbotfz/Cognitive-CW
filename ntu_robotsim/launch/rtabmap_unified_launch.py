from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions.timer_action import TimerAction
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument

from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
	ntu_sim_dir = get_package_share_directory('ntu_robotsim')
	odom_tf_dir = get_package_share_directory('odom_to_tf_ros2')

	maze = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(ntu_sim_dir, 'launch', 'cwmaze.launch.py')
		)
	)

	atlas = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(ntu_sim_dir, 'launch', 'single_robot_sim.launch.py')
		)
	)

#	robot_state_publisher = IncludeLaunchDescription(
#		PythonLaunchDescriptionSource(
#			os.path.join(ntu_sim_dir, 'launch', 'robot_state_publisher.launch.py')
#		)
#	)

	robot_state_publisher = Node(
		package='tf2_ros',
		executable='static_transform_publisher',
		name='static_transform_publisher',
		output='screen',
		prefix='xterm -e',
		arguments=['0.1', '0', '0.19', '-0.5', '0.5', '-0.5', '0.5', 'atlas/base_link', 'atlas/realsense'],
		parameters=[{'use_sim_time': True}]
	)

	world_to_map = Node(
		package='tf2_ros',
		executable='static_transform_publisher',
		arguments=['-3', '2.5', '0', '0', '0', '0', '1', 'world', 'map'],
		parameters=[{'use_sim_time': True}]
	)

	rtabmap = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(ntu_sim_dir, 'launch', 'rtabmap.launch.py')
		)
	)

	nav2 = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(ntu_sim_dir, 'launch', 'nav2.launch.py')
		)
	)

	rviz = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(ntu_sim_dir, 'launch', 'rviz.launch.py')
		)
	)

	teleop = Node(
		package='teleop_twist_keyboard',
		executable='teleop_twist_keyboard',
		name='teleop_twist_keyboard',
		prefix='xterm -e',
		output='screen',
		remappings=[('cmd_vel', '/atlas/cmd_vel')]
	)

	return LaunchDescription([
		maze,
		world_to_map,
		atlas,
		TimerAction(period=3.0, actions=[robot_state_publisher]),
		TimerAction(period=4.0, actions=[rtabmap]),
		TimerAction(period=7.0, actions=[nav2]),
		TimerAction(period=9.0, actions=[rviz]),
		TimerAction(period=8.0, actions=[teleop])
	])
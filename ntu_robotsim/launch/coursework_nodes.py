from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
	goal_detector = Node(
		package='ntu_robotsim',
		executable='goal_detector.py',
		name='goal_detector',
		output='screen',
		prefix='xterm -hold -e',
		parameters=[{'use_sim_time': True}],
	)

	traffic_sign_navigator = Node(
		package='ntu_robotsim',
		executable='traffic_sign_navigator.py',
		name='traffic_sign_navigator',
		output='screen',
		prefix='xterm -hold -e',
		parameters=[{'use_sim_time': True}],
	)

	landmark_database_node = Node(
		package='ntu_robotsim',
		executable='landmark_database_node.py',
		name='landmark_database_node',
		output='screen',
		prefix='xterm -hold -e',
		parameters=[{'use_sim_time': True}],
	)

	waypoints = Node(
		package='ntu_robotsim',
		executable='waypoints.py',
		name='waypoints',
		output='screen',
		prefix='xterm -hold -e',
		parameters=[{'use_sim_time': True}],
	)


	return LaunchDescription([
		goal_detector,
		traffic_sign_navigator,
		landmark_database_node,
		waypoints
	])
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnShutdown
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

	odom_to_tf = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(odom_tf_dir, 'launch', 'atlas_odom_to_tf.launch.py')
		)
	)

	map_transformer = Node(
		package = "tf2_ros",
		executable = "static_transform_publisher",
		arguments = ["0", "0", "0", "0", "0", "0", "map", "odom"]
	)

	octomap = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(ntu_sim_dir, 'launch', 'octomap.launch.py')
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
		remappings=[('cmd_vel', '/teleop_cmd_vel')]
	)

	coursework_nodes = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(ntu_sim_dir, 'launch', 'coursework_nodes.py')
		)
	)

	kill_nodes = RegisterEventHandler(
		OnShutdown(
			on_shutdown=[
				ExecuteProcess(
					cmd=['pkill',
						'-f', 'map_transformer',
						'-f', 'goal_detector_node',
						'-f', 'traffic_sign_navigator',
						'-f', 'landmark_database_node',
						'-f', 'waypoints'],
					shell=True
				)
			]
		)
	)

	return LaunchDescription([
		maze,
		atlas,
		odom_to_tf,
		map_transformer,
		octomap,
		nav2,
		rviz,
		teleop,
		coursework_nodes,
		kill_nodes
	])

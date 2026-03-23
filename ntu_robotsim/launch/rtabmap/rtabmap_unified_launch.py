from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnShutdown
import os

def generate_launch_description():
	ntu_sim_dir = get_package_share_directory('ntu_robotsim')
	odom_tf_dir = get_package_share_directory('odom_to_tf_ros2')
	realsense_state = os.path.join(ntu_sim_dir, 'models', 'jetbot', 'realsense.urdf')
	optical_relay_script = '/home/ntu-user/ros2_ws/install/ntu_robotsim/lib/ntu_robotsim/optical_frame.py'

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

	realsense_state_publisher = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		name='realsense_state_publisher',
		output='screen',
		prefix='xterm -hold -e',
		parameters=[{
			'robot_description': open(realsense_state).read(),
			'use_sim_time': True
		}]
	)

	realsense_to_realsense_link = Node(
		package='tf2_ros',
		executable='static_transform_publisher',
		name='base_to_realsense_link',
		output='screen',
		prefix='xterm -hold -e',
		arguments=['0', '0', '0', '0', '0', '0', 'atlas/realsense', 'atlas/realsense_link'],
		parameters=[{'use_sim_time': True}]
	)

	realsense_link_to_optical = Node(
		package='tf2_ros',
		executable='static_transform_publisher',
		name='realsense_link_to_optical',
		output='screen',
		prefix='xterm -hold -e',
		arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'atlas/realsense_link', 'atlas/optical'],
		parameters=[{'use_sim_time': True}]
	)

	optical_state_publisher = ExecuteProcess(
		cmd=['python3', optical_relay_script],
		prefix='xterm -hold -e',
		output='screen'
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

	rtabmap = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(ntu_sim_dir, 'launch', 'rtabmap', 'rtabmap.launch.py')
		)
	)

	octomap = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(ntu_sim_dir, 'launch', 'octomap.launch.py')
		)
	)

	nav2 = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(ntu_sim_dir, 'launch', 'rtabmap', 'nav2.launch.py')
		)
	)

	rviz = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(ntu_sim_dir, 'launch', 'rtabmap', 'rviz.launch.py')
		)
	)

	teleop = Node(
		package='teleop_twist_keyboard',
		executable='teleop_twist_keyboard',
		name='teleop_twist_keyboard',
		prefix='xterm -hold -e',
		output='screen',
		remappings=[('cmd_vel', '/atlas/cmd_vel')]
	)

	kill_nodes = RegisterEventHandler(
		OnShutdown(on_shutdown=[ExecuteProcess(cmd=['pkill', '-f', 'realsense_state_publisher', '-f', 'realsense_to_realsense_link', '-f', 'realsense_link_to_optical', '-f', 'map_transformer'], shell=True)])
	)

	return LaunchDescription([
		maze,
		atlas,
		map_transformer,
		realsense_state_publisher,
		realsense_to_realsense_link,
		realsense_link_to_optical,
		optical_state_publisher,
		octomap,
		rtabmap,
		nav2,
		rviz,
		teleop,
		kill_nodes
	])
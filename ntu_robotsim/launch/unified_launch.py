from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
	ntu_sim_dir = get_package_share_directory('ntu_robotsim')
	odom_tf_dir = get_package_share_directory('odom_to_tf_ros2')

	maze = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(ntu_sim_dir, 'launch', 'cwmaze.launch.py')
		)
	)

	jetbot = IncludeLaunchDescription(
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

	return LaunchDescription([
		maze,
		jetbot,
		odom_to_tf,
		map_transformer,
		nav2,
		rviz
	])

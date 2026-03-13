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

	amcl_node = Node(
		package='nav2_amcl',
		executable='amcl',
		name='amcl',
		output='screen',
		parameters=[
			os.path.join(ntu_sim_dir, 'config', 'nav2', 'amcl.yaml'),
			{'use_sim_time': LaunchConfiguration('use_sim_time')}
		]
	)

	map_server_node = Node(
		package='nav2_map_server',
		executable='map_server',
		name='map_server',
		output='screen',
		parameters=[
			{'yaml_filename': LaunchConfiguration('map')},
			{'use_sim_time': LaunchConfiguration('use_sim_time')}
		]
	)

	lifecycle_manager_node = Node(
		package='nav2_lifecycle_manager',
		executable='lifecycle_manager',
		name='lifecycle_manager_localization',
		output='screen',
		parameters=[
			{'use_sim_time': LaunchConfiguration('use_sim_time')},
			{'autostart': True},
			{'node_names': ['map_server', 'amcl']}
		]
	)

	return LaunchDescription([
		use_sim_time_arg,
		use_rviz_arg,
		amcl_node,
		map_server_node,
		lifecycle_manager_node
	])

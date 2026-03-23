from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import SetRemap
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
	parametersFile = LaunchConfiguration('parametersFile')

	parametersFileArgs = DeclareLaunchArgument(
		'parametersFile',
		default_value = os.path.join(get_package_share_directory('ntu_robotsim'), 'config', 'nav2_params.yaml')
	)

	nav2 = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py'
			)
		),
		launch_arguments = {
			'params_file': parametersFile,
			'use_sim_time': 'true',
			'autostart': 'true',
			'transform_tolerance': '1.0'
		}.items()
	)

	return LaunchDescription([
		parametersFileArgs,
		SetRemap(src='cmd_vel', dst='/atlas/cmd_vel'),
		SetRemap(src='/cmd_vel', dst='/atlas/cmd_vel'),
		nav2
	])

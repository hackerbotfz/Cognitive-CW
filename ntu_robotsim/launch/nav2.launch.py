from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
			#os.path.join(get_package_share_directory('yolo_bringup'), 'launch', 'yoloe.launch.py'
			)
		),
		launch_arguments = {'params_file': parametersFile}.items()
	)

	return LaunchDescription([parametersFileArgs, nav2])

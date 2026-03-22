from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
	ntu_sim_dir = get_package_share_directory('ntu_robotsim')

	urdf_file = os.path.join(ntu_sim_dir, 'models', 'jetbot', 'model.urdf')

	robot_state_publisher = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		name='robot_state_publisher',
		output='screen',
		prefix='xterm -e',
		parameters=[{
			'use_sim_time': True,
			'robot_description': ParameterValue(Command(['cat ', urdf_file]), value_type=str)
		}]
	)
	return LaunchDescription([robot_state_publisher])
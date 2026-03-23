from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
	ntu_sim_dir = get_package_share_directory('ntu_robotsim')
	realsense_state = os.path.join(ntu_sim_dir, 'models', 'jetbot', 'realsense.urdf')

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

	return LaunchDescription([
		realsense_state_publisher
	])
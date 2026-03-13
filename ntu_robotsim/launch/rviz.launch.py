from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
	pkg = get_package_share_directory("ntu_robotsim")

	rviz_config = os.path.join(pkg, 'config', 'single_robot.rviz')

	rviz_node = Node(
		package = 'rviz2',
		executable = 'rviz2',
		name = 'rviz2',
		arguments = ['-d', rviz_config],
		output = 'screen',
		condition = None
	)

	return LaunchDescription([
		rviz_node
	])



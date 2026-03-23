from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	octomap_server = Node(
		package = 'octomap_server2',
		executable = 'octomap_server',
		name = 'octomap_server',
		output = 'screen',
		parameters = [{
			'frame_id': 'odom',
			'base_frame_id':'atlas/base_link',
			'filter_ground': True,
			'pointcloud_min_z': -0.02,
			'pointcloud_max_z': 2.0,
			'occupancy_min_z': 0.02,
			'occupancy_max_z': 2.0
		}],
		remappings = [('cloud_in','atlas/rgbd_camera/points'),]
	)
	return LaunchDescription([octomap_server])

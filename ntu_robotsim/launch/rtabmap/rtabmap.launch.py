from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	rgbd_odometry = Node(
		package= 'rtabmap_odom',
		executable= 'rgbd_odometry',
		name= 'rgbd_odometry',
		output='screen',
		prefix='xterm -hold -e',
		parameters=[{
			'use_sim_time': True,
			'approx_sync': True,
			'subscribe_depth': True,
			'publish_tf': True,
			'frame_id': 'atlas/base_link',
			'wait_for_transform': 0.2,
			'odom_frame_id': 'odom',
			'sync_queue_size': 30,
			'qos': 1,

			'Odom/ResetCountdown': '1',
			'Odom/Strategy': '0',
			'Kp/MinDepth': '0.3',
			'Kp/MaxDepth': '1.5',

			'Vis/FeatureType': '6',
			'Vis/MaxFeatures': '1000',
			'Vis/MinInliers': '10'
		}],
		remappings=[
			('rgb/image', '/atlas/rgbd_camera/optical_image'),
			('depth/image', '/atlas/rgbd_camera/optical_depth_image'),
			('rgb/camera_info', '/atlas/rgbd_camera/optical_camera_info'),			
			('odom', '/atlas/odom_vo')
		]
	)

	return LaunchDescription([
		rgbd_odometry
		])
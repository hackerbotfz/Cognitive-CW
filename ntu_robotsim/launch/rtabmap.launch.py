from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions.timer_action import TimerAction

def generate_launch_description():
	rgbd_sync = Node(
		package='rtabmap_sync',
		executable='rgbd_sync',
		name='rgbd_sync',
		output='screen',
		prefix='xterm -e',
		parameters=[{
			'use_sim_time': True,
			'approx_sync': False,
			'qos': 2,
			'queue_size': 30,
			'sync_queue_size': 100
		}],
		remappings=[
			('rgb/image', '/atlas/rgbd_camera/image'),
			('depth/image', '/atlas/rgbd_camera/depth_image'),
			('rgb/camera_info', '/atlas/rgbd_camera/camera_info'),
			('rgbd_image', '/atlas/rgbd_camera/rgbd_image')
		]
	)

	rtabmap_octomap = Node(
		package='rtabmap_slam',
		executable='rtabmap',
		name='rtabmap',
		output='screen',
		prefix='xterm -e',
		parameters=[{
			'use_sim_time': True,
			'frame_id': 'atlas/base_link',
			'odom_frame_id': 'odom',
			'map_frame_id': 'map',
			'publish_tf': True,
			'subscribe_rgbd': True,
			'approx_sync': False,
			'sync_queue_size': 100,
			'topic_queue_size': 30,
			'qos': 2,

			# Mapping behavior
			'Mem/IncrementalMemory': 'true',
			'RGBD/OptimizeFromGraphEnd': 'false',

			# Octomap / 3D occupancy
			'Grid/Sensor': '1',
			'Grid/3D': 'true',
			'Grid/RayTracing': 'true',
			'Grid/RangeMin': '0.3',
			'Grid/RangeMax': '4.0',
			'Grid/CellSize': '0.1',

			# Ground filtering
			'Grid/NormalsSegmentation': 'true',
			'Grid/MaxGroundAngle': '10',
			'Grid/NormalK': '20',
			'Grid/ClusterRadius': '0.1',
			'Grid/MinClusterSize': '20',
			'Grid/FlatObstacleDetected': 'false',

			# Height filtering
			'Grid/MinGroundHeight': '-0.10',
			'Grid/MaxGroundHeight': '0.10',
			'Grid/MaxObstacleHeight': '2.0'

		}],
		remappings=[
			('rgbd_image', '/atlas/rgbd_camera/rgbd_image'),
			('odom', '/odom')
		],
		arguments=['--delete_db_on_start']
	)

	visual_odometry = Node(
		package='rtabmap_odom',
		executable='rgbd_odometry',
		name='visual_odometry',
		output='screen',
		prefix='xterm -e',
		parameters=[{
			'use_sim_time': True,
			'frame_id': 'atlas/base_link',
			'odom_frame_id': 'odom',
			'grid_map': 'map',
			'publish_tf': True,
			'approx_sync': False,
			'subscribe_rgbd': True,
			'topic_queue_size': 30,
			'sync_queue_size': 100,

			'Vis/MinInliers': '6',
			'Vis/InlierDistance': '0.2',
			'Kp/MaxFeatures': '1500',
			'Kp/MinDepth': '0.3',
			'Kp/MaxDepth': '6.0',

			'Odom/Strategy': '0',
			'Odom/GuessMotion': 'true',
			'Odom/ResetCountdown': '5'
		}],
		remappings=[
			('rgbd_image', '/atlas/rgbd_camera/rgbd_image'),
			('odom', '/odom')
		]
	)

	return LaunchDescription([
		rgbd_sync,
		TimerAction(period=1.5, actions=[visual_odometry]),
		TimerAction(period=3.0, actions=[rtabmap_octomap])
		])
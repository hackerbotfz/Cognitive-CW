#!/usr/bin/env python3

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import time
import math



def create_pose(x, y, theta):
	pose = PoseStamped()
	pose.header.frame_id = 'map'
	pose.header.stamp = rclpy.time.Time().to_msg()
	pose.pose.position.x = x
	pose.pose.position.y = y
	pose.pose.position.z = 0.0

	pose.pose.orientation.x = 0.0
	pose.pose.orientation.y = 0.0
	pose.pose.orientation.z = math.sin(theta / 2.0)
	pose.pose.orientation.w = math.cos(theta / 2.0)

	return pose


def main():
	rclpy.init()
	navigator = BasicNavigator()

	navigator.waitUntilNav2Active(localizer='bt_navigator')

	waypoints = [
		create_pose(-3.5, -2.6, -3.13),
		create_pose(4.5, -3.0, -1.50),
		create_pose(8.25, -3.0, -1.50),
		create_pose(2.0, 2.53, -3.13)
	]

	print(f"Starting waypoint mission with {len(waypoints)} waypoints...")

	mission_start = time.time()

	for i, waypoint in enumerate(waypoints, 1):
		print(f"\nNavigating to waypoint {i}/{len(waypoints)}")
		print(f"  Position: ({waypoint.pose.position.x:.2f}, "f"{waypoint.pose.position.y:.2f})")

		waypoint_start = time.time()

		navigator.goToPose(waypoint)

		while not navigator.isTaskComplete():
			feedback = navigator.getFeedback()
			print(feedback)
			time.sleep(1)

		waypoint_time = time.time() - waypoint_start
	
		result = navigator.getResult()
		if result == TaskResult.SUCCEEDED:
			print(f"Reached waypoint {i} in {waypoint_time:.1f} seconds")
		elif result == TaskResult.CANCELED:
			print(f"Waypoint {i} was canceled")
			print(f"Aborting mission.")
			break
		elif result == TaskResult.FAILED:
			print(f"Failed to reach waypoint {i}")
			print(f"Aborting mission.")
			break

		time.sleep(1.0)

	mission_time = time.time() - mission_start
	print(f"\nMission completed in {mission_time:.1f} seconds")

	rclpy.shutdown()

if __name__ == '__main__':
	main()

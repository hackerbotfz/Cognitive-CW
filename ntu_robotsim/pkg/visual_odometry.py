#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class VisualOdometry(Node):
    def __init__(self):
        super().__init__('visual_odometry')

        # Parameters
        self.declare_parameter('camera_topic', 'atlas/rgbd_camera/image')
        self.declare_parameter('camera_info_topic', 'atlas/rgbd_camera/camera_info')
        self.declare_parameter('odom_topic', 'atlas/visual_odom')
        self.declare_parameter('base_frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'atlas/base_link')

        self.bridge = CvBridge()

        # Camera Intrinsics (will be updated from camera_info)
        self.K = np.zeros((3,3))
        self.pp = (0,0)
        self.focal = 1.0
        self.cam_info_received = False

        # VO State
        self.prev_image = None
        self.prev_keypoints = None
        self.prev_descriptors = None
        
        # Current Pose (R=Rotation, t=Translation)
        self.cur_R = np.eye(3)
        self.cur_t = np.zeros((3, 1))

        # Feature Detector (ORB is fast and rotation invariant)
        self.orb = cv2.ORB_create(nfeatures=1000)
        # Matcher
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        # Subscribers
        self.create_subscription(
            CameraInfo, 
            self.get_parameter('camera_info_topic').value, 
            self.camera_info_callback, 
            10
        )
        self.create_subscription(
            Image, 
            self.get_parameter('camera_topic').value, 
            self.image_callback, 
            10
        )

        # Publisher
        self.odom_pub = self.create_publisher(
            Odometry, 
            self.get_parameter('odom_topic').value, 
            10
        )

        self.get_logger().info("Visual Odometry Node Initialized")

    def camera_info_callback(self, msg):
        if not self.cam_info_received:
            # Reshape K matrix from flat array
            self.K = np.array(msg.k).reshape(3, 3)
            self.focal = self.K[0, 0]
            self.pp = (self.K[0, 2], self.K[1, 2])
            self.cam_info_received = True
            self.get_logger().info(f"Camera Info received: fx={self.focal}, pp={self.pp}")

    def image_callback(self, msg):
        if not self.cam_info_received:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        # 1. Feature Detection
        kp, des = self.orb.detectAndCompute(cv_image, None)

        if des is None or len(kp) < 8:
            self.get_logger().warn("Not enough features detected")
            return

        if self.prev_image is None or self.prev_descriptors is None:
            self.prev_image = cv_image
            self.prev_keypoints = kp
            self.prev_descriptors = des
            return

        # 2. Feature Matching
        matches = self.bf.match(des, self.prev_descriptors)
        matches = sorted(matches, key=lambda x: x.distance)

        # Keep only good matches
        good_matches = matches[:200] 
        
        if len(good_matches) < 8:
            self.get_logger().warn("Not enough matches for VO")
            return

        pts1 = np.float32([self.prev_keypoints[m.trainIdx].pt for m in good_matches])
        pts2 = np.float32([kp[m.queryIdx].pt for m in good_matches])

        # 3. Compute Essential Matrix
        E, mask = cv2.findEssentialMat(pts2, pts1, self.focal, self.pp, cv2.RANSAC, 0.999, 1.0)
        
        if E is not None:
            # 4. Recover Pose
            _, R, t, mask = cv2.recoverPose(E, pts2, pts1, self.K)

            # NOTE: t is a unit vector (scale is unknown in monocular VO).
            # We assume a constant scale or speed here (e.g., 1.0 or based on avg robot speed).
            # For this basic implementation, we just integrate the unit vector.
            # In a real scenario, you'd multiply t by current_speed * dt.
            absolute_scale = 1.0 # Placeholder for scale estimation

            if (absolute_scale > 0.1): 
                self.cur_t = self.cur_t + absolute_scale * self.cur_R.dot(t)
                self.cur_R = self.cur_R.dot(R)

            self.publish_odometry(msg.header.stamp)

        # Update state
        self.prev_image = cv_image
        self.prev_keypoints = kp
        self.prev_descriptors = des

    def publish_odometry(self, timestamp):
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp
        odom_msg.header.frame_id = self.get_parameter('base_frame_id').value
        odom_msg.child_frame_id = self.get_parameter('child_frame_id').value

        # Fill Pose
        odom_msg.pose.pose.position.x = float(self.cur_t[0])
        odom_msg.pose.pose.position.y = float(self.cur_t[1])
        odom_msg.pose.pose.position.z = float(self.cur_t[2])

        # Convert Rotation Matrix to Quaternion
        # (Simplified helper or standard conversion needed here)
        q = self.rotation_matrix_to_quaternion(self.cur_R)
        odom_msg.pose.pose.orientation.w = q[0]
        odom_msg.pose.pose.orientation.x = q[1]
        odom_msg.pose.pose.orientation.y = q[2]
        odom_msg.pose.pose.orientation.z = q[3]

        self.odom_pub.publish(odom_msg)

    def rotation_matrix_to_quaternion(self, R):
        tr = R[0,0] + R[1,1] + R[2,2]
        if tr > 0:
            S = math.sqrt(tr+1.0) * 2
            qw = 0.25 * S
            qx = (R[2,1] - R[1,2]) / S
            qy = (R[0,2] - R[2,0]) / S
            qz = (R[1,0] - R[0,1]) / S
        elif (R[0,0] > R[1,1]) and (R[0,0] > R[2,2]):
            S = math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
            qw = (R[2,1] - R[1,2]) / S
            qx = 0.25 * S
            qy = (R[0,1] + R[1,0]) / S
            qz = (R[0,2] + R[2,0]) / S
        elif R[1,1] > R[2,2]:
            S = math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
            qw = (R[0,2] - R[2,0]) / S
            qx = (R[0,1] + R[1,0]) / S
            qy = 0.25 * S
            qz = (R[1,2] + R[2,1]) / S
        else:
            S = math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
            qw = (R[1,0] - R[0,1]) / S
            qx = (R[0,2] + R[2,0]) / S
            qy = (R[1,2] + R[2,1]) / S
            qz = 0.25 * S
        return [qw, qx, qy, qz]

def main(args=None):
    rclpy.init(args=args)
    node = VisualOdometry()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
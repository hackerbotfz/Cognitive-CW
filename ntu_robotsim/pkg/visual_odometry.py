#!/usr/bin/env python3

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, Odometry
from cv_bridge import CvBridge
import message_filters
from scipy.spatial.transform import Rotation as R

class RobustVisualOdometry(Node):
    def __init__(self):
        super().__init__('robust_visual_odometry')

        self.declare_parameter('rgb_topic', '/atlas/rgbd_camera/image')
        self.declare_parameter('depth_topic', '/atlas/rgbd_camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/atlas/rgbd_camera/camera_info')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'atlas/base_link')

        self.bridge = CvBridge()
        
        self.rgb_sub = message_filters.Subscriber(self, Image, self.get_parameter('rgb_topic').value)
        self.depth_sub = message_filters.Subscriber(self, Image, self.get_parameter('depth_topic').value)
        
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=10, slop=0.05
        )
        self.ts.registerCallback(self.sync_callback)
        
        self.info_sub = self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value, self.info_callback, 10
        )
        self.odom_pub = self.create_publisher(Odometry, '/atlas/visual_odom', 10)

        self.K = None
        self.prev_gray = None
        self.prev_kpts = None
        self.prev_desc = None
        self.prev_depth = None
        
        self.cur_R = np.eye(3)
        self.cur_t = np.zeros((3, 1))
        
        self.alpha = 0.85

        self.orb = cv2.ORB_create(nfeatures=2000)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)

        self.get_logger().info("Robust RGB-D VO Started. Waiting for CameraInfo...")

    def info_callback(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info(f"Intrinsics Loaded: fx={self.K[0,0]}")

    def sync_callback(self, rgb_msg, depth_msg):
        if self.K is None:
            return

        try:
            curr_gray = self.bridge.imgmsg_to_cv2(rgb_msg, 'mono8')
            raw_depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
            
            if raw_depth.dtype == np.uint16:
                curr_depth = raw_depth.astype(np.float32) / 1000.0
            else:
                curr_depth = raw_depth
        except Exception as e:
            self.get_logger().error(f"Conversion failed: {e}")
            return

        curr_kpts, curr_desc = self.orb.detectAndCompute(curr_gray, None)
        if curr_desc is None or len(curr_kpts) < 30:
            self._handle_failure("Insufficient features in current frame")
            return

        if self.prev_desc is None:
            self._update_reference(curr_gray, curr_kpts, curr_desc, curr_depth)
            return

        self.estimate_motion(curr_kpts, curr_desc, curr_depth, rgb_msg.header.stamp)
        self._update_reference(curr_gray, curr_kpts, curr_desc, curr_depth)

    def estimate_motion(self, curr_kpts, curr_desc, curr_depth, stamp):
        matches = self.matcher.knnMatch(self.prev_desc, curr_desc, k=2)
        good = [m for m, n in matches if m.distance < 0.75 * n.distance]

        obj_pts, img_pts = [], []
        cx, cy = self.K[0, 2], self.K[1, 2]
        fx, fy = self.K[0, 0], self.K[1, 1]

        for m in good:
            u, v = self.prev_kpts[m.queryIdx].pt
            z = self.prev_depth[int(v), int(u)]
            
            if np.isfinite(z) and 0.1 < z < 10.0:
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy
                obj_pts.append([x, y, z])
                img_pts.append(curr_kpts[m.trainIdx].pt)

        if len(obj_pts) < 15:
            self._handle_failure("Not enough valid depth points")
            return

        success, rvec, tvec, inliers = cv2.solvePnPRansac(
            np.array(obj_pts, dtype=np.float32), 
            np.array(img_pts, dtype=np.float32), 
            self.K, None, iterationsCount=100, reprojectionError=2.0
        )

        if not success or inliers is None or len(inliers) < 12:
            self._handle_failure("PnP RANSAC failed or low inliers")
            return

        rmat, _ = cv2.Rodrigues(rvec)
        R_rel = rmat.T
        t_rel = -rmat.T @ tvec

        if np.linalg.norm(t_rel) > 1.5:
            self._handle_failure("Motion jump exceeds physical limits")
            return

        self.cur_t += (self.cur_R @ t_rel) * self.alpha
        self.cur_R = self.cur_R @ R_rel
        
        self.publish_odom(stamp)

    def _update_reference(self, gray, kpts, desc, depth):
        self.prev_gray, self.prev_kpts, self.prev_desc, self.prev_depth = \
            gray, kpts, desc, depth

    def _handle_failure(self, reason):
        self.get_logger().warn(f"VO Tracker Reset: {reason}")
        self.prev_desc = None

    def publish_odom(self, stamp):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.get_parameter('odom_frame').value
        msg.child_frame_id = self.get_parameter('base_frame').value

        tx, ty, tz = self.cur_t.flatten()
        msg.pose.pose.position.x = float(tz)
        msg.pose.pose.position.y = float(-tx)
        msg.pose.pose.position.z = float(-ty)

        rot = R.from_matrix(self.cur_R)
        q = rot.as_quat()
        msg.pose.pose.orientation.x = float(q[2])
        msg.pose.pose.orientation.y = float(-q[0])
        msg.pose.pose.orientation.z = float(-q[1])
        msg.pose.pose.orientation.w = float(q[3])

        msg.pose.covariance = [0.01] * 36
        self.odom_pub.publish(msg)

def main():
    rclpy.init()
    node = RobustVisualOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

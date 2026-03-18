#!/usr/bin/env python3

import math
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class VisualOdometry(Node):
    def __init__(self):
        super().__init__('visual_odometry')


        self.declare_parameter('camera_topic', 'atlas/rgbd_camera/image')
        self.declare_parameter('camera_info_topic', 'atlas/rgbd_camera/camera_info')
        self.declare_parameter('odom_topic', '/atlas/visual_odom')
        self.declare_parameter('base_frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'atlas/base_link')

 
        self.declare_parameter('orb_nfeatures', 2500)
        self.declare_parameter('ratio_test', 0.75)
        self.declare_parameter('max_good_matches', 400)
        self.declare_parameter('min_features', 120)
        self.declare_parameter('min_good_matches', 40)
        self.declare_parameter('min_pose_inliers', 20)
        self.declare_parameter('min_pixel_displacement', 0.7)
        self.declare_parameter('assumed_speed_mps', 0.20)
        self.declare_parameter('max_dt_sec', 0.5)
        self.declare_parameter('ransac_prob', 0.999)
        self.declare_parameter('ransac_threshold', 1.0)
        self.declare_parameter('warn_interval_sec', 5.0)
        self.declare_parameter('stats_interval_sec', 2.0)

        self.camera_topic = str(self.get_parameter('camera_topic').value)
        self.camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.base_frame_id = str(self.get_parameter('base_frame_id').value)
        self.child_frame_id = str(self.get_parameter('child_frame_id').value)

        self.orb_nfeatures = int(self.get_parameter('orb_nfeatures').value)
        self.ratio_test = float(self.get_parameter('ratio_test').value)
        self.max_good_matches = int(self.get_parameter('max_good_matches').value)
        self.min_features = int(self.get_parameter('min_features').value)
        self.min_good_matches = int(self.get_parameter('min_good_matches').value)
        self.min_pose_inliers = int(self.get_parameter('min_pose_inliers').value)
        self.min_pixel_displacement = float(self.get_parameter('min_pixel_displacement').value)
        self.assumed_speed_mps = float(self.get_parameter('assumed_speed_mps').value)
        self.max_dt_sec = float(self.get_parameter('max_dt_sec').value)
        self.ransac_prob = float(self.get_parameter('ransac_prob').value)
        self.ransac_threshold = float(self.get_parameter('ransac_threshold').value)
        self.warn_interval_sec = float(self.get_parameter('warn_interval_sec').value)
        self.stats_interval_sec = float(self.get_parameter('stats_interval_sec').value)

        conflicting_odom_topics = {
            'odom',
            '/odom',
            'atlas/odom_ground_truth',
            '/atlas/odom_ground_truth',
            'atlas/odom',
            '/atlas/odom',
        }
        if self.odom_topic in conflicting_odom_topics:
            raise ValueError(
                f"Configured odom_topic '{self.odom_topic}' conflicts with simulator odometry topic. "
                "Use '/atlas/visual_odom' or another dedicated VO topic."
            )

        self.bridge = CvBridge()
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        self._last_warning_times = {}
        self._last_info_times = {}


        self.K: Optional[np.ndarray] = None
        self.cam_info_received = False


        self.prev_gray: Optional[np.ndarray] = None
        self.prev_keypoints = None
        self.prev_descriptors: Optional[np.ndarray] = None
        self.prev_stamp_sec: Optional[float] = None
        self.cur_R = np.eye(3, dtype=np.float64)
        self.cur_t = np.zeros((3, 1), dtype=np.float64)
        self.frames_total = 0
        self.frames_accepted = 0


        self.orb = cv2.ORB_create(nfeatures=self.orb_nfeatures, fastThreshold=10)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, 10)
        self.create_subscription(Image, self.camera_topic, self.image_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)

        self.get_logger().info(
            f"Visual Odometry started. image='{self.camera_topic}', info='{self.camera_info_topic}', odom='{self.odom_topic}'"
        )

    def _warn_throttled(self, key: str, message: str):
        now = time.monotonic()
        last = self._last_warning_times.get(key, 0.0)
        if now - last >= self.warn_interval_sec:
            self.get_logger().warn(message)
            self._last_warning_times[key] = now

    def _info_throttled(self, key: str, message: str):
        now = time.monotonic()
        last = self._last_info_times.get(key, 0.0)
        if now - last >= self.stats_interval_sec:
            self.get_logger().info(message)
            self._last_info_times[key] = now

    @staticmethod
    def _stamp_to_seconds(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def camera_info_callback(self, msg: CameraInfo):
        if self.cam_info_received:
            return

        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        if K[0, 0] <= 0.0 or K[1, 1] <= 0.0:
            self._warn_throttled('bad_intrinsics', f'Invalid camera intrinsics: fx={K[0,0]:.3f}, fy={K[1,1]:.3f}')
            return

        self.K = K
        self.cam_info_received = True
        self.get_logger().info(f"Camera intrinsics loaded: fx={K[0,0]:.2f}, fy={K[1,1]:.2f}")

    def _prepare_gray(self, msg: Image) -> Optional[np.ndarray]:
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:
            self._warn_throttled('cv_bridge', f'CV Bridge conversion failed: {exc}')
            return None

        if cv_image is None:
            self._warn_throttled('empty_frame', 'Received empty image frame')
            return None

        if cv_image.ndim == 3:
            encoding = msg.encoding.lower()
            channels = cv_image.shape[2]
            if channels == 3:
                if 'rgb' in encoding and 'bgr' not in encoding:
                    gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
                else:
                    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            elif channels == 4:
                if 'rgba' in encoding and 'bgra' not in encoding:
                    gray = cv2.cvtColor(cv_image, cv2.COLOR_RGBA2GRAY)
                else:
                    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2GRAY)
            else:
                self._warn_throttled('unsupported_channels', f'Unsupported image channels: {channels}')
                return None
        else:
            gray = cv_image

        if gray.dtype != np.uint8:
            gray = cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        gray = self.clahe.apply(gray)
        return gray

    def _refresh_reference(self, gray: np.ndarray, keypoints, descriptors: Optional[np.ndarray], stamp_sec: float):
        self.prev_gray = gray
        self.prev_keypoints = keypoints
        self.prev_descriptors = descriptors
        self.prev_stamp_sec = stamp_sec

    def image_callback(self, msg: Image):
        if not self.cam_info_received or self.K is None:
            return

        self.frames_total += 1
        gray = self._prepare_gray(msg)
        if gray is None:
            return

        stamp_sec = self._stamp_to_seconds(msg.header.stamp)
        keypoints, descriptors = self.orb.detectAndCompute(gray, None)
        feature_count = 0 if keypoints is None else len(keypoints)

        if descriptors is None or feature_count < self.min_features:
            self._warn_throttled(
                'low_features',
                f"Not enough features to initialize tracking ({feature_count} < {self.min_features})",
            )
            self._refresh_reference(gray, keypoints, descriptors, stamp_sec)
            self.publish_odometry(msg.header.stamp)
            return

        if self.prev_descriptors is None or self.prev_keypoints is None:
            self._refresh_reference(gray, keypoints, descriptors, stamp_sec)
            self.publish_odometry(msg.header.stamp)
            return


        knn_matches = self.matcher.knnMatch(self.prev_descriptors, descriptors, k=2)
        good_matches = []
        for pair in knn_matches:
            if len(pair) != 2:
                continue
            best, second = pair
            if best.distance < self.ratio_test * second.distance:
                good_matches.append(best)

        good_matches = sorted(good_matches, key=lambda m: m.distance)[: self.max_good_matches]

        if len(good_matches) < self.min_good_matches:
            self._warn_throttled(
                'low_matches',
                f"Not enough matches for VO ({len(good_matches)} < {self.min_good_matches}); "
                f"features={feature_count}, knn_pairs={len(knn_matches)}",
            )
            self._refresh_reference(gray, keypoints, descriptors, stamp_sec)
            self.publish_odometry(msg.header.stamp)
            return

        prev_points = np.float32([self.prev_keypoints[m.queryIdx].pt for m in good_matches])
        curr_points = np.float32([keypoints[m.trainIdx].pt for m in good_matches])

        displacements = np.linalg.norm(curr_points - prev_points, axis=1)
        median_disp = float(np.median(displacements))
        if median_disp < self.min_pixel_displacement:
            self._warn_throttled(
                'low_motion',
                f"Insufficient motion ({median_disp:.3f}px < {self.min_pixel_displacement:.3f}px), "
                "pose not updated",
            )
            self._refresh_reference(gray, keypoints, descriptors, stamp_sec)
            self.publish_odometry(msg.header.stamp)
            return

        E, _ = cv2.findEssentialMat(
            prev_points,
            curr_points,
            self.K,
            method=cv2.RANSAC,
            prob=self.ransac_prob,
            threshold=self.ransac_threshold,
        )
        if E is None:
            self._warn_throttled(
                'essential_failed',
                f"Essential matrix failed (good_matches={len(good_matches)}, median_disp={median_disp:.3f})",
            )
            self._refresh_reference(gray, keypoints, descriptors, stamp_sec)
            self.publish_odometry(msg.header.stamp)
            return

        inlier_count, R, t, _ = cv2.recoverPose(E, prev_points, curr_points, self.K)
        if int(inlier_count) < self.min_pose_inliers:
            self._warn_throttled(
                'pose_inliers',
                f"Pose recovery weak inliers ({int(inlier_count)} < {self.min_pose_inliers}); "
                f"good_matches={len(good_matches)}",
            )
            self._refresh_reference(gray, keypoints, descriptors, stamp_sec)
            self.publish_odometry(msg.header.stamp)
            return

        self.frames_accepted += 1
        dt = 0.0 if self.prev_stamp_sec is None else max(0.0, stamp_sec - self.prev_stamp_sec)
        dt = min(dt, self.max_dt_sec)
        scale = self.assumed_speed_mps * dt
        if scale > 0.0:
            self.cur_t = self.cur_t + scale * (self.cur_R @ t)
            self.cur_R = self.cur_R @ R
            # Keep rotation numerically well-formed
            u, _, vt = np.linalg.svd(self.cur_R)
            self.cur_R = u @ vt

        self._info_throttled(
            'vo_stats',
            f"VO stats: features={feature_count}, matches={len(knn_matches)}, "
            f"good={len(good_matches)}, inliers={int(inlier_count)}, "
            f"accepted={self.frames_accepted}/{self.frames_total}",
        )
        self._refresh_reference(gray, keypoints, descriptors, stamp_sec)
        self.publish_odometry(msg.header.stamp)

    def publish_odometry(self, timestamp):
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp
        odom_msg.header.frame_id = self.base_frame_id
        odom_msg.child_frame_id = self.child_frame_id

        odom_msg.pose.pose.position.x = float(self.cur_t[0, 0])
        odom_msg.pose.pose.position.y = float(self.cur_t[1, 0])
        odom_msg.pose.pose.position.z = float(self.cur_t[2, 0])

        q = self.rotation_matrix_to_quaternion(self.cur_R)
        odom_msg.pose.pose.orientation.w = q[0]
        odom_msg.pose.pose.orientation.x = q[1]
        odom_msg.pose.pose.orientation.y = q[2]
        odom_msg.pose.pose.orientation.z = q[3]

        self.odom_pub.publish(odom_msg)

    def rotation_matrix_to_quaternion(self, R: np.ndarray):
        tr = R[0, 0] + R[1, 1] + R[2, 2]
        if tr > 0:
            s = math.sqrt(tr + 1.0) * 2.0
            qw = 0.25 * s
            qx = (R[2, 1] - R[1, 2]) / s
            qy = (R[0, 2] - R[2, 0]) / s
            qz = (R[1, 0] - R[0, 1]) / s
        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s

        q = np.array([qw, qx, qy, qz], dtype=np.float64)
        n = np.linalg.norm(q)
        if n > 0.0:
            q /= n
        return q.tolist()


def main(args=None):
    rclpy.init(args=args)
    node = VisualOdometry()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

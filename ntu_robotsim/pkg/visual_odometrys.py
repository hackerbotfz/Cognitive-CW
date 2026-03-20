#!/usr/bin/env python3
"""
visual_odometry.py  —  Basic RGB-D Visual Odometry for real robot localisation.

Pipeline stages
---------------
  1. Image acquisition   – synchronised RGB + depth subscription
  2. Pre-processing      – grayscale conversion, depth scaling
  3. Feature detection   – ORB keypoint + descriptor extraction
  4. Feature matching    – BFMatcher with Lowe ratio test
  5. 3-D correspondence  – depth back-projection to world points
  6. Motion estimation   – PnP RANSAC (3-D → 2-D)
  7. Pose integration    – incremental R|t accumulation with SO(3) correction
  8. Odometry output     – nav_msgs/Odometry on /odom  +  odom→base_link TF

Coordinate remapping  (camera → REP-103)
-----------------------------------------
  camera:  z forward,  x right, y down
  REP-103: x forward,  y left,  z up

      pub_x  =  cam_z    (forward)
      pub_y  = -cam_x    (left)
      pub_z  = -cam_y    (up)
"""

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import message_filters
from scipy.spatial.transform import Rotation as R
from tf2_ros import TransformBroadcaster


# ── Utility ─────────────────────────────────────────────────────────────────

def _reorthogonalise(mat: np.ndarray) -> np.ndarray:
    """
    Project a 3×3 matrix back onto SO(3) via SVD.

    Repeated floating-point multiplication causes det(R) to drift from 1 and
    R^T R to drift from I.  SVD gives the nearest proper rotation matrix.
    """
    U, _, Vt = np.linalg.svd(mat)
    R_clean = U @ Vt
    if np.linalg.det(R_clean) < 0:
        U[:, -1] *= -1
        R_clean = U @ Vt
    return R_clean


def _flat_covariance(diag: float) -> list:
    """Return a flat 6×6 covariance list with *diag* on the diagonal."""
    cov = [0.0] * 36
    for i in (0, 7, 14, 21, 28, 35):
        cov[i] = diag
    return cov


# ── Node ─────────────────────────────────────────────────────────────────────

class VisualOdometry(Node):
    """
    Basic visual odometry node.

    Subscribes to synchronised RGB and depth images, runs the VO pipeline,
    and publishes:
      * nav_msgs/Odometry  on /odom  (pose + velocity estimate)
      * TF transform        odom → atlas/base_link

    Both outputs use the frame names expected by nav2_params.yaml so that
    the costmaps and bt_navigator can consume them without any relay node.
    """

    def __init__(self):
        super().__init__('visual_odometry')

        self.declare_parameter('rgb_topic',         '/atlas/rgbd_camera/image')
        self.declare_parameter('depth_topic',       '/atlas/rgbd_camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/atlas/rgbd_camera/camera_info')
        self.declare_parameter('odom_frame',        'odom')
        self.declare_parameter('base_frame',        'atlas/base_link')

        self.odom_frame       = self.get_parameter('odom_frame').value
        self.base_frame       = self.get_parameter('base_frame').value
        rgb_topic             = self.get_parameter('rgb_topic').value
        depth_topic           = self.get_parameter('depth_topic').value
        camera_info_topic     = self.get_parameter('camera_info_topic').value

        self.bridge         = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)

        # Stage 8 publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Stage 1 subscribers
        self.rgb_sub   = message_filters.Subscriber(self, Image, rgb_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=10, slop=0.05
        )
        self.ts.registerCallback(self._acquisition_callback)

        self.info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self._camera_info_callback, 10
        )

        # VO state
        self.K          = None
        self.orb        = cv2.ORB_create(nfeatures=2000)
        self.matcher    = cv2.BFMatcher(cv2.NORM_HAMMING)

        self.cur_R = np.eye(3)
        self.cur_t = np.zeros((3, 1))

        self.prev_gray  = None
        self.prev_kpts  = None
        self.prev_desc  = None
        self.prev_depth = None
        self.prev_stamp = None

        self.prev_pub_xyz = np.zeros(3)
        self.prev_pub_rpy = np.zeros(3)

        self._frame_count     = 0
        self._REORTH_INTERVAL = 30

        self.get_logger().info(
            f'VisualOdometry ready  |  TF: {self.odom_frame} → {self.base_frame}'
        )
        self.get_logger().info('Waiting for CameraInfo ...')

    # ── Camera intrinsics ────────────────────────────────────────────────

    def _camera_info_callback(self, msg: CameraInfo):
        """Latch the camera intrinsic matrix once, then unsubscribe."""
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info(
                f'CameraInfo received  fx={self.K[0,0]:.1f}  fy={self.K[1,1]:.1f}'
                f'  cx={self.K[0,2]:.1f}  cy={self.K[1,2]:.1f}'
            )
            self.destroy_subscription(self.info_sub)
            self.info_sub = None

    # ════════════════════════════════════════════════════════════════════════
    # Stage 1 — Image acquisition
    # ════════════════════════════════════════════════════════════════════════

    def _acquisition_callback(self, rgb_msg: Image, depth_msg: Image):
        """Receive synchronised RGB + depth frames and drive the pipeline."""
        if self.K is None:
            return

        # Stage 2
        curr_gray, curr_depth = self._preprocess(rgb_msg, depth_msg)
        if curr_gray is None:
            return

        # Stage 3
        curr_kpts, curr_desc = self._detect_features(curr_gray)
        if curr_desc is None:
            return

        stamp = rgb_msg.header.stamp

        if self.prev_desc is None:
            self._update_reference(curr_gray, curr_kpts, curr_desc,
                                   curr_depth, stamp)
            return

        # Stages 4-7
        self._run_pipeline(curr_kpts, curr_desc, curr_depth, stamp)
        self._update_reference(curr_gray, curr_kpts, curr_desc,
                               curr_depth, stamp)

    # ════════════════════════════════════════════════════════════════════════
    # Stage 2 — Pre-processing
    # ════════════════════════════════════════════════════════════════════════

    def _preprocess(self, rgb_msg: Image, depth_msg: Image):
        """
        Convert ROS image messages to OpenCV arrays.

        Depth is normalised to metres (uint16 images encode millimetres).
        Returns (gray, depth_float32) or (None, None) on failure.
        """
        try:
            gray  = self.bridge.imgmsg_to_cv2(rgb_msg,   'mono8')
            raw_d = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
        except Exception as exc:
            self.get_logger().error(f'[Stage 2] Image conversion failed: {exc}')
            return None, None

        depth = (raw_d.astype(np.float32) / 1000.0
                 if raw_d.dtype == np.uint16
                 else raw_d.astype(np.float32))
        return gray, depth

    # ════════════════════════════════════════════════════════════════════════
    # Stage 3 — Feature detection
    # ════════════════════════════════════════════════════════════════════════

    def _detect_features(self, gray: np.ndarray):
        """
        Detect ORB keypoints and compute binary descriptors.

        Returns (keypoints, descriptors) or (None, None) when the frame
        contains too few features to track reliably.
        """
        kpts, desc = self.orb.detectAndCompute(gray, None)
        if desc is None or len(kpts) < 30:
            self.get_logger().warn(
                f'[Stage 3] Only {len(kpts) if kpts else 0} features — '
                'skipping frame.'
            )
            return None, None
        return kpts, desc

    # ════════════════════════════════════════════════════════════════════════
    # Stages 4–7 orchestration
    # ════════════════════════════════════════════════════════════════════════

    def _run_pipeline(self, curr_kpts, curr_desc, curr_depth, stamp):
        """Drive Stages 4 through 7, then hand off to Stage 8."""

        good_matches = self._match_features(curr_desc)        # Stage 4
        if good_matches is None:
            return

        obj_pts, img_pts = self._build_correspondences(       # Stage 5
            good_matches, curr_kpts, curr_depth
        )
        if obj_pts is None:
            return

        R_rel, t_rel = self._estimate_motion(obj_pts, img_pts) # Stage 6
        if R_rel is None:
            return

        self._integrate_pose(R_rel, t_rel)                    # Stage 7
        self._publish_odometry(stamp)                         # Stage 8

    # ════════════════════════════════════════════════════════════════════════
    # Stage 4 — Feature matching
    # ════════════════════════════════════════════════════════════════════════

    def _match_features(self, curr_desc):
        """
        k-NN matching with Lowe ratio test (k=2, threshold=0.75).

        The ratio test discards ambiguous matches where the best and
        second-best descriptors are similar in distance, keeping only
        unambiguous, high-confidence correspondences.

        Returns a list of good DMatch objects, or None if too few survive.
        """
        raw = self.matcher.knnMatch(self.prev_desc, curr_desc, k=2)
        good = [
            m for pair in raw
            if len(pair) == 2
            for m, n in [pair]
            if m.distance < 0.75 * n.distance
        ]
        if len(good) < 15:
            self.get_logger().warn(
                f'[Stage 4] Only {len(good)} matches after ratio test — '
                'resetting tracker.'
            )
            self._reset_tracker()
            return None
        return good

    # ════════════════════════════════════════════════════════════════════════
    # Stage 5 — 3-D correspondence
    # ════════════════════════════════════════════════════════════════════════

    def _build_correspondences(self, good_matches, curr_kpts, curr_depth):
        """
        Back-project previous-frame keypoints to 3-D using the depth map,
        pairing each with the corresponding 2-D point in the current frame.

        The back-projection formula is:
            X = (u - cx) * Z / fx
            Y = (v - cy) * Z / fy

        Points with invalid or out-of-range depth (0.1–10 m) are discarded.
        Returns (obj_pts, img_pts) or (None, None) if too few remain.
        """
        h, w   = self.prev_depth.shape[:2]
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]

        obj_pts, img_pts = [], []
        for m in good_matches:
            u, v = self.prev_kpts[m.queryIdx].pt
            ui   = min(int(u), w - 1)
            vi   = min(int(v), h - 1)
            z    = self.prev_depth[vi, ui]

            if np.isfinite(z) and 0.1 < z < 10.0:
                obj_pts.append([(u - cx) * z / fx,
                                (v - cy) * z / fy,
                                z])
                img_pts.append(curr_kpts[m.trainIdx].pt)

        if len(obj_pts) < 15:
            self.get_logger().warn(
                f'[Stage 5] Only {len(obj_pts)} valid depth points — '
                'resetting tracker.'
            )
            self._reset_tracker()
            return None, None

        return (np.array(obj_pts, dtype=np.float32),
                np.array(img_pts, dtype=np.float32))

    # ════════════════════════════════════════════════════════════════════════
    # Stage 6 — Motion estimation
    # ════════════════════════════════════════════════════════════════════════

    def _estimate_motion(self, obj_pts, img_pts):
        """
        Estimate the inter-frame rigid transformation via PnP RANSAC.

        Given N 3-D world points (from Stage 5) and their 2-D projections
        in the current image, cv2.solvePnPRansac recovers the rotation R and
        translation t that minimise reprojection error while rejecting outliers.

        The result is inverted (R.T, -R.T @ t) to give the camera-to-world
        (i.e. robot motion) transform rather than the world-to-camera one
        returned directly by PnP.

        Returns (R_rel, t_rel) or (None, None) on failure.
        """
        ok, rvec, tvec, inliers = cv2.solvePnPRansac(
            obj_pts, img_pts,
            self.K, None,
            iterationsCount=100,
            reprojectionError=2.0,
        )

        if not ok or inliers is None or len(inliers) < 12:
            self.get_logger().warn(
                f'[Stage 6] PnP RANSAC failed  '
                f'(inliers={len(inliers) if inliers is not None else 0}) — '
                'resetting tracker.'
            )
            self._reset_tracker()
            return None, None

        rmat, _ = cv2.Rodrigues(rvec)
        R_rel   = rmat.T
        t_rel   = -rmat.T @ tvec.reshape(3, 1)

        if not np.isfinite(np.linalg.norm(t_rel)) or np.linalg.norm(t_rel) > 1.5:
            self.get_logger().warn(
                f'[Stage 6] Motion norm={np.linalg.norm(t_rel):.3f} m — '
                'rejected as outlier.'
            )
            self._reset_tracker()
            return None, None

        return R_rel, t_rel

    # ════════════════════════════════════════════════════════════════════════
    # Stage 7 — Pose integration
    # ════════════════════════════════════════════════════════════════════════

    def _integrate_pose(self, R_rel: np.ndarray, t_rel: np.ndarray):
        """
        Accumulate the inter-frame transformation into the global pose.

        The translation delta is first rotated into the world frame by the
        current accumulated rotation before being added to the global position.
        Re-orthogonalisation is applied every REORTH_INTERVAL frames to
        counteract SO(3) drift from floating-point accumulation.
        """
        self.cur_t += self.cur_R @ t_rel
        self.cur_R  = self.cur_R @ R_rel

        self._frame_count += 1
        if self._frame_count % self._REORTH_INTERVAL == 0:
            self.cur_R = _reorthogonalise(self.cur_R)

    # ════════════════════════════════════════════════════════════════════════
    # Stage 8 — Odometry output
    # ════════════════════════════════════════════════════════════════════════

    def _publish_odometry(self, stamp):
        """
        Publish nav_msgs/Odometry on /odom and broadcast the
        odom → atlas/base_link TF.

        Pose uses a camera→REP-103 axis remap so the robot's forward
        direction is +x in the world frame.

        Linear and angular velocities are finite-differenced from successive
        published poses, providing Nav2's controller server with a usable
        twist estimate.
        """
        tx, ty, tz = self.cur_t.flatten()

        # Camera → REP-103 remap
        pub_x = float(tz)
        pub_y = float(-tx)
        pub_z = float(-ty)

        rot = R.from_matrix(self.cur_R)
        q   = rot.as_quat()  # [x, y, z, w]

        qx = float(q[2])
        qy = float(-q[0])
        qz = float(-q[1])
        qw = float(q[3])

        # Velocity estimate via finite difference
        vx = vy = vz = 0.0
        wx = wy = wz = 0.0

        if self.prev_stamp is not None:
            dt = ((stamp.sec - self.prev_stamp.sec) +
                  (stamp.nanosec - self.prev_stamp.nanosec) * 1e-9)
            if dt > 1e-6:
                cur_xyz = np.array([pub_x, pub_y, pub_z])
                cur_rpy = rot.as_euler('xyz')

                lin_vel = (cur_xyz - self.prev_pub_xyz) / dt
                ang_vel = (cur_rpy - self.prev_pub_rpy) / dt

                vx, vy, vz = float(lin_vel[0]), float(lin_vel[1]), float(lin_vel[2])
                wx, wy, wz = float(ang_vel[0]), float(ang_vel[1]), float(ang_vel[2])

                self.prev_pub_xyz = cur_xyz
                self.prev_pub_rpy = cur_rpy
        else:
            self.prev_pub_xyz = np.array([pub_x, pub_y, pub_z])
            self.prev_pub_rpy = rot.as_euler('xyz')

        self.prev_stamp = stamp

        # Odometry message
        odom = Odometry()
        odom.header.stamp    = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.base_frame

        odom.pose.pose.position.x    = pub_x
        odom.pose.pose.position.y    = pub_y
        odom.pose.pose.position.z    = pub_z
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance         = _flat_covariance(0.01)

        odom.twist.twist.linear.x  = vx
        odom.twist.twist.linear.y  = vy
        odom.twist.twist.linear.z  = vz
        odom.twist.twist.angular.x = wx
        odom.twist.twist.angular.y = wy
        odom.twist.twist.angular.z = wz
        odom.twist.covariance      = _flat_covariance(0.05)

        self.odom_pub.publish(odom)

        # TF broadcast
        tf = TransformStamped()
        tf.header.stamp    = stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id  = self.base_frame

        tf.transform.translation.x = pub_x
        tf.transform.translation.y = pub_y
        tf.transform.translation.z = pub_z
        tf.transform.rotation.x    = qx
        tf.transform.rotation.y    = qy
        tf.transform.rotation.z    = qz
        tf.transform.rotation.w    = qw

        self.tf_broadcaster.sendTransform(tf)

    # ── Helpers ──────────────────────────────────────────────────────────

    def _update_reference(self, gray, kpts, desc, depth, stamp):
        self.prev_gray  = gray
        self.prev_kpts  = kpts
        self.prev_desc  = desc
        self.prev_depth = depth
        self.prev_stamp = stamp

    def _reset_tracker(self):
        """Drop the reference frame so the next frame initialises fresh."""
        self.prev_desc = None


# ── Entry point ──────────────────────────────────────────────────────────────

def main():
    rclpy_init = False
    node = None
    try:
        rclpy.init()
        rclpy_init = True
        node = VisualOdometry()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy_init:
            rclpy.shutdown()


if __name__ == '__main__':
    main()

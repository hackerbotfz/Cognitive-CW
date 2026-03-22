#!/usr/bin/env python3
"""
ROS2 node – Landmark Database Node.

Maintains a live :class:`~ntu_robotsim.landmark_database.LandmarkDatabase`
by fusing YOLO detections with depth-camera data.

Subscriptions
-------------
/yolo_detections  (std_msgs/String)
    JSON detections published by ``yolo_detector``.
/atlas/rgbd_camera/depth_image  (sensor_msgs/Image)
    Depth frame used to back-project detections to 3-D.
/atlas/rgbd_camera/camera_info  (sensor_msgs/CameraInfo)
    Intrinsic calibration for back-projection.

Publications
------------
/landmark_database/markers  (visualization_msgs/MarkerArray)
    Sphere + label markers for RViz (published at 1 Hz).
/landmark_database/all  (std_msgs/String)
    JSON array of all stored landmarks (published at 1 Hz).
/landmark_database/count  (std_msgs/Int32)
    Number of distinct landmarks currently in the database (published at 1 Hz).

Services
--------
/landmark_database/get_all  (std_srvs/srv/Trigger)
    Returns all landmarks as a JSON string in ``response.message``.
/landmark_database/query_by_label  (std_srvs/srv/Trigger)
    Pass the desired label via the ROS parameter
    ``query_label`` before calling; returns matching landmarks as JSON.

Parameters
----------
db_file : str
    Path to the JSON file where the database is persisted on disk.
    Defaults to ``~/landmark_database.json``.  The file is written (or
    updated) once per second and again when the node shuts down, so the
    most recent state is always available on disk.
merge_radius : float
    Distance threshold (metres) used to merge nearby detections of the
    same class into a single landmark entry.  Defaults to ``1.5``.
query_label : str
    Label filter used by the ``/landmark_database/query_by_label`` service.

File output
-----------
After at least one detection has been received the database is saved as a
pretty-printed JSON array to the file named by the ``db_file`` parameter.
Example default path::

    ~/landmark_database.json   →   /home/<user>/landmark_database.json
"""

import json
import math
import os
import sys
import tempfile

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Trigger

try:
    from cv_bridge import CvBridge, CvBridgeError
    import numpy as np
    _CV_AVAILABLE = True
except ImportError:
    _CV_AVAILABLE = False

# Support running directly with ``python3 landmark_database_node.py``
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ntu_robotsim.landmark_database import LandmarkDatabase  # noqa: E402


class LandmarkDatabaseNode(Node):
    """ROS2 node that builds and exposes a Landmark Database."""

    # Default camera intrinsics (typical RealSense 640×480 settings).
    # These are overwritten when /atlas/rgbd_camera/camera_info arrives.
    _DEFAULT_FX = 462.14
    _DEFAULT_FY = 462.14
    _DEFAULT_CX = 320.5
    _DEFAULT_CY = 240.5

    _DEFAULT_DB_FILENAME = 'landmark_database.json'

    def __init__(self) -> None:
        super().__init__('landmark_database_node')

        self.db = LandmarkDatabase()

        # ---- camera intrinsics ----
        self._fx = self._DEFAULT_FX
        self._fy = self._DEFAULT_FY
        self._cx = self._DEFAULT_CX
        self._cy = self._DEFAULT_CY

        # ---- latest depth frame ----
        self._depth_image = None

        # ---- dirty flag: True when the db has changed since last file write ----
        self._db_dirty: bool = False

        if _CV_AVAILABLE:
            self._bridge = CvBridge()
        else:
            self.get_logger().warn(
                'cv_bridge / numpy not available – depth back-projection disabled.'
            )

        # ---- subscriptions ----
        self.create_subscription(
            String,
            '/yolo_detections',
            self._on_detections,
            10,
        )
        self.create_subscription(
            Image,
            '/atlas/rgbd_camera/depth_image',
            self._on_depth,
            10,
        )
        self.create_subscription(
            CameraInfo,
            '/atlas/rgbd_camera/camera_info',
            self._on_camera_info,
            10,
        )

        # ---- publications ----
        self._pub_markers = self.create_publisher(
            MarkerArray,
            '/landmark_database/markers',
            10,
        )
        self._pub_all = self.create_publisher(
            String,
            '/landmark_database/all',
            10,
        )
        self._pub_count = self.create_publisher(
            Int32,
            '/landmark_database/count',
            10,
        )

        # ---- services ----
        self.create_service(
            Trigger,
            '/landmark_database/get_all',
            self._srv_get_all,
        )
        self.create_service(
            Trigger,
            '/landmark_database/query_by_label',
            self._srv_query_by_label,
        )

        # ---- parameter for label queries ----
        self.declare_parameter('query_label', '')

        # ---- merge radius for landmark deduplication (metres) ----
        self.declare_parameter('merge_radius', 1.5)

        # ---- output JSON file ----
        default_db_file = os.path.join(
            os.path.expanduser('~'), self._DEFAULT_DB_FILENAME
        )
        self.declare_parameter('db_file', default_db_file)
        self._db_file: str = (
            self.get_parameter('db_file').get_parameter_value().string_value
        )
        self.get_logger().info(
            f'Landmark database will be saved to: {self._db_file}'
        )

        # ---- 1 Hz publish timer ----
        self.create_timer(1.0, self._publish_state)

        self.get_logger().info('LandmarkDatabaseNode started')

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._fx = msg.k[0]
        self._fy = msg.k[4]
        self._cx = msg.k[2]
        self._cy = msg.k[5]

    def _on_depth(self, msg: Image) -> None:
        if not _CV_AVAILABLE:
            return
        try:
            self._depth_image = self._bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough'
            )
        except CvBridgeError as exc:
            self.get_logger().warn(f'Depth image conversion error: {exc}')

    def _on_detections(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'Invalid detection JSON: {exc}')
            return

        w = data.get('image_width', 640)
        h = data.get('image_height', 480)

        for det in data.get('detections', []):
            label = det.get('label', 'unknown')
            confidence = float(det.get('confidence', 0.0))
            cx_px = int(det.get('center_x', w / 2))
            cy_px = int(det.get('center_y', h / 2))

            x3d, y3d, z3d = self._pixel_to_3d(cx_px, cy_px)

            attrs = {
                'image_x': cx_px,
                'image_y': cy_px,
                'image_width': w,
                'image_height': h,
            }

            merge_radius = (
                self.get_parameter('merge_radius')
                .get_parameter_value()
                .double_value
            )

            lid, updated = self.db.add_or_update(
                label, x3d, y3d, z3d, confidence,
                merge_radius=merge_radius,
                attributes=attrs,
            )
            self._db_dirty = True
            action = 'Updated' if updated else 'Added'
            self.get_logger().info(
                f'{action} landmark {lid} ({label}) at '
                f'({x3d:.2f}, {y3d:.2f}, {z3d:.2f})  '
                f'conf={confidence:.2f}  total={len(self.db)}'
            )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _pixel_to_3d(self, px: int, py: int):
        """Back-project pixel (px, py) to 3-D using the depth frame.

        Returns a ``(x, y, z)`` tuple in the camera optical frame.
        Falls back to ``(0.0, 0.0, 0.0)`` when depth is unavailable
        or the sampled value is invalid.
        """
        if not _CV_AVAILABLE or self._depth_image is None:
            return 0.0, 0.0, 0.0

        h, w = self._depth_image.shape[:2]
        px = max(0, min(px, w - 1))
        py = max(0, min(py, h - 1))

        depth = float(self._depth_image[py, px])

        # Depth images may be uint16 (mm) or float32 (m).
        if self._depth_image.dtype == np.uint16:
            depth /= 1000.0

        if not math.isfinite(depth) or depth <= 0.0:
            return 0.0, 0.0, 0.0

        x = (px - self._cx) * depth / self._fx
        y = (py - self._cy) * depth / self._fy
        z = depth
        return x, y, z

    # ------------------------------------------------------------------
    # Periodic state publishing
    # ------------------------------------------------------------------

    def _publish_state(self) -> None:
        self._publish_markers()
        self._publish_json()
        self._publish_count()
        self._save_to_file()

    def _save_to_file(self, force: bool = False) -> None:
        """Write the current database to *db_file* as pretty-printed JSON.

        Skips the write when no changes have occurred since the last save
        (unless *force* is ``True``).

        The write is atomic: content is first flushed to a sibling temp file
        and then renamed over the target path so readers never see a partial
        file.
        """
        if not force and not self._db_dirty:
            return

        payload = [e.to_dict() for e in self.db.get_all()]
        db_file = self._db_file
        dir_name = os.path.dirname(db_file) or '.'
        try:
            os.makedirs(dir_name, exist_ok=True)
            fd, tmp_path = tempfile.mkstemp(
                dir=dir_name, prefix='.landmark_db_', suffix='.tmp'
            )
            try:
                with os.fdopen(fd, 'w') as fh:
                    json.dump(payload, fh, indent=2)
                os.replace(tmp_path, db_file)
            except Exception as exc:
                try:
                    os.unlink(tmp_path)
                except OSError:
                    pass
                self.get_logger().warn(
                    f'Error while writing landmark database to {db_file}: {exc}'
                )
                raise
        except OSError as exc:
            self.get_logger().warn(
                f'Could not write landmark database to {db_file}: {exc}'
            )
        else:
            self._db_dirty = False

    def _publish_count(self) -> None:
        msg = Int32()
        msg.data = len(self.db)
        self._pub_count.publish(msg)

    def _publish_markers(self) -> None:
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for entry in self.db.get_all():
            # Parse the numeric suffix from the ID for the marker integer id.
            try:
                marker_int_id = int(entry.landmark_id.rsplit('_', 1)[-1])
            except (ValueError, IndexError):
                marker_int_id = 0

            # --- sphere marker ---
            sphere = Marker()
            sphere.header.frame_id = 'atlas/realsense'
            sphere.header.stamp = stamp
            sphere.ns = 'landmark_spheres'
            sphere.id = marker_int_id
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = entry.x
            sphere.pose.position.y = entry.y
            sphere.pose.position.z = entry.z
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.3
            sphere.scale.y = 0.3
            sphere.scale.z = 0.3
            sphere.color.a = 1.0
            sphere.color.r = 1.0
            sphere.color.g = 0.5
            sphere.color.b = 0.0

            # --- text marker ---
            text = Marker()
            text.header.frame_id = 'atlas/realsense'
            text.header.stamp = stamp
            text.ns = 'landmark_labels'
            text.id = marker_int_id + 10000
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = entry.x
            text.pose.position.y = entry.y
            text.pose.position.z = entry.z + 0.4
            text.pose.orientation.w = 1.0
            text.scale.z = 0.25
            text.color.a = 1.0
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.text = f'{entry.label}\n(seen {entry.detection_count}×)'

            marker_array.markers.append(sphere)
            marker_array.markers.append(text)

        self._pub_markers.publish(marker_array)

    def _publish_json(self) -> None:
        payload = [e.to_dict() for e in self.db.get_all()]
        self._pub_all.publish(String(data=json.dumps(payload)))

    # ------------------------------------------------------------------
    # Services
    # ------------------------------------------------------------------

    def _srv_get_all(self, _request, response):
        """Return all landmarks as a JSON array in ``response.message``."""
        payload = [e.to_dict() for e in self.db.get_all()]
        response.success = True
        response.message = json.dumps(payload)
        return response

    def _srv_query_by_label(self, _request, response):
        """Return landmarks matching ``query_label`` parameter as JSON."""
        label = self.get_parameter('query_label').get_parameter_value().string_value
        if not label:
            response.success = False
            response.message = (
                'Set the query_label parameter before calling this service, e.g.:\n'
                '  ros2 param set /landmark_database_node query_label stop_sign'
            )
            return response

        matches = self.db.query_by_label(label)
        response.success = True
        response.message = json.dumps([e.to_dict() for e in matches])
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LandmarkDatabaseNode()
    try:
        rclpy.spin(node)
    finally:
        node._save_to_file(force=True)
        node.get_logger().info(
            f'Landmark database saved to {node._db_file}'
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

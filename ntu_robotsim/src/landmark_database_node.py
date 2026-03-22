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

Depth requirement
-----------------
3-D positions are computed by back-projecting the detection centre pixel
through the depth image from ``/atlas/rgbd_camera/depth_image``.  If no
depth image has been received (e.g. the camera is not publishing yet) or
the sampled depth value is zero / NaN, the node **skips** that detection
and logs a warning — it never stores a ``(0, 0, 0)`` placeholder.  Watch
the node output for ``"Skipping landmark"`` messages; if you see them,
check that the depth camera topic is active::

    ros2 topic hz /atlas/rgbd_camera/depth_image

``cv_bridge`` and ``numpy`` must also be installed; the node logs an error
at startup if they are missing.

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
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry

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

    # Depth-health warning schedule (in 1 Hz timer ticks = seconds).
    # Warn at these tick counts and then every _DEPTH_WARN_INTERVAL ticks.
    _DEPTH_WARN_TICKS = (5, 35, 65)
    _DEPTH_WARN_INTERVAL = 30

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
        self._depth_received: bool = False
        # ---- latest point cloud ----
        self._latest_point = None
        self._latest_cloud = None

        # ---- robot odometry ----
        self._robot_position = (0.0, 0.0, 0.0)
        self._robot_orientation = (0.0, 0.0, 0.0, 1.0)

        # ---- dirty flag: True when the db has changed since last file write ----
        self._db_dirty: bool = False

        # ---- throttle counter for "no depth" warnings ----
        self._no_depth_warn_count: int = 0
        self._depth_health_ticks: int = 0

        if _CV_AVAILABLE:
            self._bridge = CvBridge()
        else:
            self.get_logger().error(
                'cv_bridge / numpy are not installed – depth back-projection is '
                'disabled and all landmark positions will be unknown.  '
                'Install with: pip install opencv-python && sudo apt install '
                'ros-$ROS_DISTRO-cv-bridge'
            )

        # ---- subscriptions ----
        self.create_subscription(
            String,
            '/yolo_detections',
            self._on_detections,
            10,
        )
        self.create_subscription(
            PointCloud2,
            '/atlas/rgbd_camera/points',
            self._on_pointcloud,
            10,
        )
        self.create_subscription(
            CameraInfo,
            '/atlas/rgbd_camera/camera_info',
            self._on_camera_info,
            10,
        )
        self.create_subscription(
            Odometry,
            '/atlas/odom_ground_truth',
            self._on_odometry,
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

        # ---- load existing database from file ----
        self._load_from_file()

        # ---- 1 Hz publish timer ----
        self.create_timer(1.0, self._publish_state)

        self.get_logger().info('LandmarkDatabaseNode started')

    # ------------------------------------------------------------------
    # Startup loader
    # ------------------------------------------------------------------

    def _load_from_file(self) -> None:
        if not os.path.exists(self._db_file):
            self.get_logger().info('No existing database file found – starting fresh.')
            return
        try:
            with open(self._db_file, 'r') as f:
                entries = json.load(f)
            for e in entries:
                pos = e.get('position', {})
                self.db.add_or_update(
                    label=e['label'],
                    x=pos.get('x', 0.0),
                    y=pos.get('y', 0.0),
                    z=pos.get('z', 0.0),
                    confidence=e.get('confidence', 0.0),
                    merge_radius=0.0,
                    attributes=e.get('attributes', {}),
                )
            self.get_logger().info(
                f'Loaded {len(entries)} landmark(s) from {self._db_file}'
            )
        except Exception as exc:
            self.get_logger().warn(f'Could not load database file: {exc}')

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------

    def _on_odometry(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self._robot_position = (p.x, p.y, p.z)
        self._robot_orientation = (o.x, o.y, o.z, o.w)

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
            if not self._depth_received:
                self._depth_received = True
                self.get_logger().info('First depth frame received – 3-D positions now active.')
        except CvBridgeError as exc:
            self.get_logger().warn(f'Depth image conversion error: {exc}')

    def _on_pointcloud(self, msg: PointCloud2) -> None:
        self._latest_cloud = msg

    def _on_detections(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'Invalid detection JSON: {exc}')
            return

        w = data.get('image_width', 640)
        h = data.get('image_height', 480)

        # Count how many of each label are visible in this frame
        detections_list = data.get('detections', [])
        label_counts = {}
        for det in detections_list:
            lbl = det.get('label', 'unknown')
            label_counts[lbl] = label_counts.get(lbl, 0) + 1

        for det in detections_list:
            label = det.get('label', 'unknown')
            confidence = float(det.get('confidence', 0.0))
            cx_px = int(det.get('center_x', w / 2))
            cy_px = int(det.get('center_y', h / 2))

            pos = None
            if self._latest_cloud is not None:
                try:
                    cloud = self._latest_cloud
                    idx = cy_px * cloud.width + cx_px
                    points = list(pc2.read_points(
                        cloud,
                        field_names=("x", "y", "z"),
                        skip_nans=False,
                    ))
                    if idx < len(points):
                        x, y, z = float(points[idx][0]), float(points[idx][1]), float(points[idx][2])
                        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                            pos = (x, y, z)
                except Exception as e:
                    self.get_logger().warn(f'Point lookup error: {e}')

            if pos is None:
                self.get_logger().warn("No point cloud data yet, skipping detection")
                continue

            x3d, y3d, z3d = pos

            merge_radius = (
                self.get_parameter('merge_radius')
                .get_parameter_value()
                .double_value
            )

            # Check if a landmark of this label already exists
            existing = self.db.query_by_label(label)
            if existing:
                stored = existing[0]
                stored_count = stored.attributes.get('item_count', 1)
                new_count = label_counts.get(label, 1)
                if new_count <= stored_count:
                    continue
                # New scan has higher item count — overwrite
                self.get_logger().info(
                    f'Updating {label}: item_count {stored_count} -> {new_count}'
                )

            px, py, pz = self._robot_position
            ox, oy, oz, ow = self._robot_orientation
            attrs = {
                'image_x': cx_px,
                'image_y': cy_px,
                'image_width': w,
                'image_height': h,
                'item_count': label_counts.get(label, 1),
                'robot_position': {'x': px, 'y': py, 'z': pz},
                'robot_orientation': {'x': ox, 'y': oy, 'z': oz, 'w': ow},
            }
            lid, _ = self.db.add_or_update(
                label, x3d, y3d, z3d, confidence,
                merge_radius=merge_radius,
                attributes=attrs,
            )
            self._db_dirty = True
            self.get_logger().info(
                f'Added landmark {lid} ({label}) at '
                f'({x3d:.2f}, {y3d:.2f}, {z3d:.2f})  '
                f'conf={confidence:.2f}  '
                f'items_in_frame={label_counts.get(label, 1)}  total={len(self.db)}'
            )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _pixel_to_3d(
        self, px: int, py: int
    ) -> Optional[Tuple[float, float, float]]:
        """Back-project pixel (px, py) to 3-D using the depth frame.

        Returns a ``(x, y, z)`` tuple in the camera optical frame, or
        ``None`` when the depth value is unavailable or invalid (cv_bridge
        not installed, no depth frame received yet, or zero/NaN depth at the
        sampled pixel).  Callers must check for ``None`` and must **not**
        store ``(0, 0, 0)`` as a position placeholder.
        """
        if not _CV_AVAILABLE or self._depth_image is None:
            return None

        h, w = self._depth_image.shape[:2]
        px = max(0, min(px, w - 1))
        py = max(0, min(py, h - 1))

        depth = float(self._depth_image[py, px])

        # Depth images may be uint16 (mm) or float32 (m).
        if self._depth_image.dtype == np.uint16:
            depth /= 1000.0

        if not math.isfinite(depth) or depth <= 0.0:
            return None

        x = (px - self._cx) * depth / self._fx
        y = (py - self._cy) * depth / self._fy
        z = depth
        return x, y, z

    # ------------------------------------------------------------------
    # Periodic state publishing
    # ------------------------------------------------------------------

    def _publish_state(self) -> None:
        self._check_depth_health()
        self._publish_markers()
        self._publish_json()
        self._publish_count()
        self._save_to_file()

    def _check_depth_health(self) -> None:
        """Warn periodically if depth images have never been received."""
        if _CV_AVAILABLE and not self._depth_received:
            self._depth_health_ticks += 1
            last_scheduled = self._DEPTH_WARN_TICKS[-1]
            should_warn = (
                self._depth_health_ticks in self._DEPTH_WARN_TICKS
                or (
                    self._depth_health_ticks > last_scheduled
                    and (self._depth_health_ticks - last_scheduled)
                    % self._DEPTH_WARN_INTERVAL == 0
                )
            )
            if should_warn:
                self.get_logger().warn(
                    'No depth image received yet on '
                    '/atlas/rgbd_camera/depth_image – '
                    'landmark 3-D positions will be unknown until '
                    'the depth camera starts publishing.'
                )

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

        def clean(e):
            d = e.to_dict()
            for key in ('confidence', 'first_seen', 'last_seen'):
                d.pop(key, None)
            d['robot_position'] = d.pop('position', {})
            d.get('attributes', {}).pop('robot_position', None)
            return d
        payload = [clean(e) for e in self.db.get_all()]
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
            self.get_logger().info(f'✅ JSON written to {db_file}')

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

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received")

    finally:
        node.get_logger().info("FORCING final database save...")

        try:
            node._save_to_file(force=True)
            node.get_logger().info(
                f'Landmark database saved to {node._db_file}'
            )
        except Exception as e:
            node.get_logger().error(f'FINAL SAVE FAILED: {e}')

        node.destroy_node()

if __name__ == '__main__':
    main()

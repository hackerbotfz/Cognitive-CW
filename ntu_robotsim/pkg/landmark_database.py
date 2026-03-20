#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import json
import math
import os

_REQUIRED_KEYS = {'x': float, 'y': float, 'z': float, 'type': str, 'count': int}


def _is_valid_landmark(entry) -> bool:
    """Return True only if *entry* is a dict with all required keys and sane types."""
    if not isinstance(entry, dict):
        return False
    for key, expected_type in _REQUIRED_KEYS.items():
        value = entry.get(key)
        if value is None:
            return False
        if not isinstance(value, expected_type if expected_type is not float else (int, float)):
            return False
    if not all(math.isfinite(entry[k]) for k in ('x', 'y', 'z')):
        return False
    if entry['count'] < 1:
        return False
    return True


class LandmarkDatabase(Node):
    def __init__(self):
        super().__init__('landmark_database')

        self.declare_parameter('database_file', 'landmark_db.json')
        self.declare_parameter('distance_threshold', 0.5)
        self.declare_parameter('map_frame', 'map')

        self.db_file     = self.get_parameter('database_file').get_parameter_value().string_value
        self.dist_thresh = self.get_parameter('distance_threshold').get_parameter_value().double_value
        self.map_frame   = self.get_parameter('map_frame').get_parameter_value().string_value

        self.landmarks: dict = {}

        self._dirty: bool = False

        self.load_database()

        self.subscription = self.create_subscription(
            MarkerArray,
            '/detected_markers',
            self.marker_callback,
            10)

        self.db_publisher = self.create_publisher(MarkerArray, '/landmark_database_vis', 10)

        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info('Landmark Database Node Started')

    def load_database(self):
        """Load landmarks from JSON, validating structure and every entry."""
        if not os.path.exists(self.db_file):
            return

        try:
            with open(self.db_file, 'r') as f:
                raw = json.load(f)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Database file is not valid JSON, starting fresh: {e}')
            return
        except OSError as e:
            self.get_logger().error(f'Could not read database file: {e}')
            return

        if not isinstance(raw, dict):
            self.get_logger().error(
                f'Database root is {type(raw).__name__}, expected dict — starting fresh.')
            return

        valid, skipped = {}, 0
        for lm_id, entry in raw.items():
            if not isinstance(lm_id, str) or not lm_id.isdigit():
                self.get_logger().warning(
                    f'Skipping landmark with non-integer key: "{lm_id}"')
                skipped += 1
                continue
            if not _is_valid_landmark(entry):
                self.get_logger().warning(
                    f'Skipping malformed landmark entry for id "{lm_id}": {entry}')
                skipped += 1
                continue
            valid[lm_id] = entry

        self.landmarks = valid
        self.get_logger().info(
            f'Loaded {len(self.landmarks)} landmarks '
            f'({skipped} skipped as malformed).')

    def save_database(self):
        """Atomically write landmarks to disk using a temp-file + rename."""
        if not self._dirty:
            return

        tmp_file = self.db_file + '.tmp'
        try:
            with open(tmp_file, 'w') as f:
                json.dump(self.landmarks, f, indent=4)
            os.replace(tmp_file, self.db_file)
            self._dirty = False
        except OSError as e:
            self.get_logger().error(f'Failed to save database: {e}')
            try:
                if os.path.exists(tmp_file):
                    os.unlink(tmp_file)
            except OSError:
                pass

    def is_duplicate(self, pos) -> 'str | None':
        """Return the ID of an existing landmark within dist_thresh, or None."""
        for lm_id, data in self.landmarks.items():
            try:
                dist = math.sqrt(
                    (data['x'] - pos.x) ** 2 +
                    (data['y'] - pos.y) ** 2 +
                    (data['z'] - pos.z) ** 2
                )
            except (KeyError, TypeError):
                continue
            if dist < self.dist_thresh:
                return lm_id
        return None

    def marker_callback(self, msg: MarkerArray):
        for marker in msg.markers:
            if marker.action in (Marker.DELETE, Marker.DELETEALL):
                continue

            pos = marker.pose.position

            if not all(math.isfinite(v) for v in (pos.x, pos.y, pos.z)):
                self.get_logger().warning(
                    f'Discarding marker with non-finite position '
                    f'({pos.x}, {pos.y}, {pos.z})')
                continue

            existing_id = self.is_duplicate(pos)

            if existing_id is not None:
                lm = self.landmarks[existing_id]
                n = lm['count']

                effective_n = min(n, 10_000)
                lm['x'] = (lm['x'] * effective_n + pos.x) / (effective_n + 1)
                lm['y'] = (lm['y'] * effective_n + pos.y) / (effective_n + 1)
                lm['z'] = (lm['z'] * effective_n + pos.z) / (effective_n + 1)
                lm['count'] = n + 1
                self._dirty = True
                self.get_logger().debug(f'Updated landmark {existing_id}')
            else:
                existing_ids = [int(k) for k in self.landmarks.keys()]
                new_id = str(max(existing_ids) + 1 if existing_ids else 1)

                label = marker.text.strip() if marker.text else ''
                label = label or f'landmark_{new_id}'

                self.landmarks[new_id] = {
                    'x': pos.x,
                    'y': pos.y,
                    'z': pos.z,
                    'type': label,
                    'count': 1
                }
                self._dirty = True
                self.get_logger().info(
                    f'Added new landmark {new_id} ({label}) '
                    f'at ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})')

    def timer_callback(self):
        self.save_database()
        self.publish_database_markers()

    def publish_database_markers(self):
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for lm_id, data in self.landmarks.items():
            if not lm_id.isdigit():
                continue

            if not _is_valid_landmark(data):
                self.get_logger().warning(
                    f'Skipping malformed landmark {lm_id} during publish')
                continue

            numeric_id = int(lm_id)

            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = stamp
            m.id = numeric_id
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(data['x'])
            m.pose.position.y = float(data['y'])
            m.pose.position.z = float(data['z'])
            m.pose.orientation.w = 1.0
            m.scale.x = 0.2
            m.scale.y = 0.2
            m.scale.z = 0.2
            m.color.a = 1.0
            m.color.g = 1.0
            m.text = data['type']

            t = Marker()
            t.header.frame_id = self.map_frame
            t.header.stamp = stamp
            t.id = numeric_id + 100_000
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = float(data['x'])
            t.pose.position.y = float(data['y'])
            t.pose.position.z = float(data['z']) + 0.3
            t.scale.z = 0.2
            t.color.a = 1.0
            t.color.r = 1.0
            t.color.g = 1.0
            t.color.b = 1.0
            t.text = f"{data['type']} ({lm_id})"

            ma.markers.append(m)
            ma.markers.append(t)

        self.db_publisher.publish(ma)


def main(args=None):
    rclpy_initialised = False
    node = None
    try:
        rclpy.init(args=args)
        rclpy_initialised = True
        node = LandmarkDatabase()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.save_database()
            node.destroy_node()
        if rclpy_initialised:
            rclpy.shutdown()


if __name__ == '__main__':
    main()

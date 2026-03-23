#!/usr/bin/env python3

"""
traffic_sign_navigator.py  –  Task 5: Traffic Rules

Subscribes to goal_detector detections and adjusts the robot's velocity
based on recognised traffic signs and object posters.

Two behaviour layers:
    1. TRAFFIC STATE (persistent) – stays until a different sign overrides it
         slow_sign  → slow  (×0.3)   stays slow until fast_sign or stop_sign
         fast_sign  → fast  (×1.3)   stays fast until slow_sign or stop_sign
         stop_sign  → stop  (×0.0)   stops PERMANENTLY (end of maze)
         (default)  → normal (×1.0)

    2. SCAN MODE (temporary) – triggered by object posters (oranges, trees, cars)
         Robot STOPS to scan for scan_duration seconds,
         then resumes whatever the current traffic state speed is.

Architecture:

    ┌──────────┐      /teleop_cmd_vel       ┌─────────────────────┐      /atlas/cmd_vel     ┌───────┐

    │  teleop  │ ──────────────────────────▶│ traffic_sign_nav    │────────────────────────▶│ robot │

    └──────────┘                            │                     │                         └───────┘

                      /goal_detections      │   persistent:       │

    ┌──────────┐ ──────────────────────────▶│   slow/fast/stop    │

    │goal_det. │                            │   temporary:        │

    └──────────┘                            │   scan slowdown     │

                                            └─────────────────────┘

How to run:

    1. Start simulation + goal_detector.py (see Task 4 guide)
    2. python3 traffic_sign_navigator.py
    3. In another terminal, run teleop REMAPPED to /teleop_cmd_vel:
         ros2 run teleop_twist_keyboard teleop_twist_keyboard \
             --ros-args -r /cmd_vel:=/teleop_cmd_vel

Subscribes:
    /teleop_cmd_vel     (geometry_msgs/Twist)  – raw driver input
    /goal_detections    (std_msgs/String)      – JSON from goal_detector

Publishes:
    /atlas/cmd_vel      (geometry_msgs/Twist)  – scaled velocity to robot
    /traffic_state      (std_msgs/String)      – current state for debugging
    /scan_status        (std_msgs/String)      – "scanning <class>" or "idle"

Author : Danushka Matteo
Date   : 2025
"""

import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class TrafficSignNavigator(Node):
    """Intercepts teleop commands and scales speed based on traffic sign detections."""
    # ── Speed multipliers for each traffic state ───────────────────
    SPEED_FACTORS = {
        'stop':   0.0,
        'slow':   0.3,
        'normal': 1.0,
        'fast':   1.3,
    }

    # ── YOLO class name → traffic state (persistent) ──────────────
    SIGN_TO_STATE = {
        'stop_sign': 'stop',
        'slow_sign': 'slow',
        'fast_sign': 'fast',
    }

    # ── Object poster classes that trigger scan mode (temporary) ───
    SCAN_OBJECTS = {'orange', 'tree', 'car'}

    def __init__(self):
        super().__init__('traffic_sign_navigator')
        # ── ROS parameters ─────────────────────────────────────────
        self.declare_parameter('area_threshold', 0.005)     # sign must be ≥0.5% of image
        self.declare_parameter('scan_duration', 5.0)        # seconds to scan a poster
        self.declare_parameter('cmd_vel_topic', '/atlas/cmd_vel')
        self.declare_parameter('teleop_topic', '/teleop_cmd_vel')

        self.area_thresh      = self.get_parameter('area_threshold').get_parameter_value().double_value
        self.scan_duration    = self.get_parameter('scan_duration').get_parameter_value().double_value

        cmd_vel_topic         = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        teleop_topic          = self.get_parameter('teleop_topic').get_parameter_value().string_value

        # ── Persistent traffic state ───────────────────────────────
        #    Only changes when a NEW traffic sign is detected.
        #    Does NOT revert when the sign leaves the camera.
        self.traffic_state = 'normal'

        # ── Stop sign state ────────────────────────────────────────
        self.is_stopped = False

        # ── Scan mode state (temporary overlay) ────────────────────
        self.is_scanning = False
        self.scan_timer = None
        self.scan_target = ''                   # which poster is being scanned
        self.scanned_objects = set()            # posters already scanned (don't re-trigger)

        # ── Teleop ─────────────────────────────────────────────────
        self.last_teleop = Twist()

        # ── Subscribers ────────────────────────────────────────────
        self.teleop_sub = self.create_subscription(
            Twist, teleop_topic, self.teleop_callback, 10)
        self.detection_sub = self.create_subscription(
            String, '/goal_detections', self.detection_callback, 10)

        # ── Publishers ─────────────────────────────────────────────
        self.cmd_vel_pub  = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.state_pub    = self.create_publisher(String, '/traffic_state', 10)
        self.scan_pub     = self.create_publisher(String, '/scan_status', 10)

        # ── 10 Hz velocity publisher ───────────────────────────────
        self.vel_timer = self.create_timer(0.1, self.publish_cmd_vel)
        self.get_logger().info(
            f'Traffic Sign Navigator ready  |  '
            f'teleop←{teleop_topic}  cmd_vel→{cmd_vel_topic}  '
            f'area≥{self.area_thresh}  scan_time={self.scan_duration}s'
        )

    # ────────────────────────────────────────────────────────────────

    #  TELEOP CALLBACK

    # ────────────────────────────────────────────────────────────────

    def teleop_callback(self, msg):
        self.last_teleop = msg

    # ────────────────────────────────────────────────────────────────

    #  DETECTION CALLBACK

    # ────────────────────────────────────────────────────────────────

    def detection_callback(self, msg):
        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        # ── Separate traffic signs from object posters ─────────────
        best_sign = None
        best_sign_area = 0.0

        best_object = None
        best_object_area = 0.0

        for det in detections:
            cls  = det.get('class', '')
            area = det.get('area_ratio', 0.0)

            if area < self.area_thresh:
                continue

            if cls in self.SIGN_TO_STATE and area > best_sign_area:
                best_sign = cls
                best_sign_area = area

            if cls in self.SCAN_OBJECTS and area > best_object_area:
                best_object = cls
                best_object_area = area

        # ── Handle traffic signs (persistent state change) ─────────
        if best_sign is not None:
            new_state = self.SIGN_TO_STATE[best_sign]
            self.handle_traffic_sign(new_state)

        # ── Handle object posters (temporary scan slowdown) ────────
        if best_object is not None:
            self.handle_scan_object(best_object)

    # ────────────────────────────────────────────────────────────────

    #  TRAFFIC SIGN HANDLER (persistent)

    # ────────────────────────────────────────────────────────────────

    def handle_traffic_sign(self, new_state):
        """
        Update persistent traffic state.
        - slow/fast: set immediately, stays until next sign
        - stop: permanent stop (end of maze)
        """

        # ── STOP sign (permanent – robot stops forever) ─────────────
        if new_state == 'stop':
            if not self.is_stopped:
                self.is_stopped = True
                self.set_traffic_state('stop')
                self.get_logger().info(
                    'STOP sign detected – robot stopped permanently'
                )
            return

        # ── SLOW / FAST sign ───────────────────────────────────────
        if not self.is_stopped:
            # Only change if it's actually different
            if new_state != self.traffic_state:
                self.set_traffic_state(new_state)

    # ────────────────────────────────────────────────────────────────

    #  SCAN OBJECT HANDLER (temporary slowdown)

    # ────────────────────────────────────────────────────────────────
    def handle_scan_object(self, cls_name):

        """
        Temporarily slow down to scan an object poster.
        Only triggers once per poster type to avoid re-scanning
        the same poster every frame.
        """

        if cls_name in self.scanned_objects:
            return                              # already scanned this one

        if self.is_scanning:
            return                              # already scanning something

        self.is_scanning = True
        self.scan_target = cls_name

        self.get_logger().info(
            f'SCANNING "{cls_name}" – stopping to scan '
            f'for {self.scan_duration}s'
        )

        # Publish scan status
        scan_msg = String()
        scan_msg.data = f'scanning {cls_name}'
        self.scan_pub.publish(scan_msg)

        # Start scan timer
        if self.scan_timer is not None:
            self.scan_timer.cancel()
        self.scan_timer = self.create_timer(
            self.scan_duration, self.scan_timeout)

    # ────────────────────────────────────────────────────────────────

    #  SCAN TIMEOUT

    # ────────────────────────────────────────────────────────────────

    def scan_timeout(self):
        """Scan complete: mark as scanned, resume previous traffic speed."""
        self.get_logger().info(
            f'Scan of "{self.scan_target}" complete – '
            f'resuming traffic state "{self.traffic_state}" (×{self.SPEED_FACTORS[self.traffic_state]})'
        )

        self.scanned_objects.add(self.scan_target)
        self.is_scanning = False
        self.scan_target = ''

        # Publish idle status
        scan_msg = String()
        scan_msg.data = 'idle'
        self.scan_pub.publish(scan_msg)

        if self.scan_timer is not None:
            self.scan_timer.cancel()
            self.scan_timer = None

    # ────────────────────────────────────────────────────────────────

    #  SET TRAFFIC STATE + LOG

    # ────────────────────────────────────────────────────────────────

    def set_traffic_state(self, new_state):
        if new_state != self.traffic_state:
            old = self.traffic_state
            self.traffic_state = new_state
            factor = self.SPEED_FACTORS[new_state]
            self.get_logger().info(
                f'Traffic state: {old} → {new_state}  (speed ×{factor})'
            )

            state_msg = String()
            state_msg.data = new_state
            self.state_pub.publish(state_msg)

    # ────────────────────────────────────────────────────────────────

    #  PUBLISH CMD_VEL – 10 Hz

    # ────────────────────────────────────────────────────────────────

    def publish_cmd_vel(self):
        """
        Determine the effective speed factor and scale teleop output.

        Priority:
            1. is_stopped  → factor = 0.0  (stop sign – permanent)
            2. is_scanning → factor = 0.0  (stop to scan poster)
            3. otherwise   → factor = traffic_state factor (persistent)
        """

        if self.is_stopped or self.is_scanning:
            factor = 0.0
        else:
            factor = self.SPEED_FACTORS[self.traffic_state]

        scaled = Twist()

        scaled.linear.x  = self.last_teleop.linear.x  * factor
        scaled.linear.y  = self.last_teleop.linear.y  * factor
        scaled.linear.z  = self.last_teleop.linear.z  * factor

        scaled.angular.x = self.last_teleop.angular.x * factor
        scaled.angular.y = self.last_teleop.angular.y * factor
        scaled.angular.z = self.last_teleop.angular.z * factor

        self.cmd_vel_pub.publish(scaled)

# ════════════════════════════════════════════════════════════════════

#  ENTRY POINT

# ════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = TrafficSignNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Traffic Sign Navigator...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':

    main()
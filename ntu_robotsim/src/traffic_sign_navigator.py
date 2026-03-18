#!/usr/bin/env python3
"""
traffic_sign_navigator.py  –  Task 5: Traffic Rules

Subscribes to goal_detector detections and adjusts the robot's velocity
based on recognised traffic signs.

Architecture (teleop interceptor):
    ┌──────────┐      /teleop_cmd_vel      ┌─────────────────────┐     /atlas/cmd_vel     ┌───────┐
    │  teleop  │ ──────────────────────────▶│ traffic_sign_nav    │────────────────────────▶│ robot │
    └──────────┘                            │                     │                         └───────┘
                  /goal_detections           │  speed_factor:      │
    ┌──────────┐ ──────────────────────────▶│   stop  → 0.0       │
    │goal_det. │                            │   slow  → 0.3       │
    └──────────┘                            │   normal→ 1.0       │
                                            │   fast  → 1.8       │
                                            └─────────────────────┘

How to run:
    1. Start Unified Launch + goal_detector.py
    2. python3 traffic_sign_navigator.py


Subscribes:
    /teleop_cmd_vel     (geometry_msgs/Twist)  – raw driver input
    /goal_detections    (std_msgs/String)      – JSON from goal_detector

Publishes:
    /atlas/cmd_vel      (geometry_msgs/Twist)  – scaled velocity to robot
    /traffic_state      (std_msgs/String)      – current traffic state for debugging

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
        'fast':   1.8,
    }

    # ── Map YOLO class names → traffic state ───────────────────────
    SIGN_TO_STATE = {
        'stop_sign': 'stop',
        'slow_sign': 'slow',
        'fast_sign': 'fast',
    }

    def __init__(self):
        super().__init__('traffic_sign_navigator')

        # ── ROS parameters ─────────────────────────────────────────
        self.declare_parameter('area_threshold', 0.005)     # sign must occupy ≥0.5% of image
        self.declare_parameter('stop_duration', 3.0)        # seconds to remain stopped
        self.declare_parameter('cmd_vel_topic', '/atlas/cmd_vel')
        self.declare_parameter('teleop_topic', '/teleop_cmd_vel')

        self.area_thresh = self.get_parameter('area_threshold').get_parameter_value().double_value
        self.stop_duration = self.get_parameter('stop_duration').get_parameter_value().double_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        teleop_topic = self.get_parameter('teleop_topic').get_parameter_value().string_value

        # ── State ──────────────────────────────────────────────────
        self.traffic_state = 'normal'
        self.last_teleop = Twist()              # most recent teleop command
        self.stop_timer = None                  # active countdown after stop sign
        self.previously_detected_signs = set()  # avoid re-triggering same sign continuously

        # ── Subscribers ────────────────────────────────────────────
        self.teleop_sub = self.create_subscription(
            Twist, teleop_topic, self.teleop_callback, 10)
        self.detection_sub = self.create_subscription(
            String, '/goal_detections', self.detection_callback, 10)

        # ── Publishers ─────────────────────────────────────────────
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.state_pub   = self.create_publisher(String, '/traffic_state', 10)

        # ── Periodic publisher (10 Hz) to keep robot moving/stopped ─
        self.timer = self.create_timer(0.1, self.publish_cmd_vel)

        self.get_logger().info(
            f'Traffic Sign Navigator ready  |  '
            f'teleop←{teleop_topic}  cmd_vel→{cmd_vel_topic}  '
            f'area_thresh={self.area_thresh}'
        )

    # ────────────────────────────────────────────────────────────────
    #  TELEOP CALLBACK – store latest driver input
    # ────────────────────────────────────────────────────────────────
    def teleop_callback(self, msg):
        """Cache the latest teleop command."""
        self.last_teleop = msg

    # ────────────────────────────────────────────────────────────────
    #  DETECTION CALLBACK – update traffic state from sign detections
    # ────────────────────────────────────────────────────────────────
    def detection_callback(self, msg):
        """Parse goal_detector JSON and update traffic state."""
        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        # Find the most relevant traffic sign (highest area = closest)
        best_sign = None
        best_area = 0.0

        for det in detections:
            cls_name = det.get('class', '')
            area     = det.get('area_ratio', 0.0)

            # Only consider known traffic signs that are close enough
            if cls_name in self.SIGN_TO_STATE and area >= self.area_thresh:
                if area > best_area:
                    best_area = area
                    best_sign = cls_name

        if best_sign is None:
            # No traffic sign in view (or too far away) → revert to normal
            # but only if we're not in a timed stop
            if self.stop_timer is None and self.traffic_state != 'normal':
                self.set_traffic_state('normal')
            return

        new_state = self.SIGN_TO_STATE[best_sign]

        # ── Handle stop sign specially (timed stop) ────────────────
        if new_state == 'stop' and self.traffic_state != 'stop':
            self.set_traffic_state('stop')
            # Start a timer: after stop_duration seconds, resume normal
            if self.stop_timer is not None:
                self.stop_timer.cancel()
            self.stop_timer = self.create_timer(
                self.stop_duration, self.stop_timeout, callback_group=None)
            return

        # ── For slow / fast, update immediately ────────────────────
        if new_state != 'stop' and self.stop_timer is None:
            self.set_traffic_state(new_state)

    # ────────────────────────────────────────────────────────────────
    #  STOP TIMEOUT – resume after stop sign pause
    # ────────────────────────────────────────────────────────────────
    def stop_timeout(self):
        """Called once after stop_duration seconds; resumes normal driving."""
        self.get_logger().info('Stop duration elapsed – resuming normal speed')
        self.set_traffic_state('normal')
        if self.stop_timer is not None:
            self.stop_timer.cancel()
            self.stop_timer = None

    # ────────────────────────────────────────────────────────────────
    #  SET STATE + LOG
    # ────────────────────────────────────────────────────────────────
    def set_traffic_state(self, new_state):
        if new_state != self.traffic_state:
            factor = self.SPEED_FACTORS[new_state]
            self.get_logger().info(
                f'Traffic state: {self.traffic_state} → {new_state}  '
                f'(speed ×{factor})'
            )
            self.traffic_state = new_state

            # Publish state for other nodes / debugging
            state_msg = String()
            state_msg.data = new_state
            self.state_pub.publish(state_msg)

    # ────────────────────────────────────────────────────────────────
    #  PUBLISH CMD_VEL – 10 Hz timer callback
    # ────────────────────────────────────────────────────────────────
    def publish_cmd_vel(self):
        """Scale the latest teleop command by current speed factor and publish."""
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
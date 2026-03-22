#!/usr/bin/env python3
"""
check_landmark_db – CLI utility to inspect the Landmark Database.

Calls the ``/landmark_database/get_all`` service and prints a
human-readable summary of every stored landmark.

Usage
-----
After building and sourcing the workspace::

    ros2 run ntu_robotsim check_landmark_db.py

Or equivalently::

    python3 check_landmark_db.py
"""

import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class LandmarkDatabaseChecker(Node):
    """One-shot ROS2 node that queries the landmark database and exits."""

    def __init__(self) -> None:
        super().__init__('landmark_database_checker')
        self._client = self.create_client(Trigger, '/landmark_database/get_all')

    def run(self) -> int:
        """Wait for the service, call it, print the results, and return an exit code."""
        self.get_logger().info(
            'Waiting for /landmark_database/get_all service...'
        )
        if not self._client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                'Service /landmark_database/get_all not available. '
                'Is landmark_database_node running?'
            )
            return 1

        future = self._client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is None:
            self.get_logger().error('Service call timed out.')
            return 1

        response = future.result()
        if not response.success:
            self.get_logger().error(f'Service call failed: {response.message}')
            return 1

        try:
            landmarks = json.loads(response.message)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Failed to parse response JSON: {exc}')
            return 1

        self._print_database(landmarks)
        return 0

    # ------------------------------------------------------------------
    # Pretty-print helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _fmt_time(ts: float) -> str:
        return time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(ts))

    def _print_database(self, landmarks: list) -> None:
        sep = '─' * 60
        print()
        print('╔' + '═' * 58 + '╗')
        print(f'║  Landmark Database  ({len(landmarks)} landmark(s) stored)'
              .ljust(59) + '║')
        print('╚' + '═' * 58 + '╝')

        if not landmarks:
            print('  (database is empty)')
            print()
            return

        for i, lm in enumerate(landmarks):
            print(sep)
            pos = lm.get('position', {})
            print(
                f"  [{i}]  id          : {lm.get('landmark_id', '?')}"
            )
            print(f"       label       : {lm.get('label', '?')}")
            print(
                f"       position    : x={pos.get('x', 0.0):.3f}  "
                f"y={pos.get('y', 0.0):.3f}  "
                f"z={pos.get('z', 0.0):.3f}"
            )
            print(
                f"       confidence  : {lm.get('confidence', 0.0):.3f}"
            )
            print(
                f"       detections  : {lm.get('detection_count', 0)}"
            )
            print(
                f"       first seen  : {self._fmt_time(lm.get('first_seen', 0.0))}"
            )
            print(
                f"       last seen   : {self._fmt_time(lm.get('last_seen', 0.0))}"
            )
            attrs = lm.get('attributes', {})
            if attrs:
                print(f"       attributes  : {json.dumps(attrs)}")

        print(sep)
        print()


def main(args=None) -> None:
    rclpy.init(args=args)
    checker = LandmarkDatabaseChecker()
    exit_code = checker.run()
    checker.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()

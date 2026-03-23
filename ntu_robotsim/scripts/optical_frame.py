#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo


class OpticalFrame(Node):

    def __init__(self):
        super().__init__('optical_frame')

        # Parameter: target optical frame
        self.declare_parameter('optical_frame', 'atlas/optical')
        self.optical_frame = self.get_parameter('optical_frame').value

        # -------- Subscribers --------
        self.rgb_sub = self.create_subscription(
            Image,
            '/atlas/rgbd_camera/image',
            self.rgb_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/atlas/rgbd_camera/depth_image',
            self.depth_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/atlas/rgbd_camera/camera_info',
            self.camera_info_callback,
            10
        )

        # -------- Publishers --------
        self.rgb_pub = self.create_publisher(
            Image,
            '/atlas/rgbd_camera/optical_image',
            10
        )

        self.depth_pub = self.create_publisher(
            Image,
            '/atlas/rgbd_camera/optical_depth_image',
            10
        )

        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            '/atlas/rgbd_camera/optical_camera_info',
            10
        )

        self.get_logger().info(f'Optical frame relay started → {self.optical_frame}')

    # -------- Callbacks --------

    def rgb_callback(self, msg: Image):
        msg.header.frame_id = self.optical_frame
        self.rgb_pub.publish(msg)

    def depth_callback(self, msg: Image):
        msg.header.frame_id = self.optical_frame
        self.depth_pub.publish(msg)

    def camera_info_callback(self, msg: CameraInfo):
        msg.header.frame_id = self.optical_frame
        self.camera_info_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OpticalFrame()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3

import os

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO


class YoloDetector(Node):

    def __init__(self):
        super().__init__('yolo_detector')

        self.declare_parameter('model_path', '')
        self.declare_parameter('image_topic', '/rgbd_camera/image')

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        if not model_path:
            model_path = os.path.join(
                get_package_share_directory('ntu_robotsim'),
                'models',
                'best.pt',
            )

        if os.path.isfile(model_path):
            self.get_logger().info(f'Loading YOLO weights from {model_path}')
            self.model = YOLO(model_path)
        else:
            self.get_logger().warn(
                f'Weights not found at {model_path}; using yolov8n.pt. '
                'Place best.pt in share/ntu_robotsim/models/ for your trained model.'
            )
            self.model = YOLO('yolov8n.pt')

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10,
        )
        self.get_logger().info('YOLO detector started')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame)
        annotated = results[0].plot()
        cv2.imshow('YOLO Detection', annotated)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

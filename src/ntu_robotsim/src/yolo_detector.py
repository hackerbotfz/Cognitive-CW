#!/usr/bin/env python3

import gzip
import os
import shutil

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO


def resolve_model_path(models_dir: str):
    """Return a loadable .pt path from best.pt or best.pt.gz in the package models folder."""
    pt_path = os.path.join(models_dir, 'best.pt')
    gz_path = os.path.join(models_dir, 'best.pt.gz')

    if os.path.isfile(pt_path):
        return pt_path

    if not os.path.isfile(gz_path):
        return None

    cache_dir = os.path.join(os.path.expanduser('~/.cache/ntu_robotsim'), 'models')
    os.makedirs(cache_dir, exist_ok=True)
    cached_pt = os.path.join(cache_dir, 'best.pt')

    if not os.path.isfile(cached_pt) or os.path.getmtime(gz_path) > os.path.getmtime(cached_pt):
        with gzip.open(gz_path, 'rb') as f_in, open(cached_pt, 'wb') as f_out:
            shutil.copyfileobj(f_in, f_out)

    return cached_pt


class YoloDetector(Node):

    def __init__(self):
        super().__init__('yolo_detector')

        self.declare_parameter('model_path', '')
        self.declare_parameter('image_topic', '/rgbd_camera/image')

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        if not model_path:
            models_dir = os.path.join(get_package_share_directory('ntu_robotsim'), 'models')
            model_path = resolve_model_path(models_dir)

        if model_path and os.path.isfile(model_path):
            self.get_logger().info(f'Loading YOLO weights from {model_path}')
            self.model = YOLO(model_path)
        else:
            self.get_logger().warn(
                'Trained weights not found (best.pt / best.pt.gz); using yolov8n.pt.'
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

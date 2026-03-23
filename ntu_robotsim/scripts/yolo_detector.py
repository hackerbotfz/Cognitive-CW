#!/usr/bin/env python3

import json

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO


class YoloDetector(Node):

    def __init__(self):
        super().__init__('yolo_detector')

        self.bridge = CvBridge()

        # load your trained model
        self.model = YOLO("/home/ntu-user/ros2_ws/install/ntu_robotsim/share/ntu_robotsim/models/best.pt")

        self.subscription = self.create_subscription(
            Image,
            '/atlas/rgbd_camera/image',
            self.image_callback,
            10
        )

        # Publish detections so the landmark_database_node can consume them
        self._pub_detections = self.create_publisher(String, '/yolo_detections', 10)

        self.get_logger().info("YOLO detector started")

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]

        results = self.model(frame)

        annotated = results[0].plot()

        cv2.imshow("YOLO Detection", annotated)
        cv2.waitKey(1)

        # Build and publish detection results for the landmark database
        detections = []
        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            label = self.model.names[cls_id]
            confidence = float(box.conf[0])
            x1, y1, x2, y2 = map(float, box.xyxy[0])
            detections.append({
                'label': label,
                'confidence': confidence,
                'bbox': {'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2},
                'center_x': (x1 + x2) / 2.0,
                'center_y': (y1 + y2) / 2.0,
            })

        self._pub_detections.publish(String(data=json.dumps({
            'detections': detections,
            'image_width': w,
            'image_height': h,
        })))


def main(args=None):

    rclpy.init(args=args)
    node = YoloDetector()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

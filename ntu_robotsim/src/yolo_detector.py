#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO


class YoloDetector(Node):

    def __init__(self):
        super().__init__('yolo_detector')

        self.bridge = CvBridge()

        # load your trained model
        self.model = YOLO("/home/ntu-user/ros2_ws/src/NTU_COMP30271_CW_RobotSim/ntu_robotsim/src/best.pt")

        self.subscription = self.create_subscription(
            Image,
            '/atlas/rgbd_camera/image',
            self.image_callback,
            10
        )

        self.get_logger().info("YOLO detector started")

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        results = self.model(frame)

        annotated = results[0].plot()

        cv2.imshow("YOLO Detection", annotated)
        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)
    node = YoloDetector()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
goal_detector.py  –  Task 4: Goal Position Detection

Detects goal markers (stop_sign, slow_sign, fast_sign, oranges, trees, cars)
using a custom-trained YOLOv8 model and publishes structured detection results
so that other nodes (e.g. traffic_navigator, object_counter) can consume them.

Subscribes:
    /atlas/rgbd_camera/image   (sensor_msgs/Image)

Publishes:
    /goal_detections            (std_msgs/String)   – JSON list of detections
    /yolo/annotated_image       (sensor_msgs/Image)  – debug image with boxes

Author : Danushka Matteo
Date   : 2025
"""

import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO


class GoalDetector(Node):

    def __init__(self):
        super().__init__('goal_detector')

        # ── 1. Declare ROS parameters (tuneable at launch) ──────────────
        self.declare_parameter('model_path',
            '/home/ntu-user/ros2_ws/src/Cognitive-CW/ntu_robotsim/src/best.pt')
        self.declare_parameter('confidence_threshold', 0.60)
        self.declare_parameter('camera_topic', '/atlas/rgbd_camera/image')
        self.declare_parameter('show_gui', True)            # set False on headless machines

        # Read parameter values
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.conf_thresh = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.show_gui = self.get_parameter('show_gui').get_parameter_value().bool_value

        # ── 2. Load YOLO model ──────────────────────────────────────────
        self.get_logger().info(f'Loading YOLO model from: {model_path}')
        self.model = YOLO(model_path)
        self.get_logger().info('YOLO model loaded successfully')

        # ── 3. CV bridge for ROS ↔ OpenCV conversion ───────────────────
        self.bridge = CvBridge()

        # ── 4. Subscriber – camera images ──────────────────────────────
        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )

        # ── 5. Publishers ──────────────────────────────────────────────
        #   a) Structured detections as JSON string
        self.detection_pub = self.create_publisher(String, '/goal_detections', 10)
        #   b) Annotated image for debugging / rviz
        self.annotated_pub = self.create_publisher(Image, '/yolo/annotated_image', 10)

        # ── 6. Internal state ──────────────────────────────────────────
        self.frame_count = 0
        self.log_interval = 30          # log summary every N frames

        self.get_logger().info(
            f'Goal Detector ready  |  topic={camera_topic}  '
            f'conf≥{self.conf_thresh}  gui={self.show_gui}'
        )

    # ────────────────────────────────────────────────────────────────────
    #  IMAGE CALLBACK – runs on every incoming camera frame
    # ────────────────────────────────────────────────────────────────────
    def image_callback(self, msg):
        """Process one camera frame: detect → publish → (optional) display."""

        # Convert ROS Image → OpenCV BGR
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run YOLO inference with confidence filter
        results = self.model(frame, conf=self.conf_thresh, verbose=False)

        # ── Build a list of detections ──────────────────────────────
        detections = []
        for box in results[0].boxes:
            cls_id   = int(box.cls[0])
            cls_name = self.model.names[cls_id]
            conf     = float(box.conf[0])
            x1, y1, x2, y2 = box.xyxy[0].tolist()

            # Bounding-box centre (useful for steering towards a goal)
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0

            # Bounding-box area as fraction of image (proxy for distance)
            img_h, img_w = frame.shape[:2]
            bbox_area_ratio = ((x2 - x1) * (y2 - y1)) / (img_w * img_h)

            detections.append({
                'class':      cls_name,
                'confidence': round(conf, 3),
                'bbox':       [round(v, 1) for v in [x1, y1, x2, y2]],
                'centre':     [round(cx, 1), round(cy, 1)],
                'area_ratio': round(bbox_area_ratio, 4),
            })

        # ── Publish detections as JSON string ───────────────────────
        det_msg = String()
        det_msg.data = json.dumps(detections)
        self.detection_pub.publish(det_msg)

        # ── Publish annotated image ─────────────────────────────────
        annotated = results[0].plot()
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        self.annotated_pub.publish(annotated_msg)

        # ── Optional: show in OpenCV window ─────────────────────────
        if self.show_gui:
            cv2.imshow('Goal Detector', annotated)
            cv2.waitKey(1)

        # ── Periodic logging ────────────────────────────────────────
        self.frame_count += 1
        if detections and (self.frame_count % self.log_interval == 0):
            summary = ', '.join(
                f"{d['class']}({d['confidence']})" for d in detections
            )
            self.get_logger().info(f'[frame {self.frame_count}] Detections: {summary}')

    # ────────────────────────────────────────────────────────────────────
    #  CLEANUP
    # ────────────────────────────────────────────────────────────────────
    def destroy_node(self):
        if self.show_gui:
            cv2.destroyAllWindows()
        super().destroy_node()


# ════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ════════════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = GoalDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Goal Detector...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
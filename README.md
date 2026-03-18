# Description

Added goal_detector.py – a ROS2 node that uses a custom-trained YOLOv8 model
to detect goal markers (stop_sign, slow_sign, fast_sign, oranges, trees, cars)
from the robot's RGBD camera feed in Gazebo simulation.

What it does:
- Subscribes to /atlas/rgbd_camera/image for live camera frames
- Runs YOLOv8 inference with configurable confidence threshold (default 0.60)
- Publishes structured JSON detections to /goal_detections topic
  (class name, confidence, bounding box, centre point, area ratio)
- Publishes annotated debug image to /yolo/annotated_image for RViz
- Optional OpenCV GUI window for real-time visual feedback
- All key parameters (model path, confidence, camera topic) are
  configurable via ROS2 parameters at launch

This node serves as the detection backbone for downstream tasks
(Task 5: traffic sign response, Task 8: object counting).

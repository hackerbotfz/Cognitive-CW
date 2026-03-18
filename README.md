# Task 4 

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

# Task 5

Added traffic_sign_navigator.py – a ROS2 node that implements traffic sign
recognition and navigation speed response in simulation.

Architecture:
- Acts as a teleop interceptor between /teleop_cmd_vel and /atlas/cmd_vel
- Subscribes to /goal_detections from goal_detector.py (Task 4)
- Scales robot velocity based on closest detected traffic sign:
    slow_sign → 0.3× speed | fast_sign → 1.8× speed | stop_sign → full stop for 3s
- Only reacts when sign is close enough (area_ratio threshold)
- Publishes current traffic state to /traffic_state for debugging
- All parameters (area_threshold, stop_duration, topics) configurable via ROS2 params

Depends on: goal_detector.py (Task 4)

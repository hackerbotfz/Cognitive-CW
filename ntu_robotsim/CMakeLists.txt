cmake_minimum_required(VERSION 3.5)

project(ntu_robotsim)

find_package(ament_cmake REQUIRED)

# ROS2 build
install(DIRECTORY
  config
  models
  worlds
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/optical_frame.py
  scripts/goal_detector.py
  scripts/traffic_sign_navigator.py
  scripts/check_landmark_db.py
  scripts/yolo_detector.py
  scripts/landmark_database_node.py
  scripts/waypoints.py
  DESTINATION lib/${PROJECT_NAME}
)

# setup the environmental hooks for the Gazebo resource path
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gz_sim_resource_path.dsv.in")

ament_package()

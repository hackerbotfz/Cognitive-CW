cmake_minimum_required(VERSION 3.5)

project(ntu_robotsim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install the ntu_robotsim Python module (landmark_database, etc.)
ament_python_install_package(ntu_robotsim)

# ROS2 build – install shared resources
install(DIRECTORY
  config
  models
  worlds
  launch
  maps
  DESTINATION share/${PROJECT_NAME}
)

# Install Python executable scripts
install(PROGRAMS
  src/yolo_detector.py
  src/landmark_database_node.py
  src/check_landmark_db.py
  DESTINATION lib/${PROJECT_NAME}
)

# setup the environmental hooks for the Gazebo resource path
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gz_sim_resource_path.dsv.in")

ament_package()

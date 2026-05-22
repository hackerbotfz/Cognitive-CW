cmake_minimum_required(VERSION 3.5)

project(ntu_robotsim)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# ROS2 build
install(DIRECTORY
  config
  worlds
  launch
  urdf
  map
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# Install shell scripts
install(PROGRAMS
  src/yolo_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

# setup the environmental hooks for the Gazebo resource path
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gz_sim_resource_path.dsv.in")

ament_package()

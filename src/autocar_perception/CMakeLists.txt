cmake_minimum_required(VERSION 3.8)
project(autocar_perception)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  nodes/camera_preprocessor.py
  nodes/lidar_obstacle_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

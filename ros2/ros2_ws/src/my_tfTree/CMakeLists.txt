cmake_minimum_required(VERSION 3.8)
project(my_tfTree)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/broadcaster_node.py
  scripts/listener_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
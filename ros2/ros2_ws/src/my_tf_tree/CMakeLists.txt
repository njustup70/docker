cmake_minimum_required(VERSION 3.8)
project(my_tf_tree)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  launch/broadcaster_node.py
  launch/listener_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
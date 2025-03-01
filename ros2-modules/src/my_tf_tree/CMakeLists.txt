cmake_minimum_required(VERSION 3.8)
project(my_tf_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclpy REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)

install(PROGRAMS
  launch/depth_camera_node.launch.py
  launch/radar_node.launch.py
  launch/robot.launch.py
  launch/nagisa_tf.launch.py
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


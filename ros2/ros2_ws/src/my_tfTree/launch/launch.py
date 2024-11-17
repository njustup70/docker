from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='my_tfTree', executable='radar_node', name='radar_node'),
        Node(package='my_tfTree', executable='depth_camera_node', name='depth_camera_node'),
    ])


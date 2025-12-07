from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    ld=LaunchDescription()
    pub=Node(
        package='image_latency_test',
        executable='image_publisher_node',
        name='image_publisher_node',
        output='screen',
        parameters=[
            {"width": 1920},
            {"height": 1080},
            {"publish_rate": 30.0},
        ]
    )
    sub=Node(
        package='image_latency_test',
        executable='image_subscriber_node',
        name='image_subscriber_node',
        output='screen',
        emulate_tty=True,
    )
    intra=Node(
        package='image_latency_test',
        executable='intra_process_image_latency',
        name='intra_process_image_latency',
        output='screen',
    )
    ld.add_action(pub)
    ld.add_action(sub)
    # ld.add_action(intra)
    return ld

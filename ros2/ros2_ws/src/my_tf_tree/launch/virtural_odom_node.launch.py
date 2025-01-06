import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Declare the path to the Python node script
    script_path = os.path.join(
        os.getenv('PWD'),
        'ros2\ros2_ws\install\my_tf_tree\lib\my_tf_tree',
        'virtural_odom_node.py'
    )

    # Create the node action to run the Python node
    virtural_odometry_node = Node(
        package='my_tf_tree',
        executable='virtural_odom_node',
        arguments=[script_path],
        name='virtural_odom',
        output='screen'
    )

    # Add the node to the launch description
    ld.add_action(virtural_odometry_node)

    return ld
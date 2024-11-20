import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    urdf_path = os.path.join(
        get_package_share_directory('my_tf_tree'),
        'models',
        ''
    )

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
        doc = xacro.parse(robot_desc)
        xacro.process_doc(doc)
        robot_description = doc.toxml()

    Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
        ),
    Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),
        # 雷达和深度相机节点
    Node(
            package='my_tfTree',
            executable='radar_node', 
            name='radar_node'),
    Node(
            package='my_tfTree', 
            executable='depth_camera_node', 
            name='depth_camera_node'),

    ld = LaunchDescription()

    ld.add_action()
    ld.add_action()

    return ld
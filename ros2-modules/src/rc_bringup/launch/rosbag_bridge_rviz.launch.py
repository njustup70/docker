'''
启动rosbag 记录数据 
rosbridge 桥接ros1 ros2话题
启动foxglove用来可视化
'''
import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node
def generate_launch_description():
    ld=LaunchDescription()
    foxglove_node=ComposableNode(
        package='foxglove_bridge',
        plugin='foxglove_bridge::FoxgloveBridge',
        name='foxglove_bridge_node',
        parameters=[ {'send_buffer_limit': 1000000000}],
        extra_arguments=[{'use_intra_process_comms': True},
                    {'use_multi_threaded_executor': True}]
        )
    ros_bridge_exe=ExecuteProcess(
        cmd=["bash","-c","~/docker/ros2-modules/packages/ros-bridge/ros_bridge_run.sh"],
        output='screen',
    )
    ld.add_action(ros_bridge_exe)
    return ld

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
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node
def generate_launch_description():
    local_path=os.path.join(get_package_share_directory('rc_bringup'))
    ld=LaunchDescription()
    ld.add_action(DeclareLaunchArgument('use_rosbag_record', default_value='true', description='Record rosbag if use is True'))
    # ld.add_action(DeclareLaunchArgument('ros', default_value='5', description='Max number of rosbag files'))
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
    ros_bag_bash_path=os.path.join(local_path,'scripts/rosbag_record.py')
    ros_bag_exe=ExecuteProcess(
        cmd=["bash","-c","python3 {}".format(ros_bag_bash_path)],
        output='screen',
    )
    compose_container=ComposableNodeContainer(
        namespace='',
        name='start_container',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[foxglove_node],
        output='screen',
        emulate_tty=True,
    )
    
    # rosbag_record=LaunchConfiguration('rosbag_record')
    # if_condition = IfCondition(LaunchConfiguration('rosbag_record'))
    # print("rosbag_record:",if_condition )
    # if(rosbag_record=='True'):
    #     print("rosbag record")
    #     ld.add_action(rosbag_record)
    ld.add_action(ros_bridge_exe)
    ld.add_action(ros_bag_exe)
    ld.add_action(compose_container)
    return ld

'''
启动驱动节点和工具节点
'''
import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess,IncludeLaunchDescription,DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node 
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable

def generate_launch_description():
    ld=LaunchDescription()
    ld.add_action(DeclareLaunchArgument('use_rosbag_record', default_value='true', description='Record rosbag if use is True'))
    ld.add_action(DeclareLaunchArgument('use_tf_publish',default_value='true',description='Publish tf tree if use is True'))
    ld.add_action(DeclareLaunchArgument('use_mid360',default_value='true',description='Start mid360 node if use is True'))
    ld.add_action(DeclareLaunchArgument('use_extern_imu',default_value='true',description='Start extern imu node if use is True'))
    ld.add_action(DeclareLaunchArgument('use_imu_transform',default_value='true',description='Start imu transform node if use is True'))
    ld.add_action(DeclareLaunchArgument('use_realsense',default_value='true',description='Start realsense node if use is True'))
    ld.add_action(DeclareLaunchArgument('use_joy',default_value='true',description='是否启动手柄控制'))
    ld.add_action(DeclareLaunchArgument('use_ms_200',default_value='true',description='是否启动2d雷达'))
    get_package_share_directory('my_driver')
    get_package_share_directory('rc_bringup')
    #启动mid360
    mid360_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('my_driver'),'launch','mid360_bringup.launch.py')
        ),
        launch_arguments={
            'use_rviz': 'false',  #不启动rviz
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_mid360'))
    )
    #启动外接imu
    extern_imu_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('my_driver'),'launch','wheel_imu.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_extern_imu')))
    #启动imu转换
    imu_transform_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('perception'),'launch','imu_transform.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_imu_transform'))
    )
    #启动utils
    utils_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rc_bringup'),'launch','utils_bringup.launch.py')
        ),
        
    )
    #启动realsense
    realsense_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('my_driver'),'launch','realsense_bringup.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_realsense'))
    )
    #启动手柄
    joy_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('my_driver'),'launch','joy.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_joy'))
    )
    #启动下位机通信
    communicate_node=Node(
        package='python_pkg',
        executable='communicate',
        name='communicate',
        output='screen',
        parameters=[
            {'serial_port': '/dev/serial_x64',}
        ]
    )
    #启动ms200
    ms200_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('my_driver'),'launch','ms200_scan.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_ms_200'))
    )
    ld.add_action(mid360_launch)
    ld.add_action(extern_imu_launch)
    ld.add_action(imu_transform_launch)
    ld.add_action(realsense_launch)
    ld.add_action(utils_launch)
    ld.add_action(joy_launch)
    ld.add_action(communicate_node)
    ld.add_action(ms200_launch)
    return ld
     
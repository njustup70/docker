import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明参数
    imu_type_arg = DeclareLaunchArgument(
        'imu_type',
        default_value='a9',
        description='imu_type [a9, b9, b6]'
    )

    # 创建节点
    imu_node = Node(
        package='robot_imu',
        executable='hfi_a9_imu', 
        name='imu',
        output='screen',
        parameters=[
            {'port': '/dev/ttyUSB0'},
            {'gra_normalization': LaunchConfiguration('imu_type') == 'a9'}
        ]
    )

    return LaunchDescription([
        imu_type_arg,
        imu_node
    ])

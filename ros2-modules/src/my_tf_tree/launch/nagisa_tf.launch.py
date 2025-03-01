import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable

def generate_launch_description():
    # 定义参数
    project_arg = DeclareLaunchArgument(
        'project',
        default_value='my_tf_tree',
        description='Project name'
    )

    # 获取 Xacro 文件路径
    xacro_file = PathJoinSubstitution([
        '/home/Elaina/docker/ros2-modules/src/my_tf_tree',  # 直接指定项目路径
        'urdf', 'fishbot_base.urdf.xacro'
    ])

    # 解析 Xacro 文件并生成 URDF
    robot_description = Command([
        FindExecutable(name='xacro'),  # 查找 xacro 可执行文件
        ' ',  # 确保命令和路径之间有空格
        xacro_file  # 传入 xacro 文件路径
    ])

    # 机器人状态发布节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    return LaunchDescription([
        project_arg,
        robot_state_publisher_node
    ])

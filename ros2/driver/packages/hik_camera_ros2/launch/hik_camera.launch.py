import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 声明 use_rviz 参数，默认值为 'true'
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start Rviz2'
    )

    # 获取参数值
    use_rviz = LaunchConfiguration('use_rviz')

    # 获取 rviz 配置文件路径
    rviz_config_file = os.path.join(
        get_package_share_directory('hik_camera_ros2'),
        'config',
        'default.rviz'
    )

    # 定义相机节点
    camera_node = Node(
        package='hik_camera_ros2',
        executable='hik_camera_node',
        name='hik_camera_node',
        output='screen',
        parameters=[
            {'serial_number': 'DA4976553'}, 
            {'topic_name': '/hik_camera/image_raw'}, 
            {'exposure_time': 28000.0},
            {'gain': 10.0},
            {'frame_rate': 200.0},
            {'pixel_format': 'BayerGB8'},
        ],
    )

    # 定义 rviz2 节点 (带条件判断)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        # 【关键】只有当 use_rviz 为 true 时才启动
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        use_rviz_arg,  # 必须添加参数声明
        camera_node,
        rviz_node
    ])
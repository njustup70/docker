import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 定义一个函数，用于生成启动描述
def generate_launch_description():
    # 获取URDF文件的路径
    urdf_path = os.path.join(
        get_package_share_directory('wpr_simulation2'),
        'models',
        'wpb_home.model'
    )

    # 处理URDF文件
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
        doc = xacro.parse(robot_desc)
        xacro.process_doc(doc)
        robot_description = doc.toxml()
    
    # 定义一个节点，用于发布机器人状态
    robot_state_publisher_cmd = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description
            }],
        )

    # 定义启动配置变量，用于模拟
    pose_x = LaunchConfiguration('pose_x', default='0.0')
    pose_y = LaunchConfiguration('pose_y', default='0.0')
    pose_theta = LaunchConfiguration('pose_theta', default='0.0')

    # 声明启动参数
    declare_x_position_cmd = DeclareLaunchArgument(
    'x_pose', default_value='0.0',
    description='Specify namespace of the robot')

    declare_y_position_cmd = DeclareLaunchArgument(
    'y_pose', default_value='0.0',
    description='Specify namespace of the robot')

    # 定义一个节点，用于在Gazebo中生成实体
    start_gazebo_ros_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', "wpb_home",
            '-x', pose_x,
            '-y', pose_y,
            '-Y', pose_theta
        ],
        output='screen',
    )

    # 创建一个启动描述对象
    ld = LaunchDescription()

    # 声明启动选项
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)

    # 添加任何条件动作
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(start_gazebo_ros_spawner_cmd)

    # 返回启动描述
    return ld
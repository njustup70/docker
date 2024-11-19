from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    robot_description_config = LaunchConfiguration('robot_description', default='/home/yc-dlan/docker/ros2/ros2_ws/src/my_tfTree/myurdf_fix/urdf/myurdf_fix.urdf')
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_description',
            default_value=robot_description_config,
            description='/home/yc-dlan/docker/ros2/ros2_ws/src/my_tfTree/myurdf_fix/urdf/myurdf_fix.urdf'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_config}],
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
    ])
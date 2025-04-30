from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, ComposableNodeContainer, Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    ld=LaunchDescription()
    ld.add_action(DeclareLaunchArgument('joy_topic', default_value='/joy'))
    ld.add_action(DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'))
    ld.add_action(DeclareLaunchArgument('linear_scale', default_value='1.0'))
    ld.add_action(DeclareLaunchArgument('angular_scale', default_value='3.14'))
    joy_driver = Node(
        package='joy',
        executable='joy_node',
        name='joy_node')
    joy_teleop = Node(
        package='python_pkg',
        executable='joy',
        name='joy_teleop_node',
        parameters=[{
            'joy_topic': LaunchConfiguration('joy_topic'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'linear_scale': LaunchConfiguration('linear_scale'),
            'angular_scale': LaunchConfiguration('angular_scale')
        }]
    )
    ld.add_action(joy_driver)
    ld.add_action(joy_teleop)
    return ld
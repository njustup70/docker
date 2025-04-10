import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess,IncludeLaunchDescription,DeclareLaunchArgument
import launch

def generate_launch_description():
    # 获取当前文件的目录
    package_dir=get_package_share_directory('my_driver')
    config_dir=os.path.join(package_dir,'config')
    ld=LaunchDescription()
    ld.add_action(DeclareLaunchArgument("config_dir", default_value=config_dir+"/usb_cam_d435.yaml"))
    # ld.add_action(DeclareLaunchArgument("video_device", default_value='/dev/video0'))
    camera_node=Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam_node',
        output='screen',
        parameters=[LaunchConfiguration('config_dir')],
        remappings=[
            ('image_raw','/camera/image_raw'),
            ('camera_info','/camera/camera_info')
        ]
    )
    ld.add_action(camera_node)
    return ld
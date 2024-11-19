# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import IncludeLaunchDescription
# from launch.substitutions import LaunchConfiguration
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from ament_index_python.packages import get_package_share_directory
# def generate_launch_description():
#     # 获取 OrbbecSDK_ROS2 包的路径
#     orbbec_ws_package_prefix = get_package_share_directory('OrbbecSDK_ROS2')
#     orbbec_pkg_package_prefix = orbbec_ws_package_prefix + '/orbbec_camera'
#     # 创建 LaunchDescription 对象
#     launch_description = LaunchDescription()
#     my_package_prefix = get_package_share_directory('nagisa_orbbec')

#     yaml_path=LaunchConfiguration('config_file',default=my_package_prefix+'/config/orbbec.yaml') #深度相机的配置文件


#     # 将 OrbbecSDK_ROS2 中的 sdk_launch.py 引入
#     include_sdk_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([orbbec_pkg_package_prefix + '/launch/astra.launch.py']),
#         launch_arguments={'config_file':yaml_path
#                           }.items()
#     )

#     # 添加该操作到 launch 文件中
#     launch_description.add_action(include_sdk_launch)

#     return launch_description
# import os
# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import IncludeLaunchDescription
# from launch.substitutions import LaunchConfiguration
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from ament_index_python.packages import get_package_share_directory

# def generate_launch_description():
#     # 获取 OrbbecSDK_ROS2 包的路径
#     orbbec_ws_package_prefix = os.path.join(os.environ['HOME'], 'packages/orbbec_ws/src/OrbbecSDK_ROS2')
#     orbbec_pkg_package_prefix = os.path.join(orbbec_ws_package_prefix, 'orbbec_camera')
    
#     # 创建 LaunchDescription 对象
#     launch_description = LaunchDescription()
#     my_package_prefix = get_package_share_directory('nagisa_orbbec')

#     # 使用 LaunchConfiguration 获取 YAML 文件的路径
#     yaml_path = LaunchConfiguration('config_file', default=os.path.join(my_package_prefix, 'config', 'orbbec.yaml'))

#     # 将 OrbbecSDK_ROS2 中的 sdk_launch.py 引入
#     include_sdk_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(os.path.join(orbbec_pkg_package_prefix, 'launch', 'astra.launch.py')),
#         launch_arguments={'config_file': yaml_path}.items()
#     )

#     # 添加该操作到 launch 文件中
#     launch_description.add_action(include_sdk_launch)

#     return launch_description
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Include launch files
    package_dir = get_package_share_directory('orbbec_camera')
    launch_file_dir = os.path.join(package_dir, 'launch')
    launch1_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'gemini2L.launch.py')
        ),
        launch_arguments={
            'camera_name': 'camera_01',
            'usb_port': '6-2.4.4.2',  # replace your usb port here
            'device_num': '2'
        }.items()
    )

    launch2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'gemini2L.launch.py')
        ),
        launch_arguments={
            'camera_name': 'camera_02',
            'usb_port': '6-2.4.1',  # replace your usb port here
            'device_num': '2'
        }.items()
    )

    # If you need more cameras, just add more launch_include here, and change the usb_port and device_num

    # Launch description
    ld = LaunchDescription([
        GroupAction([launch1_include]),
        GroupAction([launch2_include]),
    ])

    return ld

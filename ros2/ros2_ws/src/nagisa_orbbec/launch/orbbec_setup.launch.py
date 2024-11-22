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
# from launch.actions import IncludeLaunchDescription, LogInfo
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

#     # 在终端打印 YAML 文件路径
#     log_yaml_path = LogInfo(msg=['Loading YAML file: ', yaml_path])

#     # 将 OrbbecSDK_ROS2 中的 sdk_launch.py 引入
#     include_sdk_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(os.path.join(orbbec_pkg_package_prefix, 'launch', 'gemini_intra_process_demo_launch.py')),

# #         launch_arguments={'depth_width':=640,'depth_height':=480,'depth_fps':=15,'depth_format':='Y16'
# # }.items()

      
#         launch_arguments={'config_file': yaml_path}.items()
#     )

#     # 添加操作到 Launch 文件中
#     launch_description.add_action(log_yaml_path)
#     launch_description.add_action(include_sdk_launch)

#     return launch_description




# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory
# import os 


# def generate_launch_description():
#     # Include launch files
#     package_dir = get_package_share_directory('orbbec_camera')
#     launch_file_dir = os.path.join(package_dir, 'launch')
#     launch1_include = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(launch_file_dir, 'gemini2L.launch.py')
#         ),
#         launch_arguments={
#             'camera_name': 'camera_01',
#             'usb_port': '6-2.4.4.2',  # replace your usb port here
#             'device_num': '2'
#         }.items()
#     )

#     launch2_include = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(launch_file_dir, 'gemini2L.launch.py')
#         ),
#         launch_arguments={
#           'depth_format', default_value='Y16'
#         }.items()
#     )

#     # If you need more cameras, just add more launch_include here, and change the usb_port and device_num

#     # Launch description
#     ld = LaunchDescription([
#         GroupAction([launch1_include]),
#         GroupAction([launch2_include]),
#     ])

#     return ld



import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, LogInfo

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def load_yaml(file_path):
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)


def merge_params(default_params, yaml_params):
    for key, value in yaml_params.items():
        if key in default_params:
            default_params[key] = value
    return default_params

def convert_value(value):
    if isinstance(value, str):
        try:
            return int(value)
        except ValueError:
            pass
        try:
            return float(value)
        except ValueError:
            pass
        if value.lower() == 'true':
            return True
        elif value.lower() == 'false':
            return False
    return value


def load_parameters(context, args):
    default_params = {arg.name: LaunchConfiguration(arg.name).perform(context) for arg in args}
    config_file_path = LaunchConfiguration('config_file_path').perform(context)
    if config_file_path:
        yaml_params = load_yaml(config_file_path)
        default_params = merge_params(default_params, yaml_params)
    skip_convert = {'config_file_path', 'usb_port', 'serial_number'}
    return {
        key: (value if key in skip_convert else convert_value(value))
        for key, value in default_params.items()
    }

    
def get_params(context, args):
        return [load_parameters(context, args)]

def create_node_action(context, args):
    params = get_params(context, args)
    ros_distro = os.environ.get("ROS_DISTRO", "humble")
    if ros_distro == "humble":
            return [
                GroupAction([
                    PushRosNamespace(LaunchConfiguration("camera_name")),
                    ComposableNodeContainer(
                        name="camera_container",
                        namespace="",
                        package="rclcpp_components",
                        executable="component_container",
                        composable_node_descriptions=[
                            ComposableNode(
                                package="orbbec_camera",
                                plugin="orbbec_camera::OBCameraNodeDriver",
                                name=LaunchConfiguration("camera_name"),
                                parameters=params,
                                extra_arguments=[{'use_intra_process_comms': True}],
                            ),
                            ComposableNode(
                                package="orbbec_camera",
                                plugin="orbbec_camera::FrameLatencyNode",
                                name="frame_latency",
                                parameters=[
                                    {"topic_name": LaunchConfiguration("topic_name")},
                                    {"topic_type": LaunchConfiguration("topic_type")},
                                ],

                                extra_arguments=[{'use_intra_process_comms': True}],
                            ),
                        ],
                        output="screen",
                        #prefix=['xterm -e gdb -ex run --args'],  
                    )
                ])
            ]

    return LaunchDescription(
        args + [
            OpaqueFunction(function=lambda context: create_node_action(context, args))
        ]
    )
def generate_launch_description():
    # 获取 OrbbecSDK_ROS2 包的路径
    orbbec_ws_package_prefix = os.path.join(os.environ['HOME'], 'packages/orbbec_ws/src/OrbbecSDK_ROS2')
    orbbec_pkg_package_prefix = os.path.join(orbbec_ws_package_prefix, 'orbbec_camera')
    
    # 创建 LaunchDescription 对象
    launch_description = LaunchDescription()
    my_package_prefix = get_package_share_directory('nagisa_orbbec')

    # 使用 LaunchConfiguration 获取 YAML 文件的路径
    yaml_path = LaunchConfiguration('config_file', default=os.path.join(my_package_prefix, 'config', 'orbbec.yaml'))

    # 在终端打印 YAML 文件路径
    log_yaml_path = LogInfo(msg=['Loading YAML file: ', yaml_path])

    # 将 OrbbecSDK_ROS2 中的 sdk_launch.py 引入
    launch_arguments = {
        'config_file': yaml_path
    }

    # 打印传递的参数
    log_launch_arguments = LogInfo(msg=['Launch arguments: ', str(launch_arguments)])

    include_sdk_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(orbbec_pkg_package_prefix, 'launch', 'gemini_intra_process_demo_launch.py')),
        launch_arguments=launch_arguments.items()
    )

    # 添加操作到 Launch 文件中
    launch_description.add_action(log_yaml_path)
    launch_description.add_action(log_launch_arguments)
    launch_description.add_action(include_sdk_launch)

    return launch_description

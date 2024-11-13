from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    my_package_prefix = get_package_share_directory('my_realsense')
    launchDescription = LaunchDescription()
    realsense_pkg_prefix = get_package_share_directory('realsense2_camera')
    root_path=LaunchConfiguration('camera_namespace',default='/')
    yaml_path=LaunchConfiguration('config_file',default=my_package_prefix+'/config/realsense.yaml')

    realsense_bringup=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([realsense_pkg_prefix + '/launch/rs_launch.py']),
        launch_arguments={'camera_namespace':root_path
        ,'config_file':yaml_path
                          }.items()
    )
    launchDescription.add_action(realsense_bringup)
    return launchDescription
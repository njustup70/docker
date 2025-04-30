import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
def generate_launch_description():
    my_path=get_package_share_directory('perception')
    param_file_path=os.path.join(my_path,'config','Initialization_result.txt')
    ld=LaunchDescription()
    ld.add_action(DeclareLaunchArgument("imu_topic",default_value="/livox/imu"))
    ld.add_action(DeclareLaunchArgument("use_grivaty2m",default_value="true"))
    ld.add_action(DeclareLaunchArgument("imu_transformed_topic",default_value="/imu_transformed"))
    ld.add_action(DeclareLaunchArgument("imu_frame",default_value="imu_link"))
    ld.add_action(DeclareLaunchArgument("lidar_frame",default_value="laser_link"))
    ld.add_action(DeclareLaunchArgument("Calibration_file",default_value=param_file_path))
    imu_transform_node=Node(
        package="python_pkg",
        executable="imu_transform",
        name="ImuTransform_node",
        parameters=[{"imu_topic":LaunchConfiguration("imu_topic")},
                    {"imu_transformed_topic":LaunchConfiguration("imu_transformed_topic")},
                    {"imu_frame":LaunchConfiguration("imu_frame")},
                    {"lidar_frame":LaunchConfiguration("lidar_frame")},
                    {"Calibration_file":LaunchConfiguration("Calibration_file")}
                    ],
        output="screen"
    )
    ld.add_action(imu_transform_node)
    return ld
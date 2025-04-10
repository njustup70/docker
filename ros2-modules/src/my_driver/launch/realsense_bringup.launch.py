from launch import LaunchDescription
from launch_ros.actions import Node 
from launch.actions import IncludeLaunchDescription,TimerAction,DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    
    #获得深度相机包的路径与自己参数文件的路径
    my_package_prefix = get_package_share_directory('my_driver')
    launchDescription = LaunchDescription()
    launchDescription.add_action(DeclareLaunchArgument('log_level', default_value='FATAL', description='Log level for realsense2_camera'))
    realsense_pkg_prefix = get_package_share_directory('realsense2_camera')
    
    root_path=LaunchConfiguration('camera_namespace',default='/') #深度相机的命名空间
    yaml_path=LaunchConfiguration('config_file',default=my_package_prefix+'/config/realsense.yaml') #深度相机的配置文件

    realsense_bringup=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([realsense_pkg_prefix + '/launch/rs_launch.py']),
        launch_arguments={'camera_namespace':root_path
        ,'config_file':yaml_path,'log_level':LaunchConfiguration('log_level')
                          }.items()
    )
    image_publish_node=Node(
        package='python_pkg',
        executable='image_bridge',
        name='image_bridge_node',
        output='screen',
        emulate_tty=True,

    )
    delayed_image_publish_node = TimerAction(
        period=2.0,  # 延迟 5 秒
        actions=[image_publish_node]
    )
    launchDescription.add_action(realsense_bringup)
    launchDescription.add_action(delayed_image_publish_node)
    return launchDescription
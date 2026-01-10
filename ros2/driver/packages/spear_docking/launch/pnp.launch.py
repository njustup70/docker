from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    image_topic_arg = DeclareLaunchArgument(
        "image_topic",
        default_value="/hik_camera/image_raw",
        description="Input image topic for spear_docking pnp_node.",
    )
    target_pose_topic_arg = DeclareLaunchArgument(
        "target_pose_topic",
        default_value="/target_pose",
        description="Output PoseStamped topic published by spear_docking pnp_node.",
    )

    pnp_node = Node(
        package="spear_docking",
        executable="pnp_node",
        name="vision_pnp_node",
        output="screen",
        remappings=[
            ("/hik_camera/image_raw", LaunchConfiguration("image_topic")),
            ("/target_pose", LaunchConfiguration("target_pose_topic")),
        ],
    )

    return LaunchDescription(
        [
            image_topic_arg,
            target_pose_topic_arg,
            pnp_node,
        ]
    )

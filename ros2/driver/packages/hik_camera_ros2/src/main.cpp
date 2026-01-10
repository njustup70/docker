#include "hik_camera_ros2/hik_camera_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HikCameraNode>());
    rclcpp::shutdown();
    return 0;
}

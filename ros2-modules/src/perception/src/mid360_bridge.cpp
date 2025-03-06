#include <fmt/format.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
using pointcloud2 = sensor_msgs::msg::PointCloud2;
namespace velodyne_ros {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    float time;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace velodyne_ros
namespace livox_ros {
struct Point {
    float x, y, z, intensity;
    uint8_t tag, line;
    double timestamp;
};
} // namespace livox_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(
    velodyne_ros::Point, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                             float, time, time)(uint16_t, ring, ring))
POINT_CLOUD_REGISTER_POINT_STRUCT(
    livox_ros::Point, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                          uint8_t, tag, tag)(uint8_t, line, line)(double, timestamp, timestamp))
class Mid360Bridge : public rclcpp::Node {
public:
    explicit Mid360Bridge(const rclcpp::NodeOptions& options)
        : Node("mid360_bridge", options) {
        RCLCPP_INFO(get_logger(), "Mid360Bridge has been started.");
        this->declare_parameter("sub_topic", "livox/lidar");
        this->declare_parameter("pub_topic", "livox/lidar/pointcloud2");
        pub_ =
            this->create_publisher<pointcloud2>(this->get_parameter("pub_topic").as_string(), 10);
        // sub_=this->create_subscription<p>(const std::string &topic_name, const rclcpp::QoS &qos,
        // CallbackT &&callback)
        sub_ = this->create_subscription<pointcloud2>(
            this->get_parameter("sub_topic").as_string(), 10,
            [this](std::unique_ptr<pointcloud2> msg_in) { callback(std::move(msg_in)); });
    }

private:
    std::shared_ptr<rclcpp::Subscription<pointcloud2>> sub_;
    std::shared_ptr<rclcpp::Publisher<pointcloud2>> pub_;
    void callback(std::unique_ptr<pointcloud2> msg_in) {
        auto cloud = std::make_unique<pcl::PointCloud<livox_ros::Point>>();
        pcl::fromROSMsg(*msg_in, *cloud);
        auto velodyne_cloud = Mid360PointToVelodyne(*cloud);
        auto msg_out        = std::make_unique<pointcloud2>();
        pcl::toROSMsg(*velodyne_cloud, *msg_out);
        msg_out->header.frame_id = msg_in->header.frame_id;
        msg_out->header.stamp    = msg_in->header.stamp;
        pub_->publish(std::move(msg_out));
    }
    std::unique_ptr<pcl::PointCloud<velodyne_ros::Point>>
        Mid360PointToVelodyne(pcl::PointCloud<livox_ros::Point>& cloud) {
        auto velodyne_cloud      = std::make_unique<pcl::PointCloud<velodyne_ros::Point>>();
        velodyne_cloud->header   = cloud.header;
        velodyne_cloud->height   = cloud.height;
        velodyne_cloud->width    = cloud.width;
        velodyne_cloud->is_dense = cloud.is_dense;
        velodyne_cloud->points.resize(cloud.points.size());
        for (size_t i = 0; i < cloud.points.size(); ++i) {
            velodyne_cloud->points[i].x         = cloud.points[i].x;
            velodyne_cloud->points[i].y         = cloud.points[i].y;
            velodyne_cloud->points[i].z         = cloud.points[i].z;
            velodyne_cloud->points[i].intensity = cloud.points[i].intensity;
            velodyne_cloud->points[i].time      = static_cast<float>(cloud.points[i].timestamp);
            velodyne_cloud->points[i].ring      = static_cast<uint16_t>(cloud.points[i].line);
        }
        return velodyne_cloud;
    }
};

RCLCPP_COMPONENTS_REGISTER_NODE(Mid360Bridge)
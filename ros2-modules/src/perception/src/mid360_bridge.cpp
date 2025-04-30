#include <fmt/format.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
using pointcloud2 = sensor_msgs::msg::PointCloud2;
namespace velodyne_ros {
struct EIGEN_ALIGN16 Point {
    float x, y, z;
    float intensity;
    float time;
    uint16_t ring;
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
        this->declare_parameter("pub_debug", true);
        _debug = this->get_parameter("pub_debug").as_bool();
        rclcpp::QoS qos(1);
        qos.best_effort();         // 不保证可靠性，但最快
        qos.durability_volatile(); // 不保留历史数据
        pub_ =
            this->create_publisher<pointcloud2>(this->get_parameter("pub_topic").as_string(), qos);
        // sub_=this->create_subscription<p>(const std::string &topic_name, const rclcpp::QoS &qos,
        // CallbackT &&callback)
        sub_ = this->create_subscription<pointcloud2>(
            this->get_parameter("sub_topic").as_string(), qos,
            [this](std::unique_ptr<pointcloud2> msg_in) { callback(std::move(msg_in)); });
    }

private:
    bool _debug = false;
    std::shared_ptr<rclcpp::Subscription<pointcloud2>> sub_;
    std::shared_ptr<rclcpp::Publisher<pointcloud2>> pub_;
    void callback(std::unique_ptr<pointcloud2> msg_in) {
        auto start_time = std::chrono::steady_clock::now();
        auto cloud      = std::make_unique<pcl::PointCloud<livox_ros::Point>>();
        pcl::fromROSMsg(*msg_in, *cloud);
        double base_time    = msg_in->header.stamp.nanosec + msg_in->header.stamp.sec * 1e9;
        auto velodyne_cloud = Mid360PointToVelodyne(*cloud, base_time);
        auto msg_out        = std::make_unique<pointcloud2>();
        pcl::toROSMsg(*velodyne_cloud, *msg_out);
        msg_out->header.frame_id = msg_in->header.frame_id;
        msg_out->header.stamp    = msg_in->header.stamp;
        pub_->publish(std::move(msg_out));
        auto end_time = std::chrono::steady_clock::now();
        auto duration =
            std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        if (_debug) {
            fmt::print("duration is {} ms \n", duration.count());
        }
    }
    std::unique_ptr<pcl::PointCloud<velodyne_ros::Point>>
        Mid360PointToVelodyne(pcl::PointCloud<livox_ros::Point>& cloud, double base_time) {
        auto velodyne_cloud      = std::make_unique<pcl::PointCloud<velodyne_ros::Point>>();
        velodyne_cloud->header   = cloud.header;
        velodyne_cloud->height   = cloud.height;
        velodyne_cloud->width    = cloud.width;
        velodyne_cloud->is_dense = cloud.is_dense;
        velodyne_cloud->points.resize(cloud.points.size());
        // 先算出点云头的时间戳

        // mid360的time是基于1970绝对时间戳,单位为ns，而velodyne的time是相对时间戳单位为s
        // mide360的时间戳是double类型，而velodyne的时间戳是float类型
        for (size_t i = 0; i < cloud.points.size(); ++i) {
            velodyne_cloud->points[i].x         = cloud.points[i].x;
            velodyne_cloud->points[i].y         = cloud.points[i].y;
            velodyne_cloud->points[i].z         = cloud.points[i].z;
            velodyne_cloud->points[i].intensity = cloud.points[i].intensity;
            double time                         = (cloud.points[i].timestamp - base_time) * 1e-9;
            velodyne_cloud->points[i].time      = static_cast<float>(time);
            velodyne_cloud->points[i].ring      = static_cast<uint16_t>(cloud.points[i].line);
            // fmt::print("time_base is {} \n", base_time);
            // fmt::print("time is {} \n", cloud.points[i].timestamp);
            // fmt::print("delta time is {} \n", time);
        }
        if (_debug) {
            fmt::print("timebase is {} \n", base_time);
            fmt::print("time raw is {} \n", cloud.points[0].timestamp);
            fmt::print("time out is {} \n", velodyne_cloud->points[500].time);
        }
        return velodyne_cloud;
    }
};

RCLCPP_COMPONENTS_REGISTER_NODE(Mid360Bridge)
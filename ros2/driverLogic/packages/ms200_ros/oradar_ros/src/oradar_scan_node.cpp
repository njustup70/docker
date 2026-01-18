// License: See LICENSE file in root directory.
// Copyright(c) 2022 Oradar Corporation. All Rights Reserved.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>
#include <iostream>
#include <string>
#include <signal.h>
#include <cmath>
#include "src/ord_lidar_driver.h"
#include <sys/time.h>

using namespace std;
using namespace ordlidar;

#define Degree2Rad(X) ((X)*M_PI / 180.)

class OradarLidarNode : public rclcpp::Node
{
public:
    OradarLidarNode() : Node("oradar_ros")
    {
        // 声明参数
        declare_parameters();
        // 获取参数值
        get_parameters();
        // 创建发布器，使用传感器数据的QoS配置
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            scan_topic_, 
            rclcpp::SensorDataQoS()  // 使用传感器数据的QoS配置
        );
        
        // 初始化雷达驱动
        init_driver();
        
        // 启动扫描循环
        scan_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&OradarLidarNode::scan_loop, this)
        );
    }

private:
    // 声明ROS2参数
    void declare_parameters()
    {
        this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
        this->declare_parameter<int>("baudrate", 230400);
        this->declare_parameter<double>("angle_max", 180.0);
        this->declare_parameter<double>("angle_min", -180.0);
        this->declare_parameter<double>("range_max", 20.0);
        this->declare_parameter<double>("range_min", 0.05);
        this->declare_parameter<bool>("clockwise", false);
        this->declare_parameter<int>("motor_speed", 10);
        this->declare_parameter<std::string>("device_model", "ms200");
        this->declare_parameter<std::string>("frame_id", "laser_frame");
        this->declare_parameter<std::string>("scan_topic", "scan");
        this->declare_parameter<double>("time_adjustment", 10.0);
    }

    // 获取ROS2参数值
    void get_parameters()
    {
        this->get_parameter("port_name", port_);
        this->get_parameter("baudrate", baudrate_);
        this->get_parameter("angle_max", angle_max_);
        this->get_parameter("angle_min", angle_min_);
        this->get_parameter("range_max", range_max_);
        this->get_parameter("range_min", range_min_);
        this->get_parameter("clockwise", clockwise_);
        this->get_parameter("motor_speed", motor_speed_);
        this->get_parameter("device_model", device_model_);
        this->get_parameter("frame_id", frame_id_);
        this->get_parameter("scan_topic", scan_topic_);
        this->get_parameter("time_adjustment", time_adjustment_);
        
        // 打印参数信息
        RCLCPP_INFO(this->get_logger(), "port_name: %s", port_.c_str());
        RCLCPP_INFO(this->get_logger(), "baudrate: %d", baudrate_);
        RCLCPP_INFO(this->get_logger(), "angle_min: %f, angle_max: %f", angle_min_, angle_max_);
        RCLCPP_INFO(this->get_logger(), "range_min: %f, range_max: %f", range_min_, range_max_);
        RCLCPP_INFO(this->get_logger(), "clockwise: %s", clockwise_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "motor_speed: %d", motor_speed_);
        RCLCPP_INFO(this->get_logger(), "device_model: %s", device_model_.c_str());
        RCLCPP_INFO(this->get_logger(), "frame_id: %s", frame_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "scan_topic: %s", scan_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "time_adjustment: %f", time_adjustment_);
    }

    // 初始化雷达驱动
    void init_driver()
    {
        // 设置雷达类型和模型
        int model = ORADAR_MS200;  // 默认为MS200，可根据需要修改
        
        // 创建雷达驱动实例
        device_ = std::make_unique<OrdlidarDriver>(ORADAR_TYPE_SERIAL, model);
        
        // 设置串口参数
        device_->SetSerialPort(port_, baudrate_);
        
        // 尝试连接雷达
        connect_lidar();
    }

    // 连接雷达设备
    void connect_lidar()
    {
        while (rclcpp::ok() && !device_->Connect())
        {
            RCLCPP_WARN(this->get_logger(), "Failed to connect to lidar, retrying...");
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
        
        if (rclcpp::ok())
        {
            RCLCPP_INFO(this->get_logger(), "Successfully connected to lidar");
            
            // 设置雷达转速
            double min_thr = (double)motor_speed_ - ((double)motor_speed_ * 0.1);
            double max_thr = (double)motor_speed_ + ((double)motor_speed_ * 0.1);
            double cur_speed = device_->GetRotationSpeed();
            
            if (cur_speed < min_thr || cur_speed > max_thr)
            {
                device_->SetRotationSpeed(motor_speed_);
                RCLCPP_INFO(this->get_logger(), "Set lidar rotation speed to %dHz", motor_speed_);
            }
        }
    }

    // 扫描循环，定期获取雷达数据并发布
    void scan_loop()
    {
        // 检查连接状态
        if (!device_->isConnected())
        {
            RCLCPP_WARN(this->get_logger(), "Lidar disconnected, trying to reconnect...");
            connect_lidar();
            return;
        }
        
        // 记录扫描开始时间
        rclcpp::Time start_scan_time = this->now();
        rclcpp::Time adjusted_start = start_scan_time - rclcpp::Duration::from_seconds(time_adjustment_);
        
        // 获取雷达数据
        full_scan_data_st scan_data;
        bool ret = device_->GrabFullScanBlocking(scan_data, 1000);
        
        // 记录扫描结束时间并计算持续时间
        rclcpp::Time end_scan_time = this->now();
        double scan_duration = (end_scan_time.seconds() - start_scan_time.seconds());
        
        // 发布激光扫描消息
        if (ret)
        {
            publish_scan(&scan_data, adjusted_start, scan_duration);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Failed to grab full scan data");
        }
    }

    // 发布激光扫描消息
    void publish_scan(full_scan_data_st *scan_frame, rclcpp::Time start, double scan_time)
    {
        auto scanMsg = std::make_unique<sensor_msgs::msg::LaserScan>();
        int point_nums = scan_frame->vailtidy_point_num;

        scanMsg->header.stamp = start;
        scanMsg->header.frame_id = frame_id_;
        scanMsg->angle_min = Degree2Rad(scan_frame->data[0].angle);
        scanMsg->angle_max = Degree2Rad(scan_frame->data[point_nums - 1].angle);
        double diff = scan_frame->data[point_nums - 1].angle - scan_frame->data[0].angle;
        scanMsg->angle_increment = Degree2Rad(diff/point_nums);
        scanMsg->scan_time = scan_time;
        scanMsg->time_increment = scan_time / point_nums;
        scanMsg->range_min = range_min_;
        scanMsg->range_max = range_max_;

        scanMsg->ranges.assign(point_nums, std::numeric_limits<float>::quiet_NaN());
        scanMsg->intensities.assign(point_nums, std::numeric_limits<float>::quiet_NaN());

        float range = 0.0;
        float intensity = 0.0;
        float dir_angle;
        unsigned int last_index = 0;

        for (int i = 0; i < point_nums; i++)
        {
            range = scan_frame->data[i].distance * 0.001;
            intensity = scan_frame->data[i].intensity;

            if ((range > range_max_) || (range < range_min_))
            {
                range = 0.0;
                intensity = 0.0;
            }

            if (!clockwise_)
            {
                dir_angle = static_cast<float>(360.f - scan_frame->data[i].angle);
            }
            else
            {
                dir_angle = scan_frame->data[i].angle;
            }

            if ((dir_angle < angle_min_) || (dir_angle > angle_max_))
            {
                range = 0;
                intensity = 0;
            }

            float angle = Degree2Rad(dir_angle);
            unsigned int index = (unsigned int)((angle - scanMsg->angle_min) / scanMsg->angle_increment);
            if (index < point_nums)
            {
                // 如果当前位置是NaN，则直接赋值
                if (std::isnan(scanMsg->ranges[index]))
                {
                    scanMsg->ranges[index] = range;
                    unsigned int err = index - last_index;
                    if (err == 2)
                    {
                        scanMsg->ranges[index - 1] = range;
                        scanMsg->intensities[index - 1] = intensity;
                    }
                }
                else
                {
                    // 否则，只有当距离小于当前值时才能重新赋值
                    if (range < scanMsg->ranges[index] && range > 0)
                    {
                        scanMsg->ranges[index] = range;
                    }
                }
                scanMsg->intensities[index] = intensity;
                last_index = index;
            }
        }

        publisher_->publish(std::move(scanMsg));
    }

    // 成员变量
    std::string port_;
    int baudrate_;
    double angle_min_;
    double angle_max_;
    double range_min_;
    double range_max_;
    bool clockwise_;
    int motor_speed_;
    std::string device_model_;
    std::string frame_id_;
    std::string scan_topic_;
    double time_adjustment_;
    
    std::unique_ptr<OrdlidarDriver> device_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr scan_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OradarLidarNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

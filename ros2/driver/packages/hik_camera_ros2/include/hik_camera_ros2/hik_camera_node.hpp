#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/float64.hpp"
#include <image_transport/image_transport.hpp>
#include <thread>
#include <string>

// 使用 extern "C" 來確保 C/C++ 兼容性，並集中管理所有 C 語言頭文件
extern "C" {
#include "MvErrorDefine.h"
#include "MvCameraControl.h"
}

class HikCameraNode : public rclcpp::Node
{
public:
    HikCameraNode();
    ~HikCameraNode();

private:
    // 初始化與關閉相機的函數
    void openCamera();
    void closeCamera();
    
    // 獨立線程中運行的圖像抓取循環
    void grabLoop();

    // 讀取並設定相機參數的函數，回傳 bool 表示成功或失敗
    bool updateParams();
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters);

    // 斷線重連的回呼函數
    void healthCheckCallback();

    // 讀取並發布實際幀率的回呼函數
    void fpsCallback();

    // 相機句柄與狀態
    void* handle_;
    bool connected_;
    bool running_;
    std::thread grab_thread_;

    // ROS 相關
    std::shared_ptr<image_transport::CameraPublisher> image_pub_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    rclcpp::TimerBase::SharedPtr health_check_timer_;
    rclcpp::TimerBase::SharedPtr fps_timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fps_pub_;
};
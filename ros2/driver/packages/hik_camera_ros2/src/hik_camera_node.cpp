#include "hik_camera_ros2/hik_camera_node.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cstring>
#include <string>
#include <map>

HikCameraNode::HikCameraNode()
: Node("hik_camera_node"), connected_(false), running_(true), handle_(nullptr)
{
    MV_CC_Initialize();
    RCLCPP_INFO(this->get_logger(), "Starting HikCameraNode...");

    this->declare_parameter("exposure_time", 5000.0);
    this->declare_parameter("gain", 5.0);
    this->declare_parameter("frame_rate", 20.0);
    this->declare_parameter("pixel_format", "Mono8");
    this->declare_parameter("serial_number", "");
    this->declare_parameter("topic_name", "/image_raw");

    std::string topic_name = this->get_parameter("topic_name").as_string();
    RCLCPP_INFO(this->get_logger(), "Image topic will be: %s", topic_name.c_str());
    image_pub_ = std::make_shared<image_transport::CameraPublisher>(this, topic_name);
    fps_pub_ = this->create_publisher<std_msgs::msg::Float64>("~/actual_frame_rate", 10);

    openCamera();

    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&HikCameraNode::parametersCallback, this, std::placeholders::_1));

    if (connected_) {
        grab_thread_ = std::thread(&HikCameraNode::grabLoop, this);
        RCLCPP_INFO(this->get_logger(), "HikCameraNode started successfully.");
    } else {
        RCLCPP_ERROR(this->get_logger(), "HikCameraNode failed to start because camera connection failed.");
    }

    health_check_timer_ = this->create_wall_timer(
        std::chrono::seconds(3),
        std::bind(&HikCameraNode::healthCheckCallback, this));
        
    fps_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&HikCameraNode::fpsCallback, this));
}

HikCameraNode::~HikCameraNode()
{
    running_ = false;
    if (grab_thread_.joinable()) {
        grab_thread_.join();
    }
    closeCamera();
    MV_CC_Finalize();
    RCLCPP_INFO(this->get_logger(), "HikCameraNode has been shut down.");
}

void HikCameraNode::openCamera()
{
    int ret = MV_OK;
    MV_CC_DEVICE_INFO_LIST devList;
    
    ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &devList);
    if (ret != MV_OK || devList.nDeviceNum == 0) {
        RCLCPP_ERROR(this->get_logger(), "No Hik camera found.");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Found %d devices.", devList.nDeviceNum);

    std::string serial_number_from_launch = this->get_parameter("serial_number").as_string();
    RCLCPP_INFO(this->get_logger(), "Parameter SN from launch file: '[%s]' (length: %zu)", 
        serial_number_from_launch.c_str(), serial_number_from_launch.length());

    int camera_index = -1;

    if (serial_number_from_launch.empty()) {
        RCLCPP_INFO(this->get_logger(), "Serial number is empty, connecting to the first camera (index 0).");
        camera_index = 0;
    } else {
        RCLCPP_INFO(this->get_logger(), "Searching for camera with specified serial number...");
        for (unsigned int i = 0; i < devList.nDeviceNum; i++) {
            MV_CC_DEVICE_INFO* pDeviceInfo = devList.pDeviceInfo[i];
            const char* current_sn_c_str = "";
            if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) {
                current_sn_c_str = (const char*)pDeviceInfo->SpecialInfo.stGigEInfo.chSerialNumber;
            } else if (pDeviceInfo->nTLayerType == MV_USB_DEVICE) {
                current_sn_c_str = (const char*)pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber;
            }
            std::string current_sn_str(current_sn_c_str);
            
            RCLCPP_INFO(this->get_logger(), "Device %d reported SN: '[%s]' (length: %zu)", 
                i, current_sn_str.c_str(), current_sn_str.length());

            if (current_sn_str == serial_number_from_launch) {
                camera_index = i;
                RCLCPP_INFO(this->get_logger(), "Found matching camera at index %d.", camera_index);
                break;
            }
        }
    }

    if (camera_index == -1) {
        RCLCPP_ERROR(this->get_logger(), "Could not find a camera to connect to.");
        return;
    }
    
    ret = MV_CC_CreateHandle(&handle_, devList.pDeviceInfo[camera_index]);
    if (ret != MV_OK) { 
        RCLCPP_ERROR(this->get_logger(), "CreateHandle failed! ret=[0x%x]", ret); 
        return; 
    }

    ret = MV_CC_OpenDevice(handle_);
    if (ret != MV_OK) { 
        RCLCPP_ERROR(this->get_logger(), "OpenDevice failed! ret=[0x%x]", ret); 
        MV_CC_DestroyHandle(handle_); 
        handle_ = nullptr; 
        return; 
    }

    if (devList.pDeviceInfo[camera_index]->nTLayerType == MV_GIGE_DEVICE) {
        int nPacketSize = MV_CC_GetOptimalPacketSize(handle_);
        if (nPacketSize > 0) {
            ret = MV_CC_SetIntValueEx(handle_, "GevSCPSPacketSize", nPacketSize);
            if(ret != MV_OK) { 
                RCLCPP_WARN(get_logger(), "Set Packet Size failed! ret [0x%x]", ret); 
            }
        } else { 
            RCLCPP_WARN(get_logger(), "Get Packet Size failed! ret [0x%x]", nPacketSize); 
        }
    }

    ret = MV_CC_SetEnumValue(handle_, "ExposureAuto", 0);
    if (ret != MV_OK) { 
        RCLCPP_WARN(this->get_logger(), "Failed to turn off Auto Exposure! ret=[0x%x]", ret); 
    }
    
    ret = MV_CC_SetEnumValue(handle_, "GainAuto", 0);
    if (ret != MV_OK) { 
        RCLCPP_WARN(this->get_logger(), "Failed to turn off Auto Gain! ret=[0x%x]", ret); 
    }
    
    ret = MV_CC_SetEnumValue(handle_, "TriggerMode", 0);
    if (ret != MV_OK) { 
        RCLCPP_ERROR(this->get_logger(), "Set TriggerMode failed! ret=[0x%x]", ret); 
        return; 
    }

    if (!updateParams()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to apply initial parameters. Aborting camera open.");
        MV_CC_CloseDevice(handle_);
        MV_CC_DestroyHandle(handle_);
        handle_ = nullptr;
        return;
    }

    ret = MV_CC_StartGrabbing(handle_);
    if (ret != MV_OK) { 
        RCLCPP_ERROR(this->get_logger(), "StartGrabbing failed! ret=[0x%x]", ret); 
        return; 
    }

    connected_ = true;
    RCLCPP_INFO(this->get_logger(), "Camera connected and started grabbing.");
}

void HikCameraNode::closeCamera()
{
    if (connected_ && handle_ != nullptr) {
        MV_CC_StopGrabbing(handle_);
        MV_CC_CloseDevice(handle_);
        MV_CC_DestroyHandle(handle_);
        handle_ = nullptr;
        connected_ = false;
        RCLCPP_INFO(this->get_logger(), "Camera closed.");
    }
}

void HikCameraNode::grabLoop()
{
    MV_FRAME_OUT stFrame = {0};

    while (rclcpp::ok() && running_) {
        if (!connected_) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        int ret = MV_CC_GetImageBuffer(handle_, &stFrame, 1000);
        
        if (ret == MV_OK) {
            if (stFrame.pBufAddr == NULL) {
                RCLCPP_WARN(this->get_logger(), "Got empty buffer from SDK.");
                MV_CC_FreeImageBuffer(handle_, &stFrame);
                continue;
            }

            cv::Mat frame_mat;
            std::string ros_encoding;
            MvGvspPixelType pixel_type = stFrame.stFrameInfo.enPixelType;

            if (pixel_type == PixelType_Gvsp_Mono8) {
                cv::Mat raw_image(stFrame.stFrameInfo.nHeight, stFrame.stFrameInfo.nWidth, CV_8UC1, stFrame.pBufAddr);
                frame_mat = raw_image.clone();
                ros_encoding = sensor_msgs::image_encodings::MONO8;
            } 
            // Extend color conversion to cover both BayerRG8 and BayerGB8
            else if (pixel_type == PixelType_Gvsp_BayerRG8 || pixel_type == PixelType_Gvsp_BayerGB8) {
                cv::Mat raw_image(stFrame.stFrameInfo.nHeight, stFrame.stFrameInfo.nWidth, CV_8UC1, stFrame.pBufAddr);
                cv::Mat color_image;

                if (pixel_type == PixelType_Gvsp_BayerRG8) {
                    cv::cvtColor(raw_image, color_image, cv::COLOR_BayerRG2BGR);
                } 
                else if (pixel_type == PixelType_Gvsp_BayerGB8) {
                    // 【强制修正】虽然格式是 GB8，但为了修复蓝色手问题，强制使用 GR 格式解码
                    // 原理：GR 格式把红色像素放在第二个位置，正好能把被误认为蓝色的红色纠正回来
                    cv::cvtColor(raw_image, color_image, cv::COLOR_BayerGR2BGR);
                }

                frame_mat = color_image.clone();
                ros_encoding = sensor_msgs::image_encodings::BGR8;
            } 
            else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                    "Received an unsupported pixel format: [0x%lx]. Cannot process.", pixel_type);
                MV_CC_FreeImageBuffer(handle_, &stFrame);
                continue;
            }

            MV_CC_FreeImageBuffer(handle_, &stFrame);

            auto image_msg = cv_bridge::CvImage(
                std_msgs::msg::Header(),
                ros_encoding,
                frame_mat
            ).toImageMsg();
            image_msg->header.stamp = this->now();
            image_msg->header.frame_id = "camera_frame";

            auto camera_info_msg = std::make_shared<sensor_msgs::msg::CameraInfo>();
            camera_info_msg->header = image_msg->header;

            image_pub_->publish(*image_msg, *camera_info_msg);

        } else {
            if (ret == MV_E_BUSY || ret == MV_E_NODATA) {
                RCLCPP_ERROR(this->get_logger(), "Camera seems disconnected or busy! Error code: [0x%x]. Closing camera to reconnect.", ret);
                closeCamera();
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                    "GetImageBuffer failed with a non-critical error code: %#x", ret);
            }
        }
    }
}

void HikCameraNode::healthCheckCallback()
{
    if (!connected_) {
        RCLCPP_INFO(this->get_logger(), "Attempting to reconnect camera...");
        openCamera();
        
        if (connected_ && !grab_thread_.joinable()) {
             grab_thread_ = std::thread(&HikCameraNode::grabLoop, this);
        } else if (connected_) {
            RCLCPP_INFO(this->get_logger(), "Camera reconnected successfully.");
        }
    }
}

bool HikCameraNode::updateParams()
{
    RCLCPP_INFO(this->get_logger(), "Applying camera parameters...");
    int ret = MV_OK;
    bool all_ok = true;

    // 設定像素格式
    const static std::map<std::string, MvGvspPixelType> PIXEL_FORMAT_MAP = {
        {"Mono8", PixelType_Gvsp_Mono8},
        {"BayerRG8", PixelType_Gvsp_BayerRG8},
        {"BayerGB8", PixelType_Gvsp_BayerGB8}
    };
    std::string pixel_format_str = this->get_parameter("pixel_format").as_string();
    if (PIXEL_FORMAT_MAP.count(pixel_format_str)) {
        ret = MV_CC_SetEnumValue(handle_, "PixelFormat", PIXEL_FORMAT_MAP.at(pixel_format_str));
        if (ret != MV_OK) { 
            RCLCPP_ERROR(this->get_logger(), "Failed to set PixelFormat to %s, ret=[0x%x]", pixel_format_str.c_str(), ret);
            all_ok = false;
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Unsupported PixelFormat specified: %s", pixel_format_str.c_str());
        all_ok = false;
    }

    // 設定曝光、增益、幀率
    double exposure_time = this->get_parameter("exposure_time").as_double();
    ret = MV_CC_SetFloatValue(handle_, "ExposureTime", exposure_time);
    if (ret != MV_OK) { 
        RCLCPP_ERROR(this->get_logger(), "Failed to set ExposureTime to %.1f, ret=[0x%x]", exposure_time, ret);
        all_ok = false;
    }

    double gain = this->get_parameter("gain").as_double();
    ret = MV_CC_SetFloatValue(handle_, "Gain", gain);
    if (ret != MV_OK) { 
        RCLCPP_ERROR(this->get_logger(), "Failed to set Gain to %.1f, ret=[0x%x]", gain, ret); 
        all_ok = false;
    }

    double frame_rate = this->get_parameter("frame_rate").as_double();
    ret = MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", frame_rate);
    if (ret != MV_OK) { 
        RCLCPP_ERROR(this->get_logger(), "Failed to set AcquisitionFrameRate to %.1f, ret=[0x%x]", frame_rate, ret); 
        all_ok = false;
    }
    
    return all_ok;
}

rcl_interfaces::msg::SetParametersResult HikCameraNode::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto &param : parameters) {
        std::string name = param.get_name();
        int ret = MV_OK;

        if (name == "exposure_time") {
            ret = MV_CC_SetFloatValue(handle_, "ExposureTime", param.as_double());
        } else if (name == "gain") {
            ret = MV_CC_SetFloatValue(handle_, "Gain", param.as_double());
        } else if (name == "frame_rate") {
            ret = MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", param.as_double());
        } // 像素格式的動態調整較複雜，暫未在此處實現
        
        if (ret != MV_OK) {
            result.successful = false;
            RCLCPP_WARN(this->get_logger(), "Failed to set parameter '%s'. ret=[0x%x]", name.c_str(), ret);
        } else {
            RCLCPP_INFO(this->get_logger(), "Dynamic >> Parameter '%s' set successfully.", name.c_str());
        }
    }
    return result;
}

void HikCameraNode::fpsCallback()
{
    if (!connected_) {
        return;
    }

    MVCC_FLOATVALUE stFloatValue = {0};
    int ret = MV_CC_GetFloatValue(handle_, "ResultingFrameRate", &stFloatValue);

    if (ret == MV_OK) {
        auto msg = std_msgs::msg::Float64();
        msg.data = stFloatValue.fCurValue;
        fps_pub_->publish(msg);
    } 
    else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
            "Failed to get ResultingFrameRate. ret=[0x%x]", ret);
    }
}

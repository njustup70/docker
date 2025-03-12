#include <ahrs_driver.h>

rclcpp::Node::SharedPtr nh_ = nullptr;

namespace FDILink {
ahrsBringup::ahrsBringup()
    : rclcpp::Node("ahrs_bringup") {
    // 只保留IMU相关参数
    this->declare_parameter("if_debug_", false);
    this->get_parameter("if_debug_", if_debug_);

    this->declare_parameter<std::int8_t>("device_type_", 1);
    this->get_parameter("device_type_", device_type_);

    this->declare_parameter<std::string>("imu_topic", "/imu");
    this->get_parameter("imu_topic", imu_topic);

    this->declare_parameter<std::string>("imu_frame_id_", "gyro_link");
    this->get_parameter("imu_frame_id_", imu_frame_id_);

    this->declare_parameter<std::string>("serial_port_", "/dev/fdilink_ahrs");
    this->get_parameter("serial_port_", serial_port_);

    this->declare_parameter<std::int64_t>("serial_baud_", 921600);
    this->get_parameter("serial_baud_", serial_baud_);

    // 只保留IMU发布者
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic.c_str(), 10);

    // 串口初始化保持不变
    try {
        serial_.setPort(serial_port_);
        serial_.setBaudrate(serial_baud_);
        serial_.setFlowcontrol(serial::flowcontrol_none);
        serial_.setParity(serial::parity_none);
        serial_.setStopbits(serial::stopbits_one);
        serial_.setBytesize(serial::eightbits);
        serial::Timeout time_out = serial::Timeout::simpleTimeout(serial_timeout_);
        serial_.setTimeout(time_out);
        serial_.open();
    } catch (serial::IOException& e) {
        RCLCPP_ERROR(this->get_logger(), "Unable to open port ");
        exit(0);
    }
    if (serial_.isOpen()) {
        RCLCPP_INFO(this->get_logger(), "Serial Port initialized");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Unable to initial Serial port ");
        exit(0);
    }
    processLoop();
}

ahrsBringup::~ahrsBringup() {
    if (serial_.isOpen())
        serial_.close();
}

void ahrsBringup::processLoop() {
    RCLCPP_INFO(this->get_logger(), "ahrsBringup::processLoop: start");
    while (rclcpp::ok()) {
        if (!serial_.isOpen()) {
            RCLCPP_WARN(this->get_logger(), "serial unopen");
        }
        // 检查帧头
        uint8_t check_head[1] = {0xff};
        size_t head_s         = serial_.read(check_head, 1);
        if (if_debug_) {
            if (head_s != 1) {
                RCLCPP_ERROR(
                    this->get_logger(), "Read serial port time out! can't read pack head.");
            }
            std::cout << std::endl;
            std::cout << "check_head: " << std::hex << (int)check_head[0] << std::dec << std::endl;
        }
        if (check_head[0] != FRAME_HEAD) {
            continue;
        }
        // 检查数据类型
        uint8_t head_type[1] = {0xff};
        size_t type_s        = serial_.read(head_type, 1);
        if (if_debug_) {
            std::cout << "head_type:  " << std::hex << (int)head_type[0] << std::dec << std::endl;
        }
        // 只处理IMU数据类型
        if (head_type[0] != TYPE_IMU) {
            continue;
        }
        // 检查数据长度
        uint8_t check_len[1] = {0xff};
        size_t len_s         = serial_.read(check_len, 1);
        if (if_debug_) {
            std::cout << "check_len: " << std::dec << (int)check_len[0] << std::endl;
        }
        if (check_len[0] != IMU_LEN) {
            RCLCPP_WARN(this->get_logger(), "head_len error (imu)");
            continue;
        }
        // 读取并校验序列号
        uint8_t check_sn[1]     = {0xff};
        size_t sn_s             = serial_.read(check_sn, 1);
        uint8_t head_crc8[1]    = {0xff};
        size_t crc8_s           = serial_.read(head_crc8, 1);
        uint8_t head_crc16_H[1] = {0xff};
        uint8_t head_crc16_L[1] = {0xff};
        size_t crc16_H_s        = serial_.read(head_crc16_H, 1);
        size_t crc16_L_s        = serial_.read(head_crc16_L, 1);
        if (if_debug_) {
            std::cout << "check_sn: " << std::hex << (int)check_sn[0] << std::dec << std::endl;
            std::cout << "head_crc8: " << std::hex << (int)head_crc8[0] << std::dec << std::endl;
            std::cout << "head_crc16_H: " << std::hex << (int)head_crc16_H[0] << std::dec
                      << std::endl;
            std::cout << "head_crc16_L: " << std::hex << (int)head_crc16_L[0] << std::dec
                      << std::endl;
        }
        // IMU数据处理
        imu_frame_.frame.header.header_start   = check_head[0];
        imu_frame_.frame.header.data_type      = head_type[0];
        imu_frame_.frame.header.data_size      = check_len[0];
        imu_frame_.frame.header.serial_num     = check_sn[0];
        imu_frame_.frame.header.header_crc8    = head_crc8[0];
        imu_frame_.frame.header.header_crc16_h = head_crc16_H[0];
        imu_frame_.frame.header.header_crc16_l = head_crc16_L[0];
        uint8_t CRC8                           = CRC8_Table(imu_frame_.read_buf.frame_header, 4);
        if (CRC8 != imu_frame_.frame.header.header_crc8) {
            RCLCPP_WARN(this->get_logger(), "header_crc8 error");
            continue;
        }
        if (!frist_sn_) {
            read_sn_  = imu_frame_.frame.header.serial_num - 1;
            frist_sn_ = true;
        }
        checkSN(TYPE_IMU);
        // CRC16校验
        uint16_t head_crc16 =
            imu_frame_.frame.header.header_crc16_l + (imu_frame_.frame.header.header_crc16_h << 8);
        size_t data_s  = serial_.read(imu_frame_.read_buf.read_msg, (IMU_LEN + 1));
        uint16_t CRC16 = CRC16_Table(imu_frame_.frame.data.data_buff, IMU_LEN);
        if (head_crc16 != CRC16 || imu_frame_.frame.frame_end != FRAME_END) {
            RCLCPP_WARN(this->get_logger(), "CRC16 or frame end check failed");
            continue;
        }
        // 发布IMU数据
        sensor_msgs::msg::Imu imu_data;
        imu_data.header.stamp    = rclcpp::Node::now();
        imu_data.header.frame_id = imu_frame_id_.c_str();
        if (device_type_ == 1) // 坐标变换处理
        {
            Eigen::Quaterniond q_ahrs(
                ahrs_frame_.frame.data.data_pack.Qw, ahrs_frame_.frame.data.data_pack.Qx,
                ahrs_frame_.frame.data.data_pack.Qy, ahrs_frame_.frame.data.data_pack.Qz);
            Eigen::Quaterniond q_r = Eigen::AngleAxisd(PI, Eigen::Vector3d::UnitZ())
                                   * Eigen::AngleAxisd(PI, Eigen::Vector3d::UnitY())
                                   * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
            Eigen::Quaterniond q_rr = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ())
                                    * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())
                                    * Eigen::AngleAxisd(PI, Eigen::Vector3d::UnitX());
            Eigen::Quaterniond q_out       = q_r * q_ahrs * q_rr;
            imu_data.orientation.w         = q_out.w();
            imu_data.orientation.x         = q_out.x();
            imu_data.orientation.y         = q_out.y();
            imu_data.orientation.z         = q_out.z();
            imu_data.angular_velocity.x    = imu_frame_.frame.data.data_pack.gyroscope_x;
            imu_data.angular_velocity.y    = -imu_frame_.frame.data.data_pack.gyroscope_y;
            imu_data.angular_velocity.z    = -imu_frame_.frame.data.data_pack.gyroscope_z;
            imu_data.linear_acceleration.x = imu_frame_.frame.data.data_pack.accelerometer_x;
            imu_data.linear_acceleration.y = -imu_frame_.frame.data.data_pack.accelerometer_y;
            imu_data.linear_acceleration.z = -imu_frame_.frame.data.data_pack.accelerometer_z;
        }
        imu_pub_->publish(imu_data);
    }
}

void ahrsBringup::checkSN(int type) {
    if (type != TYPE_IMU)
        return;

    if (++read_sn_ != imu_frame_.frame.header.serial_num) {
        if (imu_frame_.frame.header.serial_num < read_sn_) {
            sn_lost_ += 256 - (int)(read_sn_ - imu_frame_.frame.header.serial_num);
        } else {
            sn_lost_ += (int)(imu_frame_.frame.header.serial_num - read_sn_);
        }
        if (if_debug_) {
            RCLCPP_WARN(this->get_logger(), "detected sn lost.");
        }
    }
    read_sn_ = imu_frame_.frame.header.serial_num;
}

} // namespace FDILink

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    FDILink::ahrsBringup bp;
    rclcpp::spin(std::make_shared<FDILink::ahrsBringup>());
    return 0;
}
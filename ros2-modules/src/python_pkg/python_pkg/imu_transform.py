import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import numpy as np
import yaml
import os

class ImuTransform(Node):
    def __init__(self):
        super().__init__('imu_transform_node')
        self.get_logger().info("IMU 转换节点已启动")
        
        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('imu_topic', '/imu_raw'),
                ('imu_transformed_topic', '/imu_lidar_frame'),
                ('imu_frame', 'imu_link'),
                ('lidar_frame', 'lidar_link'),
                ('Calibration_file', 'config/imu_transform.yaml'),
                ('pub_tf', True),
                ('use_transform', True),
                ('use_grivaty2m', True)
            ]
        )
        
        # 获取参数
        self.imu_topic = self.get_parameter('imu_topic').value
        self.imu_transformed_topic = self.get_parameter('imu_transformed_topic').value
        self.imu_frame = self.get_parameter('imu_frame').value
        self.lidar_frame = self.get_parameter('lidar_frame').value
        calib_file = self.get_parameter('Calibration_file').value
        self.use_grivaty2m = self.get_parameter('use_grivaty2m').value
        # 订阅和发布
        self.imu_sub = self.create_subscription(
            Imu, 
            self.imu_topic, 
            self.imu_callback, 
            10
        )
        self.imu_pub = self.create_publisher(Imu, self.imu_transformed_topic, 10)
        self.imu_cnt=0
        self.imu_gravity_trans_trigger=False
        # 初始化变换矩阵
        self.imu_to_lidar = self.parse_calibration_file(calib_file)
        self._gravity = 9.81
        self.normal_avg=0
        # 初始化 TF 广播
        if self.get_parameter('pub_tf').value:
            self.static_broadcaster = StaticTransformBroadcaster(self)
            self.publish_static_tf()

    def parse_calibration_file(self, file_path):
        """支持解析 YAML/TXT 文件"""
        if not os.path.exists(file_path):
            self.get_logger().error(f"校准文件不存在: {file_path}")
            raise FileNotFoundError(f"Calibration file {file_path} not found")

        matrix = np.eye(4)
        try:
            if file_path.endswith(('.yaml', '.yml')):
                with open(file_path) as f:
                    data = yaml.safe_load(f)
                    matrix = np.array(data['transformation_matrix'])
            else:  # 处理 TXT 文件
                with open(file_path) as f:
                    matrix_started = False
                    row = 0
                    for line in f:
                        if "Homogeneous Transformation Matrix from LiDAR to IMU" in line:
                            matrix_started = True
                            continue
                        if matrix_started and row < 4:
                            vals = list(map(float, line.strip().split()))
                            matrix[row] = vals[:4]
                            row += 1
        except Exception as e:
            self.get_logger().error(f"解析校准文件失败: {str(e)}")
            raise

        return np.linalg.inv(matrix)

    def imu_callback(self, msg):
        transformed_msg = Imu()
        transformed_msg.header = msg.header
        
        # 处理重力单位转换
        if self.use_grivaty2m and self.imu_cnt < 10:
            #计算合并加速度
            normal_sum = np.sqrt(
                msg.linear_acceleration.x**2 +
                msg.linear_acceleration.y**2 +
                msg.linear_acceleration.z**2
            )
            self.normal_avg += normal_sum/10
            self.imu_cnt += 1
            return
        elif self.use_grivaty2m and self.imu_cnt == 10:
            if self.normal_avg<5:
                self.imu_gravity_trans_trigger=True
        if self.imu_gravity_trans_trigger:
            transformed_msg.linear_acceleration.x = msg.linear_acceleration.x * self._gravity
            transformed_msg.linear_acceleration.y = msg.linear_acceleration.y * self._gravity
            transformed_msg.linear_acceleration.z = msg.linear_acceleration.z * self._gravity
            transformed_msg.angular_velocity.x = msg.angular_velocity.x 
            transformed_msg.angular_velocity.y = msg.angular_velocity.y
            transformed_msg.angular_velocity.z = msg.angular_velocity.z
            self.imu_pub.publish(transformed_msg)
            return
        else:
            transformed_msg.linear_acceleration = msg.linear_acceleration
            
        # 应用坐标变换
        if self.get_parameter('use_transform').value:
            # 加速度变换
            acc = np.array([
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
                0
            ])
            acc_transformed = self.imu_to_lidar @ acc
            transformed_msg.linear_acceleration.x = acc_transformed[0]
            transformed_msg.linear_acceleration.y = acc_transformed[1]
            transformed_msg.linear_acceleration.z = acc_transformed[2]
            
            # 角速度变换
            gyro = np.array([
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
                0
            ])
            gyro_transformed = self.imu_to_lidar @ gyro
            transformed_msg.angular_velocity.x = gyro_transformed[0]
            transformed_msg.angular_velocity.y = gyro_transformed[1]
            transformed_msg.angular_velocity.z = gyro_transformed[2]
        
        # 更新坐标系
        transformed_msg.header.frame_id = self.lidar_frame
        self.imu_pub.publish(transformed_msg)

    def publish_static_tf(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.imu_frame
        transform.child_frame_id = self.lidar_frame
        
        # 设置平移
        transform.transform.translation.x = self.imu_to_lidar[0, 3]
        transform.transform.translation.y = self.imu_to_lidar[1, 3]
        transform.transform.translation.z = self.imu_to_lidar[2, 3]
        
        # 计算四元数
        rotation_matrix = self.imu_to_lidar[:3, :3]
        q = self.matrix_to_quaternion(rotation_matrix)
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        
        self.static_broadcaster.sendTransform(transform)

    @staticmethod
    def matrix_to_quaternion(matrix):
        """将旋转矩阵转换为四元数"""
        m = matrix.T  # ROS 使用主动变换矩阵
        trace = np.trace(m)
        if trace > 0:
            S = np.sqrt(trace + 1.0) * 2
            qw = 0.25 * S
            qx = (m[2, 1] - m[1, 2]) / S
            qy = (m[0, 2] - m[2, 0]) / S
            qz = (m[1, 0] - m[0, 1]) / S
        elif (m[0, 0] > m[1, 1]) and (m[0, 0] > m[2, 2]):
            S = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
            qw = (m[2, 1] - m[1, 2]) / S
            qx = 0.25 * S
            qy = (m[0, 1] + m[1, 0]) / S
            qz = (m[0, 2] + m[2, 0]) / S
        elif m[1, 1] > m[2, 2]:
            S = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
            qw = (m[0, 2] - m[2, 0]) / S
            qx = (m[0, 1] + m[1, 0]) / S
            qy = 0.25 * S
            qz = (m[1, 2] + m[2, 1]) / S
        else:
            S = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
            qw = (m[1, 0] - m[0, 1]) / S
            qx = (m[0, 2] + m[2, 0]) / S
            qy = (m[1, 2] + m[2, 1]) / S
            qz = 0.25 * S
        return np.array([qx, qy, qz, qw])

def main(args=None):
    rclpy.init(args=args)
    node = ImuTransform()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
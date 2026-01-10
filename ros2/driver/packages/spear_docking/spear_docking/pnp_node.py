import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from .vision_pnp import VisionPnP
from rclpy.qos import qos_profile_sensor_data

def rotation_matrix_to_quaternion(R):
    """鲁棒的旋转矩阵转四元数算法"""
    trace = np.trace(R)
    q = np.zeros(4)
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        q[3] = 0.25 / s
        q[0] = (R[2, 1] - R[1, 2]) * s
        q[1] = (R[0, 2] - R[2, 0]) * s
        q[2] = (R[1, 0] - R[0, 1]) * s
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            q[3] = (R[2, 1] - R[1, 2]) / s
            q[0] = 0.25 * s
            q[1] = (R[0, 1] + R[1, 0]) / s
            q[2] = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            q[3] = (R[0, 2] - R[2, 0]) / s
            q[0] = (R[0, 1] + R[1, 0]) / s
            q[1] = 0.25 * s
            q[2] = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            q[3] = (R[1, 0] - R[0, 1]) / s
            q[0] = (R[0, 2] + R[2, 0]) / s
            q[1] = (R[1, 2] + R[2, 1]) / s
            q[2] = 0.25 * s
    return q # x, y, z, w

class VisionPnPNode(Node):
    def __init__(self):
        super().__init__("vision_pnp_node")
        self.bridge = CvBridge()
        
        # 初始化算法
        self.pnp = VisionPnP(smoothing_alpha=0.7)

        # 发布目标位姿 (Camera Coordinates: Z=Depth, X=Right, Y=Down)
        self.pose_pub = self.create_publisher(PoseStamped, "/target_pose", 10)
        
        # 订阅海康相机
        self.create_subscription(Image, "/hik_camera/image_raw", self.image_callback, qos_profile_sensor_data)
        self.get_logger().info("Visual Docking Node Started.")

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return

        pose = self.pnp.estimate_pose(frame)
        
        if pose is None:
            # 可选：打印未检测到
            self.get_logger().warn("No markers detected", throttle_duration_sec=1.0)
            return

        # 提取数据
        tvec = pose.tvec.reshape(-1)
        rvec = pose.rvec.reshape(-1)
        
        # 打印调试信息 (单位: 米)
        # Z: 前后距离, X: 左右偏差
        self.get_logger().info(
            f"Distance(Z): {tvec[2]:.4f} m | Horizontal(X): {tvec[0]:.4f} m"
        )

        # 构建 ROS 消息
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "hik_camera_optical_frame"

        # 填充位置
        pose_msg.pose.position.x = float(tvec[0])
        pose_msg.pose.position.y = float(tvec[1])
        pose_msg.pose.position.z = float(tvec[2])

        # 填充姿态 (旋转矩阵转四元数)
        R, _ = cv2.Rodrigues(rvec)
        q = rotation_matrix_to_quaternion(R)
        pose_msg.pose.orientation.x = float(q[0])
        pose_msg.pose.orientation.y = float(q[1])
        pose_msg.pose.orientation.z = float(q[2])
        pose_msg.pose.orientation.w = float(q[3])

        self.pose_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VisionPnPNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
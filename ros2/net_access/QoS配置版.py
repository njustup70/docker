import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VideoPublisher(Node):
    def __init__(self, video_path, width, height):
        super().__init__('video_publisher')

        # 设置 QoS 配置为 best_effort
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10  # 缓存深度
        )

        self.publisher_ = self.create_publisher(Image, 'video_frames', qos_profile)
        self.bridge = CvBridge()
        self.video_path = video_path
        self.width = width
        self.height = height
        self.timer = self.create_timer(0.033, self.publish_frame)  # 30 FPS
        self.cap = cv2.VideoCapture(self.video_path)

        if not self.cap.isOpened():
            self.get_logger().error(f"Cannot open video file: {self.video_path}")
            rclpy.shutdown()

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info("Video playback finished.")
            self.cap.release()
            rclpy.shutdown()
            return

        # Resize the frame to the desired resolution
        resized_frame = cv2.resize(frame, (self.width, self.height))

        # Convert the resized frame to ROS 2 Image message
        msg = self.bridge.cv2_to_imgmsg(resized_frame, encoding='bgr8')
        self.publisher_.publish(msg)
        self.get_logger().info('Published a resized video frame.')

def main(args=None):
    rclpy.init(args=args)

    # Path to the video file
    video_path = '/home/eason/1.mp4'  # 替换为你的文件路径

    # Desired resolution
    width = 320  # 修改为目标宽度
    height = 240  # 修改为目标高度

    node = VideoPublisher(video_path, width, height)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
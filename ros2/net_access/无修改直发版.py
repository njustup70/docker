import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
        self.timer = self.create_timer(0.03, self.timer_callback)  # 30 FPS
        self.cap = cv2.VideoCapture('2.mp4')  # 替换为视频文件路径
        self.bridge = CvBridge()
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open video file.')
            rclpy.shutdown()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info('End of video.')
            rclpy.shutdown()
            return
        # 转换 OpenCV 图像为 ROS 图像消息
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)
        self.get_logger().info('Published frame.')

def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


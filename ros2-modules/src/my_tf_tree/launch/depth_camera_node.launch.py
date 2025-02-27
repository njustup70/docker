import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image  # 假设的消息类型

class DepthCameraNode(Node):
    def __init__(self):
        super().__init__('depth_camera_node')
        self.publisher_ = self.create_publisher(Image, 'depth_image', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # 读取深度相机数据
        depth_image = Image()
        # 填充depth_image...
        self.publisher_.publish(depth_image)

def main(args=None):
    rclpy.init(args=args)
    depth_camera_node = DepthCameraNode()
    rclpy.spin(depth_camera_node)
    depth_camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




    
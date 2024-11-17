import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

class TransformListenerNode(Node):
    def __init__(self):
        super().__init__('transform_listener_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

    def look_up_transform(self):
        try:
            # 等待并获取从'world'到'robot_base'的变换
            t = self.tf_buffer.lookup_transform('world', 'robot_base', self.get_clock().now().to_msg())
            self.get_logger().info('Received transform: %s' % t)
        except Exception as e:
            self.get_logger().info('Exception: %s' % e)

def main(args=None):
    rclpy.init(args=args)
    node = TransformListenerNode()
    timer = node.create_timer(1.0, node.look_up_transform)  # 每秒查询一次变换
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
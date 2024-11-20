import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class TransformBroadcasterNode(Node):
    def __init__(self):
        super().__init__('transform_broadcaster_node')
        self.tf_broadcaster = TransformBroadcaster(self)

    def publish_transform(self):
        # 创建一个变换对象
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'robot_base'
        t.transform.translation.x = 1.0
        t.transform.translation.y = 1.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # 广播变换
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = TransformBroadcasterNode()
    timer = node.create_timer(1.0, node.publish_transform)  # 每秒广播一次变换
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
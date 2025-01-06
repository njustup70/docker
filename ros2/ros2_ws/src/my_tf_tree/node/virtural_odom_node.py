import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
import tf2_ros

class VirtualOdometryNode(Node):
    def __init__(self):
        super().__init__('virtual_odometry_node')
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

    def timer_callback(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Set the position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Set the orientation
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = 0.0
        odom_msg.pose.pose.orientation.w = 1.0

        # Set the velocity
        odom_msg.twist.twist.linear.x = 0.1  # Example velocity
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0

        # Publish the message
        self.odom_pub.publish(odom_msg)
        self.get_logger().info('Publishing odometry message.')

        # Update position for next iteration
        self.x += 0.1
        self.y += 0.0

def main(args=None):
    rclpy.init(args=args)
    virtual_odometry_node = VirtualOdometryNode()
    try:
        rclpy.spin(virtual_odometry_node)
    except ExternalShutdownException:
        virtual_odometry_node.get_logger().info('Shutting down...')
    finally:
        virtual_odometry_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
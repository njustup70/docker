import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import math
class JoyTeleopNode(Node):
    def __init__(self):
        super().__init__('joy_teleop_node')
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('linear_scale', 1.0)
        self.declare_parameter('angular_scale',math.pi)
        self.pub_cmd_vel = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.sub_joy = self.create_subscription(Joy, self.get_parameter('joy_topic').value, self.joy_callback, 10)

        self.emergency_stop = False

        # 速度缩放参数
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value

        self.get_logger().info("Joystick Teleop Node started.")
        self.last_twist= Twist()
    def joy_callback(self, msg: Joy):
        # B键一般是按钮1（不同手柄可能不同，需根据实际情况调整）
        b_button = msg.buttons[1]

        if b_button:
            self.emergency_stop = True
            self.get_logger().warn("Emergency Stop Activated!")
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 0.0
            self.pub_cmd_vel.publish(twist)
        else:
            self.emergency_stop = False
        twist = Twist()

        if not self.emergency_stop:
            # 左摇杆：axes[0] 为左右（y轴），axes[1] 为前后（x轴）
            twist.linear.x = msg.axes[1] * self.linear_scale
            twist.linear.y = msg.axes[0] * self.linear_scale
            # 右摇杆：axes[3] 为左右（控制yaw）
            twist.angular.z = msg.axes[3] * self.angular_scale
        else:
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 0.0
        #比较是否不变,不变就不发步
        
        if self.last_twist.linear.x != twist.linear.x or \
           self.last_twist.linear.y != twist.linear.y or \
           self.last_twist.angular.z != twist.angular.z:
            # 发布速度命令
            # self.get_logger().info(f"Publishing Twist: {twist}")
            self.pub_cmd_vel.publish(twist)
        self.last_twist = twist
def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

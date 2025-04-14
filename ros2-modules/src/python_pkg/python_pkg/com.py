import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import os
import sys
sys.path.append('/home/Elaina/docker/ros2-modules/src') 
# print(sys.path)
from protocol_lib.myserial import AsyncSerial_t
class Communicate_t(Node):
    def __init__(self):
        super().__init__('communicate_t')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('serial_baudrate', 115200)
        self.sub=self.create_subscription(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            self.cmd_topic_callback,
            10)
        try:
            self.serial=AsyncSerial_t(
                self.get_parameter('serial_port').value,
                self.get_parameter('serial_baudrate').value)
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise
    def cmd_topic_callback(self, msg:Twist):
        #获得信息发给串口
        linear_x=msg.linear.x
        linear_y=msg.linear.y
        angular_z=msg.angular.z
        # 发送数据到串口"浮点 空格 浮点 空格 浮点 p
        data=f"{linear_x} {linear_y} {angular_z} p\n"
        self.serial.write(data.encode())

        # self.subscriptions= self.       
def main(args=None):
    rclpy.init(args=args)
    node=Communicate_t()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import RadarData

class RadarNode(Node):
    def __init__(self):
        super().__init__('radar_node')
        self.publisher_ = self.create_publisher(RadarData, 'radar_data', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # 读取雷达数据
        radar_data = RadarData()
        # 填充radar_data...
        self.publisher_.publish(radar_data)

def main(args=None):
    rclpy.init(args=args)
    radar_node = RadarNode()
    rclpy.spin(radar_node)
    radar_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
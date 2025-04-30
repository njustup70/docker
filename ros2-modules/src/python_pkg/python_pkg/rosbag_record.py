import os
import time
import rclpy
import rclpy.executors
from rclpy.node import Node
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message
import shutil
class SmartBagRecorder(Node):
    def __init__(self):
        super().__init__('smart_bag_recorder')

        # Declare ROS parameters with default values
        self.declare_parameter('max_size_gb', 2.0) # 2GB
        self.declare_parameter('max_folder_num', 5)
        self.declare_parameter('record_images', False)
        self.declare_parameter('record_imu', True)
        self.declare_parameter('record_lidar', True)
        self.declare_parameter('record_nav',False)
        self.max_size_bytes = int(self.get_parameter('max_size_gb').value * 1024 ** 3)
        self.max_folder_num = self.get_parameter('max_folder_num').value
        self.record_images = self.get_parameter('record_images').value
        self.record_nav=self.get_parameter('record_nav').value
        self.record_dir_root = os.path.abspath(os.path.join( os.path.expanduser('~'),"docker/ros2-modules/rosbag_record"))
        self.bag_path = self.prepare_record_path()
        print(f'\033[95m📁 Recording to: {self.bag_path}\033[0m')

        # 初始化 writer
        self.writer = SequentialWriter()
        storage_options = StorageOptions(uri=self.bag_path, storage_id='sqlite3')
        converter_options = ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        # 获取当前活跃话题并订阅
        topic_names_and_types = self.get_topic_names_and_types()
        self.subscribers = []
        self.subscribed_topics = set()
        self.init_static_subscriptions()

        # 创建大小检查定时器
        self.timer = self.create_timer(3.0,self.timerCallback)

    def prepare_record_path(self):
        os.makedirs(self.record_dir_root, exist_ok=True)
        record_dirs = sorted(
            [d for d in os.listdir(self.record_dir_root) if os.path.isdir(os.path.join(self.record_dir_root, d))],
            key=lambda d: os.path.getctime(os.path.join(self.record_dir_root, d))
        )
        if len(record_dirs) >= self.max_folder_num:
            old_path = os.path.join(self.record_dir_root, record_dirs[0])
            print(f'\033[91m📦 Removing oldest folder: {old_path}\033[0m')
            os.system(f'rm -rf "{old_path}"')
        file_name = time.strftime("%m-%d-%H-%M", time.localtime())
        file_path = os.path.join(self.record_dir_root, file_name)
        # os.makedirs(file_path)
        if os.path.exists(file_path):
            print(f'\033[93m⚠️ Existing path detected, removing: {file_path}\033[0m')
            shutil.rmtree(file_path)

        return file_path

    def create_callback(self, topic_name):
        def callback(msg):
            try:
                self.writer.write(topic_name, serialize_message(msg), self.get_clock().now().nanoseconds)
            except Exception as e:
                print(f'\033[91m⚠️ Error writing message from {topic_name}: {e}\033[0m')
        return callback
    def subscribe_topic(self, topic_name, msg_type_str):
        if topic_name in self.subscribed_topics:
            return
        try:
            msg_type = get_message(msg_type_str)
            topic_info = TopicMetadata(name=topic_name, type=msg_type_str, serialization_format='cdr')
            self.writer.create_topic(topic_info)
            sub = self.create_subscription(msg_type, topic_name, self.create_callback(topic_name), 10)
            self.subscribers.append(sub)
            self.subscribed_topics.add(topic_name)
            print(f'\033[95m✅ Recording topic: {topic_name} [{msg_type_str}]\033[0m')
        except Exception as e:
            print(f'\033[91m⛔ Failed to subscribe to {topic_name}: {e}\033[0m')

    def init_static_subscriptions(self):
        topic_names_and_types = self.get_topic_names_and_types()
        needed_types = []
        if self.record_images:
            needed_types.append('sensor_msgs/msg/CompressedImage')
        if self.get_parameter('record_imu').value:
            needed_types.append('sensor_msgs/msg/Imu')
        if self.get_parameter('record_lidar').value:
            needed_types.append('sensor_msgs/msg/LidarScan')
            needed_types.append('sensor_msgs/msg/PointCloud2')
        for topic_name, types in topic_names_and_types:
            msg_type_str = types[0]
            if msg_type_str in needed_types:
                self.subscribe_topic(topic_name, msg_type_str)
    def check_size_limit(self):
        total_size = 0
        for root, dirs, files in os.walk(self.bag_path):
            for f in files:
                total_size += os.path.getsize(os.path.join(root, f))
        if total_size > self.max_size_bytes:
            print(f'\033[91m🚫 Reached size limit ({self.max_size_bytes} bytes). Shutting down...\033[0m')
            rclpy.shutdown()

    def check_and_subscribe_nav_topics(self):
        nav_types = [
            'geometry_msgs/msg/Twist',
            'geometry_msgs/msg/TwistStamped',
            'geometry_msgs/msg/PoseStamped',
            'tf2_msgs/msg/TFMessage',
            'nav_msgs/msg/Path',
            'nav_msgs/msg/OccupancyGrid'
        ]
        topic_names_and_types = self.get_topic_names_and_types()
        for topic_name, types in topic_names_and_types:
            msg_type_str = types[0]
            if msg_type_str in nav_types:
                self.subscribe_topic(topic_name, msg_type_str)
    def timerCallback(self):
        self.check_size_limit()
        if self.record_nav:
            self.check_and_subscribe_nav_topics()
        
def main(args=None):
    rclpy.init(args=args)
    exe=rclpy.executors.MultiThreadedExecutor()
    exe.add_node(SmartBagRecorder())
    exe.spin()
    # node = SmartBagRecorder()
    # rclpy.spin(node)
    # node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

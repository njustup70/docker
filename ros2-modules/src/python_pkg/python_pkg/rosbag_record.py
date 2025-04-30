import os
import time
import rclpy
from rclpy.node import Node
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message

class SmartBagRecorder(Node):
    def __init__(self):
        super().__init__('smart_bag_recorder')

        # Declare ROS parameters with default values
        self.declare_parameter('max_size_bytes', 2 * 1024 * 1024 * 1024)  # 2GB
        self.declare_parameter('max_folder_num', 5)
        self.declare_parameter('record_images', True)

        self.max_size_bytes = self.get_parameter('max_size_bytes').value
        self.max_folder_num = self.get_parameter('max_folder_num').value
        self.record_images = self.get_parameter('record_images').value

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

        for topic_name, types in topic_names_and_types:
            msg_type_str = types[0]

            if not self.record_images and 'image' in topic_name.lower():
                print(f'\033[91m⛔ Skipping image topic: {topic_name}\033[0m')
                continue

            try:
                msg_type = get_message(msg_type_str)
                topic_info = TopicMetadata(
                    name=topic_name,
                    type=msg_type_str,
                    serialization_format='cdr')
                self.writer.create_topic(topic_info)

                sub = self.create_subscription(
                    msg_type,
                    topic_name,
                    self.create_callback(topic_name),
                    10)
                self.subscribers.append(sub)
                print(f'\033[95m✅ Recording topic: {topic_name} [{msg_type_str}]\033[0m')
            except Exception as e:
                print(f'\033[91m⛔ Failed to subscribe to {topic_name}: {e}\033[0m')

        # 创建大小检查定时器
        self.timer = self.create_timer(5.0, self.check_size_limit)

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
        return file_path

    def create_callback(self, topic_name):
        def callback(msg):
            try:
                self.writer.write(topic_name, serialize_message(msg), self.get_clock().now().nanoseconds)
            except Exception as e:
                print(f'\033[91m⚠️ Error writing message from {topic_name}: {e}\033[0m')
        return callback

    def check_size_limit(self):
        total_size = 0
        for root, dirs, files in os.walk(self.bag_path):
            for f in files:
                total_size += os.path.getsize(os.path.join(root, f))
        if total_size > self.max_size_bytes:
            print(f'\033[91m🚫 Reached size limit ({self.max_size_bytes} bytes). Shutting down...\033[0m')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = SmartBagRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

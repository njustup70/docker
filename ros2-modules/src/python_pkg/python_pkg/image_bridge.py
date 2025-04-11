import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import zmq
import numpy as np
import multiprocessing.shared_memory as shm
import threading
import time
import os
class ImageBridgeNode(Node):
    def __init__(self):
        super().__init__('image_bridge_node')
        print("ImageBridgeNode initialized")
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('socket_address', 'tcp://localhost:5555')
        self.declare_parameter('socket_receive_address', 'tcp://localhost:5556')

        self.bridge = CvBridge()
        #检查消息类型
        self.createSub(self.get_parameter('image_topic').value)

        # ZeroMQ 初始化
        ctx = zmq.Context()
        self._socket = ctx.socket(zmq.PUSH)  # 发布者
        self._socket.bind(self.get_parameter('socket_address').value)
        
        self._socket_receive = ctx.socket(zmq.PULL)  # 响应者
        self._socket_receive.bind(self.get_parameter('socket_receive_address').value)

        self._image_publishers = {}
        self._shared_memory = None
        self._shm_size = 0  # 共享内存大小
        self._shm_name = None  # 共享内存名称

        self.lock = threading.Lock()  # 线程安全锁

        # 启动共享内存监听线程
        threading.Thread(target=self.socket_image_callback, daemon=True).start()
    def createSub(self,topic_name:str):
        self.get_topic_names_and_types()
        topic_types = self.get_topic_names_and_types()
        topic_type=None
        for topic in topic_types:
            if topic[0] == topic_name:
                topic_type=topic[1][0]
                #粉色打印
                print(f"\033[95m 桥接Topic: {topic_name}, Type: {topic_type}\033[0m")
                break
        if topic_type is None:
            raise ValueError(f"Topic {topic_name} not found")
        if topic_type == 'sensor_msgs/msg/Image':
            self.image_subscriber = self.create_subscription(
                Image,
                topic_name,
                # self.ros_image_callback,
                lambda msg:[
                    self.image_callback(self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')),
                ],
                10)
        elif topic_type == 'sensor_msgs/msg/CompressedImage':
            self.image_subscriber = self.create_subscription(
                CompressedImage,
                topic_name,
                lambda msg: [
                    #解压缩图像
                    self.image_callback(self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')),
                ],
                10)
    def destroy_node(self):
        if self._shared_memory is not None:
            self._shared_memory.close()
            self._shared_memory.unlink()
        super().destroy_node()
        print("ImageBridgeNode destroyed")

    def image_callback(self, image: np.ndarray):
        """ 处理 ROS2 传入的图像，并写入共享内存 """
        # cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img_shape = image.shape
        img_size = image.nbytes


        # 仅当图像大小变化时或者共享内存不可用时候
        if self._shm_name is None or self._shm_size != img_size or not os.path.exists(f"/dev/shm/{self._shm_name}"):
        #因为shm库的bug,在接收端关闭时候会释放共享内存,这里需要检查是否存在
            if self._shared_memory:
                try:
                    self._shared_memory.close()
                    self._shared_memory.unlink()
                except  Exception as e:
                    print(f"关闭共享内存失败: {e}")
            self._shared_memory = shm.SharedMemory(create=True, size=img_size)
            self._shm_name = self._shared_memory.name
            self._shm_size = img_size
            print(f"创建共享内存: {self._shm_name}, 大小: {img_size} bytes")
            # 直接映射数据到共享内存（零拷贝）
        timestamp = time.time()
        np_array = np.ndarray(img_shape, dtype=np.uint8, buffer=self._shared_memory.buf)
        np_array[:] = image  # 直接引用，不拷贝

        # 发送共享内存信息
        self._socket.send_json({
            "shm_key": self._shm_name,
            "shape": img_shape,
            "dtype": str(image.dtype),
            "timestamp": timestamp,
        })

    def socket_image_callback(self):
        """ 监听外部请求，从共享内存读取图像并发布到 ROS2 话题 """
        while True:
            message = self._socket_receive.recv_json()
            shm_key = message['shm_key']
            shape = tuple(message['shape'])
            dtype = np.dtype(message['dtype'])
            topic = message['topic']
            # 读取共享内存（零拷贝）
            shm_image = shm.SharedMemory(name=shm_key)
            image = np.ndarray(shape, dtype=dtype, buffer=shm_image.buf)

            # 如果 topic 还没有对应的发布者，则创建
            if topic not in self._image_publishers:
                self._image_publishers[topic] = self.create_publisher(Image, topic, 10)

            # 直接转换并发布（避免额外拷贝）
            image_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            self._image_publishers[topic].publish(image_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImageBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import zmq
import numpy as np
import multiprocessing.shared_memory as shm
import threading

class ImageBridgeNode(Node):
    def __init__(self):
        super().__init__('image_bridge_node')
        print("ImageBridgeNode initialized")
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('socket_address', 'tcp://localhost:5555')
        self.declare_parameter('socket_receive_address', 'tcp://localhost:5556')

        self.bridge = CvBridge()

        self.image_subscriber = self.create_subscription(
            Image,
            self.get_parameter('image_topic').value,
            self.ros_image_callback,
            10)

        # ZeroMQ 初始化
        ctx = zmq.Context()
        self._socket = ctx.socket(zmq.PUB)  # 发布者
        self._socket.bind(self.get_parameter('socket_address').value)
        
        self._socket_receive = ctx.socket(zmq.REP)  # 响应者
        self._socket_receive.bind(self.get_parameter('socket_receive_address').value)

        self._image_publishers = {}
        self._shared_memory = None
        self._shm_size = 0  # 共享内存大小
        self._shm_name = None  # 共享内存名称

        self.lock = threading.Lock()  # 线程安全锁

        # 启动共享内存监听线程
        threading.Thread(target=self.socket_image_callback, daemon=True).start()

    def destroy_node(self):
        if self._shared_memory is not None:
            self._shared_memory.close()
            self._shared_memory.unlink()
        super().destroy_node()

    def ros_image_callback(self, msg: Image):
        """ 处理 ROS2 传入的图像，并写入共享内存 """
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img_shape = cv_image.shape
        img_size = cv_image.nbytes

    
            # 仅当图像大小变化时重新分配共享内存
        if self._shm_name is None or self._shm_size != img_size:
            if self._shared_memory:
                self._shared_memory.close()
                self._shared_memory.unlink()
            self._shared_memory = shm.SharedMemory(create=True, size=img_size)
            self._shm_name = self._shared_memory.name
            self._shm_size = img_size

            # 直接映射数据到共享内存（零拷贝）
            np_array = np.ndarray(img_shape, dtype=np.uint8, buffer=self._shared_memory.buf)
            np_array[:] = cv_image  # 直接引用，不拷贝

        # 发送共享内存信息
        self._socket.send_json({
            "shm_key": self._shm_name,
            "shape": img_shape,
            "dtype": str(cv_image.dtype)
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

            # 响应 ZMQ 请求，避免阻塞
            self._socket_receive.send_json({"status": "success"})

def main(args=None):
    rclpy.init(args=args)
    node = ImageBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

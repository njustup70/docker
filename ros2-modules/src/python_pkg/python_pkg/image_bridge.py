import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import zmq
import numpy as np
import multiprocessing.shared_memory as shm
import time
import threading
class ImageBridgeNode_t(Node):
    def __init__(self):
        super().__init__('image_bridge_node')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('socket_address', 'tcp://localhost:5555')
        self.declare_parameter('socket_receive_address', 'tcp://localhost:5556')
        self.bridge = CvBridge()
        self.image_subscriber = self.create_subscription(
            Image,
            self.get_parameter('image_topic').value,
            self.ros_image_callback,
            10)
        ctx = zmq.Context()
        self._socket = ctx.socket(zmq.PUB)  # 发布者
        self._socket.bind(self.get_parameter('socket_address').value)
        self._shared_memory = None
        self._socket_receive = ctx.socket(zmq.REP)  # 响应者
        self._socket_receive.bind(self.get_parameter('socket_receive_address').value)
        self._image_publishers={}
        #开启监听线程
        threading.Thread(target=self.socket_iamge_callback, daemon=True).start()
    def destroy_node(self):
        if self._shared_memory is not None:
            self._shared_memory.close()
            self._shared_memory.unlink()
        super().destroy_node()
    def ros_image_callback(self, msg:Image):
        # 将ROS2图像消息转换为OpenCV格式并发给共享内存
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self._shared_memory is None:
            self._shared_memory = shm.SharedMemory(create=True, size=cv_image.nbytes)
        self._shared_memory.buf[:cv_image.nbytes] = cv_image.tobytes()
        self._socket.send_json({"shm_key": self._shared_memory.name})
    def socket_iamge_callback(self):
        # 接收图像
        message = self._socket_receive.recv_json()
        shm_key = message['shm_key']
        #读取共享内存
        shm_image = shm.SharedMemory(name=shm_key)
        # 读取共享内存中的数据
        image = np.ndarray((480, 640, 3), dtype=np.uint8, buffer=shm_image.buf)
        topic=message['topic']
        if topic not in self._publishers:
            self._image_publishers[topic] = self.create_publisher(Image, topic, 10)
        # 发布图像
        image_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        self._image_publishers[topic].publish(image_msg)
def main(args=None):
    rclpy.init(args=args)
    image_bridge_node = ImageBridgeNode_t()
    rclpy.spin(image_bridge_node)
    image_bridge_node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
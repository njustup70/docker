# pub.py
import zmq
import time

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5555")  # 绑定到 5555 端口

time.sleep(1)  # 等待 SUB 连接

for i in range(10):
    message = f"hello {i}"
    print("发送:", message)
    socket.send_string(message)
    time.sleep(1)  # 控制发送间隔

import socket
import asyncio
import select
class AsyncTCPServer:
    def __init__(self, ip: str, port: int, callback=None):
        """初始化异步服务器"""
        self._ip = ip
        self._port = port
        self._callback = callback
        self._raw_data = b''
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._conn = None  # 用于与客户端通信的连接 socket
        self._start = False
        
        asyncio.create_task(self._setup())  # 异步初始化

    def __del__(self):
        self._socket.close()
        if self._conn:
            self._conn.close()

    async def _setup(self):
        """异步处理服务器初始化连接"""
        self._socket.bind((self._ip, self._port))
        self._socket.listen(5)
        while True:
            if not self._start:
                try:
                    # 服务器端
                  
                    self._conn, self._addr = await asyncio.get_running_loop().sock_accept(self._socket)
                    print("Connected by", self._addr)
                    self._start = True
                except Exception as e:
                    print("socket error:", e)
            await asyncio.sleep(0.5)

    def startListening(self,callback=None):
        """开始监听 socket 数据"""
        if callback!=None:
            self._callback = callback
        asyncio.create_task(self.__read())

    async def __read(self):
        """读取 socket 数据并调用回调函数"""
        while True:
            if not self._start:
                await asyncio.sleep(0.01)
                continue
            #如果有数据才读取
            readable, _, _ = select.select([self._conn], [], [], 0.1)
            if readable:

                data = self._conn.recv(1024)
                if(data==b''):
                    print("Client disconnected")
                    self._start=False
                    self._conn.close()
                    self._conn=None
                    continue
                self._raw_data = data
                if self._callback:
                    self._callback(data)
            await asyncio.sleep(0.01)
    def write(self, input_data: bytes) -> None:
        """向 socket 写入数据"""
        if not self._start:
            print("No connection")
            return
        self._conn.sendall(input_data)

class AsyncTCPClient:
    def __init__(self, ip: str, port: int):
        """初始化异步客户端"""
        self._ip = ip
        self._port = port
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._start = False
        
        asyncio.create_task(self._setup())  # 异步初始化

    def __del__(self):
        self._socket.close()

    async def _setup(self):
        """异步处理客户端连接"""
        while True:
            if not self._start:
                try:
                    #先清空socket
                    self._socket.close()
                    self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    await asyncio.get_running_loop().sock_connect(self._socket, (self._ip, self._port))
                    print(f"客户端成功连接到 {self._ip}:{self._port}")
                    self._start = True
                except Exception as e:
                    print("socket error:", e)
            await asyncio.sleep(0.5)

    def write(self, input_data: bytes) -> None:
        """向服务器发送数据"""
        if not self._start:
            print("No connection")
            return
        self._socket.sendall(input_data)
    def startListening(self,callback=None):
        """开始监听 socket 数据"""
        if callback:
            self._callback = callback
        asyncio.create_task(self.__read())
    async def __read(self):
        """接收服务器响应"""
        while True:
            if not self._start:
                await asyncio.sleep(0.01)
                continue
            readable, _, _ = select.select([self._socket], [], [], 0.1)
            if readable:
                data = self._socket.recv(1024)
                if data == b'':  # 如果服务器断开连接
                    print("Connection closed by server.")
                    self._start = False
                    continue
                self._raw_data = data
                if self._callback:
                    self._callback(data)
            await asyncio.sleep(0.01)
async def main():
    """测试函数，客户端和服务器双向通信"""
    # 创建客户端和服务器实例
    client = AsyncTCPClient("127.0.0.1", 11451)
    client.startListening(callback=lambda data: (
                          print(f"Client received: {data}"),
                          client.write(data)))
    server = AsyncTCPServer("127.0.0.1", 15151, callback=lambda data: server.write(data))
    server.startListening()

    while True:
        # 客户端输入数据并发送
        data = await asyncio.to_thread(input, "Please input data: ")
        client.write(data.encode())  # 客户端发送数据
        await asyncio.sleep(0.05)

if __name__ == "__main__":
    asyncio.run(main())

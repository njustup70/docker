import socket
import socketserver
import asyncio

class   AsyncTCPSock_t:
    def __init__(self,ip:str,port:int,callback=None,type="server"):
        """_summary_ 初始化异步socket

        Args:
            ip (str): _description_ ip地址
            port (int): _description_ 端口号
            callback (_type_, optional): _description_. Defaults to None. 数据到来时的回调函数
        """
        self._ip=ip
        self._port=port
        self._callback=callback
        self._raw_data=b''
        self._socket=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self._type=type
        self._start=False
        #连接服务器
        asyncio.create_task(self._setup())  # 异步初始化

    def __del__(self):
        self._socket.close()
    async def _setup(self):
        while not self._start:
            """异步处理服务器和客户端的初始化连接"""
            try:
                if self._type == "server":
                    self._socket.bind((self._ip, self._port))
                    self._socket.listen(5)
                    self._conn, self._addr = await asyncio.get_running_loop().sock_accept(self._socket)
                    print("Connected by", self._addr)
                    self._start=True
                else:
                    await asyncio.get_running_loop().sock_connect(self._socket, (self._ip, self._port))
                    print(f"客户端成功连接到 {self._ip}:{self._port}")
                    self._start=True
            except Exception as e:
                print("socket error:", e)
            await asyncio.sleep(0.5)
        # def startListening(self,callback=None,wait_time=0.01) -> None:
        
    def startListening(self,callback=None) -> None:
        """_summary_ 开始监听socket数据,启动read协程

        Args:
            callback (_type_, optional): _description_. Defaults to None. socket数据到来时的回调函数
        """
        if(self._type!="server"):
            print("startListening is only for server")
            return
        asyncio.create_task( 
            self.__read()
        )
    async def __read(self):
        """_summary_ 读取socket数据并调用回调函数,同时将数据保存在raw_data中
        """
        while True:
            #检查有没有连接上
            if not self._start:
                await asyncio.sleep(0.01)
                continue
            await asyncio.sleep(0.01)
            data = self._conn.recv(1024)
            #保留最新的数据
            self._raw_data=data
            if self._callback:
                self._callback(data)

    def write(self,input_data:bytes)->None:
        """_summary_ 向socket写入数据(阻塞函数),数据类型为bytes

        Args:
            input_data (_type_): _description_ 待写入的数据
        """
        #检查有没有连接上
        if not self._start:
            print("No connection")
            return
        self._socket.sendall(input_data)

async def main():
    '''_summary_: 测试函数,从终端输入数据,然后发送客户端,采用异步编写
        在主机的11451端口启动服务器,在15151端口启动客户端
        服务端会把接收到的数据发送给客户端
    '''
    client=AsyncTCPSock_t("127.0.0.1",11451,type="client")
    server=AsyncTCPSock_t("127.0.0.1",15151,callback=lambda data:client.write(data))
    server.startListening()
    while True:
        data = await asyncio.to_thread( input,"Please input data:")
        client.write(data.encode())
        await asyncio.sleep(0.05)
if __name__ == "__main__":
    asyncio.run(main())
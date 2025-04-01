"""_summary_串口异步读写库
@Author: LiuXuanze(Elaina-rascal)
@Date: 2024-12-29
@Description 使用方法:
1.接受:AsyncSerial_t("COM2", 115200)创建一个串口对象,然后调用startListening()开始监听串口数据,
串口数据到来时会调用callback函数,如果不传入callback函数,则会调用write函数将数据原样返回
2.发送write函数用于向串口写入数据阻塞函数

例程:
serial = AsyncSerial_t("COM2", 115200)
serial.startListening(lambda data: serial.write(data))
"""
import serial
import asyncio

class AsyncSerial_t:
    
    def __init__(self, port, baudrate):
        """_summary_初始化异步串口

        Args:
            port (_type_): _description_ 串口号
            baudrate (_type_): _description_ 波特率
        """
        self._serial = serial.Serial(port, baudrate, timeout=0)
        self._raw_data = b''
    def __del__(self):
        self._serial.close()
    def startListening(self,callback=None,wait_time=0.01) -> None:
        """_summary_ 开始监听串口数据,启动read协程

        Args:
            callback (_type_, optional): _description_. Defaults to None. 串口数据到来时的回调函数
            wait_time (_type_, optional): _description_. Defaults to 0.01. 串口数据读取间隔,单位为秒
        """
        self._wait_time=wait_time
        if callback:
            self._callback=callback
        #启动read协程
        asyncio.create_task(self.__read())
    async def __read(self):
        """_summary_ 读取串口数据并调用回调函数,同时将数据保存在raw_data中
        """
        while True:
            await asyncio.sleep(self._wait_time)
            if self._serial.in_waiting > 0:
                data = self._serial.read(self._serial.in_waiting)
                #保留最新的数据
                self._raw_data=data
                if self._callback:
                    self._callback(data)

    def getRawData(self)->bytes:
        """_summary_ 获取串口数据

        Returns:
            bytes: _description_ 最新串口接受数据
        """
        return self._raw_data
    def write(self,input_data:bytes)->None:
        """_summary_ 向串口写入数据(阻塞函数),数据类型为bytes

        Args:
            input_data (_type_): _description_ 待写入的数据
        """
        self._serial.write(input_data)
    
async def main()->None:
    """_summary_ 测试函数,从终端输入数据,然后发送到串口,采用异步编写
    """
    serial = AsyncSerial_t("COM2", 115200)
    serial.startListening(lambda data: serial.write(data))
    while True:
        data = await asyncio.to_thread( input,"Please input data:")
        serial.write(data.encode())
        await asyncio.sleep(0.05)

if __name__ == '__main__':
    """_summary_ 主函数,调用main函数
    """
    asyncio.run(main())

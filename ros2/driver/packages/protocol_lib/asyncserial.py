import asyncio
import serial
import serial_asyncio

class SerialAsync_t(asyncio.Protocol):
    def __init__(self, port, baudrate, callback=None, reconnect_interval=2):
        self.port = port
        self.baudrate = baudrate
        self.callback = callback
        self.reconnect_interval = reconnect_interval

        self.transport = None
        self.loop = asyncio.get_event_loop()

    def connection_made(self, transport):
        self.transport = transport
        print(f"\033[92m[INFO] Serial connected: {self.port} @ {self.baudrate}\033[0m")

    def data_received(self, data):
        if self.callback:
            if asyncio.iscoroutinefunction(self.callback):
                self.loop.create_task(self.callback(data))
            else:
                self.callback(data)


    def connection_lost(self, exc):
        print(f"\033[91m[WARNING] Serial connection lost: {exc}\033[0m")
        self.transport = None
        # 自动重连
        self.loop.create_task(self.reconnect())

    def write(self, data: bytes):
        if self.transport:
            self.transport.write(data)
        else:
            print("\033[91m[WARNING] Cannot write, serial not connected.\033[0m")

    async def reconnect(self):
        while self.transport is None:
            try:
                await serial_asyncio.create_serial_connection(
                    self.loop,
                    lambda: self,
                    self.port,
                    baudrate=self.baudrate
                )
                
                print("\033[92m[INFO] Reconnected successfully.\033[0m")
                break
            except serial.SerialException as e:
                print(f"\033[91m[WARNING] Reconnect failed: {e}\033[0m")
                await asyncio.sleep(self.reconnect_interval)

    def start(self):
        # 主动执行连接
        self.loop.create_task(self.reconnect())

def main():
    port = '/dev/serial_sick'  # 替换为实际串口
    baudrate = 460800

    async def data_callback(data):
        print(f"Received data: {data.hex()}")

    serial_async = SerialAsync_t(port, baudrate, callback=data_callback)
    serial_async.start()

    try:
        asyncio.get_event_loop().run_forever()
    except KeyboardInterrupt:
        print("\033[93m[INFO] Stopping serial listener...\033[0m")
if __name__ == "__main__":
    main()
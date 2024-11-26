"""
    @Author: WenXiaomo(SummerWen-Lab)
    @Date: 2024-11-26
    @Description: NJUST-UP70自用上位机向下位机通信库
"""

import struct
import serial
from crcmod.predefined import mkCrcFun

class Protocol:
    """
        :instruction-build: 构造数据帧可以直接调用 construct_frame, 若知道类型(命令字)可以直接调用其对应的方法, 会直接返回构建好的数据帧
        :instruction-receive: 解析数据帧的逻辑是先验证是否合法(CRC-Check), 再将其拆解成由命令字(command)和数据段(data_bytes)组成的字典, 根据命令字拆解数据流, 最后返回特定类型的字典
    """
    def __init__(self):
        self.crc8 = mkCrcFun('crc-8')
        self.crc16 = mkCrcFun('crc-16')

    def construct_frame(self, command, data_bytes):
        """
            构造数据帧

            :param command: 命令字 (int, 2 字节)
            :param data_bytes: 数据段 (bytes)
            :return: 完整数据帧 (bytes)
        """
       
        frame_header = 0xA5  # 帧头

        # 计算数据流长度
        data_length = len(data_bytes)
        data_length_low = data_length & 0xFF
        data_length_high = (data_length >> 8) & 0xFF

        # 生成 CRC 校验值
        header_crc = self.crc8(bytes([frame_header, data_length_low, data_length_high]))
        data_crc = self.crc16(data_bytes)
        data_crc_low = data_crc & 0xFF
        data_crc_high = (data_crc >> 8) & 0xFF

        # 构建数据帧, 详细规则见README.md
        frame = [
            frame_header,
            data_length_low,
            data_length_high,
            header_crc,
            command & 0xFF,
            (command >> 8) & 0xFF,
            *data_bytes,
            data_crc_low,
            data_crc_high
        ]
        return bytes(frame)

    def parse_frame(self, frame):
        """
            解析原始数据帧

            :param frame: 接收到的数据帧 (bytes)
            :return: 解析结果字典(命令字+数据段)
        """

        # 检查帧头
        if frame[0] != 0xA5:
            raise ValueError("Invalid frame header")
        
        # 解析数据段长度
        data_length = frame[1] + (frame[2] << 8)

        # 校验帧头 CRC8
        header_crc = frame[3]
        if header_crc != self.crc8(frame[:3]):
            raise ValueError("Header CRC8 mismatch")

        # 解析命令字
        command = frame[4] + (frame[5] << 8)

        # 提取数据段
        data_bytes = frame[6:6 + data_length]

        # 校验数据段 CRC16
        received_crc = frame[6 + data_length] + (frame[7 + data_length] << 8)
        if received_crc != self.crc16(data_bytes):
            raise ValueError("Data CRC16 mismatch")

        return {
            "command": command,
            "data_bytes": data_bytes
        }

    def handle_command(self, command, data_bytes):
        """
            根据命令字调用对应的方法

            :param command: 命令字
            :param data_bytes: 数据段 (bytes)
            :return: 处理结果
        """
        method_name = f"handle_command_{hex(command)}"
        if not hasattr(self, method_name):
            raise NotImplementedError(f"No handler implemented for command: {hex(command)}")
        
        method = getattr(self, method_name)
        return method(data_bytes)

    # 解析处理 YOLO 框数据帧方法 (command = 0x2001)
    def handle_command_0x2001(self, data_bytes):
        """
            处理 YOLO 检测框数据包

            :param data_bytes: 数据段 (bytes)
            :return: 解析后的数据
        """
        class_id, center_x, center_y, width, length = struct.unpack('<Bffff', data_bytes)
        return {
            "class_id": class_id,
            "center_x": center_x,
            "center_y": center_y,
            "width": width,
            "length": length
        }
    
    # 构建 YOLO 检测框数据帧方法 (command = 0x2001)
    def build_command_0x2001(self, class_id, center_x, center_y, width, length):
        """
            构建 YOLO 检测框数据的数据帧

            :param class_id: 目标类别 (int, 1 字节)
            :param center_x: 中心点 x 坐标 (float, 4 字节)
            :param center_y: 中心点 y 坐标 (float, 4 字节)
            :param width: 检测框宽度 (float, 4 字节)
            :param length: 检测框高度 (float, 4 字节)
            :return: 构建完成的数据帧 (bytes)
        """
        command = 0x2001 
        data_bytes = struct.pack('<Bffff', class_id, center_x, center_y, width, length)
        return self.construct_frame(command, data_bytes)


    #
    # 解析处理 or 构建 各种类型数据帧的方法放在这里
    # 

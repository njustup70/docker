#!/usr/bin/env python
#coding=UTF-8

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

from tf_transformations import euler_from_quaternion

## node init
node: Node

def callback(data):
    #这个函数是tf中的,可以将四元数转成欧拉角
    (r,p,y) = euler_from_quaternion((data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w))
    #由于是弧度制，下面将其改成角度制看起来更方便
    # node.get_logger().info("Roll = %f, Pitch = %f, Yaw = %f",r*180/3.1415926,p*180/3.1415926,y*180/3.1415926)
    # node.get_logger().info("Roll = %f, Pitch = %f, Yaw = %f",r,p,y)
    print(f"Yaw = {y*180/3.1415926}") #Roll = {r}, Pitch = {p}, 
def get_imu():
    rclpy.init()
    global node
    node = Node("get_imu_rpy_node")
    node.create_subscription(Imu, "/hfi_imu",  callback, 10) #接受topic名称
    rclpy.spin(node)

if __name__ == '__main__':
    get_imu()

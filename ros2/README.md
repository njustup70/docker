<!--
 * @Author: Nagisa 2964793117@qq.com
 * @Date: 2024-11-14 22:46:41
 * @LastEditors: Nagisa 2964793117@qq.com
 * @LastEditTime: 2024-11-14 23:01:12
 * @FilePath: \docker\ros2\README.md
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
-->
<div align="center">
<h1>ros2子模块<h1>
</div>

## 版本与发布记录
~~Elaina_v0.1~~
## 模块介绍
- 该模块启动后给其他docker提供原始的传感器数据
- 本模块包括深度相机与mid360驱动(包括源码与自己修改的)
- 包括ros_bridge(发送ros2话题到ros1目前为x86支持)
- packages是驱动库(**勿动**)，更改源码仅更改ros2_ws中
## 使用教程(已经进入docker)
- 1.构建项目(构建目录不一样)
```bash
ros2/packages/ws_livox$ ./src/livox_ros_drivers/build.sh humble 
```
```bash
ros2_ws$ colcon build
```
- 2.source环境
```bash
ros2_ws$ source install/setup.bash
```
- 3.雷达驱动
```bash
ros2 launch my_mid360 mid360_bringup.launch.py 
```
- 4.intel深度相机驱动
```bash
ros2 launch my_realsense realsense_bringup.launch.py 

- 5.tf树驱动
```bash
ros2 launch my_tf_tree robot.launch.py
```
## 修改参数说明(针对ros2_ws下的功能包)
#### 均在功能包的config与launch目录中
### mid360包MID360_config.json:
- "host_net_info"及其下面的"cmd_data_ip"等的ip为给激光雷达网卡绑定的静态ip,默认值为192.168.1.50
- "lidar_configs"下面的ip为激光雷达的广播ip,为1+激光雷达序列号的最后两位,
手头的是192.168.1.199
### realsense包realsense_bringup.launch.py
- root_path:根目录

### 源码链接
|驱动包| 描述|
|---|---|
|[`livox_ros_driver2`](https://github.com/Livox-SDK/livox_ros_driver2)|mid360封装的ros驱动|
|[`Livox-SDK`](https://github.com/Livox-SDK/Livox-SDK2)|mid360底层驱动|
|[`ros-humble-realsense2-camera`](https://github.com/IntelRealSense/realsense-ros)|深度相机ros驱动|
|[`realsense_sdk2`](https://github.com/IntelRealSense/librealsense)|深度相机底层驱动|


[realsense_ros其他文档](https://dev.intelrealsense.com/docs/ros2-wrapper)
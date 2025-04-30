<!--
 * @Author: Nagisa 2964793117@qq.com
 * @Date: 2024-11-14 22:46:41
 * @LastEditors: Nagisa 2964793117@qq.com
 * @LastEditTime: 2024-11-24 19:29:49
 * @FilePath: \docker\ros2\README.md
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
-->
<div align="center">
<h1>ros2子模块<h1>
</div>

## 版本与发布记录
~~Elaina_v0.1~~

| 功能包                                 | 介绍         |
| -------------------------------------- | ------------ |
| [my_driver](./src/my_driver/README.md) | ros2硬件驱动 |
| [rc_bringup](src/rc_bringup/README.md) | 综合启动     |

## 模块介绍
- 该模块启动后给其他docker提供原始的传感器数据
- 本模块包括深度相机与mid360驱动(包括源码与自己修改的)
- 包括ros_bridge(发送ros2话题到ros1目前为x86支持)
- packages是驱动库(**勿动**)，更改源码仅更改ros2_ws中
## 使用教程(已经进入docker)
- 1.在第一次打开项目的时候先运行自定义udev规则(注意其中的sudo权限)
```bash
ros2-modules$ sudo ./udev_init.bash
```
- ~~1.构建项目(构建目录不一样)~~ 已经废弃
```bash
ros2/packages/ws_livox$ ./src/livox_ros_drivers/build.sh humble 
~/packages/orbbec_ws$ cd ~/packages/orbbec_ws && colcon build && source install/setup.bash
```
```bash
ros2_ws$ cd docker/ros2/ros2_ws && colcon build
```
- 1.安装udev orbbec与realsense需要
``` bash
cd OrbbecSDK/misc/scripts  #Orbbec脚本在packages底下
sudo chmod +x ./install_udev_rules.sh
sudo ./install_udev_rules.sh
sudo udevadm control --reload && sudo udevadm trigger
```
``` bash
cd librealsense
sudo chmod +x ./scripts/setup_udev_rules.sh 
./scripts/setup_udev_rules.sh 
```
- 2.source环境
```bash
ros2_ws$ source install/setup.bash
```
- 3.雷达驱动
```bash
ros2 launch my_driver mid360_bringup.launch.py 
```
- 4.intel深度相机驱动
```bash
ros2 launch my_driver realsense_bringup.launch.py 
```
- 5.orbbec深度相机驱动
```bash
ros2 launch my_driver orbbec_setup.launch.py 
```
- 6.sm200线性雷达提供点云数据，需要提前给ttyACM0权限
```bash
ros2 launch my_driver ms200_scan_view.launch.py 
```
## 修改参数说明(针对ros2_ws下的功能包)
#### 均在功能包的config与launch目录中
### mid360包MID360_config.json:
- "host_net_info"及其下面的"cmd_data_ip"等的ip为给激光雷达网卡绑定的静态ip,默认值为192.168.1.50
- "lidar_configs"下面的ip为激光雷达的广播ip,为1+激光雷达序列号的最后两位,
手头的是192.168.1.199
### realsense包realsense_bringup.launch.py
- root_path:根目录
### orbbec包nagisa_orbbec的orbbec_setup.launch.py
- 直接在该launch.py文件中添加参数设置,具体可以修改的参数详见 \
[`orbbec启动参数可选项`](https://github.com/orbbec/OrbbecSDK_ROS2?tab=readme-ov-file#launch-parameters)

### 注意事项
#### livox发的pointcloud2不是标准点云,多了偏移与line
![msg](../.github/images/pointcloudmsg.png)
### 源码链接
| 驱动包                                                                            | 描述                                           |
| --------------------------------------------------------------------------------- | ---------------------------------------------- |
| [`livox_ros_driver2`](https://github.com/Livox-SDK/livox_ros_driver2)             | mid360封装的ros驱动                            |
| [`Livox-SDK`](https://github.com/Livox-SDK/Livox-SDK2)                            | mid360底层驱动                                 |
| [`ros-humble-realsense2-camera`](https://github.com/IntelRealSense/realsense-ros) | 深度相机ros驱动                                |
| [`realsense_sdk2`](https://github.com/IntelRealSense/librealsense)                | 深度相机底层驱动                               |
| [`OrbbecSDK_ROS2`](https://github.com/orbbec/OrbbecSDK_ROS2)                      | Orbbec深度相机ROS2驱动(底层与ros2驱动放在一起) |
| [`ms200`](packages/ms200_ros/README.md)                                           | 奥比中光线性雷达                               |

[realsense_ros其他文档](https://dev.intelrealsense.com/docs/ros2-wrapper)
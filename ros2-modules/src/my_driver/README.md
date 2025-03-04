# MY_DRIVER

硬件驱动的整合包

## mid360 [仓库链接](https://github.com/Livox-SDK/livox_ros_driver2)

启动命令:
```bash
ros2 launch my_driver mid360_bringup.launch.py 
```
配置说明:
- launch调用同功能包config文件夹中的[MID360_config.json](./config/MID360_config.json)
- 下面的ip指的是雷达广播码
![alt text](image.png)
![pic](doc/image1.png)

- 下面的ip指的是网口,需要调整`以太网中的ip`
![pic](doc/image2.png)
![pic](doc/image3.png)
![pic](doc/image4.png)
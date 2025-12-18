# up70_hik（自动编译，手动 launch）

目标：不改动原有 Docker 配置，新增一套专门给你现在的 `ros2_driver`（含 `hik_camera_ros2` + `spear_docking`）用的 Docker 启动方式：
- **容器启动时自动 `colcon build`**
- **你手动执行 `ros2 launch ...`**

## 0. 前提
- 这个目录下存在 `MVS/`（从宿主机 `/opt/MVS` 拷贝过来，用于构建镜像内的 `/opt/MVS`）
- 你的工作区在主机：`~/up70/src/ros2_driver`

## 1. 一键启动容器（会自动编译）
在主机执行：
```bash
cd ~/up70/src/docker/ros2/up70_hik
./run.bash up
```

看编译日志：
```bash
./run.bash logs
```

## 2. 进入容器（手动 launch）
```bash
./run.bash shell
```

容器内常用命令（手动）：
```bash
cd /home/Elaina/ros2_driver
source /opt/ros/humble/setup.bash
source install/setup.bash

# 启动海康相机
ros2 launch my_driver hik_camera_ros2.launch.py serial_number:=你的SN use_rviz:=true

# 启动 PnP
ros2 launch spear_docking pnp.launch.py
```

## 3. 停止容器
```bash
./run.bash down
```

## 4. 常见问题
- 如果 `hik_camera_ros2` 编译报错找不到 `/opt/MVS`：说明镜像内没带上 SDK（检查 `Dockerfile` 里的 `COPY ./MVS /opt/MVS`，以及本目录下是否真的有 `MVS/`）。
- 如果 RViz/GUI 不显示：通常需要主机执行一次 `xhost +local:docker` 或 `xhost +local:`（看你系统的 X11 配置）。

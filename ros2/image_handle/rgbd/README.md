# RGB-D 主逻辑容器环境

在 `up70docker/rc2027:driverLogic` 上补齐 Gemini 336L / RGB-D 依赖，仅支持 **amd64、Humble、Python 3.10**。识别源码、模型与主逻辑适配器沿用主工程，不在这里重复打包；无需 CUDA、Torch 或 ArUco。

## 1. 构建镜像

从本仓库根目录执行，也可以从其他目录传入脚本路径：

```bash
bash ros2/image_handle/rgbd/build.bash
```

默认生成本地 `up70docker/rc2027:driverLogic-rgbd`，不会推送。`--github-action` 同样只构建；`--push` 才发布。可用 `IMAGE_REPO`、`IMAGE_TAG` 改输出镜像名。目录内所有文件须一起保留。

本机若使用监听 `127.0.0.1` 的代理，可在命令前加 `BUILD_NETWORK=host`，转发已有的 `HTTP_PROXY` / `HTTPS_PROXY` / `NO_PROXY`，只影响构建、不写入镜像。

PR 自动构建和自检；合并 `master` 后才自动发布。基础镜像摘要已固定；Orbbec 固定 2.9.3，Python 依赖版本见 `requirements.txt`。基础镜像的 SciPy 1.8 不支持 NumPy 1.26，因此一并固定 SciPy 1.14.1，供主逻辑继续使用。

固定基础镜像已含 Orbbec 2.9.3、Zenoh、ROS 消息、编译工具和 USB 库，构建时逐项核对后复用；本层只补齐 Python 依赖，不更新既有 ROS 系统包。
同时固定 pip/setuptools/wheel，避免旧构建工具将交接包误装成 `UNKNOWN` 空包。

## 2. 宿主机 USB 权限

在 **Linux 宿主机**执行一次，然后重新插拔相机：

```bash
sudo bash ros2/image_handle/rgbd/install_udev.bash
```

规则取自 Orbbec ROS 驱动 2.9.3 的 `share/orbbec_camera/udev/99-obsensor-libusb.rules`，与镜像内版本构建时逐字核对；使用厂商原始权限规则。不同的旧规则会先备份。只安装到容器内不能替代宿主机权限配置。

## 3. 主逻辑使用

将主逻辑 `.devcontainer/docker-compose.yml` 中 `driverLogic-service` 的镜像改为：

```yaml
image: up70docker/rc2027:driverLogic-rgbd
```

其他配置沿用原文件：Elaina 用户、`/dev` 映射、host 网络/IPC、Zenoh 环境和路由入口。从主逻辑仓库根目录重建容器，不删除工作空间卷：

```bash
docker compose -f .devcontainer/docker-compose.yml up -d --force-recreate driverLogic-service
docker compose -f .devcontainer/docker-compose.yml exec driverLogic-service bash
```

在容器内检查：

```bash
source /opt/ros/humble/setup.bash
python3 /opt/up70-rgbd/verify_env.py
printenv RMW_IMPLEMENTATION ROS_DOMAIN_ID
```

应打印 `RGB-D environment OK` 和实际依赖路径；RMW 应沿用 `rmw_zenoh_cpp`，domain 与主逻辑其他终端一致。镜像本身不启动相机、路由、串口或底盘。

识别源码编译和模型安装沿用主工程的交接说明。加载其 ROS overlay 后启动：

```bash
ros2 launch vision_depth rgbd_side_viewer.launch.py transport:=inherit headless:=true
```

只启动一份 Gemini 驱动，不同时启动旧相机入口。环境检查通过不代表真机出流、空间精度或稳定性验收。

已在该镜像中验证交接包安装、两个 ROS 包编译、模型 CPU 推理，以及 Zenoh 进程间候选收发、空数组撤销和断流过期。尚未在主逻辑现场容器中完成相机实测。

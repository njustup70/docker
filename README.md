<div align="center">
<h1>docker仓库</h1>
</div>

## 版本和发布记录
### 当前版本
~~Elaina_v0.1~~

## 仓库介绍
- 本仓库包括docker环境以及使用的教程
- ros2驱动环境包括驱动mid360,深度相机与ros1_bridge在启动后需要自己打开驱动(目前还是x64)
- yolo环境为yolov8
## 模块介绍
| 模块 | 说明|
| --- |---|
|[`ros2`](./ros2/README.md) |ros2的驱动包|
|[`yolo`](./yolo) |yolov8环境|
|[`rosbridge`](./rosbridge/)|ros桥接包|
## 依赖
- docker 
- docker-compose
- **nvidia docker进行时(这是最重要的)**
- 当dockercompose指定runtime为nvidia时候需要nvidia docker
![image](.github/images/2.png)

## nvidia的依赖安装([详细连接](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html))
 速通教程:
 - 1.添加仓库
```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```
 - 2.更新apt
```bash
sudo apt-get update
```
 - 3.下载软件包
```bash
sudo apt-get install -y nvidia-container-toolkit
```
- 4.配置容器运行时
```bash
sudo nvidia-ctk runtime configure --runtime=docker
```
- 5.重启守护进程
```bash
sudo systemctl restart docker
```
## 使用教程(以ros2为例)[(:xxxxx$代表执行指令的目录)]
### 如果没有.devcontainer目录按键盘上的ctrl+h启动隐藏目录
### 获得项目
1. git拉取
```bash
git clone https://github.com/njustup70/docker.git
```
2. 初始化git的子模块(在git的同级目录下)
```bash
git submodule init && git submodule update

```

### 用docker-compose启动
1. 先进入docker目录
```bash
:docker$ cd ros2/.devcontainer
```
### 
2. docker-compose启动docker文件
```bash
:ros2/.devcontainer$ docker-compose up -d #-d的参数表示在后台运行
```
3. 进入容器 
```bash
:ros2/.devcontainer$ ./exe.bash
```
或者
```bash
docker exe -it ros2driver-container bash
```
## 用devcontainer启动
####  注意不能与docker-compose共用，如果要启动开发环境模式则需要先在.devcontainer目录中停止运行容器
```bash
docker-compose down
```
- 先安装devcontainer插件<br>
![images](.github/images/5.png)
- vscode打开docker/ros2(该目录下有.devcontainer)<br>
![image](.github/images/01.png)<br>
- 正常来说右下角会有在容器中启动选项
![image](.github/images/4.png)
- 如果没有则按照下面方法操作
1. 先找到远程资源管理器
2. 选择新的开发容器
3. 在容器中打开当前文件夹或者选择打开文件夹并选择.devcontainer父级目录<br>
![image](.github/images/3.png)
## 坑

### 1.找不到/bin/bash
![](.github/images/6.png)
- 原因:/bin/bash打成了/bin/bash/<br>
![](.github/images/binbash.png)
### 2.clion在容器中打开路径出错
- 原因: devcontainer.json中的workspaceFolder没写成和docker-compose.yaml中的working_dir同样路径
### 3.本仓库只包含源码，需要自己colon或者其他安装驱动，详细内容看各个子模块
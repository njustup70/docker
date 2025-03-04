<div align="center">
<h1>rosbridge子模块<h1>
</div>

# 2025-3-4:已经将ros bridge移植到ros2driver里面。

## 此处的bridge不支持端口占用检测,不支持arm64

> **注意你看的模块已经过时**
 
## 模块介绍
该模块是ros2-ros1直接的话题桥接镜像,使用场景一般为:
在本机上开一个ros1的docker,再开启本docker,本docker为当前的主机提供桥接，但是不为局域网内其他机器提供桥接
## 使用说明
### 1. 给运行指令权限
```bash
PATH TO rosbridge$ sudo chmod +x ./run.bash
```
### 2.运行命令(可启动后不管)
```bash
PATH TO rosbridge$./run.bash
```

## 问题说明
### 1.需要在开启roscore的情况下才可以开启桥接
### 2.默认的桥接是桥接所有话题之后可能更改
### 3.如果启动桥接节点但是没有成功桥接可能是docker权限的问题,docker中的挂载的文件夹的用户组与用户不是1000 
### 4.使用docker-compose.yaml 中的command的镜像建立的使用运行一个任务
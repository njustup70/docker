## 将docker推送到arm与linux平台上
### 1.安装qemu激动对ARM架构支持
```bash
docker buildx create --use

sudo apt-get install -y qemu-user-static
```
然后让docker配置支持容器
```bash
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
```
### 2.重新构建并推送镜像
```bash
docker buildx build --platform linux/amd64,linux/arm64 -t elainasuki/ros:ros2-humble-full --push .
```
### [官方文档](https://docs.docker.com/build/building/multi-platform/)
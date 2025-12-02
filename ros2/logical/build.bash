#!/bin/bash

# 获取脚本所在目录
# 获取脚本所在目录的绝对路径
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PARENT_DIR=$(dirname "$SCRIPT_DIR")

echo "脚本目录: $SCRIPT_DIR"
echo "父目录: $PARENT_DIR"

# 设置默认 tag
TAG="ros2_logical"

# 从外部传入的 IMAGE_REPO（格式：ghcr.io/user/repo 或 docker.io/user/repo）
# IMAGE_REPO=${IMAGE_REPO:-elainasuki/rc2026}
IMAGE_REPO="elainasuki/rc2026"
# 组合完整镜像名
IMAGE="$IMAGE_REPO:$TAG"

# 获取工作流中的环境变量来决定是否支持 arm64
PLATFORMS="linux/amd64"  # 默认只支持 amd64 架构

if [[ "${BUILD_ARM64}" == "true" ]]; then
    PLATFORMS="$PLATFORMS,linux/arm64"   # 如果环境变量 BUILD_ARM64 为 true，则支持 arm64 架构
fi
# 这里用的是docker build .. content是上一级而不是当前目录
# 如果传入 --github-action 参数
if [[ "$1" == "--github-action" ]]; then
    echo "构建并推送镜像: $IMAGE"
    docker buildx build \
    --platform $PLATFORMS \
    -t "$IMAGE" \
    -f "$SCRIPT_DIR/Dockerfile" \
    --cache-from "type=registry,ref=$IMAGE" \
    --cache-to type=inline \
    "$PARENT_DIR" \
    --push
else
    echo "本地构建 $IMAGE"
    docker build -t "$IMAGE" -f "$SCRIPT_DIR/Dockerfile" "$PARENT_DIR"
fi
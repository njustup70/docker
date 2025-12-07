#!/bin/bash

# 获取脚本所在目录
SCRIPT_DIR=$(dirname "$0")
PARENT_DIR="${SCRIPT_DIR%/*}"

# 设置默认 tag
TAG="yolo-gpu"

# 从外部传入的 IMAGE_REPO（格式：ghcr.io/user/repo 或 docker.io/user/repo）
# IMAGE_REPO=${IMAGE_REPO:-yc-dlan/rc2026}
IMAGE_REPO="elainasuki/rc2026"

# 组合完整镜像名
IMAGE="$IMAGE_REPO:$TAG"

# 获取工作流中的环境变量来决定是否支持 arm64
PLATFORMS="linux/amd64"  # 默认只支持 amd64 架构

if [[ "${BUILD_ARM64}" == "true" ]]; then
    PLATFORMS="$PLATFORMS"   # 如果环境变量 BUILD_ARM64 为 true，则支持 arm64 架构
fi
# 这里用的是docker build .. content是上一级而不是当前目录
# 如果传入 --github-action 参数
if [[ "$1" == "--github-action" ]]; then
    echo "构建并推送镜像: $IMAGE"
    docker buildx build \
    --platform $PLATFORMS \
    -t "$IMAGE" \
    -f "$SCRIPT_DIR/Dockerfile" \
    "$PARENT_DIR" \
    --cache-from "type=registry,ref=$IMAGE" \
    --cache-to type=inline \
    --push
else
    echo "本地构建 $IMAGE"
    docker build -t "$IMAGE" "$PARENT_DIR"
fi
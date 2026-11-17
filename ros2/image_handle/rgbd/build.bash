#!/bin/bash

# 构建上下文和 Dockerfile 都位于脚本所在目录。
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
TAG="driverLogic-rgbd"
IMAGE_REPO="up70docker/rc2027"
IMAGE="$IMAGE_REPO:$TAG"
PLATFORMS="linux/amd64"

if [[ "${1:-}" == "--github-action" ]]; then
    echo "构建并推送镜像: $IMAGE"
    docker buildx build \
        --platform "$PLATFORMS" \
        -t "$IMAGE" \
        -f "$SCRIPT_DIR/Dockerfile" \
        "$SCRIPT_DIR" \
        --cache-from "type=registry,ref=$IMAGE" \
        --cache-to type=inline \
        --push
else
    echo "本地构建 $IMAGE"
    docker build -t "$IMAGE" -f "$SCRIPT_DIR/Dockerfile" "$SCRIPT_DIR"
fi

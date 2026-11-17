#!/usr/bin/env bash
# 构建上下文固定为脚本目录，调用者不需要先 cd。
set -euo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
IMAGE="${IMAGE_REPO:-up70docker/rc2027}:${IMAGE_TAG:-driverLogic-rgbd}"
MODE=${1:-}
if [[ $# -gt 1 ]]; then
    echo 'Usage: build.bash [--github-action|--push|--help]' >&2
    exit 2
fi
case "$MODE" in
    --help|-h)
        echo 'Usage: build.bash [--github-action|--push]'
        echo 'Default: build and load locally. --github-action: CI check, no push.'
        echo '--push: explicitly publish. IMAGE_REPO and IMAGE_TAG override the output name.'
        exit 0 ;;
    ''|--github-action|--push) ;;
    *) echo "Unknown argument: $MODE" >&2; exit 2 ;;
esac
if [[ ${BUILD_ARM64:-false} != false ]]; then
    echo 'This image supports linux/amd64 only; the pinned base has no arm64 image.' >&2
    exit 2
fi
args=(--platform linux/amd64 --file "$SCRIPT_DIR/Dockerfile" --tag "$IMAGE")
# 本地代理监听 127.0.0.1 时可显式使用 host 构建网络；代理不写入镜像。
case "${BUILD_NETWORK:-default}" in
    default) ;;
    host) args+=(--network host --build-arg HTTP_PROXY --build-arg HTTPS_PROXY --build-arg NO_PROXY) ;;
    *) echo 'BUILD_NETWORK must be default or host' >&2; exit 2 ;;
esac
if [[ "$MODE" == --push ]]; then
    args+=(--push)
else
    args+=(--load)
fi
docker buildx build "${args[@]}" "$SCRIPT_DIR"

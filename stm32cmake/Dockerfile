# 改用基于 Debian 的 Node.js 镜像（使用 glibc，兼容性更好）
FROM node:24.6.0-bullseye

# 设置非交互模式，避免 apt 安装时出现交互提示
ENV DEBIAN_FRONTEND=noninteractive

# 安装系统依赖
RUN apt-get update && apt-get install -y \
    git \
    ninja-build \
    openocd 
# 安装 xpm 和 arm-none-eabi-gcc 编译器
RUN npm install -g xpm@0.20.8 \
    && xpm install @xpack-dev-tools/arm-none-eabi-gcc@14.2.1-1.1.1 --global --verbose
#安装新版的cmake
RUN apt-get update && apt-get install -y software-properties-common \
    && wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null |  apt-key add - \
    && apt-add-repository 'deb https://apt.kitware.com/ubuntu/ bionic main' \
    && apt-get update && apt-get install -y cmake
#安装clangd20
RUN wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key| apt-key add - \
    && apt-add-repository "deb http://apt.llvm.org/bullseye/ llvm-toolchain-bullseye-20 main" \
    && apt-get update && apt-get install -y clangd-20 \
    && update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-20 100
ARG PKG_PATH="/root/.local/xPacks/@xpack-dev-tools/arm-none-eabi-gcc/14.2.1-1.1.1/.content"
# 设置编译器路径到环境变量
ENV PATH="${PATH}:${PKG_PATH}/bin"
ENV ARM_GCC_PATH="${PKG_PATH}/bin"
#把gdb python添加到环境变量
ENV PYTHONPATH="${PYTHONPATH}:${PKG_PATH}/share/gcc-14.2.1/python"
#把gdb-py3软链接为gdb
RUN ln -sf /root/.local/xPacks/@xpack-dev-tools/arm-none-eabi-gcc/14.2.1-1.1.1/.content/bin/arm-none-eabi-gdb-py3 \
    /root/.local/xPacks/@xpack-dev-tools/arm-none-eabi-gcc/14.2.1-1.1.1/.content/bin/arm-none-eabi-gdb

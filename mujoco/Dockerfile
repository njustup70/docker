# 使用轻量的纯 Ubuntu 镜像
FROM ubuntu:22.04

# 设置环境变量
ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    LANG=C.UTF-8

# 仅安装必要的系统库（MuJoCo 渲染和 Python 环境）
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    libgl1-mesa-glx \
    libosmesa6-dev \
    libglew-dev \
    libglfw3 \
    git \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# 升级 pip
RUN pip3 install --no-cache-dir --upgrade pip

# 安装 MuJoCo 及可视化（2026年最新版已自带大部分依赖）
RUN pip3 install --no-cache-dir mujoco mujoco-python-viewer

# 安装 MPC 核心方案 (do-mpc + CasADi + Ipopt)
RUN pip3 install --no-cache-dir "do-mpc[full]"

# 安装 JAX (用于以后的 RL 训练 MJX)，它会自动调用容器运行时的 CUDA
# 注意：这里安装 jaxlib 时会自动关联宿主机的 CUDA 版本
RUN pip3 install --no-cache-dir "jax[cuda12]" -f https://storage.googleapis.com/jax-releases/jax_cuda_releases.html
RUN pip3 install --no-cache-dir mujoco-mjx

WORKDIR /workspace

CMD ["/bin/bash"]
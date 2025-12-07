# Start FROM PyTorch image
FROM pytorch/pytorch:2.8.0-cuda12.8-cudnn9-runtime

# Set environment variables
# Avoid DDP error and suppress TensorFlow warnings
ENV PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PIP_NO_CACHE_DIR=1 \
    PIP_BREAK_SYSTEM_PACKAGES=1 \
    MKL_THREADING_LAYER=GNU \
    OMP_NUM_THREADS=1 \
    TF_CPP_MIN_LOG_LEVEL=3

ARG USERNAME=yc-dlan
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG GROUP_NAME=wheel

# 合并所有apt-get操作并清理缓存
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    sudo nautilus wget software-properties-common curl \
    gcc git zip unzip htop libgl1 libglib2.0-0 gnupg libsm6 && \
    # 添加fish仓库并安装
    add-apt-repository ppa:fish-shell/release-3 -y && \
    apt-get update && \
    apt-get install -y fish fzf && \
    # 清理
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# 创建用户和用户组并配置权限
RUN groupadd --gid $USER_GID ${GROUP_NAME} && \
    useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME && \
    echo "$USERNAME:password" | chpasswd && \
    usermod -aG ${GROUP_NAME} $USERNAME && \
    echo "%${GROUP_NAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    chown $USERNAME:$GROUP_NAME /home/$USERNAME/.bashrc 

# 创建存放源码的目录并设置权限
RUN mkdir -p /home/${USERNAME}/packages && \
    chown -R $USERNAME:$GROUP_NAME /home/${USERNAME}/packages 

# 切换到目标目录，拉取Ultralytics源码
WORKDIR /home/${USERNAME}/packages
# 拉取官方源码到当前目录（/home/yc-dlan/packages）
RUN git clone https://github.com/ultralytics/ultralytics.git 
WORKDIR /home/${USERNAME}/packages/ultralytics
# 修改配置文件（路径已变为当前目录）
RUN sed -i '/^\[http "https:\/\/github\.com\/"\]/,+1d' .git/config && \
    sed -i -e 's/"opencv-python/"opencv-python-headless/' pyproject.toml

# 下载官方模型到源码目录
ADD https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt .

# 安装Python依赖（使用当前目录的项目文件）
RUN pip install uv && \
    uv pip install --system -e "." albumentations faster-coco-eval wandb && \
    rm -rf tmp /root/.config/Ultralytics/persistent_cache.json

# 安装fish插件（切换到普通用户）
USER $USERNAME
RUN curl -L https://github.com/oh-my-fish/oh-my-fish/raw/master/bin/install | sudo tee install_omf > /dev/null && \
    fish install_omf --noninteractive && \
    fish -c "omf install bass" && \
    sudo rm install_omf 

# 复制fish配置
COPY packages/fish /home/${USERNAME}/.config/fish

# 处理OpenCV和其他依赖
USER root
RUN pip uninstall opencv-python opencv-python-headless -y 

USER $USERNAME
RUN pip install opencv-python imutils pyzmq roslibpy comet_ml

# 可选：设置默认工作目录为源码目录
WORKDIR /home/${USERNAME}

# Usage Examples -------------------------------------------------------------------------------------------------------
# Build and Push
# t=ultralytics/ultralytics:latest && sudo docker build -f docker/Dockerfile -t $t . && sudo docker push $t

# Pull and Run with access to all GPUs
# t=ultralytics/ultralytics:latest && sudo docker pull $t && sudo docker run -it --ipc=host --gpus all $t

# Pull and Run with access to GPUs 2 and 3 (inside container CUDA devices will appear as 0 and 1)
# t=ultralytics/ultralytics:latest && sudo docker pull $t && sudo docker run -it --ipc=host --gpus '"device=2,3"' $t

# Pull and Run with local directory access
# t=ultralytics/ultralytics:latest && sudo docker pull $t && sudo docker run -it --ipc=host --gpus all -v "$(pwd)"/shared/datasets:/datasets $t

# Kill all
# sudo docker kill $(sudo docker ps -q)

# Kill all image-based
# sudo docker kill $(sudo docker ps -qa --filter ancestor=ultralytics/ultralytics:latest)

# DockerHub tag update
# t=ultralytics/ultralytics:latest tnew=ultralytics/ultralytics:v6.2 && sudo docker pull $t && sudo docker tag $t $tnew && sudo docker push $tnew

# Clean up
# sudo docker system prune -a --volumes

# Update Ubuntu drivers
# https://www.maketecheasier.com/install-nvidia-drivers-ubuntu/

# DDP test
# python -m torch.distributed.run --nproc_per_node 2 --master_port 1 train.py --epochs 3

# GCP VM from Image
# docker.io/ultralytics/ultralytics:latest

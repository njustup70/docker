FROM ros:humble

# 定义参数
ARG USERNAME=Elaina
ARG USER_UID=1000
ARG USER_GID=$USER_UID

USER root

RUN apt-get update && apt-get install -y \
    sudo \
    vim \
    nautilus \
    v4l-utils \
    git-lfs \
    python3-pip \
    libgl1-mesa-glx \
    libzbar0 \
    && git lfs install --system \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN pip3 install --no-cache-dir torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu

RUN pip3 install --no-cache-dir \
    imutils \
    ultralytics \
    openvino \
    roslibpy \
    pyzmq \
    comet_ml \
    open3d \
    py-spy \
    pyzbar \
    "numpy<2.0" \
    pyserial \
    pyserial-asyncio

ENV PYTHONPATH="/home/${USERNAME}/yolo:/home/${USERNAME}/yolo/cv_lib"

RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "export PATH=\$HOME/.local/bin:\$PATH" >> /home/${USERNAME}/.bashrc \
    && mkdir -p /home/${USERNAME}/.config/fish \
    && echo "set -gx PATH \$HOME/.local/bin \$PATH" >> /home/${USERNAME}/.config/fish/config.fish \
    && chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}

USER ${USERNAME}
WORKDIR /home/${USERNAME}
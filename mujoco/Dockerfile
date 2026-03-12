FROM elainasuki/ros:ros2-humble-full-1209

ARG USERNAME=Elaina
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG ROS_DISTRO=humble

RUN if ! getent group $USER_GID > /dev/null; then \
    groupadd --gid $USER_GID $USERNAME; \
    fi \
    # 创建用户（仅当 UID 不存在时）
    && if ! id -u $USERNAME > /dev/null 2>&1; then \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi \
    # 给用户添加 sudo 权限（可选，方便操作）
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

ENV ROS_DISTRO=${ROS_DISTRO}
ENV HOME=/home/${USERNAME}

# 升级 pip
RUN pip3 install --no-cache-dir --upgrade pip

# 安装 MuJoCo 及可视化（2026年最新版已自带大部分依赖）
RUN pip3 install --no-cache-dir mujoco mujoco-python-viewer

RUN pip3 install --no-cache-dir do-mpc casadi numpy matplotlib
RUN pip uninstall numpy -y && pip install "numpy<2"
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "source ~/ros2_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo 'bass source ~/ros2_ws/install/setup.bash' >> /home/${USERNAME}/.config/fish/config.fish

WORKDIR /home/${USERNAME}

USER $USERNAME

CMD ["/bin/bash"]
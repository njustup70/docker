
FROM ghcr.io/sloretz/ros:humble-desktop-full
# 定义用户和用户组
ARG USERNAME=Elaina
ARG USER_UID=1000
ARG USER_GID=$USER_UID
#安装必要软件
# RUN apt-get update \
# && apt-get install -y nautilus 
# 创建用户组和用户
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    # 配置环境变量
    && echo "$USERNAME:'" | chpasswd \
    # 允许用户使用 sudo
    && usermod -aG sudo $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    # 使新用户的 `.bashrc` 文件生效
    && chown $USERNAME:$USERNAME /home/$USERNAME/.bashrc

#添加雷达驱动并设置雷达驱动环境变量 
USER $USERNAME
WORKDIR /home/$USERNAME
#引出雷达驱动so

# 删除 sudo 权限

RUN  echo 'source /opt/ros/humble/setup.bash' >> /home/$USERNAME/.bashrc\ 
    && echo 'source ~/docker/rosbridge/ros-humble-ros1-bridge/install/setup.bash' >> /home/$USERNAME/.bashrc
USER $USERNAME
CMD ["/bin/bash"]


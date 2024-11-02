FROM ultralytics/ultralytics:latest
# 定义用户和用户组
ARG USERNAME=Elaina
ARG USER_UID=1000
ARG USER_GID=$USER_UID
#安装必要软件
RUN apt-get update \
    && apt-get install -y  net-tools nautilus bash-completion gdb  sudo
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

# 删除 sudo 权限
USER root
RUN deluser $USERNAME sudo && sed -i "/$USERNAME ALL=(ALL) NOPASSWD:ALL/d" /etc/sudoers \
&& echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> /home/${USERNAME}/.bashrc \
USER $USERNAME
CMD ["/bin/bash"]

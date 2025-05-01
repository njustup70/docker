#FROM osrf/ros:noetic-desktop-full
FROM ghcr.io/sloretz/ros:noetic-desktop-full
# 定义用户和用户组
ARG USERNAME=Elaina
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG GROUP_NAME=wheel
ENV DEBIAN_FRONTEND=noninteractive
#安装必要软件
RUN apt-get update \
    && apt-get install -y  net-tools nautilus bash-completion clangd git gedit gdb nano software-properties-common wget curl vim
# nautilus-extension-gnome-terminal dbus-x11 libcanberra-gtk-module libcanberra-gtk3-module git-lfs
# 创建用户组和用户

RUN groupadd --gid $USER_GID ${GROUP_NAME} \
    && useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME \
    # 配置密码
    && echo "$USERNAME:password" | chpasswd \
    # 允许用户使用 sudo
    # && usermod -aG sudo $USERNAME \
    # && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    # 允许wheel组的用户使用 sudo
    && usermod -aG ${GROUP_NAME} $USERNAME \
    && echo "%${GROUP_NAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    # 使新用户的 `.bashrc` 文件生效
    && chown $USERNAME:$GROUP_NAME /home/$USERNAME/.bashrc
USER root
RUN  add-apt-repository ppa:fish-shell/release-3 -y \
    &&  apt-get update &&  apt-get install -y fish fzf 
COPY packages/fish /home/${USERNAME}/.config/fish
WORKDIR /home/${USERNAME}
USER ${USERNAME}
RUN mkdir packages && cd packages && git clone https://github.com/Livox-SDK/Livox-SDK.git && cd Livox-SDK/build \
    && cmake .. && make -j8 && sudo make install \
    && echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> /home/${USERNAME}/.bashrc 
#安装雷达ros驱动
RUN . /opt/ros/noetic/setup.sh \
    && cd packages && mkdir ws_livox && cd ws_livox && mkdir src && cd src \
    && git clone https://github.com/Livox-SDK/livox_ros_driver.git && cd .. && catkin_make \
    && echo 'source ~/packages/ws_livox/devel/setup.bash' >> /home/Elaina/.bashrc \
    && git config --global core.filemode false

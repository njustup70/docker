FROM ghcr.io/sloretz/ros:humble-desktop-full

# 定义构建参数
ARG ROS_DISTRO=humble
ARG USERNAME=Elaina
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG GROUP_NAME=wheel

# 第一阶段：系统设置和用户创建
RUN set -eux; \
    # 清理可能的已有用户/组
    if getent passwd 1000 > /dev/null; then \
    existing_user=$(getent passwd 1000 | cut -d: -f1); \
    userdel -r "$existing_user" 2>/dev/null || true; \
    fi; \
    if getent group 1000 > /dev/null; then \
    existing_group=$(getent group 1000 | cut -d: -f1); \
    if ! getent passwd | awk -F: '{print $1":"$4}' | grep -q ":1000$"; then \
    groupdel "$existing_group" 2>/dev/null || true; \
    fi; \
    fi; \
    # 创建新用户组和用户
    groupadd --gid $USER_GID ${GROUP_NAME} && \
    useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME && \
    echo "$USERNAME:password" | chpasswd && \
    usermod -aG ${GROUP_NAME} $USERNAME && \
    echo "%${GROUP_NAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    chown $USERNAME:$GROUP_NAME /home/$USERNAME/.bashrc

# 第二阶段：批量安装系统包和ROS包
# 将所有apt-get操作合并到单个RUN指令中
RUN apt-get update && apt-get install -y \
    # 基础工具
    libc-bin net-tools bash-completion \
    software-properties-common dbus-x11 \
    # 开发工具
    gdb gdbserver vim curl wget nano \
    nautilus nautilus-extension-gnome-terminal \
    # 网络和编译工具
    iputils-ping ninja-build ccache lld \
    # fish终端
    fish fzf \
    # 语言支持
    language-pack-zh-hans language-pack-zh-hans-base gettext \
    # 添加fish PPA并安装更新
    add-apt-repository ppa:fish-shell/release-3 -y && \
    apt-get update && apt-get install -y --only-upgrade fish && \
    # 清理apt缓存减少镜像大小
    rm -rf /var/lib/apt/lists/* && \
    # 安装oh-my-fish
    curl -s https://raw.githubusercontent.com/oh-my-fish/oh-my-fish/master/bin/install > /tmp/install.fish && \
    sudo -u $USERNAME fish /tmp/install.fish --noninteractive --yes && \
    rm /tmp/install.fish && \
    # 安装fish插件
    sudo -u $USERNAME fish -c "omf install bass" && \
    # 设置umask
    echo "umask 002" | tee -a /etc/profile /etc/bash.bashrc

# 第三阶段：安装雷达驱动和ROS驱动
RUN mkdir -p /home/$USERNAME/packages && \
    cd /home/$USERNAME/packages && \
    # 安装Livox SDK
    git clone https://github.com/Livox-SDK/Livox-SDK2.git && \
    cd Livox-SDK2 && mkdir build && cd build && \
    cmake .. && make -j$(nproc) && make install && \
    # 安装ROS驱动
    mkdir -p /home/$USERNAME/packages/ws_livox/src/ && \
    cd /home/$USERNAME/packages/ws_livox/src/ && \
    git clone https://github.com/Livox-SDK/livox_ros_driver2.git && \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH && \
    ./livox_ros_driver2/build.sh $ROS_DISTRO && \
    echo "source ~/packages/ws_livox/install/setup.bash" >> /home/$USERNAME/.bashrc

# 第四阶段：安装LLVM工具链
RUN wget -qO- https://apt.llvm.org/llvm.sh | bash -s -- 19 && \
    apt-get update && apt-get install -y clang-14 && \
    update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-14 100 && \
    update-alternatives --install /usr/bin/clang clang /usr/bin/clang-14 100 && \
    update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-19 100

# 第五阶段：复制配置文件和主题
COPY packages /temp
RUN cp /temp/gcc-11.mo /usr/share/locale/zh_CN/LC_MESSAGES/ && \
    apt-get install -y /temp/clitheme_2.0-beta3-1_all.deb && \
    sudo -u $USERNAME clitheme apply-theme /temp/39neko.clithemedef.txt --yes && \
    # 复制fish配置
    cp -r /temp/fish /home/$USERNAME/.config/ && \
    chown -R $USERNAME:$GROUP_NAME /home/$USERNAME/.config/fish && \
    # 清理临时文件
    rm -rf /temp

# 设置默认用户
USER $USERNAME
WORKDIR /home/$USERNAME

# 设置环境变量
ENV ROS_DISTRO=$ROS_DISTRO
RUN apt-get update && apt-get install -y \ 
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-rqt-tf-tree \
    ros-${ROS_DISTRO}-rmw-zenoh-cpp
CMD ["/bin/bash"]
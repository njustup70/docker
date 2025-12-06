FROM eprosima/vulcanexus:kilted-base

# 定义用户和用户组
ARG USERNAME=Elaina
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG GROUP_NAME=wheel
# 创建用户和用户组
RUN set -eux; \
    # 如果存在 UID 为 1000 的用户，先删除这个用户
    if getent passwd 1000 > /dev/null; then \
    existing_user=$(getent passwd 1000 | cut -d: -f1); \
    userdel -r "$existing_user" || true; \
    fi; \
    # 如果存在 GID 为 1000 的组，且已经没有用户使用这个组，则删除
    if getent group 1000 > /dev/null; then \
    existing_group=$(getent group 1000 | cut -d: -f1); \
    if ! getent passwd | awk -F: '{print $1":"$4}' | grep -q ":1000$"; then \
    groupdel "$existing_group"; \
    fi; \
    fi; \    
    groupadd --gid $USER_GID ${GROUP_NAME} \
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
#安装必要软件终端补全,包管理器,编辑器,文件管理器
RUN apt-get update \
    && apt-get install -y libc-bin net-tools nautilus bash-completion pip gedit wget nano software-properties-common nautilus-extension-gnome-terminal dbus-x11 vim curl
#安装编译工具,调试工具拓展终端
RUN apt-get update \
    && apt-get install -y gdb gdbserver \    
    #安装fish终端
    && add-apt-repository ppa:fish-shell/release-3 -y \
    && apt-get update && sudo apt-get install -y fish fzf 
#安装fish插件
USER $USERNAME
RUN curl https://raw.githubusercontent.com/oh-my-fish/oh-my-fish/master/bin/install > /home/${USERNAME}/install.fish \
    && fish /home/${USERNAME}/install.fish --noninteractive --yes && rm /home/${USERNAME}/install.fish && fish -c "omf install bass" 
USER root
COPY packages /temp
COPY packages/fish /home/${USERNAME}/.config/fish
RUN cp /temp/gcc-11.mo /usr/share/locale/zh_CN/LC_MESSAGES/ \
    && apt-get install -y /temp/clitheme_2.0-beta3-1_all.deb language-pack-zh-hans language-pack-zh-hans-base gettext  \ 
    && sudo -u ${USERNAME} clitheme apply-theme /temp/39neko.clithemedef.txt --yes \
    && rm -rf /temp

RUN apt-get update && apt-get install -y ros-kilted-foxglove-bridge ros-kilted-rmw-cyclonedds-cpp usbutils  iputils-ping  ninja-build ccache lld ros-kilted-rqt-tf-tree \
    && echo "umask 002" | sudo tee -a /etc/profile /etc/bash.bashrc
# 使用 root 用户
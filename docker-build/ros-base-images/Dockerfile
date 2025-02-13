#FROM osrf/ros:noetic-desktop-full
FROM ghcr.io/sloretz/ros:noetic-desktop-full
# 定义用户和用户组
ARG USERNAME=Elaina
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ENV DEBIAN_FRONTEND=noninteractive
#安装必要软件
RUN apt-get update \
    && apt-get install -y  net-tools nautilus bash-completion clangd git gedit gdb nano software-properties-common wget curl
# nautilus-extension-gnome-terminal dbus-x11 libcanberra-gtk-module libcanberra-gtk3-module git-lfs
# 创建用户组和用户

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    # 配置环境变量
    && echo "$USERNAME:'" | chpasswd \
    # 允许用户使用 sudo
    && usermod -aG sudo $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && echo 'source /opt/ros/noetic/setup.bash' >> /home/$USERNAME/.bashrc \
    # 使新用户的 `.bashrc` 文件生效
    && chown $USERNAME:$USERNAME /home/$USERNAME/.bashrc
USER $USERNAME
RUN  sudo add-apt-repository ppa:fish-shell/release-3 -y \
    && sudo apt-get update && sudo apt-get install -y fish fzf \
    && curl -L https://get.oh-my.fish | sudo tee install_omf > /dev/null \
    && fish install_omf --noninteractive \
    && fish -c "omf install bass" \
    && sudo rm install_omf 
COPY packages/fish /home/${USERNAME}/.config/fish
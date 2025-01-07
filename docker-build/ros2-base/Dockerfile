FROM ghcr.io/sloretz/ros:humble-desktop-full
# 定义用户和用户组
ARG USERNAME=Elaina
ARG USER_UID=1000
ARG USER_GID=$USER_UID
#安装必要软件
RUN apt-get update \
    && apt-get install -y  net-tools nautilus bash-completion thefuck pip gdb lldb gedit clang
COPY packages /temp
#安装优化软件
RUN cp /temp/gcc.mo /usr/share/locale/zh_CN/LC_MESSAGES/ \
    && apt-get install -y /temp/clitheme_2.0-beta3-1_all.deb \
    # && yes | clitheme apply-theme /temp/39neko.clithemedef.txt  \
    && apt-get update \ 
    && apt-get install -y nautilus-extension-gnome-terminal dbus-x11 fish gettext 
# 创建用户组和用户
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME \
    # 配置密码
    && echo "$USERNAME:password" | chpasswd \
    # 允许用户使用 sudo
    && usermod -aG sudo $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    # 使新用户的 `.bashrc` 文件生效
    && chown $USERNAME:$USERNAME /home/$USERNAME/.bashrc  
#安装杂鱼终端
RUN sudo -u ${USERNAME} clitheme apply-theme /temp/39neko.clithemedef.txt --yes \
    && echo 'eval $(thefuck --alias)'>> /home/${USERNAME}/.bashrc \
    && echo 'eval $(thefuck --alias FUCK)'>> /home/${USERNAME}/.bashrc \
    && rm -rf /temp
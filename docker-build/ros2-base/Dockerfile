FROM ghcr.io/sloretz/ros:humble-desktop-full
# 定义用户和用户组
ARG USERNAME=Elaina
ARG USER_UID=1000
ARG USER_GID=$USER_UID
# 创建用户和用户组
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME \
    # 配置密码
    && echo "$USERNAME:password" | chpasswd \
    # 允许用户使用 sudo
    && usermod -aG sudo $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    # 使新用户的 `.bashrc` 文件生效
    && chown $USERNAME:$USERNAME /home/$USERNAME/.bashrc  
#安装必要软件
RUN apt-get update \
    && apt-get install -y  net-tools nautilus bash-completion pip gdb lldb gedit clang wget nano software-properties-common \
    && wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key | sudo apt-key add - \
    && add-apt-repository "deb http://apt.llvm.org/focal/ llvm-toolchain-focal main" \
    && apt-get install clangd
COPY packages /temp
#安装优化软件
RUN cp /temp/gcc-11.mo /usr/share/locale/zh_CN/LC_MESSAGES/ \
    && apt-get install -y /temp/clitheme_2.0-beta3-1_all.deb language-pack-zh-hans language-pack-zh-hans-base thefuck\
    # && yes | clitheme apply-theme /temp/39neko.clithemedef.txt  \
    && apt-get update \ 
    && apt-get install -y nautilus-extension-gnome-terminal dbus-x11 gettext  \
    && add-apt-repository ppa:fish-shell/release-3 -y \
    && apt-get update && sudo apt-get install -y fish fzf
# 创建用户组和用户

USER $USERNAME
RUN curl -L https://get.oh-my.fish | sudo tee install_omf > /dev/null \
    && fish install_omf --noninteractive \
    && fish -c "omf install bass" \
    && sudo rm install_omf 
#安装杂鱼终端
USER root
COPY packages/fish /home/${USERNAME}/.config/fish
RUN sudo -u ${USERNAME} clitheme apply-theme /temp/39neko.clithemedef.txt --yes \
    && sudo -u ${USERNAME} pip3 install thefuck \
    && echo 'eval $(thefuck --alias)'>> /home/${USERNAME}/.bashrc \
    && echo 'eval $(thefuck --alias FUCK)'>> /home/${USERNAME}/.bashrc \
    && rm -rf /temp

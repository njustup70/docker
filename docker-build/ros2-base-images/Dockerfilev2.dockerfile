#这是humble v2版本的基础镜像,主要增加了通用的包与终端添加了用户
FROM ghcr.io/sloretz/ros:humble-desktop-full
# 定义用户和用户组
ARG USERNAME=Elaina
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG GROUP_NAME=wheel
# 创建用户和用户组
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
#安装必要软件终端补全,包管理器,编辑器,文件管理器
RUN apt-get update \
    && apt-get install -y libc-bin net-tools nautilus bash-completion pip gedit wget nano software-properties-common nautilus-extension-gnome-terminal dbus-x11 vim
#安装编译工具,调试工具拓展终端
RUN apt-get update \
    && apt-get install -y gdb gdbserver \    
    # &&apt-get install -y gdb lldb clang gdbserver \
    # && wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key | sudo apt-key add - \
    # && add-apt-repository "deb http://apt.llvm.org/focal/ llvm-toolchain-focal main" \
    # && apt-get install -y clangd \
    #安装fish终端
    && add-apt-repository ppa:fish-shell/release-3 -y \
    && apt-get update && sudo apt-get install -y fish fzf 
#安装fish插件
USER $USERNAME
RUN curl -L https://get.oh-my.fish | sudo tee install_omf > /dev/null \
    && fish install_omf --noninteractive \
    && fish -c "omf install bass" \
    && sudo rm install_omf 
USER root
COPY packages /temp
COPY packages/fish /home/${USERNAME}/.config/fish
RUN cp /temp/gcc-11.mo /usr/share/locale/zh_CN/LC_MESSAGES/ \
    && apt-get install -y /temp/clitheme_2.0-beta3-1_all.deb language-pack-zh-hans language-pack-zh-hans-base thefuck gettext  \ 
    && sudo -u ${USERNAME} clitheme apply-theme /temp/39neko.clithemedef.txt --yes \
    # && sudo -u ${USERNAME} pip3 install thefuck \
    # && echo 'eval $(thefuck --alias)'>> /home/${USERNAME}/.bashrc \
    # && echo 'eval $(thefuck --alias FUCK)'>> /home/${USERNAME}/.bashrc \
    && rm -rf /temp

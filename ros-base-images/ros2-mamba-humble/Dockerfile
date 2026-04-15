FROM mambaorg/micromamba:latest
USER root

# --- 参数与环境变量配置 ---
ARG USERNAME=Elaina
ARG USER_UID=1000
ARG USER_GID=1000

ENV MAMBA_ROOT_PREFIX=/opt/conda
ENV PATH=$MAMBA_ROOT_PREFIX/bin:$PATH
ENV ROS_DISTRO=humble
ENV CPLUS_INCLUDE_PATH=$MAMBA_ROOT_PREFIX/include
ENV LIBRARY_PATH=$MAMBA_ROOT_PREFIX/lib
ENV UV_SYSTEM_PYTHON=1
# 1. 安装系统工具与编译依赖
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo build-essential cmake git \ 
    pkg-config \
    && rm -rf /var/lib/apt/lists/*
# 2. 创建用户并配置权限
RUN groupadd --gid $USER_GID $USERNAME  \
    && useradd --uid $USER_UID --gid $USER_GID -m -s /usr/bin/fish $USERNAME\
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
# 3. 配置并安装 Micromamba 环境 (仅保留 ROS 相关和辅助工具)
# 删除了 c-compiler, cxx-compiler, cmake, git，改用系统级的
RUN printf "channels:\n  - robostack-humble\n  - conda-forge\n  - nodefaults\nchannel_priority: strict\n" > /root/.mambarc && \
    micromamba install -y -n base \
    ros-humble-ros-core \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-tf2-ros ros-humble-tf2-eigen ros-humble-tf2-geometry-msgs\
    colcon-common-extensions \
    colcon-mixin \
    ninja ccache mold fish uv gdb curl\
    && micromamba clean --all --yes
SHELL ["micromamba", "run", "-n", "base", "/bin/bash", "-c"]
# # 6. 使用 UV 安装 Python 分析工具
RUN uv pip  install  viztracer py-spy
#增加其他ROS包 
RUN micromamba install -y -n base \
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-${ROS_DISTRO}-rmw-zenoh-cpp
#安装fish
RUN curl -s https://raw.githubusercontent.com/oh-my-fish/oh-my-fish/master/bin/install > /tmp/install.fish && \
    sudo -u $USERNAME fish /tmp/install.fish --noninteractive --yes && \
    rm /tmp/install.fish && \
    # 安装fish插件
    sudo -u $USERNAME fish -c "omf install bass" && \
    # 设置umask
    echo "umask 002" | tee -a /etc/profile /etc/bash.bashrc
COPY --chown=$USER_UID:$USER_GID packages/fish /home/$USERNAME/.config/fish
# --- 运行时配置 ---
USER $USERNAME
WORKDIR /home/$USERNAME
RUN mkdir -p ~/.config/fish && \
    echo 'eval "$(micromamba shell hook --shell fish)"' > ~/.config/fish/config.fish && \
    echo 'micromamba activate base' >> ~/.config/fish/config.fish 
CMD ["/usr/bin/fish"]
FROM mambaorg/micromamba:latest
USER root

# --- 参数与环境变量配置 ---
ARG USERNAME=Elaina
ARG USER_UID=1000
ARG USER_GID=1000

ENV MAMBA_ROOT_PREFIX=/opt/conda
ENV PATH=$MAMBA_ROOT_PREFIX/bin:$PATH
ENV ROS_DISTRO=humble

# 1. 安装系统工具与编译依赖
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# 2. 创建用户并配置权限
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m -s /usr/bin/fish $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# 3. 配置并安装 Micromamba 环境 (ROS2 + 开发工具)
RUN printf "channels:\n  - robostack-humble\n  - conda-forge\n  - nodefaults\nchannel_priority: strict\n" > /root/.mambarc && \
    micromamba install -y -n base \
    ros-humble-ros-core \
    ros-humble-sensor-msgs \
    cmake git ninja ccache mold fish uv c-compiler cxx-compiler gdb\
    && micromamba clean --all --yes

# 4. 编译 Livox SDK2
WORKDIR /tmp/build_sdk
RUN git clone --depth 1 https://github.com/Livox-SDK/Livox-SDK2.git && \
    cd Livox-SDK2 && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 && \
    -DCMAKE_CXX_FLAGS="-include cstdint" && \
    make -j$(nproc) && make install && \
    rm -rf /tmp/build_sdk

# 5. 编译 livox_ros_driver2
WORKDIR /home/$USERNAME/ws_livox/src
RUN git clone --depth 1 https://github.com/Livox-SDK/livox_ros_driver2.git && \
    cd livox_ros_driver2 && \
    # 在 micromamba 环境下运行编译脚本
    eval "$(micromamba shell hook --shell bash)" && \
    micromamba activate base && \
    ./build.sh ROS2 && \
    rm -rf /home/$USERNAME/ws_livox/build &&\
    # 修改工作空间权限归属新用户
    chown -R $USER_UID:$USER_GID /home/$USERNAME/ws_livox

# # 6. 使用 UV 安装 Python 分析工具
# RUN eval "$(micromamba shell hook --shell bash)" && \
#     micromamba activate base && \
#     uv pip install viztracer py-spy

# # --- 运行时配置 ---
# USER $USERNAME
# WORKDIR /home/$USERNAME

# # 7. 配置 Fish Shell 交互环境
# RUN mkdir -p ~/.config/fish && \
#     { \
#     echo 'eval "$(micromamba shell hook --shell fish)"'; \
#     echo 'micromamba activate base'; \
#     echo 'if test -f /home/'$USERNAME'/ws_livox/install/setup.fish'; \
#     echo '    source /home/'$USERNAME'/ws_livox/install/setup.fish'; \
#     echo 'end'; \
#     } >> ~/.config/fish/config.fish

# CMD ["/usr/bin/fish"]
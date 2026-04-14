# --- 第一阶段：编译构建 (Builder) ---
FROM mambaorg/micromamba:latest AS builder
USER root

# 环境变量配置
ENV MAMBA_ROOT_PREFIX=/opt/conda
ENV PATH=$MAMBA_ROOT_PREFIX/bin:$PATH

# 1. 安装基础工具 (包含 Git 和编译工具)
RUN apt-get update && apt-get install -y --no-install-recommends \
    git build-essential cmake make \
    && rm -rf /var/lib/apt/lists/*

# 2. 配置并安装 Micromamba 环境 (包含所有 ROS 依赖)
RUN printf "channels:\n  - robostack-humble\n  - conda-forge\n  - nodefaults\nchannel_priority: strict\n" > /root/.mambarc && \
    micromamba install -y -n base \
    ros-humble-ros-core \
    ros-humble-sensor-msgs \
    cmake git ninja ccache mold uv \
    && micromamba clean --all --yes

# 3. 编译 Livox SDK2 (系统级库)
WORKDIR /home/build
RUN git clone --depth 1 https://github.com/Livox-SDK/Livox-SDK2.git && \
    cd Livox-SDK2 && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && make install

# 4. 编译 livox_ros_driver2 (使用官方脚本)
WORKDIR /home/build/ws_livox/src
RUN git clone --depth 1 https://github.com/Livox-SDK/livox_ros_driver2.git && \
    cd livox_ros_driver2 && \
    # 必须在 micromamba 环境下运行脚本
    eval "$(micromamba shell hook --shell bash)" && \
    micromamba activate base && \
    ./build.sh ROS2

# --- 第二阶段：最终开发与运行环境 ---
FROM mambaorg/micromamba:latest
USER root

ARG USERNAME=Elaina
ARG USER_UID=1000
ARG USER_GID=1000

# 1. 基础系统运行时工具
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo git \
    && rm -rf /var/lib/apt/lists/*

# 2. 创建用户并配置权限
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m -s /usr/bin/fish $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# 3. 核心资产拷贝：从 builder 拷贝整个 Conda 环境和 SDK 库
COPY --from=builder /opt/conda /opt/conda
COPY --from=builder /usr/local /usr/local
# 拷贝编译好的 ROS 工作空间
COPY --from=builder --chown=$USER_UID:$USER_GID /home/build/ws_livox /home/$USERNAME/ws_livox

# 4. 环境路径配置
ENV MAMBA_ROOT_PREFIX=/opt/conda
ENV PATH=/opt/conda/bin:$PATH
ENV ROS_DISTRO=humble

# 5. 使用 UV 安装 Python 工具 (在已拷贝的 base 环境中)
RUN eval "$(micromamba shell hook --shell bash)" && \
    micromamba activate base && \
    uv pip install viztracer py-spy

USER $USERNAME
WORKDIR /home/$USERNAME

# 6. 配置 Fish Shell 交互环境
RUN mkdir -p ~/.config/fish && \
    { \
    echo 'eval "$(micromamba shell hook --shell fish)"'; \
    echo 'micromamba activate base'; \
    echo 'if test -f /home/'$USERNAME'/ws_livox/install/setup.fish'; \
    echo '    source /home/'$USERNAME'/ws_livox/install/setup.fish'; \
    echo 'end'; \
    } >> ~/.config/fish/config.fish

CMD ["/usr/bin/fish"]
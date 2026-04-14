# --- 第一阶段：编译构建 (Builder) ---
FROM mambaorg/micromamba:latest AS builder
USER root

ARG USERNAME=Elaina
ARG USER_UID=1000
ARG USER_GID=1000

# 1. 基础环境配置
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME

# 2. 配置 Micromamba 频道
RUN printf "channels:\n  - robostack-humble\n  - conda-forge\n  - nodefaults\nchannel_priority: strict\n" > /root/.mambarc

# 3. 第一阶段只装最基本的编译依赖 (ros-core + colcon + 消息类型)
RUN micromamba install -y -n base \
    ros-humble-ros-core \
    ros-humble-sensor-msgs \
    ros-humble-colcon-common-extensions \
    cmake build-essential git \
    && micromamba clean --all --yes

# 4. 编译 Livox SDK2 (系统库)
WORKDIR /home/build
RUN git clone --depth 1 https://github.com/Livox-SDK/Livox-SDK2.git && \
    mkdir -p Livox-SDK2/build && cd Livox-SDK2/build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc) && make install

# 5. 编译 livox_ros_driver2 (ROS 功能包)
WORKDIR /home/$USERNAME/ws_livox/src
RUN git clone --depth 1 https://github.com/Livox-SDK/livox_ros_driver2.git && \
    cd .. && \
    eval "$(micromamba shell hook --shell bash)" && \
    micromamba activate base && \
    colcon build --rosdistro humble --cmake-args -DROS_EDITION=ROS2 --build-base /tmp/build --install-base /opt/ros_install

# --- 第二阶段：最终运行环境 ---
FROM mambaorg/micromamba:latest
USER root

ARG USERNAME=Elaina
ARG USER_UID=1000
ARG USER_GID=1000

# 1. 基础系统工具 (Runtime)
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo  fish vim curl git \
    && rm -rf /var/lib/apt/lists/*

# 2. 创建用户
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m -s /usr/bin/fish $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# 3. 环境变量先行 (直接将路径指向之后要拷贝的位置)
ENV MAMBA_ROOT_PREFIX=/opt/conda
ENV PATH=/home/$USERNAME/ws_livox/install/bin:/opt/conda/bin:$PATH
ENV ROS_DISTRO=humble
ENV PYTHONPATH=/opt/conda/lib/python3.10/site-packages:$PYTHONPATH
ENV PATH=/opt/conda/lib/ccache:$PATH
# 4. 从第一阶段拷贝：1.SDK库, 2.编译好的ROS驱动
COPY --from=builder /usr/local /usr/local
COPY --from=builder --chown=$USER_UID:$USER_GID /opt/ros_install /home/$USERNAME/ws_livox/install

# 5. 在第二阶段安装 ROS 核心、UV 以及你需要的开发工具
RUN printf "channels:\n  - robostack-humble\n  - conda-forge\n  - nodefaults\nchannel_priority: strict\n" > /opt/conda/.mambarc && \
    micromamba install -y -n base \
    ros-humble-ros-core \
    ros-humble-sensor-msgs \
    cmake ninja ccache mold uv \
    && micromamba clean --all --yes

# 6. 使用 UV 安装 Python 调试工具
RUN uv pip install viztracer py-spy

USER $USERNAME
WORKDIR /home/$USERNAME

# 7. 配置 Fish Shell 自动加载
RUN mkdir -p ~/.config/fish && \
    echo 'eval "$(micromamba shell hook --shell fish)"' >> ~/.config/fish/config.fish && \
    echo 'micromamba activate base' >> ~/.config/fish/config.fish && \
    echo 'test -f /home/'$USERNAME'/ws_livox/install/setup.fish; and source /home/'$USERNAME'/ws_livox/install/setup.fish' >> ~/.config/fish/config.fish

CMD ["/usr/bin/fish"]
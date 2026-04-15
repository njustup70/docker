FROM elainasuki/ros:ros2-humble-core-0415

ARG USERNAME=Elaina
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG ROS_DISTRO=humble
ENV HOME=/home/${USERNAME}
USER root
#用micromamba安装mujoco casadi和pinocchio
RUN micromamba install -y \
    pinocchio \
    casadi \
    && micromamba clean -afy
# 安装 MuJoCo 及相关 Python 包
RUN uv pip install --no-cache \
    mujoco \
    mujoco-python-viewer \
    do-mpc \
    matplotlib \
    foxglove-sdk \
    "numpy<2" 
WORKDIR  /home/${USERNAME}/packages
RUN git clone --depth 1 https://github.com/acados/acados.git /tmp/acados_source \
    && cd /tmp/acados_source \
    && git submodule update --recursive --init --depth 1 \
    && mkdir build && cd build \
    && cmake -DACADOS_WITH_QPOASES=ON \
    -DACADOS_WITH_HPIPM=ON \
    -DBUILD_SHARED_LIBS=ON \
    -DACADOS_UNIT_TESTS=OFF \
    -DACADOS_EXAMPLES=OFF \
    -DCMAKE_C_FLAGS="-Wno-error=incompatible-pointer-types" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/opt/acados .. \
    && make install -j$(nproc) \
    # 有些 acados 接口需要这个路径下有特定的元数据
    && cp -rf /tmp/acados_source/interfaces/acados_template /opt/acados/interfaces/ \
    && rm -rf /tmp/acados_source
# 2. 安装 Python 接口
# 注意：在 Docker 中建议直接使用完整路径
RUN uv pip install  /opt/acados/interfaces/acados_template\
    && mkdir -p /opt/acados/bin \
    && curl -L https://github.com/acados/tera_renderer/releases/download/v0.0.34/t_renderer-v0.0.34-linux -o /opt/acados/bin/t_renderer \
    && chmod +x /opt/acados/bin/t_renderer

# 3. 设置环境变量 (这是最关键的一步，必须在 ENV 中设置，否则仅在 RUN 时有效)
ENV ACADOS_SOURCE_DIR="/opt/acados"
ENV LD_LIBRARY_PATH="${ACADOS_SOURCE_DIR}/lib:${LD_LIBRARY_PATH}"
ENV PATH="/opt/acados/bin:${PATH}"

RUN uv pip install "numpy<2"
RUN echo "source ~/ros2_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo 'source ~/ros2_ws/install/setup.zsh' >> /home/${USERNAME}/.config/fish/config.fish

WORKDIR /home/${USERNAME}
CMD ["/bin/bash"]

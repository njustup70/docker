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
    casadi 
# 安装 MuJoCo 及相关 Python 包
RUN uv pip install --no-cache \
    mujoco \
    mujoco-python-viewer \
    do-mpc \
    matplotlib \
    foxglove-sdk \
    "numpy<2" 
WORKDIR  /home/${USERNAME}/packages
RUN git clone https://github.com/acados/acados.git /opt/acados \
    && cd /opt/acados \
    && git submodule update --recursive --init \
    && mkdir build && cd build \
    && cmake -DACADOS_WITH_QPOASES=ON \
    -DACADOS_WITH_HPIPM=ON \
    -DBUILD_SHARED_LIBS=ON \
    -DACADOS_UNIT_TESTS=OFF \
    -DACADOS_EXAMPLES=OFF \
    -DCMAKE_C_FLAGS="-Wno-error=incompatible-pointer-types" \
    -DCMAKE_BUILD_TYPE=Release .. \
    && make install -j$(nproc) \
    && rm -rf /opt/acados/build
# 2. 安装 Python 接口
# 注意：在 Docker 中建议直接使用完整路径
RUN uv pip install  /opt/acados/interfaces/acados_template\
    && mkdir -p /opt/acados/bin \
    && curl -L https://github.com/acados/tera_renderer/releases/download/v0.0.34/t_renderer-v0.0.34-linux -o /opt/acados/bin/t_renderer \
    && chmod +x /opt/acados/bin/t_renderer

# 3. 设置环境变量 (这是最关键的一步，必须在 ENV 中设置，否则仅在 RUN 时有效)
ENV ACADOS_SOURCE_DIR="/opt/acados"
ENV LD_LIBRARY_PATH="${ACADOS_SOURCE_DIR}/lib:${LD_LIBRARY_PATH}"
ENV PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH
ENV PATH=/opt/openrobots/bin:$PATH
ENV PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
ENV LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
ENV CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH

RUN uv pip install "numpy<2"
RUN echo "source ~/ros2_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo 'bass source ~/ros2_ws/install/setup.bash' >> /home/${USERNAME}/.config/fish/config.fish

WORKDIR /home/${USERNAME}
CMD ["/bin/bash"]

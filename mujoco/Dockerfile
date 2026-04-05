FROM elainasuki/ros:ros2-humble-full-0404

ARG USERNAME=Elaina
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG ROS_DISTRO=humble

ENV ROS_DISTRO=${ROS_DISTRO}
ENV HOME=/home/${USERNAME}

# 升级 pip
RUN pip3 install --no-cache-dir --upgrade pip

# 安装 MuJoCo 及可视化（2026年最新版已自带大部分依赖）
RUN pip3 install --no-cache-dir mujoco mujoco-python-viewer

RUN pip3 install --no-cache-dir do-mpc casadi matplotlib foxglove-sdk

RUN  mkdir -p /etc/apt/keyrings \
    && curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
    | sudo tee /etc/apt/keyrings/robotpkg.asc \
    &&  echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
    | sudo tee /etc/apt/sources.list.d/robotpkg.list \
    && apt-get update \
    && apt install -qqy robotpkg-py3*-pinocchio
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
    -DCMAKE_BUILD_TYPE=Release .. \
    && make install -j$(nproc) \
    && rm -rf /opt/acados/build
# 2. 安装 Python 接口
# 注意：在 Docker 中建议直接使用完整路径
RUN pip install -e /opt/acados/interfaces/acados_template

# 3. 设置环境变量 (这是最关键的一步，必须在 ENV 中设置，否则仅在 RUN 时有效)
ENV ACADOS_SOURCE_DIR="/opt/acados"
ENV LD_LIBRARY_PATH="${ACADOS_SOURCE_DIR}/lib:${LD_LIBRARY_PATH}"
RUN pip uninstall numpy -y && pip install "numpy<2"

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "source ~/ros2_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo 'bass source ~/ros2_ws/install/setup.bash' >> /home/${USERNAME}/.config/fish/config.fish
ENV PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH
ENV PATH=/opt/openrobots/bin:$PATH
ENV PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
ENV LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
ENV CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
WORKDIR /home/${USERNAME}
CMD ["/bin/bash"]
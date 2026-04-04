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
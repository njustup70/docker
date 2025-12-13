FROM elainasuki/ros:ros2-humble-full-1209

# 定义用户和用户组
ARG USERNAME=Elaina
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG ROS_DISTRO=humble
# 设置工作目录
WORKDIR /home/$USERNAME/packages

# 1. 创建标准工作空间目录结构
RUN mkdir -p \
    ros_ws/src \
    && chown -R ${USERNAME}:${USER_GID} /home/${USERNAME}/packages

# 3. 复制所有源码包（放在 apt 安装之前，利用缓存）
COPY --chown=${USERNAME}:${USER_GID} ./packages/ms200_ros/oradar_ros ./ros_ws/src/oradar_ros
COPY --chown=${USERNAME}:${USER_GID} ./packages/nagisa_imu ./ros_ws/src/nagisa_imu
COPY --chown=${USERNAME}:${USER_GID} ./packages/wheel_imu ./ros_ws/src/wheel_imu
COPY --chown=${USERNAME}:${USER_GID} ./packages/hipnuc_imu ./ros_ws/src/hipnuc_imu
COPY --chown=${USERNAME}:${USER_GID} ./packages/ROS-TCP-Endpoint-ROS2v0.7.0 ./ros_ws/src/ROS-TCP-Endpoint

# 4. RoboSense LiDAR
WORKDIR /home/$USERNAME/packages/ros_ws/src
RUN git clone --recurse-submodules https://github.com/RoboSense-LiDAR/rslidar_sdk.git \
    && git clone  https://github.com/RoboSense-LiDAR/rslidar_msg.git

# 复制自定义 CMakeLists.txt
COPY --chown=${USERNAME}:${USER_GID} ./packages/rslidar_airy/CMakeLists.txt ./rslidar_sdk/

#  安装必要的工具
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-gps-msgs libpcap-dev 

# 5. MCAP 工具
USER root
COPY ./packages/mcap /usr/local/bin/
RUN chmod +x /usr/local/bin/*
# 7. 统一构建所有 ROS2 包
USER root
WORKDIR /home/$USERNAME/packages
# 构建 ros_ws（先构建依赖包）
# 修改构建指令，先构建依赖包 hipnuc_lib_package，再构建其他包
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && cd ros_ws \
    # 先构建 hipnuc_lib_package（hipnuc_imu 的依赖）
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select serial\
    && . /home/$USERNAME/packages/ros_ws/install/setup.sh \
    # 再构建目标包（移除不存在的包）
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release 
USER $USERNAME
RUN pip install \
    pyserial \
    pyzmq \
    crcmod \
    pyserial-asyncio 
USER root
# 6. 安装 apt 包（放在源码后面，因为 apt 包较大且较少变化）
# 使用 ROS_DISTRO 变量
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-backward-ros \
    ros-${ROS_DISTRO}-realsense2-camera \
    ros-${ROS_DISTRO}-imu-tools \
    ros-${ROS_DISTRO}-rosbag2 \
    ros-${ROS_DISTRO}-rosbag2-storage-mcap \
    ros-${ROS_DISTRO}-orbbec-camera \
    # 其他工具
    docker \
    docker-compose \
    # 清理apt缓存减少镜像大小
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* \
    && rm -rf /home/$USERNAME/packages/ros_ws/src
# 8. 设置环境配置
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "source ~/packages/ros_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo 'bass source ~/packages/ros_ws/install/setup.bash' >> /home/${USERNAME}/.config/fish/config.fish \
    && echo 'bass source ~/ros2_driver/install/setup.bash' >> /home/${USERNAME}/.config/fish/config.fish

# 9. 设置最终环境变量
ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
ENV PYTHONPATH=/home/$USERNAME/ros2_driver/packages:$PYTHONPATH
ENV PATH=/home/$USERNAME/.local/bin:$PATH

CMD ["/bin/bash"]
FROM elainasuki/ros:ros2-humble-full-v3
# 定义用户和用户组
ARG USERNAME=Elaina
ARG USER_UID=1000
ARG USER_GID=$USER_UID
#安装必要软件
ARG ROS_DISTRO=humble
#添加orbbec深度相机驱动
WORKDIR /home/$USERNAME/packages
#基础依赖包和功能包
RUN sudo apt-get update && sudo apt-get install -y \
    libgflags-dev \
    nlohmann-json3-dev \
    ros-humble-image-transport \
    ros-humble-image-publisher \
    ros-humble-camera-info-manager \
    ros-humble-diagnostic-updater \
    ros-humble-diagnostic-msgs \
    ros-humble-statistics-msgs \
    ros-humble-backward-ros \
    libdw-dev
RUN mkdir -p ./orbbec_ws/src/ && cd ./orbbec_ws/src/ && \
    git config --global http.sslVerify false && git config --global http.postBuffer 524288000 && \
    git clone https://github.com/orbbec/OrbbecSDK_ROS2.git

# 构建 orbbec_ws 工作区
RUN . /opt/ros/humble/setup.sh && \
    export CMAKE_PREFIX_PATH=/opt/ros/humble:${CMAKE_PREFIX_PATH} && \ 
    cd orbbec_ws &&\
    # 构建整个 packages 工作区
    colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release &&\
    echo "source ~/packages/orbbec_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc
#添加realsense驱动
RUN sudo apt-get update \
    && sudo apt-get install -y ros-humble-realsense2-camera 

#添加ms200驱动
COPY  --chown=${USERNAME}:${USER_GID} ./packages/ms200_ros/oradar_ros /home/${USERNAME}/packages/ros_ws/src/oradar_ros 
COPY --chown=${USERNAME}:${USER_GID} ./packages/nagisa_imu /home/${USERNAME}/packages/ros_ws/src/nagisa_imu 
RUN . /opt/ros/humble/setup.sh && pip3 install pyserial && cd ./ros_ws && colcon build \
    && echo 'source ~/packages/ros_ws/install/setup.bash' >> /home/$USERNAME/.bashrc
#添加轮趣imu驱动
COPY --chown=${USERNAME}:${USER_GID} ./packages/wheel_imu /home/${USERNAME}/packages/ros_ws/src/wheel_imu
RUN . /opt/ros/humble/setup.sh && cd ./ros_ws && colcon build --packages-select  serial \
    && . ./install/setup.sh && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release 
#添加bashrc配置
USER root
#增加docker 嵌套支持与usb_cam支持
RUN apt-get update && apt-get install docker docker-compose ros-humble-usb-cam ros-humble-rosbridge-suite   ros-humble-imu-tools  -y
USER $USERNAME
RUN pip install pyzmq crcmod
USER root
RUN  echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> /home/${USERNAME}/.bashrc \
    #&& echo 'source ~/docker/ros2/packages/ros-humble-ros1-bridge/install/setup.bash'>> /home/${USERNAME}/.bashrc \
    && echo 'source /opt/ros/humble/setup.bash' >> /home/$USERNAME/.bashrc \ 
    && echo 'source ~/docker/ros2-modules/install/setup.bash' >> /home/$USERNAME/.bashrc \  
    && echo 'bass source ~/docker/ros2-modules/install/setup.bash' >> /home/$USERNAME/.config/fish/config.fish \
    && echo "export ROS_LOCALHOST_ONLY=1" >> /home/$USERNAME/.bashrc \
    && echo "export ROS_LOCALHOST_ONLY=1" >> /home/$USERNAME/.config/fish/config.fish 
CMD ["/bin/bash"]


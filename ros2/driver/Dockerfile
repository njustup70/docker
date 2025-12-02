FROM elainasuki/ros:ros2-humble-full-1202
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
    echo "source ~/packages/orbbec_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo 'bass source ~/packages/orbbec_ws/install/setup.bash' >> /home/$USERNAME/.config/fish/config.fish
#添加realsense驱动
RUN sudo apt-get update \
    && sudo apt-get install -y ros-humble-realsense2-camera 

#添加ms200驱动
COPY  --chown=${USERNAME}:${USER_GID} ./packages/ms200_ros/oradar_ros /home/${USERNAME}/packages/ros_ws/src/oradar_ros 
COPY --chown=${USERNAME}:${USER_GID} ./packages/nagisa_imu /home/${USERNAME}/packages/ros_ws/src/nagisa_imu 
RUN . /opt/ros/humble/setup.sh && pip3 install pyserial && cd ./ros_ws && colcon build \
    && echo 'source ~/packages/ros_ws/install/setup.bash' >> /home/$USERNAME/.bashrc \
    && echo 'bass source ~/packages/ros_ws/install/setup.bash' >> /home/$USERNAME/.config/fish/config.fish
#添加轮趣imu驱动
COPY --chown=${USERNAME}:${USER_GID} ./packages/wheel_imu /home/${USERNAME}/packages/ros_ws/src/wheel_imu
RUN . /opt/ros/humble/setup.sh && cd ./ros_ws && colcon build --packages-select  serial \
    && . ./install/setup.sh && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release 
#添加HIPNUC imu驱动
COPY --chown=${USERNAME}:${USER_GID} ./packages/hipnuc_imu /home/${USERNAME}/packages/ros_ws/src/hipnuc_imu
RUN . /opt/ros/humble/setup.sh && cd ./ros_ws && colcon build
#添加bashrc配置
USER root
#增加docker 嵌套支持与usb_cam支持
RUN apt-get update && apt-get install docker docker-compose ros-humble-usb-cam ros-humble-rosbridge-suite   ros-humble-imu-tools ros-humble-rosbag2 ros-humble-rosbag2-storage-mcap  -y
USER $USERNAME
RUN pip install pyzmq crcmod pyserial-asyncio    
USER root
# #安装foxglove 转化工具
COPY ./packages/mcap /usr/local/bin
RUN chmod +x -R /usr/local/bin
#增加仿真支持
COPY ./packages/ROS-TCP-Endpoint-ROS2v0.7.0 /home/${USERNAME}/packages/ros_ws/src/ROS-TCP-Endpoint-ROS2v0.7.0
RUN . /opt/ros/humble/setup.sh && cd ./ros_ws && colcon build
#安装robosense
RUN apt-get update && apt-get install -y  libpcap-dev ros-humble-rmw-zenoh-cpp\
    && cd /home/${USERNAME}/packages/ros_ws/src \
    && git clone --recurse-submodules https://github.com/RoboSense-LiDAR/rslidar_sdk.git \
    && git clone https://github.com/RoboSense-LiDAR/rslidar_msg.git 
#覆盖CMakeLists.txt
COPY  ./packages/rslidar_airy/CMakeLists.txt /home/${USERNAME}/packages/ros_ws/src/rslidar_sdk/CMakeLists.txt 
RUN  cd /home/${USERNAME}/packages/ros_ws && . /opt/ros/humble/setup.sh  && colcon build 
RUN echo 'source /opt/ros/humble/setup.bash' >> /home/$USERNAME/.bashrc \ 
    && echo 'source ~/ros2_driver/install/setup.bash' >> /home/$USERNAME/.bashrc \  
    && echo 'bass source ~/ros2_driver/install/setup.bash' >> /home/$USERNAME/.config/fish/config.fish \
    && echo 'bass source ~/packages/ws_livox/install/setup.bash' >> /home/$USERNAME/.config/fish/config.fish 
ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
ENV PYTHONPATH=/home/$USERNAME/ros2_driver/packages:$PYTHONPATH
CMD ["/bin/bash"]

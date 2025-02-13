FROM elainasuki/ros:ros2-humble-full-v2
ARG USERNAME=Elaina
#安装包
RUN apt-get update && apt-get install -y ros-humble-foxglove-bridge  usbutils  iputils-ping 
USER $USERNAME
WORKDIR /home/$USERNAME
#安装雷达驱动
RUN sudo -u $USERNAME mkdir ./packages  && cd ./packages && git clone https://github.com/Livox-SDK/Livox-SDK2.git \
    && cd Livox-SDK2 && mkdir build && cd build && cmake .. && make -j6 && sudo make install  
#引出雷达驱动so
#安装雷达ros驱动
WORKDIR /home/$USERNAME/packages/
USER ${USERNAME}
RUN mkdir -p ./ws_livox/src/ && cd ./ws_livox/src/ && \
    git clone https://github.com/Livox-SDK/livox_ros_driver2.git && \
    . /opt/ros/humble/setup.sh && export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH && \
    ./livox_ros_driver2/build.sh humble && echo 'source ~/packages/ws_livox/install/setup.bash' >> /home/Elaina/.bashrc
ARG ROS_DISTRO=humble
#安装pcl新版点云库
USER root
RUN git clone --branch pcl-1.13.1 https://github.com/PointCloudLibrary/pcl.git \
    && cd ./pcl && mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Release .. && make -j8 && make install 
# 添加livox驱动
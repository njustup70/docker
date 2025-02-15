FROM elainasuki/ros:ros2-humble-full-v2
ARG USERNAME=Elaina
#安装包
RUN apt-get update && apt-get install -y ros-humble-foxglove-bridge  usbutils  iputils-ping 
# 使用 root 用户
USER root

# 安装雷达驱动
RUN mkdir -p /home/$USERNAME/packages && cd /home/$USERNAME/packages && git clone https://github.com/Livox-SDK/Livox-SDK2.git \
    && cd Livox-SDK2 && mkdir build && cd build && cmake .. && make -j6 && make install

# 安装雷达 ROS 驱动
RUN mkdir -p /home/$USERNAME/packages/ws_livox/src/ \
    && cd /home/$USERNAME/packages/ws_livox/src/ \
    && git clone https://github.com/Livox-SDK/livox_ros_driver2.git \
    && . /opt/ros/humble/setup.sh \
    && export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH \
    && ./livox_ros_driver2/build.sh humble \
    && echo 'source ~/packages/ws_livox/install/setup.bash' >> /home/$USERNAME/.bashrc

# 设置 ROS 发行版
ARG ROS_DISTRO=humble

# 安装 PCL 新版点云库
RUN apt-get update \
    && apt-get install -y ninja-build ccache lld \
    && wget -qO- https://apt.llvm.org/llvm.sh | bash -s -- 20

RUN git clone --branch pcl-1.13.1 https://github.com/PointCloudLibrary/pcl.git \
    && cd pcl && mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ -G Ninja .. \
    && ninja -j6 && ninja install

# USER $USERNAME
# WORKDIR /home/$USERNAME
# #安装雷达驱动
# RUN sudo -u $USERNAME mkdir ./packages  && cd ./packages && git clone https://github.com/Livox-SDK/Livox-SDK2.git \
#     && cd Livox-SDK2 && mkdir build && cd build && cmake .. && make -j6 && sudo make install  
# #引出雷达驱动so
# #安装雷达ros驱动
# WORKDIR /home/$USERNAME/packages/
# USER ${USERNAME}
# RUN mkdir -p ./ws_livox/src/ && cd ./ws_livox/src/ && \
#     git clone https://github.com/Livox-SDK/livox_ros_driver2.git && \
#     . /opt/ros/humble/setup.sh && export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH && \
#     ./livox_ros_driver2/build.sh humble && echo 'source ~/packages/ws_livox/install/setup.bash' >> /home/Elaina/.bashrc
# ARG ROS_DISTRO=humble
# #安装pcl新版点云库
# USER root
# RUN apt-get update && apt-get install -y ninja-build ccache lld \
#     && wget -qO- https://apt.llvm.org/llvm.sh | bash -s -- 20

# RUN git clone --branch pcl-1.13.1 https://github.com/PointCloudLibrary/pcl.git \
#     && cd ./pcl && mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Release .. && make -j6 && make install 
# # 安装新版的构建工具

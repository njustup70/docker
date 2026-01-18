FROM elainasuki/ros:ros-noetic-full

ARG USERNAME=Elaina
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG GROUP_NAME=wheel
ENV DEBIAN_FRONTEND=noninteractive

USER root
# 导入最新 ROS key
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add -
RUN rm -f /etc/apt/sources.list.d/ros*.list
# 替换 ROS 源为 USTC 镜像
RUN echo "deb https://mirrors.ustc.edu.cn/ros/ubuntu/ focal main" \
    > /etc/apt/sources.list.d/ros-latest.list

# 导航所需
RUN apt-get update && apt-get install -y \
    ros-noetic-navigation \
    ros-noetic-map-server \
    ros-noetic-amcl \
    ros-noetic-move-base \
    ros-noetic-slam-gmapping \
    ros-noetic-tf \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher \
    ros-noetic-rviz \
    && rm -rf /var/lib/apt/lists/*
#其他依赖
RUN apt-get update && apt-get install -y \
    ros-noetic-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*

USER $USERNAME
RUN mkdir -p ~/catkin_ws/src
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    cd ~/catkin_ws && catkin_make"

RUN echo "source /opt/ros/noetic/setup.bash" >> .bashrc
RUN echo "source ~/Nav/devel/setup.bash" >> .bashrc
ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
CMD ["/bin/bash"]

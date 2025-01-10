FROM elainasuki/ros:ros2-humble-full-v2
# 定义用户和用户组
ARG USERNAME=Elaina
ARG USER_UID=1000
ARG USER_GID=$USER_UID
# 创建用户组和用户

#添加雷达驱动并设置雷达驱动环境变量 
USER $USERNAME
WORKDIR /home/$USERNAME
#引出雷达驱动so

# 删除 sudo 权限

RUN  echo 'source /opt/ros/humble/setup.bash' >> /home/$USERNAME/.bashrc\ 
    && echo 'source ~/docker/rosbridge/ros-humble-ros1-bridge/install/setup.bash' >> /home/$USERNAME/.bashrc
USER $USERNAME
CMD ["/bin/bash"]


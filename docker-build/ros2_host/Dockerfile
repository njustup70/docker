FROM elainasuki/ros:ros2-humble-full-v3
# 定义用户和用户组
ARG USERNAME=Elaina
COPY ./packages/ROS-TCP-Endpoint-ROS2v0.7.0 /home/${USERNAME}/packages/ros2_ws/src/ROS-TCP-Endpoint-ROS2v0.7.0
RUN . /opt/ros/humble/setup.sh && \
    cd /home/${USERNAME}/packages/ros2_ws && \
    colcon build --symlink-install \
    && apt-get update \
    && apt-get install -y ros-humble-foxglove-bridge \
    && echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "source ~/packages/ros2_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc
RUN echo "export ROS_LOCALHOST_ONLY=1" >> /home/$USERNAME/.bashrc \
    && echo "export ROS_LOCALHOST_ONLY=1" >> /home/$USERNAME/.config/fish/config.fish 
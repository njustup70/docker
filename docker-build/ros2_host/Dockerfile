FROM elainasuki/ros:ros2-jazzy-full
# 定义用户和用户组
ARG USERNAME=Elaina
COPY ./packages/ROS-TCP-Endpoint-ROS2v0.7.0 /home/${USERNAME}/packages/ros2_ws/src/ROS-TCP-Endpoint-ROS2v0.7.0
RUN . /opt/ros/jazzy/setup.sh && \
    cd /home/${USERNAME}/packages/ros2_ws && \
    colcon build --symlink-install \
    && echo "source /opt/ros/jazzy/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "source ~/packages/ros2_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "bass source ~/packages/ros2_ws/install/setup.bash" >> /home/${USERNAME}/.config/fish/config.fish \
    && echo "export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" >> /home/${USERNAME}/.bashrc \
    && echo "export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" >> /home/${USERNAME}/.config/fish/config.fish 
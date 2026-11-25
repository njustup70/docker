FROM up70docker/ros:ros2-humble-full-1017

USER root

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake python3-pip python3-colcon-common-extensions \
    libgl1 libglib2.0-0 libsm6 libice6 libusb-1.0-0 usbutils \
    ros-humble-rclpy ros-humble-vision-msgs ros-humble-rosidl-default-generators \
    ros-humble-rviz2 ros-humble-rmw-zenoh-cpp ros-humble-orbbec-camera \
    ros-humble-cv-bridge ros-humble-image-transport ros-humble-camera-info-manager \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /home/Elaina/ros2_ws \
    && chown Elaina:"$(id -gn Elaina)" /home/Elaina/ros2_ws

USER Elaina
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp \
    ZENOH_CONFIG_OVERRIDE=transport/shared_memory/enabled=false
RUN python3 -m pip install --user --no-cache-dir \
    numpy==1.26.4 opencv-python==4.11.0.86 PyYAML==6.0.3 \
    scipy==1.14.1 pip==24.0 setuptools==70.3.0 wheel==0.45.1 \
    && echo 'source /opt/ros/humble/setup.bash' >> /home/Elaina/.bashrc

WORKDIR /home/Elaina/ros2_ws
CMD ["/bin/bash"]

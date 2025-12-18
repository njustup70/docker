FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

# ---- Build/runtime deps for hik_camera_ros2 + spear_docking ----
# - OpenCV (with aruco) + cv_bridge
# - image_transport + camera_info_manager
# - colcon toolchain for building the mounted workspace
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    pkg-config \
    python3-pip \
    python3-setuptools \
    python3-numpy \
    python3-opencv \
    libopencv-dev \
    libopencv-contrib-dev \
    python3-colcon-common-extensions \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-camera-info-manager \
    ros-humble-rclcpp \
    ros-humble-rclpy \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

# ---- Hikrobot MVS SDK ----
# Build context must contain ./MVS (copied from host /opt/MVS).
COPY ./MVS /opt/MVS

# ---- MVS runtime env ----
ENV MVCAM_SDK_PATH=/opt/MVS
ENV MVCAM_COMMON_RUNENV=/opt/MVS/lib
ENV LD_LIBRARY_PATH=/opt/MVS/lib/64:/opt/MVS/lib/CLProtocol:$LD_LIBRARY_PATH

CMD ["/bin/bash"]

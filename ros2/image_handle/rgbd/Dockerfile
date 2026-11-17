# RGB-D 增量环境：基础镜像已包含主逻辑、Humble 和 Zenoh 的公共依赖。
FROM up70docker/rc2027:driverLogic@sha256:7f7e3deb3af0adf66811ac031a259d7304f45289292a04e06da0089c540243bb

USER root
# 固定摘要的主逻辑镜像已装好这些系统依赖（含 Orbbec 2.9.3）。
# 核对后直接复用，避免 apt update 顺带改变既有 ROS/驱动环境。
RUN set -eu; \
    test "$(dpkg --print-architecture)" = amd64; \
    for package in \
        python3-pip python3-colcon-common-extensions build-essential cmake \
        libgl1 libglib2.0-0 libsm6 libice6 libusb-1.0-0 usbutils \
        ros-humble-rclpy ros-humble-vision-msgs ros-humble-rosidl-default-generators \
        ros-humble-rviz2 ros-humble-rmw-zenoh-cpp ros-humble-orbbec-camera; do \
        test "$(dpkg-query -W -f='${Status}' "$package")" = 'install ok installed'; \
    done

COPY requirements.txt 99-obsensor-libusb.rules /opt/up70-rgbd/
# 规则来自同版本驱动；宿主机安装另由 install_udev.bash 执行。
RUN cmp /opt/up70-rgbd/99-obsensor-libusb.rules \
        /opt/ros/humble/share/orbbec_camera/udev/99-obsensor-libusb.rules \
    && mkdir -p /home/Elaina/ros2_ws \
    && chown Elaina: /home/Elaina/ros2_ws

# 主逻辑以 Elaina 运行，依赖也装入其实际使用的 user site。
# 系统 OpenCV 库留给既有 C++ 驱动，只统一 pip 的 cv2 变体。
USER Elaina
RUN /usr/bin/python3 -m pip uninstall -y \
       opencv-contrib-python opencv-python-headless opencv-contrib-python-headless \
    && /usr/bin/python3 -m pip install --user --no-cache-dir --upgrade \
       -r /opt/up70-rgbd/requirements.txt

COPY verify_env.py /opt/up70-rgbd/
RUN /bin/bash -c 'source /opt/ros/humble/setup.bash && python3 -m pip check && python3 /opt/up70-rgbd/verify_env.py'

WORKDIR /home/Elaina/ros2_ws
# 保留基础镜像 ENTRYPOINT；不设置 RMW/domain，不启动路由、相机或主逻辑。
CMD ["/bin/bash"]

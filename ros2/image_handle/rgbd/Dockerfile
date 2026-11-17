# RGB-D 环境基于已包含 ROS Humble 和 Orbbec 驱动的主逻辑镜像。
FROM up70docker/rc2027:driverLogic@sha256:7f7e3deb3af0adf66811ac031a259d7304f45289292a04e06da0089c540243bb

USER root
RUN mkdir -p /home/Elaina/ros2_ws \
    && chown Elaina: /home/Elaina/ros2_ws

USER Elaina
RUN /usr/bin/python3 -m pip uninstall -y \
       opencv-contrib-python opencv-python-headless opencv-contrib-python-headless \
    && /usr/bin/python3 -m pip install --user --no-cache-dir --upgrade \
       numpy==1.26.4 opencv-python==4.11.0.86 PyYAML==6.0.3 \
       scipy==1.14.1 pip==24.0 setuptools==70.3.0 wheel==0.45.1

WORKDIR /home/Elaina/ros2_ws
CMD ["/bin/bash"]

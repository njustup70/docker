#FROM elainasuki/ros:ros2-humble-full-v3
FROM elainasuki/ros:ros2-humble-full-1202

ARG USERNAME=Elaina
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ARG GROUP_NAME=wheel
RUN apt-get update \
    && apt-get install -y  sudo vim nautilus 

USER ${USERNAME}


#安装前置依赖
RUN pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu

# 安装yolo
RUN pip install imutils ultralytics openvino
#安装其他依赖
RUN pip install zmq "numpy<2.0"
# 安装ROS通信库
RUN pip install roslibpy pyzmq
# 安装实验跟踪库
RUN pip install comet_ml

USER root
RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "export PATH=\$HOME/.local/bin:\$PATH" >> /home/$USERNAME/.bashrc \
    && echo "set -gx PATH \$HOME/.local/bin \$PATH" >> /home/$USERNAME/.config/fish/config.fish

ENV PYTHONPATH=/home/Elaina/yolo
ENV PYTHONPATH=~/yolo
ENV PYTHONPATH=/home/Elaina/yolo/cv_lib
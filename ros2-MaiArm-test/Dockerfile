# ============================================================
#  MaiArm 机械臂开发镜像
#
#  功能：ROS 2 Humble + pinocchio + RViz，用于机械臂 FK/IK/拾取仿真
#  基础镜像：up70docker/ros:ros2-humble-full-1017
#     - 已自带 ROS 2 Humble desktop-full（rviz2 / tf2 / robot_state_publisher 等）
#     - 已创建 uid=1000 的用户 Elaina，与宿主机用户同号，
#       挂载工作空间后不会产生 root 属主的文件
#  额外安装：pinocchio（apt 版）、xacro、joint-state-publisher-gui
#
#  构建/推送由同目录 build.bash 负责（GitHub Actions 自动调用 --github-action）
# ============================================================
FROM up70docker/ros:ros2-humble-full-1017

ARG USERNAME=Elaina
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG ROS_DISTRO=humble
ENV HOME=/home/${USERNAME}

USER root

# ---------- 1. apt 依赖 ----------
RUN apt-get update && apt-get install -y \
        build-essential cmake git wget sudo vim \
        python3-pip python3-numpy \
        ros-${ROS_DISTRO}-pinocchio \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-robot-state-publisher \
        ros-${ROS_DISTRO}-joint-state-publisher-gui \
        ros-${ROS_DISTRO}-tf2-ros \
        ros-${ROS_DISTRO}-xacro \
    && rm -rf /var/lib/apt/lists/*

# ---------- 2. ROS 环境注入 ----------
# 非交互式 bash（docker exec bash -c '...'）不会读 .bashrc，
# 用 BASH_ENV 兜底；工作空间路径用 $HOME 表示，不写死宿主机路径。
RUN printf '%s\n' \
        "source /opt/ros/${ROS_DISTRO}/setup.bash" \
        '[ -f "$HOME/maiarm_ws/install/setup.bash" ] && source "$HOME/maiarm_ws/install/setup.bash"' \
        > /opt/maiarm_env.sh \
    && chmod 644 /opt/maiarm_env.sh \
    && echo 'source /opt/maiarm_env.sh' >> "/home/${USERNAME}/.bashrc" \
    && printf '%s\n' 'source /opt/maiarm_env.sh' > /etc/profile.d/maiarm_ros.sh

ENV BASH_ENV=/opt/maiarm_env.sh

# ---------- 3. 工作目录 ----------
# 工作空间由宿主机挂载进来；镜像里先建好目录作为默认 WORKDIR
USER ${USERNAME}
RUN mkdir -p /home/${USERNAME}/maiarm_ws
WORKDIR /home/${USERNAME}/maiarm_ws

CMD ["/bin/bash"]

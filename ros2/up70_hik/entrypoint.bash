#!/usr/bin/env bash

#set -u

ROS_DISTRO="${ROS_DISTRO:-humble}"
WORKSPACE="${WORKSPACE:-/home/Elaina/ros2_driver}"
AUTO_BUILD_PACKAGES="${AUTO_BUILD_PACKAGES:-hik_camera_ros2 my_driver spear_docking rc_bringup}"

echo "[up70_hik] ROS_DISTRO=${ROS_DISTRO}"
echo "[up70_hik] WORKSPACE=${WORKSPACE}"
echo "[up70_hik] AUTO_BUILD_PACKAGES=${AUTO_BUILD_PACKAGES}"

if [[ ! -d "${WORKSPACE}" ]]; then
  echo "[up70_hik] ERROR: WORKSPACE not found: ${WORKSPACE}"
  echo "[up70_hik] Check your volume mount for ros2_driver."
  exec tail -f /dev/null
fi

# 解决你之前遇到的 ~/.ros/log 权限问题：强制把日志放到工作区内
export ROS_LOG_DIR="${ROS_LOG_DIR:-${WORKSPACE}/.ros_log}"
mkdir -p "${ROS_LOG_DIR}" || true

if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
else
  echo "[up70_hik] ERROR: /opt/ros/${ROS_DISTRO}/setup.bash not found"
  exec tail -f /dev/null
fi

if [[ -d "/opt/MVS" ]]; then
  export MVCAM_SDK_PATH=/opt/MVS
  export MVCAM_COMMON_RUNENV=/opt/MVS/lib
  export MVCAM_GENICAM_CLPROTOCOL=/opt/MVS/lib/CLProtocol
  export ALLUSERSPROFILE=/opt/MVS/MVFG
  export LD_LIBRARY_PATH="/opt/MVS/lib/64:/opt/MVS/lib/CLProtocol:${LD_LIBRARY_PATH:-}"
else
  echo "[up70_hik] WARN: /opt/MVS not found. hik_camera_ros2 will fail to build/run."
fi

cd "${WORKSPACE}"

echo "[up70_hik] Running colcon build..."
colcon build --symlink-install --packages-select ${AUTO_BUILD_PACKAGES}
BUILD_RC=$?
if [[ ${BUILD_RC} -ne 0 ]]; then
  echo "[up70_hik] ERROR: colcon build failed (rc=${BUILD_RC}). Container will stay alive for debugging."
else
  echo "[up70_hik] colcon build OK."
fi

echo "[up70_hik] Ready. Use: docker exec -it up70_hik bash"
exec tail -f /dev/null

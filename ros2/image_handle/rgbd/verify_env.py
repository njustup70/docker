"""检查实际运行用户的 RGB-D 环境；不初始化 ROS 或访问相机。"""

import importlib.metadata
import os
from pathlib import Path
import pwd
import subprocess
import sys
import xml.etree.ElementTree as ET

import cv2
import numpy
import rclpy
import scipy
from scipy.spatial.transform import Rotation
import yaml
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection3D


def main():
    """依赖或版本不匹配时失败，让镜像构建及时停止。"""
    assert sys.version_info[:2] == (3, 10), sys.version
    assert os.environ.get("ROS_DISTRO") == "humble"
    assert pwd.getpwuid(os.getuid()).pw_name == "Elaina"
    assert numpy.__version__ == "1.26.4", numpy.__version__
    assert cv2.__version__ == "4.11.0", cv2.__version__
    assert yaml.__version__ == "6.0.3", yaml.__version__
    assert scipy.__version__ == "1.14.1", scipy.__version__
    for package, version in (("pip", "24.0"), ("setuptools", "70.3.0"), ("wheel", "0.45.1")):
        assert importlib.metadata.version(package) == version
    assert numpy.allclose(Rotation.identity().as_matrix(), numpy.eye(3))
    variants = {"opencv-python", "opencv-contrib-python", "opencv-python-headless",
                "opencv-contrib-python-headless"}
    installed = {d.metadata["Name"].lower() for d in importlib.metadata.distributions()}
    assert installed & variants == {"opencv-python"}, installed & variants
    camera_share = Path(get_package_share_directory("orbbec_camera"))
    assert ET.parse(camera_share / "package.xml").findtext("version") == "2.9.3"
    driver = subprocess.check_output(
        ["dpkg-query", "-W", "-f=${Version}", "ros-humble-orbbec-camera"], text=True
    )
    assert driver.startswith("2.9.3-"), driver
    for package in ("rmw_zenoh_cpp", "rosidl_default_generators", "rviz2"):
        get_package_share_directory(package)
    for library in ("libOrbbecSDK.so.2.9.3", "liborbbec_camera.so", "librmw_zenoh_cpp.so"):
        deps = subprocess.check_output(["ldd", "/opt/ros/humble/lib/" + library], text=True)
        assert "not found" not in deps, deps
    assert rclpy and Image and Detection3D
    print(f"RGB-D environment OK: Python {sys.version.split()[0]}, Orbbec {driver}")
    for module in (numpy, cv2, yaml, scipy):
        print(f"{module.__name__}: {module.__version__} ({module.__file__})")


if __name__ == "__main__":
    main()

from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

from . import config


@dataclass
class Pose:
    rvec: np.ndarray  # 从板到相机的旋转向量（Rodrigues）
    tvec: np.ndarray  # 相机到杆尖的平移向量（相机坐标系）


class VisionPnP:
    def __init__(self, smoothing_alpha: float = 0.8) -> None:
        self.board = config.ARUCO_BOARD
        self.dictionary = config.ARUCO_DICT
        self.alpha = smoothing_alpha
        self.prev_pose: Optional[Pose] = None
        self.detector_params = cv2.aruco.DetectorParameters_create()

    def estimate_pose(self, image) -> Optional[Pose]:
        gray = image
        if len(image.shape) == 3 and image.shape[2] == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.detector_params)
        if ids is None or len(ids) == 0:
            return None

        # ---------------------------------------------------------
        # 修复点：OpenCV 4.5.4 强制要求传入 rvec 和 tvec 参数
        # 我们传入 None, None 表示不使用初始猜测
        # ---------------------------------------------------------
        retval, rvec, tvec = cv2.aruco.estimatePoseBoard(
            corners, ids, self.board, config.CAMERA_MATRIX, config.DIST_COEFFS, None, None
        )
        
        if retval <= 0:
            return None

        # 将平移从 ID0 中心转换到杆尖。
        R, _ = cv2.Rodrigues(rvec)
        tip_offset_cam = R @ config.BRACKET_CENTER_OFFSET.reshape(3, 1)
        tvec_tip = tvec + tip_offset_cam

        if self.prev_pose is None:
            smoothed_rvec = rvec
            smoothed_tvec = tvec_tip
        else:
            smoothed_rvec = self.alpha * self.prev_pose.rvec + (1.0 - self.alpha) * rvec
            smoothed_tvec = self.alpha * self.prev_pose.tvec + (1.0 - self.alpha) * tvec_tip

        pose = Pose(rvec=smoothed_rvec, tvec=smoothed_tvec)
        self.prev_pose = pose
        return pose
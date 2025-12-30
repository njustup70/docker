import numpy as np
import cv2

# 海康 MV-CU013-A0UC + 6mm 镜头
# 数据来源：重新调焦后的高质量标定 (2025/12/11)
# 评价：fx/fy 高度一致，光心准确，畸变合理
CAMERA_MATRIX = np.array([
    [1.31440592e+03, 0.00000000e+00, 6.26245121e+02],
    [0.00000000e+00, 1.31445088e+03, 5.25952449e+02],
    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
], dtype=np.float32)

DIST_COEFFS = np.array([
    [-0.0772329, 0.12858771, 0.00032125, 0.00108192, -0.13987908]
], dtype=np.float32)

ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# ================= 关键配置区域 =================
# 双码布局（单位：米）
# 基于台架实测数据：前后差 33mm，左右差 4mm
MARKER_SPECS = [
    # ID 0: 后码 (基准)
    {"id": 0, "length": 0.030, "position": np.array([0.0, 0.0, 0.0], dtype=np.float32)},
    
    # ID 1: 前码 (凸出)
    # 注意：如果发现 X 轴反了（移动方向相反），请把 0.004 改成 -0.004
    {"id": 1, "length": 0.030, "position": np.array([0.004, 0.0, 0.033], dtype=np.float32)},
]

# 暂时归零，用于台架纯算法验证
ROD_TIP_OFFSET = np.array([0.0, 0.0, 0.0], dtype=np.float32)
BRACKET_CENTER_OFFSET = ROD_TIP_OFFSET.copy()

def _marker_corners(center: np.ndarray, size: float) -> np.ndarray:
    half = size / 2.0
    x, y, z = center
    return np.array(
        [[x-half, y+half, z], [x+half, y+half, z], 
         [x+half, y-half, z], [x-half, y-half, z]], dtype=np.float32)

def build_board():
    corners = [_marker_corners(spec["position"], spec["length"]) for spec in MARKER_SPECS]
    ids = np.array([spec["id"] for spec in MARKER_SPECS], dtype=np.int32)
    # 兼容 OpenCV 4.5.x (ROS Humble) 和 新版
    try:
        return cv2.aruco.Board_create(corners, ARUCO_DICT, ids)
    except AttributeError:
        return cv2.aruco.Board(corners, ARUCO_DICT, ids)

ARUCO_BOARD = build_board()
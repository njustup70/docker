import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

# 棋盘格内部角点数 (列, 行)
CHECKERBOARD_SIZE = (9, 6)
# 棋盘格每个格子的边长（米）
SQUARE_SIZE = 0.020
# 采集设置
SAMPLES_NEEDED = 20
MIN_CAPTURE_INTERVAL = 2.0  # 秒


class CameraCalibrationNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_calibration_node")
        self.bridge = CvBridge()
        self.objpoints = []
        self.imgpoints = []
        self.last_capture_time = 0.0
        self.calibrated = False
        self.image_size = None

        self.objp_template = np.zeros((CHECKERBOARD_SIZE[0] * CHECKERBOARD_SIZE[1], 3), np.float32)
        grid = np.mgrid[0 : CHECKERBOARD_SIZE[0], 0 : CHECKERBOARD_SIZE[1]].T.reshape(-1, 2)
        self.objp_template[:, :2] = grid * SQUARE_SIZE

        self.criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30,
            0.001,
        )

        self.create_subscription(Image, "/hik_camera/image_raw", self.image_callback, 10)
        self.get_logger().info("Camera calibration node started. Looking for checkerboard...")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # 这里未显式引入 CvBridgeError 类型
            self.get_logger().error(f"Failed to convert image: {exc}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.image_size = (gray.shape[1], gray.shape[0])

        ret, corners = cv2.findChessboardCorners(
            gray,
            CHECKERBOARD_SIZE,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK,
        )

        if ret:
            corners2 = cv2.cornerSubPix(
                gray,
                corners,
                winSize=(11, 11),
                zeroZone=(-1, -1),
                criteria=self.criteria,
            )
            now = self._now()
            if (now - self.last_capture_time) >= MIN_CAPTURE_INTERVAL and len(self.objpoints) < SAMPLES_NEEDED:
                self.objpoints.append(self.objp_template.copy())
                self.imgpoints.append(corners2)
                self.last_capture_time = now
                self.get_logger().info(f"Captured sample {len(self.objpoints)}/{SAMPLES_NEEDED}")

            cv2.drawChessboardCorners(frame, CHECKERBOARD_SIZE, corners2, ret)

        cv2.putText(
            frame,
            f"已采集样本数: {len(self.objpoints)}/{SAMPLES_NEEDED}",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        cv2.imshow("Camera Calibration", frame)
        cv2.waitKey(1)

        if not self.calibrated and len(self.objpoints) >= SAMPLES_NEEDED:
            self.run_calibration()

    def run_calibration(self) -> None:
        if self.image_size is None:
            self.get_logger().error("Image size unknown, cannot calibrate.")
            return

        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            self.objpoints, self.imgpoints, self.image_size, None, None
        )
        if not ret:
            self.get_logger().error("Calibration failed.")
            return

        self.calibrated = True
        print("=" * 30)
        print("标定完成！请将以下参数复制到 config.py:")
        print(f"CAMERA_MATRIX = np.array({np.array2string(mtx, separator=', ')}, dtype=np.float32)")
        print(f"DIST_COEFFS = np.array({np.array2string(dist, separator=', ')}, dtype=np.float32)")
        print("=" * 30)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraCalibrationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

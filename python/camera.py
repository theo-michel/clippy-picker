"""MMlove stereo camera wrapper with MJPEG streaming for the Flask web controller."""

from __future__ import annotations

import logging
import threading
import time
from pathlib import Path
from typing import Generator

import cv2
import numpy as np

log = logging.getLogger(__name__)

DEFAULT_CALIBRATION = (
    Path(__file__).parent / "calibration" / "intrinsic" / "stereo_calibration.npz"
)


def camera_available() -> bool:
    return True


class StereoCamera:
    """Thread-safe stereo camera that produces MJPEG frames and depth via disparity."""

    def __init__(
        self,
        device_index: int = 0,
        capture_width: int = 2560,
        capture_height: int = 720,
        fps: int = 30,
        calibration_path: Path | str = DEFAULT_CALIBRATION,
        num_disparities: int = 192,
    ):
        self.device_index = device_index
        self.capture_width = capture_width
        self.capture_height = capture_height
        self.fps = fps
        self.calibration_path = Path(calibration_path)
        self._num_disparities = num_disparities

        self._cap: cv2.VideoCapture | None = None
        self._running = False
        self._lock = threading.Lock()
        self._jpeg: bytes | None = None
        self._color_frame: np.ndarray | None = None
        self._disparity: np.ndarray | None = None

        self._map_l1: np.ndarray | None = None
        self._map_l2: np.ndarray | None = None
        self._map_r1: np.ndarray | None = None
        self._map_r2: np.ndarray | None = None
        self._Q: np.ndarray | None = None
        self._focal: float = 0.0
        self._baseline: float = 0.0

        self._stereo_matcher: cv2.StereoSGBM | None = None

        self._overlay_aruco = False
        self._aruco_detector: cv2.aruco.ArucoDetector | None = None

    @property
    def running(self) -> bool:
        return self._running

    def _load_calibration(self) -> None:
        if not self.calibration_path.exists():
            raise FileNotFoundError(
                f"Stereo calibration not found at {self.calibration_path}"
            )
        data = np.load(str(self.calibration_path))
        self._map_l1 = data["map_left_1"]
        self._map_l2 = data["map_left_2"]
        self._map_r1 = data["map_right_1"]
        self._map_r2 = data["map_right_2"]
        self._Q = data["Q"]
        log.info("Loaded stereo calibration from %s", self.calibration_path)

    def start(self) -> None:
        if self._running:
            return
        self._load_calibration()
        cap = cv2.VideoCapture(self.device_index)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.capture_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.capture_height)
        if not cap.isOpened():
            raise RuntimeError(f"Cannot open camera at index {self.device_index}")

        self._focal = abs(self._Q[2, 3])
        self._baseline = abs(1.0 / self._Q[3, 2])

        block = 5
        self._stereo_matcher = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=self._num_disparities,
            blockSize=block,
            P1=8 * 3 * block * block,
            P2=32 * 3 * block * block,
            disp12MaxDiff=1,
            uniquenessRatio=5,
            speckleWindowSize=200,
            speckleRange=2,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )

        self._cap = cap
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()
        log.info(
            "Stereo camera started (device %d, %dx%d)",
            self.device_index,
            self.capture_width,
            self.capture_height,
        )

    def stop(self) -> None:
        self._running = False
        if self._cap:
            self._cap.release()
            self._cap = None
        log.info("Camera stopped")

    def set_overlay(self, *, aruco: bool = False) -> None:
        self._overlay_aruco = aruco
        if aruco and self._aruco_detector is None:
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            params = cv2.aruco.DetectorParameters()
            self._aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, params)

    def _draw_aruco_overlay(self, img: np.ndarray) -> np.ndarray:
        if self._aruco_detector is None:
            return img
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners_list, ids, _ = self._aruco_detector.detectMarkers(gray)
        if ids is None:
            return img
        for corners, marker_id in zip(corners_list, ids.ravel()):
            pts = corners[0].astype(np.int32)
            cv2.polylines(img, [pts], True, (0, 255, 0), 2)
            cx, cy = int(pts[:, 0].mean()), int(pts[:, 1].mean())
            cv2.putText(
                img,
                f"ID {marker_id}",
                (cx - 20, cy - 12),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )
            cv2.drawMarker(img, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 10, 1)
        return img

    def _loop(self) -> None:
        map_l1, map_l2 = self._map_l1, self._map_l2
        map_r1, map_r2 = self._map_r1, self._map_r2
        matcher = self._stereo_matcher
        assert map_l1 is not None and map_l2 is not None
        assert map_r1 is not None and map_r2 is not None
        assert matcher is not None

        while self._running and self._cap:
            try:
                ret, frame = self._cap.read()
                if not ret:
                    time.sleep(0.01)
                    continue

                h, w = frame.shape[:2]
                mid = w // 2
                left_raw = frame[:, :mid]
                right_raw = frame[:, mid:]

                left_rect = cv2.remap(left_raw, map_l1, map_l2, cv2.INTER_LINEAR)
                right_rect = cv2.remap(right_raw, map_r1, map_r2, cv2.INTER_LINEAR)

                left_gray = cv2.cvtColor(left_rect, cv2.COLOR_BGR2GRAY)
                right_gray = cv2.cvtColor(right_rect, cv2.COLOR_BGR2GRAY)
                disparity = (
                    matcher.compute(left_gray, right_gray).astype(np.float32) / 16.0
                )

                display = left_rect.copy()
                if self._overlay_aruco:
                    self._draw_aruco_overlay(display)

                _, buf = cv2.imencode(".jpg", display, [cv2.IMWRITE_JPEG_QUALITY, 80])
                with self._lock:
                    self._jpeg = buf.tobytes()
                    self._color_frame = left_rect
                    self._disparity = disparity
            except Exception as exc:
                log.debug("Capture error: %s", exc)
                time.sleep(0.1)

    def get_jpeg(self) -> bytes | None:
        with self._lock:
            return self._jpeg

    def generate_mjpeg(self) -> Generator[bytes, None, None]:
        dt = 1.0 / self.fps
        while self._running:
            frame = self.get_jpeg()
            if frame:
                yield (
                    b"--frame\r\n" b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
                )
            time.sleep(dt)

    def get_color_frame(self) -> np.ndarray | None:
        with self._lock:
            return self._color_frame.copy() if self._color_frame is not None else None

    def detect_aruco(self, dictionary_name: str = "DICT_4X4_50") -> list[dict]:
        frame = self.get_color_frame()
        if frame is None:
            return []
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        dict_id = getattr(cv2.aruco, dictionary_name, cv2.aruco.DICT_4X4_50)
        aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        corners_list, ids, _ = detector.detectMarkers(gray)
        if ids is None:
            return []
        results = []
        for corners, marker_id in zip(corners_list, ids.ravel()):
            pts = corners[0]
            cx = float(pts[:, 0].mean())
            cy = float(pts[:, 1].mean())
            results.append(
                {
                    "id": int(marker_id),
                    "center": (cx, cy),
                    "corners": [(float(p[0]), float(p[1])) for p in pts],
                }
            )
        return results

    def get_pointcloud_ply(
        self,
        *,
        z_min_mm: float = 100.0,
        z_max_mm: float = 1500.0,
        stride: int = 1,
    ) -> bytes | None:
        """Serialize the current frame as a binary PLY point cloud.

        Points are in the rectified left-camera frame in millimetres; colors
        come from the rectified left image. ``stride`` decimates the cloud
        (e.g. ``stride=2`` keeps one pixel in four) to keep transfers small.
        Returns ``None`` if no valid frame has been captured yet.
        """
        with self._lock:
            if (
                self._disparity is None
                or self._color_frame is None
                or self._Q is None
            ):
                return None
            disp = self._disparity.copy()
            color = self._color_frame.copy()
            Q = self._Q.copy()

        if stride < 1:
            stride = 1
        if stride > 1:
            disp = disp[::stride, ::stride]
            color = color[::stride, ::stride]

        points_m = cv2.reprojectImageTo3D(disp, Q)
        points_mm = points_m * 1000.0
        z = points_mm[..., 2]
        mask = (disp > 0) & np.isfinite(z) & (z > z_min_mm) & (z < z_max_mm)

        pts = points_mm[mask].astype(np.float32)
        # PLY wants RGB, color_frame is BGR.
        rgb = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)[mask].astype(np.uint8)

        n = pts.shape[0]
        header = (
            "ply\n"
            "format binary_little_endian 1.0\n"
            f"element vertex {n}\n"
            "property float x\n"
            "property float y\n"
            "property float z\n"
            "property uchar red\n"
            "property uchar green\n"
            "property uchar blue\n"
            "end_header\n"
        ).encode("ascii")

        # Pack as an interleaved struct: 3xfloat32 + 3xuint8 per vertex.
        vertex_dtype = np.dtype(
            [("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
             ("r", "u1"), ("g", "u1"), ("b", "u1")]
        )
        vertices = np.empty(n, dtype=vertex_dtype)
        vertices["x"] = pts[:, 0]
        vertices["y"] = pts[:, 1]
        vertices["z"] = pts[:, 2]
        vertices["r"] = rgb[:, 0]
        vertices["g"] = rgb[:, 1]
        vertices["b"] = rgb[:, 2]

        return header + vertices.tobytes()

    def deproject(
        self, u: int, v: int, *, patch: int = 0
    ) -> tuple[float, float, float] | None:
        """Pixel (u, v) in the rectified left image -> 3D point in camera frame (mm).

        Stereo matching runs at full image resolution.  Depth is computed as
        ``focal * baseline / disp`` matching depth_preview.py. The Q matrix
        is used only for X/Y reprojection.
        """
        with self._lock:
            if self._disparity is None or self._Q is None:
                return None
            h, w = self._disparity.shape
            if not (0 <= v < h and 0 <= u < w):
                return None
            if patch > 0:
                v0, v1 = max(0, v - patch), min(h, v + patch + 1)
                u0, u1 = max(0, u - patch), min(w, u + patch + 1)
                region = self._disparity[v0:v1, u0:u1]
                valid = region[region > 0]
                disp = float(np.median(valid)) if len(valid) > 0 else 0.0
            else:
                disp = float(self._disparity[v, u])
            focal = self._focal
            baseline = self._baseline
            Q = self._Q

        if disp <= 0:
            return None

        Z_mm = focal * baseline / disp * 1000.0
        cx = Q[0, 3]
        cy = Q[1, 3]
        X_mm = (u + cx) / focal * Z_mm
        Y_mm = (v + cy) / focal * Z_mm
        return (X_mm, Y_mm, Z_mm)

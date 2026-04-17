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
        device_index: int = 2,
        capture_width: int = 2560,
        capture_height: int = 720,
        fps: int = 30,
        calibration_path: Path | str = DEFAULT_CALIBRATION,
        stereo_scale: float = 0.5,
        num_disparities: int = 192,
    ):
        self.device_index = device_index
        self.capture_width = capture_width
        self.capture_height = capture_height
        self.fps = fps
        self.calibration_path = Path(calibration_path)
        self._stereo_scale = stereo_scale
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
        self._overlay_detections = False
        self._aruco_detector: cv2.aruco.ArucoDetector | None = None
        self._detector = None
        self._latest_detections: list = []

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

    def _find_stereo_index(self) -> int:
        """Scan indices 0-9 for a side-by-side stereo feed (aspect > 2:1)."""
        for idx in range(10):
            cap = cv2.VideoCapture(idx)
            if not cap.isOpened():
                continue
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.capture_width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.capture_height)
            ret, frame = cap.read()
            cap.release()
            if ret and frame is not None and frame.shape[1] / frame.shape[0] > 2.0:
                log.info("Stereo camera auto-detected at index %d", idx)
                return idx
        return -1

    def start(self) -> None:
        if self._running:
            return
        self._load_calibration()
        cap = cv2.VideoCapture(self.device_index)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.capture_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.capture_height)
        if not cap.isOpened() or cap.get(cv2.CAP_PROP_FRAME_WIDTH) / max(cap.get(cv2.CAP_PROP_FRAME_HEIGHT), 1) < 2.0:
            cap.release()
            idx = self._find_stereo_index()
            if idx < 0:
                raise RuntimeError("No stereo camera found on any index (0-9)")
            self.device_index = idx
            cap = cv2.VideoCapture(idx)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.capture_width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.capture_height)
        if not cap.isOpened():
            raise RuntimeError(f"Cannot open camera at index {self.device_index}")

        self._focal = abs(self._Q[2, 3]) * self._stereo_scale
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

    def set_overlay(self, *, aruco: bool = False, detections: bool = False) -> None:
        self._overlay_aruco = aruco
        self._overlay_detections = detections
        if aruco and self._aruco_detector is None:
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            params = cv2.aruco.DetectorParameters()
            self._aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        if detections and self._detector is None:
            from object_detection import ObjectDetector

            self._detector = ObjectDetector()

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

    def _draw_detection_overlay(self, img: np.ndarray) -> np.ndarray:
        if self._detector is None:
            return img
        dets = self._detector.detect(img)
        self._latest_detections = dets
        for det in dets:
            x1, y1, x2, y2 = det.bbox
            cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cx, cy = int(det.center_uv[0]), int(det.center_uv[1])
            cv2.drawMarker(img, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 12, 2)
            text = f"{det.label} {det.confidence:.0%}"
            cv2.putText(
                img, text, (x1, y1 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 0), 1
            )
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

                s = self._stereo_scale
                if s != 1.0:
                    left_sm = cv2.resize(left_rect, None, fx=s, fy=s)
                    right_sm = cv2.resize(right_rect, None, fx=s, fy=s)
                else:
                    left_sm, right_sm = left_rect, right_rect

                left_gray = cv2.cvtColor(left_sm, cv2.COLOR_BGR2GRAY)
                right_gray = cv2.cvtColor(right_sm, cv2.COLOR_BGR2GRAY)
                disparity = (
                    matcher.compute(left_gray, right_gray).astype(np.float32) / 16.0
                )

                display = left_rect.copy()
                if self._overlay_aruco:
                    self._draw_aruco_overlay(display)
                if self._overlay_detections:
                    self._draw_detection_overlay(display)

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

    def deproject(
        self, u: int, v: int, *, patch: int = 0
    ) -> tuple[float, float, float] | None:
        """Pixel (u, v) in the rectified left image -> 3D point in camera frame (mm).

        Stereo matching runs at ``_stereo_scale`` resolution (default 0.5).
        Depth is computed as ``focal * baseline / disp`` matching depth_preview.py.
        The Q matrix is used only for X/Y reprojection.
        """
        with self._lock:
            if self._disparity is None or self._Q is None:
                return None
            s = self._stereo_scale
            us, vs = int(round(u * s)), int(round(v * s))
            h, w = self._disparity.shape
            if not (0 <= vs < h and 0 <= us < w):
                return None
            if patch > 0:
                ps = max(1, int(round(patch * s)))
                v0, v1 = max(0, vs - ps), min(h, vs + ps + 1)
                u0, u1 = max(0, us - ps), min(w, us + ps + 1)
                region = self._disparity[v0:v1, u0:u1]
                valid = region[region > 0]
                disp = float(np.median(valid)) if len(valid) > 0 else 0.0
            else:
                disp = float(self._disparity[vs, us])
            focal = self._focal
            baseline = self._baseline
            Q = self._Q

        if disp <= 0:
            return None

        Z_mm = focal * baseline / disp * 1000.0
        cx = Q[0, 3]
        cy = Q[1, 3]
        focal_full = abs(Q[2, 3])
        X_mm = (u - cx) / focal_full * (Z_mm / 1000.0) * 1000.0
        Y_mm = (v - cy) / focal_full * (Z_mm / 1000.0) * 1000.0
        return (X_mm, Y_mm, Z_mm)

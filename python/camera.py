"""RealSense camera wrapper with MJPEG streaming for the Flask web controller."""

from __future__ import annotations

import logging
import threading
import time
from typing import Generator

import cv2
import numpy as np

log = logging.getLogger(__name__)

try:
    import pyrealsense2 as rs

    _HAS_REALSENSE = True
except ImportError:
    rs = None  # type: ignore[assignment]
    _HAS_REALSENSE = False
    log.info("pyrealsense2 not available — camera features disabled")


def realsense_available() -> bool:
    return _HAS_REALSENSE


class RealsenseCamera:
    """Thread-safe RealSense camera that produces MJPEG frames."""

    def __init__(self, width: int = 640, height: int = 480, fps: int = 30):
        if not _HAS_REALSENSE:
            raise RuntimeError("pyrealsense2 is not installed")
        self.width = width
        self.height = height
        self.fps = fps
        self._pipeline: rs.pipeline | None = None  # type: ignore[name-defined]
        self._align = None
        self._running = False
        self._lock = threading.Lock()
        self._jpeg: bytes | None = None
        self._color_frame: np.ndarray | None = None
        self._depth_data: np.ndarray | None = None
        self._depth_scale: float = 0.0
        self._intrinsics = None
        self._overlay_aruco = False
        self._aruco_detector: cv2.aruco.ArucoDetector | None = None

    @property
    def running(self) -> bool:
        return self._running

    def start(self) -> None:
        if self._running:
            return
        pipeline = rs.pipeline()  # type: ignore[union-attr]
        cfg = rs.config()  # type: ignore[union-attr]
        cfg.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)  # type: ignore[union-attr]
        cfg.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)  # type: ignore[union-attr]
        profile = pipeline.start(cfg)
        self._depth_scale = (
            profile.get_device().first_depth_sensor().get_depth_scale()
        )
        self._align = rs.align(rs.stream.color)  # type: ignore[union-attr]
        self._pipeline = pipeline
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()
        log.info("Camera started (%dx%d@%d)", self.width, self.height, self.fps)

    def stop(self) -> None:
        self._running = False
        if self._pipeline:
            try:
                self._pipeline.stop()
            except Exception:
                pass
            self._pipeline = None
        log.info("Camera stopped")

    def set_overlay(self, *, aruco: bool = False) -> None:
        """Toggle ArUco marker overlay on the MJPEG stream."""
        self._overlay_aruco = aruco
        if aruco and self._aruco_detector is None:
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            params = cv2.aruco.DetectorParameters()
            self._aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, params)

    def _draw_aruco_overlay(self, img: np.ndarray) -> np.ndarray:
        """Draw detected ArUco markers onto the frame (in-place)."""
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
            cv2.putText(img, f"ID {marker_id}", (cx - 20, cy - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.drawMarker(img, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 10, 1)
        return img

    def _loop(self) -> None:
        while self._running and self._pipeline:
            try:
                ok, frames = self._pipeline.try_wait_for_frames(1000)
                if not ok:
                    continue
                aligned = self._align.process(frames)
                depth = aligned.get_depth_frame()
                color = aligned.get_color_frame()
                if not depth or not color:
                    continue
                self._intrinsics = (
                    depth.profile.as_video_stream_profile().intrinsics
                )
                img = np.asanyarray(color.get_data())
                display = img.copy()
                if self._overlay_aruco:
                    self._draw_aruco_overlay(display)
                _, buf = cv2.imencode(".jpg", display, [cv2.IMWRITE_JPEG_QUALITY, 80])
                with self._lock:
                    self._jpeg = buf.tobytes()
                    self._color_frame = img.copy()
                    self._depth_data = np.asanyarray(depth.get_data())
            except Exception as exc:
                log.debug("Capture error: %s", exc)
                time.sleep(0.1)

    def get_jpeg(self) -> bytes | None:
        with self._lock:
            return self._jpeg

    def generate_mjpeg(self) -> Generator[bytes, None, None]:
        """Yield multipart MJPEG frames for a streaming HTTP response."""
        dt = 1.0 / self.fps
        while self._running:
            frame = self.get_jpeg()
            if frame:
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
                )
            time.sleep(dt)

    def get_color_frame(self) -> np.ndarray | None:
        """Return the latest color frame as a BGR numpy array, or None."""
        with self._lock:
            return self._color_frame.copy() if self._color_frame is not None else None

    def detect_aruco(
        self, dictionary_name: str = "DICT_4X4_50"
    ) -> list[dict]:
        """Detect ArUco markers in the current frame.

        Returns a list of dicts: [{"id": int, "center": (u, v), "corners": [(u,v),...]}]
        """
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
            pts = corners[0]  # (4, 2)
            cx = float(pts[:, 0].mean())
            cy = float(pts[:, 1].mean())
            results.append({
                "id": int(marker_id),
                "center": (cx, cy),
                "corners": [(float(p[0]), float(p[1])) for p in pts],
            })
        return results

    def deproject(self, u: int, v: int) -> tuple[float, float, float] | None:
        """Pixel (u, v) -> 3D point in camera frame (mm). None if no depth."""
        with self._lock:
            if self._depth_data is None or self._intrinsics is None:
                return None
            h, w = self._depth_data.shape
            if not (0 <= v < h and 0 <= u < w):
                return None
            depth_m = self._depth_data[v, u] * self._depth_scale
        if depth_m <= 0:
            return None
        pt = rs.rs2_deproject_pixel_to_point(self._intrinsics, [u, v], depth_m)  # type: ignore[union-attr]
        return (pt[0] * 1000, pt[1] * 1000, pt[2] * 1000)

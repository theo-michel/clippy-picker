"""YOLO-based object detection for pick-and-place."""

from __future__ import annotations

import logging
from dataclasses import dataclass
from pathlib import Path

import numpy as np

log = logging.getLogger(__name__)

DEFAULT_WEIGHTS = Path(__file__).parent / "runs" / "rpi_case" / "weights" / "best.pt"
FALLBACK_WEIGHTS = "yolo11n.pt"


@dataclass
class Detection:
    center_uv: tuple[float, float]
    bbox: tuple[int, int, int, int]  # x1, y1, x2, y2
    label: str
    confidence: float


class ObjectDetector:
    """Lazy-loading YOLO wrapper.

    Uses custom weights from ``models/best.pt`` when available,
    otherwise falls back to the pretrained YOLOv11n COCO checkpoint.
    """

    def __init__(
        self,
        weights: str | Path | None = None,
        conf: float = 0.5,
    ):
        self._weights = str(weights or (DEFAULT_WEIGHTS if DEFAULT_WEIGHTS.exists() else FALLBACK_WEIGHTS))
        self._conf = conf
        self._model = None

    def _load(self):
        from ultralytics import YOLO

        log.info("Loading YOLO weights: %s", self._weights)
        self._model = YOLO(self._weights)

    def detect(self, frame: np.ndarray, *, conf: float | None = None) -> list[Detection]:
        """Run inference on a BGR frame and return detections."""
        if self._model is None:
            self._load()

        results = self._model(frame, conf=conf or self._conf, verbose=False)
        detections: list[Detection] = []
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2
                cls_id = int(box.cls[0])
                label = r.names.get(cls_id, str(cls_id))
                detections.append(
                    Detection(
                        center_uv=(cx, cy),
                        bbox=(int(x1), int(y1), int(x2), int(y2)),
                        label=label,
                        confidence=float(box.conf[0]),
                    )
                )
        return detections

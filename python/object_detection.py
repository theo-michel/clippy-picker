"""Gemini Flash VLM-based object *pointing*.

The model is asked to point at an object described in natural language and
return a single pixel coordinate (not a bounding box).  The pick pipeline
then deprojects that pixel through the full-resolution stereo depth map.
"""

from __future__ import annotations

import json
import logging
import os
import re
from dataclasses import dataclass

import cv2
import numpy as np

log = logging.getLogger(__name__)

DEFAULT_MODEL = "gemini-3-flash-preview"


@dataclass
class Point:
    """A 2D point in image pixel coordinates.

    ``u`` is the column, ``v`` is the row.  ``label`` echoes back what the
    VLM thinks it pointed at (useful for logging / UI).
    """

    u: float
    v: float
    label: str = ""


class VLMError(RuntimeError):
    """Raised when the VLM call fails or returns an unparseable response."""


class ObjectPointer:
    """Thin wrapper around the Gemini Flash pointing API.

    The client is constructed lazily so importing this module does not
    require an API key or network access.
    """

    def __init__(self, *, model: str | None = None, api_key: str | None = None) -> None:
        self._model = model or os.environ.get("GEMINI_MODEL", DEFAULT_MODEL)
        self._api_key = api_key or os.environ.get("GEMINI_API_KEY")
        self._client = None

    def _ensure_client(self) -> None:
        if self._client is not None:
            return
        if not self._api_key:
            raise VLMError(
                "GEMINI_API_KEY is not set. Export it or pass api_key=... to ObjectPointer()."
            )
        from google import genai

        log.info("Initialising Gemini client (model=%s)", self._model)
        self._client = genai.Client(api_key=self._api_key)

    def point(self, frame: np.ndarray, description: str) -> Point | None:
        """Ask the VLM to point at ``description`` in ``frame``.

        Returns ``None`` if the model cannot locate the object.
        """
        if not description or not description.strip():
            raise ValueError("description must be a non-empty string")

        self._ensure_client()

        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if not ok:
            raise VLMError("Failed to JPEG-encode frame for VLM")

        from google.genai import types

        prompt = (
            f"Point at the {description.strip()} in the image.\n"
            "Respond with a single JSON object of the form "
            '{"point": [y, x], "label": "..."} where y and x are normalised '
            "to the 0–1000 range (y is the row, x is the column). "
            "If the object is not visible, respond with null. "
            "Return JSON only, no prose and no markdown fences."
        )

        image_part = types.Part.from_bytes(data=buf.tobytes(), mime_type="image/jpeg")
        resp = self._client.models.generate_content(
            model=self._model,
            contents=[image_part, prompt],
            config=types.GenerateContentConfig(response_mime_type="application/json"),
        )

        text = (resp.text or "").strip()
        log.debug("VLM raw response: %s", text)

        parsed = _parse_point(text)
        if parsed is None:
            log.warning("VLM did not return a point for %r (raw=%r)", description, text)
            return None

        y_norm, x_norm, label = parsed
        h, w = frame.shape[:2]
        u = float(np.clip(x_norm / 1000.0 * w, 0, w - 1))
        v = float(np.clip(y_norm / 1000.0 * h, 0, h - 1))
        return Point(u=u, v=v, label=label or description.strip())


def _parse_point(text: str) -> tuple[float, float, str] | None:
    """Extract (y, x, label) from a Gemini pointing response.

    Accepts either a single object ``{"point": [y, x], "label": ...}`` or a
    list of such objects; in the list case the first entry wins.
    """
    if not text or text.lower() == "null":
        return None

    # Strip ```json fences just in case the model ignores the instruction.
    text = re.sub(r"^```(?:json)?\s*|\s*```$", "", text.strip(), flags=re.MULTILINE)

    try:
        data = json.loads(text)
    except json.JSONDecodeError:
        m = re.search(r"[\[{].*[\]}]", text, re.DOTALL)
        if not m:
            return None
        try:
            data = json.loads(m.group(0))
        except json.JSONDecodeError:
            return None

    if isinstance(data, list):
        if not data:
            return None
        data = data[0]
    if not isinstance(data, dict):
        return None

    pt = data.get("point") or data.get("points")
    if not isinstance(pt, (list, tuple)) or len(pt) < 2:
        return None

    try:
        y = float(pt[0])
        x = float(pt[1])
    except (TypeError, ValueError):
        return None

    return y, x, str(data.get("label", ""))

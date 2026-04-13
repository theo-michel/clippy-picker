#!/usr/bin/env python3
"""Capture training images from the RealSense camera for YOLO labeling.

Shows a live preview.  Press SPACE to save the current frame,
Q or ESC to quit.  Images are saved as numbered PNGs in the output
directory (default: ``dataset/images/``).
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=Path(__file__).resolve().parent.parent / "dataset" / "images",
        help="Directory to save captured images (default: dataset/images/)",
    )
    args = parser.parse_args()

    out_dir: Path = args.output
    out_dir.mkdir(parents=True, exist_ok=True)

    existing = sorted(out_dir.glob("*.png"))
    next_idx = int(existing[-1].stem) + 1 if existing else 0

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    print(f"Saving to {out_dir}/")
    print("SPACE = capture, Q/ESC = quit")

    try:
        while True:
            ok, frames = pipeline.try_wait_for_frames(5000)
            if not ok:
                continue
            color = frames.get_color_frame()
            if not color:
                continue

            img = np.asanyarray(color.get_data())

            display = img.copy()
            cv2.putText(
                display,
                f"Saved: {next_idx}  |  SPACE=capture  Q=quit",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )
            cv2.imshow("Capture Training Images", display)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord(" "):
                path = out_dir / f"{next_idx:04d}.png"
                cv2.imwrite(str(path), img)
                print(f"  saved {path.name}")
                next_idx += 1
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

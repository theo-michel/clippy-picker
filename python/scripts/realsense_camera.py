#!/usr/bin/env python3
"""
Live RealSense depth camera feed. Shows center pixel XYZ in mm.
Press Q or ESC to quit.
"""

from __future__ import annotations

import cv2
import numpy as np
import pyrealsense2 as rs


def main() -> None:
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    profile = pipeline.start(config)
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
    align = rs.align(rs.stream.color)

    try:
        while True:
            success, frames = pipeline.try_wait_for_frames(5000)
            if not success:
                continue

            aligned = align.process(frames)
            depth_frame = aligned.get_depth_frame()
            color_frame = aligned.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            cy, cx = color_image.shape[0] // 2, color_image.shape[1] // 2
            depth_m = depth_image[cy, cx] * depth_scale
            intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
            x, y, z = rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], depth_m)

            label = f"X={x*1000:.0f}  Y={y*1000:.0f}  Z={z*1000:.0f} mm"
            cv2.drawMarker(color_image, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
            cv2.putText(color_image, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            cv2.imshow("RealSense", color_image)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

from __future__ import annotations

import cv2

from .config import CameraConfig


def open_camera(config: CameraConfig) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(config.device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.height)
    cap.set(cv2.CAP_PROP_FPS, config.fps)

    if not cap.isOpened():
        raise RuntimeError(f"cannot open camera: {config.device}")
    return cap

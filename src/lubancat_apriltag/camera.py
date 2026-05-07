from __future__ import annotations

import cv2

from .config import CameraConfig


def open_camera(config: CameraConfig) -> cv2.VideoCapture:
    backend = cv2.CAP_ANY
    if config.backend.lower() == "v4l2":
        backend = cv2.CAP_V4L2

    cap = cv2.VideoCapture(config.device, backend)
    if config.fourcc:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*config.fourcc[:4]))
    if config.buffer_size > 0:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, config.buffer_size)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.height)
    cap.set(cv2.CAP_PROP_FPS, config.fps)

    if not cap.isOpened():
        raise RuntimeError(f"cannot open camera: {config.device}")
    return cap

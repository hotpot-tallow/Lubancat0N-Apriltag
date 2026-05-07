from __future__ import annotations

import cv2

from .config import CameraConfig


def _fourcc_text(value: float) -> str:
    code = int(value)
    chars = []
    for _ in range(4):
        chars.append(chr(code & 0xFF))
        code >>= 8
    text = "".join(chars)
    return text if text.strip("\x00") else "unknown"


def _backend_id(name: str) -> int:
    if name.lower() == "v4l2":
        return cv2.CAP_V4L2
    return cv2.CAP_ANY


def _apply_camera_options(cap: cv2.VideoCapture, config: CameraConfig, use_optional: bool) -> None:
    if use_optional and config.fourcc:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*config.fourcc[:4]))
    if use_optional and config.buffer_size > 0:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, config.buffer_size)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.height)
    cap.set(cv2.CAP_PROP_FPS, config.fps)


def open_camera(config: CameraConfig) -> cv2.VideoCapture:
    attempts = [
        (_backend_id(config.backend), True),
        (_backend_id(config.backend), False),
        (cv2.CAP_ANY, False),
    ]

    for backend, use_optional in attempts:
        cap = cv2.VideoCapture(config.device, backend)
        _apply_camera_options(cap, config, use_optional=use_optional)
        if cap.isOpened():
            return cap
        cap.release()

    raise RuntimeError(f"cannot open camera: {config.device}")


def camera_info(cap: cv2.VideoCapture) -> str:
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    fourcc = _fourcc_text(cap.get(cv2.CAP_PROP_FOURCC))
    return f"{width}x{height} fps={fps:.1f} fourcc={fourcc}"

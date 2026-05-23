from __future__ import annotations

import cv2

from .config import CameraConfig


def _fourcc_text(value: float) -> str:
    """把 OpenCV 返回的 FOURCC 整数转换成人能读懂的编码名。"""
    code = int(value)
    chars = []
    for _ in range(4):
        chars.append(chr(code & 0xFF))
        code >>= 8
    text = "".join(chars)
    return text if text.strip("\x00") else "unknown"


def _backend_id(name: str) -> int:
    """根据配置选择 OpenCV 摄像头后端，当前主要支持 Linux V4L2。"""
    if name.lower() == "v4l2":
        return cv2.CAP_V4L2
    return cv2.CAP_ANY


def _apply_camera_options(cap: cv2.VideoCapture, config: CameraConfig, use_optional: bool) -> None:
    """把配置里的分辨率、帧率、缓存和像素格式写入摄像头。"""
    # fourcc/buffer_size 有些 MIPI 摄像头不支持，所以允许在第二次尝试时跳过。
    if use_optional and config.fourcc:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*config.fourcc[:4]))
    if use_optional and config.buffer_size > 0:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, config.buffer_size)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.height)
    cap.set(cv2.CAP_PROP_FPS, config.fps)


def open_camera(config: CameraConfig) -> cv2.VideoCapture:
    """按配置打开摄像头，失败时自动退回到更保守的打开方式。"""
    attempts = [
        (_backend_id(config.backend), True),
        (_backend_id(config.backend), False),
        (cv2.CAP_ANY, False),
    ]

    for backend, use_optional in attempts:
        # 同一个摄像头尝试多种方式，尽量兼容 USB 摄像头和板载 MIPI 摄像头。
        cap = cv2.VideoCapture(config.device, backend)
        _apply_camera_options(cap, config, use_optional=use_optional)
        if cap.isOpened():
            return cap
        cap.release()

    raise RuntimeError(f"cannot open camera: {config.device}")


def camera_info(cap: cv2.VideoCapture) -> str:
    """返回当前实际打开到的摄像头参数，便于调试分辨率/帧率是否生效。"""
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    fourcc = _fourcc_text(cap.get(cv2.CAP_PROP_FOURCC))
    return f"{width}x{height} fps={fps:.1f} fourcc={fourcc}"

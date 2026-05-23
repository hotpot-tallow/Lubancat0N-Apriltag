from __future__ import annotations

import threading
import time
from typing import Optional, Tuple

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
    normalized = name.lower()
    if normalized == "v4l2":
        return cv2.CAP_V4L2
    if normalized in ("gstreamer", "gst"):
        return cv2.CAP_GSTREAMER
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


def _open_raw_camera(config: CameraConfig) -> cv2.VideoCapture:
    """按配置打开一条原始 OpenCV VideoCapture，失败时自动退回到更保守的打开方式。"""
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


class ResilientCamera:
    """用后台线程读取摄像头，避免主循环直接卡死在 OpenCV 的 cap.read() 里。

    MIPI/RKISP 摄像头偶尔会让底层 read 阻塞很久，Python 层无法给 cap.read()
    设置可靠超时。这里让后台线程负责阻塞读取，主线程只等待最新帧；如果超过
    read_timeout_s 没有新帧，就释放并重开摄像头。
    """

    def __init__(self, config: CameraConfig, read_timeout_s: float = 2.0) -> None:
        self.config = config
        self.read_timeout_s = max(float(read_timeout_s), 0.1)
        self._condition = threading.Condition()
        self._cap: Optional[cv2.VideoCapture] = None
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._frame = None
        self._seq = 0
        self._delivered_seq = 0
        self._last_restart_time = 0.0
        self._open_and_start()

    def _open_and_start(self) -> None:
        """打开摄像头并启动后台读帧线程。"""
        self._stop_event = threading.Event()
        self._cap = _open_raw_camera(self.config)
        self._thread = threading.Thread(target=self._reader_loop, name="camera-reader", daemon=True)
        self._thread.start()

    def _reader_loop(self) -> None:
        """后台持续读取帧，只保留最新一帧，避免缓冲堆积导致延迟。"""
        while not self._stop_event.is_set():
            cap = self._cap
            if cap is None:
                time.sleep(0.05)
                continue
            try:
                ok, frame = cap.read()
            except cv2.error as exc:
                print(f"camera read error: {exc}")
                time.sleep(0.1)
                continue
            if not ok:
                time.sleep(0.02)
                continue
            with self._condition:
                self._frame = frame
                self._seq += 1
                self._condition.notify_all()

    def _restart(self) -> None:
        """读帧超时后重启摄像头，尽量把底层阻塞的取流链路拉回来。"""
        now = time.monotonic()
        if now - self._last_restart_time < 1.0:
            return
        self._last_restart_time = now
        print(f"camera read timeout after {self.read_timeout_s:.1f}s; restarting camera")

        self._stop_event.set()
        old_cap = self._cap
        self._cap = None
        if old_cap is not None:
            old_cap.release()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

        with self._condition:
            self._frame = None
            self._delivered_seq = self._seq

        try:
            self._open_and_start()
        except RuntimeError as exc:
            print(f"camera restart failed: {exc}")

    def read(self) -> Tuple[bool, object]:
        """返回最新一帧；如果超时没有新帧，重启摄像头并返回失败。"""
        deadline = time.monotonic() + self.read_timeout_s
        with self._condition:
            while self._seq <= self._delivered_seq:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self._condition.wait(timeout=remaining)
            if self._seq > self._delivered_seq and self._frame is not None:
                self._delivered_seq = self._seq
                return True, self._frame

        self._restart()
        return False, None

    def get(self, prop_id: int) -> float:
        """兼容 cv2.VideoCapture.get，用于打印实际分辨率、帧率等信息。"""
        cap = self._cap
        return 0.0 if cap is None else float(cap.get(prop_id))

    def isOpened(self) -> bool:
        """兼容 cv2.VideoCapture.isOpened。"""
        cap = self._cap
        return False if cap is None else bool(cap.isOpened())

    def release(self) -> None:
        """停止后台线程并释放摄像头。"""
        self._stop_event.set()
        cap = self._cap
        self._cap = None
        if cap is not None:
            cap.release()
        if self._thread is not None:
            self._thread.join(timeout=1.0)


def open_camera(config: CameraConfig, read_timeout_s: float = 2.0) -> ResilientCamera:
    """打开带超时保护的摄像头。"""
    return ResilientCamera(config, read_timeout_s=read_timeout_s)


def camera_info(cap) -> str:
    """返回当前实际打开到的摄像头参数，便于调试分辨率/帧率是否生效。"""
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    fourcc = _fourcc_text(cap.get(cv2.CAP_PROP_FOURCC))
    return f"{width}x{height} fps={fps:.1f} fourcc={fourcc}"

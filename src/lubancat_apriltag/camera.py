from __future__ import annotations

import multiprocessing as mp
import shlex
import subprocess
import threading
import time
from queue import Empty, Full, Queue
from typing import Optional, Tuple

import cv2
import numpy as np

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


def _is_gstreamer_pipeline(config: CameraConfig) -> bool:
    """判断当前 device 是否是一整条 GStreamer 管线字符串。"""
    return config.backend.lower() in ("gstreamer", "gst") and isinstance(config.device, str)


def _apply_camera_options(cap: cv2.VideoCapture, config: CameraConfig, use_optional: bool) -> None:
    """把配置里的分辨率、帧率、缓存和像素格式写入摄像头。"""
    # GStreamer 管线里已经明确写了 format/width/height/framerate/appsink。
    # 再调用 cap.set(...) 会触发 OpenCV 的 unhandled property 警告，部分板载相机还会因此取流不稳定。
    if _is_gstreamer_pipeline(config):
        return

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
    if _is_gstreamer_pipeline(config):
        # 管线字符串只能交给 GStreamer 后端；不要再把它交给 CAP_ANY/V4L2 乱试。
        attempts = [(cv2.CAP_GSTREAMER, False)]
    else:
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


class DirectCamera:
    """直接读取 OpenCV VideoCapture，主要用于已经验证稳定的 GStreamer 管线。

    GStreamer 管线自身已经用 appsink drop/max-buffers 控制缓冲。这里不再用后台线程
    反复 release/reopen，避免和 OpenCV/GStreamer 的内部线程在退出或重连时互相影响。
    """

    def __init__(self, config: CameraConfig) -> None:
        self.config = config
        self._cap = _open_raw_camera(config)

    def read(self) -> Tuple[bool, object]:
        """直接返回下一帧。"""
        return self._cap.read()

    def get(self, prop_id: int) -> float:
        """兼容 cv2.VideoCapture.get。GStreamer 管线不查询 FOURCC，避免无意义警告。"""
        if _is_gstreamer_pipeline(self.config) and prop_id == cv2.CAP_PROP_FOURCC:
            return 0.0
        return float(self._cap.get(prop_id))

    def isOpened(self) -> bool:
        """兼容 cv2.VideoCapture.isOpened。"""
        return bool(self._cap.isOpened())

    def release(self) -> None:
        """释放摄像头。"""
        self._cap.release()


def _camera_process_main(
    config: CameraConfig,
    frame_queue,
    info_queue,
    stop_event,
) -> None:
    """在独立进程里读取摄像头；如果底层 cap.read() 卡死，不会拖死主进程。"""
    cap = None
    try:
        cap = _open_raw_camera(config)
        info_queue.put(
            (
                int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                float(cap.get(cv2.CAP_PROP_FPS)),
            )
        )
        while not stop_event.is_set():
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.02)
                continue

            # 队列里只保留最新帧，主进程处理慢时直接丢旧帧，避免延迟越堆越大。
            while True:
                try:
                    frame_queue.get_nowait()
                except Empty:
                    break
            try:
                frame_queue.put_nowait(frame)
            except Full:
                pass
    finally:
        if cap is not None:
            cap.release()


class ProcessCamera:
    """把 GStreamer/OpenCV 取流隔离到子进程，避免 C 层 read() 卡住主进程。

    这比线程更适合不稳定的 MIPI/GStreamer 链路：子进程卡死时，主进程可以 terminate
    它并重启；Ctrl+C 也能先回到主进程处理。
    """

    def __init__(self, config: CameraConfig, read_timeout_s: float = 2.0) -> None:
        self.config = config
        self.read_timeout_s = max(float(read_timeout_s), 0.1)
        self._ctx = mp.get_context()
        self._frame_queue = None
        self._info_queue = None
        self._stop_event = None
        self._process = None
        self._info = (config.width, config.height, float(config.fps))
        self._start_process()

    def _start_process(self) -> None:
        """启动摄像头子进程。"""
        self._frame_queue = self._ctx.Queue(maxsize=1)
        self._info_queue = self._ctx.Queue(maxsize=1)
        self._stop_event = self._ctx.Event()
        self._process = self._ctx.Process(
            target=_camera_process_main,
            args=(self.config, self._frame_queue, self._info_queue, self._stop_event),
            daemon=True,
        )
        self._process.start()
        try:
            self._info = self._info_queue.get(timeout=self.read_timeout_s)
        except Empty:
            print("camera process started, but no camera info was returned yet")

    def _stop_process(self) -> None:
        """停止摄像头子进程；正常停不下来就强制结束。"""
        if self._stop_event is not None:
            self._stop_event.set()
        if self._process is not None:
            self._process.join(timeout=1.0)
            if self._process.is_alive():
                self._process.terminate()
                self._process.join(timeout=1.0)
        self._process = None

    def _restart(self) -> None:
        """超时未收到新帧时重启摄像头子进程。"""
        print(f"camera process timeout after {self.read_timeout_s:.1f}s; restarting camera process")
        self._stop_process()
        self._start_process()

    def read(self) -> Tuple[bool, object]:
        """从子进程获取最新帧；等待超时则重启子进程。"""
        try:
            return True, self._frame_queue.get(timeout=self.read_timeout_s)
        except Empty:
            self._restart()
            return False, None

    def get(self, prop_id: int) -> float:
        """兼容 cv2.VideoCapture.get。"""
        width, height, fps = self._info
        if prop_id == cv2.CAP_PROP_FRAME_WIDTH:
            return float(width)
        if prop_id == cv2.CAP_PROP_FRAME_HEIGHT:
            return float(height)
        if prop_id == cv2.CAP_PROP_FPS:
            return float(fps)
        return 0.0

    def isOpened(self) -> bool:
        """摄像头子进程是否仍在运行。"""
        return self._process is not None and self._process.is_alive()

    def release(self) -> None:
        """释放摄像头子进程。"""
        self._stop_process()


def _gst_stdout_command(config: CameraConfig) -> list:
    """把配置里的 GStreamer 管线改成向 stdout 输出 BGR 原始帧。"""
    pipeline = str(config.device).strip()
    if "! appsink" in pipeline:
        pipeline = pipeline.split("! appsink", 1)[0].strip()
    if "format=BGR" not in pipeline:
        pipeline = f"{pipeline} ! videoconvert ! video/x-raw,format=BGR"
    # 用 fdsink 直接吐原始帧，绕开 OpenCV 的 GStreamer VideoCapture 后端。
    return ["gst-launch-1.0", "-q"] + shlex.split(pipeline) + ["!", "fdsink", "fd=1", "sync=false"]


def _read_exact(stream, size: int) -> bytes:
    """从管道读取固定长度字节；读不到完整一帧就返回当前已读数据。"""
    chunks = []
    remaining = size
    while remaining > 0:
        chunk = stream.read(remaining)
        if not chunk:
            break
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


class GstLaunchCamera:
    """用 gst-launch-1.0 取流，Python 只读取 stdout 中的 BGR 原始帧。

    这条路径不使用 OpenCV 的 GStreamer 后端，适合 gst-launch 单独稳定、
    但 cv2.VideoCapture(..., CAP_GSTREAMER).read() 会卡死的板载 MIPI 摄像头。
    """

    def __init__(self, config: CameraConfig, read_timeout_s: float = 2.0) -> None:
        self.config = config
        self.read_timeout_s = max(float(read_timeout_s), 0.1)
        self.width = int(config.width)
        self.height = int(config.height)
        self.fps = float(config.fps)
        self._frame_size = self.width * self.height * 3
        self._queue: Queue = Queue(maxsize=1)
        self._stop_event = threading.Event()
        self._process: Optional[subprocess.Popen] = None
        self._thread: Optional[threading.Thread] = None
        self._start()

    def _start(self) -> None:
        """启动 gst-launch 子进程和 stdout 读帧线程。"""
        self._stop_event = threading.Event()
        command = _gst_stdout_command(self.config)
        self._process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=None,
            bufsize=self._frame_size * 2,
        )
        self._thread = threading.Thread(target=self._reader_loop, name="gst-launch-reader", daemon=True)
        self._thread.start()

    def _reader_loop(self) -> None:
        """持续从 gst-launch stdout 读取 BGR 帧，只保留最新帧。"""
        assert self._process is not None
        assert self._process.stdout is not None
        while not self._stop_event.is_set():
            data = _read_exact(self._process.stdout, self._frame_size)
            if len(data) != self._frame_size:
                break
            frame = np.frombuffer(data, dtype=np.uint8).reshape((self.height, self.width, 3)).copy()
            while True:
                try:
                    self._queue.get_nowait()
                except Empty:
                    break
            try:
                self._queue.put_nowait(frame)
            except Full:
                pass

    def _stop(self) -> None:
        """停止 gst-launch 子进程。"""
        self._stop_event.set()
        if self._process is not None:
            self._process.terminate()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._process is not None and self._process.poll() is None:
            self._process.kill()
            self._process.wait(timeout=1.0)
        self._process = None
        self._thread = None

    def _restart(self) -> None:
        """超时没有新帧时重启 gst-launch 进程。"""
        print(f"gst-launch frame timeout after {self.read_timeout_s:.1f}s; restarting gst-launch")
        self._stop()
        while True:
            try:
                self._queue.get_nowait()
            except Empty:
                break
        self._start()

    def read(self) -> Tuple[bool, object]:
        """返回最新帧；如果 gst-launch 停止吐帧，则重启取流进程。"""
        try:
            return True, self._queue.get(timeout=self.read_timeout_s)
        except Empty:
            self._restart()
            return False, None

    def get(self, prop_id: int) -> float:
        """兼容 cv2.VideoCapture.get。"""
        if prop_id == cv2.CAP_PROP_FRAME_WIDTH:
            return float(self.width)
        if prop_id == cv2.CAP_PROP_FRAME_HEIGHT:
            return float(self.height)
        if prop_id == cv2.CAP_PROP_FPS:
            return float(self.fps)
        return 0.0

    def isOpened(self) -> bool:
        """gst-launch 进程是否仍在运行。"""
        return self._process is not None and self._process.poll() is None

    def release(self) -> None:
        """释放 gst-launch 进程。"""
        self._stop()


def open_camera(config: CameraConfig, read_timeout_s: float = 2.0):
    """打开摄像头。

    V4L2 直连模式保留后台线程超时保护；GStreamer 管线模式用子进程隔离 C 层阻塞。
    """
    if _is_gstreamer_pipeline(config):
        return GstLaunchCamera(config, read_timeout_s=read_timeout_s)
    return ResilientCamera(config, read_timeout_s=read_timeout_s)


def camera_info(cap) -> str:
    """返回当前实际打开到的摄像头参数，便于调试分辨率/帧率是否生效。"""
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    fourcc = _fourcc_text(cap.get(cv2.CAP_PROP_FOURCC))
    return f"{width}x{height} fps={fps:.1f} fourcc={fourcc}"

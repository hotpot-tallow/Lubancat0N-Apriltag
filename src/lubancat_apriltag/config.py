from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Sequence, Tuple, Union


@dataclass(frozen=True)
class CameraConfig:
    """摄像头采集参数和内参。fx/fy/cx/cy 必须和运行分辨率对应。"""

    device: Union[int, str]
    backend: str
    fourcc: str
    buffer_size: int
    width: int
    height: int
    fps: int
    fx: float
    fy: float
    cx: float
    cy: float

    @property
    def params(self) -> Tuple[float, float, float, float]:
        """返回 OpenCV/PnP 常用的相机内参顺序。"""
        return self.fx, self.fy, self.cx, self.cy


@dataclass(frozen=True)
class AprilTagFamilyConfig:
    """一个 AprilTag 家族及该家族中需要跟踪的标签尺寸。"""

    name: str
    tag_sizes_m: Dict[int, float]
    bits_corrected: int
    max_codes: Optional[int]


@dataclass(frozen=True)
class AprilTagConfig:
    """AprilTag 检测参数；多个家族共用一次四边形搜索。"""

    families: Tuple[AprilTagFamilyConfig, ...]
    nthreads: int
    quad_decimate: float
    quad_sigma: float
    refine_edges: int
    decode_sharpening: float
    max_hamming: int
    min_decision_margin: float


@dataclass(frozen=True)
class MavlinkConfig:
    """MAVLink 串口和 LANDING_TARGET 发送参数。"""

    connection: str
    baud: int
    source_system: int
    source_component: int
    target_num: int
    send_rate_hz: float


@dataclass(frozen=True)
class AppConfig:
    """程序运行所需的完整配置。"""

    camera: CameraConfig
    apriltag: AprilTagConfig
    mavlink: MavlinkConfig
    camera_to_body: Tuple[Tuple[float, float, float], ...]


def _matrix3(value: Sequence[Sequence[float]]) -> Tuple[Tuple[float, float, float], ...]:
    """读取并校验 camera_to_body 坐标变换矩阵。"""
    rows = tuple(tuple(float(item) for item in row) for row in value)
    if len(rows) != 3 or any(len(row) != 3 for row in rows):
        raise ValueError("transform.camera_to_body must be a 3x3 matrix")
    return rows


def _tag_sizes(value: Dict[str, float], family_name: str) -> Dict[int, float]:
    sizes = {int(tag_id): float(size) for tag_id, size in value.items()}
    if not sizes:
        raise ValueError(f"apriltag family {family_name} must configure at least one tag size")
    if any(tag_id < 0 for tag_id in sizes):
        raise ValueError(f"apriltag family {family_name} contains a negative tag id")
    if any(size <= 0.0 for size in sizes.values()):
        raise ValueError(f"apriltag family {family_name} contains a non-positive tag size")
    return sizes


def _tag_families(value: dict) -> Tuple[AprilTagFamilyConfig, ...]:
    raw_families = value.get("families")
    if raw_families is None:
        name = str(value.get("family", "tag36h11"))
        bits_corrected = int(value.get("bits_corrected", 2))
        if bits_corrected < 0 or bits_corrected > 2:
            raise ValueError("apriltag.bits_corrected must be between 0 and 2")
        max_codes_value = value.get("max_codes")
        max_codes = None if max_codes_value is None else int(max_codes_value)
        if max_codes is not None and max_codes <= 0:
            raise ValueError("apriltag.max_codes must be positive")
        tag_sizes = _tag_sizes(value["tag_sizes_m"], name)
        if max_codes is not None and any(tag_id >= max_codes for tag_id in tag_sizes):
            raise ValueError("apriltag tag id must be lower than max_codes")
        return (
            AprilTagFamilyConfig(
                name,
                tag_sizes,
                bits_corrected,
                max_codes,
            ),
        )

    if not isinstance(raw_families, list) or not raw_families:
        raise ValueError("apriltag.families must be a non-empty list")

    families = []
    seen_names = set()
    for item in raw_families:
        name = str(item["name"])
        if name in seen_names:
            raise ValueError(f"duplicate apriltag family: {name}")
        seen_names.add(name)
        default_bits = 0 if name == "tagCustom48h12" else 2
        bits_corrected = int(item.get("bits_corrected", default_bits))
        if bits_corrected < 0 or bits_corrected > 2:
            raise ValueError(f"apriltag family {name} bits_corrected must be between 0 and 2")
        max_codes_value = item.get("max_codes")
        max_codes = None if max_codes_value is None else int(max_codes_value)
        if max_codes is not None and max_codes <= 0:
            raise ValueError(f"apriltag family {name} max_codes must be positive")
        tag_sizes = _tag_sizes(item["tag_sizes_m"], name)
        if max_codes is not None and any(tag_id >= max_codes for tag_id in tag_sizes):
            raise ValueError(f"apriltag family {name} tag id must be lower than max_codes")
        families.append(
            AprilTagFamilyConfig(
                name,
                tag_sizes,
                bits_corrected,
                max_codes,
            )
        )
    return tuple(families)


def load_config(path: Union[str, Path]) -> AppConfig:
    """从 JSON 文件加载配置，并转换成带类型的 dataclass。"""
    with Path(path).open("r", encoding="utf-8") as fp:
        raw = json.load(fp)

    camera = raw["camera"]
    apriltag = raw["apriltag"]
    mavlink = raw["mavlink"]
    transform = raw["transform"]

    return AppConfig(
        camera=CameraConfig(
            device=camera.get("device", 0),
            backend=str(camera.get("backend", "v4l2")),
            fourcc=str(camera.get("fourcc", "MJPG")),
            buffer_size=int(camera.get("buffer_size", 1)),
            width=int(camera.get("width", 640)),
            height=int(camera.get("height", 480)),
            fps=int(camera.get("fps", 30)),
            fx=float(camera["fx"]),
            fy=float(camera["fy"]),
            cx=float(camera["cx"]),
            cy=float(camera["cy"]),
        ),
        apriltag=AprilTagConfig(
            families=_tag_families(apriltag),
            nthreads=max(1, int(apriltag.get("nthreads", 2))),
            quad_decimate=float(apriltag.get("quad_decimate", 2.0)),
            quad_sigma=float(apriltag.get("quad_sigma", 0.0)),
            refine_edges=int(apriltag.get("refine_edges", 1)),
            decode_sharpening=float(apriltag.get("decode_sharpening", 0.25)),
            max_hamming=int(apriltag.get("max_hamming", 1)),
            min_decision_margin=float(apriltag.get("min_decision_margin", 8.0)),
        ),
        mavlink=MavlinkConfig(
            connection=str(mavlink["connection"]),
            baud=int(mavlink.get("baud", 115200)),
            source_system=int(mavlink.get("source_system", 42)),
            source_component=int(mavlink.get("source_component", 191)),
            target_num=int(mavlink.get("target_num", 0)),
            send_rate_hz=float(mavlink.get("send_rate_hz", 20.0)),
        ),
        camera_to_body=_matrix3(transform["camera_to_body"]),
    )

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Sequence, Tuple, Union


@dataclass(frozen=True)
class CameraConfig:
    device: Union[int, str]
    width: int
    height: int
    fps: int
    fx: float
    fy: float
    cx: float
    cy: float

    @property
    def params(self) -> Tuple[float, float, float, float]:
        return self.fx, self.fy, self.cx, self.cy


@dataclass(frozen=True)
class AprilTagConfig:
    family: str
    tag_sizes_m: Dict[int, float]
    tag_priority: List[int]
    quad_decimate: float
    quad_sigma: float
    refine_edges: int
    decode_sharpening: float
    max_hamming: int
    min_decision_margin: float


@dataclass(frozen=True)
class MavlinkConfig:
    connection: str
    baud: int
    source_system: int
    source_component: int
    target_num: int
    send_rate_hz: float


@dataclass(frozen=True)
class AppConfig:
    camera: CameraConfig
    apriltag: AprilTagConfig
    mavlink: MavlinkConfig
    camera_to_body: Tuple[Tuple[float, float, float], ...]


def _matrix3(value: Sequence[Sequence[float]]) -> Tuple[Tuple[float, float, float], ...]:
    rows = tuple(tuple(float(item) for item in row) for row in value)
    if len(rows) != 3 or any(len(row) != 3 for row in rows):
        raise ValueError("transform.camera_to_body must be a 3x3 matrix")
    return rows


def load_config(path: Union[str, Path]) -> AppConfig:
    with Path(path).open("r", encoding="utf-8") as fp:
        raw = json.load(fp)

    camera = raw["camera"]
    apriltag = raw["apriltag"]
    mavlink = raw["mavlink"]
    transform = raw["transform"]

    tag_sizes = {int(tag_id): float(size) for tag_id, size in apriltag["tag_sizes_m"].items()}
    tag_priority = [int(tag_id) for tag_id in apriltag.get("tag_priority", tag_sizes.keys())]

    return AppConfig(
        camera=CameraConfig(
            device=camera.get("device", 0),
            width=int(camera.get("width", 640)),
            height=int(camera.get("height", 480)),
            fps=int(camera.get("fps", 30)),
            fx=float(camera["fx"]),
            fy=float(camera["fy"]),
            cx=float(camera["cx"]),
            cy=float(camera["cy"]),
        ),
        apriltag=AprilTagConfig(
            family=apriltag.get("family", "tag36h11"),
            tag_sizes_m=tag_sizes,
            tag_priority=tag_priority,
            quad_decimate=float(apriltag.get("quad_decimate", 2.0)),
            quad_sigma=float(apriltag.get("quad_sigma", 0.0)),
            refine_edges=int(apriltag.get("refine_edges", 1)),
            decode_sharpening=float(apriltag.get("decode_sharpening", 0.25)),
            max_hamming=int(apriltag.get("max_hamming", 1)),
            min_decision_margin=float(apriltag.get("min_decision_margin", 20.0)),
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

from __future__ import annotations

from dataclasses import dataclass
from math import sqrt
from typing import Tuple

import numpy as np


Vector3 = Tuple[float, float, float]
Matrix3 = Tuple[Tuple[float, float, float], ...]
Point2 = Tuple[float, float]


@dataclass(frozen=True)
class TargetPose:
    tag_id: int
    tag_size_m: float
    corners: Tuple[Point2, Point2, Point2, Point2]
    x_cam: float
    y_cam: float
    z_cam: float
    x_body: float
    y_body: float
    z_body: float
    distance_m: float
    decision_margin: float
    hamming: int


def transform_camera_to_body(camera_xyz: Vector3, matrix: Matrix3) -> Vector3:
    vec = np.array(camera_xyz, dtype=float)
    mat = np.array(matrix, dtype=float)
    body = mat @ vec
    return float(body[0]), float(body[1]), float(body[2])


def distance(xyz: Vector3) -> float:
    return sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1] + xyz[2] * xyz[2])

from __future__ import annotations

from dataclasses import dataclass
from math import sqrt
from typing import Tuple

import cv2
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


def estimate_pose_from_corners(
    corners: Tuple[Point2, Point2, Point2, Point2],
    tag_size_m: float,
    camera_params: Tuple[float, float, float, float],
) -> Vector3:
    half_size = tag_size_m / 2.0
    object_points = np.array(
        [
            [-half_size, half_size, 0.0],
            [half_size, half_size, 0.0],
            [half_size, -half_size, 0.0],
            [-half_size, -half_size, 0.0],
        ],
        dtype=np.float32,
    )
    image_points = np.array(corners, dtype=np.float32)
    fx, fy, cx, cy = camera_params
    camera_matrix = np.array(
        [
            [fx, 0.0, cx],
            [0.0, fy, cy],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float32,
    )
    distortion = np.zeros((4, 1), dtype=np.float32)

    try:
        ok, _, tvec = cv2.solvePnP(
            object_points,
            image_points,
            camera_matrix,
            distortion,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
    except cv2.error:
        ok, _, tvec = cv2.solvePnP(
            object_points,
            image_points,
            camera_matrix,
            distortion,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
    if not ok:
        raise RuntimeError("solvePnP failed")

    x_cam, y_cam, z_cam = tvec.flatten()
    return float(x_cam), float(y_cam), float(z_cam)

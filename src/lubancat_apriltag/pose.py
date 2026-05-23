from __future__ import annotations

from dataclasses import dataclass
from math import log, sqrt
from typing import Iterable, Tuple

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
    tag_pixel_width: float
    expected_z_m: float
    decision_margin: float
    hamming: int


def transform_camera_to_body(camera_xyz: Vector3, matrix: Matrix3) -> Vector3:
    vec = np.array(camera_xyz, dtype=float)
    mat = np.array(matrix, dtype=float)
    body = mat @ vec
    return float(body[0]), float(body[1]), float(body[2])


def distance(xyz: Vector3) -> float:
    return sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1] + xyz[2] * xyz[2])


def tag_pixel_width(corners: Tuple[Point2, Point2, Point2, Point2]) -> float:
    points = np.array(corners, dtype=np.float32)
    side_lengths = [
        float(np.linalg.norm(points[index] - points[(index + 1) % 4]))
        for index in range(4)
    ]
    return sum(side_lengths) / len(side_lengths)


def expected_z_from_pixels(
    corners: Tuple[Point2, Point2, Point2, Point2],
    tag_size_m: float,
    camera_params: Tuple[float, float, float, float],
) -> float:
    fx, fy, _, _ = camera_params
    width_px = max(tag_pixel_width(corners), 1.0)
    return ((fx + fy) * 0.5) * tag_size_m / width_px


def _corner_order_candidates(
    corners: Tuple[Point2, Point2, Point2, Point2],
) -> Iterable[Tuple[Point2, Point2, Point2, Point2]]:
    ordered = tuple(corners)
    reversed_ordered = tuple(reversed(ordered))
    for candidate in (ordered, reversed_ordered):
        for offset in range(4):
            yield candidate[offset:] + candidate[:offset]


def _reprojection_error(
    object_points: np.ndarray,
    image_points: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> float:
    projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, distortion)
    return float(cv2.norm(image_points, projected, cv2.NORM_L2) / sqrt(len(projected)))


def estimate_pose_from_corners(
    corners: Tuple[Point2, Point2, Point2, Point2],
    tag_size_m: float,
    camera_params: Tuple[float, float, float, float],
) -> Vector3:
    half_size = tag_size_m / 2.0
    object_points = np.array(
        [
            [-half_size, -half_size, 0.0],
            [half_size, -half_size, 0.0],
            [half_size, half_size, 0.0],
            [-half_size, half_size, 0.0],
        ],
        dtype=np.float32,
    )
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
    expected_z = max(expected_z_from_pixels(corners, tag_size_m, camera_params), 1e-6)
    best = None

    for ordered_corners in _corner_order_candidates(corners):
        image_points = np.array(ordered_corners, dtype=np.float32)
        for flag in (cv2.SOLVEPNP_IPPE, cv2.SOLVEPNP_ITERATIVE):
            try:
                ok, rvec, tvec = cv2.solvePnP(
                    object_points,
                    image_points,
                    camera_matrix,
                    distortion,
                    flags=flag,
                )
            except cv2.error:
                continue
            if not ok:
                continue
            z_cam = float(tvec.flatten()[2])
            if z_cam <= 0.0:
                continue
            reproj = _reprojection_error(
                object_points,
                image_points,
                rvec,
                tvec,
                camera_matrix,
                distortion,
            )
            ratio_penalty = abs(log(max(z_cam / expected_z, 1e-6)))
            score = reproj + ratio_penalty * 4.0
            if best is None or score < best[0]:
                best = (score, tvec)

    if best is None:
        raise RuntimeError("solvePnP failed")

    _, tvec = best
    x_cam, y_cam, z_cam = tvec.flatten()
    return float(x_cam), float(y_cam), float(z_cam)

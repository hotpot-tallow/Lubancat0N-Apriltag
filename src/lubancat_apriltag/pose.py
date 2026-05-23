from __future__ import annotations

from dataclasses import dataclass
from math import log, sqrt
from typing import Iterable, Tuple

import cv2
import numpy as np


Vector3 = Tuple[float, float, float]
Matrix3 = Tuple[Tuple[float, float, float], ...]
Point2 = Tuple[float, float]
Quaternion = Tuple[float, float, float, float]


@dataclass(frozen=True)
class PoseEstimate:
    """PnP 解算出的目标位姿，xyz 单位为米，q 顺序为 w/x/y/z。"""

    xyz: Vector3
    q: Quaternion


@dataclass(frozen=True)
class TargetPose:
    """一次 AprilTag 检测得到的完整位姿结果。"""

    tag_id: int
    tag_size_m: float
    corners: Tuple[Point2, Point2, Point2, Point2]
    x_cam: float
    y_cam: float
    z_cam: float
    x_body: float
    y_body: float
    z_body: float
    q_body: Quaternion
    distance_m: float
    tag_pixel_width: float
    expected_z_m: float
    decision_margin: float
    hamming: int


def transform_camera_to_body(camera_xyz: Vector3, matrix: Matrix3) -> Vector3:
    """把相机坐标系下的 xyz 转换到飞控使用的机体系坐标。"""
    vec = np.array(camera_xyz, dtype=float)
    mat = np.array(matrix, dtype=float)
    body = mat @ vec
    return float(body[0]), float(body[1]), float(body[2])


def transform_quaternion_camera_to_body(q_camera: Quaternion, matrix: Matrix3) -> Quaternion:
    """把目标姿态四元数从相机坐标系转换到机体系。"""
    rotation_camera = quaternion_to_matrix(q_camera)
    camera_to_body = np.array(matrix, dtype=float)
    rotation_body = camera_to_body @ rotation_camera
    return matrix_to_quaternion(rotation_body)


def distance(xyz: Vector3) -> float:
    """计算三维向量长度，单位保持为米。"""
    return sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1] + xyz[2] * xyz[2])


def tag_pixel_width(corners: Tuple[Point2, Point2, Point2, Point2]) -> float:
    """根据四个角点估算 tag 在图像中的平均边长，单位是像素。"""
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
    """用 pinhole 近似公式 fx * tag_size / pixel_width 粗估距离。"""
    fx, fy, _, _ = camera_params
    width_px = max(tag_pixel_width(corners), 1.0)
    return ((fx + fy) * 0.5) * tag_size_m / width_px


def _corner_order_candidates(
    corners: Tuple[Point2, Point2, Point2, Point2],
) -> Iterable[Tuple[Point2, Point2, Point2, Point2]]:
    """生成角点的不同起点/方向，避免检测库角点顺序和 solvePnP 假设不一致。"""
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
    """计算 PnP 解的重投影误差，误差越小表示角点越能对回原图。"""
    projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, distortion)
    projected_points = projected.reshape(-1, 2).astype(np.float32)
    image_points_2d = image_points.reshape(-1, 2).astype(np.float32)
    return float(cv2.norm(image_points_2d, projected_points, cv2.NORM_L2) / sqrt(len(projected_points)))


def matrix_to_quaternion(matrix: np.ndarray) -> Quaternion:
    """把 3x3 旋转矩阵转换成 MAVLink 使用的 w/x/y/z 四元数。"""
    mat = matrix.astype(float)
    trace = float(np.trace(mat))
    if trace > 0.0:
        scale = sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (mat[2, 1] - mat[1, 2]) / scale
        y = (mat[0, 2] - mat[2, 0]) / scale
        z = (mat[1, 0] - mat[0, 1]) / scale
    elif mat[0, 0] > mat[1, 1] and mat[0, 0] > mat[2, 2]:
        scale = sqrt(1.0 + mat[0, 0] - mat[1, 1] - mat[2, 2]) * 2.0
        w = (mat[2, 1] - mat[1, 2]) / scale
        x = 0.25 * scale
        y = (mat[0, 1] + mat[1, 0]) / scale
        z = (mat[0, 2] + mat[2, 0]) / scale
    elif mat[1, 1] > mat[2, 2]:
        scale = sqrt(1.0 + mat[1, 1] - mat[0, 0] - mat[2, 2]) * 2.0
        w = (mat[0, 2] - mat[2, 0]) / scale
        x = (mat[0, 1] + mat[1, 0]) / scale
        y = 0.25 * scale
        z = (mat[1, 2] + mat[2, 1]) / scale
    else:
        scale = sqrt(1.0 + mat[2, 2] - mat[0, 0] - mat[1, 1]) * 2.0
        w = (mat[1, 0] - mat[0, 1]) / scale
        x = (mat[0, 2] + mat[2, 0]) / scale
        y = (mat[1, 2] + mat[2, 1]) / scale
        z = 0.25 * scale

    norm = max(sqrt(w * w + x * x + y * y + z * z), 1e-12)
    return w / norm, x / norm, y / norm, z / norm


def quaternion_to_matrix(q: Quaternion) -> np.ndarray:
    """把 w/x/y/z 四元数转换成 3x3 旋转矩阵。"""
    w, x, y, z = q
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def estimate_pose_from_corners(
    corners: Tuple[Point2, Point2, Point2, Point2],
    tag_size_m: float,
    camera_params: Tuple[float, float, float, float],
) -> PoseEstimate:
    """根据 tag 四个角点、真实尺寸和相机内参求出相机坐标系 xyz。"""
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
    # 用像素边长估算一个大致 z 值，后面用它排除贴脸假解。
    expected_z = max(expected_z_from_pixels(corners, tag_size_m, camera_params), 1e-6)
    best = None

    for ordered_corners in _corner_order_candidates(corners):
        image_points = np.array(ordered_corners, dtype=np.float32)
        for flag in (cv2.SOLVEPNP_IPPE, cv2.SOLVEPNP_ITERATIVE):
            # 同时尝试平面目标专用解法和通用迭代解法，取最合理的结果。
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
            # 综合重投影误差和粗略距离，避免选中毫米级的错误近距离解。
            score = reproj + ratio_penalty * 4.0
            if best is None or score < best[0]:
                best = (score, rvec, tvec)

    if best is None:
        raise RuntimeError("solvePnP failed")

    _, rvec, tvec = best
    x_cam, y_cam, z_cam = tvec.flatten()
    rotation_matrix, _ = cv2.Rodrigues(rvec)
    q_camera = matrix_to_quaternion(rotation_matrix)
    return PoseEstimate(
        xyz=(float(x_cam), float(y_cam), float(z_cam)),
        q=q_camera,
    )

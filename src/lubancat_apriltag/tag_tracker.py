from __future__ import annotations

from typing import Optional

import cv2
from pupil_apriltags import Detector

from .config import AppConfig
from .pose import (
    TargetPose,
    distance,
    estimate_pose_from_corners,
    expected_z_from_pixels,
    tag_pixel_width,
    transform_camera_to_body,
    transform_quaternion_camera_to_body,
)


def _build_detector(families, tag_config):
    """Build one detector for all configured families with bounded code tables."""
    # pupil_apriltags 把 bits_corrected 硬编码为 2。先用很小的 tag16h5 初始化
    # Python 封装，再清除它并按配置纠错位数挂载目标家族。
    detector = Detector(
        families="tag16h5",
        nthreads=tag_config.nthreads,
        quad_decimate=tag_config.quad_decimate,
        quad_sigma=tag_config.quad_sigma,
        refine_edges=tag_config.refine_edges,
        decode_sharpening=tag_config.decode_sharpening,
        debug=0,
    )

    old_families = dict(detector.tag_families)
    family_pointer_type = type(next(iter(old_families.values())))
    detector.libc.apriltag_detector_clear_families(detector.tag_detector_ptr)
    for old_name, old_family in old_families.items():
        destroy = getattr(detector.libc, f"{old_name}_destroy")
        destroy.restype = None
        destroy(old_family)

    detector.tag_families = {}
    detector.params["families"] = []
    for family in families:
        creator = getattr(detector.libc, f"{family.name}_create")
        creator.restype = family_pointer_type
        family_pointer = creator()
        if family.max_codes is not None:
            available_codes = int(family_pointer.contents.ncodes)
            family_pointer.contents.ncodes = min(family.max_codes, available_codes)
        detector.libc.apriltag_detector_add_family_bits(
            detector.tag_detector_ptr,
            family_pointer,
            family.bits_corrected,
        )
        detector.tag_families[family.name] = family_pointer
        detector.params["families"].append(family.name)
    return detector


class NestedTagTracker:
    """检测嵌套 AprilTag，并返回当前选中 tag 的位姿。"""

    def __init__(self, config: AppConfig) -> None:
        self.config = config
        # last_stats 给 test_tags.py 显示 raw/accepted/selected 等调试信息。
        self.last_stats = {
            "raw_count": 0,
            "accepted_count": 0,
            "raw_details": (),
            "selected_family": None,
            "selected_tag_id": None,
            "selected_size_m": None,
        }
        tag_config = config.apriltag
        # A shared detector extracts quads once, then decodes them against every
        # configured family. Per-family correction bits and code limits remain intact.
        self.families = {family.name: family for family in tag_config.families}
        self.detector = _build_detector(tag_config.families, tag_config)

    def detect(self, frame) -> Optional[TargetPose]:
        """识别一帧图像；识别失败返回 None，识别成功返回 TargetPose。"""
        if frame.ndim == 2:
            # Picamera2 后端直接返回 YUV420 的 Y 平面，不需要再次转换或复制。
            gray = frame
        elif frame.ndim == 3 and frame.shape[2] == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        elif frame.ndim == 3 and frame.shape[2] == 4:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGRA2GRAY)
        else:
            raise ValueError(f"unsupported camera frame shape: {frame.shape}")

        # 这里只让 pupil_apriltags 做角点检测，位姿由 pose.py 统一计算。
        detections = []
        for detection in self.detector.detect(gray, estimate_tag_pose=False):
            family_name = detection.tag_family
            if isinstance(family_name, bytes):
                family_name = family_name.decode("utf-8")
            family = self.families.get(str(family_name))
            if family is not None:
                detections.append((family, detection))

        raw_details = tuple(
            f"{family.name}:{int(detection.tag_id)}/h{int(detection.hamming)}"
            f"/m{float(detection.decision_margin):.1f}"
            for family, detection in detections
        )
        selected_family = None
        selected = None
        selected_size = 0.0
        accepted_count = 0

        for family, detection in detections:
            tag_id = int(detection.tag_id)
            tag_size_m = family.tag_sizes_m.get(tag_id)
            # 配置里没有尺寸的 tag 直接忽略，避免拿未知尺寸算距离。
            if tag_size_m is None:
                continue
            # hamming 和 decision_margin 是识别质量过滤条件。
            if int(detection.hamming) > self.config.apriltag.max_hamming:
                continue
            if float(detection.decision_margin) < self.config.apriltag.min_decision_margin:
                continue

            accepted_count += 1
            # 跨家族的 ID 没有可比性，直接选择物理尺寸最大的已接受标签。
            if selected is None:
                selected_family = family
                selected = detection
                selected_size = tag_size_m
                continue
            if tag_size_m > selected_size:
                selected_family = family
                selected = detection
                selected_size = tag_size_m
                continue
            if tag_size_m == selected_size and detection.decision_margin > selected.decision_margin:
                selected_family = family
                selected = detection
                selected_size = tag_size_m

        self.last_stats = {
            "raw_count": len(detections),
            "accepted_count": accepted_count,
            "raw_details": raw_details,
            "selected_family": selected_family.name if selected_family is not None else None,
            "selected_tag_id": int(selected.tag_id) if selected is not None else None,
            "selected_size_m": selected_size if selected is not None else None,
        }

        if selected is None or selected_family is None:
            return None

        # pupil_apriltags 返回四个图像角点，后续根据真实尺寸解算 xyz 和姿态四元数。
        corners = tuple((float(point[0]), float(point[1])) for point in selected.corners)
        pose_estimate = estimate_pose_from_corners(
            corners,
            selected_size,
            self.config.camera.params,
        )
        camera_xyz = pose_estimate.xyz
        body_xyz = transform_camera_to_body(camera_xyz, self.config.camera_to_body)
        body_q = transform_quaternion_camera_to_body(pose_estimate.q, self.config.camera_to_body)

        # 下面两个字段只用于调试，帮助判断 PnP 距离是否离谱。
        pixel_width = tag_pixel_width(corners)
        expected_z = expected_z_from_pixels(
            corners,
            selected_size,
            self.config.camera.params,
        )

        return TargetPose(
            tag_family=selected_family.name,
            tag_id=int(selected.tag_id),
            tag_size_m=selected_size,
            corners=corners,
            x_cam=camera_xyz[0],
            y_cam=camera_xyz[1],
            z_cam=camera_xyz[2],
            x_body=body_xyz[0],
            y_body=body_xyz[1],
            z_body=body_xyz[2],
            q_body=body_q,
            distance_m=distance(body_xyz),
            tag_pixel_width=pixel_width,
            expected_z_m=expected_z,
            decision_margin=float(selected.decision_margin),
            hamming=int(selected.hamming),
        )

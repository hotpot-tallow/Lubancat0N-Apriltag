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
)


class NestedTagTracker:
    """检测嵌套 AprilTag，并返回当前选中 tag 的位姿。"""

    def __init__(self, config: AppConfig) -> None:
        self.config = config
        # last_stats 给 test_tags.py 显示 raw/accepted/selected 等调试信息。
        self.last_stats = {
            "raw_count": 0,
            "accepted_count": 0,
            "selected_tag_id": None,
            "selected_size_m": None,
        }
        tag_config = config.apriltag
        self.detector = Detector(
            families=tag_config.family,
            nthreads=2,
            quad_decimate=tag_config.quad_decimate,
            quad_sigma=tag_config.quad_sigma,
            refine_edges=tag_config.refine_edges,
            decode_sharpening=tag_config.decode_sharpening,
            debug=0,
        )

    def detect(self, frame) -> Optional[TargetPose]:
        """识别一帧图像；识别失败返回 None，识别成功返回 TargetPose。"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 这里只让 pupil_apriltags 做角点检测，位姿由 pose.py 统一计算。
        detections = self.detector.detect(gray, estimate_tag_pose=False)
        selected = None
        selected_size = float("inf")
        accepted_count = 0

        for detection in detections:
            tag_id = int(detection.tag_id)
            tag_size_m = self.config.apriltag.tag_sizes_m.get(tag_id)
            # 配置里没有尺寸的 tag 直接忽略，避免拿未知尺寸算距离。
            if tag_size_m is None:
                continue
            # hamming 和 decision_margin 是识别质量过滤条件。
            if int(detection.hamming) > self.config.apriltag.max_hamming:
                continue
            if float(detection.decision_margin) < self.config.apriltag.min_decision_margin:
                continue
            accepted_count += 1
            # 嵌套 tag 同时出现时，默认选择物理尺寸最小的那个。
            if selected is None:
                selected = detection
                selected_size = tag_size_m
                continue
            if tag_size_m < selected_size:
                selected = detection
                selected_size = tag_size_m
                continue
            if tag_size_m == selected_size and detection.decision_margin > selected.decision_margin:
                selected = detection
                selected_size = tag_size_m

        self.last_stats = {
            "raw_count": len(detections),
            "accepted_count": accepted_count,
            "selected_tag_id": int(selected.tag_id) if selected is not None else None,
            "selected_size_m": selected_size if selected is not None else None,
        }

        if selected is None:
            return None

        # pupil_apriltags 返回四个图像角点，后续根据真实尺寸解算 xyz。
        corners = tuple((float(point[0]), float(point[1])) for point in selected.corners)
        camera_xyz = estimate_pose_from_corners(
            corners,
            selected_size,
            self.config.camera.params,
        )
        body_xyz = transform_camera_to_body(camera_xyz, self.config.camera_to_body)
        # 下面两个字段只用于调试，帮助判断 PnP 距离是否离谱。
        pixel_width = tag_pixel_width(corners)
        expected_z = expected_z_from_pixels(
            corners,
            selected_size,
            self.config.camera.params,
        )

        return TargetPose(
            tag_id=int(selected.tag_id),
            tag_size_m=selected_size,
            corners=corners,
            x_cam=camera_xyz[0],
            y_cam=camera_xyz[1],
            z_cam=camera_xyz[2],
            x_body=body_xyz[0],
            y_body=body_xyz[1],
            z_body=body_xyz[2],
            distance_m=distance(body_xyz),
            tag_pixel_width=pixel_width,
            expected_z_m=expected_z,
            decision_margin=float(selected.decision_margin),
            hamming=int(selected.hamming),
        )

from __future__ import annotations

from typing import Optional

import cv2
from pupil_apriltags import Detector

from .config import AppConfig
from .pose import TargetPose, distance, estimate_pose_from_corners, transform_camera_to_body


class NestedTagTracker:
    """Detect nested tag36h11 IDs whose physical sizes differ."""

    def __init__(self, config: AppConfig) -> None:
        self.config = config
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
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        detections = self.detector.detect(gray, estimate_tag_pose=False)
        selected = None
        selected_size = float("inf")

        for detection in detections:
            tag_id = int(detection.tag_id)
            tag_size_m = self.config.apriltag.tag_sizes_m.get(tag_id)
            if tag_size_m is None:
                continue
            if int(detection.hamming) > self.config.apriltag.max_hamming:
                continue
            if float(detection.decision_margin) < self.config.apriltag.min_decision_margin:
                continue
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

        if selected is None:
            return None

        corners = tuple((float(point[0]), float(point[1])) for point in selected.corners)
        camera_xyz = estimate_pose_from_corners(
            corners,
            selected_size,
            self.config.camera.params,
        )
        body_xyz = transform_camera_to_body(camera_xyz, self.config.camera_to_body)

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
            decision_margin=float(selected.decision_margin),
            hamming=int(selected.hamming),
        )

from __future__ import annotations

from typing import List, Optional

import cv2
from pupil_apriltags import Detector

from .config import AppConfig
from .pose import TargetPose, distance, transform_camera_to_body


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

        # pupil-apriltags accepts one tag_size per detect() call. For nested
        # tags with different IDs and sizes, run pose estimation per target ID.
        ids = list(self.config.apriltag.tag_priority)
        ids.extend(tag_id for tag_id in self.config.apriltag.tag_sizes_m if tag_id not in ids)

        for tag_id in ids:
            tag_size_m = self.config.apriltag.tag_sizes_m.get(tag_id)
            if tag_size_m is None:
                continue

            detections = self.detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=self.config.camera.params,
                tag_size=tag_size_m,
            )

            candidates: List[TargetPose] = []
            for detection in detections:
                if int(detection.tag_id) != tag_id:
                    continue
                if int(detection.hamming) > self.config.apriltag.max_hamming:
                    continue
                if float(detection.decision_margin) < self.config.apriltag.min_decision_margin:
                    continue

                x_cam, y_cam, z_cam = (float(value) for value in detection.pose_t.flatten())
                body_xyz = transform_camera_to_body(
                    (x_cam, y_cam, z_cam),
                    self.config.camera_to_body,
                )
                candidates.append(
                    TargetPose(
                        tag_id=tag_id,
                        tag_size_m=tag_size_m,
                        x_cam=x_cam,
                        y_cam=y_cam,
                        z_cam=z_cam,
                        x_body=body_xyz[0],
                        y_body=body_xyz[1],
                        z_body=body_xyz[2],
                        distance_m=distance(body_xyz),
                        decision_margin=float(detection.decision_margin),
                        hamming=int(detection.hamming),
                    )
                )

            if candidates:
                return max(candidates, key=lambda pose: pose.decision_margin)

        return None

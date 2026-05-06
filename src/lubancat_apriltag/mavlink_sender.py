from __future__ import annotations

import os
import time

os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil  # noqa: E402

from .config import MavlinkConfig
from .pose import TargetPose


class LandingTargetSender:
    def __init__(self, config: MavlinkConfig) -> None:
        self.config = config
        self.master = mavutil.mavlink_connection(
            config.connection,
            baud=config.baud,
            source_system=config.source_system,
            source_component=config.source_component,
        )

    def send(self, pose: TargetPose) -> None:
        self.master.mav.landing_target_send(
            int(time.time() * 1_000_000),
            self.config.target_num,
            mavutil.mavlink.MAV_FRAME_BODY_FRD,
            0.0,
            0.0,
            pose.distance_m,
            0.0,
            0.0,
            pose.x_body,
            pose.y_body,
            pose.z_body,
            [1.0, 0.0, 0.0, 0.0],
            mavutil.mavlink.LANDING_TARGET_TYPE_VISION_FIDUCIAL,
            1,
        )

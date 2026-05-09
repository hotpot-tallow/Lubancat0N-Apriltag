from __future__ import annotations

import os
import time
from dataclasses import dataclass
from typing import Tuple

os.environ["MAVLINK20"] = "1"

from pymavlink import mavutil  # noqa: E402

from .config import MavlinkConfig
from .pose import TargetPose


MAVLINK2_MAGIC = 0xFD
LANDING_TARGET_FRAME = getattr(mavutil.mavlink, "MAV_FRAME_BODY_FRD", 12)
LANDING_TARGET_TYPE = getattr(mavutil.mavlink, "LANDING_TARGET_TYPE_VISION_FIDUCIAL", 2)
ZERO_ROTATION_QUATERNION = (1.0, 0.0, 0.0, 0.0)


@dataclass(frozen=True)
class LandingTargetPayload:
    time_usec: int
    target_num: int
    frame: int
    angle_x: float
    angle_y: float
    distance: float
    size_x: float
    size_y: float
    x: float
    y: float
    z: float
    q: Tuple[float, float, float, float]
    target_type: int
    position_valid: int


def mavlink2_enabled() -> bool:
    return str(getattr(mavutil.mavlink, "WIRE_PROTOCOL_VERSION", "")) == "2.0"


def landing_target_payload(pose: TargetPose, target_num: int) -> LandingTargetPayload:
    return LandingTargetPayload(
        time_usec=int(time.monotonic() * 1_000_000),
        target_num=target_num,
        frame=LANDING_TARGET_FRAME,
        angle_x=0.0,
        angle_y=0.0,
        distance=pose.distance_m,
        size_x=0.0,
        size_y=0.0,
        x=pose.x_body,
        y=pose.y_body,
        z=pose.z_body,
        q=ZERO_ROTATION_QUATERNION,
        target_type=LANDING_TARGET_TYPE,
        position_valid=1,
    )


class LandingTargetSender:
    def __init__(self, config: MavlinkConfig) -> None:
        if not mavlink2_enabled():
            raise RuntimeError("pymavlink is not using MAVLink2; MAVLINK20 must be set before import")

        self.config = config
        self.master = mavutil.mavlink_connection(
            config.connection,
            baud=config.baud,
            source_system=config.source_system,
            source_component=config.source_component,
        )

    def send(self, pose: TargetPose) -> None:
        payload = landing_target_payload(pose, self.config.target_num)
        msg = self.master.mav.landing_target_encode(
            payload.time_usec,
            payload.target_num,
            payload.frame,
            payload.angle_x,
            payload.angle_y,
            payload.distance,
            payload.size_x,
            payload.size_y,
            payload.x,
            payload.y,
            payload.z,
            payload.q,
            payload.target_type,
            payload.position_valid,
        )
        self.master.mav.send(msg, force_mavlink1=False)

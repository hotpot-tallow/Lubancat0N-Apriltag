from __future__ import annotations

import os
import time
from dataclasses import dataclass
from math import atan2
from typing import Tuple

# pymavlink 必须在 import 前设置 MAVLINK20，否则扩展字段 x/y/z/position_valid 可能发不出去。
os.environ["MAVLINK20"] = "1"

from pymavlink import mavutil  # noqa: E402

from .config import MavlinkConfig
from .pose import TargetPose


MAVLINK2_MAGIC = 0xFD
LANDING_TARGET_FRAME = getattr(mavutil.mavlink, "MAV_FRAME_BODY_FRD", 12)
LANDING_TARGET_TYPE = getattr(mavutil.mavlink, "LANDING_TARGET_TYPE_VISION_FIDUCIAL", 2)


@dataclass(frozen=True)
class LandingTargetPayload:
    """LANDING_TARGET 消息的字段快照，便于打印和检查打包内容。"""

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
    """确认 pymavlink 当前是否使用 MAVLink2 协议。"""
    return str(getattr(mavutil.mavlink, "WIRE_PROTOCOL_VERSION", "")) == "2.0"


def landing_target_payload(pose: TargetPose, target_num: int) -> LandingTargetPayload:
    """把识别到的机体系目标位置转换成 MAVLink LANDING_TARGET 字段。"""
    # 兼容角度模式：Mission Planner 也会显示 angle_x/angle_y。
    # 对 BODY_FRD 位置字段，水平/垂直角度可由 x/z、y/z 反算，单位是弧度。
    angle_x = atan2(pose.x_body, pose.z_body)
    angle_y = atan2(pose.y_body, pose.z_body)
    return LandingTargetPayload(
        time_usec=int(time.monotonic() * 1_000_000),
        target_num=target_num,
        frame=LANDING_TARGET_FRAME,
        angle_x=angle_x,
        angle_y=angle_y,
        distance=pose.distance_m,
        size_x=0.0,
        size_y=0.0,
        x=pose.x_body,
        y=pose.y_body,
        z=pose.z_body,
        q=pose.q_body,
        target_type=LANDING_TARGET_TYPE,
        position_valid=1,
    )


class LandingTargetSender:
    """负责打开 MAVLink 连接并发送 LANDING_TARGET。"""

    def __init__(self, config: MavlinkConfig) -> None:
        # 位置模式需要 MAVLink2 扩展字段；如果不是 MAVLink2，直接报错避免悄悄发错格式。
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
        """发送一次 LANDING_TARGET；x/y/z/distance 的单位都是米。"""
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

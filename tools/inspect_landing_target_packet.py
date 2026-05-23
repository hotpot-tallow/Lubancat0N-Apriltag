from __future__ import annotations

import argparse
import os

# 这个检查脚本只验证打包格式，必须强制使用 MAVLink2。
os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil

from lubancat_apriltag.config import load_config
from lubancat_apriltag.mavlink_sender import MAVLINK2_MAGIC, landing_target_payload, mavlink2_enabled
from lubancat_apriltag.pose import TargetPose


def main() -> None:
    """构造一条假位姿，检查 LANDING_TARGET 是否按 MAVLink2 位置字段打包。"""
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config/example_config.json")
    args = parser.parse_args()

    config = load_config(args.config)
    # 这里的 TargetPose 是假数据，只用于检查打包后的 message id/frame/angle/x/y/z/q。
    pose = TargetPose(
        tag_id=0,
        tag_size_m=0.5,
        corners=((0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)),
        x_cam=0.0,
        y_cam=0.0,
        z_cam=1.0,
        x_body=1.0,
        y_body=0.2,
        z_body=1.5,
        q_body=(0.98, 0.01, 0.02, 0.20),
        distance_m=1.8,
        tag_pixel_width=100.0,
        expected_z_m=1.0,
        decision_margin=50.0,
        hamming=0,
    )
    payload = landing_target_payload(pose, config.mavlink.target_num)

    # 不真正打开串口，只把消息 pack 成字节，再检查 MAVLink2 magic 和字段。
    mav = mavutil.mavlink.MAVLink(
        None,
        srcSystem=config.mavlink.source_system,
        srcComponent=config.mavlink.source_component,
    )
    msg = mav.landing_target_encode(
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
    packet = msg.pack(mav, force_mavlink1=False)

    print("mavlink2_enabled:", mavlink2_enabled())
    print("packet_magic:", hex(packet[0]), "expected:", hex(MAVLINK2_MAGIC))
    print("message_id:", msg.get_msgId())
    print("frame:", payload.frame, "MAV_FRAME_BODY_FRD")
    print("angle_x/angle_y:", payload.angle_x, payload.angle_y)
    print("x/y/z:", payload.x, payload.y, payload.z)
    print("q:", payload.q)
    print("type:", payload.target_type)
    print("position_valid:", payload.position_valid)


if __name__ == "__main__":
    main()

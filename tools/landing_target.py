from __future__ import annotations

import argparse
import time

from lubancat_apriltag.camera import open_camera
from lubancat_apriltag.config import load_config
from lubancat_apriltag.debug import enable_watchdog
from lubancat_apriltag.mavlink_sender import (
    LANDING_TARGET_FRAME,
    LandingTargetSender,
    landing_target_payload,
    mavlink2_enabled,
)
from lubancat_apriltag.tag_tracker import NestedTagTracker


def main() -> None:
    """主程序：读取摄像头、识别 tag，并向飞控发送 LANDING_TARGET。"""
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config/example_config.json")
    parser.add_argument("--dry-run", action="store_true", help="print LANDING_TARGET values without opening MAVLink")
    parser.add_argument(
        "--camera-read-timeout",
        type=float,
        default=2.0,
        help="restart camera if no new frame is received within N seconds",
    )
    parser.add_argument(
        "--watchdog-timeout",
        type=float,
        default=0.0,
        help="dump Python stack every N seconds when the process appears stuck",
    )
    args = parser.parse_args()
    enable_watchdog(args.watchdog_timeout)

    config = load_config(args.config)
    cap = open_camera(config.camera, read_timeout_s=args.camera_read_timeout)
    tracker = NestedTagTracker(config)
    # dry-run 模式不打开串口，只把即将发送的内容打印出来。
    sender = None if args.dry_run else LandingTargetSender(config.mavlink)
    print(
        "LANDING_TARGET output:",
        f"mavlink2={mavlink2_enabled()}",
        f"frame={LANDING_TARGET_FRAME} (MAV_FRAME_BODY_FRD)",
        "position_valid=1",
    )

    period = 1.0 / config.mavlink.send_rate_hz
    last_send = 0.0

    try:
        while True:
            # 读取摄像头图像；失败时等待一下继续读。
            ok, frame = cap.read()
            if not ok:
                print("camera read failed")
                time.sleep(0.1)
                continue

            pose = tracker.detect(frame)
            if pose is None:
                # 当前逻辑是识别不到 tag 就不发送旧数据，避免飞控追踪过期目标。
                continue

            now = time.monotonic()
            if now - last_send < period:
                # 按配置的 send_rate_hz 限速发送，避免串口刷太快。
                continue
            last_send = now

            if sender is None:
                payload = landing_target_payload(pose, config.mavlink.target_num)
                print(
                    f"LANDING_TARGET id={pose.tag_id} "
                    f"target_num={payload.target_num} frame={payload.frame} "
                    f"angle_x={payload.angle_x:+.4f} angle_y={payload.angle_y:+.4f} "
                    f"x={payload.x:+.3f} y={payload.y:+.3f} z={payload.z:+.3f} "
                    f"dist={payload.distance:.3f} q={payload.q} "
                    f"type={payload.target_type} position_valid={payload.position_valid}"
                )
            else:
                sender.send(pose)
    finally:
        cap.release()


if __name__ == "__main__":
    main()

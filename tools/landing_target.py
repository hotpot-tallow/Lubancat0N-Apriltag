from __future__ import annotations

import argparse
import time

from lubancat_apriltag.camera import open_camera
from lubancat_apriltag.config import load_config
from lubancat_apriltag.mavlink_sender import (
    LANDING_TARGET_FRAME,
    LandingTargetSender,
    landing_target_payload,
    mavlink2_enabled,
)
from lubancat_apriltag.tag_tracker import NestedTagTracker


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config/example_config.json")
    parser.add_argument("--dry-run", action="store_true", help="print LANDING_TARGET values without opening MAVLink")
    args = parser.parse_args()

    config = load_config(args.config)
    cap = open_camera(config.camera)
    tracker = NestedTagTracker(config)
    sender = None if args.dry_run else LandingTargetSender(config.mavlink)
    print(
        "LANDING_TARGET output:",
        f"mavlink2={mavlink2_enabled()}",
        f"frame={LANDING_TARGET_FRAME} (MAV_FRAME_BODY_FRD)",
        "position_valid=1",
    )

    period = 1.0 / config.mavlink.send_rate_hz
    last_send = 0.0

    while True:
        ok, frame = cap.read()
        if not ok:
            print("camera read failed")
            time.sleep(0.1)
            continue

        pose = tracker.detect(frame)
        if pose is None:
            continue

        now = time.monotonic()
        if now - last_send < period:
            continue
        last_send = now

        if sender is None:
            payload = landing_target_payload(pose, config.mavlink.target_num)
            print(
                f"LANDING_TARGET id={pose.tag_id} "
                f"target_num={payload.target_num} frame={payload.frame} "
                f"x={payload.x:+.3f} y={payload.y:+.3f} z={payload.z:+.3f} "
                f"dist={payload.distance:.3f} q={payload.q} "
                f"type={payload.target_type} position_valid={payload.position_valid}"
            )
        else:
            sender.send(pose)


if __name__ == "__main__":
    main()

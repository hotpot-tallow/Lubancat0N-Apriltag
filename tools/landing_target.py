from __future__ import annotations

import argparse
import time

from lubancat_apriltag.camera import open_camera
from lubancat_apriltag.config import load_config
from lubancat_apriltag.mavlink_sender import LandingTargetSender
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
            print(
                f"LANDING_TARGET id={pose.tag_id} "
                f"x={pose.x_body:+.3f} y={pose.y_body:+.3f} "
                f"z={pose.z_body:+.3f} dist={pose.distance_m:.3f}"
            )
        else:
            sender.send(pose)


if __name__ == "__main__":
    main()

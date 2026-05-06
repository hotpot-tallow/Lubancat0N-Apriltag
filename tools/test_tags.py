from __future__ import annotations

import argparse
import time

from lubancat_apriltag.camera import open_camera
from lubancat_apriltag.config import load_config
from lubancat_apriltag.tag_tracker import NestedTagTracker


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config/example_config.json")
    args = parser.parse_args()

    config = load_config(args.config)
    cap = open_camera(config.camera)
    tracker = NestedTagTracker(config)

    while True:
        ok, frame = cap.read()
        if not ok:
            print("camera read failed")
            time.sleep(0.1)
            continue

        pose = tracker.detect(frame)
        if pose is None:
            print("no tag")
        else:
            print(
                f"id={pose.tag_id} size={pose.tag_size_m:.3f}m "
                f"cam=({pose.x_cam:+.3f},{pose.y_cam:+.3f},{pose.z_cam:+.3f}) "
                f"body=({pose.x_body:+.3f},{pose.y_body:+.3f},{pose.z_body:+.3f}) "
                f"dist={pose.distance_m:.3f} margin={pose.decision_margin:.1f}"
            )
        time.sleep(0.05)


if __name__ == "__main__":
    main()

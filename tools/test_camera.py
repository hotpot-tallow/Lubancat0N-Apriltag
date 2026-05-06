from __future__ import annotations

import argparse

import cv2

from lubancat_apriltag.camera import open_camera
from lubancat_apriltag.config import load_config


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config/example_config.json")
    parser.add_argument("--headless", action="store_true", help="save one frame instead of opening a preview window")
    args = parser.parse_args()

    config = load_config(args.config)
    cap = open_camera(config.camera)

    if args.headless:
        ok, frame = cap.read()
        if not ok:
            raise RuntimeError("camera read failed")
        cv2.imwrite("camera_test.jpg", frame)
        print("saved camera_test.jpg", frame.shape)
        return

    while True:
        ok, frame = cap.read()
        if not ok:
            print("camera read failed")
            continue
        cv2.imshow("camera", frame)
        if cv2.waitKey(1) == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

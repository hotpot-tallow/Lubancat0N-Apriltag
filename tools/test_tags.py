from __future__ import annotations

import argparse
import time

import cv2

from lubancat_apriltag.camera import camera_info, open_camera
from lubancat_apriltag.config import load_config
from lubancat_apriltag.pose import TargetPose
from lubancat_apriltag.tag_tracker import NestedTagTracker


def draw_status(frame, fps: float, detect_ms: float, stats) -> None:
    cv2.putText(
        frame,
        f"FPS: {fps:.1f} detect: {detect_ms:.1f}ms raw: {stats.get('raw_count', 0)} ok: {stats.get('accepted_count', 0)}",
        (10, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )


def draw_pose(frame, pose: TargetPose, fps: float, detect_ms: float, stats) -> None:
    corners = [(int(x), int(y)) for x, y in pose.corners]
    for index, start in enumerate(corners):
        end = corners[(index + 1) % len(corners)]
        cv2.line(frame, start, end, (0, 255, 0), 2)

    center_x = int(sum(point[0] for point in corners) / len(corners))
    center_y = int(sum(point[1] for point in corners) / len(corners))
    cv2.circle(frame, (center_x, center_y), 4, (0, 0, 255), -1)

    lines = [
        f"id={pose.tag_id} size={pose.tag_size_m:.3f}m",
        f"body x={pose.x_body:+.3f} y={pose.y_body:+.3f} z={pose.z_body:+.3f}m",
        f"dist={pose.distance_m:.3f}m margin={pose.decision_margin:.1f}",
    ]
    y = max(24, center_y - 48)
    for line in lines:
        cv2.putText(
            frame,
            line,
            (max(8, center_x - 120), y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )
        y += 20

    draw_status(frame, fps, detect_ms, stats)


def draw_no_tag(frame, fps: float, detect_ms: float, stats) -> None:
    draw_status(frame, fps, detect_ms, stats)
    cv2.putText(
        frame,
        "no tag",
        (10, 58),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 0, 255),
        2,
        cv2.LINE_AA,
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config/example_config.json")
    parser.add_argument("--headless", action="store_true", help="print tag pose without opening a preview window")
    parser.add_argument("--print-every", type=float, default=1.0, help="seconds between terminal prints")
    parser.add_argument("--preview-scale", type=float, default=0.5, help="scale preview window to reduce GUI load")
    args = parser.parse_args()

    config = load_config(args.config)
    cap = open_camera(config.camera)
    print("camera opened:", camera_info(cap))
    tracker = NestedTagTracker(config)
    last_frame_time = time.monotonic()
    last_print_time = 0.0
    fps = 0.0

    while True:
        ok, frame = cap.read()
        now = time.monotonic()
        frame_dt = now - last_frame_time
        last_frame_time = now
        instant_fps = 1.0 / frame_dt if frame_dt > 0.0 else 0.0
        fps = instant_fps if fps == 0.0 else fps * 0.9 + instant_fps * 0.1

        if not ok:
            print("camera read failed")
            time.sleep(0.1)
            continue

        detect_start = time.monotonic()
        pose = tracker.detect(frame)
        detect_ms = (time.monotonic() - detect_start) * 1000.0
        stats = tracker.last_stats

        if now - last_print_time >= args.print_every:
            last_print_time = now
            if pose is None:
                print(
                    f"no tag fps={fps:.1f} detect_ms={detect_ms:.1f} "
                    f"raw={stats.get('raw_count', 0)} accepted={stats.get('accepted_count', 0)}"
                )
            else:
                print(
                    f"id={pose.tag_id} size={pose.tag_size_m:.3f}m "
                    f"cam=({pose.x_cam:+.3f},{pose.y_cam:+.3f},{pose.z_cam:+.3f}) "
                    f"body=({pose.x_body:+.3f},{pose.y_body:+.3f},{pose.z_body:+.3f}) "
                    f"dist={pose.distance_m:.3f} margin={pose.decision_margin:.1f} "
                    f"fps={fps:.1f} detect_ms={detect_ms:.1f} raw={stats.get('raw_count', 0)}"
                )

        if args.headless:
            continue

        if pose is None:
            draw_no_tag(frame, fps, detect_ms, stats)
        else:
            draw_pose(frame, pose, fps, detect_ms, stats)

        preview = frame
        if args.preview_scale != 1.0:
            preview = cv2.resize(frame, None, fx=args.preview_scale, fy=args.preview_scale)
        cv2.imshow("AprilTag test", preview)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

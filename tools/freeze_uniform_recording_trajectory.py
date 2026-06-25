#!/usr/bin/env python3
"""Freeze a Gazebo replay trajectory into a deterministic, uniform-speed path.

The input is the cached paper-QP trajectory produced by export_gazebo_trajectory.py.
This tool does not solve any QP. It only resamples the existing replay frames so
that the video demo can be replayed deterministically with approximately uniform
motion.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


Point3 = tuple[float, float, float]


def point(values: list[float]) -> Point3:
    return (float(values[0]), float(values[1]), float(values[2]))


def round_point(values: Point3) -> list[float]:
    return [round(value, 6) for value in values]


def lerp(a: Point3, b: Point3, t: float) -> Point3:
    return (
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    )


def frame_motion_distance(a: dict[str, Any], b: dict[str, Any]) -> float:
    robot_distances = [
        math.dist(point(ra), point(rb))
        for ra, rb in zip(a.get("robots", [])[:3], b.get("robots", [])[:3])
    ]
    distances = robot_distances if robot_distances else [0.0]
    if a.get("payload") is not None and b.get("payload") is not None:
        distances.append(math.dist(point(a["payload"]), point(b["payload"])))
    return max(distances)


def cumulative_motion(frames: list[dict[str, Any]]) -> list[float]:
    cumulative = [0.0]
    for previous, current in zip(frames, frames[1:]):
        cumulative.append(cumulative[-1] + frame_motion_distance(previous, current))
    return cumulative


def metadata_frame(a: dict[str, Any], b: dict[str, Any], t: float) -> dict[str, Any]:
    return b if t >= 0.5 else a


def interpolate_frame(a: dict[str, Any], b: dict[str, Any], t: float, output_index: int) -> dict[str, Any]:
    meta = metadata_frame(a, b, t)
    robots = [
        round_point(lerp(point(ra), point(rb), t))
        for ra, rb in zip(a.get("robots", [])[:3], b.get("robots", [])[:3])
    ]

    payload = None
    if a.get("payload") is not None and b.get("payload") is not None:
        payload = round_point(lerp(point(a["payload"]), point(b["payload"]), t))
    elif t < 0.5 and a.get("payload") is not None:
        payload = list(a["payload"])
    elif b.get("payload") is not None:
        payload = list(b["payload"])

    return {
        "source_frame_index": int(meta.get("source_frame_index", output_index)),
        "recording_frame_index": output_index,
        "state_index": int(meta.get("state_index", 0)),
        "state_id": str(meta.get("state_id", "")),
        "task_mode": str(meta.get("task_mode", "")),
        "formation": str(meta.get("formation", "")),
        "phase": "recording_uniform",
        "robots": robots,
        "payload": payload,
    }


def resample_uniform(frames: list[dict[str, Any]], output_frames: int) -> list[dict[str, Any]]:
    if len(frames) <= 1:
        return frames

    cumulative = cumulative_motion(frames)
    total = cumulative[-1]
    if total <= 1e-9:
        return [dict(frames[0]) for _ in range(max(1, output_frames))]

    count = max(2, output_frames)
    targets = [total * i / (count - 1) for i in range(count)]
    result: list[dict[str, Any]] = []
    segment = 0
    for output_index, target in enumerate(targets):
        while segment < len(cumulative) - 2 and cumulative[segment + 1] < target:
            segment += 1
        start_distance = cumulative[segment]
        end_distance = cumulative[segment + 1]
        if end_distance <= start_distance + 1e-12:
            t = 0.0
        else:
            t = (target - start_distance) / (end_distance - start_distance)
        result.append(interpolate_frame(frames[segment], frames[segment + 1], t, output_index))
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--frames", type=int, default=240)
    args = parser.parse_args()

    payload = json.loads(args.input.read_text(encoding="utf-8"))
    frames = payload.get("replay_frames", [])
    if not frames:
        raise RuntimeError(f"No replay_frames found in {args.input}")

    uniform_frames = resample_uniform(frames, args.frames)
    payload["schema"] = "gazebo_recording_uniform_trajectory_v1"
    payload["recording_source"] = str(args.input)
    payload["recording_uniform_frames"] = len(uniform_frames)
    payload["recording_uniform_from_frames"] = len(frames)
    payload["recording_uniform_motion_length"] = cumulative_motion(frames)[-1]
    payload["replay_frames"] = uniform_frames

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(
        f"[OK] froze uniform recording trajectory: {args.out} "
        f"frames={len(uniform_frames)} from={len(frames)}",
        flush=True,
    )


if __name__ == "__main__":
    main()

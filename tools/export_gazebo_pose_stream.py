#!/usr/bin/env python3
"""Export a fixed Gazebo pose stream for the in-process replay plugin."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import replay_gazebo_trajectory as replay  # noqa: E402


PoseRow = tuple[int, str, replay.Point3, tuple[float, float, float, float]]


class PayloadVisualDynamics:
    """Small deterministic spring-damper visual model for lift/carry motion."""

    def __init__(self, *, stiffness: float, damping: float, ground_height: float) -> None:
        self.stiffness = max(0.0, stiffness)
        self.damping = max(0.0, damping)
        self.ground_height = ground_height
        self.position: replay.Point3 | None = None
        self.velocity: replay.Point3 = (0.0, 0.0, 0.0)
        self.last_reference: replay.Point3 | None = None

    def reset(self, reference: replay.Point3 | None) -> replay.Point3 | None:
        self.position = reference
        self.velocity = (0.0, 0.0, 0.0)
        self.last_reference = reference
        return reference

    def step(self, reference: replay.Point3, *, attached: bool, dt: float) -> replay.Point3:
        if not attached:
            self.reset(reference)
            return reference
        if self.position is None or self.last_reference is None:
            self.reset(reference)
            return reference

        reference_velocity = (
            (reference[0] - self.last_reference[0]) / max(dt, 1e-6),
            (reference[1] - self.last_reference[1]) / max(dt, 1e-6),
            (reference[2] - self.last_reference[2]) / max(dt, 1e-6),
        )
        error = (
            reference[0] - self.position[0],
            reference[1] - self.position[1],
            reference[2] - self.position[2],
        )
        velocity_error = (
            reference_velocity[0] - self.velocity[0],
            reference_velocity[1] - self.velocity[1],
            reference_velocity[2] - self.velocity[2],
        )
        acceleration = (
            self.stiffness * error[0] + self.damping * velocity_error[0],
            self.stiffness * error[1] + self.damping * velocity_error[1],
            0.65 * self.stiffness * error[2] + 0.75 * self.damping * velocity_error[2],
        )
        acceleration = (
            replay.clamp(acceleration[0], -4.2, 4.2),
            replay.clamp(acceleration[1], -4.2, 4.2),
            replay.clamp(acceleration[2], -4.8, 4.8),
        )
        self.velocity = (
            replay.clamp(self.velocity[0] + acceleration[0] * dt, -1.0, 1.0),
            replay.clamp(self.velocity[1] + acceleration[1] * dt, -1.0, 1.0),
            replay.clamp(self.velocity[2] + acceleration[2] * dt, -0.85, 0.85),
        )
        self.position = (
            self.position[0] + self.velocity[0] * dt,
            self.position[1] + self.velocity[1] * dt,
            max(self.ground_height, self.position[2] + self.velocity[2] * dt),
        )
        self.last_reference = reference
        return self.position


def pose_row(frame_index: int, name: str, position: replay.Point3, quaternion: tuple[float, float, float, float]) -> PoseRow:
    return (frame_index, name, position, quaternion)


def hidden_highlight_rows(frame_index: int, case: dict[str, Any]) -> list[PoseRow]:
    rows: list[PoseRow] = []
    q = replay.quaternion_from_euler(0.0, 0.0, 0.0)
    for region in case.get("regions", []):
        region_id = region.get("id")
        rows.append(pose_row(frame_index, f"highlight_current_p{region_id}", replay.HIDDEN_HIGHLIGHT, q))
        rows.append(pose_row(frame_index, f"highlight_target_p{region_id}", replay.HIDDEN_HIGHLIGHT, q))
    for bridge in case.get("bridges", []):
        rows.append(pose_row(frame_index, replay.model_name(f"highlight_bridge_{bridge.get('name', '')}"), replay.HIDDEN_HIGHLIGHT, q))
    return rows


def highlight_rows(
    frame_index: int,
    case: dict[str, Any],
    current: str | None,
    target: str | None,
    bridge: str | None,
) -> list[PoseRow]:
    rows = hidden_highlight_rows(frame_index, case)
    q = replay.quaternion_from_euler(0.0, 0.0, 0.0)
    if current:
        rows.append(pose_row(frame_index, replay.model_name(f"highlight_current_{current}"), replay.VISIBLE_CURRENT, q))
    if target:
        rows.append(pose_row(frame_index, replay.model_name(f"highlight_target_{target}"), replay.VISIBLE_TARGET, q))
    if bridge:
        rows.append(pose_row(frame_index, replay.model_name(f"highlight_bridge_{bridge}"), replay.VISIBLE_BRIDGE, q))
    return rows


def frame_rows(
    *,
    frame_index: int,
    frame: replay.ReplayFrame,
    next_frame: replay.ReplayFrame,
    next_state_id: str | None,
    state_progress: float,
    case: dict[str, Any],
    states: dict[str, dict[str, Any]],
    transitions: dict[tuple[str, str], dict[str, Any]],
    bridges: dict[tuple[frozenset[str], str], str],
    last_yaws: list[float | None],
    rotor_spin_angles: list[list[float]],
    camera_target: replay.Point3 | None,
    camera_target_yaw: float | None,
    rotor_speed: float,
    period: float,
    camera_target_z_offset: float,
    camera_target_ahead_distance: float,
    camera_target_smoothing: float,
    camera_target_yaw_smoothing: float,
    yaw_smoothing: float,
    highlight_lookahead_fraction: float,
    payload_lift_physics: bool,
    payload_dynamics: PayloadVisualDynamics,
    dt: float,
    ground_height: float,
    cable_switch_height: float,
) -> tuple[list[PoseRow], replay.Point3 | None, float | None]:
    rows: list[PoseRow] = []
    current_positions = replay.frame_positions(frame)
    next_positions = replay.frame_positions(next_frame)
    robot_positions: list[replay.Point3] = []

    for vehicle_index, (current, target) in enumerate(zip(current_positions, next_positions)):
        if (target[0] - current[0]) ** 2 + (target[1] - current[1]) ** 2 < 1e-8:
            desired_yaw = last_yaws[vehicle_index] if last_yaws[vehicle_index] is not None else 0.0
        else:
            desired_yaw = replay.yaw_between(current, target)
        yaw = replay.smooth_yaw(last_yaws[vehicle_index], desired_yaw, yaw_smoothing)
        roll, pitch = replay.motion_attitude(current, target, yaw)
        last_yaws[vehicle_index] = yaw
        robot_positions.append(current)
        rows.append(
            pose_row(
                frame_index,
                f"drone_{vehicle_index + 1}",
                current,
                replay.quaternion_from_euler(roll, pitch, yaw),
            )
        )
        for rotor_index, offset in enumerate(replay.ROTOR_OFFSETS):
            rotor_dx, rotor_dy = replay.rotate_offset((offset[0], offset[1]), yaw)
            rotor_position = (current[0] + rotor_dx, current[1] + rotor_dy, current[2] + 0.082)
            spin_direction = 1.0 if rotor_index in {0, 1} else -1.0
            rotor_spin_angles[vehicle_index][rotor_index] += rotor_speed * period
            rows.append(
                pose_row(
                    frame_index,
                    f"drone_{vehicle_index + 1}_propeller_{rotor_index}",
                    rotor_position,
                    replay.quaternion_from_euler(roll, pitch, yaw + spin_direction * rotor_spin_angles[vehicle_index][rotor_index]),
                )
            )

    desired_camera_yaw = replay.heading_between_centroids(current_positions[:3], next_positions[:3], camera_target_yaw)
    camera_target_yaw = replay.smooth_yaw(camera_target_yaw, desired_camera_yaw, camera_target_yaw_smoothing)
    raw_camera_target = replay.camera_target_from_robots(
        robot_positions[:3],
        camera_target_z_offset,
        camera_target_ahead_distance,
        camera_target_yaw,
    )
    if camera_target is None:
        camera_target = raw_camera_target
    else:
        camera_target = replay.lerp_point(camera_target, raw_camera_target, replay.clamp(camera_target_smoothing, 0.0, 1.0))
    rows.append(pose_row(frame_index, "camera_target_follow", camera_target, replay.quaternion_from_euler(0.0, 0.0, camera_target_yaw)))

    reference_payload = replay.frame_payload(frame)
    if reference_payload is None:
        payload_position = replay.HIDDEN_PAYLOAD
        payload_dynamics.reset(None)
        show_cables = False
    else:
        mode = str(frame.get("task_mode", ""))
        reference_is_above_ground = reference_payload[2] > ground_height + 0.02
        attached = payload_lift_physics and mode == "loaded" and reference_is_above_ground
        if payload_lift_physics:
            payload_position = payload_dynamics.step(reference_payload, attached=attached, dt=dt)
        else:
            payload_position = reference_payload
            payload_dynamics.reset(reference_payload)
        if payload_lift_physics:
            mean_robot_z = sum(position[2] for position in robot_positions[:3]) / max(1, len(robot_positions[:3]))
            robots_are_low = mean_robot_z <= cable_switch_height
            payload_is_above_ground = reference_payload[2] > ground_height + 0.05
            show_cables = mode == "loaded" and (robots_are_low or payload_is_above_ground)
            show_cables = show_cables or (mode == "delivered" and (robots_are_low or payload_is_above_ground))
        else:
            show_cables = replay.cable_visible(frame, payload_position, ground_height)

    payload_roll = 0.0
    payload_pitch = 0.0
    if payload_lift_physics and show_cables and payload_position != replay.HIDDEN_PAYLOAD:
        payload_roll, payload_pitch = replay.payload_attitude_from_anchors(payload_position, [replay.drone_anchor(p) for p in robot_positions[:3]])
        payload_roll *= 0.85
        payload_pitch *= 0.85

    rows.append(pose_row(frame_index, "payload_preview", payload_position, replay.quaternion_from_euler(payload_roll, payload_pitch, 0.0)))
    for hook_index, attach_offset in enumerate(replay.PAYLOAD_ATTACH_OFFSETS, start=1):
        hook_position = replay.payload_hook(payload_position, attach_offset, payload_roll, payload_pitch)
        rows.append(pose_row(frame_index, f"payload_hook_{hook_index}", hook_position, replay.quaternion_from_euler(payload_roll, payload_pitch, 0.0)))

    if show_cables:
        for vehicle_index, robot_position in enumerate(robot_positions[:3]):
            attach_offset = replay.PAYLOAD_ATTACH_OFFSETS[vehicle_index % len(replay.PAYLOAD_ATTACH_OFFSETS)]
            payload_attach = replay.payload_hook(payload_position, attach_offset, payload_roll, payload_pitch)
            anchor = replay.drone_anchor(robot_position)
            rope_points = replay.rope_curve_points(anchor, payload_attach, 1.0, replay.ROPE_SEGMENTS)
            for segment_index in range(replay.ROPE_SEGMENTS):
                start = rope_points[segment_index]
                end = rope_points[segment_index + 1]
                segment_vector = (end[0] - start[0], end[1] - start[1], end[2] - start[2])
                segment_center = ((start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0, (start[2] + end[2]) / 2.0)
                rows.append(
                    pose_row(
                        frame_index,
                        replay.rope_segment_name(vehicle_index, segment_index),
                        segment_center,
                        replay.quaternion_from_z_axis(segment_vector),
                    )
                )
    else:
        q = replay.quaternion_from_euler(0.0, 0.0, 0.0)
        for vehicle_index in range(3):
            for segment_index in range(replay.ROPE_SEGMENTS):
                rows.append(pose_row(frame_index, replay.rope_segment_name(vehicle_index, segment_index), replay.HIDDEN_PAYLOAD, q))

    current_region, target_region, bridge = replay.highlight_selection(
        frame=frame,
        next_state_id=next_state_id,
        state_progress=state_progress,
        lookahead_fraction=highlight_lookahead_fraction,
        states=states,
        transitions=transitions,
        bridges=bridges,
    )
    rows.extend(highlight_rows(frame_index, case, current_region, target_region, bridge))

    return rows, camera_target, camera_target_yaw


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trajectory", type=Path, required=True)
    parser.add_argument("--case", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--rate", type=float, default=90.0)
    parser.add_argument("--rotor-speed", type=float, default=52.0)
    parser.add_argument("--camera-target-z-offset", type=float, default=0.85)
    parser.add_argument("--camera-target-ahead-distance", type=float, default=0.75)
    parser.add_argument("--camera-target-smoothing", type=float, default=0.035)
    parser.add_argument("--camera-target-yaw-smoothing", type=float, default=0.06)
    parser.add_argument("--yaw-smoothing", type=float, default=0.45)
    parser.add_argument("--highlight-lookahead-fraction", type=float, default=0.90)
    parser.add_argument("--payload-lift-physics", action="store_true")
    parser.add_argument("--payload-physics-stiffness", type=float, default=10.0)
    parser.add_argument("--payload-physics-damping", type=float, default=4.5)
    parser.add_argument("--payload-ground-height", type=float, default=0.12)
    parser.add_argument("--start-hold-seconds", type=float, default=0.0)
    parser.add_argument("--cable-switch-height", type=float, default=0.90)
    args = parser.parse_args()

    payload = json.loads(args.trajectory.read_text(encoding="utf-8"))
    frames = replay.load_replay_frames(payload, args.trajectory)
    hold_count = max(0, int(round(args.start_hold_seconds * args.rate)))
    if hold_count and frames:
        frames = [dict(frames[0]) for _ in range(hold_count)] + frames
    case = json.loads(args.case.read_text(encoding="utf-8"))
    states = replay.state_lookup(case)
    transitions = replay.transition_lookup(case)
    bridges = replay.bridge_lookup(case)
    progress_by_frame = replay.state_span_progress(frames)
    period = 1.0 / max(1e-6, args.rate)

    last_yaws: list[float | None] = [None, None, None]
    rotor_spin_angles = [[0.0 for _ in range(4)] for _ in range(3)]
    camera_target: replay.Point3 | None = None
    camera_target_yaw: float | None = None
    payload_dynamics = PayloadVisualDynamics(
        stiffness=args.payload_physics_stiffness,
        damping=args.payload_physics_damping,
        ground_height=args.payload_ground_height,
    )

    args.out.parent.mkdir(parents=True, exist_ok=True)
    row_count = 0
    with args.out.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["frame", "model", "x", "y", "z", "qx", "qy", "qz", "qw"])
        for index, frame in enumerate(frames):
            next_frame = frames[min(index + 1, len(frames) - 1)]
            next_state_id = replay.next_distinct_state_id(frames, index)
            rows, camera_target, camera_target_yaw = frame_rows(
                frame_index=index,
                frame=frame,
                next_frame=next_frame,
                next_state_id=next_state_id,
                state_progress=progress_by_frame[index],
                case=case,
                states=states,
                transitions=transitions,
                bridges=bridges,
                last_yaws=last_yaws,
                rotor_spin_angles=rotor_spin_angles,
                camera_target=camera_target,
                camera_target_yaw=camera_target_yaw,
                rotor_speed=args.rotor_speed,
                period=period,
                camera_target_z_offset=args.camera_target_z_offset,
                camera_target_ahead_distance=args.camera_target_ahead_distance,
                camera_target_smoothing=args.camera_target_smoothing,
                camera_target_yaw_smoothing=args.camera_target_yaw_smoothing,
                yaw_smoothing=args.yaw_smoothing,
                highlight_lookahead_fraction=args.highlight_lookahead_fraction,
                payload_lift_physics=args.payload_lift_physics,
                payload_dynamics=payload_dynamics,
                dt=period,
                ground_height=args.payload_ground_height,
                cable_switch_height=args.cable_switch_height,
            )
            for frame_index, name, position, quaternion in rows:
                qx, qy, qz, qw = quaternion
                writer.writerow(
                    [
                        frame_index,
                        name,
                        f"{position[0]:.6f}",
                        f"{position[1]:.6f}",
                        f"{position[2]:.6f}",
                        f"{qx:.8f}",
                        f"{qy:.8f}",
                        f"{qz:.8f}",
                        f"{qw:.8f}",
                    ]
                )
                row_count += 1

    print(f"[OK] exported Gazebo pose stream: {args.out} frames={len(frames)} rows={row_count}", flush=True)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Replay cached paper-QP trajectory in Gazebo through set_pose_vector."""

from __future__ import annotations

import argparse
import json
import math
import re
import subprocess
import time
from pathlib import Path
from typing import Any, Sequence


Point3 = tuple[float, float, float]
ReplayFrame = dict[str, Any]
HIDDEN_PAYLOAD: Point3 = (-10.0, -10.0, -4.0)
HIDDEN_HIGHLIGHT: Point3 = (-1000.0, -1000.0, -1000.0)
VISIBLE_CURRENT: Point3 = (0.0, 0.0, 0.098)
VISIBLE_TARGET: Point3 = (0.0, 0.0, 0.128)
VISIBLE_BRIDGE: Point3 = (0.0, 0.0, 0.158)
ROTOR_OFFSETS: tuple[tuple[float, float], ...] = (
    (0.174, -0.174),
    (-0.174, 0.174),
    (0.174, 0.174),
    (-0.174, -0.174),
)
CABLE_LENGTHS: tuple[float, ...] = (0.14, 0.20, 0.26, 0.32, 0.38, 0.46, 0.54, 0.64, 0.76, 0.90, 1.08, 1.26)
ROPE_SEGMENTS = 3
PAYLOAD_ATTACH_OFFSETS: tuple[tuple[float, float, float], ...] = (
    (0.13, 0.00, 0.11),
    (-0.07, 0.11, 0.11),
    (-0.07, -0.11, 0.11),
)
GRAVITY = 9.81


def add(a: Point3, b: Point3) -> Point3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a: Point3, b: Point3) -> Point3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def scale(a: Point3, value: float) -> Point3:
    return (a[0] * value, a[1] * value, a[2] * value)


def dot(a: Point3, b: Point3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def norm(a: Point3) -> float:
    return math.sqrt(dot(a, a))


def point(item: dict[str, Any]) -> Point3:
    x, y, z = item["position"][:3]
    return (float(x), float(y), float(z))


def list_point(item: Sequence[float]) -> Point3:
    x, y, z = item[:3]
    return (float(x), float(y), float(z))


def lerp_point(p: Point3, q: Point3, alpha: float) -> Point3:
    return (
        p[0] + (q[0] - p[0]) * alpha,
        p[1] + (q[1] - p[1]) * alpha,
        p[2] + (q[2] - p[2]) * alpha,
    )


def yaw_between(p: Point3, q: Point3) -> float:
    dx = q[0] - p[0]
    dy = q[1] - p[1]
    if dx * dx + dy * dy < 1e-8:
        return 0.0
    return math.atan2(dy, dx)


def yaw_for_track(track: Sequence[Point3], index: int, last_yaw: float) -> float:
    if index + 1 < len(track):
        yaw = yaw_between(track[index], track[index + 1])
        if abs(yaw) > 1e-9:
            return yaw
    if index > 0:
        yaw = yaw_between(track[index - 1], track[index])
        if abs(yaw) > 1e-9:
            return yaw
    return last_yaw


def angle_delta(start: float, end: float) -> float:
    return (end - start + math.pi) % (2.0 * math.pi) - math.pi


def smooth_yaw(last_yaw: float | None, desired_yaw: float, smoothing: float) -> float:
    if last_yaw is None:
        return desired_yaw
    alpha = min(1.0, max(0.0, smoothing))
    return last_yaw + angle_delta(last_yaw, desired_yaw) * alpha


def clamp(value: float, lower: float, upper: float) -> float:
    return min(upper, max(lower, value))


def model_name(name: str) -> str:
    safe = [ch if ch.isalnum() or ch == "_" else "_" for ch in name.lower()]
    return "".join(safe)


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def pose_message(name: str, position: Point3, yaw: float, roll: float = 0.0, pitch: float = 0.0) -> str:
    qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
    return pose_quaternion_message(name, position, (qx, qy, qz, qw))


def pose_quaternion_message(
    name: str,
    position: Point3,
    quaternion: tuple[float, float, float, float],
) -> str:
    qx, qy, qz, qw = quaternion
    return (
        f'pose {{name: "{name}" '
        f"position: {{x: {position[0]:.6f} y: {position[1]:.6f} z: {position[2]:.6f}}} "
        f"orientation: {{x: {qx:.8f} y: {qy:.8f} z: {qz:.8f} w: {qw:.8f}}}}}"
    )


def rotate_offset(offset: tuple[float, float], yaw: float) -> tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (c * offset[0] - s * offset[1], s * offset[0] + c * offset[1])


def rotate_payload_offset(offset: Point3, roll: float, pitch: float) -> Point3:
    x, y, z = offset
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cr = math.cos(roll)
    sr = math.sin(roll)
    x_pitch = cp * x + sp * z
    y_pitch = y
    z_pitch = -sp * x + cp * z
    return (
        x_pitch,
        cr * y_pitch - sr * z_pitch,
        sr * y_pitch + cr * z_pitch,
    )


def motion_attitude(current: Point3, target: Point3, yaw: float) -> tuple[float, float]:
    dx = target[0] - current[0]
    dy = target[1] - current[1]
    forward = math.cos(yaw) * dx + math.sin(yaw) * dy
    lateral = -math.sin(yaw) * dx + math.cos(yaw) * dy
    roll = clamp(0.28 * lateral, -0.16, 0.16)
    pitch = clamp(-0.28 * forward, -0.18, 0.18)
    return roll, pitch


def quaternion_from_z_axis(vector: Point3) -> tuple[float, float, float, float]:
    length = math.sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2])
    if length < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    vx, vy, vz = vector[0] / length, vector[1] / length, vector[2] / length
    if vz < -0.999999:
        return (1.0, 0.0, 0.0, 0.0)
    qx = -vy
    qy = vx
    qz = 0.0
    qw = 1.0 + vz
    qnorm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    return (qx / qnorm, qy / qnorm, qz / qnorm, qw / qnorm)


def scalar_lerp(start: float, end: float, alpha: float) -> float:
    return start + (end - start) * alpha


def angle_lerp(start: float, end: float, alpha: float) -> float:
    return start + angle_delta(start, end) * alpha


def frame_robot_physical(frame: ReplayFrame, vehicle_index: int) -> dict[str, Any] | None:
    robots = (frame.get("physical") or {}).get("robots", [])
    if vehicle_index >= len(robots):
        return None
    return robots[vehicle_index]


def physical_value(physical: dict[str, Any] | None, key: str, default: float) -> float:
    if not physical:
        return default
    return float(physical.get(key, default))


def physical_vector(physical: dict[str, Any] | None, key: str) -> Point3:
    if not physical:
        return (0.0, 0.0, 0.0)
    values = physical.get(key, [0.0, 0.0, 0.0])
    return (float(values[0]), float(values[1]), float(values[2]))


def cable_model_name(vehicle_index: int, length: float) -> str:
    return model_name(f"payload_cable_r{vehicle_index + 1}_{int(round(length * 100)):03d}")


def rope_segment_name(vehicle_index: int, segment_index: int) -> str:
    return model_name(f"payload_rope_r{vehicle_index + 1}_s{segment_index + 1}")


def nearest_cable_length(length: float) -> float:
    return min(CABLE_LENGTHS, key=lambda item: abs(item - length))


def drone_anchor(position: Point3) -> Point3:
    return (position[0], position[1], position[2] - 0.08)


def payload_hook(center: Point3, attach_offset: Point3, roll: float, pitch: float) -> Point3:
    rotated = rotate_payload_offset(attach_offset, roll, pitch)
    return add(center, rotated)


def estimate_rope_rest_lengths(frames: Sequence[ReplayFrame]) -> list[float]:
    """Estimate fixed cable lengths from the first steady carried interval."""
    candidates: list[list[float]] = [[] for _ in range(3)]

    def collect(frame: ReplayFrame) -> None:
        payload_position = frame_payload(frame)
        if payload_position is None:
            return
        for vehicle_index, robot_position in enumerate(frame_positions(frame)[:3]):
            anchor = drone_anchor(robot_position)
            hook = payload_hook(
                payload_position,
                PAYLOAD_ATTACH_OFFSETS[vehicle_index % len(PAYLOAD_ATTACH_OFFSETS)],
                0.0,
                0.0,
            )
            candidates[vehicle_index].append(norm(sub(anchor, hook)))

    for frame in frames:
        phase = str(frame.get("phase", ""))
        if phase == "loaded_climb":
            collect(frame)
        if phase == "loaded_climb" and all(len(values) >= 6 for values in candidates):
            break

    if not all(candidates):
        candidates = [[] for _ in range(3)]
        for frame in frames:
            if str(frame.get("task_mode", "")) == "loaded" and frame_payload(frame) is not None:
                collect(frame)

    rest_lengths: list[float] = []
    for values in candidates:
        if values:
            sorted_values = sorted(values)
            rest_lengths.append(sorted_values[len(sorted_values) // 2])
        else:
            rest_lengths.append(0.9)
    return rest_lengths


def payload_attitude_from_anchors(payload_position: Point3, anchors: Sequence[Point3]) -> tuple[float, float]:
    if not anchors:
        return (0.0, 0.0)
    center = (
        sum(anchor[0] for anchor in anchors) / len(anchors),
        sum(anchor[1] for anchor in anchors) / len(anchors),
        sum(anchor[2] for anchor in anchors) / len(anchors),
    )
    dz = max(0.25, center[2] - payload_position[2])
    roll = clamp(math.atan2(center[1] - payload_position[1], dz) * 0.22, -0.24, 0.24)
    pitch = clamp(-math.atan2(center[0] - payload_position[0], dz) * 0.22, -0.24, 0.24)
    return roll, pitch


class PayloadCableDynamics:
    """Tension-only three-cable translational dynamics for the carried load."""

    def __init__(
        self,
        *,
        mass: float,
        rest_lengths: Sequence[float],
        stiffness: float,
        damping: float,
        air_damping: float,
        ground_height: float,
        physics_substeps: int,
    ) -> None:
        self.mass = max(0.05, mass)
        self.rest_lengths = [max(0.05, float(value)) for value in rest_lengths]
        self.stiffness = max(0.0, stiffness)
        self.damping = max(0.0, damping)
        self.air_damping = max(0.0, air_damping)
        self.ground_height = ground_height
        self.physics_substeps = max(1, physics_substeps)
        self.position: Point3 | None = None
        self.velocity: Point3 = (0.0, 0.0, 0.0)
        self.last_tensions: list[float] = [0.0, 0.0, 0.0]

    def reset(self, position: Point3 | None = None) -> None:
        self.position = position
        self.velocity = (0.0, 0.0, 0.0)
        self.last_tensions = [0.0, 0.0, 0.0]

    def step(
        self,
        *,
        initial_position: Point3,
        anchors: Sequence[Point3],
        anchor_velocities: Sequence[Point3],
        dt: float,
        roll: float,
        pitch: float,
    ) -> Point3:
        if self.position is None:
            self.position = initial_position
            self.velocity = (0.0, 0.0, 0.0)

        step_dt = dt / self.physics_substeps
        for _ in range(self.physics_substeps):
            force = (0.0, 0.0, -self.mass * GRAVITY)
            tensions: list[float] = []
            for vehicle_index, anchor in enumerate(anchors[:3]):
                attach_offset = PAYLOAD_ATTACH_OFFSETS[vehicle_index % len(PAYLOAD_ATTACH_OFFSETS)]
                hook = payload_hook(self.position, attach_offset, roll, pitch)
                cable = sub(anchor, hook)
                length = norm(cable)
                if length < 1e-6:
                    tensions.append(0.0)
                    continue
                direction = scale(cable, 1.0 / length)
                rest = self.rest_lengths[min(vehicle_index, len(self.rest_lengths) - 1)]
                extension = length - rest
                anchor_velocity = anchor_velocities[vehicle_index] if vehicle_index < len(anchor_velocities) else (0.0, 0.0, 0.0)
                relative_speed = dot(sub(anchor_velocity, self.velocity), direction)
                tension = max(0.0, self.stiffness * extension + self.damping * relative_speed)
                tensions.append(tension)
                force = add(force, scale(direction, tension))

            force = add(force, scale(self.velocity, -self.air_damping))
            acceleration = scale(force, 1.0 / self.mass)
            acceleration = (
                clamp(acceleration[0], -12.0, 12.0),
                clamp(acceleration[1], -12.0, 12.0),
                clamp(acceleration[2], -16.0, 16.0),
            )
            self.velocity = add(self.velocity, scale(acceleration, step_dt))
            speed = norm(self.velocity)
            if speed > 4.5:
                self.velocity = scale(self.velocity, 4.5 / speed)
            self.position = add(self.position, scale(self.velocity, step_dt))

            if self.position[2] < self.ground_height:
                self.position = (self.position[0], self.position[1], self.ground_height)
                self.velocity = (self.velocity[0] * 0.72, self.velocity[1] * 0.72, max(0.0, self.velocity[2]) * 0.18)
            self.last_tensions = tensions + [0.0] * (3 - len(tensions))

        return self.position


def rope_curve_points(
    anchor: Point3,
    payload_attach: Point3,
    tension: float,
    segment_count: int,
) -> list[Point3]:
    points: list[Point3] = []
    dx = anchor[0] - payload_attach[0]
    dy = anchor[1] - payload_attach[1]
    dz = anchor[2] - payload_attach[2]
    for segment in range(segment_count + 1):
        t = segment / segment_count
        points.append(
            (
                payload_attach[0] + dx * t,
                payload_attach[1] + dy * t,
                payload_attach[2] + dz * t,
            )
        )
    return points


def hide_rope_messages() -> list[str]:
    messages: list[str] = []
    for vehicle_index in range(3):
        for segment_index in range(ROPE_SEGMENTS):
            messages.append(pose_message(rope_segment_name(vehicle_index, segment_index), HIDDEN_PAYLOAD, 0.0))
    return messages


def call_set_pose_vector(world: str, messages: list[str], timeout_ms: int) -> bool:
    request = " ".join(messages)
    result = subprocess.run(
        [
            "ign",
            "service",
            "-s",
            f"/world/{world}/set_pose_vector",
            "--reqtype",
            "ignition.msgs.Pose_V",
            "--reptype",
            "ignition.msgs.Boolean",
            "--timeout",
            str(timeout_ms),
            "--req",
            request,
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    return result.returncode == 0


def legacy_replay_frames(payload: dict[str, Any]) -> list[ReplayFrame]:
    tracks: list[list[Point3]] = [[point(item) for item in track] for track in payload["tracks"][:3]]
    max_len = max(len(track) for track in tracks)
    frames = []
    for index in range(max_len):
        positions = [track[min(index, len(track) - 1)] for track in tracks]
        payload_position = (
            sum(position[0] for position in positions) / 3.0,
            sum(position[1] for position in positions) / 3.0,
            max(0.24, sum(position[2] for position in positions) / 3.0 - 0.46),
        )
        frames.append(
            {
                "source_frame_index": index,
                "state_index": index,
                "state_id": "legacy",
                "task_mode": "loaded",
                "formation": "unknown",
                "robots": [[round(v, 6) for v in position] for position in positions],
                "payload": [round(v, 6) for v in payload_position],
            }
        )
    return frames


def load_replay_frames(payload: dict[str, Any], trajectory: Path) -> list[ReplayFrame]:
    if payload.get("replay_frames"):
        frames = payload["replay_frames"]
    else:
        frames = legacy_replay_frames(payload)
    if not frames:
        raise RuntimeError(f"Expected nonempty replay frames in {trajectory}")
    for frame in frames:
        robots = frame.get("robots", [])
        if len(robots) != 3:
            raise RuntimeError(f"Expected three robot poses in replay frame from {trajectory}")
    return frames


def frame_positions(frame: ReplayFrame) -> list[Point3]:
    return [list_point(robot) for robot in frame["robots"][:3]]


def frame_payload(frame: ReplayFrame) -> Point3 | None:
    if frame.get("payload") is None:
        return None
    return list_point(frame["payload"])


def max_frame_displacement(
    current_positions: Sequence[Point3],
    next_positions: Sequence[Point3],
    current_payload: Point3 | None,
    next_payload: Point3 | None,
) -> float:
    distances = [
        norm(sub(target, current))
        for current, target in zip(current_positions, next_positions)
    ]
    if current_payload is not None and next_payload is not None:
        distances.append(norm(sub(next_payload, current_payload)))
    return max(distances) if distances else 0.0


def centroid(positions: Sequence[Point3]) -> Point3:
    if not positions:
        return HIDDEN_PAYLOAD
    return (
        sum(position[0] for position in positions) / len(positions),
        sum(position[1] for position in positions) / len(positions),
        sum(position[2] for position in positions) / len(positions),
    )


def heading_between_centroids(
    current_positions: Sequence[Point3],
    next_positions: Sequence[Point3],
    fallback_yaw: float | None,
) -> float:
    current_center = centroid(current_positions)
    next_center = centroid(next_positions)
    dx = next_center[0] - current_center[0]
    dy = next_center[1] - current_center[1]
    if dx * dx + dy * dy < 1e-7:
        return fallback_yaw if fallback_yaw is not None else 0.0
    return math.atan2(dy, dx)


def camera_target_from_robots(
    positions: Sequence[Point3],
    z_offset: float,
    ahead_distance: float,
    yaw: float,
) -> Point3:
    center = centroid(positions)
    if center == HIDDEN_PAYLOAD:
        return center
    return (
        center[0] + math.cos(yaw) * ahead_distance,
        center[1] + math.sin(yaw) * ahead_distance,
        center[2] + z_offset,
    )


def region_names(region_text: str | None) -> list[str]:
    if not region_text:
        return []
    return re.findall(r"P\d+", region_text)


def state_lookup(case: dict[str, Any]) -> dict[str, dict[str, Any]]:
    return {str(state["state_id"]): state for state in case.get("states", [])}


def transition_lookup(case: dict[str, Any]) -> dict[tuple[str, str], dict[str, Any]]:
    return {
        (str(transition["src"]), str(transition["dst"])): transition
        for transition in case.get("transitions", [])
    }


def bridge_lookup(case: dict[str, Any]) -> dict[tuple[frozenset[str], str], str]:
    bridges: dict[tuple[frozenset[str], str], str] = {}
    for bridge in case.get("bridges", []):
        names = frozenset(str(name) for name in bridge.get("regions", []))
        formation = str(bridge.get("formation", ""))
        bridges[(names, formation)] = str(bridge["name"])
    return bridges


def next_distinct_state_id(frames: Sequence[ReplayFrame], start_index: int) -> str | None:
    current = str(frames[start_index].get("state_id", ""))
    for frame in frames[start_index + 1 :]:
        state_id = str(frame.get("state_id", ""))
        if state_id and state_id != current:
            return state_id
    return None


def state_span_progress(frames: Sequence[ReplayFrame]) -> list[float]:
    """Return each frame's progress through its contiguous symbolic state span."""
    if not frames:
        return []

    progress = [0.0 for _ in frames]
    start = 0
    current = str(frames[0].get("state_id", ""))
    for index in range(1, len(frames) + 1):
        state_id = str(frames[index].get("state_id", "")) if index < len(frames) else None
        if state_id == current:
            continue

        end = index - 1
        span = max(1, end - start)
        for frame_index in range(start, end + 1):
            progress[frame_index] = (frame_index - start) / span
        if index < len(frames):
            start = index
            current = str(frames[index].get("state_id", ""))

    return progress


def highlight_selection(
    *,
    frame: ReplayFrame,
    next_state_id: str | None,
    state_progress: float,
    lookahead_fraction: float,
    states: dict[str, dict[str, Any]],
    transitions: dict[tuple[str, str], dict[str, Any]],
    bridges: dict[tuple[frozenset[str], str], str],
) -> tuple[str | None, str | None, str | None]:
    state_id = str(frame.get("state_id", ""))
    state = states.get(state_id, {})
    formation = str(state.get("formation", frame.get("formation", "")))

    state_names = region_names(str(state.get("region", "")))
    current = state_names[0] if state_names else None
    target = state_names[-1] if len(state_names) > 1 else None
    bridge = None
    if len(state_names) >= 2:
        bridge = bridges.get((frozenset(state_names[:2]), formation))
        return current, target, bridge

    if state_progress >= lookahead_fraction:
        transition = transitions.get((state_id, next_state_id or ""))
        transition_names = region_names(str(transition.get("certificate", ""))) if transition else []
        if len(transition_names) >= 2:
            target = transition_names[-1]
            bridge = bridges.get((frozenset(transition_names[:2]), formation))

    return current, target, bridge


def highlight_messages(
    current: str | None,
    target: str | None,
    bridge: str | None,
    last: tuple[str | None, str | None, str | None],
) -> tuple[list[str], tuple[str | None, str | None, str | None]]:
    messages: list[str] = []
    last_current, last_target, last_bridge = last

    if last_current and last_current != current:
        messages.append(pose_message(model_name(f"highlight_current_{last_current}"), HIDDEN_HIGHLIGHT, 0.0))
    if last_target and last_target != target:
        messages.append(pose_message(model_name(f"highlight_target_{last_target}"), HIDDEN_HIGHLIGHT, 0.0))
    if last_bridge and last_bridge != bridge:
        messages.append(pose_message(model_name(f"highlight_bridge_{last_bridge}"), HIDDEN_HIGHLIGHT, 0.0))

    if current:
        messages.append(pose_message(model_name(f"highlight_current_{current}"), VISIBLE_CURRENT, 0.0))
    if target:
        messages.append(pose_message(model_name(f"highlight_target_{target}"), VISIBLE_TARGET, 0.0))
    if bridge:
        messages.append(pose_message(model_name(f"highlight_bridge_{bridge}"), VISIBLE_BRIDGE, 0.0))

    return messages, (current, target, bridge)


def cable_visible(frame: ReplayFrame, payload_position: Point3, payload_ground_height: float) -> bool:
    phase = str(frame.get("phase", ""))
    mode = str(frame.get("task_mode", ""))
    if phase in {"pickup_hold", "payload_hoist", "loaded_climb", "drop_descent", "drop_hold"}:
        return True
    if mode == "loaded" and payload_position[2] > payload_ground_height + 0.05:
        return True
    return False


def cable_attached(frame: ReplayFrame, payload_position: Point3, payload_ground_height: float) -> bool:
    phase = str(frame.get("phase", ""))
    mode = str(frame.get("task_mode", ""))
    if phase in {"pickup_hold", "payload_hoist", "loaded_climb", "drop_descent", "drop_hold"}:
        return True
    if mode == "loaded" and phase != "payload_release" and payload_position[2] > payload_ground_height + 0.05:
        return True
    return False


def replay(args: argparse.Namespace) -> None:
    payload = json.loads(Path(args.trajectory).read_text(encoding="utf-8"))
    frames = load_replay_frames(payload, args.trajectory)
    if args.max_frames > 0:
        frames = frames[: args.max_frames]
    frame_step = max(1, int(args.frame_step))
    source_frame_count = len(frames)
    if frame_step > 1 and len(frames) > 1:
        stepped_frames = frames[::frame_step]
        if stepped_frames[-1] != frames[-1]:
            stepped_frames.append(frames[-1])
        frames = stepped_frames
    case_path = Path(payload.get("case", ""))
    if not case_path.is_absolute():
        case_path = Path(args.trajectory).resolve().parents[2] / case_path
    case = json.loads(case_path.read_text(encoding="utf-8")) if case_path.exists() else {}
    states = state_lookup(case)
    transitions = transition_lookup(case)
    bridges = bridge_lookup(case)
    progress_by_frame = state_span_progress(frames)

    last_yaws: list[float | None] = [None, None, None]
    last_highlight: tuple[str | None, str | None, str | None] = (None, None, None)
    rotor_spin_angles = [[0.0 for _ in range(4)] for _ in range(3)]
    ropes_visible = False
    payload_roll = 0.0
    payload_pitch = 0.0
    camera_target: Point3 | None = None
    camera_target_yaw: float | None = None
    rope_rest_lengths = [
        value * float(args.rope_rest_scale)
        for value in estimate_rope_rest_lengths(frames)
    ]
    payload_dynamics = PayloadCableDynamics(
        mass=float(args.payload_mass),
        rest_lengths=rope_rest_lengths,
        stiffness=float(args.rope_stiffness),
        damping=float(args.rope_damping),
        air_damping=float(args.payload_air_damping),
        ground_height=float(args.payload_ground_height),
        physics_substeps=int(args.payload_physics_substeps),
    )
    period = 1.0 / max(1e-6, float(args.rate))

    time.sleep(max(0.0, float(args.start_delay)))
    print(
        f"[replay] Gazebo set_pose_vector replay: world={args.world} "
        f"frames={len(frames)}/{source_frame_count} frame_step={frame_step} "
        f"substeps={args.substeps} rate={args.rate}Hz loop={args.loop}",
        flush=True,
    )
    print(
        "[replay] payload dynamics: "
        f"mass={args.payload_mass:.2f}kg rest_lengths={[round(v, 3) for v in rope_rest_lengths]} "
        f"k={args.rope_stiffness:.1f} c={args.rope_damping:.1f}",
        flush=True,
    )

    while True:
        for index, frame in enumerate(frames):
            next_frame = frames[min(index + 1, len(frames) - 1)]
            current_positions = frame_positions(frame)
            next_positions = frame_positions(next_frame)
            current_payload = frame_payload(frame)
            next_payload = frame_payload(next_frame)
            next_state_id = next_distinct_state_id(frames, index)
            substeps = max(1, int(args.substeps))
            if args.max_replay_step > 0.0:
                distance = max_frame_displacement(current_positions, next_positions, current_payload, next_payload)
                adaptive_substeps = max(1, math.ceil(distance / max(1e-6, args.max_replay_step)))
                if args.max_interpolation_substeps > 0:
                    adaptive_substeps = min(adaptive_substeps, int(args.max_interpolation_substeps))
                substeps = max(substeps, adaptive_substeps)
            for substep in range(substeps):
                tick_start = time.monotonic()
                alpha = substep / substeps
                messages: list[str] = []
                robot_positions: list[Point3] = []
                for vehicle_index, (current, target) in enumerate(zip(current_positions, next_positions)):
                    position = lerp_point(current, target, alpha)
                    current_physical = frame_robot_physical(frame, vehicle_index)
                    next_physical = frame_robot_physical(next_frame, vehicle_index)
                    if current_physical and next_physical:
                        yaw = angle_lerp(
                            physical_value(current_physical, "yaw", last_yaws[vehicle_index] or 0.0),
                            physical_value(next_physical, "yaw", last_yaws[vehicle_index] or 0.0),
                            alpha,
                        )
                        roll = scalar_lerp(
                            physical_value(current_physical, "roll", 0.0),
                            physical_value(next_physical, "roll", 0.0),
                            alpha,
                        )
                        pitch = scalar_lerp(
                            physical_value(current_physical, "pitch", 0.0),
                            physical_value(next_physical, "pitch", 0.0),
                            alpha,
                        )
                        rotor_speed = scalar_lerp(
                            physical_value(current_physical, "rotor_speed", args.rotor_speed),
                            physical_value(next_physical, "rotor_speed", args.rotor_speed),
                            alpha,
                        )
                    else:
                        if (target[0] - position[0]) ** 2 + (target[1] - position[1]) ** 2 < 1e-8:
                            desired_yaw = last_yaws[vehicle_index] if last_yaws[vehicle_index] is not None else 0.0
                        else:
                            desired_yaw = yaw_between(position, target)
                        yaw = smooth_yaw(last_yaws[vehicle_index], desired_yaw, args.yaw_smoothing)
                        roll, pitch = motion_attitude(position, target, yaw)
                        rotor_speed = args.rotor_speed
                    last_yaws[vehicle_index] = yaw
                    messages.append(pose_message(f"drone_{vehicle_index + 1}", position, yaw, roll, pitch))
                    robot_positions.append(position)
                    for rotor_index, offset in enumerate(ROTOR_OFFSETS):
                        rotor_dx, rotor_dy = rotate_offset(
                            (offset[0] * args.drone_mesh_scale, offset[1] * args.drone_mesh_scale),
                            yaw,
                        )
                        rotor_position = (
                            position[0] + rotor_dx,
                            position[1] + rotor_dy,
                            position[2] + 0.082 * args.drone_mesh_scale,
                        )
                        spin_direction = 1.0 if rotor_index in {0, 1} else -1.0
                        rotor_spin_angles[vehicle_index][rotor_index] += rotor_speed * period
                        messages.append(
                            pose_message(
                                f"drone_{vehicle_index + 1}_propeller_{rotor_index}",
                                rotor_position,
                                yaw + spin_direction * rotor_spin_angles[vehicle_index][rotor_index],
                                roll,
                                pitch,
                            )
                    )

                desired_camera_yaw = heading_between_centroids(
                    current_positions[:3],
                    next_positions[:3],
                    camera_target_yaw,
                )
                camera_target_yaw = smooth_yaw(
                    camera_target_yaw,
                    desired_camera_yaw,
                    float(args.camera_target_yaw_smoothing),
                )
                raw_camera_target = camera_target_from_robots(
                    robot_positions[:3],
                    float(args.camera_target_z_offset),
                    float(args.camera_target_ahead_distance),
                    camera_target_yaw,
                )
                if camera_target is None:
                    camera_target = raw_camera_target
                else:
                    camera_target = lerp_point(
                        camera_target,
                        raw_camera_target,
                        clamp(float(args.camera_target_smoothing), 0.0, 1.0),
                    )
                messages.append(pose_message("camera_target_follow", camera_target, camera_target_yaw))

                if current_payload is None:
                    reference_payload = HIDDEN_PAYLOAD
                    payload_position = HIDDEN_PAYLOAD
                    payload_dynamics.reset(None)
                    payload_roll = 0.0
                    payload_pitch = 0.0
                    show_cables = False
                else:
                    reference_payload = current_payload if next_payload is None else lerp_point(current_payload, next_payload, alpha)
                    anchors = [drone_anchor(position) for position in robot_positions[:3]]
                    show_cables = cable_visible(frame, reference_payload, args.payload_ground_height)
                    payload_position = reference_payload
                    payload_dynamics.reset(reference_payload)
                    payload_roll = 0.0
                    payload_pitch = 0.0
                messages.append(pose_message("payload_preview", payload_position, 0.0, payload_roll, payload_pitch))
                for hook_index, attach_offset in enumerate(PAYLOAD_ATTACH_OFFSETS, start=1):
                    hook_position = payload_hook(payload_position, attach_offset, payload_roll, payload_pitch)
                    messages.append(
                        pose_message(
                            f"payload_hook_{hook_index}",
                            hook_position,
                            0.0,
                            payload_roll,
                            payload_pitch,
                        )
                    )

                if show_cables:
                    for vehicle_index, robot_position in enumerate(robot_positions[:3]):
                        attach_offset = PAYLOAD_ATTACH_OFFSETS[vehicle_index % len(PAYLOAD_ATTACH_OFFSETS)]
                        payload_attach = payload_hook(payload_position, attach_offset, payload_roll, payload_pitch)
                        anchor = drone_anchor(robot_position)
                        rope_points = rope_curve_points(anchor, payload_attach, 1.0, ROPE_SEGMENTS)
                        for segment_index in range(ROPE_SEGMENTS):
                            start = rope_points[segment_index]
                            end = rope_points[segment_index + 1]
                            segment_vector = (
                                end[0] - start[0],
                                end[1] - start[1],
                                end[2] - start[2],
                            )
                            segment_center = (
                                (start[0] + end[0]) / 2.0,
                                (start[1] + end[1]) / 2.0,
                                (start[2] + end[2]) / 2.0,
                            )
                            messages.append(
                                pose_quaternion_message(
                                    rope_segment_name(vehicle_index, segment_index),
                                    segment_center,
                                    quaternion_from_z_axis(segment_vector),
                                )
                            )
                    ropes_visible = True
                elif ropes_visible:
                    messages.extend(hide_rope_messages())
                    ropes_visible = False

                current_region, target_region, bridge = highlight_selection(
                    frame=frame,
                    next_state_id=next_state_id,
                    state_progress=progress_by_frame[index],
                    lookahead_fraction=float(args.highlight_lookahead_fraction),
                    states=states,
                    transitions=transitions,
                    bridges=bridges,
                )
                highlight_update, last_highlight = highlight_messages(
                    current_region,
                    target_region,
                    bridge,
                    last_highlight,
                )
                messages.extend(highlight_update)

                if not call_set_pose_vector(args.world, messages, args.timeout_ms):
                    print("[replay] set_pose_vector failed; is Gazebo still running?", flush=True)
                    return

                elapsed = time.monotonic() - tick_start
                time.sleep(max(0.0, period - elapsed))
        if not args.loop:
            return


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trajectory", type=Path, required=True)
    parser.add_argument("--world", default="payload_transport_scene")
    parser.add_argument("--rate", type=float, default=9.0)
    parser.add_argument("--start-delay", type=float, default=4.0)
    parser.add_argument("--payload-drop", type=float, default=0.36)
    parser.add_argument("--payload-ground-height", type=float, default=0.12)
    parser.add_argument("--payload-mass", type=float, default=0.6)
    parser.add_argument("--rope-stiffness", type=float, default=85.0)
    parser.add_argument("--rope-damping", type=float, default=7.5)
    parser.add_argument("--rope-rest-scale", type=float, default=1.0)
    parser.add_argument("--payload-air-damping", type=float, default=0.35)
    parser.add_argument("--payload-physics-substeps", type=int, default=5)
    parser.add_argument("--drone-mesh-scale", type=float, default=1.0)
    parser.add_argument("--rotor-speed", type=float, default=52.0)
    parser.add_argument("--substeps", type=int, default=2)
    parser.add_argument("--max-replay-step", type=float, default=0.0)
    parser.add_argument("--max-interpolation-substeps", type=int, default=0)
    parser.add_argument("--camera-target-z-offset", type=float, default=0.65)
    parser.add_argument("--camera-target-ahead-distance", type=float, default=0.65)
    parser.add_argument("--camera-target-smoothing", type=float, default=0.18)
    parser.add_argument("--camera-target-yaw-smoothing", type=float, default=0.22)
    parser.add_argument("--yaw-smoothing", type=float, default=0.35)
    parser.add_argument("--highlight-lookahead-fraction", type=float, default=0.88)
    parser.add_argument("--timeout-ms", type=int, default=1000)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--frame-step", type=int, default=1)
    parser.add_argument("--loop", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    try:
        replay(args)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

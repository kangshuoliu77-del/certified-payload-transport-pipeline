#!/usr/bin/env python3
"""Export paper-QP execution frames as a cached Gazebo tracker trajectory."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_SRC = ROOT / "src" / "swarm_random_payload"
sys.path.insert(0, str(PACKAGE_SRC))

from swarm_random_payload.random_payload_model import PayloadTransportScenario  # noqa: E402


Point3 = tuple[float, float, float]
GRAVITY = 9.81


def clamp(value: float, lower: float, upper: float) -> float:
    return min(upper, max(lower, value))


def waypoint_payload(position: tuple[float, float, float], velocity: tuple[float, float, float], frame_index: int) -> dict[str, Any]:
    return {
        "position": [round(value, 6) for value in position],
        "velocity": [round(value, 6) for value in velocity],
        "frame_index": frame_index,
    }


def round_point(point: Point3) -> list[float]:
    return [round(value, 6) for value in point]


def lerp_point(start: Point3, end: Point3, tau: float) -> Point3:
    return (
        start[0] + (end[0] - start[0]) * tau,
        start[1] + (end[1] - start[1]) * tau,
        start[2] + (end[2] - start[2]) * tau,
    )


def sub_point(a: Point3, b: Point3) -> Point3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def scale_point(point: Point3, scale: float) -> Point3:
    return (point[0] * scale, point[1] * scale, point[2] * scale)


def norm(point: Point3) -> float:
    return (point[0] * point[0] + point[1] * point[1] + point[2] * point[2]) ** 0.5


def yaw_from_velocity(velocity: Point3, previous: float) -> float:
    speed_xy = (velocity[0] * velocity[0] + velocity[1] * velocity[1]) ** 0.5
    if speed_xy < 1e-4:
        return previous
    return math.atan2(velocity[1], velocity[0])


def with_robot_altitude(robots: list[list[float]], altitude: float) -> list[Point3]:
    return [(float(robot[0]), float(robot[1]), altitude) for robot in robots]


def make_replay_frame(
    *,
    source_frame_index: int,
    state_index: int,
    state_id: str,
    task_mode: str,
    formation: str,
    robots: list[Point3],
    payload: Point3 | None,
    phase: str,
) -> dict[str, Any]:
    return {
        "source_frame_index": source_frame_index,
        "state_index": state_index,
        "state_id": state_id,
        "task_mode": task_mode,
        "formation": formation,
        "phase": phase,
        "robots": [round_point(robot) for robot in robots],
        "payload": round_point(payload) if payload is not None else None,
    }


def replay_frame_payload(
    *,
    scenario: PayloadTransportScenario,
    frame_index: int,
    altitude: float,
    payload_ground_height: float,
    payload_drop: float,
    pick_ground: Point3,
    drop_ground: Point3,
) -> dict[str, Any]:
    frame = scenario.frames[frame_index]
    state = scenario.states[frame.state_index]
    robots: list[Point3] = []
    for robot in frame.robots[:3]:
        x_world, y_world = scenario.to_world(robot)
        robots.append((x_world, y_world, altitude))

    if state.task_mode == "loaded":
        payload_position = (
            sum(robot[0] for robot in robots) / 3.0,
            sum(robot[1] for robot in robots) / 3.0,
            max(payload_ground_height, altitude - payload_drop),
        )
    elif state.task_mode == "empty":
        payload_position = pick_ground
    else:
        payload_position = drop_ground

    return make_replay_frame(
        source_frame_index=frame_index,
        state_index=frame.state_index,
        state_id=state.state_id,
        task_mode=state.task_mode,
        formation=state.formation,
        robots=robots,
        payload=payload_position,
        phase="flight",
    )


def add_event_motion(
    frames: list[dict[str, Any]],
    *,
    base: dict[str, Any],
    event: str,
    start_robots: list[Point3],
    end_robots: list[Point3],
    start_payload: Point3,
    end_payload: Point3,
    steps: int,
) -> None:
    steps = max(1, steps)
    for step in range(1, steps + 1):
        tau = step / steps
        robots = [lerp_point(start, end, tau) for start, end in zip(start_robots, end_robots)]
        payload = lerp_point(start_payload, end_payload, tau)
        frames.append(
            make_replay_frame(
                source_frame_index=int(base["source_frame_index"]),
                state_index=int(base["state_index"]),
                state_id=str(base["state_id"]),
                task_mode=str(base["task_mode"]),
                formation=str(base["formation"]),
                robots=robots,
                payload=payload,
                phase=event,
            )
        )


def add_event_hold(
    frames: list[dict[str, Any]],
    *,
    base: dict[str, Any],
    event: str,
    robots: list[Point3],
    payload: Point3,
    steps: int,
) -> None:
    steps = max(1, steps)
    for _ in range(steps):
        frames.append(
            make_replay_frame(
                source_frame_index=int(base["source_frame_index"]),
                state_index=int(base["state_index"]),
                state_id=str(base["state_id"]),
                task_mode=str(base["task_mode"]),
                formation=str(base["formation"]),
                robots=robots,
                payload=payload,
                phase=event,
            )
        )


def expand_payload_events(
    raw_frames: list[dict[str, Any]],
    *,
    cruise_altitude: float,
    pickup_altitude: float,
    payload_drop: float,
    payload_ground_height: float,
    event_steps: int,
) -> list[dict[str, Any]]:
    if not raw_frames:
        return []

    expanded = [raw_frames[0]]
    for frame in raw_frames[1:]:
        previous = expanded[-1]
        prev_mode = str(previous["task_mode"])
        mode = str(frame["task_mode"])
        if prev_mode != "loaded" and mode == "loaded":
            hover_xy = with_robot_altitude(frame["robots"], cruise_altitude)
            low_robots = with_robot_altitude(frame["robots"], pickup_altitude)
            ground_payload = (
                sum(robot[0] for robot in low_robots) / 3.0,
                sum(robot[1] for robot in low_robots) / 3.0,
                payload_ground_height,
            )
            low_payload = (
                ground_payload[0],
                ground_payload[1],
                max(payload_ground_height, pickup_altitude - payload_drop),
            )
            lift_robots = with_robot_altitude(frame["robots"], min(cruise_altitude, pickup_altitude + payload_drop))
            lifted_payload = (
                ground_payload[0],
                ground_payload[1],
                max(payload_ground_height, lift_robots[0][2] - payload_drop),
            )
            carried_payload = (
                ground_payload[0],
                ground_payload[1],
                max(payload_ground_height, cruise_altitude - payload_drop),
            )
            add_event_motion(
                expanded,
                base=frame,
                event="pickup_descent",
                start_robots=hover_xy,
                end_robots=low_robots,
                start_payload=ground_payload,
                end_payload=ground_payload,
                steps=event_steps,
            )
            add_event_hold(
                expanded,
                base=frame,
                event="pickup_hold",
                robots=low_robots,
                payload=ground_payload,
                steps=max(6, event_steps // 2),
            )
            add_event_motion(
                expanded,
                base=frame,
                event="payload_hoist",
                start_robots=low_robots,
                end_robots=lift_robots,
                start_payload=ground_payload,
                end_payload=lifted_payload,
                steps=event_steps,
            )
            add_event_motion(
                expanded,
                base=frame,
                event="loaded_climb",
                start_robots=lift_robots,
                end_robots=hover_xy,
                start_payload=lifted_payload,
                end_payload=carried_payload,
                steps=event_steps,
            )
        elif prev_mode == "loaded" and mode != "loaded":
            hover_xy = with_robot_altitude(previous["robots"], cruise_altitude)
            low_robots = with_robot_altitude(previous["robots"], pickup_altitude)
            carried_payload = previous["payload"]
            if carried_payload is None:
                carried_payload = (
                    sum(robot[0] for robot in hover_xy) / 3.0,
                    sum(robot[1] for robot in hover_xy) / 3.0,
                    max(payload_ground_height, cruise_altitude - payload_drop),
                )
            else:
                carried_payload = (float(carried_payload[0]), float(carried_payload[1]), float(carried_payload[2]))
            low_payload = (
                carried_payload[0],
                carried_payload[1],
                max(payload_ground_height, pickup_altitude - payload_drop),
            )
            ground_payload = (
                carried_payload[0],
                carried_payload[1],
                payload_ground_height,
            )
            add_event_motion(
                expanded,
                base=frame,
                event="drop_descent",
                start_robots=hover_xy,
                end_robots=low_robots,
                start_payload=carried_payload,
                end_payload=low_payload,
                steps=event_steps,
            )
            add_event_hold(
                expanded,
                base=frame,
                event="drop_hold",
                robots=low_robots,
                payload=low_payload,
                steps=max(6, event_steps // 2),
            )
            add_event_motion(
                expanded,
                base=frame,
                event="payload_release",
                start_robots=low_robots,
                end_robots=low_robots,
                start_payload=low_payload,
                end_payload=ground_payload,
                steps=event_steps,
            )
            add_event_motion(
                expanded,
                base=frame,
                event="empty_climb",
                start_robots=low_robots,
                end_robots=with_robot_altitude(frame["robots"], cruise_altitude),
                start_payload=ground_payload,
                end_payload=ground_payload,
                steps=event_steps,
            )
        expanded.append(frame)
    return expanded


def add_start_hold(frames: list[dict[str, Any]], steps: int) -> list[dict[str, Any]]:
    if not frames or steps <= 0:
        return frames
    start_frame = dict(frames[0])
    start_frame["phase"] = "start_hold"
    return [dict(start_frame) for _ in range(steps)] + frames


def frame_robot_point(frame: dict[str, Any], robot_index: int) -> Point3:
    return tuple(float(value) for value in frame["robots"][robot_index][:3])  # type: ignore[return-value]


def finite_velocity(points: list[Point3], index: int, dt: float) -> Point3:
    if len(points) == 1:
        return (0.0, 0.0, 0.0)
    if index == 0:
        return scale_point(sub_point(points[1], points[0]), 1.0 / dt)
    if index == len(points) - 1:
        return scale_point(sub_point(points[-1], points[-2]), 1.0 / dt)
    return scale_point(sub_point(points[index + 1], points[index - 1]), 0.5 / dt)


def finite_acceleration(points: list[Point3], index: int, dt: float) -> Point3:
    if index == 0 or index == len(points) - 1:
        return (0.0, 0.0, 0.0)
    return scale_point(
        (
            points[index + 1][0] - 2.0 * points[index][0] + points[index - 1][0],
            points[index + 1][1] - 2.0 * points[index][1] + points[index - 1][1],
            points[index + 1][2] - 2.0 * points[index][2] + points[index - 1][2],
        ),
        1.0 / (dt * dt),
    )


def limited_acceleration(acceleration: Point3, limit: float) -> Point3:
    magnitude = norm(acceleration)
    if magnitude <= limit or magnitude < 1e-9:
        return acceleration
    return scale_point(acceleration, limit / magnitude)


def attitude_from_acceleration(acceleration: Point3, yaw: float) -> tuple[float, float]:
    ax, ay, _ = acceleration
    forward_accel = math.cos(yaw) * ax + math.sin(yaw) * ay
    lateral_accel = -math.sin(yaw) * ax + math.cos(yaw) * ay
    roll = clamp(lateral_accel / GRAVITY, -0.24, 0.24)
    pitch = clamp(-forward_accel / GRAVITY, -0.28, 0.28)
    return roll, pitch


def add_physical_metadata(
    frames: list[dict[str, Any]],
    *,
    visual_dt: float,
    vehicle_mass: float,
    payload_mass: float,
    hover_rotor_speed: float,
    acceleration_limit: float,
) -> None:
    if not frames:
        return

    vehicle_count = len(frames[0]["robots"])
    positions = [
        [frame_robot_point(frame, vehicle_index) for frame in frames]
        for vehicle_index in range(vehicle_count)
    ]
    velocities = [
        [finite_velocity(track, index, visual_dt) for index in range(len(frames))]
        for track in positions
    ]
    accelerations = [
        [limited_acceleration(finite_acceleration(track, index, visual_dt), acceleration_limit) for index in range(len(frames))]
        for track in positions
    ]

    last_yaws = [0.0 for _ in range(vehicle_count)]
    for frame_index, frame in enumerate(frames):
        physical_robots = []
        load_share = payload_mass / max(1, vehicle_count) if cable_phase(frame) else 0.0
        for vehicle_index in range(vehicle_count):
            velocity = velocities[vehicle_index][frame_index]
            acceleration = accelerations[vehicle_index][frame_index]
            yaw = yaw_from_velocity(velocity, last_yaws[vehicle_index])
            last_yaws[vehicle_index] = yaw
            roll, pitch = attitude_from_acceleration(acceleration, yaw)
            effective_mass = vehicle_mass + load_share
            thrust_vector = (acceleration[0], acceleration[1], acceleration[2] + GRAVITY)
            thrust = effective_mass * norm(thrust_vector)
            hover_thrust = effective_mass * GRAVITY
            thrust_ratio = clamp(thrust / max(1e-6, hover_thrust), 0.45, 2.2)
            rotor_speed = clamp(hover_rotor_speed * math.sqrt(thrust_ratio), 32.0, 86.0)
            physical_robots.append(
                {
                    "velocity": round_point(velocity),
                    "acceleration": round_point(acceleration),
                    "yaw": round(yaw, 6),
                    "roll": round(roll, 6),
                    "pitch": round(pitch, 6),
                    "thrust": round(thrust, 6),
                    "rotor_speed": round(rotor_speed, 6),
                }
            )

        frame["physical"] = {
            "dt": round(visual_dt, 6),
            "vehicle_mass": vehicle_mass,
            "payload_mass": payload_mass,
            "robots": physical_robots,
        }


def cable_phase(frame: dict[str, Any]) -> bool:
    phase = str(frame.get("phase", ""))
    mode = str(frame.get("task_mode", ""))
    if phase in {"pickup_hold", "payload_hoist", "loaded_climb", "drop_descent", "drop_hold", "payload_release"}:
        return True
    return mode == "loaded"


def export_trajectory(
    *,
    case_file: Path,
    out_file: Path,
    map_scale: float,
    altitude: float,
    pickup_altitude: float,
    payload_drop: float,
    payload_ground_height: float,
    event_steps: int,
    start_hold_steps: int,
    frame_stride: int,
    visual_rate: float,
    vehicle_mass: float,
    payload_mass: float,
    hover_rotor_speed: float,
    acceleration_limit: float,
    control_mode: str | None,
) -> dict[str, Any]:
    scenario = PayloadTransportScenario(map_scale=map_scale, case_file=str(case_file), control_mode=control_mode)
    selected_indices = set(range(0, len(scenario.frames), max(1, frame_stride)))
    selected_indices.update(scenario.key_frame_indices)
    selected_indices.add(len(scenario.frames) - 1)

    replay_frames: list[dict[str, Any]] = []
    pick_xy = scenario.to_world(scenario.task["pick"]["center"])
    drop_xy = scenario.to_world(scenario.task["drop"]["center"])
    pick_ground = (pick_xy[0], pick_xy[1], payload_ground_height)
    drop_ground = (drop_xy[0], drop_xy[1], payload_ground_height)
    tracks: list[list[dict[str, Any]]] = [[], [], []]
    dt = max(1e-6, scenario.paper_qp_config.time_step)
    key_indices = set(scenario.key_frame_indices)
    for frame_index in sorted(selected_indices):
        frame = scenario.frames[frame_index]
        replay_frames.append(
            replay_frame_payload(
                scenario=scenario,
                frame_index=frame_index,
                altitude=altitude,
                payload_ground_height=payload_ground_height,
                payload_drop=payload_drop,
                pick_ground=pick_ground,
                drop_ground=drop_ground,
            )
        )
        force_keep = frame_index in key_indices or frame_index == len(scenario.frames) - 1
        for idx, robot in enumerate(frame.robots[:3]):
            x_world, y_world = scenario.to_world(robot)
            control = frame.controls[idx]
            position = (x_world, y_world, altitude)
            velocity = (
                control[0] * map_scale / dt,
                -control[1] * map_scale / dt,
                0.0,
            )
            if force_keep or not tracks[idx]:
                tracks[idx].append(waypoint_payload(position, velocity, frame_index))
                continue
            prev_position = tracks[idx][-1]["position"]
            dx = position[0] - prev_position[0]
            dy = position[1] - prev_position[1]
            dz = position[2] - prev_position[2]
            if (dx * dx + dy * dy + dz * dz) ** 0.5 > 0.03:
                tracks[idx].append(waypoint_payload(position, velocity, frame_index))

    failed_checks = [name for name, passed in scenario.checks.items() if not passed]
    replay_frames = expand_payload_events(
        replay_frames,
        cruise_altitude=altitude,
        pickup_altitude=pickup_altitude,
        payload_drop=payload_drop,
        payload_ground_height=payload_ground_height,
        event_steps=event_steps,
    )
    replay_frames = add_start_hold(replay_frames, start_hold_steps)
    visual_dt = 1.0 / max(1e-6, visual_rate)
    add_physical_metadata(
        replay_frames,
        visual_dt=visual_dt,
        vehicle_mass=vehicle_mass,
        payload_mass=payload_mass,
        hover_rotor_speed=hover_rotor_speed,
        acceleration_limit=acceleration_limit,
    )
    payload = {
        "schema": "gazebo_paper_qp_trajectory_v5",
        "case": str(case_file),
        "map_scale": map_scale,
        "altitude": altitude,
        "pickup_altitude": pickup_altitude,
        "payload_drop": payload_drop,
        "payload_ground_height": payload_ground_height,
        "event_steps": event_steps,
        "start_hold_steps": start_hold_steps,
        "visual_rate": visual_rate,
        "vehicle_mass": vehicle_mass,
        "payload_mass": payload_mass,
        "hover_rotor_speed": hover_rotor_speed,
        "acceleration_limit": acceleration_limit,
        "control_mode": scenario.control_mode,
        "frame_stride": max(1, frame_stride),
        "source_frame_count": len(scenario.frames),
        "key_frame_count": len(scenario.key_frame_indices),
        "vehicle_count": 3,
        "failed_checks": failed_checks,
        "replay_frames": replay_frames,
        "tracks": tracks,
    }
    out_file.parent.mkdir(parents=True, exist_ok=True)
    out_file.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    return payload


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--map-scale", type=float, default=0.01)
    parser.add_argument("--altitude", type=float, default=1.2)
    parser.add_argument("--pickup-altitude", type=float, default=0.62)
    parser.add_argument("--payload-drop", type=float, default=0.36)
    parser.add_argument("--payload-ground-height", type=float, default=0.12)
    parser.add_argument("--event-steps", type=int, default=18)
    parser.add_argument("--start-hold-steps", type=int, default=12)
    parser.add_argument("--frame-stride", type=int, default=8)
    parser.add_argument("--visual-rate", type=float, default=12.0)
    parser.add_argument("--vehicle-mass", type=float, default=1.6)
    parser.add_argument("--payload-mass", type=float, default=0.6)
    parser.add_argument("--hover-rotor-speed", type=float, default=52.0)
    parser.add_argument("--acceleration-limit", type=float, default=3.2)
    parser.add_argument("--control-mode", default=None)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    payload = export_trajectory(
        case_file=args.case,
        out_file=args.out,
        map_scale=args.map_scale,
        altitude=args.altitude,
        pickup_altitude=args.pickup_altitude,
        payload_drop=args.payload_drop,
        payload_ground_height=args.payload_ground_height,
        event_steps=args.event_steps,
        start_hold_steps=args.start_hold_steps,
        frame_stride=args.frame_stride,
        visual_rate=args.visual_rate,
        vehicle_mass=args.vehicle_mass,
        payload_mass=args.payload_mass,
        hover_rotor_speed=args.hover_rotor_speed,
        acceleration_limit=args.acceleration_limit,
        control_mode=args.control_mode,
    )
    track_lengths = [len(track) for track in payload["tracks"]]
    print(
        f"[OK] exported Gazebo trajectory: {args.out} "
        f"frames={payload['source_frame_count']} tracks={track_lengths} failed_checks={payload['failed_checks']}"
    )


if __name__ == "__main__":
    main()

"""Live Gazebo overlays for cable visuals and certified-region highlights."""

from __future__ import annotations

import json
import math
import re
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Int32


Point3 = tuple[float, float, float]
HIDDEN: Point3 = (-10.0, -10.0, -4.0)
HIDDEN_HIGHLIGHT: Point3 = (-1000.0, -1000.0, -1000.0)
VISIBLE_CURRENT: Point3 = (0.0, 0.0, 0.140)
VISIBLE_TARGET: Point3 = (0.0, 0.0, 0.170)
VISIBLE_BRIDGE: Point3 = (0.0, 0.0, 0.220)
PAYLOAD_HOOK_OFFSETS: tuple[Point3, ...] = (
    (0.13, 0.00, 0.11),
    (-0.07, 0.11, 0.11),
    (-0.07, -0.11, 0.11),
)
DRONE_ANCHOR_OFFSET: Point3 = (0.0, 0.0, -0.08)
ROPE_SEGMENTS = 8


@dataclass
class BodyState:
    position: Point3 | None = None
    orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)


def add(a: Point3, b: Point3) -> Point3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a: Point3, b: Point3) -> Point3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def scale(a: Point3, value: float) -> Point3:
    return (a[0] * value, a[1] * value, a[2] * value)


def norm(a: Point3) -> float:
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def cross(a: Point3, b: Point3) -> Point3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def rotate_vector(q: tuple[float, float, float, float], v: Point3) -> Point3:
    qx, qy, qz, qw = q
    u = (qx, qy, qz)
    uv = cross(u, v)
    uuv = cross(u, uv)
    return add(v, add(scale(uv, 2.0 * qw), scale(uuv, 2.0)))


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


def quaternion_from_z_axis(vector: Point3) -> tuple[float, float, float, float]:
    length = norm(vector)
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


def model_name(name: str) -> str:
    return "".join(ch if ch.isalnum() or ch == "_" else "_" for ch in name.lower())


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


def pose_message(name: str, position: Point3, yaw: float = 0.0) -> str:
    return pose_quaternion_message(name, position, quaternion_from_euler(0.0, 0.0, yaw))


def rope_segment_name(vehicle_index: int, segment_index: int) -> str:
    return model_name(f"payload_rope_r{vehicle_index + 1}_s{segment_index + 1}")


def region_names(region_text: str | None) -> list[str]:
    if not region_text:
        return []
    return re.findall(r"P\d+", region_text)


def next_distinct_state_id(frames: Sequence[dict[str, Any]], start_index: int) -> str | None:
    current = str(frames[start_index].get("state_id", ""))
    for frame in frames[start_index + 1 :]:
        state_id = str(frame.get("state_id", ""))
        if state_id and state_id != current:
            return state_id
    return None


def parse_lengths(raw: Sequence[float] | str, fallback: float, count: int) -> list[float]:
    values = [float(item.strip()) for item in raw.split(",") if item.strip()] if isinstance(raw, str) else [float(item) for item in raw]
    if not values:
        values = [fallback]
    while len(values) < count:
        values.append(values[-1])
    return values[:count]


class GazeboLiveOverlay(Node):
    """Move pre-generated Gazebo overlay models using live simulator state."""

    def __init__(self) -> None:
        super().__init__("gazebo_live_overlay")

        self.declare_parameter("case_file", "")
        self.declare_parameter("trajectory_file", "")
        self.declare_parameter("world", "payload_multicopter")
        self.declare_parameter("vehicle_count", 3)
        self.declare_parameter("vehicle_model_prefix", "x3_")
        self.declare_parameter("payload_model", "payload")
        self.declare_parameter("rope_rest_lengths", "0.92,0.96,0.99")
        self.declare_parameter("attach_slack", 0.12)
        self.declare_parameter("ground_height", 0.11)
        self.declare_parameter("rate_hz", 8.0)
        self.declare_parameter("timeout_ms", 500)
        self.declare_parameter("status_period", 5.0)

        self.case_file = Path(str(self.get_parameter("case_file").value))
        self.trajectory_file = Path(str(self.get_parameter("trajectory_file").value))
        self.world = str(self.get_parameter("world").value)
        self.vehicle_count = max(1, min(3, int(self.get_parameter("vehicle_count").value)))
        self.vehicle_model_prefix = str(self.get_parameter("vehicle_model_prefix").value)
        self.payload_model = str(self.get_parameter("payload_model").value)
        self.rest_lengths = parse_lengths(self.get_parameter("rope_rest_lengths").value, 0.95, self.vehicle_count)
        self.attach_slack = float(self.get_parameter("attach_slack").value)
        self.ground_height = float(self.get_parameter("ground_height").value)
        self.timeout_ms = int(self.get_parameter("timeout_ms").value)
        self.status_period = max(0.0, float(self.get_parameter("status_period").value))

        self.case = json.loads(self.case_file.read_text(encoding="utf-8")) if self.case_file.exists() else {}
        self.frames = self.load_frames(self.trajectory_file)
        self.states = {str(state["state_id"]): state for state in self.case.get("states", [])}
        self.transitions = {
            (str(transition["src"]), str(transition["dst"])): transition
            for transition in self.case.get("transitions", [])
        }
        self.bridges = self.bridge_lookup()
        self.target_index = 0
        self.last_highlight: tuple[str | None, str | None, str | None] = (None, None, None)
        self.ropes_visible = False
        self.last_status_at = 0.0

        self.drones = [BodyState() for _ in range(self.vehicle_count)]
        self.payload = BodyState()

        for idx in range(self.vehicle_count):
            self.create_subscription(
                Odometry,
                f"/model/{self.vehicle_model_prefix}{idx + 1}/odometry",
                lambda msg, vehicle_index=idx: self.on_drone_odom(vehicle_index, msg),
                10,
            )
        self.create_subscription(Odometry, f"/model/{self.payload_model}/odometry", self.on_payload_odom, 10)
        self.create_subscription(Int32, "/gazebo_tracker/target_index", self.on_target_index, 10)

        rate_hz = max(1.0, float(self.get_parameter("rate_hz").value))
        self.timer = self.create_timer(1.0 / rate_hz, self.on_timer)
        self.get_logger().info(
            f"Gazebo live overlay ready: frames={len(self.frames)} world={self.world} "
            f"rest_lengths={[round(v, 3) for v in self.rest_lengths]}"
        )

    @staticmethod
    def load_frames(trajectory_file: Path) -> list[dict[str, Any]]:
        if not trajectory_file.exists():
            return []
        payload = json.loads(trajectory_file.read_text(encoding="utf-8"))
        return list(payload.get("replay_frames") or [])

    def bridge_lookup(self) -> dict[tuple[frozenset[str], str], str]:
        bridges: dict[tuple[frozenset[str], str], str] = {}
        for bridge in self.case.get("bridges", []):
            names = frozenset(str(name) for name in bridge.get("regions", []))
            formation = str(bridge.get("formation", ""))
            bridges[(names, formation)] = str(bridge["name"])
        return bridges

    def on_drone_odom(self, vehicle_index: int, msg: Odometry) -> None:
        self.drones[vehicle_index] = self.state_from_odom(msg)

    def on_payload_odom(self, msg: Odometry) -> None:
        self.payload = self.state_from_odom(msg)

    def on_target_index(self, msg: Int32) -> None:
        self.target_index = max(0, int(msg.data))

    @staticmethod
    def state_from_odom(msg: Odometry) -> BodyState:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        return BodyState(
            position=(p.x, p.y, p.z),
            orientation=(q.x, q.y, q.z, q.w),
        )

    def on_timer(self) -> None:
        messages: list[str] = []
        messages.extend(self.rope_messages())
        messages.extend(self.highlight_messages())
        if not messages:
            return
        if not self.call_set_pose_vector(messages):
            self.log_once("Gazebo live overlay could not update set_pose_vector")

    def rope_messages(self) -> list[str]:
        if self.payload.position is None or any(drone.position is None for drone in self.drones):
            return []

        cable_lengths = [self.cable_geometry(idx)[2] for idx in range(self.vehicle_count)]
        close_enough = all(
            length < self.rest_lengths[idx] + self.attach_slack + 0.35
            for idx, length in enumerate(cable_lengths)
        )
        payload_lifted = self.payload.position[2] > self.ground_height + 0.05
        should_show = close_enough or payload_lifted

        if not should_show:
            if not self.ropes_visible:
                return []
            self.ropes_visible = False
            return self.hide_rope_messages()

        messages: list[str] = []
        for vehicle_index in range(self.vehicle_count):
            payload_hook, drone_anchor, length = self.cable_geometry(vehicle_index)
            if length < 1e-6:
                continue
            for segment_index in range(ROPE_SEGMENTS):
                start_alpha = segment_index / ROPE_SEGMENTS
                end_alpha = (segment_index + 1) / ROPE_SEGMENTS
                start = (
                    payload_hook[0] + (drone_anchor[0] - payload_hook[0]) * start_alpha,
                    payload_hook[1] + (drone_anchor[1] - payload_hook[1]) * start_alpha,
                    payload_hook[2] + (drone_anchor[2] - payload_hook[2]) * start_alpha,
                )
                end = (
                    payload_hook[0] + (drone_anchor[0] - payload_hook[0]) * end_alpha,
                    payload_hook[1] + (drone_anchor[1] - payload_hook[1]) * end_alpha,
                    payload_hook[2] + (drone_anchor[2] - payload_hook[2]) * end_alpha,
                )
                vector = sub(end, start)
                center = ((start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0, (start[2] + end[2]) / 2.0)
                messages.append(
                    pose_quaternion_message(
                        rope_segment_name(vehicle_index, segment_index),
                        center,
                        quaternion_from_z_axis(vector),
                    )
                )
        self.ropes_visible = True
        return messages

    @staticmethod
    def hide_rope_messages() -> list[str]:
        return [
            pose_message(rope_segment_name(vehicle_index, segment_index), HIDDEN)
            for vehicle_index in range(3)
            for segment_index in range(ROPE_SEGMENTS)
        ]

    def cable_geometry(self, vehicle_index: int) -> tuple[Point3, Point3, float]:
        payload_offset = rotate_vector(
            self.payload.orientation,
            PAYLOAD_HOOK_OFFSETS[vehicle_index % len(PAYLOAD_HOOK_OFFSETS)],
        )
        drone_offset = rotate_vector(self.drones[vehicle_index].orientation, DRONE_ANCHOR_OFFSET)
        payload_hook = add(self.payload.position or (0.0, 0.0, 0.0), payload_offset)
        drone_anchor = add(self.drones[vehicle_index].position or (0.0, 0.0, 0.0), drone_offset)
        return payload_hook, drone_anchor, norm(sub(drone_anchor, payload_hook))

    def highlight_messages(self) -> list[str]:
        if not self.frames:
            return []
        frame_index = min(self.target_index, len(self.frames) - 1)
        frame = self.frames[frame_index]
        next_state_id = next_distinct_state_id(self.frames, frame_index)
        current, target, bridge = self.highlight_selection(frame, next_state_id)
        last_current, last_target, last_bridge = self.last_highlight

        messages: list[str] = []
        if last_current and last_current != current:
            messages.append(pose_message(model_name(f"highlight_current_{last_current}"), HIDDEN_HIGHLIGHT))
        if last_target and last_target != target:
            messages.append(pose_message(model_name(f"highlight_target_{last_target}"), HIDDEN_HIGHLIGHT))
        if last_bridge and last_bridge != bridge:
            messages.append(pose_message(model_name(f"highlight_bridge_{last_bridge}"), HIDDEN_HIGHLIGHT))

        if current:
            messages.append(pose_message(model_name(f"highlight_current_{current}"), VISIBLE_CURRENT))
        if target:
            messages.append(pose_message(model_name(f"highlight_target_{target}"), VISIBLE_TARGET))
        if bridge:
            messages.append(pose_message(model_name(f"highlight_bridge_{bridge}"), VISIBLE_BRIDGE))

        self.last_highlight = (current, target, bridge)
        return messages

    def highlight_selection(self, frame: dict[str, Any], next_state_id: str | None) -> tuple[str | None, str | None, str | None]:
        state_id = str(frame.get("state_id", ""))
        state = self.states.get(state_id, {})
        formation = str(state.get("formation", frame.get("formation", "")))

        transition = self.transitions.get((state_id, next_state_id or ""))
        names = region_names(str(transition.get("certificate", ""))) if transition else []
        if not names:
            names = region_names(str(state.get("region", "")))

        current = names[0] if names else None
        target = names[-1] if len(names) > 1 else current
        bridge = None
        if len(names) >= 2:
            bridge = self.bridges.get((frozenset(names[:2]), formation))
        return current, target, bridge

    def call_set_pose_vector(self, messages: list[str]) -> bool:
        request = " ".join(messages)
        result = subprocess.run(
            [
                "ign",
                "service",
                "-s",
                f"/world/{self.world}/set_pose_vector",
                "--reqtype",
                "ignition.msgs.Pose_V",
                "--reptype",
                "ignition.msgs.Boolean",
                "--timeout",
                str(self.timeout_ms),
                "--req",
                request,
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        return result.returncode == 0

    def log_once(self, text: str) -> None:
        if self.status_period <= 0.0:
            return
        now = time.monotonic()
        if now - self.last_status_at >= self.status_period:
            self.last_status_at = now
            self.get_logger().warn(text)


def main() -> None:
    rclpy.init()
    node = GazeboLiveOverlay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

"""Apply physical cable-tension forces between Gazebo multicopters and payload."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity, EntityWrench


Point3 = tuple[float, float, float]

PAYLOAD_HOOK_OFFSETS: tuple[Point3, ...] = (
    (0.13, 0.00, 0.11),
    (-0.07, 0.11, 0.11),
    (-0.07, -0.11, 0.11),
)
DRONE_ANCHOR_OFFSET: Point3 = (0.0, 0.0, -0.08)


@dataclass
class BodyState:
    position: Point3 | None = None
    orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    linear_velocity: Point3 = (0.0, 0.0, 0.0)
    angular_velocity: Point3 = (0.0, 0.0, 0.0)


def add(a: Point3, b: Point3) -> Point3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a: Point3, b: Point3) -> Point3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def scale(a: Point3, value: float) -> Point3:
    return (a[0] * value, a[1] * value, a[2] * value)


def dot(a: Point3, b: Point3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a: Point3, b: Point3) -> Point3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def norm(a: Point3) -> float:
    return math.sqrt(dot(a, a))


def rotate_vector(q: tuple[float, float, float, float], v: Point3) -> Point3:
    qx, qy, qz, qw = q
    u = (qx, qy, qz)
    uv = cross(u, v)
    uuv = cross(u, uv)
    return add(v, add(scale(uv, 2.0 * qw), scale(uuv, 2.0)))


def point_from_json(point: dict[str, Any]) -> tuple[float, float]:
    return (float(point["x"]), float(point["y"]))


def to_world(point: tuple[float, float], height: float, scale_value: float) -> tuple[float, float]:
    return (point[0] * scale_value, (height - point[1]) * scale_value)


def parse_lengths(raw: Sequence[float] | str, fallback: float, count: int) -> list[float]:
    if isinstance(raw, str):
        values = [float(item.strip()) for item in raw.split(",") if item.strip()]
    else:
        values = [float(item) for item in raw]
    if not values:
        values = [fallback]
    while len(values) < count:
        values.append(values[-1])
    return values[:count]


class GazeboPayloadTether(Node):
    """Tension-only cable model implemented through Gazebo link wrenches."""

    def __init__(self) -> None:
        super().__init__("gazebo_payload_tether")

        self.declare_parameter("case_file", "")
        self.declare_parameter("map_scale", 0.02)
        self.declare_parameter("vehicle_count", 3)
        self.declare_parameter("wrench_topic", "/world/payload_multicopter/wrench")
        self.declare_parameter("payload_model", "payload")
        self.declare_parameter("payload_link", "payload_link")
        self.declare_parameter("vehicle_model_prefix", "x3_")
        self.declare_parameter("vehicle_link", "base_link")
        self.declare_parameter("rope_rest_lengths", "0.92,0.96,0.99")
        self.declare_parameter("rope_stiffness", 58.0)
        self.declare_parameter("rope_damping", 6.0)
        self.declare_parameter("max_tension", 16.0)
        self.declare_parameter("tension_ramp_seconds", 0.0)
        self.declare_parameter("apply_drone_reaction", False)
        self.declare_parameter("attach_slack", 0.10)
        self.declare_parameter("attach_max_drone_height", 0.90)
        self.declare_parameter("detach_distance", 1.75)
        self.declare_parameter("ground_height", 0.11)
        self.declare_parameter("drop_detach_radius", 0.85)
        self.declare_parameter("rate_hz", 80.0)
        self.declare_parameter("status_period", 2.0)

        self.case_file = str(self.get_parameter("case_file").value)
        self.map_scale = float(self.get_parameter("map_scale").value)
        self.vehicle_count = max(1, min(3, int(self.get_parameter("vehicle_count").value)))
        self.payload_model = str(self.get_parameter("payload_model").value)
        self.payload_link = str(self.get_parameter("payload_link").value)
        self.vehicle_model_prefix = str(self.get_parameter("vehicle_model_prefix").value)
        self.vehicle_link = str(self.get_parameter("vehicle_link").value)
        self.rest_lengths = parse_lengths(
            self.get_parameter("rope_rest_lengths").value,
            fallback=0.95,
            count=self.vehicle_count,
        )
        self.k = float(self.get_parameter("rope_stiffness").value)
        self.c = float(self.get_parameter("rope_damping").value)
        self.max_tension = float(self.get_parameter("max_tension").value)
        self.tension_ramp_seconds = max(0.0, float(self.get_parameter("tension_ramp_seconds").value))
        self.apply_drone_reaction = bool(self.get_parameter("apply_drone_reaction").value)
        self.attach_slack = float(self.get_parameter("attach_slack").value)
        self.attach_max_drone_height = float(self.get_parameter("attach_max_drone_height").value)
        self.detach_distance = float(self.get_parameter("detach_distance").value)
        self.ground_height = float(self.get_parameter("ground_height").value)
        self.drop_detach_radius = float(self.get_parameter("drop_detach_radius").value)
        self.status_period = max(0.0, float(self.get_parameter("status_period").value))

        self.drop_xy = self.load_drop_point()
        self.attached = False
        self.attached_at: float | None = None
        self.drop_released = False
        self.last_status_at = 0.0
        self.drones = [BodyState() for _ in range(self.vehicle_count)]
        self.payload = BodyState()

        self.wrench_pub = self.create_publisher(
            EntityWrench,
            str(self.get_parameter("wrench_topic").value),
            30,
        )
        for idx in range(self.vehicle_count):
            self.create_subscription(
                Odometry,
                f"/model/{self.vehicle_model_prefix}{idx + 1}/odometry",
                lambda msg, vehicle_index=idx: self.on_drone_odom(vehicle_index, msg),
                10,
            )
        self.create_subscription(Odometry, f"/model/{self.payload_model}/odometry", self.on_payload_odom, 10)

        rate_hz = max(1.0, float(self.get_parameter("rate_hz").value))
        self.timer = self.create_timer(1.0 / rate_hz, self.on_timer)
        self.get_logger().info(
            "Payload tether ready: "
            f"rest_lengths={[round(v, 3) for v in self.rest_lengths]} "
            f"k={self.k:.1f} c={self.c:.1f} max_tension={self.max_tension:.1f} "
            f"drone_reaction={self.apply_drone_reaction}"
        )

    def load_drop_point(self) -> tuple[float, float] | None:
        if not self.case_file:
            return None
        case_path = Path(self.case_file)
        if not case_path.exists():
            return None
        import json

        case = json.loads(case_path.read_text(encoding="utf-8"))
        drop = case.get("task", {}).get("regions", {}).get("drop")
        if not drop:
            return None
        return to_world(point_from_json(drop["center"]), float(case["height"]), self.map_scale)

    def on_drone_odom(self, vehicle_index: int, msg: Odometry) -> None:
        self.drones[vehicle_index] = self.state_from_odom(msg)

    def on_payload_odom(self, msg: Odometry) -> None:
        self.payload = self.state_from_odom(msg)

    @staticmethod
    def state_from_odom(msg: Odometry) -> BodyState:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular
        return BodyState(
            position=(p.x, p.y, p.z),
            orientation=(q.x, q.y, q.z, q.w),
            linear_velocity=(linear.x, linear.y, linear.z),
            angular_velocity=(angular.x, angular.y, angular.z),
        )

    def on_timer(self) -> None:
        if self.payload.position is None or any(drone.position is None for drone in self.drones):
            return

        cable_lengths = [self.cable_geometry(idx)[2] for idx in range(self.vehicle_count)]
        drones_low_enough = all(
            drone.position is not None and drone.position[2] <= self.attach_max_drone_height
            for drone in self.drones[: self.vehicle_count]
        )
        if (
            not self.attached
            and not self.drop_released
            and drones_low_enough
            and all(length < self.rest_lengths[idx] + self.attach_slack for idx, length in enumerate(cable_lengths))
        ):
            self.attached = True
            self.attached_at = time.monotonic()
            self.get_logger().info(
                "Payload tether attached by cable proximity: "
                f"lengths={[round(value, 3) for value in cable_lengths]}"
            )

        if self.attached and self.should_detach(cable_lengths):
            self.attached = False
            self.attached_at = None
            self.drop_released = True
            self.get_logger().info("Payload tether detached at drop zone")

        if not self.attached:
            self.log_status(cable_lengths)
            return

        payload_force = (0.0, 0.0, 0.0)
        payload_torque = (0.0, 0.0, 0.0)
        drone_forces: list[tuple[Point3, Point3]] = []
        tensions: list[float] = []

        for idx in range(self.vehicle_count):
            payload_hook, drone_anchor, length, direction, hook_offset, anchor_offset = self.cable_geometry(idx)
            if length < 1e-6:
                drone_forces.append(((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)))
                continue

            drone = self.drones[idx]
            hook_velocity = add(self.payload.linear_velocity, cross(self.payload.angular_velocity, hook_offset))
            anchor_velocity = add(drone.linear_velocity, cross(drone.angular_velocity, anchor_offset))
            relative_speed = dot(sub(anchor_velocity, hook_velocity), direction)
            extension = length - self.rest_lengths[idx]
            tension = max(0.0, min(self.max_tension, self.k * extension + self.c * relative_speed))
            if self.tension_ramp_seconds > 0.0 and self.attached_at is not None:
                ramp = min(1.0, max(0.0, (time.monotonic() - self.attached_at) / self.tension_ramp_seconds))
                tension *= ramp
            tensions.append(tension)

            force_on_payload = scale(direction, tension)
            payload_force = add(payload_force, force_on_payload)
            payload_torque = add(payload_torque, cross(hook_offset, force_on_payload))

            force_on_drone = scale(force_on_payload, -1.0)
            torque_on_drone = cross(anchor_offset, force_on_drone)
            drone_forces.append((force_on_drone, torque_on_drone))

        self.publish_wrench(f"{self.payload_model}::{self.payload_link}", payload_force, payload_torque)
        if self.apply_drone_reaction:
            for idx, (force, torque) in enumerate(drone_forces, start=1):
                self.publish_wrench(f"{self.vehicle_model_prefix}{idx}::{self.vehicle_link}", force, torque)
        self.log_status(cable_lengths, tensions)

    def log_status(self, cable_lengths: Sequence[float], tensions: Sequence[float] | None = None) -> None:
        if self.status_period <= 0.0:
            return
        now = time.monotonic()
        if now - self.last_status_at < self.status_period:
            return
        self.last_status_at = now

        thresholds = [self.rest_lengths[idx] + self.attach_slack for idx in range(self.vehicle_count)]
        payload_z = self.payload.position[2] if self.payload.position is not None else float("nan")
        if tensions is None:
            self.get_logger().info(
                "Payload tether waiting: "
                f"payload_z={payload_z:.2f} "
                f"lengths={[round(value, 3) for value in cable_lengths]} "
                f"attach_thresholds={[round(value, 3) for value in thresholds]}"
            )
            return

        self.get_logger().info(
            "Payload tether active: "
            f"payload_z={payload_z:.2f} "
            f"lengths={[round(value, 3) for value in cable_lengths]} "
            f"tensions={[round(value, 2) for value in tensions]}"
        )

    def cable_geometry(self, vehicle_index: int) -> tuple[Point3, Point3, float, Point3, Point3, Point3]:
        payload_offset = rotate_vector(
            self.payload.orientation,
            PAYLOAD_HOOK_OFFSETS[vehicle_index % len(PAYLOAD_HOOK_OFFSETS)],
        )
        drone_offset = rotate_vector(self.drones[vehicle_index].orientation, DRONE_ANCHOR_OFFSET)
        payload_hook = add(self.payload.position or (0.0, 0.0, 0.0), payload_offset)
        drone_anchor = add(self.drones[vehicle_index].position or (0.0, 0.0, 0.0), drone_offset)
        cable = sub(drone_anchor, payload_hook)
        length = norm(cable)
        direction = scale(cable, 1.0 / length) if length > 1e-9 else (0.0, 0.0, 1.0)
        return payload_hook, drone_anchor, length, direction, payload_offset, drone_offset

    def should_detach(self, cable_lengths: Sequence[float]) -> bool:
        if self.payload.position is None:
            return False
        if self.payload.position[2] > self.ground_height + 0.08:
            return False
        if self.drop_xy is not None:
            dist_drop = math.hypot(self.payload.position[0] - self.drop_xy[0], self.payload.position[1] - self.drop_xy[1])
            if dist_drop > self.drop_detach_radius:
                return False
        return True

    def publish_wrench(self, entity_name: str, force: Point3, torque: Point3) -> None:
        msg = EntityWrench()
        msg.entity.name = entity_name
        msg.entity.type = Entity.LINK
        msg.wrench.force.x = force[0]
        msg.wrench.force.y = force[1]
        msg.wrench.force.z = force[2]
        msg.wrench.torque.x = torque[0]
        msg.wrench.torque.y = torque[1]
        msg.wrench.torque.z = torque[2]
        self.wrench_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = GazeboPayloadTether()
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

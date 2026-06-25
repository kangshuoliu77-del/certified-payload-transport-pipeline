"""ROS 2 outer-loop velocity tracker for the Gazebo X3 multicopter demo."""

from __future__ import annotations

import json
import math
from pathlib import Path
from dataclasses import dataclass
import time
from typing import Any, List, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Int32


Point3 = Tuple[float, float, float]


@dataclass(frozen=True)
class Waypoint:
    position: Point3
    velocity: Point3 = (0.0, 0.0, 0.0)


@dataclass
class VehicleState:
    pose: Point3 | None = None
    yaw: float = 0.0
    target_index: int = 0


def point_from_json(point: dict[str, Any]) -> tuple[float, float]:
    return (float(point["x"]), float(point["y"]))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class GazeboVelocityTracker(Node):
    """Track case waypoints by commanding Gazebo's MulticopterVelocityControl."""

    def __init__(self) -> None:
        super().__init__("gazebo_velocity_tracker")

        self.declare_parameter("case_file", "")
        self.declare_parameter("trajectory_file", "")
        self.declare_parameter("map_scale", 0.01)
        self.declare_parameter("target_altitude", 1.2)
        self.declare_parameter("vehicle_count", 3)
        self.declare_parameter("trajectory_mode", "robots")
        self.declare_parameter("control_mode", "")
        self.declare_parameter("command_frame", "world")
        self.declare_parameter("frame_stride", 8)
        self.declare_parameter("use_feedforward", True)
        self.declare_parameter("feedforward_activation_radius", 0.55)
        self.declare_parameter("synchronize_vehicles", False)
        self.declare_parameter("startup_hold_seconds", 0.0)
        self.declare_parameter("max_waypoint_advance_per_tick", 40)
        self.declare_parameter("progress_search_window", 80)
        self.declare_parameter("lookahead_waypoints", 6)
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("kp_xy", 0.85)
        self.declare_parameter("kp_z", 0.9)
        self.declare_parameter("kp_yaw", 1.8)
        self.declare_parameter("max_xy_speed", 1.0)
        self.declare_parameter("max_z_speed", 0.45)
        self.declare_parameter("max_yaw_rate", 1.2)
        self.declare_parameter("waypoint_tolerance", 0.28)
        self.declare_parameter("status_period", 2.0)

        self.case_file = str(self.get_parameter("case_file").value)
        self.trajectory_file = str(self.get_parameter("trajectory_file").value)
        self.map_scale = float(self.get_parameter("map_scale").value)
        self.target_altitude = float(self.get_parameter("target_altitude").value)
        self.kp_xy = float(self.get_parameter("kp_xy").value)
        self.kp_z = float(self.get_parameter("kp_z").value)
        self.kp_yaw = float(self.get_parameter("kp_yaw").value)
        self.max_xy_speed = float(self.get_parameter("max_xy_speed").value)
        self.max_z_speed = float(self.get_parameter("max_z_speed").value)
        self.max_yaw_rate = float(self.get_parameter("max_yaw_rate").value)
        self.waypoint_tolerance = float(self.get_parameter("waypoint_tolerance").value)
        self.status_period = max(0.0, float(self.get_parameter("status_period").value))

        self.vehicle_count = max(1, min(3, int(self.get_parameter("vehicle_count").value)))
        self.trajectory_mode = str(self.get_parameter("trajectory_mode").value)
        self.control_mode = str(self.get_parameter("control_mode").value).strip()
        self.command_frame = str(self.get_parameter("command_frame").value).strip().lower()
        self.frame_stride = max(1, int(self.get_parameter("frame_stride").value))
        self.use_feedforward = bool(self.get_parameter("use_feedforward").value)
        self.feedforward_activation_radius = max(0.0, float(self.get_parameter("feedforward_activation_radius").value))
        self.synchronize_vehicles = bool(self.get_parameter("synchronize_vehicles").value)
        self.startup_hold_seconds = max(0.0, float(self.get_parameter("startup_hold_seconds").value))
        self.max_waypoint_advance_per_tick = max(1, int(self.get_parameter("max_waypoint_advance_per_tick").value))
        self.progress_search_window = max(1, int(self.get_parameter("progress_search_window").value))
        self.lookahead_waypoints = max(0, int(self.get_parameter("lookahead_waypoints").value))
        rate_hz = max(1.0, float(self.get_parameter("rate_hz").value))
        self.started_at = time.monotonic()
        self.global_target_index = 0
        self.last_status_at = 0.0
        self.waypoint_phases: list[str] = []

        self.get_logger().info(
            f"Loading Gazebo tracker trajectory from {self.case_file}; mode={self.trajectory_mode}"
        )
        self.waypoints = self.load_waypoints(Path(self.case_file))
        if not self.waypoints or any(not track for track in self.waypoints):
            raise RuntimeError("No waypoints available for Gazebo velocity tracker")

        self.vehicles = [VehicleState() for _ in range(self.vehicle_count)]
        self.last_commands: list[Point3] = [(0.0, 0.0, 0.0) for _ in range(self.vehicle_count)]
        self.cmd_pubs = []
        self.enable_pubs = []
        self.odom_subs = []
        self.target_index_pub = self.create_publisher(Int32, "/gazebo_tracker/target_index", 10)
        for idx in range(self.vehicle_count):
            vehicle_id = idx + 1
            cmd_topic = f"/X3_{vehicle_id}/gazebo/command/twist"
            enable_topic = f"/X3_{vehicle_id}/enable"
            odom_topic = f"/model/x3_{vehicle_id}/odometry"
            self.cmd_pubs.append(self.create_publisher(Twist, cmd_topic, 10))
            self.enable_pubs.append(self.create_publisher(Bool, enable_topic, 10))
            self.odom_subs.append(
                self.create_subscription(
                    Odometry,
                    odom_topic,
                    lambda msg, vehicle_index=idx: self.on_odom(vehicle_index, msg),
                    10,
                )
            )
        self.timer = self.create_timer(1.0 / rate_hz, self.on_timer)

        self.get_logger().info(
            f"Tracking {len(self.waypoints[0])} Gazebo waypoints per vehicle from {self.case_file}; "
            f"vehicles={self.vehicle_count} mode={self.trajectory_mode} "
            f"feedforward={self.use_feedforward} sync={self.synchronize_vehicles} "
            f"command_frame={self.command_frame}"
        )

    def load_waypoints(self, case_file: Path) -> List[List[Waypoint]]:
        if self.trajectory_mode in {"file", "cached", "trajectory_file"}:
            return self.load_cached_waypoints(Path(self.trajectory_file))
        if self.trajectory_mode in {"frames", "paper_qp", "paper_qp_frames", "execution_frames"}:
            return self.load_execution_frame_waypoints(case_file)
        return self.load_state_waypoints(case_file)

    def load_cached_waypoints(self, trajectory_file: Path) -> List[List[Waypoint]]:
        if not trajectory_file.exists():
            raise FileNotFoundError(f"Gazebo trajectory cache not found: {trajectory_file}")
        payload = json.loads(trajectory_file.read_text(encoding="utf-8"))
        self.waypoint_phases = [
            str(frame.get("phase", "flight"))
            for frame in payload.get("replay_frames", [])
        ]
        tracks: List[List[Waypoint]] = []
        for raw_track in payload.get("tracks", [])[: self.vehicle_count]:
            track = []
            for item in raw_track:
                position = tuple(float(value) for value in item["position"][:3])
                if self.use_feedforward:
                    velocity = tuple(float(value) for value in item.get("velocity", [0.0, 0.0, 0.0])[:3])
                else:
                    velocity = (0.0, 0.0, 0.0)
                track.append(Waypoint(position, velocity))  # type: ignore[arg-type]
            tracks.append(track)
        while len(tracks) < self.vehicle_count:
            tracks.append([])
        if self.waypoint_phases and len(self.waypoint_phases) != len(tracks[0]):
            self.get_logger().warn(
                "Cached trajectory phase count does not match track length; "
                "phase-aware target gating is disabled"
            )
            self.waypoint_phases = []
        self.get_logger().info(f"Loaded cached Gazebo trajectory: {trajectory_file}")
        return tracks

    def load_state_waypoints(self, case_file: Path) -> List[List[Waypoint]]:
        case = json.loads(case_file.read_text(encoding="utf-8"))
        height = float(case["height"])
        tracks: List[List[Waypoint]] = [[] for _ in range(self.vehicle_count)]

        for state in case.get("states", []):
            placement = state.get("placement") or {}
            if self.trajectory_mode == "robots" and placement.get("robots"):
                raw_points = placement["robots"][: self.vehicle_count]
            else:
                raw_points = [placement.get("center")] * self.vehicle_count
            if len(raw_points) < self.vehicle_count or any(point is None for point in raw_points):
                continue
            for idx, raw_point in enumerate(raw_points[: self.vehicle_count]):
                x_img, y_img = point_from_json(raw_point)
                point = (x_img * self.map_scale, (height - y_img) * self.map_scale, self.target_altitude)
                waypoint = Waypoint(point)
                if not tracks[idx] or math.dist(tracks[idx][-1].position, point) > 0.20:
                    tracks[idx].append(waypoint)

        return tracks

    def load_execution_frame_waypoints(self, case_file: Path) -> List[List[Waypoint]]:
        from .random_payload_model import PayloadTransportScenario

        control_mode = self.control_mode or None
        scenario = PayloadTransportScenario(map_scale=self.map_scale, case_file=str(case_file), control_mode=control_mode)
        failed_checks = [name for name, passed in scenario.checks.items() if not passed]
        if failed_checks:
            self.get_logger().warn(f"Scenario checks failed while loading Gazebo frames: {failed_checks}")

        tracks: List[List[Waypoint]] = [[] for _ in range(self.vehicle_count)]
        selected_indices = set(range(0, len(scenario.frames), self.frame_stride))
        selected_indices.update(scenario.key_frame_indices)
        selected_indices.add(len(scenario.frames) - 1)

        dt = max(1e-6, scenario.paper_qp_config.time_step)
        key_indices = set(scenario.key_frame_indices)
        for frame_index in sorted(selected_indices):
            frame = scenario.frames[frame_index]
            force_keep = frame_index in key_indices or frame_index == len(scenario.frames) - 1
            for idx, robot in enumerate(frame.robots[: self.vehicle_count]):
                x_world, y_world = scenario.to_world(robot)
                velocity = (0.0, 0.0, 0.0)
                if self.use_feedforward and idx < len(frame.controls):
                    control = frame.controls[idx]
                    velocity = (
                        control[0] * self.map_scale / dt,
                        -control[1] * self.map_scale / dt,
                        0.0,
                    )
                point = (x_world, y_world, self.target_altitude)
                if force_keep or not tracks[idx] or math.dist(tracks[idx][-1].position, point) > 0.03:
                    tracks[idx].append(Waypoint(point, velocity))

        self.get_logger().info(
            f"Loaded {len(scenario.frames)} scenario frames; using every {self.frame_stride} frame "
            f"plus {len(scenario.key_frame_indices)} key frames"
        )

        return tracks

    def on_odom(self, vehicle_index: int, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.vehicles[vehicle_index].pose = (p.x, p.y, p.z)
        self.vehicles[vehicle_index].yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def on_timer(self) -> None:
        enable_msg = Bool()
        enable_msg.data = True
        for pub in self.enable_pubs:
            pub.publish(enable_msg)

        if self.synchronize_vehicles and time.monotonic() - self.started_at >= self.startup_hold_seconds:
            self.update_global_target()

        for idx, vehicle in enumerate(self.vehicles):
            if vehicle.pose is None:
                continue
            cmd = self.command_for_vehicle(idx, vehicle)
            self.last_commands[idx] = (cmd.linear.x, cmd.linear.y, cmd.linear.z)
            self.cmd_pubs[idx].publish(cmd)
        self.publish_target_index()
        self.log_status()

    def update_global_target(self) -> None:
        if any(vehicle.pose is None for vehicle in self.vehicles):
            return

        advanced = 0
        max_index = min(len(track) for track in self.waypoints[: self.vehicle_count]) - 1

        # In a real Gazebo flight loop the vehicles will not hit every exported
        # paper-QP sample exactly. Track progress using the formation centroid
        # and command a small lookahead point, instead of waiting for all three
        # vehicles to enter the same tight waypoint ball at the same instant.
        actual_center = self.vehicle_centroid()
        if actual_center is None:
            return

        current_phase = self.target_phase(self.global_target_index)
        current_errors = self.individual_target_errors(self.global_target_index)
        max_xy_error = max(error[0] for error in current_errors)
        max_z_error = max(abs(error[1]) for error in current_errors)
        if self.is_cable_phase(current_phase):
            z_tolerance = 0.62 if current_phase in {"pickup_hold", "payload_hoist", "loaded_climb"} else 0.35
            if max_xy_error > max(0.35, 1.15 * self.waypoint_tolerance) or max_z_error > z_tolerance:
                return
        elif max_xy_error > max(0.95, 2.2 * self.waypoint_tolerance) or max_z_error > 0.55:
            return

        phase_search_window = 1 if self.is_cable_phase(current_phase) else self.progress_search_window
        phase_lookahead = 0 if self.is_cable_phase(current_phase) else self.lookahead_waypoints
        search_end = min(max_index, self.global_target_index + phase_search_window)
        best_index = self.global_target_index
        best_score = float("inf")
        for candidate in range(self.global_target_index, search_end + 1):
            target_center = self.target_centroid(candidate)
            dx = target_center[0] - actual_center[0]
            dy = target_center[1] - actual_center[1]
            dz = target_center[2] - actual_center[2]
            score = math.hypot(dx, dy) + 0.35 * abs(dz)
            if score < best_score:
                best_score = score
                best_index = candidate

        desired_index = min(max_index, max(self.global_target_index + 1, best_index + phase_lookahead))
        if desired_index > self.global_target_index:
            advanced = min(desired_index - self.global_target_index, self.max_waypoint_advance_per_tick)
            self.global_target_index += advanced

        if advanced:
            for vehicle in self.vehicles:
                vehicle.target_index = self.global_target_index
            self.get_logger().info(f"Gazebo synchronized target {self.global_target_index + 1}/{max_index + 1}")

    def vehicle_centroid(self) -> Point3 | None:
        poses = [vehicle.pose for vehicle in self.vehicles[: self.vehicle_count] if vehicle.pose is not None]
        if len(poses) != self.vehicle_count:
            return None
        return (
            sum(pose[0] for pose in poses) / self.vehicle_count,
            sum(pose[1] for pose in poses) / self.vehicle_count,
            sum(pose[2] for pose in poses) / self.vehicle_count,
        )

    def target_centroid(self, target_index: int) -> Point3:
        points = [
            self.waypoints[idx][min(target_index, len(self.waypoints[idx]) - 1)].position
            for idx in range(self.vehicle_count)
        ]
        return (
            sum(point[0] for point in points) / self.vehicle_count,
            sum(point[1] for point in points) / self.vehicle_count,
            sum(point[2] for point in points) / self.vehicle_count,
        )

    def target_phase(self, target_index: int) -> str:
        if not self.waypoint_phases:
            return "flight"
        return self.waypoint_phases[min(target_index, len(self.waypoint_phases) - 1)]

    @staticmethod
    def is_cable_phase(phase: str) -> bool:
        return phase in {
            "pickup_descent",
            "pickup_hold",
            "payload_hoist",
            "loaded_climb",
            "drop_descent",
            "drop_hold",
            "payload_release",
            "empty_climb",
        }

    def individual_target_errors(self, target_index: int) -> list[tuple[float, float]]:
        errors = []
        for idx, vehicle in enumerate(self.vehicles[: self.vehicle_count]):
            if vehicle.pose is None:
                errors.append((float("inf"), float("inf")))
                continue
            target = self.waypoints[idx][min(target_index, len(self.waypoints[idx]) - 1)].position
            x, y, z = vehicle.pose
            errors.append((math.hypot(target[0] - x, target[1] - y), target[2] - z))
        return errors

    def vehicle_reached_target(self, vehicle_index: int, target_index: int) -> bool:
        vehicle = self.vehicles[vehicle_index]
        if vehicle.pose is None:
            return False
        track = self.waypoints[vehicle_index]
        target = track[min(target_index, len(track) - 1)].position
        x, y, z = vehicle.pose
        return math.hypot(target[0] - x, target[1] - y) < self.waypoint_tolerance and abs(target[2] - z) < 0.22

    def log_status(self) -> None:
        if self.status_period <= 0.0:
            return
        now = time.monotonic()
        if now - self.last_status_at < self.status_period:
            return
        self.last_status_at = now

        target_index = self.global_target_index if self.synchronize_vehicles else self.vehicles[0].target_index
        errors = []
        commands = []
        for idx, vehicle in enumerate(self.vehicles):
            if vehicle.pose is None:
                errors.append(None)
                commands.append(None)
                continue
            track = self.waypoints[idx]
            target = track[min(target_index, len(track) - 1)].position
            x, y, z = vehicle.pose
            errors.append((round(math.hypot(target[0] - x, target[1] - y), 3), round(target[2] - z, 3)))
            commands.append(tuple(round(value, 3) for value in self.last_commands[idx]))
        self.get_logger().info(
            f"Gazebo tracker status target={target_index + 1}/{len(self.waypoints[0])} "
            f"errors_xy_z={errors} cmd_body_xyz={commands}"
        )

    def publish_target_index(self) -> None:
        msg = Int32()
        if self.synchronize_vehicles:
            msg.data = int(self.global_target_index)
        else:
            msg.data = int(max(vehicle.target_index for vehicle in self.vehicles))
        self.target_index_pub.publish(msg)

    def command_for_vehicle(self, vehicle_index: int, vehicle: VehicleState) -> Twist:
        track = self.waypoints[vehicle_index]
        target_index = min(self.global_target_index if self.synchronize_vehicles else vehicle.target_index, len(track) - 1)
        vehicle.target_index = target_index
        phase = self.target_phase(target_index)
        target_waypoint = track[target_index]
        target = target_waypoint.position
        x, y, z = vehicle.pose or (0.0, 0.0, 0.0)

        if time.monotonic() - self.started_at < self.startup_hold_seconds:
            cmd = Twist()
            cmd.linear.z = clamp(self.kp_z * (target[2] - z), self.max_z_speed)
            return cmd

        ex = target[0] - x
        ey = target[1] - y
        ez = target[2] - z
        dist_xy = math.hypot(ex, ey)

        if not self.synchronize_vehicles:
            advanced = 0
            while (
                dist_xy < self.waypoint_tolerance
                and abs(ez) < 0.22
                and vehicle.target_index < len(track) - 1
                and advanced < self.max_waypoint_advance_per_tick
            ):
                vehicle.target_index += 1
                target_waypoint = track[vehicle.target_index]
                target = target_waypoint.position
                ex = target[0] - x
                ey = target[1] - y
                ez = target[2] - z
                dist_xy = math.hypot(ex, ey)
                advanced += 1
            if advanced and vehicle_index == 0:
                self.get_logger().info(f"Gazebo target {vehicle.target_index + 1}/{len(track)}")

        cmd = Twist()

        if dist_xy > 1e-3:
            target_yaw = math.atan2(ey, ex)
            yaw_error = wrap_angle(target_yaw - vehicle.yaw)
        else:
            yaw_error = 0.0

        # Feedforward is useful only once the vehicle is close to the current
        # moving reference. If the vehicle is far behind, the target velocity can
        # point away from the current waypoint and cancel the stabilizing error
        # feedback. In that case, prioritize convergence to the waypoint.
        use_feedforward_now = self.use_feedforward and dist_xy <= max(
            self.feedforward_activation_radius,
            1.8 * self.waypoint_tolerance,
        )
        ff_vx = target_waypoint.velocity[0] if use_feedforward_now else 0.0
        ff_vy = target_waypoint.velocity[1] if use_feedforward_now else 0.0

        world_vx = self.kp_xy * ex + ff_vx
        world_vy = self.kp_xy * ey + ff_vy
        max_xy_speed = self.max_xy_speed
        max_z_speed = self.max_z_speed
        if self.is_cable_phase(phase):
            max_xy_speed = min(max_xy_speed, 0.65)

        speed = math.hypot(world_vx, world_vy)
        if speed > max_xy_speed:
            world_vx *= max_xy_speed / speed
            world_vy *= max_xy_speed / speed

        if self.command_frame == "body":
            c = math.cos(vehicle.yaw)
            s = math.sin(vehicle.yaw)
            cmd.linear.x = c * world_vx + s * world_vy
            cmd.linear.y = -s * world_vx + c * world_vy
        else:
            cmd.linear.x = world_vx
            cmd.linear.y = world_vy
        cmd.linear.z = clamp(self.kp_z * ez, max_z_speed)
        cmd.angular.z = clamp(self.kp_yaw * yaw_error, self.max_yaw_rate)

        if vehicle.target_index == len(track) - 1 and dist_xy < self.waypoint_tolerance:
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0

        return cmd


def main() -> None:
    rclpy.init()
    node = GazeboVelocityTracker()
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

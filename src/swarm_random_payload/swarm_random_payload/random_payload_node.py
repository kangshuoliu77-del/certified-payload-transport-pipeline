"""ROS 2 node that visualizes the certified payload-transport workflow."""

from __future__ import annotations

import math
from typing import List, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped, TransformStamped
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from .random_payload_model import (
    FORMATION_LAYERS,
    PayloadTransportScenario,
    Point2,
    signed_distance_to_polygons,
    workspace_clearance,
)


Color = Tuple[float, float, float, float]

FREE_SPACE: Color = (0.82, 0.88, 0.95, 0.12)
REGION_FILL: Color = (0.18, 0.75, 0.44, 0.24)
REGION_OUTLINE: Color = (0.04, 0.45, 0.25, 0.90)
OBSTACLE_COLOR: Color = (0.20, 0.30, 0.44, 0.98)
OBSTACLE_TOP_COLOR: Color = (0.34, 0.48, 0.66, 0.98)
OBSTACLE_SIDE_COLORS: List[Color] = [
    (0.18, 0.23, 0.31, 0.94),
    (0.13, 0.17, 0.24, 0.94),
    (0.22, 0.27, 0.35, 0.94),
    (0.16, 0.20, 0.28, 0.94),
]
OBSTACLE_EDGE_COLOR: Color = (0.96, 0.58, 0.20, 0.96)
SAFETY_BOUNDARY_COLOR: Color = REGION_OUTLINE
LOADED: Color = (0.15, 0.39, 0.92, 0.92)
DELIVERED: Color = (0.49, 0.23, 0.93, 0.92)
HOME_COLOR: Color = (0.06, 0.46, 0.43, 0.35)
PICK_COLOR: Color = (0.98, 0.45, 0.10, 0.35)
DROP_COLOR: Color = (0.75, 0.09, 0.36, 0.35)
CURRENT_REGION_OUTLINE: Color = (0.08, 0.22, 1.0, 1.0)
OTHER_BRIDGE_REGION_OUTLINE: Color = (0.0, 0.78, 0.95, 1.0)
BRIDGE_OUTLINE: Color = (1.0, 0.88, 0.0, 1.0)
AGENT_COLORS: List[Color] = [(0.92, 0.12, 0.12, 1.0), (0.0, 0.78, 0.90, 1.0), (0.10, 0.35, 0.95, 1.0)]
DRONE_BODY_MESH = "package://swarm_random_payload/meshes/NXP-HGD-CF.dae"
DRONE_MOTOR_BASE_MESH = "package://swarm_random_payload/meshes/5010Base.dae"
DRONE_MOTOR_BELL_MESH = "package://swarm_random_payload/meshes/5010Bell.dae"
ROTOR_OFFSETS: List[Point2] = [(0.174, -0.174), (-0.174, 0.174), (0.174, 0.174), (-0.174, -0.174)]


def make_point(x: float, y: float, z: float = 0.0) -> Point:
    point = Point()
    point.x = float(x)
    point.y = float(y)
    point.z = float(z)
    return point


def make_pose(x: float, y: float, z: float = 0.0, yaw: float = 0.0) -> Pose:
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = float(z)
    pose.orientation.z = math.sin(0.5 * yaw)
    pose.orientation.w = math.cos(0.5 * yaw)
    return pose


def rotate_offset(origin: Point2, offset: Point2, yaw: float) -> Point2:
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        origin[0] + offset[0] * cos_yaw - offset[1] * sin_yaw,
        origin[1] + offset[0] * sin_yaw + offset[1] * cos_yaw,
    )


def centroid(points: Sequence[Point2]) -> Point2:
    return (sum(p[0] for p in points) / len(points), sum(p[1] for p in points) / len(points))


class MarkerBuilder:
    """Small helper that keeps marker IDs stable within one publish cycle."""

    def __init__(self, frame_id: str, stamp) -> None:
        self.frame_id = frame_id
        self.stamp = stamp
        self.markers = MarkerArray()
        self.next_id = 0

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.action = Marker.DELETEALL
        self.markers.markers.append(marker)

    def marker(self, ns: str, marker_type: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.stamp
        marker.ns = ns
        marker.id = self.next_id
        self.next_id += 1
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def add(self, marker: Marker) -> None:
        self.markers.markers.append(marker)


class PayloadTransportDemo(Node):
    """Animate the certified symbolic workflow in RViz."""

    def __init__(self) -> None:
        super().__init__("random_payload_demo")

        self.declare_parameter("dt", 0.08)
        self.declare_parameter("frames_per_tick", 1)
        self.declare_parameter("initial_hold_seconds", 0.0)
        self.declare_parameter("final_hold_seconds", 0.0)
        self.declare_parameter("loop_demo", True)
        self.declare_parameter("publish_frame", "map")
        self.declare_parameter("show_region_labels", True)
        self.declare_parameter("show_bridge_labels", True)
        self.declare_parameter("show_formation_layer_graph", False)
        self.declare_parameter("show_formation_graph_edges", False)
        self.declare_parameter("show_symbolic_route", False)
        self.declare_parameter("show_agent_trajectories", False)
        self.declare_parameter("show_current_status_text", False)
        self.declare_parameter("show_cinematic_hud", False)
        self.declare_parameter("show_controller_arrows", True)
        self.declare_parameter("show_agent_labels", True)
        self.declare_parameter("publish_centroid_path", False)
        self.declare_parameter("publish_follow_tf", True)
        self.declare_parameter("follow_frame", "payload_follow")
        self.declare_parameter("follow_frame_height", 0.85)
        self.declare_parameter("follow_lock_yaw", False)
        self.declare_parameter("follow_position_alpha", 0.16)
        self.declare_parameter("follow_yaw_alpha", 0.10)
        self.declare_parameter("use_drone_mesh", True)
        self.declare_parameter("show_propeller_blur", True)
        self.declare_parameter("drone_mesh_scale", 0.44)
        self.declare_parameter("map_scale", 0.01)
        self.declare_parameter("display_scale", 0.018)
        self.declare_parameter("case_file", "")
        self.declare_parameter("control_mode", "")

        self.dt = float(self.get_parameter("dt").value)
        self.frames_per_tick = max(1, int(self.get_parameter("frames_per_tick").value))
        self.initial_hold_ticks = max(0, int(math.ceil(float(self.get_parameter("initial_hold_seconds").value) / self.dt)))
        self.final_hold_ticks = max(0, int(math.ceil(float(self.get_parameter("final_hold_seconds").value) / self.dt)))
        self.loop_demo = bool(self.get_parameter("loop_demo").value)
        self.frame_id = str(self.get_parameter("publish_frame").value)
        self.show_region_labels = bool(self.get_parameter("show_region_labels").value)
        self.show_bridge_labels = bool(self.get_parameter("show_bridge_labels").value)
        self.show_formation_layer_graph = bool(self.get_parameter("show_formation_layer_graph").value)
        self.show_formation_graph_edges = bool(self.get_parameter("show_formation_graph_edges").value)
        self.show_symbolic_route = bool(self.get_parameter("show_symbolic_route").value)
        self.show_agent_trajectories = bool(self.get_parameter("show_agent_trajectories").value)
        self.show_current_status_text = bool(self.get_parameter("show_current_status_text").value)
        self.show_cinematic_hud = bool(self.get_parameter("show_cinematic_hud").value)
        self.show_controller_arrows = bool(self.get_parameter("show_controller_arrows").value)
        self.show_agent_labels = bool(self.get_parameter("show_agent_labels").value)
        self.publish_centroid_path = bool(self.get_parameter("publish_centroid_path").value)
        self.publish_follow_tf = bool(self.get_parameter("publish_follow_tf").value)
        self.follow_frame = str(self.get_parameter("follow_frame").value)
        self.follow_frame_height = float(self.get_parameter("follow_frame_height").value)
        self.follow_lock_yaw = bool(self.get_parameter("follow_lock_yaw").value)
        self.follow_position_alpha = float(self.get_parameter("follow_position_alpha").value)
        self.follow_yaw_alpha = float(self.get_parameter("follow_yaw_alpha").value)
        self.use_drone_mesh = bool(self.get_parameter("use_drone_mesh").value)
        self.show_propeller_blur = bool(self.get_parameter("show_propeller_blur").value)
        self.drone_mesh_scale = float(self.get_parameter("drone_mesh_scale").value)
        self.map_scale = float(self.get_parameter("map_scale").value)
        self.display_scale = float(self.get_parameter("display_scale").value)
        case_file = str(self.get_parameter("case_file").value)
        case_file = case_file if case_file else None
        control_mode = str(self.get_parameter("control_mode").value)
        control_mode = control_mode if control_mode else None

        self.scenario = PayloadTransportScenario(map_scale=self.map_scale, case_file=case_file, control_mode=control_mode)
        failed = [name for name, ok in self.scenario.checks.items() if not ok]
        if failed:
            raise RuntimeError(f"Payload workflow checks failed: {failed}")
        self.scenario.map_scale = self.display_scale

        self.frame_index = 0
        self.initial_hold_tick = 0
        self.final_hold_tick = 0
        self.centroid_history: List[Point2] = []
        self.agent_histories: List[List[Point2]] = [[], [], []]
        self.agent_yaws: List[float] = [0.0, 0.0, 0.0]
        self.follow_center: Point2 | None = None
        self.follow_yaw = 0.0
        self.rotor_spin_angle = 0.0

        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_follow_tf else None
        self.pose_pub = self.create_publisher(PoseArray, "swarm/poses", 10)
        self.path_pub = self.create_publisher(Path, "swarm/centroid_path", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "swarm/markers", 10)
        self.status_pub = self.create_publisher(String, "swarm/status", 10)

        self.timer = self.create_timer(self.dt, self.on_timer)
        self.get_logger().info(
            "Certified payload transport demo started. Add MarkerArray /swarm/markers and PoseArray /swarm/poses in RViz2."
        )

    def on_timer(self) -> None:
        self.rotor_spin_angle = math.fmod(self.rotor_spin_angle + 7.5 * self.dt, 2.0 * math.pi)
        frame = self.scenario.frames[self.frame_index]
        robots_img = frame.robots
        robots = self.scenario.to_world_poly(robots_img)
        if self.frame_index == 0 and self.initial_hold_tick < self.initial_hold_ticks:
            self.publish_follow_transform(robots)
            self.publish_poses(robots)
            if self.publish_centroid_path:
                self.publish_path()
            self.publish_markers(frame.state_index, robots)
            self.publish_status(frame.state_index)
            self.initial_hold_tick += 1
            return

        c = centroid(robots)
        self.centroid_history.append(c)
        for idx, robot in enumerate(robots):
            self.agent_histories[idx].append(robot)

        self.publish_follow_transform(robots)
        self.publish_poses(robots)
        if self.publish_centroid_path:
            self.publish_path()
        self.publish_markers(frame.state_index, robots)
        self.publish_status(frame.state_index)

        last_frame_index = len(self.scenario.frames) - 1
        if self.frame_index >= last_frame_index:
            if self.loop_demo:
                if self.final_hold_tick < self.final_hold_ticks:
                    self.final_hold_tick += 1
                    self.frame_index = last_frame_index
                else:
                    self.frame_index = 0
                    self.initial_hold_tick = 0
                    self.final_hold_tick = 0
                    self.follow_center = None
                    self.centroid_history.clear()
                    for history in self.agent_histories:
                        history.clear()
            else:
                self.frame_index = last_frame_index
            return

        self.final_hold_tick = 0
        self.frame_index = min(last_frame_index, self.frame_index + self.frames_per_tick)

    def publish_poses(self, robots: Sequence[Point2]) -> None:
        msg = PoseArray()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in robots:
            msg.poses.append(make_pose(x, y, 0.25))
        self.pose_pub.publish(msg)

    def publish_follow_transform(self, robots: Sequence[Point2]) -> None:
        if self.tf_broadcaster is None or not robots:
            return

        center = centroid(robots)
        smooth_center = self.smoothed_follow_center(center)
        yaw = 0.0 if self.follow_lock_yaw else self.follow_heading(smooth_center)

        msg = TransformStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.child_frame_id = self.follow_frame
        msg.transform.translation.x = smooth_center[0]
        msg.transform.translation.y = smooth_center[1]
        msg.transform.translation.z = self.follow_frame_height
        msg.transform.rotation.z = math.sin(0.5 * yaw)
        msg.transform.rotation.w = math.cos(0.5 * yaw)
        self.tf_broadcaster.sendTransform(msg)

    def smoothed_follow_center(self, target_center: Point2) -> Point2:
        if self.follow_center is None:
            self.follow_center = target_center
            return target_center

        alpha = max(0.01, min(1.0, self.follow_position_alpha))
        x = self.follow_center[0] + alpha * (target_center[0] - self.follow_center[0])
        y = self.follow_center[1] + alpha * (target_center[1] - self.follow_center[1])
        self.follow_center = (x, y)
        return self.follow_center

    def follow_heading(self, current_center: Point2) -> float:
        next_index = min(self.frame_index + 1, len(self.scenario.frames) - 1)
        next_center = centroid(self.scenario.to_world_poly(self.scenario.frames[next_index].robots))
        dx = next_center[0] - current_center[0]
        dy = next_center[1] - current_center[1]
        if math.hypot(dx, dy) > 1e-4:
            target_yaw = math.atan2(dy, dx)
            self.follow_yaw = self.smoothed_angle(self.follow_yaw, target_yaw, self.follow_yaw_alpha)
        return self.follow_yaw

    def smoothed_angle(self, current: float, target: float, alpha: float) -> float:
        alpha = max(0.01, min(1.0, alpha))
        delta = math.atan2(math.sin(target - current), math.cos(target - current))
        return current + alpha * delta

    def publish_path(self) -> None:
        msg = Path()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.centroid_history:
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.header.stamp = msg.header.stamp
            pose.pose = make_pose(x, y, 0.06)
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    def publish_status(self, state_index: int) -> None:
        state = self.scenario.states[state_index]
        frame = self.scenario.frames[self.frame_index]
        min_pair = self.current_min_pairwise_distance(frame.robots)
        min_obs = min(signed_distance_to_polygons(robot, self.scenario.obstacles) for robot in frame.robots)
        min_workspace = workspace_clearance(frame.robots, self.scenario.width, self.scenario.height)
        msg = String()
        msg.data = (
            f"state={state.state_id} paper={state.paper_state} mode={state.task_mode} "
            f"formation={state.formation} region={state.region} frame={self.frame_index}/{len(self.scenario.frames)-1} "
            f"min_pair={min_pair * self.map_scale:.3f}m min_obstacle_clearance={min_obs * self.map_scale:.3f}m "
            f"min_workspace_clearance={min_workspace * self.map_scale:.3f}m"
        )
        if frame.controller == "paper_qp":
            delta1 = frame.qp_delta1 if frame.qp_delta1 is not None else float("nan")
            delta2 = frame.qp_delta2 if frame.qp_delta2 is not None else float("nan")
            msg.data += (
                f" controller=paper_qp qp={frame.qp_status} "
                f"delta1={delta1:.3f} delta2={delta2:.3f}"
            )
        if state.event:
            msg.data += f" event={state.event}"
        self.status_pub.publish(msg)

    def publish_markers(self, state_index: int, robots: Sequence[Point2]) -> None:
        stamp = self.get_clock().now().to_msg()
        mb = MarkerBuilder(self.frame_id, stamp)
        self.add_static_map(mb)
        self.add_current_zone_and_bridge(mb, state_index)
        if self.show_symbolic_route:
            self.add_symbolic_route(mb)
        if self.show_agent_trajectories:
            self.add_agent_trajectories(mb)
        if self.show_formation_layer_graph:
            self.add_formation_layer_graph(mb)
        self.add_dynamic_swarm(mb, state_index, robots)
        if self.show_cinematic_hud:
            self.add_cinematic_hud(mb, state_index, robots)
        self.marker_pub.publish(mb.markers)

    def add_static_map(self, mb: MarkerBuilder) -> None:
        self.add_workspace(mb)
        self.add_boundary_obstacles(mb)
        for region in self.scenario.regions:
            poly = self.scenario.to_world_poly(region["poly"])
            self.add_polygon(mb, "certified_regions", poly, 0.015, REGION_FILL)
            self.add_outline(mb, "certified_region_outlines", poly, 0.035, REGION_OUTLINE)

        for idx, obstacle in enumerate(self.scenario.obstacles):
            self.add_obstacle_prism(mb, idx, self.scenario.to_world_poly(obstacle))
        self.add_task_regions(mb)

    def add_workspace(self, mb: MarkerBuilder) -> None:
        marker = mb.marker("workspace", Marker.CUBE)
        cx, cy = self.scenario.to_world((self.scenario.width / 2.0, self.scenario.height / 2.0))
        marker.pose = make_pose(cx, cy, -0.02)
        marker.scale.x = self.scenario.world_length(self.scenario.width)
        marker.scale.y = self.scenario.world_length(self.scenario.height)
        marker.scale.z = 0.02
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = FREE_SPACE
        mb.add(marker)
        corners = [
            self.scenario.to_world((0.0, 0.0)),
            self.scenario.to_world((self.scenario.width, 0.0)),
            self.scenario.to_world((self.scenario.width, self.scenario.height)),
            self.scenario.to_world((0.0, self.scenario.height)),
        ]
        self.add_outline(mb, "workspace_boundary", corners, 0.01, (0.38, 0.45, 0.55, 0.85))

    def add_boundary_obstacles(self, mb: MarkerBuilder) -> None:
        width = self.scenario.world_length(self.scenario.width)
        height = self.scenario.world_length(self.scenario.height)
        margin = self.scenario.world_length(self.scenario.obstacle_margin)
        outer_corners = [
            (-margin, -margin),
            (width + margin, -margin),
            (width + margin, height + margin),
            (-margin, height + margin),
        ]
        self.add_outline(mb, "outer_safety_boundary", outer_corners, 0.13, SAFETY_BOUNDARY_COLOR, width=0.025)

    def add_task_regions(self, mb: MarkerBuilder) -> None:
        task_colors = {"home": HOME_COLOR, "pick": PICK_COLOR, "drop": DROP_COLOR}
        for name, task in self.scenario.task.items():
            cx, cy = self.scenario.to_world(task["center"])
            size = self.scenario.world_length(task["size"])
            marker = mb.marker(f"task_{name}", Marker.CUBE)
            marker.pose = make_pose(cx, cy, 0.055)
            marker.scale.x = size
            marker.scale.y = size
            marker.scale.z = 0.045
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = task_colors[name]
            mb.add(marker)
            label_color = (task_colors[name][0], task_colors[name][1], task_colors[name][2], 1.0)
            self.add_text(mb, "task_labels", (cx, cy), 0.28, task["label"], label_color, z=0.32)

        rx, ry = self.scenario.to_world(self.scenario.return_point["point"])
        self.add_sphere(mb, "return_point", (rx, ry), 0.18, DELIVERED, z=0.18)
        self.add_text(mb, "task_labels", (rx, ry), 0.24, "RETURN", DELIVERED, z=0.46)

    def add_current_zone_and_bridge(self, mb: MarkerBuilder, state_index: int) -> None:
        current_region = self.current_region_for_state_index(state_index)
        current_region_name = None
        if current_region is not None:
            current_region_name = current_region["name"]
            poly = self.scenario.to_world_poly(current_region["poly"])
            self.add_outline(mb, "current_certified_region", poly, 0.155, CURRENT_REGION_OUTLINE, width=0.06)

        bridge = self.bridge_for_transition_to_state(state_index)
        if bridge is not None:
            for region_name in bridge["regions"]:
                if region_name == current_region_name:
                    continue
                other_region = self.region_named(region_name)
                if other_region is None:
                    continue
                other_poly = self.scenario.to_world_poly(other_region["poly"])
                self.add_outline(mb, "other_bridge_region", other_poly, 0.155, OTHER_BRIDGE_REGION_OUTLINE, width=0.06)
            poly = self.scenario.to_world_poly(bridge["poly"])
            self.add_outline(mb, "active_bridge_certificate", poly, 0.205, BRIDGE_OUTLINE, width=0.03)

    def current_region_for_state_index(self, state_index: int) -> dict | None:
        if not self.scenario.states:
            return None
        index = min(max(state_index, 0), len(self.scenario.states) - 1)
        if index == 0:
            names = sorted(self.region_names_from_label(self.scenario.states[0].region))
            return self.region_named(names[0]) if names else None

        source_names = self.region_names_from_label(self.scenario.states[index - 1].region)
        target_names = self.region_names_from_label(self.scenario.states[index].region)
        shared = sorted(source_names.intersection(target_names))
        if shared:
            return self.region_named(shared[0])

        if index - 1 < len(self.scenario.transitions):
            certificate_names = sorted(self.region_names_from_label(self.scenario.transitions[index - 1].certificate))
            if len(certificate_names) == 1:
                return self.region_named(certificate_names[0])

        source_single = sorted(source_names)
        if len(source_single) == 1:
            return self.region_named(source_single[0])
        target_single = sorted(target_names)
        if len(target_single) == 1:
            return self.region_named(target_single[0])
        return None

    def bridge_for_transition_to_state(self, state_index: int) -> dict | None:
        if state_index <= 0 or state_index - 1 >= len(self.scenario.transitions):
            return None
        transition = self.scenario.transitions[state_index - 1]
        region_pair = self.region_names_from_label(transition.certificate)
        if len(region_pair) != 2:
            return None

        target_state = self.scenario.states[min(state_index, len(self.scenario.states) - 1)]
        fallback = None
        for bridge in self.scenario.bridges.values():
            if set(bridge["regions"]) != region_pair:
                continue
            if fallback is None:
                fallback = bridge
            if bridge["formation"] == target_state.formation:
                return bridge
        return fallback

    def region_names_from_label(self, label: str) -> set[str]:
        return {part.strip() for part in label.split(" ∩ ") if part.strip()}

    def region_named(self, name: str) -> dict | None:
        for region in self.scenario.regions:
            if region["name"] == name:
                return region
        return None

    def add_symbolic_route(self, mb: MarkerBuilder) -> None:
        state_points = {state.state_id: self.scenario.to_world(state.point) for state in self.scenario.states}
        for transition in self.scenario.transitions:
            p = state_points[transition.src]
            q = state_points[transition.dst]
            color = LOADED if self.scenario.edge_phase(transition) == "loaded" else DELIVERED
            self.add_arrow(mb, "certified_symbolic_edges", p, q, 0.11, color, z=0.34)
            mx = (p[0] + q[0]) / 2.0
            my = (p[1] + q[1]) / 2.0
            self.add_text(mb, "edge_certificates", (mx, my), 0.10, transition.certificate, color, z=0.48)

    def add_agent_trajectories(self, mb: MarkerBuilder) -> None:
        for idx, history in enumerate(self.agent_histories):
            if len(history) < 2:
                continue
            color = AGENT_COLORS[idx]
            points = [(x, y, 0.58 + idx * 0.015) for x, y in history]
            self.add_line(mb, f"agent_{idx + 1}_trajectory", points, 0.035, color)

    def add_formation_layer_graph(self, mb: MarkerBuilder) -> None:
        z0 = 1.05
        dz = 0.42
        width = self.scenario.world_length(self.scenario.width)
        height = self.scenario.world_length(self.scenario.height)
        cx, cy = self.scenario.to_world((self.scenario.width / 2.0, self.scenario.height / 2.0))
        for formation, layer in FORMATION_LAYERS.items():
            z = z0 + layer * dz
            marker = mb.marker("formation_layers", Marker.CUBE)
            marker.pose = make_pose(cx, cy, z)
            marker.scale.x = width
            marker.scale.y = height
            marker.scale.z = 0.01
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0.52, 0.62, 0.75, 0.11)
            mb.add(marker)
            self.add_text(mb, "formation_layer_labels", (0.35, cy + height / 2.0 + 0.25), 0.16, formation, (0.18, 0.25, 0.36, 0.95), z=z + 0.08)

        state_points = {}
        for state in self.scenario.states:
            x, y = self.scenario.to_world(state.point)
            z = z0 + FORMATION_LAYERS[state.formation] * dz + 0.07
            state_points[state.state_id] = (x, y, z)

        if self.show_formation_graph_edges:
            for transition in self.scenario.transitions:
                p = state_points[transition.src]
                q = state_points[transition.dst]
                color = LOADED if self.scenario.edge_phase(transition) == "loaded" else DELIVERED
                self.add_line(mb, "formation_graph_edges", [p, q], 0.035, color)

    def add_dynamic_swarm(self, mb: MarkerBuilder, state_index: int, robots: Sequence[Point2]) -> None:
        state = self.scenario.states[state_index]
        frame = self.scenario.frames[self.frame_index]
        color = LOADED if state.task_mode in {"empty", "loaded"} else DELIVERED
        self.add_polygon(mb, "current_envelope", robots, 0.62, (color[0], color[1], color[2], 0.18))
        closed = list(robots) + [robots[0]]
        line_points = [(x, y, 0.72) for x, y in closed]
        self.add_line(mb, "current_formation", line_points, 0.035, (0.02, 0.03, 0.05, 0.95))

        headings = self.current_agent_headings(robots)
        for idx, (x, y) in enumerate(robots):
            safety = mb.marker("agent_safety_radius", Marker.CYLINDER)
            safety.pose = make_pose(x, y, 0.60)
            safety.scale.x = safety.scale.y = self.scenario.world_length(self.scenario.safe_distance) * 0.70
            safety.scale.z = 0.012
            safety.color.r, safety.color.g, safety.color.b, safety.color.a = (*AGENT_COLORS[idx][:3], 0.055)
            mb.add(safety)
            if self.use_drone_mesh:
                self.add_drone_mesh(mb, idx, (x, y), headings[idx])
            else:
                self.add_sphere(mb, "agents", (x, y), 0.14, AGENT_COLORS[idx], z=0.82)
            if self.show_agent_labels:
                self.add_text(mb, "agent_labels", (x, y), 0.10, str(idx + 1), (1.0, 1.0, 1.0, 1.0), z=1.13)

        if self.show_controller_arrows:
            next_frame_index = min(self.frame_index + 1, len(self.scenario.frames) - 1)
            next_robots = self.scenario.to_world_poly(self.scenario.frames[next_frame_index].robots)
            for (x, y), (nx, ny) in zip(robots, next_robots):
                dx = nx - x
                dy = ny - y
                length = math.hypot(dx, dy)
                if length < 1e-5:
                    continue
                visual_length = min(0.45, max(0.16, length * 6.0))
                end = (x + dx / length * visual_length, y + dy / length * visual_length)
                self.add_arrow(mb, "controller_outputs", (x, y), end, 0.055, (0.96, 0.65, 0.05, 0.95), z=1.06)

        c = centroid(robots)
        payload_loaded = state.task_mode == "loaded"
        if payload_loaded:
            marker = mb.marker("payload", Marker.CUBE)
            marker.pose = make_pose(c[0], c[1], 0.92)
            marker.scale.x = marker.scale.y = 0.20
            marker.scale.z = 0.14
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0.98, 0.45, 0.10, 0.95)
            mb.add(marker)
            shadow = mb.marker("payload_shadow", Marker.CYLINDER)
            shadow.pose = make_pose(c[0], c[1], 0.61)
            shadow.scale.x = shadow.scale.y = 0.32
            shadow.scale.z = 0.01
            shadow.color.r, shadow.color.g, shadow.color.b, shadow.color.a = (0.0, 0.0, 0.0, 0.18)
            mb.add(shadow)
            for x, y in robots:
                self.add_line(
                    mb,
                    "payload_suspension_lines",
                    [(x, y, 0.78), (c[0], c[1], 0.96)],
                    0.018,
                    (0.05, 0.06, 0.08, 0.82),
                )
        if self.show_current_status_text:
            self.add_text(
                mb,
                "current_status",
                (c[0], c[1]),
                0.13,
                f"{state.task_mode} / target {state.formation} / d_min {self.current_min_pairwise_distance(frame.robots) * self.map_scale:.2f}m",
                color,
                z=1.20,
            )

    def add_cinematic_hud(self, mb: MarkerBuilder, state_index: int, robots: Sequence[Point2]) -> None:
        state = self.scenario.states[state_index]
        current_region = self.current_region_for_state_index(state_index)
        bridge = self.bridge_for_transition_to_state(state_index)

        current_label = current_region["name"] if current_region is not None else state.region
        if bridge is None:
            bridge_label = "--"
            next_label = state.region
        else:
            bridge_label = " & ".join(bridge["regions"])
            other_regions = [name for name in bridge["regions"] if name != current_label]
            next_label = other_regions[0] if other_regions else state.region

        total_frames = max(1, len(self.scenario.frames) - 1)
        progress = min(1.0, max(0.0, self.frame_index / total_frames))
        world_width = self.scenario.world_length(self.scenario.width)
        world_height = self.scenario.world_length(self.scenario.height)
        panel_width = min(6.4, max(4.8, world_width * 0.55))
        panel_height = 0.62
        panel_x = panel_width / 2.0 + 0.18
        panel_y = world_height + 0.46
        panel_z = 1.05

        panel = mb.marker("cinematic_hud_panel", Marker.CUBE)
        panel.pose = make_pose(panel_x, panel_y, panel_z)
        panel.scale.x = panel_width
        panel.scale.y = panel_height
        panel.scale.z = 0.035
        panel.color.r, panel.color.g, panel.color.b, panel.color.a = (0.04, 0.06, 0.10, 0.58)
        mb.add(panel)

        title_color = (0.90, 0.96, 1.00, 0.98)
        muted_color = (0.64, 0.76, 0.88, 0.95)
        self.add_text(
            mb,
            "cinematic_hud_text",
            (panel_x, panel_y + 0.13),
            0.13,
            f"{state.task_mode.upper()}  |  {state.formation.upper()} formation  |  state {state_index + 1}/{len(self.scenario.states)}",
            title_color,
            z=panel_z + 0.08,
        )
        self.add_text(
            mb,
            "cinematic_hud_text",
            (panel_x, panel_y - 0.08),
            0.095,
            f"current: {current_label}    bridge: {bridge_label}    next: {next_label}",
            muted_color,
            z=panel_z + 0.08,
        )

        bar_width = panel_width - 0.50
        bar_back = mb.marker("cinematic_hud_progress", Marker.CUBE)
        bar_back.pose = make_pose(panel_x, panel_y - 0.25, panel_z + 0.02)
        bar_back.scale.x = bar_width
        bar_back.scale.y = 0.035
        bar_back.scale.z = 0.025
        bar_back.color.r, bar_back.color.g, bar_back.color.b, bar_back.color.a = (0.28, 0.35, 0.45, 0.80)
        mb.add(bar_back)

        fill_width = max(0.03, bar_width * progress)
        bar_fill = mb.marker("cinematic_hud_progress", Marker.CUBE)
        bar_fill.pose = make_pose(panel_x - bar_width / 2.0 + fill_width / 2.0, panel_y - 0.25, panel_z + 0.05)
        bar_fill.scale.x = fill_width
        bar_fill.scale.y = 0.045
        bar_fill.scale.z = 0.030
        bar_fill.color.r, bar_fill.color.g, bar_fill.color.b, bar_fill.color.a = (0.00, 0.78, 0.95, 0.95)
        mb.add(bar_fill)

    def current_agent_headings(self, robots: Sequence[Point2]) -> List[float]:
        next_index = min(self.frame_index + 1, len(self.scenario.frames) - 1)
        prev_index = max(self.frame_index - 1, 0)
        next_robots = self.scenario.to_world_poly(self.scenario.frames[next_index].robots)
        prev_robots = self.scenario.to_world_poly(self.scenario.frames[prev_index].robots)

        headings: List[float] = []
        for idx, ((x, y), (nx, ny), (px, py)) in enumerate(zip(robots, next_robots, prev_robots)):
            dx = nx - x
            dy = ny - y
            if math.hypot(dx, dy) < 1e-4:
                dx = x - px
                dy = y - py
            if math.hypot(dx, dy) < 1e-4:
                yaw = self.agent_yaws[idx]
            else:
                yaw = math.atan2(dy, dx)
            headings.append(yaw)
        self.agent_yaws = headings
        return headings

    def add_drone_mesh(self, mb: MarkerBuilder, idx: int, xy: Point2, yaw: float) -> None:
        scale = self.drone_mesh_scale
        color = AGENT_COLORS[idx]
        z_base = 0.88

        shadow = mb.marker("drone_shadow", Marker.CYLINDER)
        shadow.pose = make_pose(xy[0], xy[1], 0.605)
        shadow.scale.x = shadow.scale.y = 0.46
        shadow.scale.z = 0.012
        shadow.color.r, shadow.color.g, shadow.color.b, shadow.color.a = (0.0, 0.0, 0.0, 0.20)
        mb.add(shadow)

        self.add_mesh(
            mb,
            "drone_body",
            DRONE_BODY_MESH,
            xy,
            z_base,
            scale,
            yaw + math.pi,
            color=None,
            embedded_materials=True,
        )

        for rotor_idx, offset in enumerate(ROTOR_OFFSETS):
            rotor_xy = rotate_offset(xy, (offset[0] * scale, offset[1] * scale), yaw)
            self.add_mesh(
                mb,
                "drone_motor_base",
                DRONE_MOTOR_BASE_MESH,
                rotor_xy,
                z_base + 0.006,
                scale,
                yaw - 0.45,
                color=None,
                embedded_materials=True,
            )
            self.add_mesh(
                mb,
                "drone_motor_bell",
                DRONE_MOTOR_BELL_MESH,
                rotor_xy,
                z_base + 0.035,
                scale,
                yaw + self.rotor_spin_angle,
                color=None,
                embedded_materials=True,
            )
            self.add_propeller_visual(mb, rotor_idx, rotor_xy, z_base + 0.075, yaw, color)

    def add_propeller_visual(self, mb: MarkerBuilder, rotor_idx: int, xy: Point2, z: float, yaw: float, color: Color) -> None:
        spin_direction = 1.0 if rotor_idx in {0, 1} else -1.0
        spin_yaw = yaw + spin_direction * self.rotor_spin_angle

        if self.show_propeller_blur:
            disk = mb.marker("propeller_blur", Marker.CYLINDER)
            disk.pose = make_pose(xy[0], xy[1], z, spin_yaw)
            disk.scale.x = disk.scale.y = 0.40 * self.drone_mesh_scale
            disk.scale.z = 0.008
            disk.color.r, disk.color.g, disk.color.b, disk.color.a = (0.02, 0.02, 0.02, 0.24)
            mb.add(disk)

        blade = mb.marker("propeller_blade", Marker.CUBE)
        blade.pose = make_pose(xy[0], xy[1], z + 0.006, spin_yaw)
        blade.scale.x = 0.46 * self.drone_mesh_scale
        blade.scale.y = 0.035 * self.drone_mesh_scale
        blade.scale.z = 0.010
        blade.color.r, blade.color.g, blade.color.b, blade.color.a = (0.02, 0.02, 0.02, 0.82)
        mb.add(blade)

        blade_cross = mb.marker("propeller_blade", Marker.CUBE)
        blade_cross.pose = make_pose(xy[0], xy[1], z + 0.008, spin_yaw + math.pi / 2.0)
        blade_cross.scale.x = 0.34 * self.drone_mesh_scale
        blade_cross.scale.y = 0.025 * self.drone_mesh_scale
        blade_cross.scale.z = 0.009
        blade_cross.color.r, blade_cross.color.g, blade_cross.color.b, blade_cross.color.a = (*color[:3], 0.55)
        mb.add(blade_cross)

    def add_mesh(
        self,
        mb: MarkerBuilder,
        ns: str,
        mesh_resource: str,
        xy: Point2,
        z: float,
        scale: float,
        yaw: float,
        color: Color | None,
        embedded_materials: bool,
    ) -> None:
        marker = mb.marker(ns, Marker.MESH_RESOURCE)
        marker.mesh_resource = mesh_resource
        marker.mesh_use_embedded_materials = embedded_materials
        marker.pose = make_pose(xy[0], xy[1], z, yaw)
        marker.scale.x = marker.scale.y = marker.scale.z = scale
        if color is None:
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (1.0, 1.0, 1.0, 1.0)
        else:
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        mb.add(marker)

    def current_min_pairwise_distance(self, robots_img: Sequence[Point2]) -> float:
        best = float("inf")
        for i, p in enumerate(robots_img):
            for q in robots_img[i + 1 :]:
                dx = p[0] - q[0]
                dy = p[1] - q[1]
                best = min(best, (dx * dx + dy * dy) ** 0.5)
        return best

    def add_obstacle_prism(self, mb: MarkerBuilder, idx: int, poly: Sequence[Point2]) -> None:
        if len(poly) < 3:
            return

        base_z = 0.035
        top_z = 0.78
        rect = self.axis_aligned_rect(poly)
        if rect is not None:
            min_x, max_x, min_y, max_y = rect
            block = mb.marker(f"obstacle_solid_{idx}", Marker.CUBE)
            block.pose = make_pose((min_x + max_x) / 2.0, (min_y + max_y) / 2.0, (base_z + top_z) / 2.0)
            block.scale.x = max_x - min_x
            block.scale.y = max_y - min_y
            block.scale.z = top_z - base_z
            block.color.r, block.color.g, block.color.b, block.color.a = OBSTACLE_COLOR
            mb.add(block)
        else:
            # Extra horizontal slices make non-rectangular obstacles read as filled volumes in RViz.
            for layer in range(1, 10):
                z = base_z + (top_z - base_z) * layer / 10.0
                self.add_polygon(mb, f"obstacle_volume_fill_{idx}", poly, z, OBSTACLE_COLOR, two_sided=True)

        self.add_polygon(mb, f"obstacle_top_{idx}", poly, top_z + 0.006, OBSTACLE_TOP_COLOR, two_sided=True)

        closed = list(poly) + [poly[0]]
        for side_idx, (p, q) in enumerate(zip(closed, closed[1:])):
            sides = mb.marker(f"obstacle_side_{idx}_{side_idx}", Marker.TRIANGLE_LIST)
            side_color = OBSTACLE_SIDE_COLORS[side_idx % len(OBSTACLE_SIDE_COLORS)]
            sides.color.r, sides.color.g, sides.color.b, sides.color.a = side_color
            p0 = make_point(p[0], p[1], base_z)
            q0 = make_point(q[0], q[1], base_z)
            p1 = make_point(p[0], p[1], top_z)
            q1 = make_point(q[0], q[1], top_z)
            sides.points.extend([p0, q0, q1, p0, q1, p1])
            sides.points.extend([p0, q1, q0, p0, p1, q1])
            mb.add(sides)

        self.add_outline(mb, f"obstacle_top_outline_{idx}", poly, top_z + 0.012, OBSTACLE_EDGE_COLOR, width=0.055)
        self.add_outline(mb, f"obstacle_base_outline_{idx}", poly, base_z + 0.012, (0.05, 0.06, 0.08, 0.78), width=0.028)
        for point_idx, p in enumerate(poly):
            self.add_line(
                mb,
                f"obstacle_vertical_edge_{idx}_{point_idx}",
                [(p[0], p[1], base_z + 0.015), (p[0], p[1], top_z + 0.015)],
                0.035,
                OBSTACLE_EDGE_COLOR,
            )

    def axis_aligned_rect(self, poly: Sequence[Point2]) -> tuple[float, float, float, float] | None:
        if len(poly) != 4:
            return None
        xs = [p[0] for p in poly]
        ys = [p[1] for p in poly]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        corners = {(round(min_x, 6), round(min_y, 6)), (round(max_x, 6), round(min_y, 6)), (round(max_x, 6), round(max_y, 6)), (round(min_x, 6), round(max_y, 6))}
        actual = {(round(x, 6), round(y, 6)) for x, y in poly}
        if actual != corners:
            return None
        return min_x, max_x, min_y, max_y

    def add_polygon(self, mb: MarkerBuilder, ns: str, poly: Sequence[Point2], z: float, color: Color, two_sided: bool = False) -> None:
        if len(poly) < 3:
            return
        marker = mb.marker(ns, Marker.TRIANGLE_LIST)
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        first = poly[0]
        for i in range(1, len(poly) - 1):
            for p in (first, poly[i], poly[i + 1]):
                marker.points.append(make_point(p[0], p[1], z))
            if two_sided:
                for p in (first, poly[i + 1], poly[i]):
                    marker.points.append(make_point(p[0], p[1], z))
        mb.add(marker)

    def add_outline(self, mb: MarkerBuilder, ns: str, poly: Sequence[Point2], z: float, color: Color, width: float = 0.025) -> None:
        if len(poly) < 2:
            return
        points = [(p[0], p[1], z) for p in list(poly) + [poly[0]]]
        self.add_line(mb, ns, points, width, color)

    def add_line(self, mb: MarkerBuilder, ns: str, points: Sequence[Tuple[float, float, float]], width: float, color: Color) -> None:
        marker = mb.marker(ns, Marker.LINE_STRIP)
        marker.scale.x = width
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        for p in points:
            marker.points.append(make_point(p[0], p[1], p[2]))
        mb.add(marker)

    def add_arrow(self, mb: MarkerBuilder, ns: str, p: Point2, q: Point2, width: float, color: Color, z: float) -> None:
        marker = mb.marker(ns, Marker.ARROW)
        marker.points.append(make_point(p[0], p[1], z))
        marker.points.append(make_point(q[0], q[1], z))
        marker.scale.x = width * 0.32
        marker.scale.y = width * 0.78
        marker.scale.z = width * 1.10
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        mb.add(marker)

    def add_sphere(self, mb: MarkerBuilder, ns: str, xy: Point2, radius: float, color: Color, z: float) -> None:
        marker = mb.marker(ns, Marker.SPHERE)
        marker.pose = make_pose(xy[0], xy[1], z)
        marker.scale.x = marker.scale.y = marker.scale.z = radius
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        mb.add(marker)

    def add_text(self, mb: MarkerBuilder, ns: str, xy: Point2, size: float, text: str, color: Color, z: float) -> None:
        marker = mb.marker(ns, Marker.TEXT_VIEW_FACING)
        marker.pose = make_pose(xy[0], xy[1], z)
        marker.scale.z = size
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.text = text
        mb.add(marker)


def main() -> None:
    rclpy.init()
    node = PayloadTransportDemo()
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

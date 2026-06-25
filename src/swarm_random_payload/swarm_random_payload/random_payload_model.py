"""Data-driven certified payload transport scenario for ROS 2 visualization."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

from .paper_qp_controller import PaperQPConfig, PaperQPController

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:  # pragma: no cover - available after ROS install
    get_package_share_directory = None


Point2 = Tuple[float, float]
Polygon2 = List[Point2]

DEFAULT_MAP_SCALE = 0.01
FORMATION_LAYERS = {"line": 0.0, "triangle": 1.0}

TEMPLATES = {
    "line": {
        "robots": [(-1.0, 0.0), (0.0, 0.0), (1.0, 0.0)],
        "envelope": [(-1.08, -0.08), (1.08, -0.08), (1.08, 0.08), (-1.08, 0.08)],
    },
    "triangle": {
        "robots": [(0.0, -0.68), (-0.78, 0.58), (0.78, 0.58)],
        "envelope": [(0.0, -0.98), (1.08, 0.76), (-1.08, 0.76)],
    },
}


def default_case_path() -> Path:
    if get_package_share_directory is not None:
        try:
            package_data = Path(get_package_share_directory("swarm_random_payload")) / "data"
            generated = package_data / "generated_case.json"
            return generated if generated.exists() else package_data / "demo1_case.json"
        except Exception:
            pass
    source_data = Path(__file__).resolve().parents[1] / "data"
    generated = source_data / "generated_case.json"
    return generated if generated.exists() else source_data / "demo1_case.json"


def load_case(case_file: str | Path | None = None) -> dict[str, Any]:
    path = Path(case_file) if case_file else default_case_path()
    return json.loads(path.read_text(encoding="utf-8"))


def point_from_json(payload: dict[str, Any]) -> Point2:
    return (float(payload["x"]), float(payload["y"]))


def poly_from_json(points: Sequence[dict[str, Any]]) -> Polygon2:
    return [point_from_json(point) for point in points]


def obstacle_poly(obs: dict[str, Any]) -> Polygon2:
    if obs.get("type") == "rect":
        x = float(obs["x"])
        y = float(obs["y"])
        w = float(obs["w"])
        h = float(obs["h"])
        x0, x1 = sorted((x, x + w))
        y0, y1 = sorted((y, y + h))
        return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    return poly_from_json(obs["points"])


def regions_from_case(case: dict[str, Any]) -> List[dict[str, Any]]:
    return [
        {
            "id": int(region["id"]),
            "name": region["name"],
            "poly": poly_from_json(region["poly"]),
            "seed": point_from_json(region["seed"]),
            "area": float(region["area"]),
        }
        for region in case["regions"]
    ]


def task_from_case(case: dict[str, Any]) -> Dict[str, Dict[str, Any]]:
    return {
        name: {
            "label": task["label"],
            "role": task["role"],
            "center": point_from_json(task["center"]),
            "size": float(task["size"]),
        }
        for name, task in case["task"]["regions"].items()
    }


def return_point_from_case(case: dict[str, Any]) -> Dict[str, Any]:
    return {
        "label": case["task"]["returnPoint"]["label"],
        "role": case["task"]["returnPoint"]["role"],
        "point": point_from_json(case["task"]["returnPoint"]["point"]),
        "region_id": case["task"]["returnPoint"].get("regionId"),
    }


CASE = load_case()
PIXEL_W = float(CASE["width"])
PIXEL_H = float(CASE["height"])
FORMATION_SCALE = float(CASE["constants"]["formationScale"])
SAFE_DISTANCE = float(CASE["constants"]["safeDistance"])
OBSTACLE_MARGIN = float(CASE["constants"]["obstacleMargin"])
PLACEMENT_BOUNDARY_MARGIN = float(CASE["constants"]["placementBoundaryMargin"])

OBSTACLES: List[Polygon2] = [obstacle_poly(obs) for obs in CASE["obstacles"]]
OBSTACLE: Polygon2 = OBSTACLES[0] if OBSTACLES else []

REGIONS = regions_from_case(CASE)


def sub(a: Point2, b: Point2) -> Point2:
    return (a[0] - b[0], a[1] - b[1])


def cross(a: Point2, b: Point2) -> float:
    return a[0] * b[1] - a[1] * b[0]


def dist2(a: Point2, b: Point2) -> float:
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return dx * dx + dy * dy


def polygon_area(poly: Sequence[Point2]) -> float:
    if len(poly) < 3:
        return 0.0
    total = 0.0
    for i, p in enumerate(poly):
        total += cross(p, poly[(i + 1) % len(poly)])
    return total / 2.0


def polygon_centroid(poly: Sequence[Point2]) -> Point2:
    area = polygon_area(poly)
    if abs(area) < 1e-9:
        return (
            sum(p[0] for p in poly) / max(1, len(poly)),
            sum(p[1] for p in poly) / max(1, len(poly)),
        )
    cx = 0.0
    cy = 0.0
    for i, p in enumerate(poly):
        q = poly[(i + 1) % len(poly)]
        k = cross(p, q)
        cx += (p[0] + q[0]) * k
        cy += (p[1] + q[1]) * k
    return (cx / (6.0 * area), cy / (6.0 * area))


REGION_LABEL_POS = {region["id"]: polygon_centroid(region["poly"]) for region in REGIONS}

TASK = task_from_case(CASE)
RETURN_POINT = return_point_from_case(CASE)


@dataclass(frozen=True)
class Placement:
    center: Point2
    theta: float
    scale: float
    formation: str
    s_min: float = 0.0
    robots: Optional[Tuple[Point2, Point2, Point2]] = None


@dataclass(frozen=True)
class WorkflowState:
    state_id: str
    paper_state: str
    region: str
    formation: str
    task_mode: str
    point: Point2
    task_label: Optional[str] = None
    event: Optional[str] = None


@dataclass(frozen=True)
class WorkflowTransition:
    src: str
    dst: str
    certificate: str
    meaning: str


@dataclass(frozen=True)
class ExecutionFrame:
    robots: Tuple[Point2, Point2, Point2]
    target_placement: Placement
    target_robots: Tuple[Point2, Point2, Point2]
    state_index: int
    segment_index: int
    controls: Tuple[Point2, Point2, Point2]
    controller: str = "legacy"
    qp_success: Optional[bool] = None
    qp_delta1: Optional[float] = None
    qp_delta2: Optional[float] = None
    qp_status: Optional[str] = None
    qp_max_constraint_violation: float = 0.0


def placement_from_json(payload: dict[str, Any]) -> Placement:
    robots = None
    if payload.get("custom_robots"):
        raw_robots = tuple(point_from_json(point) for point in payload["robots"])
        robots = (raw_robots[0], raw_robots[1], raw_robots[2])
    return Placement(
        center=point_from_json(payload["center"]),
        theta=float(payload["theta"]),
        scale=float(payload["scale"]),
        formation=str(payload["formation"]),
        s_min=float(payload.get("s_min", 0.0)),
        robots=robots,
    )


def to_world(point: Point2, scale: float = DEFAULT_MAP_SCALE) -> Point2:
    return (point[0] * scale, (PIXEL_H - point[1]) * scale)


def world_length(value: float, scale: float = DEFAULT_MAP_SCALE) -> float:
    return value * scale


def transform(point: Point2, center: Point2, theta: float, scale: float) -> Point2:
    c = math.cos(theta)
    s = math.sin(theta)
    x, y = point
    return (center[0] + scale * (c * x - s * y), center[1] + scale * (s * x + c * y))


def point_in_polygon(point: Point2, poly: Sequence[Point2]) -> bool:
    x, y = point
    inside = False
    j = len(poly) - 1
    for i, pi in enumerate(poly):
        pj = poly[j]
        crosses = (pi[1] > y) != (pj[1] > y)
        if crosses:
            x_intersection = (pj[0] - pi[0]) * (y - pi[1]) / (pj[1] - pi[1] + 1e-12) + pi[0]
            if x < x_intersection:
                inside = not inside
        j = i
    return inside


def point_segment_distance(p: Point2, a: Point2, b: Point2) -> float:
    ab = sub(b, a)
    denom = ab[0] * ab[0] + ab[1] * ab[1]
    if denom < 1e-12:
        return math.sqrt(dist2(p, a))
    ap = sub(p, a)
    tau = max(0.0, min(1.0, (ap[0] * ab[0] + ap[1] * ab[1]) / denom))
    q = (a[0] + tau * ab[0], a[1] + tau * ab[1])
    return math.sqrt(dist2(p, q))


def closest_point_on_segment(p: Point2, a: Point2, b: Point2) -> Point2:
    ab = sub(b, a)
    denom = ab[0] * ab[0] + ab[1] * ab[1]
    if denom < 1e-12:
        return a
    ap = sub(p, a)
    tau = max(0.0, min(1.0, (ap[0] * ab[0] + ap[1] * ab[1]) / denom))
    return (a[0] + tau * ab[0], a[1] + tau * ab[1])


def closest_point_on_polygon(p: Point2, poly: Sequence[Point2]) -> Point2:
    best = poly[0]
    best_d2 = float("inf")
    for i, a in enumerate(poly):
        candidate = closest_point_on_segment(p, a, poly[(i + 1) % len(poly)])
        d2 = dist2(p, candidate)
        if d2 < best_d2:
            best = candidate
            best_d2 = d2
    return best


def signed_distance_to_polygon(p: Point2, poly: Sequence[Point2]) -> float:
    closest = closest_point_on_polygon(p, poly)
    distance = math.sqrt(dist2(p, closest))
    return -distance if point_in_polygon(p, poly) else distance


def signed_distance_to_polygons(p: Point2, polygons: Sequence[Sequence[Point2]]) -> float:
    if not polygons:
        return float("inf")
    return min(signed_distance_to_polygon(p, poly) for poly in polygons)


def max_distance(a: Sequence[Point2], b: Sequence[Point2]) -> float:
    return max(math.sqrt(dist2(p, q)) for p, q in zip(a, b))


def workspace_clearance(robots: Sequence[Point2], width: float, height: float) -> float:
    if not robots:
        return float("inf")
    return min(min(x, width - x, y, height - y) for x, y in robots)


def enforce_workspace_safety(
    robots: Sequence[Point2],
    width: float,
    height: float,
    margin: float,
) -> Tuple[Point2, Point2, Point2]:
    if not robots:
        return tuple(robots)  # type: ignore[return-value]
    min_x = min(p[0] for p in robots)
    max_x = max(p[0] for p in robots)
    min_y = min(p[1] for p in robots)
    max_y = max(p[1] for p in robots)
    dx = 0.0
    dy = 0.0
    if min_x < margin:
        dx = margin - min_x
    elif max_x > width - margin:
        dx = width - margin - max_x
    if min_y < margin:
        dy = margin - min_y
    elif max_y > height - margin:
        dy = height - margin - max_y
    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        return tuple((float(x), float(y)) for x, y in robots)  # type: ignore[return-value]
    return tuple((float(x + dx), float(y + dy)) for x, y in robots)  # type: ignore[return-value]


def smoothstep(tau: float) -> float:
    return tau * tau * (3.0 - 2.0 * tau)


def angle_delta(start: float, end: float) -> float:
    return (end - start + math.pi) % (2.0 * math.pi) - math.pi


def interpolate_placement(start: Placement, end: Placement, tau: float) -> Placement:
    a = smoothstep(tau)
    center = (
        start.center[0] + (end.center[0] - start.center[0]) * a,
        start.center[1] + (end.center[1] - start.center[1]) * a,
    )
    theta = start.theta + angle_delta(start.theta, end.theta) * a
    scale = start.scale + (end.scale - start.scale) * a
    return Placement(center=center, theta=theta, scale=scale, formation=end.formation, s_min=end.s_min)


class PayloadTransportScenario:
    """Certified symbolic workflow and controller-generated execution frames."""

    def __init__(
        self,
        map_scale: float = DEFAULT_MAP_SCALE,
        segment_steps: int = 24,
        case_file: str | None = None,
        control_mode: str | None = None,
    ) -> None:
        self.map_scale = map_scale
        self.segment_steps = segment_steps
        self.case = load_case(case_file)
        self.control_mode = self._resolve_control_mode(control_mode)
        self.width = float(self.case["width"])
        self.height = float(self.case["height"])
        self.formation_scale = float(self.case["constants"]["formationScale"])
        self.safe_distance = float(self.case["constants"]["safeDistance"])
        self.safety_margin = float(self.case["constants"].get("safetyMargin", 0.0))
        self.obstacle_margin = float(self.case["constants"]["obstacleMargin"])
        self.placement_boundary_margin = float(self.case["constants"]["placementBoundaryMargin"])
        self.workspace_robot_margin = max(self.placement_boundary_margin, self.safety_margin)
        self.obstacles = [obstacle_poly(obs) for obs in self.case["obstacles"]]
        self.regions = regions_from_case(self.case)
        self.region_label_pos = {region["id"]: polygon_centroid(region["poly"]) for region in self.regions}
        self.region_allowed_placements = self._load_region_allowed_placements()
        self.task = task_from_case(self.case)
        self.return_point = return_point_from_case(self.case)
        self.bridges = self._load_bridges()
        self.states, self.transitions, self.placements = self._load_workflow()
        self.state_by_id = {state.state_id: state for state in self.states}
        self.paper_qp_config = PaperQPConfig.from_case(self.case, self.map_scale)
        self.paper_qp_controller = self._make_paper_qp_controller() if self.control_mode == "paper_qp" else None
        self.frames, self.key_frame_indices = self._build_execution()
        self.checks = self.validate()

    def _resolve_control_mode(self, override: str | None) -> str:
        if override:
            return override
        control = self.case.get("control", {})
        if isinstance(control, dict):
            return str(control.get("mode", "legacy"))
        return "legacy"

    def _make_paper_qp_controller(self) -> PaperQPController:
        return PaperQPController(
            width=self.width,
            height=self.height,
            map_scale=self.map_scale,
            safe_distance=self.safe_distance,
            obstacle_margin=self.obstacle_margin,
            workspace_margin=self.workspace_robot_margin,
            config=self.paper_qp_config,
        )

    def _load_bridges(self) -> Dict[str, Dict[str, Any]]:
        bridges = {}
        for bridge in self.case["bridges"]:
            bridges[bridge["name"]] = {
                "name": bridge["name"],
                "regions": tuple(bridge["regions"]),
                "poly": poly_from_json(bridge["poly"]),
                "centroid": point_from_json(bridge["centroid"]),
                "formation": bridge["formation"],
                "mode": bridge.get("mode", "loaded"),
                "placement": placement_from_json(bridge["placement"]),
            }
        return bridges

    def _load_region_allowed_placements(self) -> Dict[str, Dict[str, Placement]]:
        allowed: Dict[str, Dict[str, Placement]] = {}
        for region in self.case["regions"]:
            name = str(region["name"])
            allowed[name] = {}
            for formation, payload in region.get("allowed", {}).items():
                allowed[name][formation] = placement_from_json(payload)
        return allowed

    def _load_workflow(self) -> Tuple[List[WorkflowState], List[WorkflowTransition], List[Placement]]:
        states = []
        placements = []
        for item in self.case["states"]:
            states.append(
                WorkflowState(
                    state_id=item["state_id"],
                    paper_state=item["paper_state"],
                    region=item["region"],
                    formation=item["formation"],
                    task_mode=item["task_mode"],
                    point=point_from_json(item["point"]),
                    task_label=item.get("task_label"),
                    event=item.get("event"),
                )
            )
            placements.append(placement_from_json(item["placement"]))
        transitions = [
            WorkflowTransition(
                src=item["src"],
                dst=item["dst"],
                certificate=item["certificate"],
                meaning=item["meaning"],
            )
            for item in self.case["transitions"]
        ]
        return states, transitions, placements

    def placed_robots(self, placement: Placement) -> Polygon2:
        if placement.robots is not None:
            return list(placement.robots)
        return [transform(p, placement.center, placement.theta, placement.scale) for p in TEMPLATES[placement.formation]["robots"]]

    def placed_envelope(self, placement: Placement) -> Polygon2:
        return [transform(p, placement.center, placement.theta, placement.scale) for p in TEMPLATES[placement.formation]["envelope"]]

    def to_world(self, point: Point2) -> Point2:
        return (point[0] * self.map_scale, (self.height - point[1]) * self.map_scale)

    def to_world_poly(self, poly: Sequence[Point2]) -> Polygon2:
        return [self.to_world(p) for p in poly]

    def world_length(self, value: float) -> float:
        return world_length(value, self.map_scale)

    def _build_execution(self) -> Tuple[List[ExecutionFrame], List[int]]:
        if self.control_mode == "paper_qp":
            return self._build_paper_qp_execution()
        return self._build_legacy_execution()

    def _build_legacy_execution(self) -> Tuple[List[ExecutionFrame], List[int]]:
        frames: List[ExecutionFrame] = []
        key_indices: List[int] = []
        robots = tuple(self.placed_robots(self.placements[0]))  # type: ignore[assignment]
        zero_controls = ((0.0, 0.0), (0.0, 0.0), (0.0, 0.0))
        frames.append(
            ExecutionFrame(
                robots=robots,
                target_placement=self.placements[0],
                target_robots=robots,
                state_index=0,
                segment_index=0,
                controls=zero_controls,
            )
        )
        key_indices.append(0)

        for idx, placement in enumerate(self.placements[1:], start=1):
            previous_placement = self.placements[idx - 1]
            target = tuple(self.placed_robots(placement))  # type: ignore[assignment]
            uses_template_interpolation = (
                previous_placement.formation == placement.formation
                and previous_placement.robots is None
                and placement.robots is None
            )
            if uses_template_interpolation:
                distance = math.sqrt(dist2(previous_placement.center, placement.center))
                steps = max(self.segment_steps, int(distance / 10.0))
                for step in range(1, steps + 1):
                    interpolated = interpolate_placement(previous_placement, placement, step / steps)
                    next_robots = tuple(self.placed_robots(interpolated))  # type: ignore[assignment]
                    for _ in range(4):
                        next_robots = enforce_pairwise_safety(next_robots, min_distance=self.safe_distance + 0.25)
                        next_robots = enforce_obstacle_safety(
                            next_robots,
                            self.obstacles,
                            margin=self.obstacle_margin + 2.0,
                        )
                        next_robots = enforce_workspace_safety(
                            next_robots,
                            self.width,
                            self.height,
                            self.workspace_robot_margin,
                        )
                    next_robots = enforce_workspace_safety(
                        next_robots,
                        self.width,
                        self.height,
                        self.workspace_robot_margin,
                    )
                    controls = tuple(sub(q, p) for p, q in zip(robots, next_robots))  # type: ignore[assignment]
                    robots = next_robots
                    frames.append(
                        ExecutionFrame(
                            robots=robots,
                            target_placement=placement,
                            target_robots=target,
                            state_index=idx,
                            segment_index=step,
                            controls=controls,
                        )
                    )
                key_indices.append(len(frames) - 1)
                continue

            min_steps = self.segment_steps
            max_steps = max(self.segment_steps * 3, 72)
            for step in range(1, max_steps + 1):
                robots, controls = controller_step(
                    robots,
                    target,
                    self.obstacles,
                    width=self.width,
                    height=self.height,
                    safe_distance=self.safe_distance,
                    obstacle_margin=self.obstacle_margin,
                    workspace_margin=self.workspace_robot_margin,
                )
                frames.append(
                    ExecutionFrame(
                        robots=robots,
                        target_placement=placement,
                        target_robots=target,
                        state_index=idx,
                        segment_index=step,
                        controls=controls,
                    )
                )
                if step >= min_steps and max_distance(robots, target) < 0.85:
                    break
            if max_distance(robots, target) >= 1e-6:
                robots = enforce_workspace_safety(
                    target,
                    self.width,
                    self.height,
                    self.workspace_robot_margin,
                )
                frames.append(
                    ExecutionFrame(
                        robots=robots,
                        target_placement=placement,
                        target_robots=target,
                        state_index=idx,
                        segment_index=step + 1,
                        controls=zero_controls,
                    )
                )
                key_indices.append(len(frames) - 1)
        return frames, key_indices

    def _build_paper_qp_execution(self) -> Tuple[List[ExecutionFrame], List[int]]:
        if self.paper_qp_controller is None:
            raise RuntimeError("paper_qp execution requested without a PaperQPController")

        frames: List[ExecutionFrame] = []
        key_indices: List[int] = []
        robots = tuple(self.placed_robots(self.placements[0]))  # type: ignore[assignment]
        zero_controls = ((0.0, 0.0), (0.0, 0.0), (0.0, 0.0))
        frames.append(
            ExecutionFrame(
                robots=robots,
                target_placement=self.placements[0],
                target_robots=robots,
                state_index=0,
                segment_index=0,
                controls=zero_controls,
                controller="paper_qp",
                qp_success=True,
                qp_delta1=0.0,
                qp_delta2=0.0,
                qp_status="initial",
                qp_max_constraint_violation=0.0,
            )
        )
        key_indices.append(0)

        max_steps = max(
            self.segment_steps,
            int(math.ceil(self.paper_qp_config.fixed_time_bound / self.paper_qp_config.time_step)),
        )
        for idx, placement in enumerate(self.placements[1:], start=1):
            last_step = 0
            source_placement = self.placements[idx - 1]
            goal_placements = self._paper_qp_transition_goal_placements(idx, placement)
            for goal_placement in goal_placements:
                final_target = tuple(self.placed_robots(goal_placement))  # type: ignore[assignment]
                transition_start = robots
                pieces = max(
                    1,
                    int(math.ceil(max_distance(transition_start, final_target) / self.paper_qp_config.max_subgoal_distance)),
                )
                for piece in range(1, pieces + 1):
                    tau = piece / pieces
                    if source_placement.robots is None and goal_placement.robots is None:
                        subgoal_placement = interpolate_placement(source_placement, goal_placement, tau)
                        target = tuple(self.placed_robots(subgoal_placement))  # type: ignore[assignment]
                    else:
                        target = tuple(
                            (
                                start[0] + (final[0] - start[0]) * tau,
                                start[1] + (final[1] - start[1]) * tau,
                            )
                            for start, final in zip(transition_start, final_target)
                        )
                        target = target  # type: ignore[assignment]
                    for step in range(1, max_steps + 1):
                        result = self.paper_qp_controller.step(robots, target, self.obstacles)
                        robots = result.robots
                        last_step += 1
                        frames.append(
                            ExecutionFrame(
                                robots=robots,
                                target_placement=placement,
                                target_robots=target,
                                state_index=idx,
                                segment_index=last_step,
                                controls=result.controls,
                                controller="paper_qp",
                                qp_success=result.success,
                                qp_delta1=result.delta1,
                                qp_delta2=result.delta2,
                                qp_status=result.status,
                                qp_max_constraint_violation=result.max_constraint_violation,
                            )
                        )
                        if max_distance(robots, target) <= self.paper_qp_config.target_tolerance:
                            break
                        if not result.success:
                            break
                    if not frames[-1].qp_success:
                        break
                if not frames[-1].qp_success:
                    break
                source_placement = goal_placement
            if last_step == 0:
                frames.append(
                    ExecutionFrame(
                        robots=robots,
                        target_placement=placement,
                        target_robots=final_target,
                        state_index=idx,
                        segment_index=0,
                        controls=zero_controls,
                        controller="paper_qp",
                        qp_success=False,
                        qp_status="no_step",
                        qp_max_constraint_violation=float("inf"),
                    )
                )
            key_indices.append(len(frames) - 1)
        return frames, key_indices

    def _paper_qp_transition_goal_placements(self, idx: int, final_placement: Placement) -> List[Placement]:
        if not self.paper_qp_config.use_certified_region_subgoals:
            return [final_placement]
        src_regions = set(self.states[idx - 1].region.split(" ∩ "))
        dst_regions = set(self.states[idx].region.split(" ∩ "))
        common_regions = sorted(src_regions.intersection(dst_regions))
        goals: List[Placement] = []
        for region_name in common_regions:
            candidate = self.region_allowed_placements.get(region_name, {}).get(final_placement.formation)
            if candidate is None:
                continue
            start_robots = tuple(self.placed_robots(self.placements[idx - 1]))  # type: ignore[assignment]
            candidate_robots = tuple(self.placed_robots(candidate))  # type: ignore[assignment]
            final_robots = tuple(self.placed_robots(final_placement))  # type: ignore[assignment]
            if max_distance(start_robots, candidate_robots) < 8.0 or max_distance(candidate_robots, final_robots) < 8.0:
                continue
            direct = max_distance(start_robots, final_robots)
            detour = max_distance(start_robots, candidate_robots) + max_distance(candidate_robots, final_robots)
            if direct > self.paper_qp_config.max_subgoal_distance and detour <= direct * 1.8:
                goals.append(candidate)
                break
        goals.append(final_placement)
        return goals

    def edge_phase(self, transition: WorkflowTransition) -> str:
        dst_state = self.state_by_id[transition.dst]
        src_state = self.state_by_id[transition.src]
        if dst_state.task_mode in {"delivered", "done"} or src_state.state_id.startswith("s_drop"):
            return "delivered"
        return "loaded"

    def state_phase(self, state: WorkflowState) -> str:
        return "loaded" if state.task_mode in {"empty", "loaded"} else "delivered"

    def validate(self) -> Dict[str, bool]:
        loaded_states = [s for s in self.states if s.task_mode == "loaded"]
        checks = {
            "generated_route_found": bool(self.case.get("checks", {}).get("route_found")),
            "loaded_states_use_triangle": all(s.formation == "triangle" for s in loaded_states),
            "return_states_use_line": all(s.formation == "line" for s in self.states if s.task_mode == "delivered"),
            "final_home_triangle": self.states[-1].formation == "triangle",
            "nonempty_execution": len(self.frames) > 0,
            "continuous_execution": self.max_frame_jump() < 22.0,
            "pairwise_safe_execution": self.min_pairwise_distance() >= self.safe_distance - 1e-6,
            "obstacle_safe_execution": self.min_obstacle_clearance() >= self.obstacle_margin - 1e-6,
            "workspace_safe_execution": self.min_workspace_clearance() >= self.workspace_robot_margin - 1e-6,
        }
        if self.control_mode == "paper_qp":
            checks.update(
                {
                    "paper_qp_solver_success": self.paper_qp_solver_success(),
                    "paper_qp_constraints_satisfied": self.max_qp_constraint_violation()
                    <= self.paper_qp_config.constraint_tolerance,
                    "paper_qp_delta2_nonnegative": self.min_qp_delta2() >= -1e-7,
                    "paper_qp_fxt_delta1_nonpositive": self.max_qp_delta1() <= 1e-6,
                    "paper_qp_targets_reached": self.max_key_frame_error() <= self.paper_qp_config.target_tolerance + 1e-6,
                }
            )
        return checks

    def max_frame_jump(self) -> float:
        if len(self.frames) < 2:
            return 0.0
        worst = 0.0
        for prev, cur in zip(self.frames, self.frames[1:]):
            for p, q in zip(prev.robots, cur.robots):
                worst = max(worst, math.sqrt(dist2(p, q)))
        return worst

    def min_pairwise_distance(self) -> float:
        best = float("inf")
        for frame in self.frames:
            for i, p in enumerate(frame.robots):
                for q in frame.robots[i + 1 :]:
                    best = min(best, math.sqrt(dist2(p, q)))
        return best

    def min_obstacle_clearance(self) -> float:
        best = float("inf")
        for frame in self.frames:
            for robot in frame.robots:
                best = min(best, signed_distance_to_polygons(robot, self.obstacles))
        return best

    def min_workspace_clearance(self) -> float:
        best = float("inf")
        for frame in self.frames:
            best = min(best, workspace_clearance(frame.robots, self.width, self.height))
        return best

    def paper_qp_solver_success(self) -> bool:
        return all(frame.qp_success is not False for frame in self.frames)

    def max_qp_delta1(self) -> float:
        values = [frame.qp_delta1 for frame in self.frames if frame.qp_delta1 is not None and frame.segment_index > 0]
        return max(values) if values else 0.0

    def min_qp_delta2(self) -> float:
        values = [frame.qp_delta2 for frame in self.frames if frame.qp_delta2 is not None and frame.segment_index > 0]
        return min(values) if values else 0.0

    def max_qp_constraint_violation(self) -> float:
        values = [frame.qp_max_constraint_violation for frame in self.frames if frame.segment_index > 0]
        return max(values) if values else 0.0

    def max_key_frame_error(self) -> float:
        if not self.key_frame_indices:
            return float("inf")
        worst = 0.0
        for frame_index, placement in zip(self.key_frame_indices, self.placements):
            frame = self.frames[frame_index]
            target = tuple(self.placed_robots(placement))  # type: ignore[assignment]
            worst = max(worst, max_distance(frame.robots, target))
        return worst


def enforce_pairwise_safety(robots: Sequence[Point2], min_distance: float = SAFE_DISTANCE) -> Tuple[Point2, Point2, Point2]:
    adjusted = [tuple(p) for p in robots]
    for _ in range(8):
        changed = False
        for i in range(len(adjusted)):
            for j in range(i + 1, len(adjusted)):
                p = adjusted[i]
                q = adjusted[j]
                dx = p[0] - q[0]
                dy = p[1] - q[1]
                distance = math.hypot(dx, dy)
                if distance >= min_distance:
                    continue
                changed = True
                if distance < 1e-6:
                    dx, dy, distance = 1.0, 0.0, 1.0
                correction = 0.5 * (min_distance - distance)
                ux = dx / distance
                uy = dy / distance
                adjusted[i] = (p[0] + ux * correction, p[1] + uy * correction)
                adjusted[j] = (q[0] - ux * correction, q[1] - uy * correction)
        if not changed:
            break
    return tuple(adjusted)  # type: ignore[return-value]


def enforce_obstacle_safety(
    robots: Sequence[Point2],
    obstacles: Sequence[Sequence[Point2]],
    margin: float = OBSTACLE_MARGIN,
) -> Tuple[Point2, Point2, Point2]:
    adjusted: List[Point2] = []
    for robot in robots:
        best_poly = min(obstacles, key=lambda poly: abs(signed_distance_to_polygon(robot, poly))) if obstacles else []
        if not best_poly:
            adjusted.append(tuple(robot))
            continue
        closest = closest_point_on_polygon(robot, best_poly)
        vx = robot[0] - closest[0]
        vy = robot[1] - closest[1]
        dist = math.hypot(vx, vy)
        signed = signed_distance_to_polygon(robot, best_poly)
        if signed < margin:
            if dist < 1e-6:
                cx, cy = polygon_centroid(best_poly)
                vx = robot[0] - cx
                vy = robot[1] - cy
                dist = math.hypot(vx, vy)
                if dist < 1e-6:
                    vx, vy, dist = 1.0, 0.0, 1.0
            ux = vx / dist
            uy = vy / dist
            adjusted.append((closest[0] + ux * margin, closest[1] + uy * margin))
        else:
            adjusted.append(tuple(robot))
    return tuple(adjusted)  # type: ignore[return-value]


def controller_step(
    robots: Sequence[Point2],
    target: Sequence[Point2],
    obstacles: Sequence[Sequence[Point2]],
    width: float = PIXEL_W,
    height: float = PIXEL_H,
    gain: float = 0.16,
    max_step: float = 18.0,
    safe_distance: float = SAFE_DISTANCE,
    obstacle_margin: float = OBSTACLE_MARGIN,
    workspace_margin: float = PLACEMENT_BOUNDARY_MARGIN,
) -> Tuple[Tuple[Point2, Point2, Point2], Tuple[Point2, Point2, Point2]]:
    raw_next: List[Point2] = []
    controls: List[Point2] = []
    for robot, goal in zip(robots, target):
        ux = gain * (goal[0] - robot[0])
        uy = gain * (goal[1] - robot[1])
        speed = math.hypot(ux, uy)
        if speed > max_step:
            scale = max_step / speed
            ux *= scale
            uy *= scale
        raw_next.append((robot[0] + ux, robot[1] + uy))

    safe_next = tuple(raw_next)  # type: ignore[assignment]
    for _ in range(4):
        safe_next = enforce_pairwise_safety(safe_next, min_distance=safe_distance + 0.25)
        safe_next = enforce_obstacle_safety(safe_next, obstacles, margin=obstacle_margin + 2.0)
        safe_next = enforce_workspace_safety(safe_next, width, height, workspace_margin)
    safe_next = enforce_pairwise_safety(safe_next, min_distance=safe_distance + 0.25)
    safe_next = enforce_workspace_safety(safe_next, width, height, workspace_margin)
    for robot, nxt in zip(robots, safe_next):
        controls.append((nxt[0] - robot[0], nxt[1] - robot[1]))
    return tuple(safe_next), tuple(controls)  # type: ignore[return-value]

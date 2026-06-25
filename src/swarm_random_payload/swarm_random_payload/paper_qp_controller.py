"""Paper-aligned CBF/FxT-CLF QP controller for payload transport demos."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Sequence, Tuple

try:
    import casadi as ca
except Exception:  # pragma: no cover - only required for paper_qp mode
    ca = None


Point2 = Tuple[float, float]


@dataclass(frozen=True)
class PaperQPConfig:
    """Parameters matching the continuous-layer QP in the paper."""

    time_step: float = 0.04
    fixed_time_bound: float = 4.0
    mu: float = 2.0
    u_max_mps: float = 5.0
    centroid_tolerance: float = 0.85
    formation_tolerance: float = 0.85
    target_tolerance: float = 1.0
    delta1_quadratic_weight: float = 1.0
    delta1_linear_weight: float = 0.0
    delta2_quadratic_weight: float = 0.1
    constraint_tolerance: float = 1e-3
    max_subgoal_distance: float = 1.0e9
    use_certified_region_subgoals: bool = False
    enforce_discrete_safety: bool = True

    @classmethod
    def from_case(cls, payload: dict[str, Any], map_scale: float) -> "PaperQPConfig":
        control = payload.get("control", {}) if isinstance(payload.get("control"), dict) else {}
        raw = control.get("paper_qp", {}) if isinstance(control.get("paper_qp"), dict) else {}
        u_max_mps = float(raw.get("uMaxMetersPerSecond", raw.get("u_max_mps", cls.u_max_mps)))
        if "uMaxPxPerSecond" in raw:
            u_max_mps = float(raw["uMaxPxPerSecond"]) * map_scale
        return cls(
            time_step=float(raw.get("timeStep", raw.get("time_step", cls.time_step))),
            fixed_time_bound=float(raw.get("fixedTimeBound", raw.get("fixed_time_bound", cls.fixed_time_bound))),
            mu=float(raw.get("mu", cls.mu)),
            u_max_mps=u_max_mps,
            centroid_tolerance=float(raw.get("centroidTolerance", raw.get("centroid_tolerance", cls.centroid_tolerance))),
            formation_tolerance=float(raw.get("formationTolerance", raw.get("formation_tolerance", cls.formation_tolerance))),
            target_tolerance=float(raw.get("targetTolerance", raw.get("target_tolerance", cls.target_tolerance))),
            delta1_quadratic_weight=float(
                raw.get("delta1QuadraticWeight", raw.get("delta1_quadratic_weight", cls.delta1_quadratic_weight))
            ),
            delta1_linear_weight=float(
                raw.get("delta1LinearWeight", raw.get("delta1_linear_weight", cls.delta1_linear_weight))
            ),
            delta2_quadratic_weight=float(
                raw.get("delta2QuadraticWeight", raw.get("delta2_quadratic_weight", cls.delta2_quadratic_weight))
            ),
            constraint_tolerance=float(raw.get("constraintTolerance", raw.get("constraint_tolerance", cls.constraint_tolerance))),
            max_subgoal_distance=float(raw.get("maxSubgoalDistance", raw.get("max_subgoal_distance", cls.max_subgoal_distance))),
            use_certified_region_subgoals=bool(
                raw.get(
                    "useCertifiedRegionSubgoals",
                    raw.get("use_certified_region_subgoals", cls.use_certified_region_subgoals),
                )
            ),
            enforce_discrete_safety=bool(
                raw.get("enforceDiscreteSafety", raw.get("enforce_discrete_safety", cls.enforce_discrete_safety))
            ),
        )


@dataclass(frozen=True)
class PaperQPStep:
    robots: Tuple[Point2, Point2, Point2]
    controls: Tuple[Point2, Point2, Point2]
    success: bool
    delta1: float
    delta2: float
    status: str
    max_constraint_violation: float = 0.0


def sub(a: Point2, b: Point2) -> Point2:
    return (a[0] - b[0], a[1] - b[1])


def dist2(a: Point2, b: Point2) -> float:
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return dx * dx + dy * dy


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
        candidate_d2 = dist2(p, candidate)
        if candidate_d2 < best_d2:
            best = candidate
            best_d2 = candidate_d2
    return best


def signed_distance_and_gradient(point: Point2, obstacles: Sequence[Sequence[Point2]], margin: float) -> tuple[float, Point2]:
    if not obstacles:
        return (1e6, (0.0, 0.0))

    best_poly = min(obstacles, key=lambda poly: abs(_signed_distance(point, poly)))
    closest = closest_point_on_polygon(point, best_poly)
    vx = point[0] - closest[0]
    vy = point[1] - closest[1]
    distance = math.hypot(vx, vy)
    inside = point_in_polygon(point, best_poly)
    if distance < 1e-9:
        cx = sum(p[0] for p in best_poly) / len(best_poly)
        cy = sum(p[1] for p in best_poly) / len(best_poly)
        vx = point[0] - cx
        vy = point[1] - cy
        distance = math.hypot(vx, vy)
    if distance < 1e-9:
        vx, vy, distance = 1.0, 0.0, 1.0

    sign = -1.0 if inside else 1.0
    signed = sign * distance
    # h_O >= 0 is the obstacle-free safe set. The gradient points toward
    # increasing signed clearance, i.e. away from the closest obstacle boundary.
    grad = (sign * vx / distance, sign * vy / distance)
    return (signed - margin, grad)


def _signed_distance(point: Point2, poly: Sequence[Point2]) -> float:
    closest = closest_point_on_polygon(point, poly)
    distance = math.sqrt(dist2(point, closest))
    return -distance if point_in_polygon(point, poly) else distance


class PaperQPController:
    """CasADi/IPOPT implementation of the paper's continuous QP layer."""

    def __init__(
        self,
        width: float,
        height: float,
        map_scale: float,
        safe_distance: float,
        obstacle_margin: float,
        workspace_margin: float,
        config: PaperQPConfig,
    ) -> None:
        if ca is None:
            raise RuntimeError("paper_qp mode requires casadi, matching the paper's IPOPT-based QP implementation")
        self.width = width
        self.height = height
        self.map_scale = map_scale
        self.safe_distance = safe_distance
        self.obstacle_margin = obstacle_margin
        self.workspace_margin = workspace_margin
        self.config = config
        self.u_max = config.u_max_mps
        self.width_m = width * map_scale
        self.height_m = height * map_scale
        self.safe_distance_m = safe_distance * map_scale
        self.obstacle_margin_m = obstacle_margin * map_scale
        self.workspace_margin_m = workspace_margin * map_scale
        self.centroid_tolerance_m = config.centroid_tolerance * map_scale
        self.formation_tolerance_m = config.formation_tolerance * map_scale
        self.alpha1 = config.mu * math.pi / (2.0 * config.fixed_time_bound)
        self.alpha2 = self.alpha1
        self.gamma1 = 1.0 + 1.0 / config.mu
        self.gamma2 = 1.0 - 1.0 / config.mu
        self.last_solution = [0.0] * 8
        self._build_solver()

    def _build_solver(self) -> None:
        z = ca.MX.sym("z", 8)
        p = ca.MX.sym("p", 21)
        u = z[:6]
        delta1 = z[6]
        delta2 = z[7]
        dt = self.config.time_step
        x = p[:6]
        target = p[6:12]
        obs_h = p[12:15]
        obs_grad = p[15:21]

        def robot(vec, idx):
            return vec[(2 * idx) : (2 * idx + 2)]

        centroid = (robot(x, 0) + robot(x, 1) + robot(x, 2)) / 3.0
        target_centroid = (robot(target, 0) + robot(target, 1) + robot(target, 2)) / 3.0
        centroid_input = (robot(u, 0) + robot(u, 1) + robot(u, 2)) / 3.0

        constraints = []
        lower = []
        upper = []

        for idx in range(3):
            ui = robot(u, idx)
            constraints.append(ca.dot(ui, ui))
            lower.append(-ca.inf)
            upper.append(self.u_max * self.u_max)

        def positive_part(value):
            return 0.5 * (value + ca.sqrt(value * value + 1e-10))

        h_w = ca.dot(centroid - target_centroid, centroid - target_centroid) - self.centroid_tolerance_m**2
        clf_rhs = (
            delta1 * h_w
            - self.alpha1 * positive_part(h_w) ** self.gamma1
            - self.alpha2 * positive_part(h_w) ** self.gamma2
        )
        constraints.append(2.0 * ca.dot(centroid - target_centroid, centroid_input) - clf_rhs)
        lower.append(-ca.inf)
        upper.append(0.0)

        for i in range(3):
            for j in range(i + 1, 3):
                x_ij = robot(x, i) - robot(x, j)
                f_ij = robot(target, i) - robot(target, j)
                u_ij = robot(u, i) - robot(u, j)
                h_f = ca.dot(x_ij - f_ij, x_ij - f_ij) - self.formation_tolerance_m**2
                rhs = (
                    delta1 * h_f
                    - self.alpha1 * positive_part(h_f) ** self.gamma1
                    - self.alpha2 * positive_part(h_f) ** self.gamma2
                )
                constraints.append(2.0 * ca.dot(x_ij - f_ij, u_ij) - rhs)
                lower.append(-ca.inf)
                upper.append(0.0)

        for i in range(3):
            for j in range(i + 1, 3):
                x_ij = robot(x, i) - robot(x, j)
                u_ij = robot(u, i) - robot(u, j)
                h_d = ca.dot(x_ij, x_ij) - self.safe_distance_m**2
                constraints.append(2.0 * ca.dot(x_ij, u_ij) + delta2 * h_d)
                lower.append(0.0)
                upper.append(ca.inf)
                if self.config.enforce_discrete_safety:
                    next_ij = x_ij + dt * u_ij
                    constraints.append(ca.dot(next_ij, next_ij) - self.safe_distance_m**2)
                    lower.append(0.0)
                    upper.append(ca.inf)

        for idx in range(3):
            ui = robot(u, idx)
            gx = obs_grad[2 * idx]
            gy = obs_grad[2 * idx + 1]
            constraints.append(gx * ui[0] + gy * ui[1] + delta2 * obs_h[idx])
            lower.append(0.0)
            upper.append(ca.inf)
            if self.config.enforce_discrete_safety:
                constraints.append(obs_h[idx] + dt * (gx * ui[0] + gy * ui[1]))
                lower.append(0.0)
                upper.append(ca.inf)

            xi = robot(x, idx)
            workspace_constraints = [
                (xi[0] - self.workspace_margin_m, ui[0]),
                (self.width_m - self.workspace_margin_m - xi[0], -ui[0]),
                (xi[1] - self.workspace_margin_m, ui[1]),
                (self.height_m - self.workspace_margin_m - xi[1], -ui[1]),
            ]
            for h_boundary, hdot_boundary in workspace_constraints:
                constraints.append(hdot_boundary + delta2 * h_boundary)
                lower.append(0.0)
                upper.append(ca.inf)
            if self.config.enforce_discrete_safety:
                next_x = xi[0] + dt * ui[0]
                next_y = xi[1] + dt * ui[1]
                next_workspace_constraints = [
                    next_x - self.workspace_margin_m,
                    self.width_m - self.workspace_margin_m - next_x,
                    next_y - self.workspace_margin_m,
                    self.height_m - self.workspace_margin_m - next_y,
                ]
                for h_next_boundary in next_workspace_constraints:
                    constraints.append(h_next_boundary)
                    lower.append(0.0)
                    upper.append(ca.inf)

        constraints.append(delta1)
        lower.append(-ca.inf)
        upper.append(0.0)
        constraints.append(delta2)
        lower.append(0.0)
        upper.append(ca.inf)

        objective = (
            ca.dot(u, u)
            + self.config.delta1_quadratic_weight * delta1 * delta1
            + self.config.delta1_linear_weight * delta1
            + self.config.delta2_quadratic_weight * delta2 * delta2
        )
        nlp = {"x": z, "p": p, "f": objective, "g": ca.vertcat(*constraints)}
        options = {
            "ipopt.print_level": 0,
            "ipopt.sb": "yes",
            "print_time": False,
            "ipopt.max_iter": 300,
            "ipopt.tol": 1e-6,
        }
        self.solver = ca.nlpsol("paper_swarm_qp", "ipopt", nlp, options)
        self.lower = lower
        self.upper = upper

    def step(
        self,
        robots: Sequence[Point2],
        target: Sequence[Point2],
        obstacles: Sequence[Sequence[Point2]],
    ) -> PaperQPStep:
        obstacle_data_px = [signed_distance_and_gradient(robot, obstacles, self.obstacle_margin) for robot in robots]
        obstacle_data = [(h * self.map_scale, grad) for h, grad in obstacle_data_px]
        params = []
        for point in robots:
            params.extend([point[0] * self.map_scale, point[1] * self.map_scale])
        for point in target:
            params.extend([point[0] * self.map_scale, point[1] * self.map_scale])
        params.extend(item[0] for item in obstacle_data)
        for _, grad in obstacle_data:
            params.extend([grad[0], grad[1]])

        initial_guess = self._initial_guess(robots, target)
        try:
            solution = self.solver(
                x0=initial_guess,
                p=params,
                lbg=self.lower,
                ubg=self.upper,
            )
            values = [float(v) for v in solution["x"].full().reshape((-1,))]
            self.last_solution = values
            stats = self.solver.stats()
            violation = self._max_constraint_violation(solution["g"])
            success = bool(stats.get("success", False)) or violation <= self.config.constraint_tolerance
            status = str(stats.get("return_status", "unknown"))
        except Exception as exc:
            values = [0.0] * 8
            success = False
            status = type(exc).__name__
            violation = float("inf")

        next_robots = []
        controls = []
        for idx, robot in enumerate(robots):
            ux = values[2 * idx]
            uy = values[2 * idx + 1]
            control = (ux * self.config.time_step / self.map_scale, uy * self.config.time_step / self.map_scale)
            controls.append(control)
            next_robots.append((robot[0] + control[0], robot[1] + control[1]))

        return PaperQPStep(
            robots=(next_robots[0], next_robots[1], next_robots[2]),
            controls=(controls[0], controls[1], controls[2]),
            success=success,
            delta1=values[6],
            delta2=values[7],
            status=status,
            max_constraint_violation=violation,
        )

    def _max_constraint_violation(self, raw_constraints: Any) -> float:
        values = [float(v) for v in raw_constraints.full().reshape((-1,))]
        worst = 0.0
        for value, low, high in zip(values, self.lower, self.upper):
            low_f = float(low)
            high_f = float(high)
            if math.isfinite(low_f) and value < low_f:
                worst = max(worst, low_f - value)
            if math.isfinite(high_f) and value > high_f:
                worst = max(worst, value - high_f)
        return worst

    def _initial_guess(self, robots: Sequence[Point2], target: Sequence[Point2]) -> list[float]:
        guess: list[float] = []
        horizon = max(self.config.time_step, self.config.fixed_time_bound)
        for robot, goal in zip(robots, target):
            ux = (goal[0] - robot[0]) * self.map_scale / horizon
            uy = (goal[1] - robot[1]) * self.map_scale / horizon
            speed = math.hypot(ux, uy)
            if speed > self.u_max:
                scale = self.u_max / speed
                ux *= scale
                uy *= scale
            guess.extend([ux, uy])
        guess.extend([-1.0, 1.0])
        return guess

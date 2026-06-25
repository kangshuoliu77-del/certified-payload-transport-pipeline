#!/usr/bin/env python3
"""Generate a random-seed IRIS payload-transport case.

The input is a task-only map exported from the browser designer. The script
samples mandatory, path-biased, narrow-passage, open-space, and random seed
points, calls Drake IRIS for each useful seed, verifies obstacle separation,
and then builds a certified symbolic route from the generated regions.
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import heapq
import json
import math
from pathlib import Path
import random
import sys
from typing import Any, Iterable

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from figures.drake_iris_server import compute_drake_iris  # noqa: E402

try:
    from shapely.geometry import MultiPolygon, Point, Polygon, box
    from shapely.ops import unary_union
except Exception as exc:  # pragma: no cover
    raise RuntimeError("generate_case.py requires shapely") from exc


Point2 = tuple[float, float]
Polygon2 = list[Point2]

PIXEL_W = 980.0
PIXEL_H = 650.0
INTERIOR_MAX_DETOUR_RATIO = 1.2
INTERIOR_MIN_CLEARANCE_GAIN = 24.0
DEFAULT_TASK_PLACEMENT_TOLERANCES = {
    "home": 47.0,
    "pick": 47.0,
    "drop": 47.0,
    "return": 90.0,
}

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


@dataclass(frozen=True)
class Placement:
    center: Point2
    theta: float
    scale: float
    formation: str
    s_min: float
    robots: tuple[Point2, Point2, Point2] | None = None


def obstacle_points(obs: dict[str, Any]) -> Polygon2:
    if obs.get("type") == "rect":
        x = float(obs["x"])
        y = float(obs["y"])
        w = float(obs["w"])
        h = float(obs["h"])
        x0, x1 = sorted((x, x + w))
        y0, y1 = sorted((y, y + h))
        return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    return [(float(p["x"]), float(p["y"])) for p in obs["points"]]


def clean_polygon(points: Polygon2) -> Polygon:
    poly = Polygon(points)
    if not poly.is_valid:
        poly = poly.buffer(0)
    return poly


def obstacle_geometry(obstacles: list[dict[str, Any]], margin: float) -> Polygon | MultiPolygon:
    pieces = []
    for obs in obstacles:
        poly = clean_polygon(obstacle_points(obs))
        if margin > 0.0:
            poly = poly.buffer(margin, join_style=2)
        pieces.append(poly)
    return unary_union(pieces) if pieces else Polygon()


def region_polygon(vertices: list[dict[str, float]] | list[list[float]] | Polygon2) -> Polygon:
    points: Polygon2 = []
    for item in vertices:
        if isinstance(item, dict):
            points.append((float(item["x"]), float(item["y"])))
        else:
            points.append((float(item[0]), float(item[1])))
    return clean_polygon(points)


def polygon_to_json(poly: Polygon) -> list[dict[str, float]]:
    if poly.is_empty:
        return []
    coords = list(poly.exterior.coords)[:-1]
    return [{"x": round(float(x), 6), "y": round(float(y), 6)} for x, y in coords]


def point_tuple(payload: dict[str, float]) -> Point2:
    return (float(payload["x"]), float(payload["y"]))


def task_points(task: dict[str, Any]) -> dict[str, Point2]:
    return {
        "home": point_tuple(task["regions"]["home"]["center"]),
        "pick": point_tuple(task["regions"]["pick"]["center"]),
        "drop": point_tuple(task["regions"]["drop"]["center"]),
        "return": point_tuple(task["returnPoint"]["point"]),
    }


def task_placement_tolerances(source: dict[str, Any]) -> dict[str, float]:
    raw = source.get("taskPlacementTolerance", {})
    if not isinstance(raw, dict):
        raw = {}
    return {
        name: float(raw.get(name, DEFAULT_TASK_PLACEMENT_TOLERANCES[name]))
        for name in DEFAULT_TASK_PLACEMENT_TOLERANCES
    }


def source_manual_seeds(source: dict[str, Any]) -> list[Point2]:
    seeds: list[Point2] = []
    for item in source.get("manualSeeds", []):
        if isinstance(item, dict):
            seeds.append(point_tuple(item))
    for region in source.get("regions", []):
        seed = region.get("seed") if isinstance(region, dict) else None
        if isinstance(seed, dict):
            seeds.append(point_tuple(seed))
    return seeds


def source_region_seed_ids(source: dict[str, Any]) -> dict[tuple[float, float], int]:
    result: dict[tuple[float, float], int] = {}
    for region in source.get("regions", []):
        if not isinstance(region, dict) or "id" not in region:
            continue
        seed = region.get("seed")
        if isinstance(seed, dict):
            point = point_tuple(seed)
            result[(round(point[0], 6), round(point[1], 6))] = int(region["id"])
    return result


def source_region_ids(source: dict[str, Any]) -> set[int]:
    result: set[int] = set()
    for region in source.get("regions", []):
        if isinstance(region, dict) and "id" in region:
            result.add(int(region["id"]))
    return result


def next_region_id(used: set[int]) -> int:
    candidate = 1
    while candidate in used:
        candidate += 1
    return candidate


def template_min_distance(template_name: str) -> float:
    robots = TEMPLATES[template_name]["robots"]
    best = float("inf")
    for i, p in enumerate(robots):
        for q in robots[i + 1 :]:
            best = min(best, math.hypot(p[0] - q[0], p[1] - q[1]))
    return best


def transform(point: Point2, center: Point2, theta: float, scale: float) -> Point2:
    c = math.cos(theta)
    s = math.sin(theta)
    x, y = point
    return (center[0] + scale * (c * x - s * y), center[1] + scale * (s * x + c * y))


def envelope_polygon(placement: Placement) -> Polygon:
    points = [
        transform(vertex, placement.center, placement.theta, placement.scale)
        for vertex in TEMPLATES[placement.formation]["envelope"]
    ]
    return clean_polygon(points)


def placed_robots(placement: Placement) -> Polygon2:
    if placement.robots is not None:
        return list(placement.robots)
    return [
        transform(vertex, placement.center, placement.theta, placement.scale)
        for vertex in TEMPLATES[placement.formation]["robots"]
    ]


def orientation_candidates(template_name: str) -> list[float]:
    if template_name == "line":
        return [2.0 * math.pi * i / 48.0 for i in range(48)]
    return [math.pi * i / 12.0 for i in range(12)]


def ordered_orientation_candidates(template_name: str, preferred_theta: float | None = None) -> list[float]:
    candidates = orientation_candidates(template_name)
    if preferred_theta is None:
        return candidates
    return sorted(candidates, key=lambda theta: angular_distance(theta, preferred_theta))


def angular_distance(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def scale_candidates(template_name: str, formation_scale: float, safe_distance: float) -> list[float]:
    s_min = safe_distance / template_min_distance(template_name)
    if s_min > formation_scale:
        return []
    values = [formation_scale]
    for i in range(1, 9):
        values.append(formation_scale - (formation_scale - s_min) * i / 8.0)
    values.append(s_min)
    return sorted(set(round(v, 6) for v in values), reverse=True)


def centered_placement(
    region: Polygon,
    center: Point2,
    formation: str,
    formation_scale: float,
    safe_distance: float,
    preferred_theta: float | None = None,
) -> Placement | None:
    s_min = safe_distance / template_min_distance(formation)
    for scale in scale_candidates(formation, formation_scale, safe_distance):
        for theta in ordered_orientation_candidates(formation, preferred_theta):
            placement = Placement(center=center, theta=theta, scale=scale, formation=formation, s_min=s_min)
            if region.covers(envelope_polygon(placement)):
                return placement
    return None


def any_placement(
    region: Polygon,
    formation: str,
    formation_scale: float,
    safe_distance: float,
) -> Placement | None:
    s_min = safe_distance / template_min_distance(formation)
    centroid = region.representative_point()
    xs, ys = region.exterior.xy
    min_x, max_x = max(0.0, min(xs)), min(PIXEL_W, max(xs))
    min_y, max_y = max(0.0, min(ys)), min(PIXEL_H, max(ys))
    raw_centers = [
        (float(centroid.x), float(centroid.y)),
        (float(region.centroid.x), float(region.centroid.y)),
        ((min_x + max_x) / 2.0, (min_y + max_y) / 2.0),
    ]
    centers = []
    for center in raw_centers:
        if region.covers(Point(*center)):
            centers.append(center)

    step = 36.0
    y = min_y + step / 2.0
    while y <= max_y + 1e-6:
        x = min_x + step / 2.0
        while x <= max_x + 1e-6:
            p = Point(x, y)
            if region.covers(p):
                centers.append((x, y))
            x += step
        y += step

    best: tuple[float, Placement] | None = None
    for scale in scale_candidates(formation, formation_scale, safe_distance):
        for center in centers:
            for theta in orientation_candidates(formation):
                placement = Placement(center=center, theta=theta, scale=scale, formation=formation, s_min=s_min)
                env = envelope_polygon(placement)
                if not region.covers(env):
                    continue
                clearance = env.distance(region.boundary)
                workspace_clearance = env.distance(box(0.0, 0.0, PIXEL_W, PIXEL_H).boundary)
                score = scale * 500.0 + clearance * 5.0 + workspace_clearance * 3.0
                if best is None or score > best[0]:
                    best = (score, placement)
    return best[1] if best else None


def point_segment_distance(point: Point2, start: Point2, end: Point2) -> float:
    vx = end[0] - start[0]
    vy = end[1] - start[1]
    length_sq = vx * vx + vy * vy
    if length_sq < 1e-9:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    tau = ((point[0] - start[0]) * vx + (point[1] - start[1]) * vy) / length_sq
    tau = min(1.0, max(0.0, tau))
    proj = (start[0] + tau * vx, start[1] + tau * vy)
    return math.hypot(point[0] - proj[0], point[1] - proj[1])


def directed_placement(
    region: Polygon,
    formation: str,
    formation_scale: float,
    safe_distance: float,
    start: Point2,
    end: Point2,
    previous_theta: float | None = None,
    next_theta: float | None = None,
) -> Placement | None:
    s_min = safe_distance / template_min_distance(formation)
    xs, ys = region.exterior.xy
    min_x, max_x = max(0.0, min(xs)), min(PIXEL_W, max(xs))
    min_y, max_y = max(0.0, min(ys)), min(PIXEL_H, max(ys))
    midpoint = ((start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0)
    raw_centers = [
        midpoint,
        (float(region.representative_point().x), float(region.representative_point().y)),
        (float(region.centroid.x), float(region.centroid.y)),
        ((min_x + max_x) / 2.0, (min_y + max_y) / 2.0),
    ]
    centers: list[Point2] = []
    for center in raw_centers:
        if region.covers(Point(*center)):
            centers.append(center)

    step = 24.0
    y = min_y + step / 2.0
    while y <= max_y + 1e-6:
        x = min_x + step / 2.0
        while x <= max_x + 1e-6:
            point = Point(x, y)
            if region.covers(point):
                centers.append((x, y))
            x += step
        y += step

    best: tuple[tuple[float, float, float, float, float], Placement] | None = None
    for scale in scale_candidates(formation, formation_scale, safe_distance):
        for center in centers:
            for theta in orientation_candidates(formation):
                placement = Placement(center=center, theta=theta, scale=scale, formation=formation, s_min=s_min)
                env = envelope_polygon(placement)
                if not region.covers(env):
                    continue
                segment_distance = point_segment_distance(center, start, end)
                midpoint_distance = math.hypot(center[0] - midpoint[0], center[1] - midpoint[1])
                clearance = env.distance(region.boundary)
                workspace_clearance = env.distance(box(0.0, 0.0, PIXEL_W, PIXEL_H).boundary)
                angle_cost = 0.0
                if previous_theta is not None:
                    angle_cost += angular_distance(theta, previous_theta)
                if next_theta is not None:
                    angle_cost += 0.5 * angular_distance(theta, next_theta)
                score = (
                    round(scale, 6),
                    -round(angle_cost, 6),
                    -round(segment_distance, 6),
                    -round(midpoint_distance, 6),
                    round(clearance, 6),
                    round(workspace_clearance, 6),
                )
                if best is None or score > best[0]:
                    best = (score, placement)
    return best[1] if best else None


def normalize(vector: Point2) -> Point2 | None:
    length = math.hypot(vector[0], vector[1])
    if length < 1e-9:
        return None
    return (vector[0] / length, vector[1] / length)


def point_along(center: Point2, axis: Point2, distance: float) -> Point2:
    return (center[0] + axis[0] * distance, center[1] + axis[1] * distance)


def workspace_point_clearance(point: Point2) -> float:
    return min(point[0], PIXEL_W - point[0], point[1], PIXEL_H - point[1])


def min_rotated_rect_side(poly: Polygon) -> float:
    if poly.is_empty:
        return 0.0
    rect = poly.minimum_rotated_rectangle
    coords = list(rect.exterior.coords)
    if len(coords) < 5:
        return 0.0
    lengths = [
        math.hypot(coords[i + 1][0] - coords[i][0], coords[i + 1][1] - coords[i][1])
        for i in range(4)
    ]
    return min(lengths)


def custom_placement(formation: str, robots: list[Point2], axis: Point2, formation_scale: float, safe_distance: float) -> Placement:
    center = (
        sum(point[0] for point in robots) / len(robots),
        sum(point[1] for point in robots) / len(robots),
    )
    return Placement(
        center=center,
        theta=math.atan2(axis[1], axis[0]),
        scale=formation_scale,
        formation=formation,
        s_min=safe_distance / template_min_distance(formation),
        robots=(robots[0], robots[1], robots[2]),
    )


def single_file_stage_robots(center: Point2, axis: Point2, safe_distance: float, stage: int) -> list[Point2]:
    step = safe_distance + 2.0
    return [point_along(center, axis, (stage - robot_idx) * step) for robot_idx in range(3)]


def single_file_passage(
    region_a: dict[str, Any],
    region_b: dict[str, Any],
    inter: Polygon,
    formation: str,
    formation_scale: float,
    safe_distance: float,
) -> dict[str, Any] | None:
    if formation != "line" or inter.is_empty:
        return None

    union = unary_union([region_a["_poly"], region_b["_poly"]])
    base_axis = normalize((
        region_b["seed"]["x"] - region_a["seed"]["x"],
        region_b["seed"]["y"] - region_a["seed"]["y"],
    )) or (1.0, 0.0)

    raw_centers = [
        (float(inter.representative_point().x), float(inter.representative_point().y)),
        (float(inter.centroid.x), float(inter.centroid.y)),
        (
            (float(inter.bounds[0]) + float(inter.bounds[2])) / 2.0,
            (float(inter.bounds[1]) + float(inter.bounds[3])) / 2.0,
        ),
    ]
    midpoint = (
        (region_a["seed"]["x"] + region_b["seed"]["x"]) / 2.0,
        (region_a["seed"]["y"] + region_b["seed"]["y"]) / 2.0,
    )
    xs, ys = inter.exterior.xy
    min_x, max_x = max(0.0, min(xs)), min(PIXEL_W, max(xs))
    min_y, max_y = max(0.0, min(ys)), min(PIXEL_H, max(ys))
    centers: list[Point2] = []
    for center in raw_centers:
        if inter.covers(Point(*center)):
            centers.append(center)
    step = max(28.0, safe_distance * 0.5)
    y = min_y + step / 2.0
    while y <= max_y + 1e-6:
        x = min_x + step / 2.0
        while x <= max_x + 1e-6:
            if inter.covers(Point(x, y)):
                centers.append((x, y))
            x += step
        y += step

    unique_centers: list[Point2] = []
    for center in centers:
        if not any(math.hypot(center[0] - seen[0], center[1] - seen[1]) < 2.0 for seen in unique_centers):
            unique_centers.append(center)
    centers = sorted(
        unique_centers,
        key=lambda center: (
            -Point(*center).distance(inter.boundary),
            math.hypot(center[0] - midpoint[0], center[1] - midpoint[1]),
        ),
    )[:32]

    axes: list[Point2] = []
    axis_candidates = [base_axis]
    for k in range(12):
        axis_candidates.append((math.cos(math.pi * k / 12.0), math.sin(math.pi * k / 12.0)))
    for axis in axis_candidates:
        if axis[0] * base_axis[0] + axis[1] * base_axis[1] < 0.0:
            axis = (-axis[0], -axis[1])
        if not any(abs(axis[0] - seen[0]) < 1e-6 and abs(axis[1] - seen[1]) < 1e-6 for seen in axes):
            axes.append(axis)

    best: tuple[tuple[float, float, float, float], list[Placement]] | None = None
    for center in centers:
        for axis in axes:
            stage_points = [single_file_stage_robots(center, axis, safe_distance, stage) for stage in range(3)]
            if not all(all(union.covers(Point(*robot)) for robot in robots) for robots in stage_points):
                continue
            if not region_a["_poly"].covers(Point(*stage_points[0][2])):
                continue
            if not region_b["_poly"].covers(Point(*stage_points[2][0])):
                continue
            workspace_clearance = min(workspace_point_clearance(robot) for robots in stage_points for robot in robots)
            if workspace_clearance < 4.0:
                continue
            alignment = axis[0] * base_axis[0] + axis[1] * base_axis[1]
            center_clearance = Point(*center).distance(inter.boundary)
            midpoint_distance = math.hypot(center[0] - midpoint[0], center[1] - midpoint[1])
            placements = [
                custom_placement(formation, robots, axis, formation_scale, safe_distance)
                for robots in stage_points
            ]
            score = (
                round(workspace_clearance, 6),
                round(center_clearance, 6),
                round(alignment, 6),
                -round(midpoint_distance, 6),
            )
            if best is None or score > best[0]:
                best = (score, placements)

    if best is None:
        return None
    placements = best[1]
    return {
        "kind": "single_file",
        "passage": {
            "order": [region_a["name"], region_b["name"]],
            "stages": [placement_json(placement) for placement in placements],
        },
        "placement": placement_json(placements[1]),
    }


def placement_json(placement: Placement) -> dict[str, Any]:
    payload = {
        "center": {"x": round(placement.center[0], 6), "y": round(placement.center[1], 6)},
        "theta": round(placement.theta, 9),
        "scale": round(placement.scale, 6),
        "formation": placement.formation,
        "s_min": round(placement.s_min, 6),
        "robots": [{"x": round(x, 6), "y": round(y, 6)} for x, y in placed_robots(placement)],
        "envelope": polygon_to_json(envelope_polygon(placement)),
    }
    if placement.robots is not None:
        payload["custom_robots"] = True
    return payload


def bridge_placement(bridge: dict[str, Any], formation: str) -> Placement:
    return Placement(
        center=point_tuple(bridge["placement"]["center"]),
        theta=float(bridge["placement"]["theta"]),
        scale=float(bridge["placement"]["scale"]),
        formation=formation,
        s_min=float(bridge["placement"]["s_min"]),
    )


def payload_placement(payload: dict[str, Any]) -> Placement:
    placement = payload["placement"]
    robots = None
    if placement.get("custom_robots"):
        raw_robots = [point_tuple(point) for point in placement["robots"]]
        robots = (raw_robots[0], raw_robots[1], raw_robots[2])
    return Placement(
        center=point_tuple(placement["center"]),
        theta=float(placement["theta"]),
        scale=float(placement["scale"]),
        formation=str(placement["formation"]),
        s_min=float(placement["s_min"]),
        robots=robots,
    )


def bridge_polygon(bridge: dict[str, Any]) -> Polygon:
    return Polygon([(float(point["x"]), float(point["y"])) for point in bridge["poly"]])


def passage_stage_placements(bridge: dict[str, Any], previous_region: str, next_region: str) -> list[Placement]:
    stages = bridge.get("passage", {}).get("stages", [])
    if not stages:
        return []
    order = bridge.get("passage", {}).get("order", bridge.get("regions", []))
    if order[:2] == [previous_region, next_region]:
        selected = stages
    elif order[:2] == [next_region, previous_region]:
        selected = list(reversed(stages))
    else:
        selected = stages
    placements: list[Placement] = []
    for item in selected:
        raw_robots = [point_tuple(point) for point in item["robots"]]
        robots = (raw_robots[0], raw_robots[1], raw_robots[2])
        placements.append(
            Placement(
                center=point_tuple(item["center"]),
                theta=float(item["theta"]),
                scale=float(item["scale"]),
                formation=str(item["formation"]),
                s_min=float(item["s_min"]),
                robots=robots,
            )
        )
    return placements


def should_insert_interior_state(
    region: Polygon,
    previous: Placement,
    interior: Placement,
    next_placement: Placement,
) -> bool:
    direct = math.hypot(
        next_placement.center[0] - previous.center[0],
        next_placement.center[1] - previous.center[1],
    )
    if direct < 1e-6:
        return False
    detour = (
        math.hypot(interior.center[0] - previous.center[0], interior.center[1] - previous.center[1])
        + math.hypot(next_placement.center[0] - interior.center[0], next_placement.center[1] - interior.center[1])
    )
    if detour / direct > INTERIOR_MAX_DETOUR_RATIO:
        return False

    previous_clearance = region.boundary.distance(Point(previous.center))
    interior_clearance = region.boundary.distance(Point(interior.center))
    next_clearance = region.boundary.distance(Point(next_placement.center))
    return interior_clearance - max(previous_clearance, next_clearance) >= INTERIOR_MIN_CLEARANCE_GAIN


def add_unique_seed(seeds: list[Point2], seed: Point2, free: Polygon | MultiPolygon, min_gap: float = 18.0) -> None:
    x = min(max(seed[0], 0.0), PIXEL_W)
    y = min(max(seed[1], 0.0), PIXEL_H)
    p = Point(x, y)
    if not free.covers(p):
        return
    if any(math.hypot(x - q[0], y - q[1]) < min_gap for q in seeds):
        return
    seeds.append((x, y))


def interpolate(a: Point2, b: Point2, tau: float) -> Point2:
    return (a[0] + (b[0] - a[0]) * tau, a[1] + (b[1] - a[1]) * tau)


def sample_seeds(
    points: dict[str, Point2],
    free: Polygon | MultiPolygon,
    obstacle_union: Polygon | MultiPolygon,
    budget: int,
    rng: random.Random,
) -> list[Point2]:
    seeds: list[Point2] = []
    for key in ("home", "pick", "drop", "return"):
        add_unique_seed(seeds, points[key], free, min_gap=1.0)
        if len(seeds) >= budget:
            return seeds

    task_order = [points["home"], points["pick"], points["drop"], points["return"], points["home"]]
    for a, b in zip(task_order, task_order[1:]):
        for i in range(1, 10):
            base = interpolate(a, b, i / 10.0)
            for _ in range(3):
                add_unique_seed(seeds, (base[0] + rng.uniform(-45, 45), base[1] + rng.uniform(-45, 45)), free)
                if len(seeds) >= budget:
                    return seeds

    for point in points.values():
        for radius in (45.0, 85.0, 130.0):
            for i in range(12):
                theta = 2.0 * math.pi * i / 12.0 + rng.uniform(-0.12, 0.12)
                add_unique_seed(seeds, (point[0] + radius * math.cos(theta), point[1] + radius * math.sin(theta)), free)
                if len(seeds) >= budget:
                    return seeds

    scored_grid: list[tuple[float, Point2]] = []
    for y in range(35, int(PIXEL_H), 38):
        for x in range(35, int(PIXEL_W), 38):
            p = Point(float(x), float(y))
            if not free.covers(p):
                continue
            d = p.distance(obstacle_union)
            boundary = p.distance(box(0.0, 0.0, PIXEL_W, PIXEL_H).boundary)
            narrow_score = -abs(d - 24.0) + rng.random() * 0.01
            open_score = d + 0.15 * boundary + rng.random() * 0.01
            scored_grid.append((narrow_score, (float(x), float(y))))
            scored_grid.append((open_score, (float(x), float(y))))

    for _, seed in sorted(scored_grid, reverse=True)[: budget // 2]:
        add_unique_seed(seeds, seed, free)
        if len(seeds) >= budget:
            return seeds

    attempts = 0
    while len(seeds) < budget and attempts < budget * 80:
        attempts += 1
        seed = (rng.uniform(0.0, PIXEL_W), rng.uniform(0.0, PIXEL_H))
        add_unique_seed(seeds, seed, free)
    return seeds


def duplicate_region(candidate: Polygon, regions: list[dict[str, Any]], iou_threshold: float = 0.94) -> bool:
    for region in regions:
        existing = region["_poly"]
        inter = candidate.intersection(existing).area
        union = candidate.union(existing).area
        if union > 1e-6 and inter / union >= iou_threshold:
            return True
    return False


def build_allowed(poly: Polygon, formation_scale: float, safe_distance: float) -> dict[str, Placement]:
    allowed: dict[str, Placement] = {}
    for formation in ("line", "triangle"):
        placement = any_placement(poly, formation, formation_scale, safe_distance)
        if placement is not None:
            allowed[formation] = placement
    return allowed


def task_region_candidates(
    regions: list[dict[str, Any]],
    point: Point2,
    formation: str,
    formation_scale: float,
    safe_distance: float,
    tolerance: float = 0.0,
) -> list[tuple[int, Placement]]:
    candidates = []
    preferred_theta = math.radians(30.0) if formation == "triangle" else None
    for idx, region in enumerate(regions):
        placement = centered_placement(region["_poly"], point, formation, formation_scale, safe_distance, preferred_theta)
        if placement is not None:
            candidates.append((idx, placement))
            continue
        if tolerance <= 0.0:
            continue
        best: tuple[float, Placement] | None = None
        for radius in (min(20.0, tolerance), min(45.0, tolerance), min(75.0, tolerance), tolerance):
            if radius <= 1e-6:
                continue
            for k in range(16):
                theta = 2.0 * math.pi * k / 16.0
                center = (point[0] + radius * math.cos(theta), point[1] + radius * math.sin(theta))
                if math.hypot(center[0] - point[0], center[1] - point[1]) > tolerance:
                    continue
                placement = centered_placement(region["_poly"], center, formation, formation_scale, safe_distance, preferred_theta)
                if placement is None:
                    continue
                offset = math.hypot(center[0] - point[0], center[1] - point[1])
                score = (offset, -placement.scale)
                if best is None or score < best[0]:
                    best = (score, placement)
        if best is not None:
            candidates.append((idx, best[1]))
    candidates.sort(
        key=lambda item: (
            math.hypot(item[1].center[0] - point[0], item[1].center[1] - point[1]),
            math.hypot(regions[item[0]]["seed"]["x"] - point[0], regions[item[0]]["seed"]["y"] - point[1]),
            -regions[item[0]]["_poly"].area,
        )
    )
    return candidates


def seed_offset(region: dict[str, Any], point: Point2) -> float:
    return math.hypot(region["seed"]["x"] - point[0], region["seed"]["y"] - point[1])


def distance(a: Point2, b: Point2) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def region_center(region: dict[str, Any]) -> Point2:
    poly = region.get("_poly")
    if poly is not None:
        point = poly.representative_point()
        return (float(point.x), float(point.y))
    seed = region["seed"]
    return (float(seed["x"]), float(seed["y"]))


def bridge_center(bridge: dict[str, Any]) -> Point2:
    point = bridge.get("centroid", {})
    return (float(point["x"]), float(point["y"]))


def graph_edge_cost(regions: list[dict[str, Any]], src: int, dst: int, bridge: dict[str, Any]) -> float:
    center = bridge_center(bridge)
    return max(
        distance(region_center(regions[src]), center)
        + distance(center, region_center(regions[dst])),
        1e-6,
    )


def region_path_cost(regions: list[dict[str, Any]], path: tuple[list[int], list[dict[str, Any]]]) -> float:
    nodes, bridges = path
    return sum(graph_edge_cost(regions, nodes[idx], nodes[idx + 1], bridge) for idx, bridge in enumerate(bridges))


def nearest_allowed_candidates(
    regions: list[dict[str, Any]],
    point: Point2,
    formation: str,
    limit: int = 8,
) -> list[tuple[int, Placement]]:
    candidates: list[tuple[float, int, Placement]] = []
    for idx, region in enumerate(regions):
        placement = region.get("_allowed", region.get("allowed", {})).get(formation)
        if placement is None:
            continue
        offset = math.hypot(placement.center[0] - point[0], placement.center[1] - point[1])
        candidates.append((offset, idx, placement))
    candidates.sort(key=lambda item: item[0])
    return [(idx, placement) for _, idx, placement in candidates[:limit]]


def bridge_graph(
    regions: list[dict[str, Any]],
    formation: str,
    formation_scale: float,
    safe_distance: float,
    enable_single_file_bridges: bool = False,
) -> tuple[dict[int, list[tuple[int, dict[str, Any]]]], dict[tuple[int, int], dict[str, Any]]]:
    graph: dict[int, list[tuple[int, dict[str, Any]]]] = {idx: [] for idx in range(len(regions))}
    bridges: dict[tuple[int, int], dict[str, Any]] = {}
    for i in range(len(regions)):
        for j in range(i + 1, len(regions)):
            inter = regions[i]["_poly"].intersection(regions[j]["_poly"])
            if inter.is_empty or inter.area < 40.0:
                continue
            if hasattr(inter, "geoms"):
                inter = max(inter.geoms, key=lambda g: g.area)
            if inter.area < 40.0:
                continue
            placement = any_placement(inter, formation, formation_scale, safe_distance)
            narrow_candidate = (
                enable_single_file_bridges
                and formation == "line"
                and (placement is None or min_rotated_rect_side(inter) < safe_distance * 1.1)
            )
            passage = None
            if narrow_candidate:
                passage = single_file_passage(regions[i], regions[j], inter, formation, formation_scale, safe_distance)
            narrow_bridge = passage is not None
            if placement is None and passage is None:
                continue
            bridge_placement_payload = passage["placement"] if narrow_bridge else placement_json(placement)  # type: ignore[arg-type]
            bridge = {
                "name": f"b{regions[i]['id']}_{regions[j]['id']}_{formation}",
                "regions": [regions[i]["name"], regions[j]["name"]],
                "formation": formation,
                "kind": "single_file" if narrow_bridge else "formation",
                "poly": polygon_to_json(inter),
                "centroid": {
                    "x": round(float(inter.representative_point().x), 6),
                    "y": round(float(inter.representative_point().y), 6),
                },
                "placement": bridge_placement_payload,
            }
            if narrow_bridge:
                bridge["passage"] = passage["passage"]
            bridges[(i, j)] = bridge
            bridges[(j, i)] = bridge
            graph[i].append((j, bridge))
            graph[j].append((i, bridge))
    return graph, bridges


def find_region_path(
    regions: list[dict[str, Any]],
    graph: dict[int, list[tuple[int, dict[str, Any]]]],
    starts: Iterable[int],
    goals: set[int],
) -> tuple[list[int], list[dict[str, Any]]] | None:
    queue: list[tuple[float, int, int]] = []
    parent: dict[int, tuple[int, dict[str, Any]] | None] = {}
    best_cost: dict[int, float] = {}
    tie = 0
    for start in starts:
        heapq.heappush(queue, (0.0, tie, start))
        tie += 1
        parent[start] = None
        best_cost[start] = 0.0
    goal = None
    while queue:
        cost, _, node = heapq.heappop(queue)
        if cost > best_cost.get(node, float("inf")) + 1e-9:
            continue
        if node in goals:
            goal = node
            break
        for nxt, bridge in graph[node]:
            next_cost = cost + graph_edge_cost(regions, node, nxt, bridge)
            if next_cost >= best_cost.get(nxt, float("inf")) - 1e-9:
                continue
            best_cost[nxt] = next_cost
            parent[nxt] = (node, bridge)
            heapq.heappush(queue, (next_cost, tie, nxt))
            tie += 1
    if goal is None:
        return None
    nodes = [goal]
    bridges = []
    while parent[nodes[-1]] is not None:
        prev, bridge = parent[nodes[-1]]  # type: ignore[misc]
        bridges.append(bridge)
        nodes.append(prev)
    nodes.reverse()
    bridges.reverse()
    return nodes, bridges


def find_region_path_via(
    regions: list[dict[str, Any]],
    graph: dict[int, list[tuple[int, dict[str, Any]]]],
    starts: Iterable[int],
    goals: set[int],
    via: list[int],
) -> tuple[list[int], list[dict[str, Any]]] | None:
    current_starts = list(starts)
    all_nodes: list[int] = []
    all_bridges: list[dict[str, Any]] = []
    for waypoint in via:
        segment = find_region_path(regions, graph, current_starts, {waypoint})
        if segment is None:
            return None
        nodes, bridges = segment
        if all_nodes:
            all_nodes.extend(nodes[1:])
        else:
            all_nodes.extend(nodes)
        all_bridges.extend(bridges)
        current_starts = [waypoint]

    segment = find_region_path(regions, graph, current_starts, goals)
    if segment is None:
        return None
    nodes, bridges = segment
    if all_nodes:
        all_nodes.extend(nodes[1:])
    else:
        all_nodes.extend(nodes)
    all_bridges.extend(bridges)
    return all_nodes, all_bridges


def resolve_route_hints(regions: list[dict[str, Any]], raw_hints: dict[str, Any]) -> dict[str, list[int]]:
    hints: dict[str, list[int]] = {}
    by_name = {region["name"]: idx for idx, region in enumerate(regions)}
    for leg, raw_items in raw_hints.items():
        if not isinstance(raw_items, list):
            raw_items = [raw_items]
        indices: list[int] = []
        for item in raw_items:
            idx: int | None = None
            if isinstance(item, str):
                idx = by_name.get(item)
            elif isinstance(item, int):
                for region_idx, region in enumerate(regions):
                    if region["id"] == item:
                        idx = region_idx
                        break
            elif isinstance(item, dict) and "x" in item and "y" in item:
                point = Point(float(item["x"]), float(item["y"]))
                containing = [
                    (
                        math.hypot(
                            regions[region_idx]["seed"]["x"] - point.x,
                            regions[region_idx]["seed"]["y"] - point.y,
                        ),
                        region["_poly"].representative_point().distance(point),
                        region_idx,
                    )
                    for region_idx, region in enumerate(regions)
                    if region["_poly"].covers(point)
                ]
                if containing:
                    idx = min(containing)[2]
                else:
                    idx = min(
                        range(len(regions)),
                        key=lambda region_idx: math.hypot(
                            regions[region_idx]["seed"]["x"] - point.x,
                            regions[region_idx]["seed"]["y"] - point.y,
                        ),
                    )
            if idx is not None and idx not in indices:
                indices.append(idx)
        hints[leg] = indices
    return hints


def state_payload(
    state_id: str,
    paper_state: str,
    region: str,
    formation: str,
    task_mode: str,
    placement: Placement,
    task_label: str | None = None,
    event: str | None = None,
) -> dict[str, Any]:
    return {
        "state_id": state_id,
        "paper_state": paper_state,
        "region": region,
        "formation": formation,
        "task_mode": task_mode,
        "point": {"x": round(placement.center[0], 6), "y": round(placement.center[1], 6)},
        "task_label": task_label,
        "event": event,
        "placement": placement_json(placement),
    }


def assemble_route(
    regions: list[dict[str, Any]],
    points: dict[str, Point2],
    formation_scale: float,
    safe_distance: float,
    route_hints: dict[str, list[int]] | None = None,
    insert_interior_states: bool = False,
    enable_single_file_bridges: bool = False,
    placement_tolerances: dict[str, float] | None = None,
    allow_return_repair: bool = True,
) -> dict[str, Any] | None:
    route_hints = route_hints or {}
    placement_tolerances = placement_tolerances or DEFAULT_TASK_PLACEMENT_TOLERANCES
    triangle_graph, triangle_bridges = bridge_graph(regions, "triangle", formation_scale, safe_distance)
    line_graph, line_bridges = bridge_graph(
        regions,
        "line",
        formation_scale,
        safe_distance,
        enable_single_file_bridges=enable_single_file_bridges,
    )

    home_tri = task_region_candidates(
        regions, points["home"], "triangle", formation_scale, safe_distance, tolerance=placement_tolerances["home"]
    )
    pick_tri = task_region_candidates(
        regions, points["pick"], "triangle", formation_scale, safe_distance, tolerance=placement_tolerances["pick"]
    )
    pick_line = task_region_candidates(
        regions, points["pick"], "line", formation_scale, safe_distance, tolerance=placement_tolerances["pick"]
    )
    drop_tri = task_region_candidates(
        regions, points["drop"], "triangle", formation_scale, safe_distance, tolerance=placement_tolerances["drop"]
    )
    drop_line = task_region_candidates(
        regions, points["drop"], "line", formation_scale, safe_distance, tolerance=placement_tolerances["drop"]
    )
    return_line = task_region_candidates(
        regions, points["return"], "line", formation_scale, safe_distance, tolerance=placement_tolerances["return"]
    )
    home_line = task_region_candidates(
        regions, points["home"], "line", formation_scale, safe_distance, tolerance=placement_tolerances["home"]
    )

    return_repaired = False
    if not return_line and allow_return_repair:
        return_line = nearest_allowed_candidates(regions, points["return"], "line")
        return_repaired = bool(return_line)

    if not all((home_tri, home_line, pick_line, pick_tri, drop_tri, drop_line, return_line)):
        return None

    def placement_offset(placement: Placement, point: Point2) -> float:
        return math.hypot(placement.center[0] - point[0], placement.center[1] - point[1])

    def by_region(candidates: list[tuple[int, Placement]]) -> dict[int, Placement]:
        result: dict[int, Placement] = {}
        for idx, placement in candidates:
            result.setdefault(idx, placement)
        return result

    pick_tri_by_region = by_region(pick_tri)
    drop_line_by_region = by_region(drop_line)
    home_line_by_region = by_region(home_line)

    triangle_plan = None
    for home_i, home_p in home_tri:
        home_line_p = home_line_by_region.get(home_i)
        if home_line_p is None:
            continue
        for pick_line_i, pick_line_p in pick_line:
            pick_p = pick_tri_by_region.get(pick_line_i)
            if pick_p is None:
                continue
            path_home_pick = find_region_path_via(regions, line_graph, [home_i], {pick_line_i}, route_hints.get("home_to_pick", []))
            if path_home_pick is None:
                continue
            for drop_i, drop_p in drop_tri:
                path_pick_drop = find_region_path_via(regions, triangle_graph, [pick_line_i], {drop_i}, route_hints.get("pick_to_drop", []))
                if path_pick_drop is None:
                    continue
                task_offset = (
                    placement_offset(home_p, points["home"])
                    + placement_offset(home_line_p, points["home"])
                    + placement_offset(pick_line_p, points["pick"])
                    + placement_offset(pick_p, points["pick"])
                    + placement_offset(drop_p, points["drop"])
                )
                task_seed_offset = (
                    seed_offset(regions[home_i], points["home"])
                    + seed_offset(regions[pick_line_i], points["pick"])
                    + seed_offset(regions[drop_i], points["drop"])
                )
                path_cost = region_path_cost(regions, path_home_pick) + region_path_cost(regions, path_pick_drop)
                path_len = len(path_home_pick[0]) + len(path_pick_drop[0])
                score = (round(task_offset, 6), round(path_cost, 6), path_len, round(task_seed_offset, 6))
                if triangle_plan is None or score < triangle_plan[0]:
                    triangle_plan = (
                        score,
                        home_i,
                        home_p,
                        home_line_p,
                        pick_line_i,
                        pick_line_p,
                        pick_p,
                        drop_i,
                        drop_p,
                        path_home_pick,
                        path_pick_drop,
                    )

    line_plan = None
    drop_line_candidates = (
        [(triangle_plan[7], drop_line_by_region[triangle_plan[7]])]
        if triangle_plan is not None and triangle_plan[7] in drop_line_by_region
        else drop_line
    )
    home_line_candidates = (
        [(triangle_plan[1], triangle_plan[3])]
        if triangle_plan is not None
        else home_line
    )
    for drop_line_i, drop_line_p in drop_line_candidates:
        for return_i, return_p in return_line:
            path_drop_return = find_region_path_via(regions, line_graph, [drop_line_i], {return_i}, route_hints.get("drop_to_return", []))
            if path_drop_return is None:
                continue
            for home_line_i, home_line_p in home_line_candidates:
                path_return_home = find_region_path_via(regions, line_graph, [return_i], {home_line_i}, route_hints.get("return_to_home", []))
                if path_return_home is None:
                    continue
                path_len = len(path_drop_return[0]) + len(path_return_home[0])
                path_cost = region_path_cost(regions, path_drop_return) + region_path_cost(regions, path_return_home)
                switch_penalty = 0
                if triangle_plan is not None:
                    switch_penalty += 2 if drop_line_i != triangle_plan[7] else 0
                    switch_penalty += 2 if home_line_i != triangle_plan[1] else 0
                return_offset = placement_offset(return_p, points["return"])
                task_offset = placement_offset(drop_line_p, points["drop"]) + placement_offset(home_line_p, points["home"])
                route_seed_offset = (
                    seed_offset(regions[return_i], points["return"])
                    + seed_offset(regions[drop_line_i], points["drop"])
                    + seed_offset(regions[home_line_i], points["home"])
                )
                score = (
                    round(return_offset, 6),
                    round(task_offset, 6),
                    round(path_cost, 6),
                    path_len + switch_penalty,
                    round(route_seed_offset, 6),
                )
                if line_plan is None or score < line_plan[0]:
                    line_plan = (
                        score,
                        drop_line_i,
                        drop_line_p,
                        return_i,
                        return_p,
                        home_line_i,
                        home_line_p,
                        path_drop_return,
                        path_return_home,
                        return_offset,
                    )

    if triangle_plan is None or line_plan is None:
        return None

    (
        _,
        home_i,
        home_p,
        home_line_p,
        pick_line_i,
        pick_line_p,
        pick_p,
        drop_i,
        drop_p,
        path_home_pick,
        path_pick_drop,
    ) = triangle_plan
    (
        _,
        drop_line_i,
        drop_line_p,
        return_i,
        return_p,
        home_line_i,
        home_line_p,
        path_drop_return,
        path_return_home,
        return_offset,
    ) = line_plan

    states: list[dict[str, Any]] = []
    transitions: list[dict[str, str]] = []
    selected_bridges: dict[str, dict[str, Any]] = {}

    def add_state(payload: dict[str, Any]) -> str:
        if states and states[-1]["state_id"] == payload["state_id"]:
            return payload["state_id"]
        states.append(payload)
        if len(states) > 1:
            transitions.append(
                {
                    "src": states[-2]["state_id"],
                    "dst": states[-1]["state_id"],
                    "certificate": payload["region"],
                    "meaning": payload.get("event") or "certified transition",
                }
            )
        return payload["state_id"]

    add_state(state_payload("s_home_start", f"({regions[home_i]['name']}, triangle)", regions[home_i]["name"], "triangle", "empty", home_p, "home"))

    def add_bridge_states(
        path: tuple[list[int], list[dict[str, Any]]],
        mode: str,
        formation: str,
        prefix: str,
        final_placement: Placement,
        include_interiors: bool = False,
    ) -> None:
        path_nodes, path_bridges = path
        for idx, bridge in enumerate(path_bridges, start=1):
            prev_region = regions[path_nodes[idx - 1]]["name"]
            next_region = regions[path_nodes[idx]]["name"]
            region_label = f"{prev_region} ∩ {next_region}"
            if bridge.get("kind") == "single_file":
                selected_bridge = dict(bridge)
                selected_bridge["mode"] = mode
                selected_bridges[bridge["name"]] = selected_bridge
                stages = passage_stage_placements(bridge, prev_region, next_region)
                for stage_idx, placement in enumerate(stages, start=1):
                    add_state(
                        state_payload(
                            f"{prefix}_{idx}_{stage_idx}_{bridge['name']}",
                            f"({region_label}, {formation}, single-file {stage_idx}/3)",
                            region_label,
                            formation,
                            mode,
                            placement,
                        )
                    )
                continue
            previous_placement = payload_placement(states[-1])
            next_anchor = (
                bridge_placement(path_bridges[idx], formation)
                if idx < len(path_bridges)
                else final_placement
            )
            placement = directed_placement(
                bridge_polygon(bridge),
                formation,
                formation_scale,
                safe_distance,
                previous_placement.center,
                next_anchor.center,
                previous_theta=previous_placement.theta,
                next_theta=next_anchor.theta,
            ) or bridge_placement(bridge, formation)
            selected_bridge = dict(bridge)
            selected_bridge["mode"] = mode
            selected_bridge["placement"] = placement_json(placement)
            selected_bridges[bridge["name"]] = selected_bridge
            add_state(
                state_payload(
                    f"{prefix}_{idx}_{bridge['name']}",
                    f"({region_label}, {formation})",
                    region_label,
                    formation,
                    mode,
                    placement,
                )
            )
            arrival_region_idx = path_nodes[idx]
            if include_interiors and idx < len(path_bridges):
                interior = regions[arrival_region_idx].get("_allowed", {}).get(formation)
                if interior is None:
                    continue
                next_placement = bridge_placement(path_bridges[idx], formation)
                if not should_insert_interior_state(
                    regions[arrival_region_idx]["_poly"],
                    placement,
                    interior,
                    next_placement,
                ):
                    continue
                add_state(
                    state_payload(
                        f"{prefix}_{idx}_p{regions[arrival_region_idx]['id']}_interior",
                        f"({regions[arrival_region_idx]['name']}, {formation})",
                        regions[arrival_region_idx]["name"],
                        formation,
                        mode,
                        interior,
                    )
                )

    add_state(
        state_payload(
            "s_home_start_line",
            f"({regions[home_i]['name']}, line)",
            regions[home_i]["name"],
            "line",
            "empty",
            home_line_p,
            "home",
            "switch to line before outbound",
        )
    )
    add_bridge_states(path_home_pick, "empty", "line", "b_home_pick", pick_line_p)
    add_state(
        state_payload(
            "s_pick_line",
            f"({regions[pick_line_i]['name']}, line)",
            regions[pick_line_i]["name"],
            "line",
            "empty",
            pick_line_p,
            "pick",
            "arrive at pick in line",
        )
    )
    add_state(
        state_payload(
            "s_pick",
            f"({regions[pick_line_i]['name']}, triangle)",
            regions[pick_line_i]["name"],
            "triangle",
            "empty",
            pick_p,
            "pick",
            "switch to triangle before payload load",
        )
    )
    add_bridge_states(path_pick_drop, "loaded", "triangle", "b_pick_drop", drop_p)
    add_state(state_payload("s_drop", f"({regions[drop_i]['name']}, triangle)", regions[drop_i]["name"], "triangle", "loaded", drop_p, "drop", "drop payload"))
    if drop_line_i != drop_i:
        add_state(
            state_payload(
                "s_drop_line",
                f"({regions[drop_line_i]['name']}, line)",
                regions[drop_line_i]["name"],
                "line",
                "delivered",
                drop_line_p,
                "drop",
                "switch to line after payload release",
            )
        )
    add_bridge_states(path_drop_return, "delivered", "line", "b_drop_return", return_p, include_interiors=insert_interior_states)
    return_event = None
    if return_repaired:
        return_event = f"nearest certified RETURN placement, offset={return_offset:.1f}px"
    elif return_offset > 1.0:
        return_event = f"certified RETURN-region placement, offset={return_offset:.1f}px"
    add_state(
        state_payload(
            "s_return",
            f"({regions[return_i]['name']}, line)",
            regions[return_i]["name"],
            "line",
            "delivered",
            return_p,
            "return",
            return_event,
        )
    )
    add_bridge_states(path_return_home, "delivered", "line", "b_return_home", home_line_p, include_interiors=insert_interior_states)
    if home_line_i != home_i:
        add_state(state_payload("s_home_line", f"({regions[home_line_i]['name']}, line)", regions[home_line_i]["name"], "line", "delivered", home_line_p, "home"))
    add_state(state_payload("s_home_final", f"({regions[home_i]['name']}, triangle)", regions[home_i]["name"], "triangle", "done", home_p, "home", "switch back to triangle"))

    return {
        "states": states,
        "transitions": transitions,
        "bridges": list(selected_bridges.values()),
        "route_region_paths": {
            "home_to_pick": [regions[i]["name"] for i in path_home_pick[0]],
            "pick_to_drop": [regions[i]["name"] for i in path_pick_drop[0]],
            "drop_to_return": [regions[i]["name"] for i in path_drop_return[0]],
            "return_to_home": [regions[i]["name"] for i in path_return_home[0]],
        },
        "bridge_counts": {
            "triangle": len({b["name"] for b in triangle_bridges.values()}),
            "line": len({b["name"] for b in line_bridges.values()}),
        },
        "return_repaired": return_repaired,
        "return_offset_px": return_offset,
        "return_exact_center": return_offset <= 1.0,
    }


def generate(args: argparse.Namespace) -> dict[str, Any]:
    source = json.loads(Path(args.input).read_text(encoding="utf-8"))
    obstacles = source["obstacles"]
    safety_margin = float(source.get("safetyMargin", 8.0))
    obstacle_margin = float(source.get("obstacleMargin", max(16.0, safety_margin)))
    formation_scale = float(source.get("formationScale", 58.0))
    safe_distance = float(source.get("safeDistance", 54.0))
    source_regions_only = bool(
        getattr(args, "source_regions_only", False)
        or source.get("sourceRegionsOnly", False)
    )
    insert_interior_states = bool(source.get("insertInteriorStates", False))
    enable_single_file_bridges = bool(source.get("enableSingleFileBridges", False))

    inflated_obstacles = obstacle_geometry(obstacles, obstacle_margin)
    free = box(0.0, 0.0, PIXEL_W, PIXEL_H).difference(inflated_obstacles)
    points = task_points(source["task"])
    placement_tolerances = task_placement_tolerances(source)
    allow_return_repair = bool(source.get("allowReturnRepair", True))
    rng = random.Random(args.random_seed)
    seeds: list[Point2] = []
    manual_seeds = source_manual_seeds(source)
    source_seed_ids = source_region_seed_ids(source)
    reserved_region_ids = source_region_ids(source)
    used_region_ids: set[int] = set()
    for seed in manual_seeds:
        add_unique_seed(seeds, seed, free, min_gap=1.0)
    if not source_regions_only:
        for seed in sample_seeds(points, free, inflated_obstacles, args.seed_budget, rng):
            add_unique_seed(seeds, seed, free, min_gap=1.0)
    seeds = seeds[: args.seed_budget]

    regions: list[dict[str, Any]] = []
    route = None

    def try_accept_region(seed_index: int, seed: Point2) -> None:
        try:
            iris = compute_drake_iris(
                {
                    "width": PIXEL_W,
                    "height": PIXEL_H,
                    "obstacles": obstacles,
                    "seed": {"x": seed[0], "y": seed[1]},
                    "configuration_space_margin": obstacle_margin,
                    "iteration_limit": args.iteration_limit,
                    "termination_threshold": 1e-3,
                    "relative_termination_threshold": 1e-3,
                    "random_seed": args.random_seed,
                }
            )
        except Exception as exc:
            print(f"[skip] seed {seed_index}: IRIS failed: {exc}", file=sys.stderr)
            return
        poly = region_polygon(iris["region"]["vertices"])
        if poly.area < args.min_region_area:
            return
        overlap_area = poly.intersection(inflated_obstacles).area if poly.intersects(inflated_obstacles) else 0.0
        if overlap_area > args.obstacle_overlap_tolerance:
            print(f"[skip] seed {seed_index}: region intersects inflated obstacle", file=sys.stderr)
            return
        if duplicate_region(poly, regions):
            return
        allowed = build_allowed(poly, formation_scale, safe_distance)
        if not allowed:
            return
        seed_key = (round(seed[0], 6), round(seed[1], 6))
        region_id = source_seed_ids.get(seed_key)
        if region_id is None or region_id in used_region_ids:
            region_id = next_region_id(used_region_ids | reserved_region_ids)
        used_region_ids.add(region_id)
        region = {
            "id": region_id,
            "name": f"P{region_id}",
            "seed": {"x": round(seed[0], 6), "y": round(seed[1], 6)},
            "poly": polygon_to_json(poly),
            "area": round(float(poly.area), 6),
            "allowed": {name: placement_json(placement) for name, placement in allowed.items()},
            "_allowed": allowed,
            "drake": {
                "ellipse": iris["region"]["max_volume_inscribed_ellipsoid"],
                "chebyshev_center": iris["region"]["chebyshev_center"],
                "options": iris["options"],
            },
            "_poly": poly,
        }
        regions.append(region)
        print(
            f"[accept] P{region_id}: area={region['area']:.1f}, allowed={','.join(sorted(allowed))}",
            file=sys.stderr,
        )

    seed_index = 0
    for seed_index, seed in enumerate(seeds, start=1):
        try_accept_region(seed_index, seed)
        if len(regions) >= args.max_regions:
            break

    route_hints = resolve_route_hints(regions, source.get("routeHints", {}))
    if len(regions) < args.min_regions:
        raise RuntimeError(f"accepted only {len(regions)} IRIS regions; need at least {args.min_regions}")
    if route is None:
        route = assemble_route(
            regions,
            points,
            formation_scale,
            safe_distance,
            route_hints,
            insert_interior_states=insert_interior_states,
            enable_single_file_bridges=enable_single_file_bridges,
            placement_tolerances=placement_tolerances,
            allow_return_repair=allow_return_repair,
        )
    if route is None:
        raise RuntimeError(f"failed to synthesize a connected route from {len(regions)} IRIS regions")

    clean_regions = []
    for region in regions:
        item = dict(region)
        item.pop("_poly", None)
        item.pop("_allowed", None)
        clean_regions.append(item)

    case = {
        "schema": "random_seed_payload_case_v1",
        "width": PIXEL_W,
        "height": PIXEL_H,
        "source_map": str(Path(args.input).resolve()),
        "sampling": {
            "random_seed": args.random_seed,
            "seed_budget": args.seed_budget,
            "seed_count": len(seeds),
            "region_count": len(clean_regions),
            "strategy": [
                "source region seeds only",
            ] if source_regions_only else [
                "mandatory task seeds",
                "path-biased jitter seeds",
                "rings around task points",
                "narrow-passage grid seeds",
                "open-space grid seeds",
                "uniform random seeds",
            ],
            "insert_interior_states": insert_interior_states,
            "enable_single_file_bridges": enable_single_file_bridges,
            "task_placement_tolerances": placement_tolerances,
            "allow_return_repair": allow_return_repair,
        },
        "constants": {
            "formationScale": formation_scale,
            "safeDistance": safe_distance,
            "safetyMargin": safety_margin,
            "obstacleMargin": obstacle_margin,
            "placementBoundaryMargin": 4.0,
        },
        "obstacles": obstacles,
        "task": source["task"],
        "regions": clean_regions,
        "bridges": route["bridges"],
        "states": route["states"],
        "transitions": route["transitions"],
        "route_region_paths": route["route_region_paths"],
        "bridge_counts": route["bridge_counts"],
        "route_hints": {
            leg: [regions[idx]["name"] for idx in indices]
            for leg, indices in route_hints.items()
        },
        "checks": {
            "regions_do_not_intersect_inflated_obstacles": True,
            "route_found": True,
            "task_order": ["home", "pick", "drop", "return", "home"],
            "loaded_states_use_triangle": all(s["formation"] == "triangle" for s in route["states"] if s["task_mode"] == "loaded"),
            "return_states_use_line": all(s["formation"] == "line" for s in route["states"] if s["task_mode"] == "delivered"),
            "return_repaired": route["return_repaired"],
            "return_offset_px": round(float(route["return_offset_px"]), 6),
            "return_exact_center": bool(route["return_exact_center"]),
        },
    }
    return case


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", default=str(ROOT / "data/demo1_map.json"))
    parser.add_argument("--output", default=str(ROOT / "data/demo1_case.json"))
    parser.add_argument("--package-output", default=str(ROOT / "src/swarm_random_payload/data/demo1_case.json"))
    parser.add_argument("--random-seed", type=int, default=31)
    parser.add_argument("--seed-budget", type=int, default=1000)
    parser.add_argument("--max-regions", type=int, default=60)
    parser.add_argument("--min-regions", type=int, default=60)
    parser.add_argument("--iteration-limit", type=int, default=70)
    parser.add_argument("--min-region-area", type=float, default=900.0)
    parser.add_argument("--obstacle-overlap-tolerance", type=float, default=1.0)
    parser.add_argument(
        "--source-regions-only",
        action="store_true",
        help="Use only seeds from exported source regions; do not add automatic sampling seeds.",
    )
    args = parser.parse_args()

    case = generate(args)
    body = json.dumps(case, indent=2, ensure_ascii=False)
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    Path(args.output).write_text(body + "\n", encoding="utf-8")
    if args.package_output:
        Path(args.package_output).parent.mkdir(parents=True, exist_ok=True)
        Path(args.package_output).write_text(body + "\n", encoding="utf-8")
    print(
        "generated "
        f"{len(case['regions'])} regions, {len(case['bridges'])} selected bridges, "
        f"{len(case['states'])} route states -> {args.output}"
    )


if __name__ == "__main__":
    main()

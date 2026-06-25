#!/usr/bin/env python3
"""Export a payload-transport case as a static Gazebo Sim SDF scene.

The exported world is intentionally a scene companion for RViz, not a
replacement for the QP/controller pipeline.  RViz remains the authoritative
viewer for certified regions, bridges, and controller state.  Gazebo provides
a physically styled 3D map, obstacles, task zones, payload, and vehicle
visuals from the same case file.
"""

from __future__ import annotations

import argparse
import html
import json
import math
from pathlib import Path
from typing import Any, Iterable, Sequence


Point = tuple[float, float]
RGBA = tuple[float, float, float, float]

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MESH_DIR = ROOT / "src" / "swarm_random_payload" / "meshes"
DEFAULT_OUT_DIR = ROOT / "out" / "gazebo"

COLORS: dict[str, RGBA] = {
    "floor": (0.78, 0.83, 0.90, 1.0),
    "boundary": (0.05, 0.25, 0.18, 1.0),
    "obstacle": (0.18, 0.24, 0.34, 1.0),
    "obstacle_top": (0.38, 0.52, 0.70, 1.0),
    "home": (0.06, 0.46, 0.43, 0.62),
    "pick": (0.98, 0.45, 0.10, 0.62),
    "drop": (0.75, 0.09, 0.36, 0.62),
    "return": (0.49, 0.23, 0.93, 0.90),
    "label_text": (1.0, 1.0, 1.0, 1.0),
    "label_post": (0.04, 0.04, 0.04, 1.0),
    "drone_red": (0.92, 0.12, 0.12, 1.0),
    "drone_cyan": (0.0, 0.78, 0.90, 1.0),
    "drone_blue": (0.10, 0.35, 0.95, 1.0),
    "payload": (0.92, 0.72, 0.22, 1.0),
    "route": (1.0, 0.88, 0.0, 0.72),
    "region": (0.42, 0.78, 0.46, 0.30),
    "workspace_outline": (0.08, 0.40, 0.28, 0.92),
    "cable": (0.02, 0.02, 0.02, 0.92),
    "current_region": (0.08, 0.28, 1.0, 0.96),
    "target_region": (0.0, 0.88, 0.95, 0.96),
    "bridge_region": (1.0, 0.86, 0.0, 0.98),
}


def rgba(color: RGBA) -> str:
    return " ".join(f"{channel:.3f}" for channel in color)


def material(color: RGBA) -> str:
    color_text = rgba(color)
    return (
        "<material>"
        f"<ambient>{color_text}</ambient>"
        f"<diffuse>{color_text}</diffuse>"
        "<specular>0.12 0.12 0.12 1</specular>"
        "</material>"
    )


def pose(x: float, y: float, z: float, yaw: float = 0.0) -> str:
    return f"{x:.6f} {y:.6f} {z:.6f} 0 0 {yaw:.6f}"


def model_name(name: str) -> str:
    safe = [ch if ch.isalnum() or ch == "_" else "_" for ch in name.lower()]
    return "".join(safe)


def point_from_json(point: dict[str, Any]) -> Point:
    return (float(point["x"]), float(point["y"]))


def obstacle_poly(obstacle: dict[str, Any]) -> list[Point]:
    if obstacle.get("type") == "rect":
        x = float(obstacle["x"])
        y = float(obstacle["y"])
        w = float(obstacle["w"])
        h = float(obstacle["h"])
        x0, x1 = sorted((x, x + w))
        y0, y1 = sorted((y, y + h))
        return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    return [point_from_json(point) for point in obstacle["points"]]


def to_world(point: Point, height: float, scale: float) -> Point:
    return (point[0] * scale, (height - point[1]) * scale)


def to_world_poly(points: Sequence[Point], height: float, scale: float) -> list[Point]:
    return [to_world(point, height, scale) for point in points]


def bounds(points: Sequence[Point]) -> tuple[float, float, float, float]:
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    return min(xs), max(xs), min(ys), max(ys)


def add_box_model(
    parts: list[str],
    *,
    name: str,
    center: tuple[float, float, float],
    size: tuple[float, float, float],
    color: RGBA,
    static: bool = True,
) -> None:
    x, y, z = center
    sx, sy, sz = size
    safe_name = model_name(name)
    parts.append(
        f"""
    <model name="{safe_name}">
      <static>{str(static).lower()}</static>
      <pose>{pose(x, y, z)}</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx:.6f} {sy:.6f} {sz:.6f}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx:.6f} {sy:.6f} {sz:.6f}</size></box></geometry>
          {material(color)}
        </visual>
      </link>
    </model>"""
    )


def add_cylinder_model(
    parts: list[str],
    *,
    name: str,
    center: tuple[float, float, float],
    radius: float,
    length: float,
    color: RGBA,
    static: bool = True,
) -> None:
    x, y, z = center
    safe_name = model_name(name)
    parts.append(
        f"""
    <model name="{safe_name}">
      <static>{str(static).lower()}</static>
      <pose>{pose(x, y, z)}</pose>
      <link name="link">
        <collision name="collision">
          <geometry><cylinder><radius>{radius:.6f}</radius><length>{length:.6f}</length></cylinder></geometry>
        </collision>
        <visual name="visual">
          <geometry><cylinder><radius>{radius:.6f}</radius><length>{length:.6f}</length></cylinder></geometry>
          {material(color)}
        </visual>
      </link>
    </model>"""
    )


def add_sphere_model(
    parts: list[str],
    *,
    name: str,
    center: tuple[float, float, float],
    radius: float,
    color: RGBA,
    static: bool = True,
) -> None:
    x, y, z = center
    safe_name = model_name(name)
    parts.append(
        f"""
    <model name="{safe_name}">
      <static>{str(static).lower()}</static>
      <pose>{pose(x, y, z)}</pose>
      <link name="link">
        <visual name="visual">
          <geometry><sphere><radius>{radius:.6f}</radius></sphere></geometry>
          {material(color)}
        </visual>
      </link>
    </model>"""
    )


def add_text_label_model(
    parts: list[str],
    *,
    name: str,
    label: str,
    center: tuple[float, float, float],
    size: float,
    color: RGBA,
    static: bool = True,
) -> None:
    x, y, z = center
    safe_name = model_name(name)
    safe_label = html.escape(label)
    parts.append(
        f"""
    <model name="{safe_name}">
      <static>{str(static).lower()}</static>
      <pose>{pose(x, y, z)}</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <text>
              <string>{safe_label}</string>
              <font>Arial</font>
              <size>{size:.6f}</size>
              <align>center</align>
            </text>
          </geometry>
          <cast_shadows>false</cast_shadows>
          {material(color)}
        </visual>
      </link>
    </model>"""
    )


def add_line_box_model(
    parts: list[str],
    *,
    name: str,
    start: tuple[float, float],
    end: tuple[float, float],
    z: float,
    thickness: float,
    height: float,
    color: RGBA,
    static: bool = True,
) -> None:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = math.hypot(dx, dy)
    if length < 1e-9:
        return
    center = ((start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0, z)
    yaw = math.atan2(dy, dx)
    safe_name = model_name(name)
    parts.append(
        f"""
    <model name="{safe_name}">
      <static>{str(static).lower()}</static>
      <pose>{pose(center[0], center[1], center[2], yaw)}</pose>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>{length:.6f} {thickness:.6f} {height:.6f}</size></box></geometry>
          {material(color)}
        </visual>
      </link>
    </model>"""
    )


def add_polygon_outline(
    parts: list[str],
    *,
    name: str,
    points: Sequence[Point],
    z: float,
    thickness: float,
    height: float,
    color: RGBA,
) -> None:
    if len(points) < 2:
        return
    for idx, start in enumerate(points):
        end = points[(idx + 1) % len(points)]
        add_line_box_model(
            parts,
            name=f"{name}_edge_{idx}",
            start=start,
            end=end,
            z=z,
            thickness=thickness,
            height=height,
            color=color,
        )


def add_polygon_highlight_model(
    parts: list[str],
    *,
    name: str,
    points: Sequence[Point],
    z: float,
    thickness: float,
    height: float,
    color: RGBA,
) -> None:
    """Create one movable outline model whose visuals start outside the scene."""
    if len(points) < 2:
        return

    visual_parts: list[str] = []
    for idx, start in enumerate(points):
        end = points[(idx + 1) % len(points)]
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = math.hypot(dx, dy)
        if length < 1e-9:
            continue
        center = ((start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0)
        yaw = math.atan2(dy, dx)
        visual_parts.append(
            f"""
        <visual name="edge_{idx}">
          <pose>{center[0]:.6f} {center[1]:.6f} 0 0 0 {yaw:.6f}</pose>
          <geometry><box><size>{length:.6f} {thickness:.6f} {height:.6f}</size></box></geometry>
          {material(color)}
        </visual>"""
        )

    if not visual_parts:
        return
    parts.append(
        f"""
    <model name="{model_name(name)}">
      <static>true</static>
      <pose>-1000 -1000 -1000 0 0 0</pose>
      <link name="link">
        {''.join(visual_parts)}
      </link>
    </model>"""
    )


def rotate_offset(offset: Point, yaw: float) -> Point:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (c * offset[0] - s * offset[1], s * offset[0] + c * offset[1])


def add_drone_model(
    parts: list[str],
    *,
    name: str,
    xy: Point,
    yaw: float,
    color: RGBA,
    mesh_dir: Path,
    mesh_scale: float,
    altitude: float,
) -> None:
    body_uri = (mesh_dir / "NXP-HGD-CF.dae").resolve().as_uri()
    motor_base_uri = (mesh_dir / "5010Base.dae").resolve().as_uri()
    motor_bell_uri = (mesh_dir / "5010Bell.dae").resolve().as_uri()
    rotor_offsets = [(0.174, -0.174), (-0.174, 0.174), (0.174, 0.174), (-0.174, -0.174)]
    visuals = [
        f"""
        <visual name="body_mesh">
          <pose>0 0 0.025 0 0 3.141593</pose>
          <geometry>
            <mesh>
              <uri>{body_uri}</uri>
              <scale>{mesh_scale:.5f} {mesh_scale:.5f} {mesh_scale:.5f}</scale>
            </mesh>
          </geometry>
        </visual>""",
    ]
    for rotor_idx, (ox, oy) in enumerate(rotor_offsets):
        visuals.append(
            f"""
        <visual name="motor_base_{rotor_idx}">
          <pose>{ox:.4f} {oy:.4f} 0.032 0 0 -0.450000</pose>
          <geometry>
            <mesh>
              <uri>{motor_base_uri}</uri>
              <scale>{mesh_scale:.5f} {mesh_scale:.5f} {mesh_scale:.5f}</scale>
            </mesh>
          </geometry>
        </visual>
        <visual name="motor_bell_{rotor_idx}">
          <pose>{ox:.4f} {oy:.4f} 0.028 0 0 0</pose>
          <geometry>
            <mesh>
              <uri>{motor_bell_uri}</uri>
              <scale>{mesh_scale:.5f} {mesh_scale:.5f} {mesh_scale:.5f}</scale>
            </mesh>
          </geometry>
        </visual>"""
        )
    parts.append(
        f"""
    <model name="{model_name(name)}">
      <static>true</static>
      <pose>{pose(xy[0], xy[1], altitude, yaw)}</pose>
      <link name="base_link">
        {''.join(visuals)}
      </link>
    </model>"""
    )


def add_propeller_model(
    parts: list[str],
    *,
    name: str,
    xy: Point,
    yaw: float,
    color: RGBA,
    mesh_dir: Path,
    mesh_scale: float,
    altitude: float,
    rotor_idx: int,
) -> None:
    prop_uri = (mesh_dir / ("1345_prop_ccw.stl" if rotor_idx in {0, 1} else "1345_prop_cw.stl")).resolve().as_uri()
    tint_color = (color[0], color[1], color[2], 0.42)
    parts.append(
        f"""
    <model name="{model_name(name)}">
      <static>true</static>
      <pose>{pose(xy[0], xy[1], altitude, yaw)}</pose>
      <link name="link">
        <visual name="propeller_mesh">
          <pose>-0.022000 -0.146385 -0.016000 0 0 0</pose>
          <geometry>
            <mesh>
              <uri>{prop_uri}</uri>
              <scale>{mesh_scale * 0.846153846:.5f} {mesh_scale * 0.846153846:.5f} {mesh_scale * 0.846153846:.5f}</scale>
            </mesh>
          </geometry>
        </visual>
        <visual name="propeller_motion_blur">
          <pose>0 0 0.006 0 0 0</pose>
          <geometry><cylinder><radius>{0.185 * mesh_scale:.6f}</radius><length>0.006</length></cylinder></geometry>
          {material(tint_color)}
        </visual>
      </link>
    </model>"""
    )


def add_cable_model(parts: list[str], *, name: str, length: float, radius: float) -> None:
    parts.append(
        f"""
    <model name="{model_name(name)}">
      <static>true</static>
      <pose>{pose(-10.0, -10.0, -4.0)}</pose>
      <link name="link">
        <visual name="visual">
          <geometry><cylinder><radius>{radius:.6f}</radius><length>{length:.6f}</length></cylinder></geometry>
          {material(COLORS["cable"])}
        </visual>
      </link>
    </model>"""
    )


def add_cable_models(parts: list[str], *, vehicle_count: int, lengths: Sequence[float], radius: float) -> None:
    for vehicle_index in range(vehicle_count):
        for length in lengths:
            add_cable_model(
                parts,
                name=f"payload_cable_r{vehicle_index + 1}_{int(round(length * 100)):03d}",
                length=length,
                radius=radius,
            )


def add_rope_segment_models(
    parts: list[str],
    *,
    vehicle_count: int,
    segment_count: int,
    segment_length: float,
    radius: float,
) -> None:
    for vehicle_index in range(vehicle_count):
        for segment_index in range(segment_count):
            add_cable_model(
                parts,
                name=f"payload_rope_r{vehicle_index + 1}_s{segment_index + 1}",
                length=segment_length,
                radius=radius,
            )


def initial_state(case: dict[str, Any]) -> dict[str, Any]:
    for state in case.get("states", []):
        if state.get("state_id") == "s_home_start":
            return state
    return case["states"][0]


def route_centers(case: dict[str, Any]) -> Iterable[Point]:
    for state in case.get("states", []):
        placement = state.get("placement") or {}
        center = placement.get("center")
        if center:
            yield point_from_json(center)
    for bridge in case.get("bridges", []):
        placement = bridge.get("placement") or {}
        center = placement.get("center")
        if center:
            yield point_from_json(center)


def export_world(
    case_file: Path,
    out_file: Path,
    map_scale: float,
    mesh_scale: float,
    mesh_dir: Path,
    drone_altitude: float,
    obstacle_height: float,
) -> Path:
    case = json.loads(case_file.read_text(encoding="utf-8"))
    width = float(case["width"])
    height = float(case["height"])
    obstacle_margin = float(case["constants"].get("obstacleMargin", 8.0)) * map_scale

    out_file.parent.mkdir(parents=True, exist_ok=True)

    world_width = width * map_scale
    world_height = height * map_scale
    center_x = world_width / 2.0
    center_y = world_height / 2.0
    parts: list[str] = []

    add_box_model(
        parts,
        name="workspace_floor",
        center=(center_x, center_y, -0.012),
        size=(world_width + 2.0 * obstacle_margin, world_height + 2.0 * obstacle_margin, 0.024),
        color=COLORS["floor"],
    )

    rail_thickness = max(0.045, obstacle_margin * 0.18)
    expanded_w = world_width + 2.0 * obstacle_margin
    expanded_h = world_height + 2.0 * obstacle_margin
    add_box_model(parts, name="boundary_south", center=(center_x, -obstacle_margin, 0.055), size=(expanded_w, rail_thickness, 0.11), color=COLORS["boundary"])
    add_box_model(parts, name="boundary_north", center=(center_x, world_height + obstacle_margin, 0.055), size=(expanded_w, rail_thickness, 0.11), color=COLORS["boundary"])
    add_box_model(parts, name="boundary_west", center=(-obstacle_margin, center_y, 0.055), size=(rail_thickness, expanded_h, 0.11), color=COLORS["boundary"])
    add_box_model(parts, name="boundary_east", center=(world_width + obstacle_margin, center_y, 0.055), size=(rail_thickness, expanded_h, 0.11), color=COLORS["boundary"])

    add_polygon_outline(
        parts,
        name="original_workspace",
        points=[(0.0, 0.0), (world_width, 0.0), (world_width, world_height), (0.0, world_height)],
        z=0.040,
        thickness=0.020,
        height=0.012,
        color=COLORS["workspace_outline"],
    )

    for region in case.get("regions", []):
        poly = to_world_poly([point_from_json(point) for point in region["poly"]], height, map_scale)
        add_polygon_outline(
            parts,
            name=f"iris_region_{region['id']}",
            points=poly,
            z=0.050,
            thickness=0.016,
            height=0.010,
            color=COLORS["region"],
        )
        add_polygon_highlight_model(
            parts,
            name=f"highlight_current_p{region['id']}",
            points=poly,
            z=0.0,
            thickness=0.050,
            height=0.026,
            color=COLORS["current_region"],
        )
        add_polygon_highlight_model(
            parts,
            name=f"highlight_target_p{region['id']}",
            points=poly,
            z=0.0,
            thickness=0.050,
            height=0.026,
            color=COLORS["target_region"],
        )

    for bridge in case.get("bridges", []):
        poly = to_world_poly([point_from_json(point) for point in bridge.get("poly", [])], height, map_scale)
        add_polygon_highlight_model(
            parts,
            name=f"highlight_bridge_{bridge['name']}",
            points=poly,
            z=0.0,
            thickness=0.040,
            height=0.030,
            color=COLORS["bridge_region"],
        )

    for idx, obstacle in enumerate(case.get("obstacles", []), start=1):
        poly = to_world_poly(obstacle_poly(obstacle), height, map_scale)
        min_x, max_x, min_y, max_y = bounds(poly)
        sx = max_x - min_x
        sy = max_y - min_y
        add_box_model(
            parts,
            name=f"obstacle_{idx}",
            center=((min_x + max_x) / 2.0, (min_y + max_y) / 2.0, obstacle_height / 2.0),
            size=(sx, sy, obstacle_height),
            color=COLORS["obstacle"],
        )
        add_box_model(
            parts,
            name=f"obstacle_{idx}_top_color",
            center=((min_x + max_x) / 2.0, (min_y + max_y) / 2.0, obstacle_height + 0.006),
            size=(sx, sy, 0.012),
            color=COLORS["obstacle_top"],
        )

    task = case.get("task", {})
    for key, task_region in task.get("regions", {}).items():
        center = to_world(point_from_json(task_region["center"]), height, map_scale)
        size = float(task_region["size"]) * map_scale
        color = COLORS.get(key, COLORS["route"])
        add_box_model(
            parts,
            name=f"task_{key}",
            center=(center[0], center[1], 0.022),
            size=(size, size, 0.044),
            color=color,
        )

    return_spec = task.get("returnPoint")
    if return_spec:
        xy = to_world(point_from_json(return_spec["point"]), height, map_scale)
        add_cylinder_model(parts, name="return_point", center=(xy[0], xy[1], 0.13), radius=0.13, length=0.26, color=COLORS["return"])

    state = initial_state(case)
    placement = state["placement"]
    robots = [point_from_json(robot) for robot in placement["robots"]]
    theta = float(placement.get("theta", 0.0))
    drone_colors = [COLORS["drone_red"], COLORS["drone_cyan"], COLORS["drone_blue"]]
    for idx, robot in enumerate(robots):
        xy = to_world(robot, height, map_scale)
        yaw = -theta
        add_drone_model(
            parts,
            name=f"drone_{idx + 1}",
            xy=xy,
            yaw=yaw,
            color=drone_colors[idx % len(drone_colors)],
            mesh_dir=mesh_dir,
            mesh_scale=mesh_scale,
            altitude=drone_altitude,
        )
        rotor_offsets = [(0.174, -0.174), (-0.174, 0.174), (0.174, 0.174), (-0.174, -0.174)]
        for rotor_idx, offset in enumerate(rotor_offsets):
            dx, dy = rotate_offset((offset[0] * mesh_scale, offset[1] * mesh_scale), yaw)
            add_propeller_model(
                parts,
                name=f"drone_{idx + 1}_propeller_{rotor_idx}",
                xy=(xy[0] + dx, xy[1] + dy),
                yaw=yaw,
                color=drone_colors[idx % len(drone_colors)],
                mesh_dir=mesh_dir,
                mesh_scale=mesh_scale,
                altitude=drone_altitude + 0.082,
                rotor_idx=rotor_idx,
            )

    initial_camera_target = (
        sum(to_world(robot, height, map_scale)[0] for robot in robots) / len(robots),
        sum(to_world(robot, height, map_scale)[1] for robot in robots) / len(robots),
        drone_altitude + 0.65,
    )
    add_sphere_model(
        parts,
        name="camera_target_follow",
        center=initial_camera_target,
        radius=0.025,
        color=(0.15, 0.55, 1.0, 0.04),
        static=True,
    )

    pick = task.get("regions", {}).get("pick")
    if pick:
        pick_xy = to_world(point_from_json(pick["center"]), height, map_scale)
        initial_payload_center = (pick_xy[0], pick_xy[1], 0.11)
    else:
        initial_payload_center = (-10.0, -10.0, -4.0)
    add_box_model(
        parts,
        name="payload_preview",
        center=initial_payload_center,
        size=(0.34, 0.34, 0.18),
        color=COLORS["payload"],
        static=True,
    )
    for idx, offset in enumerate(((0.13, 0.0, 0.11), (-0.07, 0.11, 0.11), (-0.07, -0.11, 0.11)), start=1):
        add_sphere_model(
            parts,
            name=f"payload_hook_{idx}",
            center=(
                initial_payload_center[0] + offset[0],
                initial_payload_center[1] + offset[1],
                initial_payload_center[2] + offset[2],
            ),
            radius=0.035,
            color=COLORS["cable"],
            static=True,
        )
    add_cable_models(
        parts,
        vehicle_count=3,
        lengths=(0.14, 0.20, 0.26, 0.32, 0.38, 0.46, 0.54, 0.64, 0.76, 0.90, 1.08, 1.26),
        radius=0.008,
    )
    add_rope_segment_models(
        parts,
        vehicle_count=3,
        segment_count=3,
        segment_length=0.32,
        radius=0.011,
    )

    world = f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="payload_transport_scene">
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>

    <gravity>0 0 -9.8</gravity>
    <scene>
      <ambient>0.68 0.72 0.78 1</ambient>
      <background>0.92 0.95 1.0 1</background>
      <shadows>false</shadows>
      <grid>false</grid>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>{pose(center_x - 2.0, center_y - 3.0, 8.0, 0.0)}</pose>
      <diffuse>0.95 0.93 0.86 1</diffuse>
      <specular>0.30 0.30 0.30 1</specular>
      <direction>-0.35 0.45 -0.82</direction>
    </light>
    {''.join(parts)}
  </world>
</sdf>
"""
    out_file.write_text(world, encoding="utf-8")
    return out_file


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", type=Path, help="Path to a demo*_case.json file.")
    parser.add_argument("--demo", default="demo8", help="Demo id such as 8 or demo8. Used when --case is omitted.")
    parser.add_argument("--out", type=Path, help="Output SDF world path.")
    parser.add_argument("--map-scale", type=float, default=0.01, help="Meters per map unit.")
    parser.add_argument("--mesh-scale", type=float, default=0.34, help="Gazebo visual scale for the drone mesh.")
    parser.add_argument("--mesh-dir", type=Path, default=DEFAULT_MESH_DIR, help="Directory containing drone mesh assets.")
    parser.add_argument("--drone-altitude", type=float, default=1.65, help="Initial visual flight altitude in meters.")
    parser.add_argument("--obstacle-height", type=float, default=1.85, help="Vertical obstacle height in meters.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    demo = str(args.demo)
    if not demo.startswith("demo"):
        demo = f"demo{demo}"

    case_file = args.case or ROOT / "data" / f"{demo}_case.json"
    if not case_file.exists():
        raise FileNotFoundError(case_file)

    out_file = args.out or DEFAULT_OUT_DIR / f"{demo}_gazebo_scene.sdf"
    exported = export_world(
        case_file,
        out_file,
        args.map_scale,
        args.mesh_scale,
        args.mesh_dir,
        args.drone_altitude,
        args.obstacle_height,
    )
    print(f"[OK] exported Gazebo scene: {exported}")


if __name__ == "__main__":
    main()

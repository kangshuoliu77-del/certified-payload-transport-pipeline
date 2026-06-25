#!/usr/bin/env python3
"""Export a Gazebo world with dynamic X500-style multicopters over a demo map."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from export_gazebo_scene import (
    COLORS,
    DEFAULT_MESH_DIR,
    DEFAULT_OUT_DIR,
    ROOT,
    add_box_model,
    add_cylinder_model,
    add_polygon_highlight_model,
    add_polygon_outline,
    add_rope_segment_models,
    bounds,
    material,
    model_name,
    obstacle_poly,
    point_from_json,
    pose,
    rgba,
    to_world,
    to_world_poly,
)


ROTOR_OFFSETS = ((0.174, -0.174), (-0.174, 0.174), (0.174, 0.174), (-0.174, -0.174))


def first_state(case: dict[str, Any]) -> dict[str, Any]:
    for state in case.get("states", []):
        if state.get("state_id") == "s_home_start":
            return state
    return case["states"][0]


def x500_plugins(namespace: str) -> str:
    return f"""
      <plugin
        filename="ignition-gazebo-multicopter-motor-model-system"
        name="gz::sim::systems::MulticopterMotorModel">
        <robotNamespace>{namespace}</robotNamespace>
        <jointName>rotor_0_joint</jointName>
        <linkName>rotor_0</linkName>
        <turningDirection>ccw</turningDirection>
        <timeConstantUp>0.0125</timeConstantUp>
        <timeConstantDown>0.025</timeConstantDown>
        <maxRotVelocity>800.0</maxRotVelocity>
        <motorConstant>8.54858e-06</motorConstant>
        <momentConstant>0.016</momentConstant>
        <commandSubTopic>gazebo/command/motor_speed</commandSubTopic>
        <motorNumber>0</motorNumber>
        <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
        <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
        <motorSpeedPubTopic>motor_speed/0</motorSpeedPubTopic>
        <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
        <motorType>velocity</motorType>
      </plugin>
      <plugin
        filename="ignition-gazebo-multicopter-motor-model-system"
        name="gz::sim::systems::MulticopterMotorModel">
        <robotNamespace>{namespace}</robotNamespace>
        <jointName>rotor_1_joint</jointName>
        <linkName>rotor_1</linkName>
        <turningDirection>ccw</turningDirection>
        <timeConstantUp>0.0125</timeConstantUp>
        <timeConstantDown>0.025</timeConstantDown>
        <maxRotVelocity>800.0</maxRotVelocity>
        <motorConstant>8.54858e-06</motorConstant>
        <momentConstant>0.016</momentConstant>
        <commandSubTopic>gazebo/command/motor_speed</commandSubTopic>
        <motorNumber>1</motorNumber>
        <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
        <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
        <motorSpeedPubTopic>motor_speed/1</motorSpeedPubTopic>
        <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
        <motorType>velocity</motorType>
      </plugin>
      <plugin
        filename="ignition-gazebo-multicopter-motor-model-system"
        name="gz::sim::systems::MulticopterMotorModel">
        <robotNamespace>{namespace}</robotNamespace>
        <jointName>rotor_2_joint</jointName>
        <linkName>rotor_2</linkName>
        <turningDirection>cw</turningDirection>
        <timeConstantUp>0.0125</timeConstantUp>
        <timeConstantDown>0.025</timeConstantDown>
        <maxRotVelocity>800.0</maxRotVelocity>
        <motorConstant>8.54858e-06</motorConstant>
        <momentConstant>0.016</momentConstant>
        <commandSubTopic>gazebo/command/motor_speed</commandSubTopic>
        <motorNumber>2</motorNumber>
        <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
        <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
        <motorSpeedPubTopic>motor_speed/2</motorSpeedPubTopic>
        <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
        <motorType>velocity</motorType>
      </plugin>
      <plugin
        filename="ignition-gazebo-multicopter-motor-model-system"
        name="gz::sim::systems::MulticopterMotorModel">
        <robotNamespace>{namespace}</robotNamespace>
        <jointName>rotor_3_joint</jointName>
        <linkName>rotor_3</linkName>
        <turningDirection>cw</turningDirection>
        <timeConstantUp>0.0125</timeConstantUp>
        <timeConstantDown>0.025</timeConstantDown>
        <maxRotVelocity>800.0</maxRotVelocity>
        <motorConstant>8.54858e-06</motorConstant>
        <momentConstant>0.016</momentConstant>
        <commandSubTopic>gazebo/command/motor_speed</commandSubTopic>
        <motorNumber>3</motorNumber>
        <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
        <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
        <motorSpeedPubTopic>motor_speed/3</motorSpeedPubTopic>
        <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
        <motorType>velocity</motorType>
      </plugin>
      <plugin
        filename="ignition-gazebo-multicopter-control-system"
        name="gz::sim::systems::MulticopterVelocityControl">
        <robotNamespace>{namespace}</robotNamespace>
        <commandSubTopic>gazebo/command/twist</commandSubTopic>
        <enableSubTopic>enable</enableSubTopic>
        <comLinkName>base_link</comLinkName>
        <velocityGain>2.7 2.7 3.4</velocityGain>
        <attitudeGain>2 3 0.15</attitudeGain>
        <angularRateGain>0.4 0.52 0.18</angularRateGain>
        <maximumLinearAcceleration>2 2 8.0</maximumLinearAcceleration>
        <maximumLinearVelocity>2.0 2.0 2.2</maximumLinearVelocity>
        <maximumAngularVelocity>1.5 1.5 1.5</maximumAngularVelocity>
        <rotorConfiguration>
          <rotor>
            <jointName>rotor_0_joint</jointName>
            <forceConstant>8.54858e-06</forceConstant>
            <momentConstant>0.016</momentConstant>
            <direction>1</direction>
          </rotor>
          <rotor>
            <jointName>rotor_1_joint</jointName>
            <forceConstant>8.54858e-06</forceConstant>
            <momentConstant>0.016</momentConstant>
            <direction>1</direction>
          </rotor>
          <rotor>
            <jointName>rotor_2_joint</jointName>
            <forceConstant>8.54858e-06</forceConstant>
            <momentConstant>0.016</momentConstant>
            <direction>-1</direction>
          </rotor>
          <rotor>
            <jointName>rotor_3_joint</jointName>
            <forceConstant>8.54858e-06</forceConstant>
            <momentConstant>0.016</momentConstant>
            <direction>-1</direction>
          </rotor>
        </rotorConfiguration>
      </plugin>
      <plugin
        filename="ignition-gazebo-odometry-publisher-system"
        name="ignition::gazebo::systems::OdometryPublisher">
        <dimensions>3</dimensions>
      </plugin>"""


def inertial_block(mass: float, ixx: float, iyy: float, izz: float) -> str:
    return f"""
        <inertial>
          <mass>{mass:.6f}</mass>
          <inertia>
            <ixx>{ixx:.8f}</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>{iyy:.8f}</iyy><iyz>0</iyz>
            <izz>{izz:.8f}</izz>
          </inertia>
        </inertial>"""


def add_payload_model(
    parts: list[str],
    *,
    xy: tuple[float, float],
    z: float,
    size: tuple[float, float, float] = (0.36, 0.26, 0.18),
    mass: float = 0.35,
) -> None:
    sx, sy, sz = size
    ixx = mass * (sy * sy + sz * sz) / 12.0
    iyy = mass * (sx * sx + sz * sz) / 12.0
    izz = mass * (sx * sx + sy * sy) / 12.0
    parts.append(
        f"""
    <model name="payload">
      <pose>{pose(xy[0], xy[1], z + sz / 2.0)}</pose>
      <self_collide>true</self_collide>
      <link name="payload_link">
        {inertial_block(mass, ixx, iyy, izz)}
        <collision name="payload_collision">
          <geometry><box><size>{sx:.6f} {sy:.6f} {sz:.6f}</size></box></geometry>
          <surface>
            <friction>
              <ode><mu>0.8</mu><mu2>0.8</mu2></ode>
            </friction>
          </surface>
        </collision>
        <visual name="payload_visual">
          <geometry><box><size>{sx:.6f} {sy:.6f} {sz:.6f}</size></box></geometry>
          {material(COLORS["payload"])}
        </visual>
      </link>
      <plugin
        filename="ignition-gazebo-odometry-publisher-system"
        name="ignition::gazebo::systems::OdometryPublisher">
        <dimensions>3</dimensions>
      </plugin>
    </model>"""
    )


def add_x500_model(
    parts: list[str],
    *,
    name: str,
    namespace: str,
    xy: tuple[float, float],
    yaw: float,
    altitude: float,
    mesh_dir: Path,
) -> None:
    body_uri = (mesh_dir / "NXP-HGD-CF.dae").resolve().as_uri()
    motor_base_uri = (mesh_dir / "5010Base.dae").resolve().as_uri()
    motor_bell_uri = (mesh_dir / "5010Bell.dae").resolve().as_uri()
    base_visuals = [
        f"""
        <visual name="frame_and_electronics">
          <pose>0 0 0.025 0 0 3.141592654</pose>
          <geometry><mesh><uri>{body_uri}</uri><scale>1 1 1</scale></mesh></geometry>
        </visual>"""
    ]
    for idx, (ox, oy) in enumerate(ROTOR_OFFSETS):
        base_visuals.append(
            f"""
        <visual name="motor_base_{idx}">
          <pose>{ox:.6f} {oy:.6f} 0.032 0 0 -0.45</pose>
          <geometry><mesh><uri>{motor_base_uri}</uri><scale>1 1 1</scale></mesh></geometry>
        </visual>"""
        )

    rotor_links = []
    rotor_joints = []
    for idx, (ox, oy) in enumerate(ROTOR_OFFSETS):
        rotor_links.append(
            f"""
      <link name="rotor_{idx}">
        <pose>{ox:.6f} {oy:.6f} 0.060000 0 0 0</pose>
        {inertial_block(0.025, 0.000015, 0.000015, 0.000028)}
        <collision name="rotor_collision">
          <geometry><cylinder><radius>0.045</radius><length>0.016</length></cylinder></geometry>
        </collision>
        <visual name="propeller_blade_long">
          <pose>0 0 0.005 0 0 0</pose>
          <geometry><box><size>0.340 0.026 0.006</size></box></geometry>
          {material((0.012, 0.014, 0.018, 0.78))}
        </visual>
        <visual name="propeller_blade_cross">
          <pose>0 0 0.007 0 0 1.570796327</pose>
          <geometry><box><size>0.280 0.020 0.005</size></box></geometry>
          {material((0.020, 0.024, 0.030, 0.58))}
        </visual>
        <visual name="rotor_motion_blur">
          <pose>0 0 0.001 0 0 0</pose>
          <geometry><cylinder><radius>0.170</radius><length>0.004</length></cylinder></geometry>
          {material((0.04, 0.045, 0.05, 0.34))}
        </visual>
        <visual name="motor_bell">
          <pose>0 0 -0.032 0 0 0</pose>
          <geometry><mesh><uri>{motor_bell_uri}</uri><scale>1 1 1</scale></mesh></geometry>
        </visual>
      </link>"""
        )
        rotor_joints.append(
            f"""
      <joint name="rotor_{idx}_joint" type="revolute">
        <parent>base_link</parent>
        <child>rotor_{idx}</child>
        <axis>
          <xyz>0 0 1</xyz>
          <limit><lower>-1e16</lower><upper>1e16</upper></limit>
        </axis>
      </joint>"""
        )

    parts.append(
        f"""
    <model name="{model_name(name)}">
      <pose>{pose(xy[0], xy[1], altitude, yaw)}</pose>
      <link name="base_link">
        {inertial_block(1.55, 0.029125, 0.029125, 0.055225)}
        <collision name="body_collision">
          <pose>0 0 0.025 0 0 0</pose>
          <geometry><box><size>0.42 0.42 0.08</size></box></geometry>
        </collision>
        {''.join(base_visuals)}
      </link>
      {''.join(rotor_links)}
      {''.join(rotor_joints)}
      {x500_plugins(namespace)}
    </model>"""
    )


def export_world(
    case_file: Path,
    out_file: Path,
    map_scale: float,
    altitude: float,
    mesh_dir: Path,
    obstacle_height: float,
) -> Path:
    case = json.loads(case_file.read_text(encoding="utf-8"))
    width = float(case["width"])
    height = float(case["height"])
    obstacle_margin = float(case["constants"].get("obstacleMargin", 8.0)) * map_scale
    world_width = width * map_scale
    world_height = height * map_scale
    center_x = world_width / 2.0
    center_y = world_height / 2.0

    out_file.parent.mkdir(parents=True, exist_ok=True)
    parts: list[str] = []

    add_box_model(
        parts,
        name="workspace_floor",
        center=(center_x, center_y, -0.012),
        size=(world_width + 2.0 * obstacle_margin, world_height + 2.0 * obstacle_margin, 0.024),
        color=COLORS["floor"],
    )

    # Show the same safety band used by the planner around the workspace
    # boundary. It sits just outside the original map instead of becoming a
    # large surrounding wall.
    rail = max(0.045, obstacle_margin)
    rail_height = 0.24
    rail_center_z = rail_height / 2.0
    add_box_model(
        parts,
        name="boundary_south",
        center=(center_x, -obstacle_margin / 2.0, rail_center_z),
        size=(world_width + 2.0 * obstacle_margin, rail, rail_height),
        color=COLORS["boundary"],
    )
    add_box_model(
        parts,
        name="boundary_north",
        center=(center_x, world_height + obstacle_margin / 2.0, rail_center_z),
        size=(world_width + 2.0 * obstacle_margin, rail, rail_height),
        color=COLORS["boundary"],
    )
    add_box_model(
        parts,
        name="boundary_west",
        center=(-obstacle_margin / 2.0, center_y, rail_center_z),
        size=(rail, world_height + 2.0 * obstacle_margin, rail_height),
        color=COLORS["boundary"],
    )
    add_box_model(
        parts,
        name="boundary_east",
        center=(world_width + obstacle_margin / 2.0, center_y, rail_center_z),
        size=(rail, world_height + 2.0 * obstacle_margin, rail_height),
        color=COLORS["boundary"],
    )

    add_polygon_outline(
        parts,
        name="expanded_workspace_boundary",
        points=[
            (-obstacle_margin, -obstacle_margin),
            (world_width + obstacle_margin, -obstacle_margin),
            (world_width + obstacle_margin, world_height + obstacle_margin),
            (-obstacle_margin, world_height + obstacle_margin),
        ],
        z=0.095,
        thickness=0.025,
        height=0.018,
        color=COLORS["boundary"],
    )

    add_polygon_outline(
        parts,
        name="original_workspace",
        points=[(0.0, 0.0), (world_width, 0.0), (world_width, world_height), (0.0, world_height)],
        z=0.04,
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
            thickness=0.014,
            height=0.010,
            color=COLORS["region"],
        )
        add_polygon_highlight_model(
            parts,
            name=f"highlight_current_{region['name']}",
            points=poly,
            z=0.0,
            thickness=0.058,
            height=0.032,
            color=COLORS["current_region"],
        )
        add_polygon_highlight_model(
            parts,
            name=f"highlight_target_{region['name']}",
            points=poly,
            z=0.0,
            thickness=0.058,
            height=0.032,
            color=COLORS["target_region"],
        )

    for bridge in case.get("bridges", []):
        poly = to_world_poly([point_from_json(point) for point in bridge.get("poly", [])], height, map_scale)
        add_polygon_highlight_model(
            parts,
            name=f"highlight_bridge_{bridge['name']}",
            points=poly,
            z=0.0,
            thickness=0.052,
            height=0.036,
            color=COLORS["bridge_region"],
        )

    for idx, obstacle in enumerate(case.get("obstacles", []), start=1):
        poly = to_world_poly(obstacle_poly(obstacle), height, map_scale)
        min_x, max_x, min_y, max_y = bounds(poly)
        sx = max_x - min_x
        sy = max_y - min_y
        obstacle_z = obstacle_height
        add_box_model(
            parts,
            name=f"obstacle_{idx}",
            center=((min_x + max_x) / 2.0, (min_y + max_y) / 2.0, obstacle_z / 2.0),
            size=(sx, sy, obstacle_z),
            color=COLORS["obstacle"],
        )
        add_box_model(
            parts,
            name=f"obstacle_{idx}_top_color",
            center=((min_x + max_x) / 2.0, (min_y + max_y) / 2.0, obstacle_z + 0.015),
            size=(sx, sy, 0.030),
            color=COLORS["obstacle_top"],
        )

    for key, task_region in case.get("task", {}).get("regions", {}).items():
        center = to_world(point_from_json(task_region["center"]), height, map_scale)
        size = float(task_region["size"]) * map_scale
        add_box_model(
            parts,
            name=f"task_{key}",
            center=(center[0], center[1], 0.022),
            size=(size, size, 0.044),
            color=COLORS.get(key, COLORS["route"]),
        )

    return_spec = case.get("task", {}).get("returnPoint")
    if return_spec:
        xy = to_world(point_from_json(return_spec["point"]), height, map_scale)
        add_cylinder_model(parts, name="return_point", center=(xy[0], xy[1], 0.08), radius=0.13, length=0.16, color=COLORS["return"])

    pick_region = case.get("task", {}).get("regions", {}).get("pick")
    if pick_region:
        payload_xy = to_world(point_from_json(pick_region["center"]), height, map_scale)
        add_payload_model(parts, xy=payload_xy, z=0.02)
        add_rope_segment_models(
            parts,
            vehicle_count=3,
            segment_count=8,
            segment_length=0.18,
            radius=0.007,
        )

    start = first_state(case)
    start_yaw = -float(start["placement"].get("theta", 0.0))
    start_robots = start["placement"].get("robots") or [start["placement"]["center"]]
    if len(start_robots) >= 3:
        for idx, robot in enumerate(start_robots[:3], start=1):
            xy = to_world(point_from_json(robot), height, map_scale)
            add_x500_model(
                parts,
                name=f"x3_{idx}",
                namespace=f"X3_{idx}",
                xy=xy,
                yaw=start_yaw,
                altitude=altitude,
                mesh_dir=mesh_dir,
            )
    else:
        start_xy = to_world(point_from_json(start["placement"]["center"]), height, map_scale)
        add_x500_model(parts, name="x3_1", namespace="X3_1", xy=start_xy, yaw=start_yaw, altitude=altitude, mesh_dir=mesh_dir)

    world = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="payload_multicopter">
    <physics name="4ms" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="ignition-gazebo-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-apply-link-wrench-system" name="gz::sim::systems::ApplyLinkWrench"/>
    <plugin filename="ignition-gazebo-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <scene>
      <ambient>0.68 0.72 0.78 1</ambient>
      <background>0.92 0.95 1.0 1</background>
      <shadows>true</shadows>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>{pose(center_x - 2.0, center_y - 3.0, 8.0)}</pose>
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
    parser.add_argument("--case", type=Path, help="Path to demo*_case.json.")
    parser.add_argument("--demo", default="demo8", help="Demo id such as 8 or demo8.")
    parser.add_argument("--out", type=Path, help="Output SDF world path.")
    parser.add_argument("--map-scale", type=float, default=0.01)
    parser.add_argument("--altitude", type=float, default=1.6)
    parser.add_argument("--obstacle-height", type=float, default=3.20)
    parser.add_argument("--mesh-dir", type=Path, default=DEFAULT_MESH_DIR)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    demo = str(args.demo)
    if not demo.startswith("demo"):
        demo = f"demo{demo}"
    case_file = args.case or ROOT / "data" / f"{demo}_case.json"
    out_file = args.out or DEFAULT_OUT_DIR / f"{demo}_multicopter_world.sdf"
    if not case_file.exists():
        raise FileNotFoundError(case_file)
    exported = export_world(case_file, out_file, args.map_scale, args.altitude, args.mesh_dir, args.obstacle_height)
    print(f"[OK] exported dynamic Gazebo multicopter world: {exported}")


if __name__ == "__main__":
    main()

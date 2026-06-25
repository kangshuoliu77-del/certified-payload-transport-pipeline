# Gazebo Dynamic Demo Baseline

This file records the current stable Gazebo dynamic demo baseline. Keep this
version as the fallback before experimenting with a full cable/joint payload
simulation.

## Default Command

```bash
cd certified-payload-transport-pipeline
source /opt/ros/humble/setup.bash
colcon build --packages-select swarm_random_payload
source install/setup.bash

./scripts/run_gazebo_flying_demo.sh 8
```

## What This Baseline Contains

- X500-style multicopter meshes generated from the local mesh assets.
- Gazebo `MulticopterMotorModel` and `MulticopterVelocityControl` plugins for
  each drone.
- A ROS 2 velocity tracker following the cached paper-QP trajectory.
- A physical payload rigid body in Gazebo.
- Optional three-cable tension forces applied through Gazebo link wrenches
  when `ENABLE_PAYLOAD_TETHER=1` is set.
- Live rope visuals driven by Gazebo odometry.
- Current IRIS region, target IRIS region, and bridge/intersection highlights.
- Obstacles exported as tall 3D blocks.
- A workspace safety band expanded outward by the same obstacle margin used in
  the case file.

## Stable Parameters

The current default script parameters are:

```text
FLIGHT_ALTITUDE=1.6
MAP_SCALE=0.02
OBSTACLE_HEIGHT=3.20
MAX_Z_SPEED=1.30
WAYPOINT_TOLERANCE=0.75

ENABLE_PAYLOAD_TETHER=0
```

The optional tension-only cable prototype can be enabled with:

```bash
ENABLE_PAYLOAD_TETHER=1 ./scripts/run_gazebo_flying_demo.sh 8
```

Its current experimental parameters are:

```text
rope_rest_lengths=0.86,0.90,0.93
rope_stiffness=36.0
rope_damping=10.0
max_tension=12.5
tension_ramp_seconds=2.5
attach_slack=0.82
attach_max_drone_height=0.90
```

For demo8, the workspace is `980 x 650` pixels and the safety margin is
`16` pixels. With `MAP_SCALE=0.02`, the visible boundary band is offset by
`0.32 m` from the original workspace boundary.

## What Is Not Solved Yet

This baseline is a dynamics prototype, not a complete cable-suspended-load
simulator. The default run keeps the strong planner/formation visualization
stable. The optional cable model is tension-only and implemented through
external wrenches. It is useful for early pickup and load-lift experiments, but
it is not yet the final physical model for a paper claim about suspended-load
dynamics.

The next full-physics version should use explicit Gazebo constraints:

- rigid payload body with realistic inertia,
- cable links or distance constraints,
- ball/universal joints at drone anchors and payload hooks,
- attach/detach logic at PICK and DROP,
- a suspended-load-aware controller rather than only a waypoint velocity
  tracker.

Until that version is implemented and verified, keep this baseline available
for advisor demos because the route, formation, map, and visualization are
stable.

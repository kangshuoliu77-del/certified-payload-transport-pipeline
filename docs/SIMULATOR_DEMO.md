# Simulator Demo Layer

This project uses a presentation-first demo strategy. The current recommended
Gazebo video path is:

```bash
./scripts/run_gazebo_recording_lift_demo.sh 8
```

That path uses a precomputed paper-QP execution, a uniform recording trajectory,
and an in-process Gazebo replay plugin. It is the cleanest Gazebo demo for
advisor/video recording.

The broader simulator strategy is:

1. **RViz presentation/cinematic view** is the main algorithm viewer.
   It shows IRIS regions, current region, target region, bridge certificates,
   task markers, payload state, trajectories, and the animated drones.

2. **Gazebo Sim recording view** is the 3D environment companion.
   The static scripts open the same demo map as an SDF world with a floor,
   boundary, 3D obstacle blocks, task regions, return marker, route anchors,
   payload, and drone visuals.  The dynamic script additionally starts three
   Gazebo X500-style multicopters with motor dynamics, velocity-control
   plugins, live IRIS highlights, and an optional tension-only cable/payload
   prototype.

This follows the planner-centric style used by FAST-Lab / Fei Gao group
quadrotor demos: RViz is used for planner state and debugging, while a simulator
or rendered scene can provide the more realistic 3D environment impression.

## Why Gazebo Sim

Gazebo Fortress is already installed on this machine and integrates cleanly
with ROS 2 Humble.  Isaac Sim can produce more photorealistic renders, but it is
heavier and less reliable on this laptop for quick advisor demos.  For the
current paper-QP pipeline, Gazebo is the better practical target.

## What Is Simulated

The Gazebo scene is a visual/static companion, not the safety-critical planner:

- It reads the same `demo*_case.json` as the RViz pipeline.
- It uses the same map scale as the controller visualization.
- Obstacles, task regions, return point, payload, and initial drones are placed
  from the case data.
- It does not replace IRIS, bridge construction, route generation, or the QP
  controller.

The authoritative planning and controller evidence remains in the ROS/RViz
pipeline.

## Run Gazebo Only

```bash
cd certified-payload-transport-pipeline
./scripts/run_gazebo_scene.sh 8
```

This exports:

```text
out/gazebo/demo8_gazebo_scene.sdf
```

and opens it with:

```bash
ign gazebo out/gazebo/demo8_gazebo_scene.sdf
```

## Run Gazebo + RViz Together

Build the ROS package first:

```bash
cd certified-payload-transport-pipeline
source /opt/ros/humble/setup.bash
colcon build --packages-select swarm_random_payload
```

Then run:

```bash
./scripts/run_gazebo_rviz_demo.sh 8
```

This opens Gazebo for the 3D scene and RViz for the actual animated
paper-QP/IRIS/bridge demo.

## Run Gazebo Recording Lift Replay

Use this when recording the current polished Gazebo demo:

```bash
cd certified-payload-transport-pipeline
source /opt/ros/humble/setup.bash
source install/setup.bash

./scripts/stop_demo_processes.sh
./scripts/run_gazebo_recording_lift_demo.sh 8
```

This generates missing Gazebo worlds and trajectory caches from the case file,
then opens Gazebo with the in-process replay plugin.

## Older Gazebo Cinematic Replay

Use this when you want the demo to be visible inside Gazebo itself:

```bash
cd certified-payload-transport-pipeline
source /opt/ros/humble/setup.bash

./scripts/run_gazebo_cinematic_demo.sh 8
```

This generates a Gazebo world with the map, 3D obstacles, task regions, return
marker, route traces, three drone models, and a payload model. The moving
models follow the paper-QP trajectory through Gazebo's trajectory-follower
plugin, so the replay is stable and does not show the startup altitude drop of
the experimental physics-control demo.

## Run The Presentation Demo

Use this when recording a video or showing the pipeline to an advisor:

```bash
cd certified-payload-transport-pipeline
source /opt/ros/humble/setup.bash
colcon build --packages-select swarm_random_payload
source install/setup.bash

./scripts/run_presentation_demo.sh 8
```

This avoids Gazebo startup transients and follows the certified paper-QP
trajectory directly in RViz. It is the cleanest current view of the algorithm:
realistic drone mesh, payload, task regions, current region, target region, and
bridge certificate.

## Run The Dynamic Multicopter Demo

This starts three Gazebo X3 multicopters with Gazebo's built-in multicopter
motor model and velocity controller.  The script first exports the paper-QP
execution frames to a cached Gazebo trajectory file, then starts Gazebo paused,
brings up `ros_gz_bridge`, starts the live certified-region overlay, primes the
X3 velocity controllers, unpauses physics, and lets a ROS 2 tracker publish
enable and velocity commands. Set `ENABLE_PAYLOAD_TETHER=1` to also start the
experimental tension-only payload tether node.

```bash
cd certified-payload-transport-pipeline
source /opt/ros/humble/setup.bash
colcon build --packages-select swarm_random_payload
source install/setup.bash

./scripts/run_gazebo_flying_demo.sh 8
```

This writes:

```text
out/gazebo/demo8_x500_multicopter_world.sdf
out/gazebo/demo8_paper_qp_trajectory.json
```

and proves the Gazebo dynamics/control path:

```text
paper-QP execution frames
  -> cached Gazebo trajectory
  -> ROS 2 outer-loop velocity tracker
  -> ros_gz_bridge
  -> X3 enable + Twist command topics
  -> Gazebo MulticopterVelocityControl for each X3
  -> Gazebo motor dynamics / odometry
  -> live current/target/bridge highlights
```

The dynamic script is now the stable Gazebo baseline for advisor demos, but it
is still not the final suspended-load physics model. The current cable model is
tension-only and implemented through Gazebo link wrenches, and it is disabled
by default so the planner/formation demonstration stays stable. It is useful
for early descent, cable attachment, payload lift, and drone dynamics tests,
while a future full-physics version should replace it with explicit cable/joint
constraints and a suspended-load-aware controller. The exact baseline
parameters are recorded in `docs/GAZEBO_BASELINE.md`.

## Common Gazebo Pitfalls

- `run_gazebo_scene.sh` and `run_gazebo_rviz_demo.sh` are static scene scripts.
  They do not publish control inputs, so the drones will not fly.
- `run_gazebo_flying_demo.sh` is the dynamic script.  It bridges both
  `/X3_i/gazebo/command/twist` and `/X3_i/enable`; the enable topics are
  required by Gazebo's `MulticopterVelocityControl` plugin.
- If Gazebo appears stuck, kill old simulator processes before rerunning:

```bash
killall -q parameter_bridge gazebo_velocity_tracker || true
pkill -f "^ruby .*ign gazebo" || true
```

- To verify motion from another terminal:

```bash
ros2 topic echo /model/x3_1/odometry --once
ros2 topic echo /X3_1/gazebo/command/twist --once
ros2 topic echo /X3_1/enable --once
```

## Good Explanation For Advisor

```text
We use RViz as the main certified-planning viewer, similar to common
quadrotor-planning demos, because it exposes the current certified region,
bridge certificate, route, and controller state clearly.  Gazebo is added as a
separate scene companion generated from the same case file, so the demo also
has a simulator-style 3D environment without changing the planner or QP
controller.
```

# Certified Payload Transport Pipeline

Certified geometric abstraction and paper-QP execution pipeline for multi-UAV
payload transport in cluttered environments.

This repository contains a complete ROS 2 / RViz / Gazebo demo stack for a
three-quadrotor payload transport task:

```text
HOME -> PICK -> DROP -> RETURN -> HOME
```

The pipeline converts obstacle maps into IRIS-style certified convex regions,
filters them by formation feasibility, constructs bridge certificates between
overlapping regions, generates a symbolic route, and executes the route with a
paper-style QP controller. RViz is used as the certified planning/debugging
viewer, while Gazebo provides the polished 3D recording demo.

## Demo Videos

### 1. RViz Planar Paper-QP View

Certified regions, bridge transitions, task regions, payload state, and the
paper-QP execution in the original planar RViz visualization.

[![RViz planar paper-QP demo](https://raw.githubusercontent.com/kangshuoliu77-del/certified-payload-transport-pipeline/main/docs/assets/gifs/demo8-rviz-planar-qp-preview.gif)](https://github.com/kangshuoliu77-del/certified-payload-transport-pipeline/blob/main/docs/assets/videos/demo8-rviz-planar-qp.mp4)

[Open full MP4](https://github.com/kangshuoliu77-del/certified-payload-transport-pipeline/blob/main/docs/assets/videos/demo8-rviz-planar-qp.mp4)

### 2. RViz 3D Tracking View

The same certified pipeline shown with realistic drone meshes, payload, ropes,
and a 3D tracking camera in RViz.

[![RViz 3D tracking demo](https://raw.githubusercontent.com/kangshuoliu77-del/certified-payload-transport-pipeline/main/docs/assets/gifs/demo8-rviz-3d-follow-preview.gif)](https://github.com/kangshuoliu77-del/certified-payload-transport-pipeline/blob/main/docs/assets/videos/demo8-rviz-3d-follow.mp4)

[Open full MP4](https://github.com/kangshuoliu77-del/certified-payload-transport-pipeline/blob/main/docs/assets/videos/demo8-rviz-3d-follow.mp4)

### 3. Gazebo 3D Recording View

Polished Gazebo recording view with 3D obstacles, X500-style drones, payload,
delayed cable attach/detach at PICK/DROP, current/target region highlights, and
bridge certificate highlights.

[![Gazebo 3D recording demo](https://raw.githubusercontent.com/kangshuoliu77-del/certified-payload-transport-pipeline/main/docs/assets/gifs/demo8-gazebo-3d-recording-preview.gif)](https://github.com/kangshuoliu77-del/certified-payload-transport-pipeline/blob/main/docs/assets/videos/demo8-gazebo-3d-recording.mp4)

[Open full MP4](https://github.com/kangshuoliu77-del/certified-payload-transport-pipeline/blob/main/docs/assets/videos/demo8-gazebo-3d-recording.mp4)

## Pipeline Overview

```text
map JSON
  -> Drake IRIS certified convex regions
  -> formation-feasible certified states
  -> bridge certificates between overlapping regions
  -> symbolic task route
  -> paper-QP continuous execution
  -> RViz / Gazebo visualization
```

Core ideas:

- **Certified regions:** obstacle-free convex polytopes generated from the map.
- **Formation-feasible states:** symbolic states are admitted only when the full
  formation envelope fits inside a certified region.
- **Bridge certificates:** transitions between neighboring regions are admitted
  only when their intersection contains a valid formation placement.
- **Paper-QP execution:** the continuous controller enforces convergence,
  inter-robot safety, obstacle safety, input limits, and workspace bounds.

## Repository Layout

```text
data/                  Standard demo maps and executable case JSON files.
docs/                  Engineering notes, runbooks, and reading guides.
docs/assets/videos/    GitHub demo videos.
docs/assets/images/    Demo video cover images.
figures/               IRIS map designer UI and Drake IRIS HTTP backend.
gazebo_pose_replay/    Gazebo in-process replay plugin source.
scripts/               User-facing commands for build, demos, Gazebo, verify.
src/                   ROS 2 package: swarm_random_payload.
tools/                 Offline generators, verifiers, and Gazebo exporters.
out/                   Generated Gazebo worlds, trajectories, snapshots.
build/ install/ log/   colcon-generated ROS 2 build artifacts.
```

See [docs/PROJECT_STRUCTURE.md](docs/PROJECT_STRUCTURE.md) for the detailed
file map.

## Quick Start

Build the ROS 2 package:

```bash
cd certified-payload-transport-pipeline
source /opt/ros/humble/setup.bash
colcon build --packages-select swarm_random_payload
source install/setup.bash
```

Run the standard RViz pipeline demo:

```bash
./scripts/run_demo.sh 8
```

Run the presentation RViz demo:

```bash
./scripts/run_presentation_demo.sh 8
```

Run the current Gazebo recording demo:

```bash
./scripts/stop_demo_processes.sh
./scripts/run_gazebo_recording_lift_demo.sh 8
```

The Gazebo script regenerates missing output files automatically from
`data/demo8_case.json`.

## Rebuild And Verify Cases

Rebuild one case:

```bash
python3 tools/rebuild_cases.py --case demo8 --retries 12
```

Rebuild all standard cases:

```bash
./scripts/rebuild_standard_qp_demos.sh
```

Verify all standard cases:

```bash
./scripts/verify_standard_qp_demos.sh
```

## Reading Order

1. [docs/PROJECT_STRUCTURE.md](docs/PROJECT_STRUCTURE.md)
2. [docs/RUNBOOK.md](docs/RUNBOOK.md)
3. [docs/CODE_READING_ORDER.md](docs/CODE_READING_ORDER.md)
4. [docs/PIPELINE_OVERVIEW.md](docs/PIPELINE_OVERVIEW.md)
5. [FILE_GUIDE.md](FILE_GUIDE.md)

## Notes

- This folder is the engineering/demo pipeline, not the full paper source.
- The symbolic route is currently generated procedurally over the certified
  formation/bridge graph. Full GR(1)/TuLiP synthesis is not included here.
- The Gazebo recording demo is a polished visualization of the paper-QP
  pipeline. It is not yet a full physical suspended-load dynamics claim.

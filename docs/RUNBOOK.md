# Runbook

Use this file for day-to-day commands.

## Build ROS 2 Package

```bash
cd certified-payload-transport-pipeline
source /opt/ros/humble/setup.bash
colcon build --packages-select swarm_random_payload
source install/setup.bash
```

`source /opt/ros/humble/setup.bash` loads ROS 2 Humble commands.
`colcon build` builds the package.
`source install/setup.bash` makes the built package discoverable in the current
terminal.

## Run RViz Pipeline Demo

```bash
./scripts/run_demo.sh 8
```

This launches the ROS node for a standard case. It is the direct algorithm
viewer.

Presentation RViz view:

```bash
./scripts/run_presentation_demo.sh 8
```

## Run Gazebo Recording Demo

```bash
./scripts/stop_demo_processes.sh
./scripts/run_gazebo_recording_lift_demo.sh 8
```

This is the current video-recording version. It opens Gazebo and replays a
uniform trajectory with drone mesh, payload, ropes, 3D obstacles, region
highlights, and delayed cable attach/detach.

If Gazebo looks stale, stop old processes first:

```bash
./scripts/stop_demo_processes.sh
```

## Rebuild Cases

Rebuild one case:

```bash
python3 tools/rebuild_cases.py --case demo6 --retries 12
```

Rebuild all standard cases:

```bash
./scripts/rebuild_standard_qp_demos.sh
```

The random seed sampling can change the generated regions and route. More
accepted regions usually give the route search better geometric choices.

## Verify Cases

```bash
./scripts/verify_standard_qp_demos.sh
```

This checks structural validity and paper-QP execution safety.

## Clean Generated Build Artifacts

Only do this when you want a fresh ROS build:

```bash
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --packages-select swarm_random_payload
source install/setup.bash
```

Do not delete `data/`, `src/`, `tools/`, `scripts/`, or `gazebo_pose_replay/`.

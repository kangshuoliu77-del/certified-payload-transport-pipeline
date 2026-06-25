# swarm_random_payload

ROS 2 package for the payload-transport pipeline.

This package is responsible for the runtime ROS/RViz layer. It does not create
IRIS regions by itself; it consumes generated case files from `data/` and
publishes the visualization/execution state.

## Package Layout

`launch/`
: ROS launch files. `case_payload_demo.launch.py` is the main algorithm viewer;
  `cinematic_payload_demo.launch.py` is the presentation RViz viewer.

`config/`
: Runtime YAML and RViz display configuration.

`data/`
: Package-installed copies of `demoN_case.json` and `demoN_map.json`.

`meshes/`, `materials/`
: Drone mesh and texture assets used by RViz/Gazebo visualization.

`swarm_random_payload/random_payload_node.py`
: ROS node that publishes the visual scene.

`swarm_random_payload/random_payload_model.py`
: Scenario model, case parsing, execution frames, and verification checks.

`swarm_random_payload/paper_qp_controller.py`
: Paper-style QP controller.

## Main Runtime Command

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select swarm_random_payload
source install/setup.bash
./scripts/run_demo.sh 8
```

Presentation RViz view:

```bash
./scripts/run_presentation_demo.sh 8
```

## ROS Concepts In This Package

- The package is discovered through `package.xml`, `setup.py`, and
  `resource/swarm_random_payload`.
- Launch files start the node with a selected case file.
- The node publishes visualization markers, pose arrays, paths, and task state
  topics for RViz.
- RViz is the main certified-planning viewer; Gazebo recording is exported from
  the same case through top-level `tools/` scripts.

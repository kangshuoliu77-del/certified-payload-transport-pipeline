# Code Reading Order

Read only the logic-bearing files first. Generated outputs and build folders can
wait.

## 1. Case Generation

`tools/generate_case.py`
: Builds IRIS regions, filters formation-feasible regions, computes bridge
  candidates, and creates symbolic states/transitions.

`tools/rebuild_cases.py`
: Reads `data/case_manifest.json` and rebuilds one or more demos. This is where
  retry behavior and case copying are coordinated.

## 2. Runtime Model And QP

`src/swarm_random_payload/swarm_random_payload/random_payload_model.py`
: Loads a case, builds the scenario, generates execution frames, and runs
  validation checks.

`src/swarm_random_payload/swarm_random_payload/paper_qp_controller.py`
: Implements the paper-style QP controller: convergence constraints, safety
  constraints, input bounds, and per-step solve logic.

## 3. ROS/RViz Layer

`src/swarm_random_payload/swarm_random_payload/random_payload_node.py`
: ROS 2 node. Publishes drones, payload, obstacles, IRIS regions, current/target
  regions, bridges, and trajectory markers.

`src/swarm_random_payload/launch/case_payload_demo.launch.py`
: Main launch entry for the RViz/ROS pipeline demo.

`src/swarm_random_payload/config/cinematic_payload_demo.rviz`
: RViz display configuration for the presentation view.

## 4. Gazebo Recording Layer

`tools/export_gazebo_scene.py`
: Converts the same case file into a Gazebo SDF world with 3D obstacles, task
  regions, drones, payload, ropes, and highlight geometry.

`tools/export_gazebo_trajectory.py`
: Exports the paper-QP execution as a Gazebo-friendly trajectory cache.

`tools/freeze_uniform_recording_trajectory.py`
: Converts the QP execution into a uniform-speed recording trajectory.

`tools/export_gazebo_pose_stream.py`
: Converts the uniform recording trajectory into a per-frame pose CSV for the
  Gazebo replay plugin. This is where the current cable timing and lift visual
  dynamics are handled.

`gazebo_pose_replay/PayloadPoseReplaySystem.cc`
: Gazebo system plugin that reads the pose CSV and updates model poses in
  process.

`scripts/run_gazebo_recording_lift_demo.sh`
: Main Gazebo recording command.

## 5. Files To Skip At First

`build/`, `install/`, `log/`
: colcon-generated files.

`out/`
: generated worlds, trajectories, snapshots, and temporary recording artifacts.

`.python_drake/`
: local dependency bundle.

`__pycache__/`, `*.pyc`
: Python cache files.

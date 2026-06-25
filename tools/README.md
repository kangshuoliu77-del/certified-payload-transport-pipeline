# Tools

Offline Python tools for building and exporting cases.

## Case Pipeline

`generate_case.py`
: Core generator. Produces IRIS regions, formation-feasible states, bridge
  certificates, route states, transitions, and case JSON.

`rebuild_cases.py`
: Manifest-driven wrapper for rebuilding standard demos.

`build_case_from_map.py`
: Build one case from one exported map JSON.

`verify_cases.py`
: Validate generated cases and paper-QP execution checks.

## Gazebo Export

`export_gazebo_scene.py`
: Export a case as a Gazebo SDF world.

`export_gazebo_trajectory.py`
: Export paper-QP execution frames for Gazebo.

`freeze_uniform_recording_trajectory.py`
: Convert execution frames into a uniform-speed recording trajectory.

`export_gazebo_pose_stream.py`
: Export per-frame model poses for the Gazebo replay plugin. This includes
  camera target, drone poses, payload, ropes, current/target/bridge highlights,
  start hold, and delayed cable attach/detach timing.

`replay_gazebo_trajectory.py`
: Older Python service-call replay path. Useful for reference, but the current
  recording demo uses the C++ plugin instead.

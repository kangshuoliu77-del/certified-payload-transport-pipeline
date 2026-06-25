# Scripts

These are the user-facing commands. Run them from the repository root unless a
script says otherwise.

## Main Entrypoints

`run_demo.sh`
: Run a standard case through the ROS/RViz pipeline.

`run_presentation_demo.sh`
: Cleaner RViz presentation view for advisor discussion.

`run_gazebo_recording_lift_demo.sh`
: Current polished Gazebo recording demo. This is the version with X500-style
  mesh, payload, ropes, delayed cable attach/detach, and stable replay.

`stop_demo_processes.sh`
: Stop stale ROS/Gazebo demo processes before rerunning a demo.

## Case Management

`rebuild_standard_qp_demos.sh`
: Rebuild all eight standard demo cases.

`verify_standard_qp_demos.sh`
: Verify all standard cases.

## Gazebo Variants

`run_gazebo_recording_demo.sh`
: Stable recording replay without the lift-visual timing adjustments.

`run_gazebo_cinematic_demo.sh`
: Older Gazebo cinematic path using Python replay service calls.

`run_gazebo_scene.sh`
: Static Gazebo scene only.

`run_gazebo_rviz_demo.sh`
: Gazebo scene plus RViz algorithm viewer.

`run_gazebo_flying_demo.sh`
: Experimental Gazebo dynamics/control path. Keep this separate from the stable
  recording demo.

`build_gazebo_pose_replay_plugin.sh`
: Build the C++ in-process Gazebo pose replay plugin.

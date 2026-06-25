#!/usr/bin/env bash
set -eo pipefail

# Stop only the processes that are started by this repository's demo scripts.
# Keeping this as a small script avoids stale RViz/Gazebo/bridge processes
# stacking markers or fighting over ROS/Gazebo topics between demo runs.

PATTERNS=(
  "run_gazebo_flying_demo.sh"
  "run_gazebo_cinematic_demo.sh"
  "swarm_random_payload.*/random_payload_demo"
  "random_payload_demo --ros-args"
  "swarm_random_payload.*/gazebo_velocity_tracker"
  "swarm_random_payload.*/gazebo_payload_tether"
  "swarm_random_payload.*/gazebo_live_overlay"
  "ros2 run swarm_random_payload gazebo_velocity_tracker"
  "ros2 run swarm_random_payload gazebo_payload_tether"
  "ros2 run swarm_random_payload gazebo_live_overlay"
  "tools/replay_gazebo_trajectory.py"
  "replay_gazebo_trajectory.py"
  "tools/export_gazebo_trajectory.py"
  "export_gazebo_trajectory.py"
  "ros_gz_bridge parameter_bridge"
  "/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge"
  "ign gazebo .*certified-payload-transport-pipeline"
  "ign gazebo server"
  "ign gazebo gui"
  "payload_cinematic_rviz"
)

for pattern in "${PATTERNS[@]}"; do
  while read -r pid args; do
    if [ -z "${pid:-}" ]; then
      continue
    fi
    if [ "$pid" = "$$" ] || [ "$pid" = "$PPID" ]; then
      continue
    fi
    case "$args" in
      *stop_demo_processes.sh*) continue ;;
      *"awk -v pat="*) continue ;;
      *"ps -eo pid=,args="*) continue ;;
    esac
    kill -INT "$pid" 2>/dev/null || true
    sleep 0.05
    if kill -0 "$pid" 2>/dev/null; then
      kill -TERM "$pid" 2>/dev/null || true
    fi
    sleep 0.05
    if kill -0 "$pid" 2>/dev/null; then
      kill -KILL "$pid" 2>/dev/null || true
    fi
  done < <(ps -eo pid=,args= | awk -v pat="$pattern" '$0 ~ pat {pid=$1; $1=""; sub(/^ /, "", $0); print pid, $0}')
done

sleep 0.3

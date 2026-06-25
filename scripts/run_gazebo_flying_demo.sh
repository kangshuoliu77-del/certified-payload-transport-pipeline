#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEMO="${1:-8}"

cat <<'EOF'
[dynamic] Gazebo physics/control prototype:
[dynamic]   X500 multicopter dynamics + velocity controller
[dynamic]   physical payload rigid body
[dynamic]   payload cable tensions through Gazebo link wrenches
EOF

case "$DEMO" in
  1|demo1) DEMO_NAME="demo1" ;;
  2|demo2) DEMO_NAME="demo2" ;;
  3|demo3) DEMO_NAME="demo3" ;;
  4|demo4) DEMO_NAME="demo4" ;;
  5|demo5) DEMO_NAME="demo5" ;;
  6|demo6) DEMO_NAME="demo6" ;;
  7|demo7) DEMO_NAME="demo7" ;;
  8|demo8) DEMO_NAME="demo8" ;;
  *)
    echo "Unknown demo: $DEMO" >&2
    echo "Use 1-8 or demo1-demo8." >&2
    exit 2
    ;;
esac

CASE_FILE="$ROOT_DIR/data/${DEMO_NAME}_case.json"
WORLD_FILE="$ROOT_DIR/out/gazebo/${DEMO_NAME}_x500_multicopter_world.sdf"
TRAJECTORY_FILE="$ROOT_DIR/out/gazebo/${DEMO_NAME}_paper_qp_trajectory.json"
ROS_LOG_DIR="${ROS_LOG_DIR:-$ROOT_DIR/log/ros}"
FLIGHT_ALTITUDE="${FLIGHT_ALTITUDE:-1.6}"
FRAME_STRIDE="${FRAME_STRIDE:-8}"
MAP_SCALE="${MAP_SCALE:-0.02}"
MAX_TRACK_STEP="${MAX_TRACK_STEP:-0.05}"
EVENT_STEPS="${EVENT_STEPS:-48}"
EVENT_HOLD_STEPS="${EVENT_HOLD_STEPS:-96}"
USE_FEEDFORWARD="${USE_FEEDFORWARD:-false}"
FEEDFORWARD_RADIUS="${FEEDFORWARD_RADIUS:-0.55}"
MAX_WAYPOINT_ADVANCE="${MAX_WAYPOINT_ADVANCE:-1}"
PROGRESS_SEARCH_WINDOW="${PROGRESS_SEARCH_WINDOW:-80}"
LOOKAHEAD_WAYPOINTS="${LOOKAHEAD_WAYPOINTS:-1}"
COMMAND_FRAME="${COMMAND_FRAME:-body}"
STARTUP_HOLD_SECONDS="${STARTUP_HOLD_SECONDS:-4.0}"
KP_XY="${KP_XY:-0.85}"
KP_Z="${KP_Z:-3.2}"
MAX_XY_SPEED="${MAX_XY_SPEED:-1.2}"
MAX_Z_SPEED="${MAX_Z_SPEED:-2.00}"
WAYPOINT_TOLERANCE="${WAYPOINT_TOLERANCE:-0.75}"
OBSTACLE_HEIGHT="${OBSTACLE_HEIGHT:-2.80}"
ENABLE_PAYLOAD_TETHER="${ENABLE_PAYLOAD_TETHER:-1}"

GAZEBO_PID=""
BRIDGE_PID=""
TRACKER_PID=""
TETHER_PID=""
OVERLAY_PID=""

cleanup() {
  for pid in "$OVERLAY_PID" "$TETHER_PID" "$TRACKER_PID" "$BRIDGE_PID" "$GAZEBO_PID"; do
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT INT TERM

if [ ! -f "$CASE_FILE" ]; then
  echo "Missing case file: $CASE_FILE" >&2
  exit 3
fi

source /opt/ros/humble/setup.bash
if [ ! -f "$ROOT_DIR/install/setup.bash" ]; then
  echo "Missing install/setup.bash. Build first:" >&2
  echo "  cd $ROOT_DIR" >&2
  echo "  source /opt/ros/humble/setup.bash" >&2
  echo "  colcon build --packages-select swarm_random_payload" >&2
  exit 4
fi
source "$ROOT_DIR/install/setup.bash"

mkdir -p "$ROS_LOG_DIR" "$(dirname "$WORLD_FILE")"
export ROS_LOG_DIR

python3 "$ROOT_DIR/tools/export_gazebo_multicopter_world.py" \
  --case "$CASE_FILE" \
  --out "$WORLD_FILE" \
  --map-scale "$MAP_SCALE" \
  --altitude "$FLIGHT_ALTITUDE" \
  --obstacle-height "$OBSTACLE_HEIGHT"

echo "[trajectory] exporting paper-QP execution frames before Gazebo starts"
python3 "$ROOT_DIR/tools/export_gazebo_trajectory.py" \
  --case "$CASE_FILE" \
  --out "$TRAJECTORY_FILE" \
  --map-scale "$MAP_SCALE" \
  --altitude "$FLIGHT_ALTITUDE" \
  --pickup-altitude 0.62 \
  --payload-drop 0.36 \
  --payload-ground-height 0.12 \
  --visual-rate 12.0 \
  --event-steps "$EVENT_STEPS" \
  --event-hold-steps "$EVENT_HOLD_STEPS" \
  --frame-stride "$FRAME_STRIDE" \
  --max-track-step "$MAX_TRACK_STEP"

echo "[gazebo] starting dynamic multicopter world paused"
if [ "${GAZEBO_HEADLESS:-0}" = "1" ]; then
  env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY -u ALL_PROXY -u all_proxy \
    ign gazebo -r -s "$WORLD_FILE" &
else
  env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY -u ALL_PROXY -u all_proxy \
    ign gazebo "$WORLD_FILE" &
fi
GAZEBO_PID="$!"

sleep 5

echo "[bridge] three X3 command and odometry topics"
ros2 run ros_gz_bridge parameter_bridge \
  "/X3_1/gazebo/command/twist@geometry_msgs/msg/Twist]ignition.msgs.Twist" \
  "/X3_2/gazebo/command/twist@geometry_msgs/msg/Twist]ignition.msgs.Twist" \
  "/X3_3/gazebo/command/twist@geometry_msgs/msg/Twist]ignition.msgs.Twist" \
  "/X3_1/enable@std_msgs/msg/Bool]ignition.msgs.Boolean" \
  "/X3_2/enable@std_msgs/msg/Bool]ignition.msgs.Boolean" \
  "/X3_3/enable@std_msgs/msg/Bool]ignition.msgs.Boolean" \
  "/world/payload_multicopter/wrench@ros_gz_interfaces/msg/EntityWrench]ignition.msgs.EntityWrench" \
  "/model/x3_1/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry" \
  "/model/x3_2/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry" \
  "/model/x3_3/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry" \
  "/model/payload/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry" &
BRIDGE_PID="$!"

sleep 2

if [ "$ENABLE_PAYLOAD_TETHER" = "1" ] || [ "$ENABLE_PAYLOAD_TETHER" = "true" ]; then
  echo "[tether] applying physical payload cable tensions through Gazebo wrenches"
  ros2 run swarm_random_payload gazebo_payload_tether \
    --ros-args \
    -p case_file:="$CASE_FILE" \
    -p map_scale:="$MAP_SCALE" \
    -p vehicle_count:=3 \
    -p wrench_topic:=/world/payload_multicopter/wrench \
    -p rope_rest_lengths:="0.72,0.76,0.80" \
    -p rope_stiffness:=12.0 \
    -p rope_damping:=3.5 \
    -p max_tension:=3.2 \
    -p tension_ramp_seconds:=4.0 \
    -p apply_drone_reaction:=false \
    -p attach_slack:=1.00 \
    -p attach_max_drone_height:=1.05 \
    -p detach_distance:=1.75 \
    -p ground_height:=0.11 \
    -p rate_hz:=80.0 \
    -p status_period:=2.0 &
  TETHER_PID="$!"
else
  echo "[tether] disabled by default; set ENABLE_PAYLOAD_TETHER=1 for the tension-only physics prototype"
fi

sleep 1

echo "[tracker] following ${DEMO_NAME} paper-QP execution frames with Gazebo velocity control"
ros2 run swarm_random_payload gazebo_velocity_tracker \
  --ros-args \
  -p case_file:="$CASE_FILE" \
  -p trajectory_file:="$TRAJECTORY_FILE" \
  -p map_scale:="$MAP_SCALE" \
  -p target_altitude:="$FLIGHT_ALTITUDE" \
  -p vehicle_count:=3 \
  -p trajectory_mode:=trajectory_file \
  -p command_frame:="$COMMAND_FRAME" \
  -p frame_stride:="$FRAME_STRIDE" \
  -p use_feedforward:="$USE_FEEDFORWARD" \
  -p feedforward_activation_radius:="$FEEDFORWARD_RADIUS" \
  -p synchronize_vehicles:=true \
  -p startup_hold_seconds:="$STARTUP_HOLD_SECONDS" \
  -p max_waypoint_advance_per_tick:="$MAX_WAYPOINT_ADVANCE" \
  -p progress_search_window:="$PROGRESS_SEARCH_WINDOW" \
  -p lookahead_waypoints:="$LOOKAHEAD_WAYPOINTS" \
  -p kp_xy:="$KP_XY" \
  -p kp_z:="$KP_Z" \
  -p max_xy_speed:="$MAX_XY_SPEED" \
  -p max_z_speed:="$MAX_Z_SPEED" \
  -p waypoint_tolerance:="$WAYPOINT_TOLERANCE" \
  -p status_period:=2.0 &
TRACKER_PID="$!"

sleep 1

echo "[overlay] live rope visuals and certified current/target/bridge highlights"
ros2 run swarm_random_payload gazebo_live_overlay \
  --ros-args \
  -p case_file:="$CASE_FILE" \
  -p trajectory_file:="$TRAJECTORY_FILE" \
  -p world:=payload_multicopter \
  -p vehicle_count:=3 \
  -p rope_rest_lengths:="0.86,0.90,0.93" \
  -p attach_slack:=0.82 \
  -p ground_height:=0.11 \
  -p rate_hz:=8.0 \
  -p timeout_ms:=500 \
  -p status_period:=5.0 &
OVERLAY_PID="$!"

sleep 2

echo "[gazebo] unpausing physics after tracker startup"
ign service -s /world/payload_multicopter/control \
  --reqtype ignition.msgs.WorldControl \
  --reptype ignition.msgs.Boolean \
  --timeout 3000 \
  --req 'pause: false' >/dev/null || {
    echo "[warn] failed to unpause Gazebo through service; press Play in Gazebo GUI" >&2
  }

wait "$GAZEBO_PID"

# Cinematic ROS/RViz Demo

This demo profile is meant for paper videos and advisor-facing previews.  It
keeps the planner/controller unchanged and only improves the ROS/RViz
presentation layer.

## Design Style

The profile follows the common ZJU FAST-Lab / Fei Gao group demonstration
style: ROS nodes publish planner state, TF, and visualization markers, while
RViz is used as the main high-quality planner viewer.  This is intentionally
lighter than a full physics simulator and is better suited for repeatable
multi-robot planning videos.

Reference style:

- EGO-Planner: https://github.com/ZJU-FAST-Lab/ego-planner
- EGO-Planner-swarm: https://github.com/ZJU-FAST-Lab/ego-planner-swarm
- EGO-Planner-v2: https://github.com/ZJU-FAST-Lab/EGO-Planner-v2

The quadrotor mesh used here is a PX4/X500-style visualization asset, not a
mesh copied from the Fei Gao group repositories.  The borrowed idea is the
presentation style: a planner-centric ROS/RViz demo with clear map, trajectory,
vehicle, and planning-state markers.

## What It Shows

- Certified IRIS regions as translucent green polytopes.
- Current certified region as a blue outline.
- Adjacent target region as a cyan outline.
- Active bridge certificate as a yellow outline.
- HOME / PICK / DROP / RETURN task markers.
- 3D obstacle blocks.
- Realistic PX4/X500-style quadrotor meshes.
- Payload cube and suspension lines during loaded transport.
- Smooth `payload_follow` TF for RViz third-person camera following.
- Optional cinematic HUD showing mode, formation, current region, bridge, and
  next region.

## Run

Build once:

```bash
cd certified-payload-transport-pipeline
source /opt/ros/humble/setup.bash
colcon build --packages-select swarm_random_payload
```

Run demo 8 with RViz:

```bash
cd certified-payload-transport-pipeline
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/run_cinematic_demo.sh 8
```

Run another demo:

```bash
./scripts/run_cinematic_demo.sh 6
```

The HUD is off by default for a clean recording.  Enable it for an explanatory
shot:

```bash
ros2 launch swarm_random_payload cinematic_payload_demo.launch.py show_cinematic_hud:=true
```

Capture the current RViz window as a PNG:

```bash
./scripts/capture_cinematic_screenshot.sh
```

The screenshot helper raises the RViz window, captures it, and writes a PNG to
`screenshots/`.  It is useful for quickly checking whether the map, certified
regions, bridge outline, task markers, and vehicle meshes are visible before
recording a video.

## Record

Record demo 8 for 45 seconds:

```bash
cd certified-payload-transport-pipeline
./scripts/record_cinematic_demo.sh 8 45
```

The video is written to:

```text
recordings/demo8_<timestamp>.mp4
```

On X11 the script uses `ffmpeg`; on Wayland it uses `wf-recorder` when
available.  If neither is installed:

```bash
sudo apt install ffmpeg
```

If the captured area is wrong on X11, set:

```bash
RECORD_RESOLUTION=1920x1080 RECORD_OFFSET=0,0 ./scripts/record_cinematic_demo.sh 8 45
```

RViz requires a real desktop display.  If a remote/sandboxed terminal cannot
open RViz or DDS sockets, run the same command in a normal local desktop
terminal.

Run the cinematic node without opening RViz:

```bash
ros2 launch swarm_random_payload cinematic_payload_demo.launch.py start_rviz:=false
```

## RViz Camera

The RViz config uses:

```text
View Type: ThirdPersonFollower
Target Frame: payload_follow
Distance: 8.0
Pitch: 0.72
Yaw: 0.0
```

If the camera is too tight, increase `Distance`.  If the scene is too flat,
increase `Pitch`.  If the camera feels shaky, lower:

```bash
follow_position_alpha:=0.10 follow_yaw_alpha:=0.06
```

If it feels delayed, raise:

```bash
follow_position_alpha:=0.22 follow_yaw_alpha:=0.14
```

## Important Notes

The quadrotor mesh is only a visualization asset.  It does not change the QP,
IRIS regions, bridge construction, certified formation states, or collision
checks.  The safety-critical geometry remains the formation envelope and the
case checks.

A concise explanation to give an advisor:

```text
The demo follows the ROS/RViz planner-visualization style used by FAST-Lab
quadrotor planning projects.  The X500-style vehicle mesh is only a visual
asset; the planning, IRIS certificates, bridge certificates, and QP controller
are still from our payload-transport pipeline.
```

# BPX ROS 2 Workspace

ROS 2 Humble (Ubuntu 22.04) workspace for the MirrorMe Black Panther X (BPX)
quadruped. Developed against the official SDK:
https://github.com/mirrormerobotics/bpx_sdk_open (cloned at
`../bpx_sdk_open/`).

## Layout

```
ros2_ws/
  src/
    bpx_description/   URDF (12 joints, SDK names/order) + Insight 9 camera frames
    bpx_bridge/        SDK wrapper node: telemetry out, cmd_vel + services in
    bpx_safety/        Depth obstacle arbiter gating cmd_vel (stop/slow modes)
```

Planned packages (see ../RESEARCH.md roadmap):
`bpx_perception` (Insight 9 detection/SLAM consumption),
`bpx_voice` (wake word → Whisper → LLM → Piper).

## Prerequisites

```bash
# ROS 2 Humble (Jetson / Ubuntu 22.04)
sudo apt install ros-humble-desktop ros-humble-xacro ros-humble-joint-state-publisher-gui

# BPX SDK Python bindings (from the local clone)
pip3 install --upgrade pip
pip3 install ../bpx_sdk_open          # compiles the native extension (needs py3.8+, C++17)
python3 -c "import bpx_sdk; print('ok')"
```

Without `bpx_sdk` installed the bridge runs in **MOCK mode**: it publishes
placeholder telemetry and logs commands, so the whole ROS 2 stack can be
exercised on a desk with no robot.

## Build

```bash
cd ~/bpx-project/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
# Bridge + robot_state_publisher (mock mode if SDK missing / robot absent)
ros2 launch bpx_bridge bpx_bringup.launch.py

# Real robot over RJ45 (default 10.21.20.1) or WiFi AP (10.21.40.1)
ros2 launch bpx_bridge bpx_bringup.launch.py robot_ip:=10.21.40.1
```

Inspect the robot without walking:

```bash
ros2 topic echo /bpx_bridge/joint_states
ros2 topic echo /diagnostics
rviz2   # RobotModel + /bpx_bridge/odom + TF
```

## Safety model (read before enabling motion)

1. `bpx_bridge` starts **disarmed**. All motion services and `cmd_vel` are
   rejected until `enable_motion` is called:
   `ros2 service call /bpx_bridge/enable_motion std_srvs/Trigger`
2. The SDK zeroes all commands on every connect/reconnect and never replays
   targets; the bridge mirrors this by disarming on any observed link loss.
3. A `cmd_vel` watchdog sends zero velocity if the command stream goes stale
   (`cmd_vel_timeout`, default 0.5 s).
4. `zero_positions` requires feet, shanks, and knee joints on the ground
   (per the SDK README).

## Safety chain

Command flow with the safety arbiter in the loop:

```
teleop / voice agent / autonomy ──▶ cmd_vel_unsafe ──▶ bpx_safety ──▶ cmd_vel ──▶ bpx_bridge
```

`bpx_safety` checks the Insight 9 depth image for obstacles in a forward
zone (distance + horizontal cone + height band that excludes floor and
ceiling). When triggered it zeroes (`mode: stop`) or scales down
(`mode: slow`) the command, and only releases after the zone has been
clear for `clear_time` seconds. State is on `~/active` (Bool) and
`/diagnostics`. If you bypass it (publishing `cmd_vel` directly), the
bridge's `cmd_vel` watchdog and arm/disarm logic remain as the last line
of defense, but you lose obstacle protection — don't do this while
walking.

Test the gate without a camera by replaying a rosbag of depth images into
`/insight9/depth` (or point `depth_topic` at any depth source) and running:

```bash
ros2 topic pub -r 10 /cmd_vel_unsafe geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
ros2 topic echo /cmd_vel          # should match while zone is clear
ros2 topic echo /bpx_safety/active
```

## Interface summary

| Topic/Service | Type | Notes |
|---|---|---|
| `~/joint_states` | sensor_msgs/JointState | 12 joints, SDK names/order |
| `~/imu` | sensor_msgs/Imu | 200 Hz native rate on robot |
| `~/odom` | nav_msgs/Odometry | SDK leg odometry (50 Hz native) + TF odom→base_link |
| `~/motion_state`, `~/gait` | std_msgs/String | e.g. `Motion`, `Walk` |
| `/diagnostics` | DiagnosticArray | battery, temps, connection, arm state |
| `cmd_vel` | geometry_msgs/Twist | clamped to gait limits, watchdog-guarded |
| `~/active` (bpx_safety) | std_msgs/Bool | true while the obstacle gate is blocking |
| `cmd_vel_unsafe` (bpx_safety in) | geometry_msgs/Twist | gate input from command sources |
| `~/stand_up` `~/sit_down` `~/damping` `~/walk` `~/run` `~/pace` `~/bound` `~/pronk` `~/bipedal` `~/inv_bipedal` `~/left_flip` `~/right_flip` `~/jump_{up,front,back,left,right}` | std_srvs/Trigger | motion commands |
| `~/zero_positions` `~/enable_motion` `~/reconnect` | std_srvs/Trigger | safety / lifecycle |

## Notes

- `bpx_description` now carries the **official BPX model** (kinematics,
  inertias, meshes) from the MirrorMe BPX Discord package. Joint
  names/order match the SDK, so bridge `joint_states` drive it directly.
- Community additions in the URDF: Insight 9 camera mount frames — the
  mount pose is a **placeholder** to calibrate against the real
  perception head.
- The "root link has inertia" KDL warning at startup is expected (the
  official model defines torso inertia) and harmless for visualization.
- SDK protocol has no authentication: keep the robot network isolated.
- Gait velocity limits (`bpx_sdk_open` README): Walk `[1.5, 0.5, 2.0]`,
  Bipedal `[0.8, 0.5, 1.5]`, WalkPhase `[1.5, 1.0, 2.0]`, Running
  x∈[-1.5, 3.0], y∈[-1, 1], yaw∈[-2, 2]. The robot additionally reduces
  lateral/yaw authority as forward speed increases.

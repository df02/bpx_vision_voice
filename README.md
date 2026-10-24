# BPX Vision & Voice Stack

[![CI](https://github.com/df02/bpx_vision_voice/actions/workflows/ci.yml/badge.svg)](https://github.com/df02/bpx_vision_voice/actions/workflows/ci.yml)

An open-source ROS 2 workspace for the [MirrorMe **Black Panther X (BPX)**](https://www.mirrorme.bot/)
quadruped robot: vision, safety, and (planned) a fully-local voice assistant —
no cloud services required.

> 🚧 **Project status:** early development. The scaffold builds and runs in
> **mock mode** (no robot needed). Hardware bring-up sections are written
> against the official BPX SDK v1.0.9 and will be validated as robots ship.

## What it does

| Package | Purpose | Status |
|---|---|---|
| [`bpx_bridge`](ros2_ws/src/bpx_bridge) | Connects ROS 2 to the BPX over Ethernet/WiFi using the official [`bpx_sdk_open`](https://github.com/mirrormerobotics/bpx_sdk_open): publishes joint states, IMU, and leg odometry; accepts `cmd_vel` and motion commands (stand, sit, gaits, jumps) via topics/services | ✅ scaffold, hardware validation pending |
| [`bpx_safety`](ros2_ws/src/bpx_safety) | Depth-camera obstacle arbiter: stops or slows the robot when something is in a forward zone. Sits between your commands and the bridge | ✅ scaffold, tuning on hardware pending |
| [`bpx_description`](ros2_ws/src/bpx_description) | URDF model of the BPX (12 joints, SDK joint names) + Intel/Looper Insight 9 camera frames | ✅ placeholders to be refined |
| `bpx_perception` | Consume Insight 9 onboard detection / SLAM into a world model | 🔜 planned |
| `bpx_voice` | Wake word → local Whisper STT → local LLM (tool calling) → Piper TTS, wired to the bridge services | 🔜 planned |

## Hardware used in this project

- **Robot:** MirrorMe Black Panther X (Allwinner A733-based; used as-is via its network SDK)
- **Compute:** NVIDIA Jetson Orin NX Super 16 GB (primary brain), Raspberry Pi 5 (planned: voice front-end)
- **Camera:** Looper Robotics **Insight 9** spatial AI camera (ROS 2 Humble, on-device SLAM)

Other BPX-compatible depth cameras (RealSense D435/D555, Manifold Odin1) can be
substituted — `bpx_safety` works with any depth image topic.

## Architecture

```
                  ┌──────────────────────────── Jetson (ROS 2 Humble) ───────────────────────────┐
                  │                                                                              │
 voice agent ──▶ cmd_vel_unsafe ──▶ bpx_safety ──▶ cmd_vel ──▶ bpx_bridge ──TCP/UDP──▶ BPX robot │
 (planned)          (teleop etc.)       ▲                                  ▲                     │
                                    Insight 9                            └── leg odom / IMU ──┐ │
                                    depth + detections                                        ▼ │
                                                                                  motion control / telemetry
```

## Quick start

Impatient? Three commands once ROS 2 Humble is installed:

```bash
pip3 install bpx_sdk_open_src/          # official BPX SDK (or skip → mock mode)
cd ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch bpx_bridge bpx_bringup.launch.py
```

No robot needed to try it — without the SDK the stack runs in **mock mode**
and publishes placeholder telemetry you can watch in RViz.

**→ Follow [`SETUP.md`](SETUP.md) for the complete, beginner-friendly,
step-by-step guide** (Jetson setup, ROS 2 install, robot networking, safety
chain, teleop, troubleshooting).

## Safety first (please read)

- A quadruped with 8 kg payload capability and 4 m/s top speed can hurt people
  and itself. **Test with the robot off the ground or on a tether first.**
- The bridge starts **disarmed**. Nothing moves until you explicitly call the
  `enable_motion` service after connecting.
- `bpx_safety` is a convenience layer, not a certified safety system. Keep your
  hands on an E-stop (the BPX app/remote) during any testing.
- The BPX network protocol has **no authentication** — keep the robot on an
  isolated network.

## Documentation

- [`SETUP.md`](SETUP.md) — step-by-step setup from a blank Jetson to a walking robot
- [`ros2_ws/README.md`](ros2_ws/README.md) — package/interface reference
- [`RESEARCH.md`](RESEARCH.md) — hardware research notes (BPX SDK, cameras, SoC, voice stack)

## Getting help & contributing

- 🐞 Problems or questions: [open an issue](https://github.com/df02/bpx_vision_voice/issues)
- 🔧 Fixes and improvements: pull requests welcome — especially hardware
  validation results (joint limits, camera topics, tuning numbers)
- 📖 Start with [`SETUP.md`](SETUP.md) if you're new

## Credits & license

Built on the official [`mirrormerobotics/bpx_sdk_open`](https://github.com/mirrormerobotics/bpx_sdk_open)
(Apache-2.0). This workspace is licensed Apache-2.0. BPX is a product of
MirrorMe Technology — this project is community-driven and unaffiliated.

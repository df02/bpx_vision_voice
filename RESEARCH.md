# Black Panther X — Vision & Voice Development Research

*Compiled Oct 2026. Sources: mirrorme.bot, mirrormetech.com, linux-sunxi.org, CNX Software, looper-robotics.com, realsenseai.com, NVIDIA developer docs, and the official **`mirrormerobotics/bpx_sdk_open`** GitHub repo (cloned locally at `bpx_sdk_open/`, SDK v1.0.9, docs dated 2026-09-08).*

**Status update:** We already own the **Insight 9** camera. The BPX SDK is public, network-based, and ships Python bindings — see §3. A starter ROS 2 Humble workspace now exists at **`ros2_ws/`** (`bpx_description` URDF + `bpx_bridge` SDK wrapper node with mock mode — see `ros2_ws/README.md`).

---

## 1. The Platform: MirrorMe Black Panther X (BPX)

Open, industrial-grade modular quadruped. 200+ components, 3D-printable shells, open interfaces/code, "open perception head" for sensors.

**Confirmed specs (vendor marketing):** 12 DOF, 4 m/s top speed, 8 kg payload, ~18 cm stair height, 720° side flip. Price ~$1,200–$1,500. Models: `BPX`, `BPX Pro`, `BPW` (per `RobotModel` enum in SDK). BPX supports bipedal/inverted stance and 5-direction jumps; **BPX Pro does not support bipedal stance**. Vendor also offers "OpenClaw" natural-language control ($100/yr subscription) — our local voice+LLM stack is the self-hosted alternative.

The robot itself runs Linux (SDK performs clock/RTC writes to `CLOCK_REALTIME`; NTP state is reported), controlled over Ethernet/WiFi — see §3.

---

## 2. Onboard SoC: Allwinner A733MX-HN3

| Item | Detail |
|---|---|
| CPU | 2× Cortex-A76 @ 2.0 GHz + 6× Cortex-A55 @ 1.8 GHz + RISC-V E902 |
| NPU | Optional 3 TOPS (config-dependent) |
| RAM | LPDDR4/4X/5, up to 16 GB |
| Linux support | BSP available (Radxa Cubie A7A, ArmSoM Sige6); mainline kernel support in progress |

**Implication:** The A733 hosts the robot's locomotion controller. We don't build on it directly — the SDK gives us full network access to everything we need (§3). Perception and LLM run on the Jetson. The A733 stays sealed; we only interact over its network API.

---

## 3. BPX SDK (`mirrormerobotics/bpx_sdk_open`) — KEY FINDINGS

Local clone: `~/bpx-project/bpx_sdk_open/`. Apache 2.0. Lightweight C++ SDK with **Python bindings** (`pip install .`, wheels for linux x86_64/aarch64, macOS arm64, Windows). Works on **Ubuntu 22.04+** — a perfect match for ROS 2 Humble on Jetson (aarch64 `.so` provided).

### 3.1 Architecture: network client, no serial bridge
- TCP control channel + UDP state-upload streams to our host.
- IPs: **`10.21.20.1`** RJ45 direct · **`10.21.40.1`** robot WiFi AP · DHCP when robot joins Station WiFi.
- Client UDP ports: state `9873`, joint state `7895` (constants in `include/bpx_sdk_config.h`).
- **So the Jetson can simply sit on the robot's network (ideally wired RJ45) and run both the SDK client and ROS 2.** No microcontroller or serial gateway needed.

### 3.2 Three control layers (classes, all with state read APIs)
| Class | Use |
|---|---|
| `RequestRobotState` | Telemetry only |
| `MotionLevelControl` (inherits state) | High-level: `setStandUp()`, `setSitDown()`, `setDamping()`, `setWalk()`, `setRunning()`, `setVelocity(x, y, yaw)`, `setBipedal()`, `setInvBipedal()`, `setPace()`, `setBound()`, `setPronk()`, `setLeftFlip()/setRightFlip()`, 5× jump APIs |
| `JointLevelControl` (inherits state) | 12-DOF `torque = kp·(pos_err) + kd·(vel_err) + tff`; `setJointCommand(kp, pos, kd, vel, tff)`; high-rate joint/IMU feedback in `DevelopingState` |

### 3.3 State feedback (rates = min(requested, native cap))
- Joints **1000 Hz**, IMU **200 Hz**, **leg odometry 50 Hz** (body velocity, world position, orientation quat, angular velocity — ideal for `robot_localization`/Nav2), motion state 10 Hz, battery/temp 1 Hz. Upload rate settable 1–200 Hz (default 100).
- Available: joint pos/vel/torque, IMU rpy/quat/acc/omega, battery %/current, motor+driver temps (12×), charger inputs, timestamps per group.
- **Joint order** (from `motion_types.h`): `fl_hip_roll, fl_hip_pitch, fl_knee, fr_hip_roll, fr_hip_pitch, fr_knee, hl_*, hr_*` — direct URDF input.

### 3.4 Safety & connection semantics (important)
- Robot IP default **`10.21.20.1`**; ping before connecting.
- Reconnect w/ bounded backoff (100 ms–2 s). **On every (re)connect the robot zeroes command state** — motion commands must be re-issued explicitly; partial joint setters return `false` until a full frame is sent. Good — no command replay after Wi-Fi loss.
- `setZeroPositionsFlag()` requires feet/shanks/joints on ground (examples zero at startup).
- Motion command rate settable (`setMotionCommandRate(50)`).
- **Time sync is automatic** on connect (offsets >10 min applied to robot clock+RTC; error codes 9/10 for NTP/RTC issues). Protocol has **no authentication** — only use on trusted networks; disable `sdk_time_sync_enabled` on untrusted ones.
- `ControlMode` feedback: `RemoteControl=1` (app/RC) vs **`Navigator=2`** — a navigation mode exists; likely how an external computer (us) takes command authority. Must be confirmed experimentally.

### 3.5 Example worth running first
`motion_level_control_server_example` (C++ or Python): owns the SDK connection, accepts text commands on `127.0.0.1:50051` (`status`, `stand`, `sit`, `velocity X Y YAW`, `gait NAME`, `jump up|front|back|left|right`). Great for early bench testing and even as a stop-gap command interface from the voice/LLM layer.

### 3.6 Gait velocity limits (`[x, y, yaw]` m/s, rad/s)
Walk `[1.5, 0.5, 2.0]` · Bipedal `[0.8, 0.5, 1.5]` · WalkPhase `[1.5, 1.0, 2.0]` · Running x∈[-1.5, 3.0], y∈[-1, 1], yaw∈[-2, 2]. Lateral/yaw authority shrinks as |x| grows.

---

## 4. Camera: Looper Robotics Insight 9 (owned)

- Built around **D-Robotics RDK X5**: 8× A55 @ 1.5 GHz + **10 TOPS BPU** — depth, **V-SLAM**, and object detection run **on-device**.
- RGB: 8.4 MP Sony IMX415, 4K, 188° diag / 157° horiz FOV, 0.01 lux.
- Stereo depth: 2× SmartSens SG0132 global shutter, 1280×960@60 fps, 100 mm baseline, 0.1–30 m (optimal **0.2–8 m**), <2% error close range, <8% @10 m.
- IMU: Bosch BMI088, 24 g-rated — designed for legged-locomotion vibration.
- **ROS 2 Humble preinstalled**; USB-C (5 V/2 A, ~5 W) with USB virtual Ethernet; **Cyclone DDS** — topics appear directly on the host, no driver build needed. SDK for Ubuntu 22.04 (recommended), Win11, macOS.
- 176 g, 129×33.8×35 mm, passive aluminum, −20…85 °C.
- Output: pixel-synced raw view, rectified view, neural depth map; onboard SLAM pose; object detection.

**Plan:** the Insight 9 is the primary and only camera needed for Phase 1–2. Revisit adding the **Odin1** (~$999, 70 m dToF, MindSLAM, ROS 2 driver) later only if long-range mapping/autonomy demands it; the **D435/D435i/D555** remain cheap fallbacks or a rear-facing secondary. RealSense spun out of Intel (Jul 2025) — D-series still supported.

---

## 5. Compute Boards

| Board | CPU | AI perf | Role |
|---|---|---|---|
| Raspberry Pi 5 (16 GB) | 4× A76 @ 2.4 GHz | none (optional Hailo) | Bench/dev machine; optional audio front-end |
| Jetson Orin Nano Super 8 GB | 6× A78AE | 67 TOPS sparse INT8 (JetPack 6.2+), 102 GB/s | Dev bench; STT/TTS; 3–4B LLM |
| **Jetson Orin NX Super 16 GB** | 8× A78AE | 157 TOPS sparse INT8 (Super: 77 GPU + 80 on 2×DLA), 102 GB/s | **Primary brain: vision AND voice (single-board)** |

**Compute decision (Oct 2026): voice runs on the Jetson too — no RPi in the loop.** Rationale:
- The Insight 9 offloads the *continuous* vision work (depth/SLAM/detection on its 10 TOPS BPU), so the Jetson GPU is mostly idle between voice bursts. Voice is bursty and small: wake word (a few % of one core) + whisper.cpp on GPU (sub-second for 3 s clips) + 4B LLM Q4 (~20–40 tok/s decode, ~4.5 GB incl. KV cache) + Piper TTS (~5–10% CPU).
- Memory budget (16 GB): OS+ROS ≈ 2 GB, 4B LLM ≈ 4.5 GB, Whisper ≈ 0.5 GB, TensorRT vision ≈ 1 GB → ~8 GB used, ample headroom. An 8B Q4 LLM also fits.
- Measured Orin NX 16 GB Super LLM speeds (JetPack 6.2, llama.cpp/MLC): Llama 3.1 8B Q4 ≈ 18–23 tok/s; Qwen 2.5 7B ≈ 20–23.5; 3B ≈ 40+; 1.7B ≈ 60–69; 14B ≈ 7 (practical ceiling). Note: Super Mode raises clocks but **not** memory bandwidth on the NX (stays 102 GB/s) — decode is bandwidth-bound.
- Raspberry Pi **AI HAT+ 2** (Hailo-10H, 40 TOPS INT4, 8 GB dedicated RAM, ~$130–200, Jan 2026) runs ≤6–7B LLMs/VLMs/Whisper with Ollama integration — but duplicates what the Jetson does better (larger models, CUDA/TensorRT/VLM ecosystem, same box as vision). **Not added.** Revisit only if: (a) a resident continuous VLM is needed alongside a large LLM, (b) battery runtime demands a ~10 W always-on audio path, or (c) redundancy is desired.
- Network: Jetson ↔ BPX via RJ45 (`10.21.20.1` net); Insight 9 ↔ Jetson via USB-C virtual Ethernet. ROS 2 spans both links with Cyclone DDS.

---

## 6. ROS 2

- **ROS 2 Humble on Ubuntu 22.04 (JetPack 6.x)** — matches Jetson support *and* the Insight 9's preinstalled distro *and* the SDK's Ubuntu 22.04 requirement. Humble EOL May 2027 → plan Jazzy migration later.
- Cyclone DDS everywhere (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`).
- Planned nodes:
  1. **`bpx_bridge`** — wraps the SDK (Python bindings to start; C++ if we need >200 Hz joint streaming). Publishes: `joint_states` (12 DOF, names from §3.3), `sensor_msgs/Imu` (200 Hz), `nav_msgs/Odometry` (leg odom, 50 Hz), battery/temp diagnostics, motion state/gait. Subscribes: `cmd_vel` → `setVelocity()` (clamped to gait limits), gait/action services (`stand`, `sit`, `damping`, `walk`, `run`, `jump_*`), joint command topic. Handle **reconnect-zeroing** semantics (§3.4) inside the bridge: on reconnect, hold until operator/stack re-arms.
  2. **`insight9_perception`** — consumes Insight 9 topics (RGB, rectified, depth, SLAM pose, detections); TF tree: `insight9_optical → base_link`; publish detected-object markers + `vision_msgs`.
  3. **`safety_arbiter`** — depth-based obstacle stop: Insight 9 depth in a forward cone below threshold → zero `cmd_vel` / `setDamping()` on hazard.
  4. **`voice_agent`** (Phase 3) — wake word → Whisper → LLM w/ tool calling → Piper; tools call `bpx_bridge` services and ask `insight9_perception` about the scene.
- URDF: 12 joints (names/order known), fixed/revolute placeholders for legs + camera mount; `robot_state_publisher`.

---

## 7. Suggested Phased Roadmap

**Phase 0 — Bench bring-up (no walking)**
- Clone SDK (done: `bpx_sdk_open/`); build examples; `pip install ./bpx_sdk_open` on Jetson and dev machine
- Ping `10.21.20.1`, run `request_robot_state_example` — confirm telemetry (joints, IMU, odom, battery)
- Run `motion_level_control_server_example` → `stand`, `sit`, small `velocity` commands on a test stand/tether
- Flash Jetson: JetPack 6.2+, Super Mode, ROS 2 Humble + Cyclone DDS; install Insight 9 SDK, verify topics appear
- Probe `ControlMode` / Navigator switching behavior

**Phase 1 — Vision foundation**
- Mount Insight 9 on perception head (USB-C power from Jetson or BPX rail)
- TF: camera extrinsics vs `base_link`; verify SLAM pose vs leg odom (compare with IMU + odom in RViz2)
- rosbag2 capture while walking (walk, then pace/bound/running) — check vibration, motion blur, USB stability at speed

**Phase 2 — Perception & `bpx_bridge`**
- Build on the existing `ros2_ws/bpx_bridge` scaffold: validate joint/IMU/odom values against the SDK examples, then teleop via ROS 2 `cmd_vel`
- `ros2_ws/bpx_safety` scaffold ready: depth obstacle gate (`cmd_vel_unsafe` → gate → `cmd_vel`); confirm Insight 9 topic names and calibrate the height band / stop distance on hardware
- Object/person detection from Insight 9 onboard output (10 TOPS BPU) or TensorRT on Jetson
- `safety_arbiter`: depth-based stop zones
- Sensor fusion experiment: leg odom + IMU (`robot_localization`), Insight 9 SLAM pose as third source

**Phase 3 — Voice loop**
- Mic + speaker on RPi 5 (or Jetson USB): wake word → **whisper.cpp** → **Ollama/llama.cpp w/ tool calling** (Qwen3-4B or Llama-3.1-8B Q4) → **Piper TTS**
- LLM tools map to `bpx_bridge` services + scene queries; e.g. "walk forward slowly", "sit", "what do you see?"

**Phase 4 — Integration**
- VLM on camera feed (Qwen2.5-VL / VILA via TensorRT-LLM) for scene Q&A; persistent memory; end-to-end latency tuning

---

## 8. Open Questions / Risks

1. **Navigator control mode (§3.4)** — how does an external host take command authority vs. the RC/app? Test on hardware early.
2. **DevelopingState** requirement for high-rate joint feedback — how is it entered, and is it safe alongside motion?
3. **Insight 9 connectivity details** — USB virtual Ethernet addressing, whether topics stream over that link reliably at 60 fps; SDK maturity.
4. **Power budget** — Orin NX (~25 W) + Insight 9 (5 W) + RPi 5 from BPX rails; confirm payload power specs with MirrorMe.
5. **Vibration at 4 m/s** — global shutter + BMI088 help, but validate mounting stiffness; rosbag early.
6. **Humble EOL May 2027** — plan Jazzy migration.
7. **SDK protocol has no authentication** — keep the robot network isolated (dedicated subnet/USB-Ethernet link between Jetson and BPX is an option).
8. SDK is early-stage ("more APIs and state fields will be added") — pin to v1.0.9, watch upstream `mirrormerobotics/bpx_sdk_open` for updates.

---

## 9. Key Links

- BPX: https://www.mirrorme.bot/ · https://www.mirrormetech.com/en/product_4.html
- **BPX SDK: https://github.com/mirrormerobotics/bpx_sdk_open** (local: `~/bpx-project/bpx_sdk_open/`)
- Insight 9: https://looper-robotics.com/ (+ developer guide & hardware docs)
- A733: https://linux-sunxi.org/A733 · https://sbcwiki.com/docs/soc-manufacturers/allwinner/a733/
- RealSense (fallback): https://realsenseai.com/ · D555: https://realsenseai.com/products/d555-poe/
- Jetson Super Mode: NVIDIA blog "JetPack 6.2 Brings Super Mode to Jetson Orin Nano and NX"
- Voice/LLM refs: "Building a Local Robot Brain on Jetson Orin Nano Super" (thomasthelliez.com); NVIDIA forum: Open Voice OS on Jetson Orin Nano

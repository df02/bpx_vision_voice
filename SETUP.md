# BPX Vision & Voice Stack — Setup Guide

A complete, beginner-friendly walkthrough from a blank computer to a
ROS 2-connected Black Panther X. Every step is copy-paste ready. Commands run
in a **terminal** — open one with `Ctrl+Alt+T` on Ubuntu.

**How this guide is organized:**

| Part | What you'll have at the end |
|---|---|
| [Part 1](#part-1--what-you-need) | Hardware & accounts checklist |
| [Part 2](#part-2--jetson-setup-ubuntu--jetpack) | A flashed, updated Jetson |
| [Part 3](#part-3--install-ros-2-humble) | ROS 2 Humble installed |
| [Part 4](#part-4--get-the-code-and-install-the-bpx-sdk) | Project code + BPX SDK installed |
| [Part 5](#part-5--build-the-workspace) | Workspace built |
| [Part 6](#part-6--test-without-the-robot-mock-mode) | **A running stack, no robot needed** |
| [Part 7](#part-7--connect-to-the-bpx-robot-first-time) | Live telemetry from the real robot |
| [Part 8](#part-8--walking-commands-through-the-safety-chain) | Walking via ROS 2, through the safety gate |
| [Part 9](#part-9--seeing-the-robot-rviz2) | 3D visualization |
| [Part 10](#part-10--insight-9-camera) | Insight 9 camera streaming |
| [Part 11](#part-11--troubleshooting) | Fixes for common problems |

> 💡 New to Linux/ROS 2? Read the [Glossary](#glossary) at the bottom first.

---

## Part 1 — What you need

**Hardware**

- [ ] MirrorMe Black Panther X (charged, firmware up to date via the BPX app)
- [ ] NVIDIA Jetson Orin NX 16 GB (or Orin Nano 8 GB) on a carrier board, plus its power supply
- [ ] Looper Robotics Insight 9 camera (or any depth camera)
- [ ] A regular Ethernet cable (RJ45) — this is the most reliable way to talk to the robot
- [ ] A second computer (Windows/Mac/Linux) with the NVIDIA SDK Manager installed — used **once** to flash the Jetson
- [ ] USB keyboard/mouse + monitor for the Jetson's first boot (or SSH after)

**Software (all free)**

- NVIDIA SDK Manager (on the second computer) — https://developer.nvidia.com/sdk-manager
- A GitHub account (to get the code and ask questions)

**Know before you start**

| Thing | Value |
|---|---|
| Robot IP over Ethernet cable | `10.21.20.1` |
| Robot IP on its WiFi hotspot | `10.21.40.1` |
| This project's ROS 2 version | Humble (Ubuntu 22.04) |

---

## Part 2 — Jetson setup (Ubuntu + JetPack)

> **Which JetPack?** Use **JetPack 6.2.3** (L4T 36.5.2, Ubuntu 22.04).
> JetPack 7.2.x exists for the Orin NX, but it's Ubuntu 24.04 → ROS 2 Jazzy,
> and our camera (Insight 9) ships ROS 2 Humble — mixing Humble and Jazzy in
> one ROS network is unsupported and painful. JP7.2's Orin support is also
> young (known boot-issue overlay, prerelease PyTorch wheels, no r39 AI
> containers yet). Plan the JP7.2 + Jazzy migration when ROS 2 Humble nears
> end-of-life (May 2027) or when you start heavy LLM work — see
> `RESEARCH.md` §5.

The Jetson needs **JetPack 6.2.3 or newer 6.2.x** (Ubuntu 22.04 preinstalled).

> ⚠️ **Flashing host OS — learn from our pain:**
> - **Windows 11 does not work reliably** (SDK Manager isn't supported on
>   Windows, and USB recovery-mode passthrough through WSL2/VMs fails
>   unpredictably). Don't burn hours on it — go straight to a Linux host.
> - **Ubuntu 26.04 does not work** with SDK Manager / JetPack 6.2.x — the
>   SDK tooling lags the newest distro releases.
> - **Use Ubuntu 24.04 LTS** (confirmed working in practice) or 22.04 LTS
>   (the officially supported host). Both run fine from a USB stick /
>   spare partition — you only need this host when (re)flashing.
> - SDK Manager is not available for macOS.

> **Optional SDKs in SDK Manager (DeepStream, Holoscan, etc.): skip them.**
> DeepStream is multi-stream video analytics — we have one camera whose
> on-board NPU already runs detection. Holoscan is an ultra-low-latency
> sensor framework for special bridge hardware — not our path. Both can be
> added later without reflashing.
>
> **Jetson Platform Services: skip too** — it's a Docker microservices suite
> for commercial video/analytics deployments, unrelated to a ROS 2 robot.
>
> **Runtime vs SDK components: choose the full SDK/development components
> for the Jetson.** Runtime-only CUDA omits `nvcc` and headers, but we
> *compile* whisper.cpp and llama.cpp with CUDA on-board (voice phase), so
> the dev toolchain must be present.
>
> DO make sure these are selected: **CUDA (dev), cuDNN, TensorRT, OpenCV
> (CUDA), and the NVIDIA Container Toolkit**. Skip "Sources" packages
> (kernel/BSP dev only) and host-side components beyond the flasher.
> Use an NVMe SSD, not an SD card, as root storage
> (a 256 GB NVMe is confirmed for this build — ample: full stack + models +
> containers land well under 100 GB; just watch rosbag recordings).

1. On your second computer, install and open **NVIDIA SDK Manager**.
2. Sign in with an NVIDIA developer account, select your Jetson module
   (Orin NX 16 GB), and flash **JetPack 6.2.3**. Follow the wizard — it will
   ask you to put the Jetson into recovery mode (usually: hold the RECOVERY
   button while pressing RESET, or per your carrier board's manual).
   - 📖 NVIDIA's official walkthrough: https://docs.nvidia.com/jetpack/
3. When flashing finishes, boot the Jetson with a monitor attached and
   complete the Ubuntu setup (username, password, WiFi optional).
4. **Enable "Super Mode"** (free performance boost for Orin boards):

   ```bash
   sudo nvpmodel -m 0
   sudo jetson_clocks
   ```

5. Update everything:

   ```bash
   sudo apt update && sudo apt upgrade -y
   ```

> 💡 Set the Jetson to not sleep/lock during long demos:
> *Settings → Power → Screen Blank → Never.*

---

## Part 3 — Install ROS 2 Humble

Option A — **automated** (recommended): run our helper script from this repo:

```bash
cd ~/bpx-project          # after Part 4, or clone first (see note below)
./scripts/setup_ros2.sh
```

> 📌 If you don't have the repo yet, grab it first:
> `git clone https://github.com/df02/bpx_vision_voice.git ~/bpx-project`

Option B — **manual**: copy-paste these blocks one at a time.

```bash
# 1. Add the ROS 2 package repository
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
```

```bash
# 2. Install ROS 2 Humble + build tools (this downloads ~1 GB, be patient)
sudo apt install -y ros-humble-desktop ros-dev-tools ros-humble-teleop-twist-keyboard
```

```bash
# 3. Load ROS 2 in every new terminal automatically
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

```bash
# 4. Verify — this should print a list of topics like /rosout, /parameter_events
ros2 topic list
```

---

## Part 4 — Get the code and install the BPX SDK

```bash
# 1. Get this project
git clone https://github.com/df02/bpx_vision_voice.git ~/bpx-project
cd ~/bpx-project
```

```bash
# 2. Get the official BPX SDK (MirrorMe's own repo, Apache-2.0)
git clone https://github.com/mirrormerobotics/bpx_sdk_open.git
```

```bash
# 3. Install the SDK's Python bindings (compiles for ~1-2 minutes)
pip3 install --upgrade pip
pip3 install ./bpx_sdk_open

# 4. Verify
python3 -c "import bpx_sdk; print('BPX SDK OK')"
```

> ⚠️ If step 3 fails with a compiler error, install build tools:
> `sudo apt install -y build-essential python3-dev`
> then retry.
>
> 💡 **No SDK / no robot? No problem.** The whole stack runs in *mock mode*
> without `bpx_sdk` — skip this and go straight to Part 5.

---

## Part 5 — Build the workspace

```bash
cd ~/bpx-project/ros2_ws

# Make sure the ROS 2 environment is loaded in THIS terminal
# (rosdep needs it to know the distro; a fresh terminal after Part 3 has it via ~/.bashrc)
source /opt/ros/humble/setup.bash

# Install any missing ROS dependencies automatically
rosdep update
rosdep install --from-paths src --ignore-src -y

# Build (~1-2 minutes)
colcon build --symlink-install

# Load this workspace in every new terminal automatically
echo "source ~/bpx-project/ros2_ws/install/setup.bash" >> ~/.bashrc
source install/setup.bash
```

✅ **Check:** `ros2 pkg list | grep bpx` should print `bpx_bridge`, `bpx_description`, `bpx_safety`.

---

## Part 6 — Test without the robot (mock mode) 🎉

If `bpx_sdk` is installed but the robot is off, or the SDK isn't installed at
all, the bridge automatically runs in **mock mode**: it pretends to be a robot
and publishes placeholder data. Perfect for learning the tools.

```bash
# Terminal 1 — start the robot description + bridge
ros2 launch bpx_bridge bpx_bringup.launch.py
```

You'll see a warning like `running in MOCK mode`. Now open a **second terminal**:

```bash
# Terminal 2 — watch the fake joint data stream by
ros2 topic echo /bpx_bridge/joint_states
```

Press `Ctrl+C` to stop either program.

✅ **Check:** you see 12 joints (`fl_hip_roll_joint` … `hr_knee_joint`) with
numbers changing. Congratulations — the whole ROS 2 side works.

---

## Part 7 — Connect to the BPX robot (first time)

### 7.1 Wire it up

1. Power on the robot (but leave it lying down, motors relaxed).
2. Connect the Ethernet cable from the Jetson to the robot's RJ45 port.
3. The robot is `10.21.20.1`. Your Jetson needs an address in the same range:

   *GUI way:* Settings → Network → Wired → gear icon → IPv4 → **Manual** →
   Address `10.21.20.100`, Netmask `255.255.255.0` → Apply.

4. Test the connection:

   ```bash
   ping -c 3 10.21.20.1
   ```

   ✅ You want `3 packets transmitted, 3 received`. (❌ See
   [Troubleshooting](#part-11--troubleshooting).)

### 7.2 See live telemetry (robot still lying down — safe)

```bash
# Terminal 1 — bridge, now with the real robot
ros2 launch bpx_bridge bpx_bringup.launch.py

# Terminal 2 — watch the robot's real battery/temperature/connection status
ros2 topic echo /diagnostics
```

✅ **Check:** `connected` appears, with `battery_percent` and motor
temperatures. Tilt the robot gently by hand and watch joint values change:

```bash
ros2 topic echo /bpx_bridge/joint_states
```

### 7.3 Using the robot's WiFi instead of the cable

Join the robot's WiFi hotspot from the Jetson, then use its IP:

```bash
ros2 launch bpx_bridge bpx_bringup.launch.py robot_ip:=10.21.40.1
```

> ⚠️ The WiFi link can drop while the robot moves. The bridge is built for
> this (it disarms on every disconnect), but the **cable is strongly
> recommended while learning**.

---

## Part 8 — Walking commands (through the safety chain)

> ⚠️ **Read first:** place the robot on the floor with clear space around it,
> keep the BPX phone app / remote handy as your E-stop, and start slowly.

The command flow is:

```
your commands ──▶ cmd_vel_unsafe ──▶ bpx_safety (obstacle gate) ──▶ cmd_vel ──▶ bpx_bridge ──▶ robot
```

With three terminals:

```bash
# Terminal 1 — bridge + robot model
ros2 launch bpx_bridge bpx_bringup.launch.py
```

```bash
# Terminal 2 — safety gate (works with or without camera; no camera = always clear)
ros2 launch bpx_safety bpx_safety.launch.py
```

```bash
# Terminal 3 — ARM the robot's motion. Nothing moves until you do this.
ros2 service call /bpx_bridge/enable_motion std_srvs/srv/Trigger
# Expected: success=True, message='motion armed'
```

Still in Terminal 3, get it up on its feet:

```bash
ros2 service call /bpx_bridge/stand_up std_srvs/srv/Trigger
```

Now drive it with the keyboard:

```bash
# Terminal 4 — keyboard teleop (note: publishes to cmd_vel_UNSAFE, through the gate)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_unsafe
```

- `i` = forward, `,` = back, `j`/`l` = turn left/right, `k` = stop
- Keep speeds small at first: keys `z` and `x` reduce/increase speed

When done:

```bash
ros2 service call /bpx_bridge/sit_down std_srvs/srv/Trigger
```

**Handy commands:**

| Command | Effect |
|---|---|
| `ros2 service call /bpx_bridge/stand_up std_srvs/srv/Trigger` | stand up |
| `ros2 service call /bpx_bridge/sit_down std_srvs/srv/Trigger` | lie down |
| `ros2 service call /bpx_bridge/damping std_srvs/srv/Trigger` | relax all joints (safe state) |
| `ros2 service call /bpx_bridge/walk std_srvs/srv/Trigger` | walking gait |
| `ros2 service call /bpx_bridge/run std_srvs/srv/Trigger` | running gait (fast!) |
| `ros2 service call /bpx_bridge/jump_up std_srvs/srv/Trigger` | jump (robot must be standing, not moving) |
| `ros2 topic echo /bpx_safety/active` | `true` = obstacle gate is currently blocking |

> 🔒 **Why `enable_motion`?** The BPX SDK wipes all motion commands every
> time the connection drops and reconnects (so a stale "walk forward" can
> never resume by itself). The bridge mirrors this: after any disconnect you
> must arm again. It also sends a zero-velocity command if your command
> stream stops — release the keys and the robot stops within 0.5 s.

---

## Part 9 — Seeing the robot (RViz2)

```bash
# New terminal while the bridge is running:
rviz2
```

In RViz2: **Add** (bottom left) → **By topic** →
- `RobotModel` (from `/robot_description`) — the 3D robot
- `Odometry` (from `/bpx_bridge/odom`) — where the robot thinks it is
- `Imu` (from `/bpx_bridge/imu`) — orientation

Set **Fixed Frame** (top left) to `odom`. Move the robot by hand (lying down,
damping mode) and watch the model follow.

### Better than RViz for day-to-day: Foxglove Studio

Foxglove (https://foxglove.dev — free desktop app or browser) is a modern
ROS 2 visualization cockpit. Recommended for all bring-up sessions:

```bash
# On the Jetson:
sudo apt install ros-humble-foxglove-bridge
ros2 run foxglove_bridge foxglove_bridge
# Then connect from https://app.foxglove.dev or the desktop app:
#   Open connection -> Foxglove WebSocket -> ws://<jetson-ip>:8765
```

Panels that map onto this project:
- **Image**: the compressed color topic *and* depth render directly
- **Teleop**: set its output topic to `cmd_vel_unsafe` → keyboard/on-screen
  driving *through the safety gate* (primary desk-testing teleop)
- **Plot**: `/diagnostics` fields (battery, motor temps), status topics
- **3D**: renders the URDF + TF natively
- **Record**: captures sessions to MCAP (readable by rosbag2 too)

---

## Part 10 — Insight 9 camera

The Insight 9 is **ROS-native: there is no SDK or driver to install.** The
camera runs its own ROS 2 node onboard and publishes topics directly when
plugged in. (Guide: Looper's "Insight9 ROS Native Bring-Up Guide", firmware
v2.1.4.)

1. Connect the camera to the Jetson with a **good USB-C data cable** —
   rated 5 Gbps minimum, 10 Gbps recommended. Cheap cables power the camera
   but starve its data link. The camera is powered over this same cable.
2. Wait ~20 seconds for boot.
3. In a terminal with ROS 2 sourced:

   ```bash
   source /opt/ros/humble/setup.bash
   ros2 topic list | grep camera
   ```

   ✅ You should see topics under `/camera/camera/`, including
   `/camera/camera/imu`, `/camera/camera/vio_100hz`, and
   `/camera/camera/vio_status`.

4. Sanity checks:

   ```bash
   ros2 topic hz /camera/camera/imu        # should read ~400 Hz
   ros2 topic echo /camera/camera/vio_status --once
   ```

5. **Enable depth** — it's disabled by default (USB bandwidth). Open the
   camera's web dashboard at **http://looper.local** and toggle depth
   output on. Then:

   ```bash
   ros2 topic hz /camera/camera/depth/image_rect_raw   # ~15 Hz
   ```

6. The safety node is preconfigured for these topic names — just launch it:

   ```bash
   ros2 launch bpx_safety bpx_safety.launch.py
   ```

7. **Calibrate the stop zone** (one-time): the defaults
   (`stop_distance: 1.2 m`, height band −0.3…+1.5 m relative to camera) are
   conservative starting points. Bring an obstacle toward the robot's front
   and watch `ros2 topic echo /bpx_safety/active` flip to `true` at a
   distance you're comfortable with. Adjust in
   `ros2_ws/src/bpx_safety/config/bpx_safety.yaml`.

### Viewing the live image

Easiest — `rqt_image_view`. Install it **plus** the compressed-transport
plugin (without it you'll get "Unable to load plugin for transport
'image_transport/compressed_sub'" and no color picture):

```bash
sudo apt install ros-humble-rqt-image-view ros-humble-image-transport-plugins
rqt_image_view
# pick the color topic ending in /compressed from the dropdown
```

In RViz2: set **Fixed Frame** to `insight9_camera_link` (needs the robot
description running, e.g. `ros2 launch bpx_bridge bpx_bringup.launch.py`),
then **Add → By topic →** the color topic ending in **`/compressed`** **→
Image**. Use the **Image** display, not the Camera display — the Camera
display's topic dropdown only lists raw `sensor_msgs/Image` topics, which
excludes the color stream (compressed) entirely. Save the config when it
works.

⚠️ Don't use `/camera/camera/color/image_raw` — it's NV12 format, which
RViz2 cannot render. The `/compressed` MJPEG topic is the one to view.
(The dashboard at `looper.local` also shows stream previews.)

Useful facts (firmware v2.1.4): IMU publishes at 400 Hz; stereo runs
20–50 fps; VIO pose is available at 100 Hz (`/camera/camera/vio_100hz`);
the RGB stream is portrait-oriented (1088×1920 default MJPEG); on-device
YOLO models (person detection) can be flashed from the web dashboard.

### Recommended dashboard settings (looper.local)

| Setting | Value | Why |
|---|---|---|
| Depth output | **ON** | Required by `bpx_safety` (obstacle gate) |
| Pose with covariance | **ON** | `/camera/camera/vio_image_cov` — the EKF in `bpx_nav` needs VIO *with* covariance; cheap bandwidth; keeps recorded bags complete |
| Detection model | **Flash the human/person detection model** (Model setting / Model writing menus) | Required by `bpx_follow`. Model "writing" = flashing via Looper's website (convert online → redirect to device flash page) |

After flashing a detection model, confirm the detection topic name and
message type — `bpx_follow` assumes `vision_msgs/Detection2DArray` and
its config is adjustable if the camera publishes differently:

```bash
ros2 topic list | grep -i -E "detect|yolo|person"
ros2 topic info <topic>
```

> If discovery fails: restart the ROS 2 daemon (`ros2 daemon stop &&
> ros2 daemon start`), verify the cable, and check the device address
> (`169.254.10.1` by default). Don't hand-edit the host's virtual network
> interface.

---

## Part 11 — Troubleshooting

| Problem | Likely cause & fix |
|---|---|
| `ping 10.21.20.1` gets no reply | Cable not clicked in; robot not fully booted; your Jetson's wired IP isn't set to `10.21.20.100`/netmask `255.255.255.0` (Part 7.1 step 3) |
| Bridge says "Initial connect … failed" but ping works | Robot is on but its SDK server isn't up yet — wait ~30 s, or call the `~/reconnect` service; check firmware is current in the BPX app |
| `colcon: command not found` | ROS 2 not in this terminal: `source /opt/ros/humble/setup.bash` (it's auto-sourced in new terminals if Part 3 step 3 was done) |
| `ROS distro is not set` / `Cannot locate rosdep definition` | Same cause: source ROS first — `source /opt/ros/humble/setup.bash` — then re-run the `rosdep install` command |
| `import bpx_sdk` fails | Rebuild it: `pip3 install --force-reinstall ./bpx_sdk_open`; if it complains about a compiler: `sudo apt install -y build-essential python3-dev` |
| Topics appear on one terminal but not another | You forgot `source ~/bpx-project/ros2_ws/install/setup.bash` in that terminal |
| Robot ignores `cmd_vel` | Bridge not armed: `ros2 service call /bpx_bridge/enable_motion std_srvs/srv/Trigger`; or the safety gate is blocking: check `ros2 topic echo /bpx_safety/active`; or the gait isn't `walk` |
| Robot stopped by itself | Safety gate triggered by an obstacle (expected!) — clear the area, it resumes after `clear_time`; or cmd_vel stream went stale for >0.5 s |
| RViz shows no robot model | Fixed Frame isn't set to `odom`, or the bridge/launch isn't running |
| WiFi drops while walking | Expected with the hotspot; robot re-arms required. Use the Ethernet cable. |
| `ROS_TIME` / time warnings at startup | Harmless during the first seconds while clocks sync |

Still stuck? [Open an issue](https://github.com/df02/bpx_vision_voice/issues)
with: the exact command you ran, the full terminal output, and your hardware
(BPX model, Jetson model, camera).

---

## Glossary

- **ROS 2 (Robot Operating System)** — the robotics middleware everything here
  is built on. Programs ("nodes") exchange data as "topics" (streams) and
  "services" (one-shot requests).
- **Topic** — a named data stream. `ros2 topic echo /some/topic` prints what's
  flowing through it.
- **Service** — a request/response call, like pressing a button:
  `ros2 service call /some/service std_srvs/srv/Trigger`.
- **Node** — one program in the ROS 2 system. `bpx_bridge` and `bpx_safety`
  are nodes.
- **URDF** — the robot's 3D description file (links, joints) used by RViz.
- **JetPack** — NVIDIA's OS + drivers for Jetson. Version 6.2+ = Ubuntu 22.04
  + "Super Mode" performance boost.
- **colcon** — the build tool for ROS 2 workspaces.
- **rosdep** — auto-installs the ROS dependencies your packages declare.
- **Mock mode** — the bridge pretending to be a robot so you can develop
  without hardware.
- **cmd_vel** — the standard ROS topic for velocity commands
  (forward/lateral speed + turn rate).
- **E-stop** — emergency stop. On the BPX: the phone app / remote controller.

---

## What's next?

- 📷 `bpx_perception` — object detection & mapping from the Insight 9
- 🎙️ `bpx_voice` — talk to your robot: wake word → Whisper → local LLM →
  Piper voice, commanding the robot through the same safety chain
- 🗺️ Autonomy with Nav2 using the bridge's odometry

Contributions welcome — see the README for the project layout.

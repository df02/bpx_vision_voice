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

The Jetson needs NVIDIA's **JetPack 6.2 or newer** (Ubuntu 22.04 preinstalled).

1. On your second computer, install and open **NVIDIA SDK Manager**.
2. Sign in with an NVIDIA developer account, select your Jetson module
   (Orin NX 16 GB), and flash **JetPack 6.2.x**. Follow the wizard — it will
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

---

## Part 10 — Insight 9 camera

The Insight 9 is special: it does its own depth + SLAM processing and just
streams ready-made ROS 2 topics over USB.

1. Connect the camera to the Jetson with a **USB-C data cable** (the camera
   is powered over this same cable).
2. Install the Looper Robotics SDK following their developer guide:
   https://looper-robotics.com/ — it makes the camera appear as a network
   device and starts its ROS 2 node.
3. Find the actual topic names it publishes:

   ```bash
   ros2 topic list | grep -i insight   # adjust if named differently
   ```

4. Point the safety node at the real topics:

   ```bash
   ros2 launch bpx_safety bpx_safety.launch.py depth_topic:=<the depth topic you found>
   ```

5. **Calibrate the stop zone** (one-time, walking area setup): the defaults
   (`stop_distance: 1.2 m`, height band −0.3…+1.5 m relative to camera) are
   conservative starting points. Walk an obstacle toward the robot's front
   and watch `ros2 topic echo /bpx_safety/active` flip to `true` at a
   distance you're comfortable with. Adjust in
   `ros2_ws/src/bpx_safety/config/bpx_safety.yaml`.

> ⚠️ If the camera topic names differ from this guide, open an issue —
> we'll update the docs.

---

## Part 11 — Troubleshooting

| Problem | Likely cause & fix |
|---|---|
| `ping 10.21.20.1` gets no reply | Cable not clicked in; robot not fully booted; your Jetson's wired IP isn't set to `10.21.20.100`/netmask `255.255.255.0` (Part 7.1 step 3) |
| Bridge says "Initial connect … failed" but ping works | Robot is on but its SDK server isn't up yet — wait ~30 s, or call the `~/reconnect` service; check firmware is current in the BPX app |
| `colcon: command not found` | ROS 2 not in this terminal: `source /opt/ros/humble/setup.bash` (it's auto-sourced in new terminals if Part 3 step 3 was done) |
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

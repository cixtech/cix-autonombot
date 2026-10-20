# AutonomBot - Autonomous Navigation Robot

[中文](README.zh-CN.md)

AutonomBot is an indoor AMR stack built on ROS 2 and Nav2 (originally a university project; thanks to the upstream team for the hardware design). This fork targets the **CIX P1 edge platform**: Gazebo simulation, localization, path planning, and **NPU-accelerated vision** (SDK package `cix_ros_noe_yolo`). See [NOTICE](NOTICE) for the change list, and the upstream repo at [houimliraed/autonomous_ai_rbt](https://github.com/houimliraed/autonomous_ai_rbt).

The robot uses LiDAR, camera, and related sensors for costmaps and dynamic avoidance. Vision runs YOLO on the board NPU by default, offloading detection from the main CPU so simulation, Nav2, and vision can run in parallel more comfortably.

## Video Demo

![AutonomBot Demo](media/autonombot_demo_fast.gif)

> **Demo contents:**
> - Autonomous navigation in simulation
> - Real-time obstacle detection and avoidance
> - Dynamic path planning and re-routing
> - Multi-sensor fusion (LiDAR + camera + ultrasonic)

*Full video: [Download WebM](media/autonombot_demo.webm)*

## Features

- **Autonomous navigation**: Path planning and obstacle avoidance with Nav2
- **Multi-sensor fusion**: LiDAR, camera, ultrasonic
- **Computer vision**: YOLOv8 object detection (CIX NPU)
- **SLAM mapping**: Real-time mapping and localization
- **Hardware support**: CIX P1 (CD8180, aarch64)

## Platform notes

| Item | Detail |
|------|--------|
| OS | Ubuntu 22.04 + ROS 2 Humble (aarch64) only — **Ubuntu 24.04 / ROS 2 Jazzy is not supported** |
| Hardware | CIX P1 board + on-board NPU runtime (`libnoe`, etc., typically under `/usr/share/cix/lib`) |
| Workspace | **This repository is itself a colcon workspace** and may live at any path (e.g. `~/cix-autonombot`, or under an SDK `demos/...` tree). `run_sim.sh` uses the repo root and does not depend on a fixed parent directory name. |
| NPU package | `cix_ros_noe_yolo` is **not vendored** in this repo; provide it via an SDK deb, or by building the SDK `ros_packages/` elsewhere and setting `ROBOTICS_SDK_ROOT`. |

Simulation worlds such as `small_house` / `small_warehouse` come from AWS RoboMaker (Apache 2.0); see [NOTICE](NOTICE).

Below, `$DEMO_WS` means **this repository’s root** (the directory that contains `src/` and `scripts/`). Replace it with your actual clone path.

## Quick Start

### 1. Get this repository

```bash
git clone <this repository URL> "$DEMO_WS"
cd "$DEMO_WS"
```

(If you use a full [CIX Robotics SDK](https://github.com/cix-robotics/robotics_sdk) checkout, `$DEMO_WS` may instead point at a demo subdirectory inside that tree — the same workflow applies.)

### 2. Install the ROS 2 environment (Humble / aarch64)

- If you already have a full SDK tree, you can use its `scripts/setup_ros2_humble_arm64.sh`.
- Or install ROS 2 Humble + Nav2 / Gazebo and related dependencies yourself.

### 3. Build this workspace

```bash
cd "$DEMO_WS"
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### 4. Install `cix_ros_noe_yolo` (choose one)

NPU YOLO is enabled by default; the package must be on the ROS search path.

**Option A: Build from SDK source** (the SDK may live at any path; it need not be next to `$DEMO_WS`)

```bash
cd <your_robotics_sdk>/ros_packages
source /opt/ros/humble/setup.bash
colcon build --base-paths cix_ros_noe_yolo --packages-select cix_ros_noe_yolo --symlink-install
```

When launching, set (the name must be **`ROBOTICS_SDK_ROOT`**, not `ROBOTICS_SDK`):

```bash
export ROBOTICS_SDK_ROOT=<your_robotics_sdk>
```

**Option B: Install from deb**

> Not available yet. Once the deb is installed into `/opt/ros/humble`, you typically **do not** need `ROBOTICS_SDK_ROOT`.

### 5. Download NPU model weights (`.cix`)

The `.cix` models are not shipped with this repository or the deb (AGPL-3.0). Download them once after the package is installed:

```bash
source /opt/ros/humble/setup.bash
# For a source build, source its install first; for a deb install this is usually unnecessary
source "$ROBOTICS_SDK_ROOT/ros_packages/install/setup.bash"   # option A only

sudo bash "$(ros2 pkg prefix cix_ros_noe_yolo)/share/cix_ros_noe_yolo/scripts/download_models.sh"
```

Model directory:

`$(ros2 pkg prefix cix_ros_noe_yolo)/share/cix_ros_noe_yolo/model/`

### 6. Verify the environment

```bash
cd "$DEMO_WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
# For option A also:
# export ROBOTICS_SDK_ROOT=<your_robotics_sdk>
# source "$ROBOTICS_SDK_ROOT/ros_packages/install/setup.bash"

ros2 pkg prefix autonombot_launch
ros2 pkg prefix cix_ros_noe_yolo
```

Both commands should print an install prefix, not `Package not found`.

### 7. Launch the simulation

```bash
cd "$DEMO_WS"
# Required for option A (source SDK):
export ROBOTICS_SDK_ROOT=<your_robotics_sdk>

./scripts/run_sim.sh launch
```

`run_sim.sh` cleans up stale processes, starts Gazebo + Nav2 + vision, and opens `rqt_image_view` (default topic `/vision/yolo_debug_image`). To view another topic: `./scripts/view_yolo_debug.sh /camera/image_raw`.

If you use the source-built `cix_ros_noe_yolo` long term, you can add `export ROBOTICS_SDK_ROOT=...` to `~/.bashrc`.

### Remote terminal (SSH / serial console)

Gazebo opens a window, so the launch needs a working X display on the board. The
on-board desktop terminal already has one; a remote shell does not. Remote X11
forwarding (MobaXterm serial console with `DISPLAY=localhost:10.0`, `ssh -X`)
cannot open the Gazebo window here, and the launch exits after a few seconds.

Export these before `run_sim.sh`:

```bash
# SSH / remote terminal only; the on-board desktop Terminal normally does not need this
export DISPLAY=:0
export XDG_RUNTIME_DIR=/run/user/$(id -u)
export XAUTHORITY=$(ls -t /run/user/$(id -u)/.mutter-Xwaylandauth.* | head -1)

./scripts/run_sim.sh launch
```

### Stop

Press `Ctrl+C` in the launch terminal. Any leftover ROS / Gazebo / RViz
processes from a crashed session are cleaned up automatically the next time
`run_sim.sh` starts.

## NPU vision overview

`run_sim.sh` starts NPU YOLO together with the simulation stack — you do not need a second terminal to launch the inference node.

| Component | Role |
|-----------|------|
| `cix_ros_noe_yolo` (`npu_yolo_coco_inference`) | Subscribes to `/camera/image_raw`, runs YOLOv8n `.cix` on the NPU, publishes detections |
| `advanced_vision_navigator` | When NPU is active, consumes external detections and publishes obstacle cloud / markers / `/vision_debug` for Nav2 |

Main topics:

| Topic | Type / meaning |
|-------|----------------|
| `/camera/image_raw` | Raw simulated camera |
| `/vision/detections_2d` | NPU detections (`vision_msgs/Detection2DArray`) |
| `/npu_vision_active` | Whether NPU claims vision (`std_msgs/Bool`) |
| `/vision/yolo_debug_image` | NPU YOLO debug image (default `rqt_image_view` topic) |
| `/vision_debug` | Annotated debug image |
| `/vision_obstacles` | Vision obstacle cloud (feeds Nav2 local costmap) |

Deeper integration notes: [docs/npu_vision_npu_integration.md](docs/npu_vision_npu_integration.md), [docs/yolo_advanced_vision.md](docs/yolo_advanced_vision.md).

## Network / multi-terminal environment

The simulation launch isolates DDS: `ROS_DOMAIN_ID=61`, localhost-only, and a Fast DDS shared-memory profile (written to `/tmp/fastdds_shm_only.xml` at startup). **Any new terminal** that needs `ros2 topic` / `ros2 node` against the sim must match that environment:

```bash
cd "$DEMO_WS"
source scripts/env.sh
# For option A also export ROBOTICS_SDK_ROOT=... and source its ros_packages/install/setup.bash
```

> `ros2 topic hz` / `ros2 topic list` use the background `ros2` daemon. If a daemon was started with a different environment, run `ros2 daemon stop` once in the matched terminal; it restarts automatically.

## Optional checks after launch

```bash
ros2 node list | grep yolo
ros2 topic list | grep -E 'camera|vision'
ros2 topic hz /camera/image_raw
ros2 topic hz /vision/detections_2d
ros2 topic hz /vision_debug
ros2 topic echo /npu_vision_active --once
```

## Usage

- **Navigation**: Send a goal in RViz with “2D Goal Pose”.
- **Joystick**: Started automatically with the simulation (merged with Nav2 via `twist_mux`).
- **Images**: `run_sim.sh` opens `rqt_image_view` on `/vision/yolo_debug_image` after ~20s; or run `./scripts/view_yolo_debug.sh [topic]`.

## Workspace layout

- `src/` - ROS 2 package sources (long-term maintenance)
- `build/` - colcon intermediates (safe to delete and rebuild)
- `install/` - installed outputs; `source install/setup.bash` before running
- `log/` - build logs
- `docs/` - extra documentation
- `media/` - demo GIF / WebM
- `scripts/` - `run_sim.sh`, `env.sh`, etc.

Typical flow: edit under `src` → `colcon build` → `source install/setup.bash` → `./scripts/run_sim.sh launch`.

Package layout details (Chinese): [docs/仓库目录与功能包说明.md](docs/仓库目录与功能包说明.md)

## Package structure

Eight packages under `src/`:

| Package | Role |
|---------|------|
| `autonombot_launch` | Top-level launch and configs wiring simulation / drivers / navigation / vision |
| `autonombot_description` | URDF/xacro, meshes, Gazebo worlds and models |
| `autonombot_firmware` | ros2_control hardware interface, serial / sensor nodes |
| `autonombot_drivers` | Python drivers and controllers (twist relay, teleop integration, etc.) |
| `autonombot_localization` | AMCL, robot_localization, map server, ultrasonic fusion |
| `autonombot_mapping` | SLAM (`slam_toolbox`), map assets under `maps/` |
| `autonombot_navigation` | Nav2 planners, controllers, behavior trees, parameters |
| `autonombot_vision` | Vision navigation (`advanced_vision_navigator`), obstacles and costmap |

NPU YOLO (`cix_ros_noe_yolo`) is **not** in this repository (see Quick Start). There is no separate `autonombot_planning` package here.

## Dependencies

- Ubuntu 22.04 + ROS 2 Humble (aarch64)
- Nav2 Navigation Stack
- Gazebo Simulation
- OpenCV 4.x, PCL
- CIX Robotics SDK (`cix_ros_noe_yolo`) and the board NPU runtime

## License

Apache 2.0 License. See [LICENSE](LICENSE).

This repository is a fork of [houimliraed/autonomous_ai_rbt](https://github.com/houimliraed/autonomous_ai_rbt); see [NOTICE](NOTICE) for the upstream base commit and a summary of CIX modifications.

Simulation worlds and some models are derived from AWS RoboMaker Small House / Small Warehouse (Apache 2.0); see [NOTICE](NOTICE).

The NPU `.cix` models are derived from Ultralytics weights and are licensed AGPL-3.0; they are not distributed with this repository. Downloading them accepts that license. If you have a full SDK tree, see also its root `NOTICE.md`.

Optional CPU Ultralytics YOLO (`ultralytics` / `torch` / `torchvision`) is **not** part of the default install. If you need that path, install it yourself from `src/autonombot_vision/requirements-cpu-optional.txt` and accept Ultralytics AGPL-3.0; the supported default remains the NPU path (`cix_ros_noe_yolo`).

We do not accept third-party contributions at this time.

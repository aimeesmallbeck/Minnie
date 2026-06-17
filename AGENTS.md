# Aimee Robot ROS2 Workspace — Agent Guide

This file contains project-specific information for AI coding agents working on the Aimee robot software stack. The reader of this file is assumed to know nothing about the project.

---

## Project Overview

This is a **ROS2 Humble Hawksbill** workspace for the **Aimee social assistance robot**, designed to run on the **Arduino UNO Q** (an ARM64/aarch64 single-board computer with 4GB RAM). It is a complete robotics software stack encompassing voice interaction, computer vision, manipulator control, mobile base navigation, cloud connectivity, and local LLM inference.

The project is licensed under the **Mozilla Public License 2.0 (MPL-2.0)**. Many source files contain SPDX headers:
```
# SPDX-FileCopyrightText: Copyright (C) ARDUINO SRL (http://www.arduino.cc)
# SPDX-License-Identifier: MPL-2.0
```

---

## Technology Stack

- **Middleware:** ROS2 Humble Hawksbill
- **DDS:** Cyclone DDS (`rmw_cyclonedds_cpp`) — chosen for Nav2 stability. `setup_env.sh` sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.
- **Primary Language:** Python 3 with `rclpy`
- **Secondary Languages:** C++14/C99 (ldlidar driver, OBSBOT SDK sample, ROS2 message generation)
- **Build Tool:** `colcon`
- **Containerization:** Docker + Docker Compose
- **Process Management:** systemd

### Key Python Dependencies

| Package | Version | Purpose |
|---------|---------|---------|
| `aiohttp` | 3.13.5 | Async HTTP |
| `flask` | latest | Web dashboard (monitor) |
| `gtts` | latest | Google TTS fallback |
| `pygame` | 2.6.1 | Audio playback |
| `vosk` | 0.3.45 | Offline STT |
| `pykokoro` / `kokorog2p` | 0.6.x | Local TTS |
| `paho-mqtt` | 2.1.0 | Cloud bridge |
| `requests` | 2.33.1 | HTTP |
| `numpy` | `<2` | Numerical ops |
| `opencv-python-headless` | 4.13.0.92 | Vision |
| `python-osc` | latest | OBSBOT camera control |
| `pillow` | latest | Image processing |
| `pyserial` | latest | Serial comms |
| `pyyaml` | latest | Config parsing |

### ML / Hardware Runtime

- **llama.cpp** (`llama-server`, `llama-cli`) — local LLM inference on GGUF models
- **Edge Impulse** — wake word detection (`.eim` model)
- **Vosk** — offline speech recognition
- **OBSBOT Device SDK** (`libdev.so`) — PTZ camera control

---

## Workspace Structure

```
aimee-robot-ws/
├── src/                    # 21 ROS2 packages (Python + C++)
├── build/                  # colcon build artifacts (gitignored)
├── install/                # colcon install artifacts (gitignored)
├── log/                    # colcon build logs (gitignored)
├── config/                 # Robot YAML configs, intent JSON, skills YAML
├── deploy/                 # Bootstrap scripts, model sync, deployment docs
├── docs/                   # Architecture and specification docs
├── scripts/                # Brick generator, routing fix, udev setup
├── systemd/                # Systemd services for auto-start
├── models/                 # LLM weights (.gguf), ONNX models (gitignored)
├── lib/                    # Prebuilt binaries (llama.cpp, etc.) (gitignored)
├── vosk-models/            # Vosk speech models (gitignored)
├── libdev_v2.1.0_8/        # OBSBOT Device SDK (vendored prebuilt C++ lib)
├── obsbot_helper/          # OBSBOT helper utilities
├── .arduino-bricks/        # Edge Impulse wake-word models
├── Dockerfile              # ROS2 runtime container
├── docker-compose.yml      # Docker service definition
├── setup_env.sh            # Environment sourcing script
├── .env / .env.example     # Environment variables and secrets
└── fastdds_*.xml           # DDS configuration profiles (legacy Fast DDS)
```

---

## Package Organization

There are **21 packages** in `src/`:

### Custom Messages, Bringup, and Description

| Package | Build Type | Description |
|---------|------------|-------------|
| `aimee_msgs` | `ament_cmake` | Custom ROS2 messages, services, actions (14 msg, 1 srv, 3 action) |
| `aimee_bringup` | `ament_cmake` | Launch files: `robot.launch.py`, `core.launch.py`, `slam.launch.py`, `nav2.launch.py`, `vision_pipeline.launch.py` |
| `aimee_description` | `ament_cmake` | URDF robot descriptions |

### Voice & Audio (Python)

| Package | Description |
|---------|-------------|
| `aimee_wake_word_ei` | Edge Impulse keyword spotting → publishes `/wake_word/detected` |
| `aimee_voice_manager` | STT pipeline (Vosk local + Whisper/Lemonfox cloud fallback) → publishes `/voice/transcription` |
| `aimee_tts` | TTS with Lemonfox (primary), Kokoro, gTTS fallback → subscribes `/tts/speak` |

### Intelligence & Cloud (Python)

| Package | Description |
|---------|-------------|
| `aimee_intent_router` | Classifies voice intents, routes to local skills or AimeeCloud |
| `aimee_skill_manager` | Skill execution engine — action server `/skill/execute` |
| `aimee_llm_server` | Non-blocking LLM action server with streaming + preemption — `/llm/generate` |
| `aimee_cloud_bridge` | MQTT bridge to AimeeCloud — AimeeAgent protocol, voice metadata, local command execution |

### Vision (Python)

| Package | Description |
|---------|-------------|
| `aimee_vision_obsbot` | OBSBOT Tiny 2 PTZ/tracking via OSC SDK |
| `aimee_vision_pipeline` | Color detection, multi-object tracking |
| `aimee_perception` | 3D pose estimation, grasp strategy planning |

### Hardware Control (Python)

| Package | Description |
|---------|-------------|
| `aimee_ugv02_controller` | Waveshare UGV02 / Wave Rover base control (JSON serial + HTTP) |
| `aimee_lerobot_bridge` | RoArm-M3 arm control via HTTP/JSON; LeRobot integration |
| `aimee_manipulation` | Arm controller, PickPlace action server |

### Utilities (Python)

| Package | Description |
|---------|-------------|
| `aimee_ros2_monitor` | Web dashboard (port 8081) — node status, logs, camera stream, topic Hz |
| `aimee_test_dashboard` | Hardware test dashboard with simulation mode |
| `arduino` | Arduino brick framework utilities (`app_utils/brick.py`) |
| `aimee_brick_template` | Template/boilerplate for creating new Arduino bricks |

### External / C++

| Package | Description |
|---------|-------------|
| `ldlidar_stl_ros2` | C++ LDLiDAR driver (LD06, LD19, STL27L) with launch files |

**Node File Pattern:** Each Python package typically contains:
- `<package_name>/<node_name>_node.py` — main ROS2 node(s)
- `<package_name>/brick/` — Arduino brick adapter submodules (for cloud_bridge, intent_router, skill_manager, llm_server, voice_manager)
- `config/brick_config.yaml` or `config/cloud_bridge.yaml` — package-specific configs

---

## Build and Runtime Commands

### Environment Setup

Always source the environment before building or running:

```bash
source ~/aimee-robot-ws/setup_env.sh
```

This script:
- Sources ROS2 Humble
- Sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- Sources the workspace `install/setup.bash` if present
- Sets model paths, cloud endpoints, and `PYTHONPATH`

### Building

```bash
# Full workspace build
colcon build --symlink-install

# Build specific packages
colcon build --packages-select aimee_ugv02_controller aimee_bringup --symlink-install

# Reduce parallelism for 4GB RAM constraint
colcon build --symlink-install --parallel-workers 1
```

### Docker Build

```bash
docker compose build
docker compose run --rm aimee-robot bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"
```

### Launching

```bash
# Full robot bringup (config-driven)
ros2 launch aimee_bringup robot.launch.py robot:=ron

# Core stack only (always launched software nodes)
ros2 launch aimee_bringup core.launch.py

# Debug mode: disable resource-heavy subsystems
ros2 launch aimee_bringup robot.launch.py use_voice:=false use_llm:=false
```

Launch file architecture:
- `robot.launch.py` — Top-level bringup. Reads `config/robot_config.yaml`, includes `core.launch.py`, conditionally adds base/arm/camera/lidar nodes.
- `core.launch.py` — Software stack always launched: voice_manager, tts, monitor, llm_backend (llama-server), llm_server, intent_router, skill_manager, cloud_bridge.

### Running Individual Nodes

```bash
# Example: TTS node only
ros2 run aimee_tts tts_node --ros-args -p default_engine:="lemonfox" -p lemonfox_api_key:="YOUR_KEY"

# Example: OBSBOT camera node only
ros2 run aimee_vision_obsbot obsbot_node --ros-args -p host:="192.168.5.1"
```

---

## Configuration

### Robot Hardware Config

`config/robot_config.yaml` defines hardware profiles for the robot fleet:
- **`ron`** — UGV02 base + RoArm-M3 arm + OBSBOT
- **`wren`** — Wave Rover base + OBSBOT
- **`minnie`** — Wave Rover base + OBSBOT + LD19 lidar

### Environment Variables

Copy and edit `.env` from `.env.example`:

```bash
cp ~/aimee-robot-ws/.env.example ~/aimee-robot-ws/.env
```

Key variables:
- `AIMEE_ROBOT_NAME` — Robot identity (`ron`, `wren`, `minnie`)
- `AIMEE_DEVICE_ID` — Unique device identifier
- `ROS_DOMAIN_ID` — Isolate robots on shared Wi-Fi (e.g., Ron=42, Minnie=43, Wren=44)
- `LEMONFOX_API_KEY` — TTS and Whisper STT fallback
- `AIMEE_CLOUD_ENDPOINT` / `AIMEE_CLOUD_BROKER_HOST` — Cloud connectivity
- `OBSBOT_IP` — OBSBOT Tiny 2 USB RNDIS IP (default `192.168.5.1`)
- `ROARM_SERIAL_PORT` / `UGV02_SERIAL_PORT` — Serial ports for arm and base
- `ALSA_CARD_INDEX` — ALSA audio device card index

**`.env` is gitignored. Never commit it.**

---

## Code Style Guidelines

### Python

- Standard ROS2 `rclpy` patterns with `snake_case` naming
- SPDX license headers are required on new Python files:
  ```python
  # SPDX-FileCopyrightText: Copyright (C) ARDUINO SRL (http://www.arduino.cc)
  # SPDX-License-Identifier: MPL-2.0
  ```
- No root-level formatter config (no `.black`, `.isort`, `.flake8` at workspace root)
- Several `setup.py` files declare `extras_require={'test': ['pytest']}`

### C++

- CMake packages use `-Wall -Wextra -Wpedantic` compiler flags
- `ldlidar_stl_ros2` includes `ament_lint_auto` for standard ROS2 C++ linting

---

## Testing Instructions

**There is no formal test suite.** There are no `test/` or `tests/` directories in source packages.

Existing ad-hoc test / utility scripts:

| File | Purpose |
|------|---------|
| `src/aimee_vision_obsbot/test_obsbot.py` | Standalone OBSBOT camera test |
| `src/aimee_lerobot_bridge/test_arm_basic.py` | Basic arm movement test |
| `src/aimee_lerobot_bridge/scripts/test_arm_basic.py` | Script version of arm test |
| `src/aimee_manipulation/aimee_manipulation/test_pick_place_client.py` | PickPlace action client test |
| `square_test.py` (root) | Open-loop SLAM square test (forward + turn × 4) |
| `test_cmdvel.py` (root) | Root-level cmd_vel publisher test |
| `test_pick_place.sh` (root) | Shell script for pick/place testing |
| `test_session.sh` (root) | Session test script |

Testing is primarily manual and integration-based on real hardware.

### Manual Topic Tests

```bash
# Test TTS
ros2 topic pub /voice/speak std_msgs/String "data: 'Hello!'"

# Test motion
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}}"

# View topics
ros2 topic list
ros2 topic echo /voice/transcription
```

---

## Deployment

### Docker Containerization

The production runtime uses Docker Compose:
- `privileged: true` for hardware access
- `network_mode: host` for ROS2 multicast/DDS
- Memory limit: 3GB (for 4GB UNO Q)
- Healthcheck via `ros2 node list`
- Binds `/dev`, `~/.asoundrc`, `~/.config`

### Bootstrap (New Board)

`deploy/bootstrap.sh` is a one-shot provisioning script for fresh Arduino UNO Q boards. It installs Docker, clones the repo, creates `.env`, sets up ALSA, builds the image, builds the workspace, and installs the systemd service.

### Model Sync

`deploy/sync-models.sh` is an rsync helper to copy large binary assets (LLM weights, Vosk models, Edge Impulse models, `lib/`) between boards. These assets are **not stored in git**.

### Systemd Services

- `systemd/aimee-robot.service` — Main robot service. Runs `docker compose up`, auto-restarts, with resource limits (3G address space, 2G RSS).
- `systemd/aimee-routing.service` — Network routing fix preventing OBSBOT USB RNDIS from hijacking the default gateway.

---

## Brick Framework

Many packages contain a `brick/` subdirectory, implementing a modular "brick" architecture where ROS2 nodes can be wrapped as Arduino App Lab bricks.

- `arduino` package provides `app_utils/brick.py` — base brick utilities
- `aimee_brick_template` provides boilerplate for new bricks
- `scripts/create_brick.py` — Brick generator tool (`--name`, `--class`, `--description`, `--category`)

Categories: `hardware`, `audio`, `vision`, `sensors`, `network`, `general`

---

## Security Considerations

- **Never commit `.env` to git.** It is already in `.gitignore`.
- All API keys are injected via environment variables. No hardcoded secrets exist in source code.
- `.env.example` is safe to commit (placeholders only).
- The Docker container runs `privileged: true` and `network_mode: host` for hardware and DDS access.
- Third-party vendored libraries (`libdev_v2.1.0_8/`, OBSBOT SDK) are subject to their own licenses.

---

## Documentation

Key documentation files in the repo:

| File | Content |
|------|---------|
| `README.md` | Quick start, package overview, bootstrap, config, troubleshooting |
| `Aimee_Project_Plan.md` | Comprehensive project plan v2.2 — architecture, brick framework, implementation phases, hardware specs, cloud protocol |
| `CHECKPOINT.md` | Session checkpoint log — daily work log, current state, known issues, modified files |
| `docs/AimeeCloud_Protocol_v1.4.md` | AimeeCloud MQTT/WebSocket protocol spec |
| `docs/BRICK_TO_STANDARD_ROS2_MIGRATION.md` | Migration guide from Arduino Brick framework to standard ROS2 |
| `docs/EDGE_IMPULSE_MODELS.md` | Edge Impulse model deployment docs |
| `docs/FUTURE_ENHANCEMENTS.md` | Roadmap |
| `docs/HARDWARE_SPEC.md` | UNO Q specs, Ron/Wren/Minnie hardware configs, resource constraints |
| `docs/TTS_PRODUCTION_PLAN.md` | TTS production deployment plan |

Individual packages also contain their own `README.md` files with topic interfaces and parameter docs.

---

## Important Notes for Agents

- The target hardware is an **Arduino UNO Q with 4GB RAM**. Always consider memory constraints. Use `--parallel-workers 1` for builds if needed.
- **Cyclone DDS is the default RMW**, not Fast DDS. The `fastdds_*.xml` files are legacy.
- The OBSBOT Tiny 2 camera uses USB RNDIS and can hijack the default gateway. The routing fix service exists for this reason.
- Large binary models (LLM weights, speech models, Edge Impulse models, `lib/`) are **not in git** and must be synced via `deploy/sync-models.sh` or downloaded separately.
- There is **no CI/CD pipeline** in this repository.
- Fleet configuration is config-driven via `config/robot_config.yaml`.

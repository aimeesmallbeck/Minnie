# Aimee Robot ROS2 Workspace

ROS2 Humble workspace for the Aimee social assistance robot running on **Arduino UNO Q**.

Request access to AimeeCloud API at http://www.aimeecloud.com

## Quick Start

```bash
# Source the environment
source ~/aimee-robot-ws/setup_env.sh

# Build the workspace
colcon build --symlink-install

# Launch the robot
ros2 launch aimee_bringup robot.launch.py robot:=ron
```

## Directory Structure

```
aimee-robot-ws/
├── src/
│   ├── aimee_msgs/           # Custom ROS2 messages and actions
│   ├── aimee_bringup/        # Launch files and configuration
│   ├── aimee_brick_template/ # Template for creating new bricks
│   │   └── brick_template/
│   └── aimee_*/              # Other packages (created in subsequent phases)
├── config/
│   ├── robot_config.yaml     # Hardware configuration
│   └── skills_config.yaml    # Intent routing and skills
├── scripts/
│   ├── create_brick.py       # Brick generator tool
│   └── fix_routing.sh        # Network routing fix
├── systemd/
│   ├── aimee-robot.service   # Main robot service
│   └── aimee-routing.service # Network routing service
├── fastdds_shm.xml           # Fast DDS Shared Memory configuration
├── setup_env.sh              # Environment setup script
└── README.md                 # This file
```

## ROS2 Packages

Each package below can be used standalone or as part of the full robot stack. All nodes are written in Python (rclpy) unless noted.

### Voice & Audio

| Package | Node(s) | Description |
|---------|---------|-------------|
| `aimee_wake_word_ei` | `wake_word_ei_node` | Edge Impulse keyword spotting — publishes `/wake_word/detected` |
| `aimee_voice_manager` | `voice_manager_node` | STT pipeline (Vosk local + Whisper/Lemonfox cloud fallback) — publishes `/voice/transcription` |
| `aimee_tts` | `tts_node` | Text-to-Speech with Lemonfox (primary), Kokoro, and gTTS (fallback) — subscribes `/tts/speak` |

### Intelligence & Cloud

| Package | Node(s) | Description |
|---------|---------|-------------|
| `aimee_intent_router` | `intent_router_node` | Classifies voice intents and routes to local skills or AimeeCloud — publishes `/intent/classified` |
| `aimee_skill_manager` | `skill_manager_node` | Skill execution engine — action server `/skill/execute` |
| `aimee_llm_server` | `llm_server_node` | Non-blocking LLM action server with streaming + preemption — action server `/llm/generate` |
| `aimee_cloud_bridge` | `cloud_bridge_node` | MQTT bridge to AimeeCloud — handles AimeeAgent protocol, voice metadata, and local command execution |

### Vision

| Package | Node(s) | Description |
|---------|---------|-------------|
| `aimee_vision_obsbot` | `obsbot_node`, `obsbot_keepalive_node` | OBSBOT Tiny 2 PTZ/tracking control via OSC SDK — subscribes `/camera/*` topics |
| `aimee_vision_pipeline` | `color_detector_node`, `object_tracker_node` | Color-based object detection and multi-object tracking — publishes `/vision/detections` |
| `aimee_perception` | `pose_estimator_node`, `grasp_planner_node` | 3D pose estimation and grasp strategy planning |

### Hardware Control

| Package | Node(s) | Description |
|---------|---------|-------------|
| `aimee_ugv02_controller` | `ugv02_controller_node`, `ugv02_teleop_node` | Waveshare UGV02 base control via JSON serial protocol — subscribes `/cmd_vel` |
| `aimee_lerobot_bridge` | `roarm_m3_http_driver` | RoArm-M3 arm control via HTTP/JSON — subscribes `/arm/command` |
| `aimee_manipulation` | `arm_controller_node`, `pick_place_server` | Arm control and PickPlace action server — action server `/manipulation/pick_place` |

### Utilities

| Package | Node(s) | Description |
|---------|---------|-------------|
| `aimee_ros2_monitor` | `monitor_node` | Web dashboard (port 8081) — node status, logs, camera stream, topic Hz |
| `aimee_test_dashboard` | `dashboard_node` | Hardware test dashboard with simulation mode |
| `aimee_bringup` | — | Launch files and robot profiles — entry point for `core.launch.py` |
| `aimee_msgs` | — | Custom ROS2 messages, services, and actions used across all packages |
| `aimee_brick_template` | — | Template/boilerplate for creating new Arduino bricks |

### Using Individual Packages

You don't need the full robot to use a single node. For example, to run just the TTS node:

```bash
ros2 run aimee_tts tts_node --ros-args -p default_engine:="lemonfox" -p lemonfox_api_key:="YOUR_KEY"
```

Or to use just the OBSBOT camera control:

```bash
ros2 run aimee_vision_obsbot obsbot_node --ros-args -p host:="192.168.5.1"
```

Each package has its own `README.md` in `src/<package>/` with topic interfaces and parameter docs.

## New Board Setup (Bootstrap)

For a fresh Arduino UNO Q, run the bootstrap script:

```bash
curl -fsSL https://raw.githubusercontent.com/aimeesmallbeck/Minnie/main/deploy/bootstrap.sh | bash
```

This will install Docker, clone this repo, build the container, and install the systemd service.

### Sync Models from an Existing Board

Large binary models (LLM weights, speech models, Edge Impulse wake-word models) are **not stored in git**. If you have another board already set up, sync them over:

```bash
cd ~/aimee-robot-ws
bash deploy/sync-models.sh arduino@<EXISTING_BOARD_IP>
```

## Post-Installation Configuration

After bootstrap (or cloning), you **must** configure your environment before starting the robot.

### 1. Environment Variables (`.env`)

Copy the template and edit:

```bash
cp ~/aimee-robot-ws/.env.example ~/aimee-robot-ws/.env
nano ~/aimee-robot-ws/.env
```

#### Required for All Setups

| Variable | Purpose | Example |
|----------|---------|---------|
| `AIMEE_ROBOT_NAME` | Unique name for this robot | `ron`, `minnie`, `wren` |
| `AIMEE_DEVICE_ID` | Unique device identifier | `arduino-uno-q-001` |
| `ROS_DOMAIN_ID` | ROS2 domain (use unique IDs per robot on same network) | `42` |

> **Fleet Note:** If running multiple robots on the same Wi-Fi, give each a different `ROS_DOMAIN_ID` (e.g., Ron=42, Minnie=43, Wren=44) to prevent crosstalk.

#### Required for Cloud Features

| Variable | Purpose | How to Obtain |
|----------|---------|---------------|
| `LEMONFOX_API_KEY` | TTS and Whisper STT fallback | https://www.lemonfox.ai/ (free tier available) |

#### Cloud Connection

| Variable | Default | Purpose |
|----------|---------|---------|
| `AIMEE_CLOUD_ENDPOINT` | `https://aimeecloud.com` | AimeeCloud API endpoint |
| `AIMEE_CLOUD_BROKER_HOST` | `aimeecloud.com` | MQTT broker host |
| `AIMEE_CLOUD_BROKER_PORT` | `443` | MQTT broker port |

#### Hardware-Specific

| Variable | Default | Purpose |
|----------|---------|---------|
| `OBSBOT_IP` | `192.168.5.1` | OBSBOT Tiny 2 USB RNDIS IP |
| `ROARM_SERIAL_PORT` | `/dev/ttyUSB0` | RoArm-M3 serial port |
| `UGV02_SERIAL_PORT` | `/dev/ttyACM0` | UGV02 base serial port |
| `ALSA_CARD_INDEX` | `0` | ALSA audio device card index |

#### Fast DDS Profile

| Variable | Default | Purpose |
|----------|---------|---------|
| `FASTRTPS_PROFILE` | `fastdds_shm.xml` | Shared Memory profile. Use `fastdds_disable_shm.xml` if you see `open_and_lock_file` errors. |

### 2. Hardware Configuration

Edit `config/robot_config.yaml` for your specific robot hardware:

```yaml
ron:
  hardware:
    base:
      serial_port: "/dev/ttyACM0"
    arm:
      serial_port: "/dev/ttyUSB0"
    cameras:
      main:
        ip: "192.168.5.1"
```

### 3. Audio Setup

The bootstrap creates a default `~/.asoundrc`. If audio doesn't work:

```bash
# List audio devices
aplay -l

# Update card index in .env if needed
ALSA_CARD_INDEX=1
```

### 4. Network Routing (OBSBOT Camera)

The OBSBOT Tiny 2 uses USB RNDIS and can hijack the default gateway. The bootstrap installs `aimee-routing.service` to fix this automatically. If cloud connectivity fails:

```bash
# Check routing
ip route show

# Fix manually
sudo ~/aimee-robot-ws/scripts/fix_routing.sh
```

## Critical Optimizations

This workspace includes three critical optimizations for the UNO Q's 4GB RAM:

### 1. Fast DDS Shared Memory

Prevents message duplication between local nodes:

```bash
# Already configured in setup_env.sh
export FASTRTPS_DEFAULT_PROFILES_FILE=~/aimee-robot-ws/fastdds_shm.xml
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

### 2. LLM Action Server

Non-blocking LLM with streaming and cancellation support.

### 3. Network Routing Fix

Prevents OBSBOT USB RNDIS from hijacking default gateway.

## Creating New Bricks

Use the brick generator script:

```bash
python3 scripts/create_brick.py \
    --name my_brick \
    --class MyBrick \
    --description "My awesome brick" \
    --category hardware
```

Categories: `hardware`, `audio`, `vision`, `sensors`, `network`, `general`

## Installation

### 1. Install System Services

```bash
# Network routing fix
sudo cp systemd/aimee-routing.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable aimee-routing

# Main robot service
sudo cp systemd/aimee-robot.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable aimee-robot
```

### 2. Add to .bashrc

```bash
echo 'source ~/aimee-robot-ws/setup_env.sh' >> ~/.bashrc
```

## Development

### Building

```bash
cd ~/aimee-robot-ws
source setup_env.sh
colcon build --symlink-install
```

### Testing

```bash
# Test voice
ros2 topic pub /voice/speak std_msgs/String "data: 'Hello!'"

# Test motion
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}}"

# View topics
ros2 topic list
ros2 topic echo /voice/transcription
```

## Troubleshooting

### Memory Issues

If running out of RAM:
- Check SHM configuration is loaded: `echo $FASTRTPS_DEFAULT_PROFILES_FILE`
- Monitor memory: `free -h`
- Reduce node count in launch file

### Network Issues

If cloud connectivity fails:
```bash
# Check routing
ip route show

# Fix routing manually
sudo ~/aimee-robot-ws/scripts/fix_routing.sh

# Test connectivity
ping 8.8.8.8
ping 192.168.5.1
```

### OBSBOT Camera Not Found

```bash
# Check USB network interface
ip addr show | grep 192.168.5

# List USB devices
lsusb | grep OBSBOT
```

### Docker Permission Denied

If you get permission errors after bootstrap:
```bash
newgrp docker
# Or log out and back in
```

### Out of Memory During Build

The UNO Q has 4GB RAM. If builds fail:
```bash
# Reduce parallel jobs
docker compose run --rm aimee-robot bash -c "colcon build --symlink-install --parallel-workers 1"
```

## Security Notes

- **Never commit `.env` to git.** It contains API keys and is already in `.gitignore`.
- All API keys are injected via environment variables. No hardcoded secrets exist in source code.
- The `.env.example` file is safe to commit and contains only placeholder values.

## License

This project is licensed under the **Mozilla Public License 2.0** (MPL-2.0).

### Third-Party Software

This repository includes vendored prebuilt libraries that are subject to their own licenses:

- **OBSBOT Device SDK** (`libdev_v2.1.0_8/`): Prebuilt binaries and headers from OBSBOT Technology Co., Ltd. used for camera control via the UVC/USB SDK. TODO: Review and document the exact OBSBOT SDK license terms. See `libdev_v2.1.0_8/LICENSE` (pending) or [OBSBOT Developer Center](https://www.obsbot.com/) for details.
- **Edge Impulse models** (`.arduino-bricks/ei-models/`): Subject to Edge Impulse licensing terms.
- **Vosk speech recognition models** (`vosk-models/`): Subject to Vosk/Apache 2.0 licensing terms.

> ⚠️ **TODO:** Audit all third-party dependencies and vendored libraries to ensure proper license attribution and compatibility with MPL-2.0. Track progress in `docs/licensing-audit.md` (to be created).

MPL-2.0

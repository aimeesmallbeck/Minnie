# Aimee Robot Project Plan v2.2
## Modular Skill-Based Architecture on ROS2 + Arduino Bricks

**Date:** April 11, 2026 (Updated)  
**Platform:** Arduino UNO Q (Raspberry Pi)  
**Architecture:** ROS2 Humble + Arduino Brick Framework  
**Cloud:** AimeeCloud (Digital Ocean)  

---

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [System Architecture](#system-architecture)
3. [Critical Optimizations](#critical-optimizations)
   - [Memory Optimization (4GB Limit)](#memory-optimization-4gb-limit)
   - [LLM Action Server](#llm-action-server)
   - [Network Routing](#network-routing)
4. [Hardware Specifications](#hardware-specifications)
5. [The Brick Framework](#the-brick-framework)
6. [ROS2 Node Architecture](#ros2-node-architecture)
7. [Brick Library](#brick-library)
8. [OBSBOT SDK Integration](#obsbot-sdk-integration)
9. [AimeeCloud Integration](#aimeecloud-integration)
10. [Implementation Phases](#implementation-phases)
11. [Configuration](#configuration)
12. [Next Steps](#next-steps)

---

## Executive Summary

This project plan outlines a complete rebuild of the Aimee robot system using **ROS2 Humble** as the core middleware combined with Arduino Q's **Brick Framework** for modular component management. The architecture follows a "Brain Stem + Skills" pattern where the UNO Q runs local real-time components, while extensible Skills can be deployed locally or in AimeeCloud.

### Key Design Decisions

| Decision | Rationale |
|----------|-----------|
| **ROS2 Humble** | Industry-standard, rich ecosystem, ARM64 support, LTS |
| **Arduino Brick Framework** | Modular, hot-pluggable components, standardized interfaces |
| **Edge Impulse Wake Word** | Custom-trained keyword spotting model |
| **OSC Protocol (OBSBOT)** | Network-based camera control, no USB drivers needed |
| **Cyclone DDS** | Stable, lightweight DDS — Nav2-recommended replacement for Fast DDS |
| **SQLite + ChromaDB** | Structured + semantic memory for user context |

### Robot Fleet

| Robot | Base | Arm | Main Camera | Lidar |
|-------|------|-----|-------------|-------|
| **Ron** | Waveshare UGV02 (0.23 m track) | RoArm-M3 | OBSBOT Tiny 2 | — |
| **Wren** | Waveshare Wave Rover (0.172 m track) | — | OBSBOT Tiny 2 | — |
| **Minnie** | Waveshare Wave Rover (0.172 m track) | — | OBSBOT Tiny 2 | LD19 |

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         AIMEE ROBOT SYSTEM v2.2                             │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                    BRICK LAYER (Arduino App Lab)                    │   │
│  │                                                                     │   │
│  │   ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌──────────┐  │   │
│  │   │ WakeWord    │  │ LocalASR    │  │ LocalTTS    │  │ LocalLLM │  │   │
│  │   │ Brick (EI)  │  │ Brick (Vosk)│  │ Brick(Piper)│  │ Brick    │  │   │
│  │   └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  └────┬─────┘  │   │
│  │          │                │                │              │        │   │
│  │   ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │        │   │
│  │   │ UGV02Ctrl   │  │ RoArmCtrl   │  │ CloudBridge │        │        │   │
│  │   │ Brick       │  │ Brick       │  │ Brick       │        │        │   │
│  │   └──────┬──────┘  └──────┬──────┘  └──────┬──────┘        │        │   │
│  │          └────────────────┴────────────────┘              │        │   │
│  │                             │                              │        │   │
│  └─────────────────────────────┼──────────────────────────────┼────────┘   │
│                                │                              │             │
│  ╔═════════════════════════════╧══════════════════════════════╧═══════════╗ │
│  ║                    ROS2 BRIDGE LAYER (rclpy)                           ║ │
│  ║                                                                         ║ │
│  ║   ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐   ║ │
│  ║   │ ROS2 Node   │  │ ROS2 Node   │  │ ROS2 Node   │  │ ROS2 Node   │   ║ │
│  ║   │ /voice      │  │ /intent     │  │ /memory     │  │ /skills     │   ║ │
│  ║   │ _manager    │  │ _router     │  │ _manager    │  │ _manager    │   ║ │
│  ║   └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘   ║ │
│  ║          │                │                │                │          ║ │
│  ║          └────────────────┴────────────────┴────────────────┘          ║ │
│  ║                            │                                           ║ │
│  ║                    ROS2 Topic Bus (Cyclone DDS)                        ║ │
│  ╚════════════════════════════╧═══════════════════════════════════════════╝ │
│                                │                                            │
│  ┌─────────────────────────────┼─────────────────────────────────────────┐  │
│  │                    SKILL LAYER (Local + Cloud)                        │  │
│  │                           │                                           │  │
│  │   ┌─────────────┐  ┌──────┴──────┐  ┌─────────────┐  ┌─────────────┐ │  │
│  │   │ SkillRobot  │  │ AimeeCloud  │  │ SkillGame   │  │ SkillIdentity│ │  │
│  │   │ Control     │  │ Client      │  │ Module      │  │             │ │  │
│  │   └─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘ │  │
│  └───────────────────────────────────────────────────────────────────────┘  │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
                                     │
                        ┌────────────┴────────────┐
                        ▼                         ▼
              ┌──────────────────┐      ┌──────────────────┐
              │   AimeeCloud     │      │   ROS2 Monitor   │
              │   (Skills API)   │      │  (Local :8081)   │
              └──────────────────┘      └──────────────────┘
```

---

## Critical Optimizations

Based on Gemini's feedback, the following optimizations are **critical** for production deployment on the UNO Q's 4GB RAM constraint:

### Memory Optimization & DDS Stability (4GB Limit)

**Problem:** ROS2 Humble's default Fast DDS middleware has documented stability issues with Nav2: discovery "storms" that spike CPU, service hangs in lifecycle transitions, and SHM segment leaks after crashes. On a 4GB ARM64 board, these issues cause navigation failures and require manual XML tuning.

**Solution:** Switch to **Cyclone DDS**, the RMW implementation recommended by the Nav2 maintainers for production use.

**Installation:**
```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

**Environment Setup:**
```bash
# Add to ~/.bashrc (or docker-compose.yml)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

**Benefits:**
- **Plug-and-play Nav2**: No XML tuning or Discovery Server required
- **Lightweight discovery**: Avoids the multicast "discovery storms" that plague Fast DDS in large node graphs
- **Reliable services**: Lifecycle transitions and BT Navigator service calls complete without hangs
- **Lower memory footprint**: Better memory management for large messages (OccupancyGrid, LaserScan) on 4GB RAM
- **No SHM cleanup issues**: Eliminates shared-memory segment leaks after node crashes

**Comparison:**

| Feature | Fast DDS (Default) | Cyclone DDS (Our Choice) |
|---|---|---|
| Out-of-the-box Nav2 | Requires XML tuning or Discovery Server | Works plug-and-play |
| Discovery | High overhead; prone to "storms" | Lightweight and stable |
| Service Reliability | Historically prone to hangs in complex BTs | Renowned for reliability |
| Performance | Better raw throughput | Better stability for complex graphs |

---

### LLM Action Server

**Problem:** ROS2 Services are blocking and wait for 100% completion. With Qwen2.5 taking 2-5 seconds per generation, this locks up the calling node.

**Solution:** Implement LLM as a **ROS2 Action Server** for:
- Long-running task support
- Live feedback (streaming tokens)
- Preemption (cancel generation on "Stop!" command)

**Action Definition:**
```idl
# aimee_msgs/action/LLMGenerate.action
# Goal
string prompt
string system_context
int32 max_tokens
float32 temperature
bool stream
---
# Result
string response
bool success
string error_message
float32 generation_time
---
# Feedback
string partial_response
int32 tokens_generated
int32 tokens_total
bool is_complete
```

**Implementation:**
```python
#!/usr/bin/env python3
"""llm_action_server.py - Non-blocking LLM with streaming"""

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from aimee_msgs.action import LLMGenerate
from arduino.app_bricks.local_llm import LocalLLM


class LLMActionServer(Node):
    def __init__(self):
        super().__init__('llm_action_server')
        
        self.llm = LocalLLM(
            model="Qwen2.5-0.5B-Instruct-Q4_K_M.gguf",
            host="localhost",
            port=8080
        )
        
        self._action_server = ActionServer(
            self,
            LLMGenerate,
            'llm/generate',
            self.execute_callback
        )
        
        self._current_goal = None
        
    async def execute_callback(self, goal_handle):
        """Handle LLM generation with streaming feedback"""
        self._current_goal = goal_handle
        request = goal_handle.request
        
        feedback_msg = LLMGenerate.Feedback()
        result = LLMGenerate.Result()
        
        full_response = []
        tokens_generated = 0
        
        try:
            # Stream tokens as they're generated
            for token in self.llm.chat_stream(request.prompt):
                # Check for cancellation
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.error_message = "Generation cancelled by user"
                    self.get_logger().info('LLM generation cancelled')
                    return result
                
                full_response.append(token)
                tokens_generated += 1
                
                # Send feedback every few tokens
                if tokens_generated % 3 == 0:
                    feedback_msg.partial_response = ''.join(full_response)
                    feedback_msg.tokens_generated = tokens_generated
                    feedback_msg.is_complete = False
                    goal_handle.publish_feedback(feedback_msg)
                    
            # Complete
            final_response = ''.join(full_response)
            result.response = final_response
            result.success = True
            result.generation_time = 0.0  # Calculate actual time
            
            goal_handle.succeed()
            self.get_logger().info(f'LLM generation complete: {tokens_generated} tokens')
            
        except Exception as e:
            result.success = False
            result.error_message = str(e)
            goal_handle.abort()
            self.get_logger().error(f'LLM generation failed: {e}')
            
        return result


def main(args=None):
    rclpy.init(args=args)
    node = LLMActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Client Usage (Intent Router):**
```python
from rclpy.action import ActionClient
from aimee_msgs.action import LLMGenerate

class IntentRouter(Node):
    def __init__(self):
        self._llm_client = ActionClient(self, LLMGenerate, 'llm/generate')
        
    async def generate_response(self, prompt):
        goal_msg = LLMGenerate.Goal()
        goal_msg.prompt = prompt
        goal_msg.max_tokens = 150
        goal_msg.stream = True
        
        # Non-blocking send
        self._send_goal_future = self._llm_client.send_goal_async(
            goal_msg,
            feedback_callback=self.on_llm_feedback
        )
        
    def on_llm_feedback(self, feedback_msg):
        # Update UI or speak partial response
        partial = feedback_msg.feedback.partial_response
        self.get_logger().info(f'LLM: {partial}')
        
    def cancel_generation(self):
        # User said "Stop!"
        if self._current_goal_handle:
            self._current_goal_handle.cancel_goal_async()
```

**Benefits:**
- Non-blocking: Intent router continues processing while LLM generates
- Streaming: Can start speaking response while still generating
- Preemptable: Immediate cancellation on "Stop!" command
- Progress feedback: Shows thinking/generation status

---

### Network Routing

**Problem:** OBSBOT Tiny 2 uses USB RNDIS at `192.168.5.1`, which can cause the UNO Q to route general internet traffic through the USB interface instead of Wi-Fi, breaking AimeeCloud connections.

**Solution:** Configure routing tables to ensure Wi-Fi is the default gateway for internet traffic.

**Check Current Routing:**
```bash
# View routing table
ip route show

# Expected output:
default via 192.168.1.1 dev wlan0 proto dhcp metric 600
192.168.5.0/24 dev usb0 proto kernel scope link src 192.168.5.2 metric 100
```

**Problem Scenario:**
```bash
# WRONG - USB interface has lower metric (higher priority)
default via 192.168.5.1 dev usb0 proto dhcp metric 100  # OBSBOT
192.168.1.0/24 dev wlan0 proto kernel scope link src 192.168.1.50 metric 600
```

**Fix: Persistent Routing Configuration:**
```bash
# /etc/dhcpcd.conf
# Ensure Wi-Fi has higher priority (lower metric)
interface wlan0
    metric 100

interface usb0
    metric 800
    # Only route OBSBOT subnet through USB
    static routers=
    static domain_name_servers=
```

**Alternative: Route Script:**
```bash
#!/bin/bash
# /home/arduino/aimee-robot/scripts/fix_routing.sh

# Delete default route via USB if it exists
sudo ip route del default via 192.168.5.1 2>/dev/null

# Ensure default route is via Wi-Fi
sudo ip route add default via 192.168.1.1 dev wlan0 metric 100

# Add specific route for OBSBOT (local only)
sudo ip route add 192.168.5.0/24 dev usb0 metric 800

echo "Routing configured:"
ip route show
```

**Systemd Service for Persistence:**
```ini
# /etc/systemd/system/aimee-routing.service
[Unit]
Description=Aimee Robot Network Routing
After=network.target

[Service]
Type=oneshot
ExecStart=/home/arduino/aimee-robot/scripts/fix_routing.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

**Verification:**
```bash
# Enable service
sudo systemctl enable aimee-routing.service

# Test connectivity
ping -c 3 8.8.8.8          # Internet via Wi-Fi
ping -c 3 192.168.5.1      # OBSBOT via USB

curl -I https://aimeecloud.example.com  # Cloud API
```

**Benefits:**
- Ensures cloud connectivity regardless of USB camera connection
- Isolates OBSBOT traffic to local subnet only
- Automatic recovery on network changes

---

## Hardware Specifications

### Ron (UGV02 + RoArm-M3)

```yaml
robot:
  name: "ron"
  type: "ugv_arm"
  
  hardware:
    base:
      type: "waveshare_ugv02"
      serial_port: "/dev/ttyACM0"
      baud: 115200
      protocol: "json"
      
    arm:
      type: "roarm_m3"
      serial_port: "/dev/ttyUSB0"
      baud: 115200
      control_mode: "json"  # Waveshare JSON protocol
      degrees_of_freedom: 6
      
    cameras:
      main:
        type: "obsbot_tiny2"
        connection: "usb_network"
        ip: "192.168.5.1"  # USB RNDIS default
        osc_port: 16284
        resolution: "4K"
        
      arm_end:
        type: "ov7670"
        interface: "i2c"
        i2c_bus: 1
        
    led_matrix:
      type: "max7219"
      interface: "spi"
      spi_bus: 0
      cs_pin: 8
      
    sensors:
      imu: "mpu6050"
      battery_monitor: "ina219"
```

### Wren (Wave Rover)

```yaml
robot:
  name: "wren"
  type: "wave_rover"
  
  hardware:
    base:
      type: "waveshare_wave_rover"
      serial_port: "/dev/ttyACM0"
      baud: 115200
      
    cameras:
      main:
        type: "obsbot_tiny2"
        ip: "192.168.5.2"
        osc_port: 16284
```

### UGV02 Communication Protocol

```python
# Serial protocol (Waveshare standard)
# Baud: 115200, 8N1

# Move command
{"T": 1, "L": 0.5, "R": 0.5}  # Left/Right speed -1.0 to 1.0

# Get odometry
{"T": 2} → Response: {"T":2,"L":100,"R":100,"X":10,"Y":5,"Z":90}

# LED control
{"T": 3, "R": 255, "G": 0, "B": 0}  # RGB values
```

### RoArm-M3 Communication Protocol

```python
# Serial: /dev/ttyUSB0, 115200

# Move to position (Joint control)
{"T":101,"joint":[90,90,90,90,90,90],"spd":10}

# IK control (Cartesian)
{"T":104,"x":200,"y":0,"z":150,"t":90,"r":0,"g":0,"spd":10}

# Gripper
{"T":110,"g":90}  # 0-90 degrees
```

---

## The Brick Framework

### What is a Brick?

In the Arduino Q environment, a **Brick** is a standardized, modular component that:
- Is self-contained with its own dependencies
- Uses the `@brick` decorator for registration
- Has a `brick_config.yaml` for configuration
- Follows a consistent lifecycle: `initialize()` → `run()` → `shutdown()`
- Can be hot-plugged without affecting other components

### Brick Structure

```
brick_name/
├── pyproject.toml              # Package metadata & dependencies
├── src/
│   └── arduino/
│       └── app_bricks/
│           └── brick_name/
│               ├── __init__.py         # Exports main class
│               ├── brick_config.yaml   # Config variables
│               └── brick_name.py       # Main implementation
```

### Existing Bricks in Your System

```python
# From chatbot-local-llm - Formal brick structure
from arduino.app_bricks.local_llm import LocalLLM
llm = LocalLLM(model="Qwen2.5-0.5B", host="localhost", port=8080)
response = llm.chat("Hello, Aimee!")

# From aimee-voice-offline - Simple brick pattern
from bricks.aimee_local_asr import LocalAsrBrick
asr = LocalAsrBrick()
transcript = asr.transcribe()

from bricks.aimee_local_tts import LocalTtsBrick
tts = LocalTtsBrick()
tts.speak("Hello, I'm Aimee!")
```

---

## ROS2 Node Architecture

> **⚠️ DEVELOPMENT NOTE:** The UNO Q board has a container running ROS2. Six nodes have been developed following the Arduino Brick pattern. When creating new nodes, compare against the existing structure to maintain consistency.

Each ROS2 node wraps a brick and handles:
1. **Topic Pub/Sub** - ROS2 communication
2. **Parameter Server** - Dynamic configuration
3. **Services/Actions** - Request/response patterns
4. **Brick Lifecycle** - Initialize, run, cleanup

### Existing ROS2 Nodes (10 Nodes Complete)

| Node | Package | Purpose | ROS2 Interface | Status |
|------|---------|---------|----------------|--------|
| `wake_word_ei` | `aimee_wake_word_ei` | Edge Impulse wake word detection | Publishes `/wake_word/detected` | ✅ **COMPLETE** |
| `voice_manager` | `aimee_voice_manager` | Vosk STT/Voice recording (standard ROS2 node) | Publishes `/voice/transcription` | ✅ **COMPLETE** |
| `tts` | `aimee_tts` | Lemonfox primary + Kokoro/gTTS fallback | Subscribes `/tts/speak` | ✅ **COMPLETE** |
| `llm_server` | `aimee_llm_server` | LLM Action Server | **Action** `/llm/generate` | ✅ **COMPLETE** |
| `intent_router` | `aimee_intent_router` | Intent classification & routing (local only, rest → AimeeAgent) | Publishes `/intent/classified` | ✅ **COMPLETE** |
| `skill_manager` | `aimee_skill_manager` | Skill execution management | **Action** `/skill/execute` | ✅ **COMPLETE** |
| `obsbot_node` | `aimee_vision_obsbot` | OBSBOT PTZ/tracking control (SDK) | Subscribes `/camera/*` topics | ✅ **COMPLETE** |
| `usb_camera` | `usb_cam` | OBSBOT video streaming (UVC) | Publishes `/camera/image_raw` | ✅ **COMPLETE** |
| `color_detector` | `aimee_vision_pipeline` | Color-based object detection | Publishes `/vision/detections` | ✅ **COMPLETE** |
| `object_tracker` | `aimee_vision_pipeline` | Multi-object tracking | Publishes `/vision/tracked_objects` | ✅ **COMPLETE** |
| `pose_estimator` | `aimee_perception` | 3D pose estimation | Publishes `/vision/detections_3d` | ✅ **COMPLETE** |
| `grasp_planner` | `aimee_perception` | Grasp strategy planning | Publishes `/manipulation/grasp_pose` | ✅ **COMPLETE** |
| `arm_controller` | `aimee_manipulation` | Arm control (simulated) | Subscribes `/arm/command` | ✅ **COMPLETE** |
| `pick_place_server` | `aimee_manipulation` | PickPlace action server | **Action** `/manipulation/pick_place` | ✅ **COMPLETE** |
| `ros2_monitor` | `aimee_ros2_monitor` | ROS2 management console | Web UI + `/rosout` | ✅ **COMPLETE** |

### Node Structure Pattern

All nodes follow this consistent Arduino Brick pattern:

```
aimee_<node_name>/
├── aimee_<node_name>/
│   ├── __init__.py
│   ├── <node_name>_node.py      # ROS2 node (publishes/subscribes)
│   └── brick/
│       ├── __init__.py
│       └── <brick_name>.py      # Brick implementation (@brick decorator)
├── package.xml
├── setup.py
└── setup.cfg
```

### Core ROS2 Topics

| Topic | Message Type | Publisher | Subscriber |
|-------|--------------|-----------|------------|
| `/wake_word/detected` | `WakeWordDetection` | WakeWord Brick | Voice Manager |
| `/voice/transcription` | `Transcription` | ASR Brick | Intent Router |
| `/voice/partial` | `Transcription` | ASR Brick | (UI/Dashboard) |
| `/tts/speak` | `String` | Any Skill | TTS Brick |
| `/tts/is_speaking` | `Bool` | TTS Brick | Skills |
| `/intent/classified` | `Intent` | Intent Router | Skill Manager |
| `/intent/status` | `String` | Intent Router | Dashboard |
| `/skill/execute` | `ExecuteSkill` (Action) | Intent Router | Skill Manager |
| `/skill/status` | `String` | Skill Manager | Dashboard |
| `/cmd_vel` | `Twist` | Skill/Nav | Motion Controller |
| `/arm/command` | `ArmCommand` | Skill | Arm Controller |
| `/camera/image_raw` | `Image` | Camera Driver | Vision Pipeline |
| `/vision/detections` | `ObjectDetection` | Color Detector | Pose Estimator |
| `/vision/tracked_objects` | `ObjectDetection` | Object Tracker | Pose Estimator |
| `/vision/detections_3d` | `ObjectDetection` | Pose Estimator | Grasp Planner |
| `/manipulation/grasp_pose` | `GraspPose` | Grasp Planner | PickPlace Server |
| `/manipulation/pick_place` | `PickPlace` (Action) | Skills | PickPlace Server |
| `/camera/face_detected` | `FaceDetection` | Face Recog | Skills |
| `/robot/state` | `RobotState` | System | All Nodes |
| `/llm/generate` | `LLMGenerate` (Action) | Skills / Other nodes | LLM Server |

---

## Brick Library

### Core Bricks (Must Have)

| Brick | Purpose | ROS2 Interface | Status |
|-------|---------|----------------|--------|
| `brick_wake_word_ei` | Edge Impulse keyword spotting | Publishes `/wake_word/detected` | ✅ **COMPLETE** |
| `brick_local_asr` | Vosk speech-to-text | Publishes `/voice/transcription` | ✅ **COMPLETE** |
| `brick_local_tts` | Piper/gTTS text-to-speech | Subscribes `/voice/speak` | ✅ **COMPLETE** |
| `brick_local_llm` | Local LLM inference | **Action** `/llm/generate` | ✅ **COMPLETE** |
| `brick_ugv02_ctrl` | Waveshare UGV02 base control | Subscribes `/cmd_vel` | **TODO** |
| `brick_roarm_ctrl` | RoArm-M3 arm control | Subscribes `/arm/command` | **TODO** |
| `brick_cloud_bridge` | AimeeCloud communication | Pub/Sub cloud topics | **TODO** |
| `brick_memory_sqlite` | Local persistence | Service `/memory/*` | **TODO** |
| `brick_vision_obsbot` | OBSBOT Tiny 2 control (SDK) | Publishes `/camera/*` | ✅ **COMPLETE** |
| `brick_color_detector` | Color-based object detection | Publishes `/vision/detections` | ✅ **COMPLETE** |
| `brick_pose_estimator` | Monocular 3D pose estimation | Publishes `/vision/detections_3d` | ✅ **COMPLETE** |
| `brick_grasp_planner` | Grasp strategy planning | Publishes `/manipulation/grasp_pose` | ✅ **COMPLETE** |
| `brick_arm_controller` | RoArm-M3 arm control | Subscribes `/arm/command` | ✅ **COMPLETE (Simulated)** |

### Optional Bricks (Nice to Have)

| Brick | Purpose |
|-------|---------|
| `brick_led_matrix` | MAX7219 LED face expressions |
| `brick_face_recognition` | Face detection/recognition |
| `brick_battery_monitor` | Power management |
| `brick_emergency_stop` | Safety systems |

---

## OBSBOT SDK Integration

### OSC Protocol Overview

The OBSBOT Tiny 2 SDK uses **OSC (Open Sound Control)** protocol over UDP:
- **Default IP:** 192.168.5.1 (USB RNDIS)
- **Send Port:** 16284 (camera listening)
- **Receive Port:** 9000 (for status)

### Available OSC Commands

| Feature | OSC Address | Parameters | Description |
|---------|-------------|------------|-------------|
| **Gimbal Movement** | `/OBSBOT/WebCam/General/SetGimbalUp` | `[speed]` (0-100) | Pan/Tilt control |
| | `/OBSBOT/WebCam/General/SetGimbalDown` | `[speed]` | |
| | `/OBSBOT/WebCam/General/SetGimbalLeft` | `[speed]` | |
| | `/OBSBOT/WebCam/General/SetGimbalRight` | `[speed]` | |
| **Zoom** | `/OBSBOT/WebCam/General/SetZoom` | `[level]` (0-100) | Digital zoom 1x-4x |
| **AI Tracking** | `/OBSBOT/WebCam/General/SetTrackingMode` | `[mode]` | Tracking modes |
| **Sleep/Wake** | `/OBSBOT/WebCam/General/Sleep` | `[]` | Power management |
| | `/OBSBOT/WebCam/General/WakeUp` | `[]` | |
| **Presets** | `/OBSBOT/WebCam/General/SetPresetPosition` | `[position_id]` | Save positions |
| | `/OBSBOT/WebCam/General/CallPresetPosition` | `[position_id]` | Recall positions |

### AI Tracking Modes

| Mode ID | Mode Name | Description |
|---------|-----------|-------------|
| `0` | `normal` | Person tracking (default) |
| `1` | `upper_body` | Upper body tracking |
| `2` | `closeup` | Closeup tracking |
| `3` | `headless` | Below head tracking |
| `4` | `desk` | Desk mode (30° down) |
| `5` | `whiteboard` | Whiteboard mode |
| `6` | `hand` | Hand tracking |
| `7` | `group` | Group tracking |

### Brick Implementation: brick_vision_obsbot

```python
#!/usr/bin/env python3
"""OBSBOT Vision Brick - PTZ Camera Control via OSC SDK"""

import asyncio
from typing import Optional, Callable
from pythonosc.udp_client import SimpleUDPClient
from pythonosc.osc_server import AsyncIOOSCUDPServer
from pythonosc.dispatcher import Dispatcher
from arduino.app_utils import brick

@brick
class ObsbotBrick:
    """Control OBSBOT Tiny 2 camera via OSC protocol."""
    
    def __init__(
        self,
        host: str = "192.168.5.1",
        send_port: int = 16284,
        receive_port: int = 9000,
        name: str = "obsbot_main",
        **kwargs
    ):
        self.host = host
        self.send_port = send_port
        self.receive_port = receive_port
        self.name = name
        self.client: Optional[SimpleUDPClient] = None
        self.server: Optional[AsyncIOOSCUDPServer] = None
        self.is_connected = False
        self.current_zoom = 0
        self.tracking_mode = "off"
        self.status_callbacks: list[Callable] = []
        
        self.TRACKING_MODES = {
            "off": -1, "normal": 0, "upper_body": 1, "closeup": 2,
            "headless": 3, "desk": 4, "whiteboard": 5, "hand": 6, "group": 7
        }
        
    async def initialize(self):
        """Initialize OSC client and server"""
        self.client = SimpleUDPClient(self.host, self.send_port)
        
        dispatcher = Dispatcher()
        dispatcher.map("/OBSBOT/WebCam/General/Response", self._on_status)
        dispatcher.map("/OBSBOT/WebCam/General/Error", self._on_error)
        
        self.server = AsyncIOOSCUDPServer(
            ("0.0.0.0", self.receive_port), dispatcher, asyncio.get_event_loop()
        )
        
        self.server_transport, _ = await self.server.create_serve_endpoint()
        self._send("/OBSBOT/WebCam/General/GetStatus")
        await asyncio.sleep(0.5)
        
        self.is_connected = True
        return self
        
    def _send(self, address: str, args: list = None):
        if self.client:
            self.client.send_message(address, args or [])
            
    def gimbal_up(self, speed: int = 50):
        self._send("/OBSBOT/WebCam/General/SetGimbalUp", [speed])
        
    def gimbal_down(self, speed: int = 50):
        self._send("/OBSBOT/WebCam/General/SetGimbalDown", [speed])
        
    def gimbal_left(self, speed: int = 50):
        self._send("/OBSBOT/WebCam/General/SetGimbalLeft", [speed])
        
    def gimbal_right(self, speed: int = 50):
        self._send("/OBSBOT/WebCam/General/SetGimbalRight", [speed])
        
    def set_zoom(self, level: int):
        """Set zoom level: 0-100 (0=1x, 50=2x, 100=4x)"""
        self.current_zoom = level
        self._send("/OBSBOT/WebCam/General/SetZoom", [level])
        
    def set_tracking_mode(self, mode: str):
        mode_id = self.TRACKING_MODES.get(mode, 0)
        self.tracking_mode = mode
        self._send("/OBSBOT/WebCam/General/SetTrackingMode", [mode_id])
        
    def sleep(self):
        self._send("/OBSBOT/WebCam/General/Sleep")
        
    def wake_up(self):
        self._send("/OBSBOT/WebCam/General/WakeUp")
        
    def save_preset(self, position_id: int):
        self._send("/OBSBOT/WebCam/General/SetPresetPosition", [position_id])
        
    def recall_preset(self, position_id: int):
        self._send("/OBSBOT/WebCam/General/CallPresetPosition", [position_id])
        
    def track_face(self, face_position: tuple):
        """Auto-adjust gimbal to track face position (x, y: 0.0-1.0)"""
        x, y = face_position
        if x < 0.4:
            self.gimbal_left(speed=int((0.5 - x) * 100))
        elif x > 0.6:
            self.gimbal_right(speed=int((x - 0.5) * 100))
        if y < 0.4:
            self.gimbal_up(speed=int((0.5 - y) * 100))
        elif y > 0.6:
            self.gimbal_down(speed=int((y - 0.5) * 100))
```

### Benefits of OSC-Based SDK

| Benefit | Explanation |
|---------|-------------|
| **No USB complexity** | Network protocol - no kernel drivers |
| **Multiple cameras** | Each camera has unique IP |
| **Async-friendly** | UDP doesn't block |
| **Well-documented** | Standard OSC protocol |
| **Cross-platform** | Works on Linux/Windows/macOS |
| **Remote capable** | Can control over network |

---

## AimeeCloud Integration

### Branding & Naming Conventions
Any external cloud service integration **must** be branded as **AimeeCloud** in code, configuration, documentation, and user-facing messages. Do not use generic terms such as `cloud_proxy`, `cloud_bridge`, or `cloud_skill` in user-facing logic, ROS message fields, or configuration files.

| Context | Correct | Incorrect |
|---------|---------|-----------|
| `skill_name` in Intent message | `AimeeCloud` | `cloud_proxy` |
| User-facing docs / logs | "AimeeCloud" | "the cloud", "cloud bridge" |
| Node reference | "AimeeCloud client" | "cloud bridge node" |

### Intent Routing to AimeeCloud

The Intent Router loads keyword/phrase matching rules from an external JSON configuration (`/workspace/config/aimee_intent_config.json`). The config is reloaded on every utterance so edits take effect without restarting the node.

**Routing Logic:**
- Intents with `skill_name` in `local_only_skill_names` (`movement`, `arm_control`, `camera`) execute locally with fallback TTS.
- All other intents are forwarded to AimeeCloud by setting `skill_name = "AimeeCloud"`.
- The AimeeCloud Client (ACC) subscribes to `/intent/classified` and forwards any message where `msg.skill_name == "AimeeCloud"` as an **`AimeeAgent`** request.

**AimeeAgent Mode**

Instead of sending `type: "intent"` messages to AimeeCloud, the ACC now publishes `type: "AimeeAgent"` messages. This bypasses AimeeCloud's keyword router and sends the request directly to the LLM agent. Responses come back with `sub_type: "aimee_agent"` and may include a `commands` array that the ACC executes locally.

**Voice Metadata (Protocol v1.2)**

Every outbound response now includes a `voice` object that tells the robot which TTS voice to use. The ACC maps the `voice.id` to the local TTS engine and publishes formatted speak messages (e.g., `lemonfox|sarah:Hello!`).

```json
{
  "voice": {
    "persona": "aimee-default",
    "id": "sarah",
    "provider": "lemonfox",
    "lang": "en",
    "description": "Warm, friendly default Aimee voice"
  }
}
```

For rich storytelling, responses may also include `voice_segments` — an array of `{speaker, text, voice}` objects that the robot synthesizes and plays sequentially.

**AimeeAgent Command Reference:**

| Command | Action | ROS2 Output |
|---------|--------|-------------|
| `motor` | `{ "action": "forward", "duration_ms": 1000 }` | `Twist` on `/cmd_vel` |
| `arm` | `{ "action": "raise" }` | `ArmCommand` on `/arm/command` |
| `gripper` | `{ "action": "open" }` | `ArmCommand` on `/arm/command` |
| `snapshot` | `{ "camera": "front", "purpose": "analysis" }` | Capture → upload response to cloud |
| `game_move` | `{ "game": "tic-tac-toe", "position": 4 }` | `CloudIntent` on `/game/command` |

**AimeeCloud-Compatible Intent Types (legacy keyword router):**
`chat`, `weather`, `news`, `story`, `game`, `help`, `status`, `robot_forward/backward/left/right/stop`, `arm_raise/lower`, `gripper_open/close`, `unclassified`

### Cloud API Contract

The AimeeCloud protocol is documented in `docs/AimeeCloud_Protocol_v1.4.md`.

#### Robot → Cloud Request (AimeeAgent)

```json
{
  "type": "AimeeAgent",
  "device_id": "arduino-uno-q-001",
  "session_id": "sess-abc-123",
  "payload": "Look at the red block and tell me what you see",
  "timestamp": "2026-04-16T11:30:00Z"
}
```

#### Cloud → Robot Response (AimeeAgent)

```json
{
  "type": "response",
  "sub_type": "aimee_agent",
  "session_id": "sess-abc-123",
  "device_id": "arduino-uno-q-001",
  "text": "Sure, let me take a look.",
  "tts": "Sure, let me take a look.",
  "voice": {
    "persona": "aimee-default",
    "id": "sarah",
    "provider": "lemonfox",
    "lang": "en"
  },
  "commands": [
    { "type": "snapshot", "camera": "front", "purpose": "analysis" }
  ],
  "context": {
    "active_context": null,
    "context_stack": []
  },
  "timestamp": "2026-04-16T11:30:00Z"
}
```

### Dashboard Integration

The **local ROS2 Management Console** runs on the UNO Q at `http://192.168.1.100:8081`:

| Widget | Data Source | ROS2 Topic |
|--------|-------------|------------|
| **Core Control** | `aimee_bringup/core.launch.py` | Start/stop entire ROS2 stack |
| **Node Launcher** | `ros2 run` | Start/stop individual nodes |
| **Node Status** | `/api/nodes` | Visual cards per running node |
| **Log Viewer** | `/rosout` | Real-time logs with filtering |
| **Camera Feed** | `/camera/image_raw` | MJPEG stream with PTZ controls |
| **Topic Monitor** | `ros2 topic list` | Live Hz / bandwidth estimates |
| **Robot Status** | `/system/status` | Battery, temp, uptime |
| **Voice Activity** | `/voice/transcription` | Live transcription |
| **Intent Log** | `/intent/routing` | Classified intents |
| **Skill Status** | `/skills/active` | Running skills |
| **Memory Browser** | Service `/memory/query` | User data (privacy-safe) |
| **Location Map** | `/nav/position` | Robot location |

The old `aimee_test_dashboard` (direct hardware tester) has been retired. All control now flows through ROS2 topics and actions.

---

## Implementation Phases

### Phase 1: Core Infrastructure (Week 1-2)
- [x] ROS2 Humble installation & workspace setup
- [x] Create `~/aimee-robot-ws/` directory structure
- [x] `aimee_msgs` package with custom message types
- [x] `aimee_bringup` with launch files
- [x] Brick template/boilerplate code
- [ ] Systemd service files

### Phase 2: Voice Bricks (Week 3) ✅ COMPLETE
- [x] `brick_wake_word_ei` - Edge Impulse integration
- [x] `brick_local_asr` - Vosk server wrapper
- [x] `brick_local_tts` - Piper TTS integration
- [x] ROS2 `voice_manager_node`
- [x] ROS2 `intent_router_node`

### Phase 3: Intelligence Bricks (Week 4) ✅ COMPLETE
- [x] `brick_local_llm` - Local LLM inference
- [x] `brick_intent_router` - Intent classification
- [x] ROS2 `llm_server_node` (Action Server)
- [x] ROS2 `skill_manager_node`

### Phase 4: Hardware Control Bricks (Week 5) ✅ COMPLETE
- [x] `aimee_ugv02_controller` - UGV02 base control with JSON protocol
- [x] `ugv02_controller_node` - Serial comm, odometry, TF broadcast
- [x] `ugv02_teleop_node` - Keyboard teleoperation
- [x] Nav2 configuration for UGV02
- [x] Multi-base platform support (UGV02 + Wave Rover via parameterized YAML)
- [ ] Real hardware testing
- [ ] `brick_roarm_ctrl` - RoArm-M3 control (awaiting hardware)
- [ ] ROS2 `motion_manager_node`

### Phase 4b: SLAM & Navigation (2026-04-22) ✅ COMPLETE
- [x] `aimee_description` package — URDF for Minnie (`base_footprint`, `base_link`, `base_laser`, `camera`)
- [x] `robot_state_publisher` integration in `robot.launch.py`
- [x] `slam_toolbox` online sync launch (`slam.launch.py`)
- [x] `nav2_bringup` wrapper launch (`nav2.launch.py`)
- [x] Combined robot + SLAM + Nav2 launch (`navigation.launch.py`)
- [x] Nav2 params tuned for Minnie (`robot_radius: 0.15`, `max_vel_x: 0.5`, Humble-compatible)
- [x] SLAM params tuned for LD19 (`max_laser_range: 12.0`, `min_laser_range: 0.05`)
- [x] Static TF tree managed via URDF (no duplicate publishers)
- [x] Config-driven bringup supports `use_lidar` CLI override

### Phase 4c: Integrated Navigation — AimeeNav (2026-04-23) 🔄 IN PROGRESS
- [x] `aimee_nav` package created — self-contained navigation node
- [x] Direct LD19 lidar driver (`ld19_driver.py`) — no distributed `ldlidar_stl_ros2` node
- [x] Direct Wave Rover driver (`wave_rover_driver.py`) — HTTP/serial, in-process odometry
- [x] Local occupancy grid (`local_grid_map.py`) — 2D grid with Bresenham ray-casting
- [x] Simple A* planner (`simple_planner.py`) — grid-based path planning
- [x] Reactive obstacle avoidance (`obstacle_avoidance.py`) — sector analysis + VFF
- [x] PID heading/velocity controller (`pid_controller.py`)
- [x] `robot.launch.py` supports `navigation_mode: integrated` vs `distributed`
- [x] Performance tuning for UNO Q: reduced nav rate, conditional grid updates, decimated publishing
- [ ] Obstacle avoidance test validated on hardware
- [ ] Goal-directed navigation test validated on hardware
- [ ] Action server `navigate_to_pose` (stretch goal)

**Design rationale:** The distributed stack (`ldlidar` → `slam_toolbox` → `nav2` → `base_controller`) works but consumes ~400 MB RAM and suffers from DDS queue overflows on the 4 GB UNO Q. AimeeNav replaces the entire pipeline with a single Python node, targeting ~100 MB RAM and zero inter-node latency for the control loop.

### Phase 5: Cloud Integration (Week 6)
- [ ] `brick_cloud_bridge` - AimeeCloud communication
- [ ] Cloud skill dispatcher
- [ ] Offline message queue

### Phase 6: Vision Bricks (Week 7) ✅ COMPLETE
- [x] `aimee_vision_obsbot` - OBSBOT Tiny 2 PTZ/tracking control (SDK-based)
- [x] `usb_cam` driver - Dedicated MJPEG video streaming node (C++)
- [x] `aimee_vision_pipeline` - Color-based object detection
- [x] `aimee_perception` - 3D pose estimation & grasp planning
- [x] `aimee_manipulation` - Arm control & PickPlace action
- [x] New message types: ObjectDetection, GraspPose, ArmCommand
- [x] PickPlace.action server with full feedback
- [ ] Face recognition pipeline (future)
- [ ] OV7670 arm-end camera (future)
- [ ] Multi-camera support (future)

### Phase 7: Memory & Skills Framework (Week 8)
- [ ] `brick_memory_sqlite` - Context persistence
- [ ] Skill base class and loader
- [ ] `SkillRobotControl` - Hardware control
- [ ] `SkillIdentity` - Self-introduction
- [ ] `SkillGameModule` - Games and quizzes
- [ ] `SkillCloudProxy` - Cloud dispatcher

### Phase 8: Integration & Polish (Week 9-10)
- [x] ROS2 Management Console (`aimee_ros2_monitor`)
- [ ] System integration testing
- [ ] Performance optimization
- [ ] Error handling & recovery
- [ ] Documentation
- [ ] Deployment scripts
- [ ] Dynamic AimeeCloud capabilities based on active ROS2 nodes (post-hardware arrival)

---

## Configuration

### robot_config.yaml (Ron)

```yaml
robot:
  name: "ron"
  type: "ugv_arm"
  
  hardware:
    base:
      type: "waveshare_ugv02"
      serial_port: "/dev/ttyACM0"
      baud: 115200
    arm:
      type: "roarm_m3"
      serial_port: "/dev/ttyUSB0"
      baud: 115200
      control_mode: "json"
    cameras:
      main:
        type: "obsbot_tiny2"
        ip: "192.168.5.1"
        osc_port: 16284
        tracking_mode: "normal"
      arm_end:
        type: "ov7670"
        i2c_bus: 1

voice:
  stt_model: "/home/arduino/models/vosk-model-small-en-us-0.15"
  wake_word_model: "/home/arduino/.arduino-bricks/models/custom-ei/model.eim"
  tts_engine: "piper"
  piper_model: "/home/arduino/models/amy.onnx"

llm:
  backend: "llama.cpp"
  model: "/home/arduino/models/Qwen2.5-0.5B-Instruct-Q4_K_M.gguf"
  server_url: "http://localhost:8080"
  max_tokens: 150

memory:
  db_path: "/home/arduino/aimee-robot/config/user_memory.db"
  vector_db_path: "/home/arduino/aimee-robot/config/vector_db"
  max_history: 50

cloud:
  endpoint: "wss://aimeecloud.example.com/skills"
  api_key: "${AIMEE_API_KEY}"
  offline_queue_size: 100
```

### skills_config.yaml

```yaml
intent_routing:
  robot_control:
    intents:
      - robot_forward
      - robot_backward
      - robot_left
      - robot_right
      - robot_stop
      - arm_raise
      - arm_lower
      - arm_wave
      - gripper_open
      - gripper_close
    target: skill_robot_control
    execute_locally: true

  cloud_skills:
    # All unmatched intents are automatically routed to AimeeCloud as AimeeAgent
    target: AimeeCloud
    
emergency:
  intents:
    - emergency_stop
    - reset
  target: skill_emergency
  priority: 1
```

---

## Development Commands

```bash
# Build workspace
cd ~/aimee-robot-ws
colcon build --symlink-install

# Source environment
source install/setup.bash

# Launch core system
ros2 launch aimee_bringup core.launch.py

# Launch specific robot (auto-detected from hostname or AIMEE_ROBOT_NAME)
ros2 launch aimee_bringup robot.launch.py

# Launch robot + SLAM + Nav2
ros2 launch aimee_bringup navigation.launch.py slam:=True

# Launch with existing map (localization + navigation)
ros2 launch aimee_bringup navigation.launch.py slam:=False map:=/path/to/map.yaml

# Disable heavy software to save RAM during navigation
ros2 launch aimee_bringup navigation.launch.py slam:=True use_voice:=false use_llm:=false

# Test voice
ros2 topic pub /voice/speak std_msgs/String "data: 'Hello, I am Aimee'"

# Test motion
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"

# View topics
ros2 topic list

# View tf tree
ros2 run tf2_tools view_frames

# Check node status
ros2 node list
ros2 node info /voice_manager
```

---

## Next Steps

### Immediate Priority (Current Session)

Based on the current state with 6 ROS2 nodes complete, the next logical steps are:

#### Option A: Hardware Control Bricks (Recommended)
Create the UGV02 and RoArm-M3 control bricks to enable physical robot movement:

1. **Create `aimee_ugv02_controller` package**
   - Follow the existing node pattern (see `aimee_voice_manager` for reference)
   - Structure: `aimee_ugv02_controller/aimee_ugv02_controller/brick/ugv02_controller.py`
   - ROS2 node: Subscribes to `/cmd_vel`, publishes `/robot/state`
   - Brick: Handles serial communication with Waveshare JSON protocol

2. **Create `aimee_roarm_controller` package**
   - Similar structure to UGV02 controller
   - Subscribes to `/arm/command`
   - Supports joint and IK control modes

3. **Test Serial Communication**
   - Verify `/dev/ttyACM0` (UGV02) and `/dev/ttyUSB0` (RoArm) detection
   - Test basic movement commands

#### Option B: Cloud Bridge
Create the AimeeCloud communication brick:

1. **Create `aimee_cloud_bridge` package**
   - WebSocket client for cloud communication
   - Offline message queue
   - Skill response dispatcher

#### Option C: OBSBOT Vision Brick
Create the camera control brick:

1. **Create `aimee_vision_obsbot` package**
   - OSC protocol implementation
   - PTZ control and AI tracking
   - Face tracking coordination

#### Option D: Checkpoint System (Meta Task)
Create a checkpoint/summary system to track daily progress:

1. Create `/home/arduino/CHECKPOINT.md` template
2. Document current state at end of each session
3. Track completed tasks and blockers

---

## Checkpoint / Daily Summary Template

> **Use this template at the end of each work session to capture state:**

```markdown
# Aimee Robot - Session Checkpoint

**Date:** YYYY-MM-DD
**Session Focus:** [e.g., Hardware Control Bricks]

## Completed Today
- [ ] Task 1
- [ ] Task 2

## In Progress
- [ ] Task 3 (50% complete)

## Blockers/Issues
- None / [Description of issue]

## Next Session Priority
1. [Highest priority task]
2. [Secondary task]

## Files Modified
- `path/to/file1.py` - [brief description of changes]
- `path/to/file2.py` - [brief description of changes]

## Testing Notes
- [Any test results or observations]
```

---

## Appendix: Directory Structure

```
~/aimee-robot-ws/
├── src/
│   ├── aimee_msgs/                 # ✅ Custom ROS2 message types
│   ├── aimee_description/          # ✅ Robot URDF descriptions
│   ├── aimee_bringup/              # ✅ Launch files, configs & system integration
│   ├── aimee_wake_word_ei/         # ✅ Wake word detection
│   ├── aimee_voice_manager/        # ✅ Voice/STT management
│   ├── aimee_tts/                  # ✅ Text-to-speech
│   ├── aimee_llm_server/           # ✅ LLM action server
│   ├── aimee_intent_router/        # ✅ Intent classification
│   ├── aimee_skill_manager/        # ✅ Skill execution
│   ├── arduino/                    # ✅ Brick framework utilities
│   │
│   ├── aimee_ugv02_controller/     # ✅ Base robot control (UGV02 + Wave Rover)
│   ├── aimee_cloud_bridge/         # ✅ AimeeCloud client (ACC)
│   ├── aimee_vision_obsbot/        # ✅ Camera control
│   ├── aimee_vision_pipeline/      # ✅ Color detection & tracking
│   ├── aimee_perception/           # ✅ 3D estimation & grasp planning
│   ├── aimee_manipulation/         # ✅ Arm control & PickPlace skill
│   ├── aimee_memory/               # TODO: Context persistence
│   └── aimee_skills/               # TODO: Skill framework
│       ├── aimee_skill_interface/  # Base skill class
│       ├── skill_robot_control/    # Hardware control skill
│       ├── skill_identity/         # Identity/story skill
│       ├── skill_games/            # Game module skill
│       └── aimee_cloud/            # AimeeCloud skill dispatcher
│
├── config/
│   ├── robot_config.yaml           # Robot hardware configuration
│   ├── skills_config.yaml          # Enabled skills & routing
│   └── user_memory.db              # SQLite user context database
│
├── skills/                         # Dynamically loaded skills
│   ├── local/                      # Built-in local skills
│   └── cloud_manifests/            # Cloud skill endpoint definitions
│
├── models/                         # AI models (LLM, Vosk, etc.)
│   ├── vosk/
│   ├── llm/
│   └── vision/
│
└── docs/
    ├── architecture.md
    ├── ros2_cheatsheet.md
    └── api_reference.md
```

---

## Summary of Current State

| Component | Status | Notes |
|-----------|--------|-------|
| ROS2 Workspace | ✅ Complete | `~/aimee-robot-ws/` with colcon build |
| Message Types | ✅ Complete | `aimee_msgs` with custom types |
| Wake Word | ✅ Complete | Edge Impulse integration |
| Voice Manager | ✅ Complete | Vosk STT with partial results (migrated to standard ROS2 node, auto-recovery for arecord stalls) |
| TTS | ✅ Complete | Lemonfox primary + Kokoro/gTTS fallback (standard ROS2 node) |
| LLM Server | ✅ Complete | Action server with streaming |
| Intent Router | ✅ Complete | External JSON config, exact-phrase matching, hot-reload |
| Skill Manager | ✅ Complete | Action server for skill execution |
| Arduino Utils | ✅ Complete | `@brick` decorator framework |
| UGV02 / Wave Rover Control | ✅ Complete | JSON serial protocol, parameterized odometry, multi-base support |
| RoArm Control | ✅ Complete | Simulated (ready for real arm) |
| SLAM / Nav2 | ✅ Complete | slam_toolbox + Nav2 Humble, config-driven bringup |
| AimeeNav (Integrated) | 🔄 In Progress | Single-node navigation to reduce RAM/DDS load on UNO Q |
| Cloud Bridge | ✅ Complete | AimeeCloud MQTT client with AimeeAgent support, session clear + auto-reconnect |
| Vision/OBSBOT | ✅ Complete | SDK-based camera control |
| Vision Pipeline | ✅ Complete | Color detection & tracking |
| Perception | ✅ Complete | 3D pose estimation & grasp planning |
| Manipulation | ✅ Complete | PickPlace action server |
| Memory | ⬜ TODO | Context persistence |
| PickPlace Skill | 🔄 Ready | Waiting for voice integration |
| ROS2 Monitor | ✅ COMPLETE | Management console at :8081 with Cloud Session clear, TTS/STT tests, log viewer |

---

*Document Version: 2.9*  
*Last Updated: April 23, 2026*  
*Vision System: COMPLETE*  
*ROS2 Monitor: COMPLETE*  
*Intent Router: COMPLETE (external config + exact-phrase matching + AimeeAgent forwarding)*
*TTS: COMPLETE (Lemonfox primary + Kokoro/gTTS fallback with voice metadata support)*  
*Voice Manager: COMPLETE (migrated to standard ROS2 node with auto-recovery)*  
*SLAM / Nav2: COMPLETE (slam_toolbox + Nav2 Humble, URDF-based TF tree)*  
*AimeeNav: IN PROGRESS (integrated single-node navigation, performance-tuned for UNO Q)*  
*Base Controller: COMPLETE (multi-platform via parameterized YAML)*  
*Author: AI Assistant*  
*Status: Phase 4 Complete - Hardware Control + SLAM/Navigation*

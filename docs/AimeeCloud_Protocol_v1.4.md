# AimeeCloud Robot Communication Protocol Specification

## For: Robot Firmware / Device Agent Developers

---

## 1. Overview

This document defines the exact communication protocol between a robot (or any device) and **AimeeCloud**. The robot must implement an MQTT client that connects to the AimeeCloud broker and follows the message schemas below. The browser test client at `https://aimeecloud.com/aimee` is a reference implementation of this same protocol.

**Transport:** MQTT  
**Broker (TCP):** `aimeecloud.com:1883` (for robots with MQTT libraries)  
**Broker (WSS):** `wss://aimeecloud.com/aimeecloud-mqtt` (for browser/WebSocket-capable devices)  
**Audio Streaming (WSS):** `wss://aimeecloud.com/ws/v1` (optional, opt-in native audio)  
**Authentication:** API key via `api_key` field (or `X-API-Key` / `x-api-key`)

---

## 2. Topic Structure

All communication uses the device ID in the topic path. The device ID must be **stable and unique** per robot (e.g., `arduino-uno-q-001`).

| Topic | Direction | Purpose |
|-------|-----------|---------|
| `aimeecloud/device/{device_id}/connect` | Robot → Cloud | Session initialization and reconnection |
| `aimeecloud/device/{device_id}/in` | Robot → Cloud | Intents, game moves, system messages |
| `aimeecloud/device/{device_id}/out` | Cloud → Robot | All responses, TTS text, game state updates, snapshot requests |
| `aimeecloud/device/{device_id}/status` | Cloud → Robot | Session lifecycle events (disconnect, expiry) |
| `aimeecloud/device/{device_id}/system` | Cloud → Robot | System/operational messages (config, diagnostics, protocol updates) |
| `aimeecloud/device/{device_id}/system/in` | Robot → Cloud | Robot system reports and acknowledgments |

**Audio Streaming (Optional):**
Robots that support native audio conversation may open a secondary WebSocket to `wss://aimeecloud.com/ws/v1`. This channel carries bidirectional audio (Opus or PCM16) and control events. See **Section 10** for the full audio streaming protocol. The MQTT channel remains the command-and-control transport for all robot commands.

> **Note:** When audio streaming is active, the robot should not send `intent` messages over MQTT for voice input, and should not expect `tts` text in MQTT `response` messages. Motor, arm, gripper, snapshot, and game commands continue to use MQTT.

> **Important:** The robot must subscribe to `out` and `status` **before** publishing `connect`.

---

## 3. Session Lifecycle

### 3.1 Initial Connection

On boot or when the user activates the robot, publish a `connect` message:

```json
{
  "type": "connect",
  "device_id": "arduino-uno-q-001",
  "robot_name": "Aimee",
  "robot_personality": "Adorable Brat",
  "gemini_voice": "Fenrir",
  "user_profile": {
    "name": "Scott",
    "location": "home",
    "language": "en-US"
  },
  "capabilities": {
    "input": ["voice", "text"],
    "output": ["tts", "display", "motors", "led"]
  },
  "robot_config": {
    "has_motors": true,
    "has_arm": true,
    "has_gripper": true,
    "has_camera": true,
    "has_expressions": true,
    "expression_types": ["happy", "sad", "surprised", "greeting", "celebration"]
  },
  "session_context": {
    "ram_mb": 512,
    "storage_gb": 32,
    "cpu": "RP2040 dual-core @ 133MHz",
    "battery": "18650 Li-ion 2600mAh",
    "manufacturer": "Arduino",
    "model": "UNO R4 WiFi"
  },
  "request_session_id": null,
  "timestamp": "2026-04-14T09:46:05Z"
}
```

**Field descriptions:**
- `device_id` — Your robot's stable unique ID.
- `robot_name` *(optional)* — Display name the agent should use when referring to itself. Falls back to "Aimee" if omitted.
- `robot_personality` *(optional)* — Personality directive for the agent's tone (e.g., "Adorable Brat", "Helpful Companion"). Falls back to the cloud default if omitted.
- `gemini_voice` *(optional)* — Desired Gemini prebuilt voice name (e.g., `Fenrir`, `Puck`, `Aoede`). Falls back to the cloud default if omitted or unsupported.
- `user_profile` — Key-value map of user info. Can be empty `{}`.
- `capabilities` — What the robot can do at the input/output level. Used by the cloud to tailor responses.
- `robot_config` *(optional)* — Detailed physical capability flags. The cloud uses this to filter function declarations and tailor the agent prompt so it only asks the robot to do things it can actually do. See **Section 14** for the full schema.
- `session_context` *(optional)* — Free-form key-value map of robot/environment details the agent can reference during the session (e.g., RAM, CPU, battery, model). The cloud injects this into the agent prompt so questions like "How much RAM do you have?" can be answered from the provided specs. See **Section 14.4**.
- `request_session_id` — `null` for new sessions, or a previous `session_id` to resume.

### 3.2 Session Init Ack

The cloud responds on `out` with:

```json
{
  "type": "session_init",
  "session_id": "sess_abc123def4567890",
  "device_id": "arduino-uno-q-001",
  "status": "connected",
  "expires_in": 600,
  "ttl": 600,
  "timestamp": "2026-04-14T09:46:05Z"
}
```

**Robot action:** Store `session_id` in non-volatile or durable memory. All subsequent messages must include it.

### 3.3 Reconnection After WiFi Drop

If the robot loses connection and reconnects within the TTL (10 minutes), send the stored `session_id`:

```json
{
  "type": "connect",
  "device_id": "arduino-uno-q-001",
  "robot_name": "Aimee",
  "robot_personality": "Adorable Brat",
  "gemini_voice": "Fenrir",
  "user_profile": { "name": "Scott" },
  "capabilities": { "input": ["voice"], "output": ["tts"] },
  "robot_config": {
    "has_motors": true,
    "has_arm": false,
    "has_gripper": false,
    "has_camera": false,
    "has_expressions": true,
    "expression_types": ["happy", "sad", "greeting"]
  },
  "session_context": {
    "ram_mb": 256,
    "storage_gb": 16,
    "cpu": "ESP32-S3 @ 240MHz",
    "battery": "Li-Po 2000mAh"
  },
  "request_session_id": "sess_abc123def4567890",
  "timestamp": "2026-04-14T09:55:00Z"
}
```

**Result:** The cloud resumes the existing session with all game state and chat history intact.

### 3.4 Disconnect Notification (Optional but Recommended)

When the robot powers down or goes to sleep, publish:

```json
{
  "type": "disconnect",
  "device_id": "arduino-uno-q-001",
  "session_id": "sess_abc123def4567890",
  "timestamp": "2026-04-14T10:00:00Z"
}
```

> **MQTT LWT:** Set a Last Will and Testament on the MQTT connection to publish this message automatically if the robot drops unexpectedly.

### 3.5 Session Expiry

If the cloud publishes this on `status`, the session is gone:

```json
{
  "type": "status",
  "device_id": "arduino-uno-q-001",
  "session_id": "sess_abc123def4567890",
  "status": "expired",
  "timestamp": "2026-04-14T10:10:00Z"
}
```

**Robot action:** Discard the stored `session_id` and start a new session on next connect.

---

## 4. Message Types (Robot → Cloud)

### 4.1 Intent

The robot's intent classifier parses the user's speech (or text input) and publishes the result. If you do not have an on-device classifier, you may send the raw text and omit the `intent` field—the cloud will classify it for you.

#### Cloud-Proxy Unclassified Intent

If the robot's on-device classifier is uncertain about a request (confidence < 0.6), or if the request is completely unrecognized, the robot should **not** speak a local error message. Instead, it must forward the request to AimeeCloud for LLM-based classification and fulfillment.

Publish an intent with `intent.intent = "unclassified"`:

```json
{
  "type": "intent",
  "device_id": "arduino-uno-q-001",
  "session_id": "sess_abc123def4567890",
  "payload": "ummm what's the... weather thingy?",
  "intent": {
    "intent": "unclassified",
    "category": "cloud_proxy",
    "confidence": 0.0,
    "text": "ummm what's the... weather thingy?",
    "source": "keyword"
  },
  "timestamp": "2026-04-14T09:46:05Z"
}
```

**AimeeCloud will then:**
1. Feed the raw text to the LLM intent classifier.
2. If the LLM determines it is **valid speech** (e.g., weather, chat, game), it classifies the intent and completes the request automatically.
3. If the LLM determines it is **random noise, mumbling, or incomplete**, it responds with:
   ```json
   {
     "type": "response",
     "sub_type": "chat_response",
     "text": "I didn't catch that. Could you say it again?",
     "tts": "I didn't catch that. Could you say it again?",
     "source": "llm_classifier",
     "intent": "unclassified"
   }
   ```

**Robot rule:** Never generate local "I didn't understand" TTS when `chat_routing == 'cloud'` and the intent is unclassified or low-confidence. Always proxy to the cloud and wait for the response on `out`.

```json
{
  "type": "intent",
  "device_id": "arduino-uno-q-001",
  "session_id": "sess_abc123def4567890",
  "payload": "what's the weather?",
  "intent": {
    "intent": "weather",
    "category": "cloud_skill",
    "confidence": 0.85,
    "text": "what's the weather?",
    "source": "keyword"
  },
  "timestamp": "2026-04-14T09:46:05Z"
}
```

**Without on-device classification:**

```json
{
  "type": "intent",
  "device_id": "arduino-uno-q-001",
  "session_id": "sess_abc123def4567890",
  "payload": "what color is the sky?",
  "timestamp": "2026-04-14T09:46:05Z"
}
```

### 4.2 Game Move

When the user makes a move in an active game, publish a `game_move`. The exact structure of `move` depends on the game.

#### Chess

```json
{
  "type": "game_move",
  "device_id": "arduino-uno-q-001",
  "session_id": "sess_abc123def4567890",
  "game": "chess",
  "move": { "from": "e2", "to": "e4" },
  "timestamp": "2026-04-14T09:46:05Z"
}
```

#### Tic-Tac-Toe

```json
{
  "type": "game_move",
  "device_id": "arduino-uno-q-001",
  "session_id": "sess_abc123def4567890",
  "game": "tic-tac-toe",
  "move": { "position": 4 },
  "timestamp": "2026-04-14T09:46:05Z"
}
```

`position` is 0–8 (top-left to bottom-right). The cloud also accepts natural language via `move: { "text": "center" }`, but board indices are preferred for robots.

#### Yahtzee

```json
{
  "type": "game_move",
  "device_id": "arduino-uno-q-001",
  "session_id": "sess_abc123def4567890",
  "game": "yahtzee",
  "move": { "action": "hold", "indices": [0, 2] },
  "timestamp": "2026-04-14T09:46:05Z"
}
```

Actions: `hold`, `reroll`, `score`

For `score`:
```json
{ "action": "score", "category": "chance" }
```

### 4.3 Ping

Useful for keepalive or latency checks:

```json
{
  "type": "ping",
  "device_id": "arduino-uno-q-001",
  "session_id": "sess_abc123def4567890",
  "timestamp": "2026-04-14T09:46:05Z"
}
```

**Cloud response on `out`:**

```json
{
  "type": "response",
  "sub_type": "pong",
  "device_id": "arduino-uno-q-001",
  "session_id": "sess_abc123def4567890",
  "timestamp": "2026-04-14T09:46:05Z"
}
```

---

## 4.4 System Messages (Robot → Cloud)

The robot can publish operational/status messages to `system/in`.

### Status Report
```json
{
  "type": "status_report",
  "device_id": "arduino-uno-q-001",
  "device_status": {
    "battery_percent": 67,
    "wifi_rssi": -42,
    "firmware_version": "1.2.3",
    "uptime_seconds": 1840
  },
  "timestamp": "2026-04-14T09:46:05Z"
}
```

### Acknowledgment
```json
{
  "type": "ack",
  "device_id": "arduino-uno-q-001",
  "ack_for": "protocol_update",
  "msg_id": "proto-v2-20260414",
  "timestamp": "2026-04-14T09:46:05Z"
}
```

### Diagnostics Response
```json
{
  "type": "diagnostics_response",
  "device_id": "arduino-uno-q-001",
  "diagnostics": {
    "memory_free_mb": 124,
    "cpu_temp_c": 42,
    "last_error": null
  },
  "timestamp": "2026-04-14T09:46:05Z"
}
```

### 4.5 Snapshot Response (Robot → Cloud)

Returned by the robot after completing a camera snapshot request. Published on `in`.

```json
{
  "type": "snapshot_response",
  "session_id": "sess_abc123def4567890",
  "device_id": "arduino-uno-q-001",
  "request_id": "snap_7f8a9b",
  "success": true,
  "message": "Snapshot captured: 3840x2160, 976420 bytes",
  "format": "jpeg",
  "image_base64": "/9j/4AAQSkZJRgABAQAAAQABAAD...",
  "timestamp": "2026-04-14T09:46:05Z"
}
```

| Field | Type | Description |
|-------|------|-------------|
| `type` | string | `"snapshot_response"` |
| `session_id` | string | Echo back the active session ID |
| `device_id` | string | Robot device ID |
| `request_id` | string | Echo back the request's `request_id` |
| `success` | bool | `true` if JPEG captured successfully |
| `message` | string | Status or error details |
| `format` | string | `"jpeg"` (omitted on failure) |
| `image_base64` | string | Base64-encoded JPEG bytes (omitted on failure) |
| `timestamp` | string | ISO 8601 timestamp |

---

## 5. Response Types (Cloud → Robot)

Most cloud-to-robot traffic arrives on the `out` topic with `type: "response"` and a `sub_type` discriminator. The exception is `snapshot_request` (see 5.5), which uses its own top-level `type`.

### 5.1 Chat Response (`sub_type: "chat_response"`)

Returned for chat, help, status, and general knowledge questions.

```json
{
  "type": "response",
  "sub_type": "chat_response",
  "session_id": "sess_abc123def4567890",
  "device_id": "arduino-uno-q-001",
  "intent": "chat",
  "text": "The sky is blue because of the way sunlight scatters in the atmosphere.",
  "tts": "The sky is blue because of the way sunlight scatters in the atmosphere.",
  "source": "llm",
  "context": {
    "active_context": null,
    "context_stack": []
  },
  "timestamp": "2026-04-14T09:46:05Z"
}
```

**Robot action:** Speak the `tts` string. Optionally display the `text` string if the robot has a screen.

### 5.2 Game Update (`sub_type: "game_update"`)

Returned when a game starts or when a move is processed.

```json
{
  "type": "response",
  "sub_type": "game_update",
  "session_id": "sess_abc123def4567890",
  "device_id": "arduino-uno-q-001",
  "intent": "game",
  "game": "tic-tac-toe",
  "state": {
    "board": ["O", "", "", "", "X", "", "", "", ""],
    "current_turn": "X",
    "game_status": "playing"
  },
  "text": "I placed O in the top left.\n O |   |  \n-----------\n   | X |  \n-----------\n   |   |  \nYour turn!",
  "tts": "I placed O in the top left. Your turn!",
  "context": {
    "active_context": "Game: tic-tac-toe",
    "context_stack": []
  },
  "timestamp": "2026-04-14T09:46:05Z"
}
```

**Game state fields:**
- `state.board` — Array of 9 strings for tic-tac-toe
- `state.current_turn` — `"X"` or `"O"`
- `state.game_status` — `"playing"`, `"X_won"`, `"O_won"`, or `"draw"`

**Robot action:**
1. Update your local game representation from `state`.
2. If `game_status` is `X_won`, `O_won`, or `draw`, announce the result and end the game.
3. Otherwise, speak `tts` and wait for the user's next move.

### 5.3 Robot Command (`sub_type: "robot_command"`)

Returned for robot movement, arm, and gripper intents.

```json
{
  "type": "response",
  "sub_type": "robot_command",
  "session_id": "sess_abc123def4567890",
  "device_id": "arduino-uno-q-001",
  "intent": "robot_forward",
  "text": "Moving forward",
  "tts": "Okay, moving forward",
  "command": {
    "motor": "forward",
    "duration_ms": 1000
  },
  "timestamp": "2026-04-14T09:46:05Z"
}
```

**Command payloads by intent:**

| Intent | Command Object |
|--------|----------------|
| `robot_forward` | `{ "motor": "forward", "duration_ms": 1000 }` |
| `robot_backward` | `{ "motor": "backward", "duration_ms": 1000 }` |
| `robot_stop` | `{ "motor": "stop", "duration_ms": 0 }` |
| `robot_left` | `{ "motor": "left", "duration_ms": 500 }` |
| `robot_right` | `{ "motor": "right", "duration_ms": 500 }` |
| `robot_wave` | `{ "motor": "wave", "duration_ms": 1000 }` |
| `arm_raise` | `{ "arm": "raise" }` |
| `arm_lower` | `{ "arm": "lower" }` |
| `gripper_open` | `{ "gripper": "open" }` |
| `gripper_close` | `{ "gripper": "close" }` |

**Robot action:** Execute the hardware command, then speak `tts`.

### 5.4 Error Response (`sub_type: "error"`)

Returned when something goes wrong.

```json
{
  "type": "response",
  "sub_type": "error",
  "session_id": "sess_abc123def4567890",
  "device_id": "arduino-uno-q-001",
  "text": "I didn't understand that move.",
  "tts": "I didn't understand that move.",
  "error": "INVALID_GAME_MOVE",
  "timestamp": "2026-04-14T09:46:05Z"
}
```

**Common error codes:**
- `SESSION_NOT_FOUND` — The session ID is invalid or expired. Start a new session.
- `NO_ACTIVE_GAME` — A `game_move` was sent but no game is in progress.
- `INVALID_GAME_MOVE` — The move format was wrong or illegal.
- `GAME_START_ERROR` — Failed to start the requested game.
- `SNAPSHOT_FAILED` — Camera capture failed or device was busy.

### 5.5 Snapshot Request (Cloud → Robot)

AimeeCloud requests a high-resolution camera snapshot for vision tasks. Sent on `out` with `type: "snapshot_request"`.

```json
{
  "type": "snapshot_request",
  "session_id": "sess_abc123def4567890",
  "device_id": "arduino-uno-q-001",
  "request_id": "snap_7f8a9b",
  "resolution": "4k",
  "quality": 95,
  "timestamp": "2026-04-14T09:46:05Z"
}
```

| Field | Type | Description |
|-------|------|-------------|
| `type` | string | `"snapshot_request"` |
| `session_id` | string | Active session ID |
| `device_id` | string | Robot device ID |
| `request_id` | string | Unique snapshot request ID |
| `resolution` | string | `"4k"`, `"1080p"`, or `"720p"` (default `"1080p"`) |
| `quality` | int | JPEG quality `1–100` (default `95`) |
| `timestamp` | string | ISO 8601 timestamp |

**Robot action:** See Section 7 for the full capture workflow.

---

## 6. System Messages (Cloud → Robot)

AimeeCloud can push operational messages to the robot on the `system` topic. The robot should subscribe to this topic and handle each `type` accordingly.

### Protocol Update
```json
{
  "type": "protocol_update",
  "device_id": "arduino-uno-q-001",
  "msg_id": "proto-v2-20260414",
  "version": "2.0",
  "url": "https://aimeecloud.com/protocols/robot-protocol-v2.pdf",
  "effective_at": "2026-04-14T12:00:00Z",
  "timestamp": "2026-04-14T09:46:05Z"
}
```

**Robot action:** Acknowledge with an `ack` message. Apply the new protocol at `effective_at` or on next boot.

### Config Update
```json
{
  "type": "config_update",
  "device_id": "arduino-uno-q-001",
  "msg_id": "cfg-001",
  "config": {
    "tts_volume": 0.8,
    "idle_timeout_seconds": 30,
    "chat_routing": "cloud"
  },
  "timestamp": "2026-04-14T09:46:05Z"
}
```

### Diagnostics Request
```json
{
  "type": "diagnostics_request",
  "device_id": "arduino-uno-q-001",
  "msg_id": "diag-042",
  "timestamp": "2026-04-14T09:46:05Z"
}
```

**Robot action:** Publish a `diagnostics_response` to `system/in`.

### Restart Request
```json
{
  "type": "restart",
  "device_id": "arduino-uno-q-001",
  "msg_id": "rst-007",
  "reason": "firmware_update",
  "timestamp": "2026-04-14T09:46:05Z"
}
```

### Firmware Available
```json
{
  "type": "firmware_available",
  "device_id": "arduino-uno-q-001",
  "msg_id": "fw-3.0.1",
  "version": "3.0.1",
  "download_url": "https://ota.aimeecloud.com/firmware/3.0.1.bin",
  "timestamp": "2026-04-14T09:46:05Z"
}
```

---

## 7. Snapshot Service Protocol

AimeeCloud can request a camera snapshot for vision tasks (game-state analysis, object identification, etc.). The robot uses its ROS2 `/camera/capture_snapshot` service and returns the image over MQTT.

### 7.1 Required Robot Workflow

When the robot receives `snapshot_request` on `out`:

1. **Capture** a photo from the requested `camera`.
2. **Build** a `snapshot_response` JSON:
   - On success: Base64-encode the JPEG into `image_base64`
   - On failure: omit `image_base64` and `format`, set `success: false`
3. **Publish** `snapshot_response` to `aimeecloud/device/{id}/in` within 8 seconds.

> **Note:** The legacy workflow required stopping `usb_camera`, calling `/camera/capture_snapshot`, and restarting. The current recommended approach uses an `obsbot_node` ring buffer to avoid V4L2 device contention.

### 7.2 Timing & Timeout

| Resolution | Typical Capture Time |
|------------|---------------------|
| 720p | ~1 second |
| 1080p | ~1–2 seconds |
| 4k | ~2–3 seconds |

**Legacy pipeline:** AimeeCloud uses a 15-second stall-detection window for inline `snapshot` commands.

**Audio pipeline:** AimeeCloud waits up to **8 seconds** for a `snapshot_response` after a `take_snapshot` function call. If the robot does not respond in time, the request is discarded and the LLM receives an error result.

### 7.3 Error Scenarios

| Scenario | Robot Response |
|----------|---------------|
| Camera busy | `success: false`, message: "Camera device busy" |
| Camera service unavailable | `success: false`, message: "Camera service unavailable" |
| Timeout (>8s) | Cloud discards request |
| Image > 2 MB Base64 | Downscale and retry, or return `success: false` |

### 7.4 ROS2 Service Details

- **Service:** `/camera/capture_snapshot`
- **Type:** `aimee_msgs/srv/CaptureSnapshot`
- **Node:** `obsbot_camera` (`aimee_vision_obsbot` package)
- **Device:** `/dev/video2` (OBSBOT Tiny 2 Lite)

---

## 8. Context Management & Interruptions

AimeeCloud handles **mid-game interruptions** automatically.

### Example Flow

1. User says: `"play tic tac toe"` → Cloud starts game, `active_context = "Game: tic-tac-toe"`
2. User says: `"what's the weather?"` → Cloud responds with weather **and** appends a resume hint to the TTS:
   ```json
   {
     "tts": "It's sunny and 72 degrees outside. Back to Tic-Tac-Toe, your move!",
     "context": {
       "active_context": "Game: tic-tac-toe",
       "was_interrupted": true,
       "previous_context": "Game: tic-tac-toe",
       "return_to": "tic-tac-toe"
     }
   }
   ```
3. User makes a game move → The game continues from its previous state.

**Robot action:** You do not need to implement interruption logic. Simply speak the `tts` string and continue handling game moves normally.

---

## 9. On-Device Intent Classification (Optional)

If your robot has an on-device intent classifier, use these intent names so the cloud routes correctly:

| Intent | When to Use |
|--------|-------------|
| `chat` | General conversation, questions starting with who/what/when/where/how/why |
| `weather` | Weather, temperature, forecast requests |
| `news` | News, headlines |
| `story` | Storytelling, bedtime stories |
| `game` | Starting a game (tic-tac-toe, chess, yahtzee, candyland) |
| `help` | Help requests |
| `status` | "How are you?" |
| `robot_forward` | Move forward |
| `robot_backward` | Move backward |
| `robot_stop` | Stop |
| `robot_left` | Turn left |
| `robot_right` | Turn right |
| `robot_wave` | Wave / dance |
| `arm_raise` | Raise arm |
| `arm_lower` | Lower arm |
| `gripper_open` | Open gripper |
| `gripper_close` | Close gripper |

If you omit the `intent` field, the cloud will classify the `payload` text for you.

If confidence is below 0.6, or the intent is `"unclassified"`, AimeeCloud automatically escalates to the LLM classifier. The robot does not need any special handling for this case beyond waiting for the response on `out`.

---

## 10. Audio Streaming Protocol (Optional)

Robots that support real-time conversational audio can use a secondary WebSocket connection for native bidirectional audio streaming. When enabled, voice input and output travel over this WebSocket instead of the MQTT `intent` / `tts` text flow. All motor, arm, gripper, snapshot, and expression commands still use the MQTT topics above.

**Endpoint:** `wss://aimeecloud.com/ws/v1`

**Prerequisites:** The robot must have a valid API key. An MQTT session is not strictly required before opening the audio WebSocket, but the same `device_id` and `session_id` are used.

### 10.1 Handshake

The first message on the WebSocket must be a JSON `session_start`:

```json
{
  "type": "session_start",
  "api_key": "ak_live_xxxxxxxx",
  "device_id": "arduino-uno-q-001",
  "session_id": "sess_abc123def4567890",
  "robot_name": "Aimee",
  "robot_personality": "Adorable Brat",
  "gemini_voice": "Fenrir",
  "provider": "gemini",
  "capabilities": {
    "audio_in": { "codec": "pcm16", "sample_rate": 16000 },
    "audio_out": { "codec": "pcm16", "sample_rate": 24000 }
  },
  "robot_config": {
    "has_motors": true,
    "has_arm": true,
    "has_gripper": true,
    "has_camera": true,
    "has_expressions": true,
    "expression_types": ["happy", "sad", "surprised", "greeting", "celebration"]
  },
  "session_context": {
    "ram_mb": 512,
    "storage_gb": 32,
    "cpu": "RP2040 dual-core @ 133MHz",
    "battery": "18650 Li-ion 2600mAh"
  }
}
```

| Field | Required | Description |
|-------|----------|-------------|
| `type` | yes | Must be `"session_start"` |
| `api_key` | yes | Your AimeeCloud API key |
| `device_id` | yes | Stable unique device ID |
| `session_id` | no | Existing MQTT session ID to resume, or omit/null for a new audio session |
| `robot_name` | no | Display name the agent should use when referring to itself. Default: `"Aimee"` |
| `robot_personality` | no | Personality directive for the agent's tone. Default: cloud-configured persona |
| `gemini_voice` | no | Desired Gemini prebuilt voice name. Default: cloud-configured voice |
| `provider` | no | `"gemini"` (default) or `"openai"` |
| `capabilities.audio_in` | no | Robot microphone format. Default: `{"codec":"pcm16","sample_rate":16000}` |
| `capabilities.audio_out` | no | Robot speaker format. Default: `{"codec":"pcm16","sample_rate":24000}` |
| `robot_config` | no | Detailed physical capability flags. See **Section 14** for the full schema. |
| `session_context` | no | Free-form key-value map of robot/environment details. See **Section 14.4**. |

**Codec options:** `pcm16` or `opus`. Opus requires build tools on the server; if unavailable, the robot should fall back to PCM16 JSON mode.

**Server response (`session_ready`):**
```json
{
  "type": "session_ready",
  "session_id": "sess_audio_abc123",
  "status": "connected",
  "server_info": {
    "model": "gemini-2.5-flash-native-audio",
    "supported_codecs": ["opus", "pcm16"],
    "provider": "gemini"
  },
  "timestamp": "2026-04-14T09:46:05Z"
}
```

If the handshake fails, the server sends an `error` message and closes the WebSocket.

### 10.2 Robot → Cloud Messages

After handshake, the robot sends:

**Audio data** — Either:
- **Binary Opus frames** (if `audio_in.codec === "opus"`). Send raw Opus-encoded binary WebSocket messages.
- **JSON PCM16 chunks:**
  ```json
  {
    "type": "audio_chunk",
    "format": "pcm16",
    "sample_rate": 16000,
    "data": "<base64-encoded PCM16 little-endian>"
  }
  ```

**Voice Activity Detection (optional):**
```json
{
  "type": "vad_event",
  "event": "start"
}
```
`event` can be `"start"` or `"end"`.

**User interrupt (optional):**
```json
{
  "type": "interrupt"
}
```
Tells the cloud to stop speaking immediately.

### 10.3 Cloud → Robot Messages

**Audio output:**
- **Binary Opus frames** (if `audio_out.codec === "opus"`).
- **JSON PCM16 chunks:**
  ```json
  {
    "type": "audio_chunk",
    "seq": 0,
    "format": "pcm16",
    "sample_rate": 24000,
    "data": "<base64-encoded PCM16 little-endian>"
  }
  ```
  `seq` increments per chunk. The robot should play these in order.

**Transcript (optional):**
```json
{
  "type": "text_delta",
  "text": "It's sunny and 72 degrees."
}
```

**Function call start:**
```json
{
  "type": "function_call_start",
  "call_id": "call_abc123",
  "name": "take_snapshot"
}
```
The robot may show a visual indicator that a tool is being invoked.

**Function call end:**
```json
{
  "type": "function_call_end",
  "call_id": "call_abc123",
  "duration_ms": 842
}
```
Or on error:
```json
{
  "type": "function_call_end",
  "call_id": "call_abc123",
  "duration_ms": 120,
  "error": "Camera service unavailable"
}
```

**Interrupted:**
```json
{
  "type": "interrupted"
}
```
Sent when the cloud's speech was cut off (e.g., by a user interrupt or a new turn).

**Error:**
```json
{
  "type": "error",
  "code": "TIER_LIMIT_EXCEEDED",
  "message": "Max concurrent audio streams (1) reached.",
  "recoverable": true
}
```

### 10.4 Audio Streaming Error Codes

| Code | Meaning | Recoverable |
|------|---------|-------------|
| `INVALID_PARAMS` | Missing `api_key` or `device_id` | No |
| `INVALID_API_KEY` | API key not recognized | No |
| `TIER_LIMIT_EXCEEDED` | Concurrent stream quota reached (free tier = 1) | Yes |
| `EXPECTED_SESSION_START` | First message was not `session_start` | No |
| `PROVIDER_CONNECT_FAILED` | Could not connect to Gemini/OpenAI realtime | No |
| `PROVIDER_ERROR` | Provider-side error during streaming | Yes |
| `MESSAGE_PARSE_ERROR` | Malformed JSON or binary data | Yes |

### 10.5 Relationship to MQTT

When audio streaming is active:
- **Do not use** MQTT `intent` messages for voice input (audio is the input).
- **Do not expect** `tts` fields in MQTT `response` messages for voice output (audio comes over WebSocket).
- **Continue using** MQTT for: `robot_command`, `snapshot_request`, `game_update`, `system` messages, and `status` events.
- The robot should keep its MQTT connection alive even while streaming audio.

### 10.6 Minimal Audio Streaming Example

```python
import websocket, base64, json

ws = websocket.create_connection("wss://aimeecloud.com/ws/v1")

# 1. Handshake
ws.send(json.dumps({
    "type": "session_start",
    "api_key": api_key,
    "device_id": device_id,
    "session_id": session_id,
    "capabilities": {
        "audio_in": {"codec": "pcm16", "sample_rate": 16000},
        "audio_out": {"codec": "pcm16", "sample_rate": 24000}
    }
}))

# 2. Wait for session_ready
msg = json.loads(ws.recv())
assert msg["type"] == "session_ready"

# 3. Stream audio from microphone
while True:
    pcm16_bytes = microphone.read()  # 20-40ms of PCM16 @ 16kHz
    ws.send(json.dumps({
        "type": "audio_chunk",
        "format": "pcm16",
        "sample_rate": 16000,
        "data": base64.b64encode(pcm16_bytes).decode()
    }))

    # 4. Receive and play response audio (non-blocking poll)
    while ws.recv_available():  # pseudocode — use your lib's poll method
        resp = ws.recv()
        if isinstance(resp, bytes):
            speaker.play_opus(resp)
        else:
            msg = json.loads(resp)
            if msg["type"] == "audio_chunk":
                speaker.play_pcm16(base64.b64decode(msg["data"]))
            elif msg["type"] == "interrupted":
                speaker.stop()
```

## 11. State Diagram

```
[Robot Boot]
    |
    v
[Subscribe to out + status]
    |
    v
[Publish connect] ----> [Cloud returns session_init]
    |
    v
[Idle] <----> [Publish intent / game_move / ping]
    |                |
    |                v
    |         [Cloud returns response on out]
    |                |
    |         [Speak TTS / Execute command / Render game]
    |
[WiFi drop]
    |
    v
[Reconnect within 10 min] ----> [Publish connect with request_session_id]
    |                                  |
    v                                  v
[Session resumes] <------------- [Same session_id, state intact]
    |
[No reconnect within 10 min]
    |
    v
[Session expired] ----> [Start new session]
```

---

## 12. Quick Reference: Minimal Robot Implementation

### Step 1: Connect and Subscribe
```python
# Pseudocode
mqtt.connect("aimeecloud.com", 1883)
mqtt.subscribe(f"aimeecloud/device/{device_id}/out")
mqtt.subscribe(f"aimeecloud/device/{device_id}/status")
```

### Step 2: Request Session
```python
mqtt.publish(
    f"aimeecloud/device/{device_id}/connect",
    json.dumps({
        "type": "connect",
        "device_id": device_id,
        "robot_name": "Aimee",
        "robot_personality": "Adorable Brat",
        "gemini_voice": "Fenrir",
        "user_profile": { "name": user_name },
        "capabilities": { "input": ["voice"], "output": ["tts"] },
        "robot_config": {
            "has_motors": true,
            "has_arm": false,
            "has_gripper": false,
            "has_camera": true,
            "has_expressions": true,
            "expression_types": ["happy", "sad", "greeting"]
        },
        "session_context": {
            "ram_mb": 256,
            "storage_gb": 16,
            "cpu": "ESP32-S3 @ 240MHz",
            "battery": "Li-Po 2000mAh"
        },
        "request_session_id": stored_session_id
    })
)
```

### Step 3: Handle Incoming Messages
```python
def on_message(topic, payload):
    data = json.loads(payload)
    
    if data["type"] == "session_init":
        store_session_id(data["session_id"])
    
    elif data["type"] == "response":
        if data["sub_type"] == "chat_response":
            speak(data["tts"])
        elif data["sub_type"] == "game_update":
            update_game_state(data["state"])
            speak(data["tts"])
        elif data["sub_type"] == "robot_command":
            execute_command(data["command"])
            speak(data["tts"])
        elif data["sub_type"] == "error":
            speak(data["tts"])
    
    elif data["type"] == "status" and data["status"] == "expired":
        clear_session_id()
```

### Step 4: Send User Speech
```python
mqtt.publish(
    f"aimeecloud/device/{device_id}/in",
    json.dumps({
        "type": "intent",
        "session_id": get_session_id(),
        "payload": transcribed_user_text
    })
)
```

### Step 5: Send Game Move
```python
mqtt.publish(
    f"aimeecloud/device/{device_id}/in",
    json.dumps({
        "type": "game_move",
        "session_id": get_session_id(),
        "game": "tic-tac-toe",
        "move": { "position": 4 }
    })
)
```

### Step 6: Handle Snapshot Request
```python
elif data["type"] == "snapshot_request":
    # 1. stop usb_camera
    # 2. call /camera/capture_snapshot
    # 3. base64-encode image.data
    mqtt.publish(
        f"aimeecloud/device/{device_id}/in",
        json.dumps({
            "type": "snapshot_response",
            "session_id": get_session_id(),
            "request_id": data["request_id"],
            "success": True,
            "message": "Snapshot captured",
            "format": "jpeg",
            "image_base64": encoded_image
        })
    )
    # 4. restart usb_camera
```

---

## 13. Testing Against the Reference Client

The browser at `https://aimeecloud.com/aimee` sends and receives **identical** messages. You can:
- Open the browser, start a session, and capture the MQTT frames.
- Replay those same JSON payloads from your robot.
- Expect identical responses from the cloud.

If a message works in the browser but fails from the robot, the issue is on the robot side (topic formatting, JSON encoding, missing `session_id`, etc.).

---

## 14. Capabilities & Robot Configuration

### 14.1 Capabilities Object

Tell the cloud what your robot can do at the input/output level so it can format responses appropriately:

```json
{
  "input": ["voice", "text", "button"],
  "output": ["tts", "display", "led", "motors", "arm"]
}
```

- `input` — How the user interacts with the robot.
- `output` — How the robot can present information back.

This object is still used for game engine formatting and high-level routing.

### 14.2 Robot Configuration Object

Use `robot_config` to describe the robot's physical hardware. The cloud uses these flags to build the agent prompt and to decide which functions to expose to the audio-native LLM. If a capability is `false`, the agent will not try to use it.

```json
{
  "has_motors": true,
  "has_arm": true,
  "has_gripper": true,
  "has_camera": true,
  "has_expressions": true,
  "expression_types": ["happy", "sad", "surprised", "greeting", "celebration"]
}
```

| Field | Type | Description |
|-------|------|-------------|
| `has_motors` | bool | Robot has a mobile base / can execute `motor_command`. |
| `has_arm` | bool | Robot has an arm / can execute `arm_command`. |
| `has_gripper` | bool | Robot has a gripper / can execute `gripper_command` (pick/place). |
| `has_camera` | bool | Robot has a camera / can execute `take_snapshot`. |
| `has_expressions` | bool | Robot can display emotional expressions via `set_expression`. |
| `expression_types` | string[] | Supported expression names. Defaults to `happy`, `sad`, `surprised`, `greeting`, `celebration` if omitted. |

### 14.3 Agent Identity Fields

These optional fields let the robot customize the agent's identity and voice at session start:

| Field | Type | Description |
|-------|------|-------------|
| `robot_name` | string | Display name the agent uses when referring to itself. Default: `"Aimee"`. |
| `robot_personality` | string | Short personality directive (e.g., `"Adorable Brat"`, `"Helpful Companion"`). Default: cloud-configured persona. |
| `gemini_voice` | string | Gemini prebuilt voice name (e.g., `Fenrir`, `Puck`, `Aoede`). Default: cloud-configured voice. |

> **Note:** If the cloud does not yet support dynamic prompt generation from these fields, it will store them in the session and fall back to the current defaults. Robot firmware should send them now so the cloud can start honoring them as soon as the server-side support lands.

### 14.4 Session Context Object

Use `session_context` to attach free-form robot specifications, environment details, or any other facts the agent should know during the session. The cloud injects this context into the agent prompt, so the agent can answer questions like "How much RAM do you have?" or "What's your battery?" directly from the provided data.

```json
{
  "ram_mb": 512,
  "storage_gb": 32,
  "cpu": "RP2040 dual-core @ 133MHz",
  "battery": "18650 Li-ion 2600mAh",
  "manufacturer": "Arduino",
  "model": "UNO R4 WiFi",
  "firmware_version": "1.2.3",
  "wifi_band": "2.4GHz"
}
```

- Keys and values are **arbitrary strings, numbers, or booleans**. Choose field names that are self-explanatory.
- The agent will see the context as plain text in its system prompt, so prefer human-readable keys (e.g., `ram_mb`, `battery`) over opaque abbreviations.
- Keep the object reasonably small (recommended under 2 KB) to avoid bloating the prompt.
- This context is scoped to the session. Send updated values on reconnection if specs change at runtime.

---

## 15. Version

**Protocol Version:** 1.4  
**Last Updated:** 2026-06-11  
**Contact:** Refer to AimeeCloud infrastructure team for broker URL changes or auth updates.

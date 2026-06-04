#!/usr/bin/env python3
"""
AIMEE ROS2 Monitor - Lightweight Dashboard

A simple web-based dashboard for monitoring ROS2:
- rqt_console-like log viewer (captures /rosout)
- Node status widgets (visual cards for each node)
- Topic monitoring with live data flow

Usage:
    ros2 run aimee_ros2_monitor monitor_node
    
Then open browser to: http://localhost:8081
"""

import json
import logging
import math
import os
import requests
import subprocess
import threading
import time
from collections import deque
from datetime import datetime
from typing import Dict, List, Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import Log
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist

try:
    from aimee_msgs.msg import TrackingCommand, Transcription
    from aimee_msgs.srv import CaptureSnapshot
    AIMEE_MSGS_AVAILABLE = True
except ImportError:
    AIMEE_MSGS_AVAILABLE = False
    CaptureSnapshot = None
    Transcription = None

try:
    from rosidl_runtime_py.utilities import get_message
    from rosidl_runtime_py import message_to_ordereddict
    ROSIDL_PY_AVAILABLE = True
except ImportError:
    ROSIDL_PY_AVAILABLE = False

from flask import Flask, render_template, jsonify, request, Response, make_response

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Get template directory from package share (works in both build and install)
try:
    from ament_index_python.packages import get_package_share_directory
    template_dir = os.path.join(get_package_share_directory('aimee_ros2_monitor'), 'templates')
except Exception:
    template_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'templates')
app = Flask(__name__, template_folder=template_dir)

# Global reference to ROS2 node (for native pub from Flask threads)
_ros_node = None

# ==================== Global State ====================

# Log storage (rqt_console-like)
MAX_LOGS = 1000
_logs_buffer = deque(maxlen=MAX_LOGS)
_log_stats = {
    'debug': 0,
    'info': 0,
    'warn': 0,
    'error': 0,
    'fatal': 0
}

# Node tracking
_nodes_cache = {}
_nodes_last_update = 0

# Topic tracking
_topics_cache = {}

# Pipeline state (voice / intent / tts / cloud)
_pipeline_state = {
    'voice': {'text': '', 'timestamp': 0, 'is_partial': False},
    'intent': {'type': '', 'action': '', 'confidence': 0.0, 'timestamp': 0},
    'tts': {'text': '', 'timestamp': 0},
    'cloud': {'connected': False, 'session_id': '', 'timestamp': 0},
}

# System status
_system_status = {
    'ros2_running': False,
    'daemon_running': False,
    'last_nodes_scan': 0,
    'last_topics_scan': 0,
}

# ==================== Node Control ====================
# Registry of managed nodes and their processes
_managed_nodes: Dict[str, Dict] = {}
_core_process = None

# Node definitions - available nodes that can be started/stopped
NODE_DEFINITIONS = {
    'obsbot_camera': {
        'name': 'OBSBOT Camera',
        'ros_name': '/obsbot_camera',
        'package': 'aimee_vision_obsbot',
        'executable': 'obsbot_node',
        'args': [],
        'category': 'vision',
        'icon': '📷'
    },
    'usb_camera': {
        'name': 'USB Camera',
        'ros_name': '/camera/usb_cam',
        'package': 'usb_cam',
        'executable': 'usb_cam_node_exe',
        'args': [
            '--ros-args',
            '-p', 'video_device:=/dev/video2',
            '-p', 'image_width:=640',
            '-p', 'image_height:=480',
            '-p', 'pixel_format:=raw_mjpeg',
            '-p', 'io_method:=mmap',
            '-p', 'camera_name:=usb_camera',
            '-r', '__ns:=/camera'
        ],
        'category': 'vision',
        'icon': '🎥'
    },
    'wake_word': {
        'name': 'Wake Word',
        'ros_name': '/wake_word_ei',
        'package': 'aimee_wake_word_ei',
        'executable': 'wake_word_node',
        'args': [],
        'category': 'audio',
        'icon': '🎯'
    },
    'voice_manager': {
        'name': 'Voice Manager',
        'ros_name': '/voice_manager',
        'package': 'aimee_voice_manager',
        'executable': 'voice_manager_node',
        'args': [
            '--ros-args',
            '-p', 'audio_device:=default',
            '-p', 'model_path:=/home/arduino/vosk-models/vosk-model-small-en-us-0.15',
            '-p', 'energy_threshold:=45.0',
            '-p', 'min_command_length:=0.3',
            '-p', 'whisper_enabled:=true',
            '-p', f'whisper_api_key:={os.getenv("LEMONFOX_API_KEY", "")}',
            '-p', 'whisper_api_base_url:=https://api.lemonfox.ai/v1/audio/transcriptions',
        ],
        'category': 'audio',
        'icon': '🎤'
    },
    'intent_router': {
        'name': 'Intent Router',
        'ros_name': '/intent_router',
        'package': 'aimee_intent_router',
        'executable': 'intent_router_node',
        'args': [],
        'category': 'ai',
        'icon': '🧠'
    },
    'skill_manager': {
        'name': 'Skill Manager',
        'ros_name': '/skill_manager',
        'package': 'aimee_skill_manager',
        'executable': 'skill_manager_node',
        'args': [],
        'category': 'skills',
        'icon': '🦾'
    },
    'tts': {
        'name': 'Text-to-Speech',
        'ros_name': '/tts',
        'package': 'aimee_tts',
        'executable': 'tts_node',
        'args': [
            '--ros-args',
            '-p', 'default_engine:=lemonfox',
            '-p', 'fallback_engine:=gtts',
            '-p', 'auto_fallback:=true',
            '-p', 'default_voice:=sarah',
            '-p', f'lemonfox_api_key:={os.getenv("LEMONFOX_API_KEY", "")}',
            '-p', 'lemonfox_api_base_url:=https://api.lemonfox.ai/v1',
            '-p', 'volume:=1.0',
        ],
        'category': 'audio',
        'icon': '🔊'
    },
    'llm_server': {
        'name': 'LLM Server',
        'ros_name': '/llm_server',
        'package': 'aimee_llm_server',
        'executable': 'llm_server_node',
        'args': [],
        'category': 'ai',
        'icon': '🧠'
    },
    'aimee_cloud_client': {
        'name': 'AimeeCloud Client (ACC)',
        'ros_name': '/aimee_cloud_client',
        'package': 'aimee_cloud_bridge',
        'executable': 'cloud_bridge_node',
        'args': [
            '--ros-args',
            '-r', '__node:=aimee_cloud_client',
            '-p', 'snapshot_resolution:=640x480',
            '-p', 'snapshot_quality:=85',
        ],
        'category': 'ai',
        'icon': '☁️'
    },
    'lerobot_bridge': {
        'name': 'LeRobot Bridge',
        'ros_name': '/lerobot_bridge',
        'package': 'aimee_lerobot_bridge',
        'executable': 'lerobot_bridge_node',
        'args': [],
        'category': 'skills',
        'icon': '🤖'
    },
    'ugv02_controller': {
        'name': 'UGV02 Controller',
        'ros_name': '/ugv02_controller',
        'package': 'aimee_ugv02_controller',
        'executable': 'ugv02_controller_node',
        'args': [],
        'category': 'skills',
        'icon': '🚗'
    },
    'ros2_monitor': {
        'name': 'ROS2 Monitor',
        'ros_name': '/ros2_monitor',
        'package': 'aimee_ros2_monitor',
        'executable': 'monitor_node',
        'args': [],
        'category': 'tools',
        'icon': '🔍'
    },
}

# ==================== Flask Routes ====================

@app.route('/')
def index():
    """Main dashboard page."""
    resp = make_response(render_template('index.html'))
    resp.headers['Cache-Control'] = 'no-store, no-cache, must-revalidate, max-age=0'
    resp.headers['Pragma'] = 'no-cache'
    resp.headers['Expires'] = '0'
    resp.headers['Vary'] = '*'
    resp.headers['Last-Modified'] = datetime.now().strftime('%a, %d %b %Y %H:%M:%S GMT')
    return resp


@app.route('/api/logs')
def get_logs():
    """Get recent logs (rqt_console style)."""
    level_filter = request.args.get('level', 'all')
    node_filter = request.args.get('node', 'all')
    search = request.args.get('search', '').lower()
    limit = int(request.args.get('limit', 100))
    
    filtered_logs = []
    for log in reversed(_logs_buffer):  # Newest first
        if level_filter != 'all' and log['level'] != level_filter:
            continue
        if node_filter != 'all' and node_filter not in log['node']:
            continue
        if search and search not in log['msg'].lower():
            continue
        
        filtered_logs.append(log)
        if len(filtered_logs) >= limit:
            break
    
    return jsonify({
        'logs': filtered_logs,
        'total': len(_logs_buffer),
        'filtered': len(filtered_logs),
        'stats': _log_stats
    })


@app.route('/api/logs/clear', methods=['POST'])
def clear_logs():
    """Clear log buffer."""
    global _logs_buffer, _log_stats
    _logs_buffer.clear()
    _log_stats = {'debug': 0, 'info': 0, 'warn': 0, 'error': 0, 'fatal': 0}
    return jsonify({'success': True})


@app.route('/api/nodes')
def get_nodes():
    """Get running ROS2 nodes with status."""
    global _nodes_cache, _nodes_last_update
    
    if time.time() - _nodes_last_update > 5:
        _refresh_nodes()
    
    return jsonify({
        'nodes': list(_nodes_cache.values()),
        'count': len(_nodes_cache),
        'last_update': _nodes_last_update
    })


@app.route('/api/nodes/<path:node_name>')
def get_node_details(node_name):
    """Get detailed info about a specific node using native ROS2 graph API."""
    node_name = '/' + node_name.lstrip('/')
    
    if _ros_node is None:
        return jsonify({'success': False, 'error': 'ROS2 node not available'}), 503
    
    try:
        parts = node_name.split('/')
        base_name = parts[-1]
        namespace = '/'.join(parts[:-1]) or '/'
        
        publishers = _ros_node.get_publisher_names_and_types_by_node(base_name, namespace)
        subscribers = _ros_node.get_subscriber_names_and_types_by_node(base_name, namespace)
        services = _ros_node.get_service_names_and_types_by_node(base_name, namespace)
        
        return jsonify({
            'success': True,
            'node': node_name,
            'publishers': [{'name': name, 'type': types[0] if types else 'unknown'} for name, types in publishers],
            'subscriptions': [{'name': name, 'type': types[0] if types else 'unknown'} for name, types in subscribers],
            'services': [{'name': name, 'type': types[0] if types else 'unknown'} for name, types in services],
        })
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500


@app.route('/api/topics/<path:topic_name>/subscribe', methods=['POST'])
def topic_subscribe(topic_name):
    """Start a native echo subscription for a topic."""
    topic_name = '/' + topic_name.lstrip('/')
    if _ros_node is None:
        return jsonify({'success': False, 'error': 'ROS2 node not available'}), 503
    topic_info = _topics_cache.get(topic_name)
    if not topic_info:
        return jsonify({'success': False, 'error': f'Topic {topic_name} not found'}), 404
    msg_type = topic_info.get('type', 'unknown')
    if msg_type == 'unknown' or not ROSIDL_PY_AVAILABLE:
        return jsonify({'success': False, 'error': 'Unknown message type or rosidl_runtime_py unavailable'}), 400
    if msg_type in MonitorNode._ECHO_SKIP_TYPES:
        return jsonify({'success': False, 'error': f'Topic type {msg_type} is too large for live inspection'}), 400
    ok = _ros_node.ensure_echo_subscription(topic_name, msg_type)
    if ok:
        return jsonify({'success': True, 'topic': topic_name, 'type': msg_type})
    return jsonify({'success': False, 'error': 'Failed to create subscription'}), 500


@app.route('/api/topics/<path:topic_name>/latest')
def topic_latest(topic_name):
    """Get the latest cached message for a topic."""
    topic_name = '/' + topic_name.lstrip('/')
    if _ros_node is None:
        return jsonify({'success': False, 'error': 'ROS2 node not available'}), 503
    latest = _ros_node.get_echo_latest(topic_name)
    if latest is None:
        return jsonify({'success': True, 'topic': topic_name, 'waiting': True, 'data': None})
    return jsonify({'success': True, 'topic': topic_name, 'waiting': False, 'data': latest['data'], 'received_at': latest['timestamp']})


@app.route('/api/topics/<path:topic_name>/unsubscribe', methods=['POST'])
def topic_unsubscribe(topic_name):
    """Stop the echo subscription for a topic."""
    topic_name = '/' + topic_name.lstrip('/')
    if _ros_node is not None:
        _ros_node.destroy_echo_subscription(topic_name)
    return jsonify({'success': True, 'topic': topic_name})


@app.route('/api/topics')
def get_topics():
    """Get ROS2 topics with metadata."""
    return jsonify({
        'topics': list(_topics_cache.values()),
        'count': len(_topics_cache)
    })


def _read_cpu_times():
    """Parse /proc/stat to get (idle, total) CPU time."""
    try:
        with open('/proc/stat', 'r') as f:
            line = f.readline()
        parts = line.strip().split()
        if parts[0] != 'cpu':
            return None
        values = list(map(int, parts[1:]))
        idle = values[3]
        total = sum(values)
        return idle, total
    except Exception:
        return None


def _get_cpu_percent():
    """Calculate CPU usage percentage over a short sample."""
    first = _read_cpu_times()
    if first is None:
        return None
    time.sleep(0.2)
    second = _read_cpu_times()
    if second is None:
        return None
    idle_delta = second[0] - first[0]
    total_delta = second[1] - first[1]
    if total_delta == 0:
        return 0.0
    return round(100.0 * (1.0 - idle_delta / total_delta), 1)


def _get_ram_info():
    """Return (used_mb, total_mb, percent) from /proc/meminfo."""
    try:
        mem_total = 0
        mem_available = 0
        with open('/proc/meminfo', 'r') as f:
            for line in f:
                if line.startswith('MemTotal:'):
                    mem_total = int(line.split()[1])
                elif line.startswith('MemAvailable:'):
                    mem_available = int(line.split()[1])
        total_mb = mem_total // 1024
        used_mb = (mem_total - mem_available) // 1024
        percent = round(used_mb / total_mb * 100, 1) if total_mb else 0.0
        return used_mb, total_mb, percent
    except Exception:
        return None, None, None


def _get_temperature():
    """Read CPU/board temperature from thermal zones (millidegrees -> C)."""
    try:
        best = None
        import glob
        for path in glob.glob('/sys/class/thermal/thermal_zone*/temp'):
            try:
                with open(path, 'r') as f:
                    val = int(f.read().strip())
                if val > 0:
                    c = val / 1000.0
                    if best is None or c > best:
                        best = c
            except Exception:
                continue
        return round(best, 1) if best is not None else None
    except Exception:
        return None


@app.route('/api/system')
def get_system_status():
    """Get overall system status."""
    return jsonify({
        'ros2_running': _system_status['ros2_running'],
        'log_stats': _log_stats,
        'uptime': time.time() - _system_status.get('start_time', time.time()),
        'timestamp': datetime.now().isoformat()
    })


@app.route('/api/system/metrics')
def get_system_metrics():
    """Get CPU, RAM, and temperature metrics."""
    cpu_percent = _get_cpu_percent()
    ram_used_mb, ram_total_mb, ram_percent = _get_ram_info()
    temp_c = _get_temperature()
    return jsonify({
        'cpu_percent': cpu_percent,
        'ram_used_mb': ram_used_mb,
        'ram_total_mb': ram_total_mb,
        'ram_percent': ram_percent,
        'temp_c': temp_c,
    })


@app.route('/api/pipeline')
def get_pipeline_state():
    """Get live voice/intent/tts/cloud pipeline state."""
    return jsonify({
        'voice': _pipeline_state['voice'],
        'intent': _pipeline_state['intent'],
        'tts': _pipeline_state['tts'],
        'cloud': _pipeline_state['cloud'],
    })


# ==================== Camera Control ====================

@app.route('/api/camera/control', methods=['POST'])
def camera_control():
    """Control camera PTZ and tracking."""
    data = request.get_json() or {}
    command = data.get('command')
    
    try:
        if command == 'ptz':
            direction = data.get('direction')
            speed = data.get('speed', 50)
            
            twist = Twist()
            if direction == 'up':
                twist.angular.y = speed / 100.0
            elif direction == 'down':
                twist.angular.y = -speed / 100.0
            elif direction == 'left':
                twist.angular.z = speed / 100.0
            elif direction == 'right':
                twist.angular.z = -speed / 100.0
            elif direction == 'zoom_in':
                twist.linear.z = speed / 100.0
            elif direction == 'zoom_out':
                twist.linear.z = -speed / 100.0
            elif direction == 'stop':
                pass
            
            if _ros_node is not None:
                _ros_node._ptz_pub.publish(twist)
            else:
                return jsonify({'success': False, 'error': 'ROS2 node not ready'}), 503
            
            return jsonify({'success': True, 'command': command, 'direction': direction})
        
        elif command == 'stop':
            if _ros_node is None:
                return jsonify({'success': False, 'error': 'ROS2 node not ready'}), 503
            _ros_node._ptz_pub.publish(Twist())
            return jsonify({'success': True, 'command': command})
        
        elif command == 'tracking':
            if not AIMEE_MSGS_AVAILABLE or _ros_node is None:
                return jsonify({'success': False, 'error': 'Tracking command not available'}), 503
            
            mode = data.get('mode', 'normal')
            cmd_msg = TrackingCommand()
            cmd_msg.command = 'start'
            cmd_msg.mode = mode
            _ros_node._tracking_pub.publish(cmd_msg)
            return jsonify({'success': True, 'command': command, 'mode': mode})
        
        elif command == 'stop_tracking':
            if not AIMEE_MSGS_AVAILABLE or _ros_node is None:
                return jsonify({'success': False, 'error': 'Tracking command not available'}), 503
            
            cmd_msg = TrackingCommand()
            cmd_msg.command = 'stop'
            _ros_node._tracking_pub.publish(cmd_msg)
            return jsonify({'success': True, 'command': command})
        
        else:
            return jsonify({'success': False, 'error': f'Unknown command: {command}'}), 400
            
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500


# Lemonfox voice list (matching tts_engines.py)
LEMONFOX_VOICES = [
    "sarah", "jessica", "liam", "echo", "adam",
    "onyx", "fable", "nova", "shimmer", "alloy"
]


@app.route('/api/tts/voices')
def tts_voices():
    """Return available TTS voices."""
    return jsonify({
        'voices': LEMONFOX_VOICES,
        'default': 'sarah'
    })


@app.route('/api/tts/speak', methods=['POST'])
def tts_speak():
    """Publish a TTS message for testing with optional voice selection."""
    data = request.get_json() or {}
    text = data.get('text', '')
    voice = data.get('voice', '')
    if not text:
        return jsonify({'success': False, 'error': 'No text provided'}), 400
    if _ros_node is None:
        return jsonify({'success': False, 'error': 'ROS2 node not ready'}), 503
    try:
        msg = String()
        if voice:
            msg.data = f"lemonfox|{voice}:{text}"
        else:
            msg.data = text
        _ros_node._tts_pub.publish(msg)
        return jsonify({'success': True, 'text': msg.data})
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500


@app.route('/api/stt/submit', methods=['POST'])
def stt_submit():
    """Simulate an STT transcription by publishing to /voice/transcription."""
    data = request.get_json() or {}
    text = data.get('text', '')
    if not text:
        return jsonify({'success': False, 'error': 'No text provided'}), 400
    if _ros_node is None:
        return jsonify({'success': False, 'error': 'ROS2 node not ready'}), 503
    if _ros_node._stt_pub is None:
        return jsonify({'success': False, 'error': 'Transcription publisher not available'}), 503
    try:
        msg = Transcription()
        msg.text = text
        msg.confidence = 1.0
        msg.source = 'monitor_test'
        msg.is_command = True
        msg.is_partial = False
        msg.wake_word_detected = False
        msg.wake_word = ""
        msg.timestamp = _ros_node.get_clock().now().to_msg()
        msg.session_id = "monitor"
        _ros_node._stt_pub.publish(msg)
        return jsonify({'success': True, 'text': text})
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500


# Local LLM server URLs to try (matching aimee_llm_server and chatbot-local-llm configs)
_LLM_SERVER_URLS = [
    "http://127.0.0.1:8080",
    "http://172.17.0.1:8080",
    "http://172.25.0.1:8080",
    "http://localhost:8080",
]


@app.route('/api/llm/generate', methods=['POST'])
def llm_generate():
    """Generate text using the local LLM server (direct HTTP API test)."""
    data = request.get_json() or {}
    prompt = data.get('prompt', '')
    if not prompt:
        return jsonify({'success': False, 'error': 'No prompt provided'}), 400

    payload = {
        "messages": [
            {"role": "system", "content": "You are AIMEE, a helpful robot assistant. Be concise and direct."},
            {"role": "user", "content": prompt}
        ],
        "max_tokens": data.get('max_tokens', 150),
        "temperature": data.get('temperature', 0.7),
        "stream": False
    }

    for url in _LLM_SERVER_URLS:
        try:
            resp = requests.post(f"{url}/v1/chat/completions", json=payload, timeout=30)
            if resp.status_code == 200:
                result = resp.json()
                content = result.get("choices", [{}])[0].get("message", {}).get("content", "")
                return jsonify({'success': True, 'response': content})
        except Exception:
            continue

    return jsonify({'success': False, 'error': 'LLM server not available'}), 503


def _stop_usb_camera():
    """Stop usb_camera node to free V4L2 device for snapshot."""
    try:
        # Target the actual executable path to avoid killing unrelated shells
        subprocess.run(
            ['pkill', '-f', '/opt/ros/humble/lib/usb_cam/usb_cam_node_exe'],
            capture_output=True, timeout=5
        )
        # Wait up to 5 seconds for /dev/video2 to be released
        for _ in range(25):
            time.sleep(0.2)
            check = subprocess.run(
                ['lsof', '/dev/video2'],
                capture_output=True, timeout=5
            )
            if check.returncode != 0:
                logger.info("Stopped usb_camera for snapshot")
                return True
        logger.warning("usb_camera still holding /dev/video2 after 5s")
        return False
    except Exception as e:
        logger.warning(f"Failed to stop usb_camera: {e}")
        return False


def _start_usb_camera():
    """Start usb_camera node using monitor configuration."""
    # Avoid spawning duplicate processes
    check = subprocess.run(['pgrep', '-f', 'usb_cam_node_exe'], capture_output=True, timeout=2)
    if check.returncode == 0:
        logger.info("usb_camera already running, skip restart")
        return True
    node_def = NODE_DEFINITIONS.get('usb_camera')
    if not node_def:
        return False
    try:
        cmd_parts = [
            'source /opt/ros/humble/setup.bash',
            'source /workspace/install/setup.bash',
            f'ros2 run {node_def["package"]} {node_def["executable"]}'
        ]
        if node_def.get('args'):
            cmd_parts[-1] += ' ' + ' '.join(node_def['args'])
        cmd = ' && '.join(cmd_parts)
        proc = subprocess.Popen(
            cmd, shell=True, executable='/bin/bash',
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True, close_fds=True
        )
        logger.info(f"Started usb_camera after snapshot (PID: {proc.pid})")
        return True
    except Exception as e:
        logger.error(f"Failed to start usb_camera: {e}")
        return False


@app.route('/api/snapshot', methods=['POST'])
def take_snapshot():
    """Trigger a camera snapshot and return the image."""
    if _ros_node is None:
        return jsonify({'success': False, 'error': 'ROS2 node not ready'}), 503
    
    data = request.get_json() or {}
    resolution = data.get('resolution', '')
    quality = data.get('quality', 95)
    
    usb_check = subprocess.run(['pgrep', '-f', 'usb_cam_node_exe'], capture_output=True, timeout=2)
    usb_was_running = usb_check.returncode == 0
    
    result = None
    for attempt in range(2):
        if usb_was_running:
            stopped = _stop_usb_camera()
            if stopped:
                time.sleep(0.5)  # let V4L2 driver settle
        try:
            result = _ros_node.capture_snapshot(resolution=resolution, quality=quality)
            if result.get('success') or 'busy' not in result.get('message', '').lower():
                break
            logger.warning(f"Snapshot busy on attempt {attempt+1}, retrying...")
            time.sleep(1.5)
        except Exception as e:
            result = {'success': False, 'message': str(e)}
            break
    
    if usb_was_running:
        _start_usb_camera()
    return jsonify(result)


@app.route('/api/snapshot/send', methods=['POST'])
def send_snapshot_to_cloud():
    """Capture snapshot and forward it to AimeeCloud via the cloud bridge."""
    if _ros_node is None:
        return jsonify({'success': False, 'error': 'ROS2 node not ready'}), 503
    
    data = request.get_json() or {}
    resolution = data.get('resolution', '')
    quality = data.get('quality', 95)
    
    usb_check = subprocess.run(['pgrep', '-f', 'usb_cam_node_exe'], capture_output=True, timeout=2)
    usb_was_running = usb_check.returncode == 0
    
    result = None
    for attempt in range(2):
        if usb_was_running:
            stopped = _stop_usb_camera()
            if stopped:
                time.sleep(0.5)
        try:
            result = _ros_node.capture_snapshot(resolution=resolution, quality=quality)
            if result.get('success') or 'busy' not in result.get('message', '').lower():
                break
            logger.warning(f"Snapshot busy on attempt {attempt+1}, retrying...")
            time.sleep(1.5)
        except Exception as e:
            result = {'success': False, 'message': str(e)}
            break
    
    if result and result.get('success'):
        import json as _json
        payload = {
            'image_base64': result.get('image_base64', ''),
            'request_id': data.get('request_id', ''),
            'session_id': data.get('session_id', ''),
        }
        msg = String()
        msg.data = _json.dumps(payload)
        _ros_node._cloud_snapshot_pub.publish(msg)
        if usb_was_running:
            _start_usb_camera()
        return jsonify({'success': True, 'message': 'Snapshot sent to AimeeCloud'})
    
    if usb_was_running:
        _start_usb_camera()
    return jsonify({'success': False, 'message': result.get('message', 'Snapshot failed')}), 500


@app.route('/api/camera/frame.jpg')
def camera_frame():
    """Serve the latest compressed camera frame as a JPEG image."""
    if _ros_node is None:
        return Response('', status=503)
    
    frame = _ros_node.get_camera_frame()
    if not frame:
        return Response('', status=204)
    
    resp = make_response(frame)
    resp.headers['Content-Type'] = 'image/jpeg'
    resp.headers['Cache-Control'] = 'no-store, no-cache, must-revalidate, max-age=0'
    return resp


# ==================== Node Control Endpoints ====================

@app.route('/api/nodes/definitions')
def get_node_definitions():
    """Get available node definitions."""
    return jsonify({
        'nodes': NODE_DEFINITIONS,
        'count': len(NODE_DEFINITIONS)
    })


@app.route('/api/nodes/running')
def get_running_nodes():
    """Get a simple list of running node IDs."""
    ros_running = {}
    for node_id, node_def in NODE_DEFINITIONS.items():
        ros_name = node_def.get('ros_name', '')
        if ros_name and ros_name in _nodes_cache:
            ros_running[node_id] = True
    
    core_running = _core_process is not None and _core_process.poll() is None
    
    return jsonify({
        'nodes': ros_running,
        'core_running': core_running
    })


@app.route('/api/nodes/managed')
def get_managed_nodes():
    """Get status of managed nodes + actual ROS2 graph state."""
    global _managed_nodes
    
    # Clean up finished processes
    nodes_response = {}
    for node_id, info in list(_managed_nodes.items()):
        proc = info.get('process')
        if proc and proc.poll() is not None:
            info['status'] = 'stopped'
            info['exit_code'] = proc.returncode
        
        nodes_response[node_id] = {
            'node_id': info.get('node_id'),
            'name': info.get('name'),
            'status': info.get('status'),
            'pid': info.get('pid'),
            'started_at': info.get('started_at'),
            'stopped_at': info.get('stopped_at'),
            'exit_code': info.get('exit_code'),
            'package': info.get('package'),
            'executable': info.get('executable')
        }
    
    # Determine which nodes are actually running in ROS2 graph
    ros_running = {}
    for node_id, node_def in NODE_DEFINITIONS.items():
        ros_name = node_def.get('ros_name', '')
        if ros_name and ros_name in _nodes_cache:
            ros_running[node_id] = True
    
    core_running = _core_process is not None and _core_process.poll() is None
    
    return jsonify({
        'nodes': nodes_response,
        'core_running': core_running,
        'ros_running': ros_running
    })


@app.route('/api/nodes/start', methods=['POST'])
def start_node():
    """Start a managed node."""
    global _managed_nodes
    
    data = request.get_json() or {}
    node_id = data.get('node_id')
    
    if not node_id or node_id not in NODE_DEFINITIONS:
        return jsonify({'success': False, 'error': 'Invalid node_id'}), 400
    
    # Check if already running in our managed set
    if node_id in _managed_nodes:
        proc = _managed_nodes[node_id].get('process')
        if proc and proc.poll() is None:
            return jsonify({'success': False, 'error': 'Node already running'}), 409
    
    node_def = NODE_DEFINITIONS[node_id]
    
    try:
        cmd_parts = [
            'source /opt/ros/humble/setup.bash',
            'source /workspace/install/setup.bash',
            f'ros2 run {node_def["package"]} {node_def["executable"]}'
        ]
        if node_def.get('args'):
            cmd_parts[-1] += ' ' + ' '.join(node_def['args'])
        
        cmd = ' && '.join(cmd_parts)
        
        proc = subprocess.Popen(
            cmd,
            shell=True,
            executable='/bin/bash',
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            start_new_session=True,
            close_fds=True
        )
        
        _managed_nodes[node_id] = {
            'node_id': node_id,
            'name': node_def['name'],
            'process': proc,
            'pid': proc.pid,
            'status': 'running',
            'started_at': datetime.now().isoformat(),
            'package': node_def['package'],
            'executable': node_def['executable']
        }
        
        logger.info(f"Started node {node_id} (PID: {proc.pid})")
        
        # Brief wait for usb_camera to register in ROS graph
        if node_id == 'usb_camera':
            for _ in range(15):
                time.sleep(0.2)
                if _ros_node is not None:
                    try:
                        node_names = _ros_node.get_node_names_and_namespaces()
                        _refresh_nodes(node_names)
                    except Exception:
                        pass
                if node_def.get('ros_name') and node_def['ros_name'] in _nodes_cache:
                    break
        
        return jsonify({
            'success': True,
            'node_id': node_id,
            'pid': proc.pid,
            'status': 'running'
        })
        
    except Exception as e:
        logger.error(f"Failed to start node {node_id}: {e}")
        return jsonify({'success': False, 'error': str(e)}), 500


@app.route('/api/nodes/stop', methods=['POST'])
def stop_node():
    """Stop a node (managed or core-launched)."""
    global _managed_nodes
    
    data = request.get_json() or {}
    node_id = data.get('node_id')
    
    if not node_id or node_id not in NODE_DEFINITIONS:
        return jsonify({'success': False, 'error': 'Invalid node_id'}), 400
    
    node_def = NODE_DEFINITIONS[node_id]
    proc = None
    
    # If we have it in managed nodes, use that process
    if node_id in _managed_nodes:
        proc = _managed_nodes[node_id].get('process')
    
    try:
        import signal
        
        if proc:
            # Managed process - kill by process group
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                proc.wait(timeout=2)
            except (subprocess.TimeoutExpired, ProcessLookupError):
                pass
            # Always follow up with SIGKILL
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                proc.wait(timeout=2)
            except Exception:
                pass
            
            _managed_nodes[node_id]['status'] = 'stopped'
            _managed_nodes[node_id]['stopped_at'] = datetime.now().isoformat()
        
        # Core-launched or orphaned process - find by executable name
        executable = node_def['executable']
        subprocess.run(
            ['pkill', '-f', executable],
            capture_output=True, timeout=5
        )
        time.sleep(0.5)
        # Force kill if still running
        check = subprocess.run(
            ['pgrep', '-f', executable],
            capture_output=True, timeout=2
        )
        if check.returncode == 0:
            subprocess.run(
                ['pkill', '-9', '-f', executable],
                capture_output=True, timeout=5
            )
        
        # Extra cleanup for usb_camera: also target the absolute path
        if node_id == 'usb_camera':
            time.sleep(0.5)
            subprocess.run(
                ['pkill', '-9', '-f', '/opt/ros/humble/lib/usb_cam/usb_cam_node_exe'],
                capture_output=True, timeout=5
            )
        
        logger.info(f"Stopped node {node_id}")
        
        return jsonify({
            'success': True,
            'node_id': node_id,
            'status': 'stopped'
        })
        
    except Exception as e:
        logger.error(f"Failed to stop node {node_id}: {e}")
        return jsonify({'success': False, 'error': str(e)}), 500


@app.route('/api/core/start', methods=['POST'])
def start_core():
    """Start core ROS2 nodes (bringup)."""
    global _core_process
    
    if _core_process and _core_process.poll() is None:
        return jsonify({'success': False, 'error': 'Core already running'}), 409
    
    try:
        cmd = 'source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 launch aimee_bringup core.launch.py'
        
        _core_process = subprocess.Popen(
            cmd,
            shell=True,
            executable='/bin/bash',
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            start_new_session=True,
            close_fds=True
        )
        
        logger.info(f"Started core (PID: {_core_process.pid})")
        
        return jsonify({
            'success': True,
            'pid': _core_process.pid,
            'status': 'starting'
        })
        
    except Exception as e:
        logger.error(f"Failed to start core: {e}")
        return jsonify({'success': False, 'error': str(e)}), 500


@app.route('/api/core/stop', methods=['POST'])
def stop_core():
    """Stop core ROS2 nodes."""
    global _core_process
    
    if not _core_process:
        return jsonify({'success': False, 'error': 'Core not running'}), 404
    
    try:
        import signal
        os.killpg(os.getpgid(_core_process.pid), signal.SIGTERM)
        
        try:
            _core_process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(_core_process.pid), signal.SIGKILL)
            _core_process.wait(timeout=2)
        
        logger.info("Stopped core")
        
        return jsonify({
            'success': True,
            'status': 'stopped'
        })
        
    except Exception as e:
        logger.error(f"Failed to stop core: {e}")
        return jsonify({'success': False, 'error': str(e)}), 500


@app.route('/api/core/status')
def core_status():
    """Get core status."""
    global _core_process
    
    is_running = _core_process is not None and _core_process.poll() is None
    
    return jsonify({
        'running': is_running,
        'pid': _core_process.pid if is_running else None
    })


@app.route('/api/session/clear', methods=['POST'])
def clear_session():
    """Clear the current AimeeCloud session so the next request starts fresh."""
    if _ros_node is None:
        return jsonify({'success': False, 'error': 'ROS2 node not ready'}), 503
    try:
        msg = Bool()
        msg.data = True
        _ros_node._clear_session_pub.publish(msg)
        return jsonify({'success': True, 'message': 'Session clear requested'})
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500


# ==================== Map Console Flask Routes ====================

@app.route('/maps')
def map_console_page():
    """Serve the map console HTML page."""
    return render_template('map_console.html')

@app.route('/api/map/image.png')
def map_image_png():
    """Render current occupancy grid as PNG with robot pose overlay."""
    if _ros_node is None:
        return jsonify({'error': 'ROS2 node not ready'}), 503
    try:
        png_bytes = _ros_node.render_map_png()
        if not png_bytes:
            return jsonify({'error': 'No map data available'}), 503
        return Response(png_bytes, mimetype='image/png')
    except Exception as e:
        logger.error(f"Map PNG render error: {e}")
        return jsonify({'error': str(e)}), 500

@app.route('/api/map/pose')
def map_pose():
    """Return current robot pose."""
    if _ros_node is None:
        return jsonify({'error': 'ROS2 node not ready'}), 503
    try:
        with _ros_node._map_lock:
            pose = dict(_ros_node._latest_pose)
            pose['timestamp'] = _ros_node._latest_pose_time
        return jsonify(pose)
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/api/maps')
def list_maps():
    """List all maps in the library."""
    if _ros_node is None:
        return jsonify({'success': False, 'error': 'ROS2 node not ready'}), 503
    try:
        cli = _ros_node._get_map_mgr_client('list_maps')
        if not cli.wait_for_service(timeout_sec=2.0):
            return jsonify({'success': False, 'error': 'Map manager not available'}), 503
        from aimee_msgs.srv import ListMaps
        future = cli.call_async(ListMaps.Request())
        # Simple synchronous wait
        start = time.time()
        while not future.done() and time.time() - start < 3.0:
            time.sleep(0.05)
        if not future.done():
            return jsonify({'success': False, 'error': 'Timeout'}), 504
        resp = future.result()
        maps = []
        for i in range(len(resp.ids)):
            maps.append({
                'id': resp.ids[i],
                'name': resp.names[i] if i < len(resp.names) else resp.ids[i],
                'description': resp.descriptions[i] if i < len(resp.descriptions) else '',
                'type': resp.types[i] if i < len(resp.types) else 'unknown',
            })
        return jsonify({'success': True, 'maps': maps})
    except Exception as e:
        logger.error(f"list_maps error: {e}")
        return jsonify({'success': False, 'error': str(e)}), 500

@app.route('/api/maps', methods=['POST'])
def save_map_api():
    """Save current map to library."""
    if _ros_node is None:
        return jsonify({'success': False, 'error': 'ROS2 node not ready'}), 503
    try:
        data = request.get_json(force=True) or {}
        map_id = data.get('map_id', '').strip()
        name = data.get('name', map_id).strip()
        description = data.get('description', '')
        if not map_id:
            return jsonify({'success': False, 'message': 'map_id required'}), 400
        cli = _ros_node._get_map_mgr_client('save_map')
        if not cli.wait_for_service(timeout_sec=2.0):
            return jsonify({'success': False, 'error': 'Map manager not available'}), 503
        from aimee_msgs.srv import SaveMap
        req = SaveMap.Request()
        req.map_id = map_id
        req.name = name
        req.description = description
        future = cli.call_async(req)
        start = time.time()
        while not future.done() and time.time() - start < 5.0:
            time.sleep(0.05)
        if not future.done():
            return jsonify({'success': False, 'message': 'Timeout'}), 504
        resp = future.result()
        return jsonify({'success': resp.success, 'message': resp.message})
    except Exception as e:
        logger.error(f"save_map error: {e}")
        return jsonify({'success': False, 'message': str(e)}), 500

@app.route('/api/maps/<path:map_id>/load', methods=['POST'])
def load_map_api(map_id):
    """Load a map from library and localize."""
    if _ros_node is None:
        return jsonify({'success': False, 'error': 'ROS2 node not ready'}), 503
    try:
        cli = _ros_node._get_map_mgr_client('load_map')
        if not cli.wait_for_service(timeout_sec=2.0):
            return jsonify({'success': False, 'error': 'Map manager not available'}), 503
        from aimee_msgs.srv import LoadMap
        req = LoadMap.Request()
        req.map_id = map_id
        req.localize = True
        future = cli.call_async(req)
        start = time.time()
        while not future.done() and time.time() - start < 5.0:
            time.sleep(0.05)
        if not future.done():
            return jsonify({'success': False, 'message': 'Timeout'}), 504
        resp = future.result()
        return jsonify({'success': resp.success, 'message': resp.message})
    except Exception as e:
        logger.error(f"load_map error: {e}")
        return jsonify({'success': False, 'message': str(e)}), 500

@app.route('/api/maps/<path:map_id>', methods=['DELETE'])
def delete_map_api(map_id):
    """Delete a map from library."""
    if _ros_node is None:
        return jsonify({'success': False, 'error': 'ROS2 node not ready'}), 503
    try:
        cli = _ros_node._get_map_mgr_client('delete_map')
        if not cli.wait_for_service(timeout_sec=2.0):
            return jsonify({'success': False, 'error': 'Map manager not available'}), 503
        from aimee_msgs.srv import DeleteMap
        req = DeleteMap.Request()
        req.map_id = map_id
        future = cli.call_async(req)
        start = time.time()
        while not future.done() and time.time() - start < 3.0:
            time.sleep(0.05)
        if not future.done():
            return jsonify({'success': False, 'message': 'Timeout'}), 504
        resp = future.result()
        return jsonify({'success': resp.success, 'message': resp.message})
    except Exception as e:
        logger.error(f"delete_map error: {e}")
        return jsonify({'success': False, 'message': str(e)}), 500

@app.route('/api/location', methods=['POST'])
def set_location_api():
    """Publish location name to /location_name topic."""
    if _ros_node is None:
        return jsonify({'success': False, 'error': 'ROS2 node not ready'}), 503
    try:
        data = request.get_json(force=True) or {}
        location = data.get('location', '').strip()
        if not location:
            return jsonify({'success': False, 'message': 'location required'}), 400
        pub = _ros_node.create_publisher(String, '/location_name', 10)
        msg = String()
        msg.data = location
        pub.publish(msg)
        # Give DDS a moment then destroy
        time.sleep(0.1)
        _ros_node.destroy_publisher(pub)
        return jsonify({'success': True, 'message': f'Location set: {location}'})
    except Exception as e:
        logger.error(f"set_location error: {e}")
        return jsonify({'success': False, 'message': str(e)}), 500

@app.route('/api/exploration/start', methods=['POST'])
def start_exploration_api():
    """Enable exploration mode."""
    if _ros_node is None:
        return jsonify({'success': False, 'error': 'ROS2 node not ready'}), 503
    try:
        pub = _ros_node.create_publisher(String, '/exploration_command', 10)
        msg = String()
        msg.data = 'start'
        pub.publish(msg)
        time.sleep(0.1)
        _ros_node.destroy_publisher(pub)
        _ros_node._exploration_active = True
        return jsonify({'success': True, 'message': 'Exploration started'})
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)}), 500

@app.route('/api/exploration/stop', methods=['POST'])
def stop_exploration_api():
    """Disable exploration mode."""
    if _ros_node is None:
        return jsonify({'success': False, 'error': 'ROS2 node not ready'}), 503
    try:
        pub = _ros_node.create_publisher(String, '/exploration_command', 10)
        msg = String()
        msg.data = 'stop'
        pub.publish(msg)
        time.sleep(0.1)
        _ros_node.destroy_publisher(pub)
        _ros_node._exploration_active = False
        return jsonify({'success': True, 'message': 'Exploration stopped'})
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)}), 500

@app.route('/api/exploration/status')
def exploration_status_api():
    """Get exploration status."""
    if _ros_node is None:
        return jsonify({'active': False, 'error': 'ROS2 node not ready'}), 503
    try:
        return jsonify({
            'active': getattr(_ros_node, '_exploration_active', False),
        })
    except Exception as e:
        return jsonify({'active': False, 'error': str(e)}), 500


# ==================== Helper Functions ====================

def _refresh_nodes(native_api=None):
    """Refresh the nodes cache using native ROS2 API when available."""
    global _nodes_cache, _nodes_last_update
    
    # Build reverse lookup from ros_name -> definition
    ros_name_to_def = {}
    for def_info in NODE_DEFINITIONS.values():
        rn = def_info.get('ros_name', '')
        if rn:
            ros_name_to_def[rn] = def_info
    
    try:
        if native_api is not None:
            nodes = {}
            for name, namespace in native_api:
                full_name = (namespace.rstrip('/') + '/' + name) if namespace != '/' else '/' + name
                if not full_name.startswith('/'):
                    full_name = '/' + full_name
                
                # Skip transient CLI nodes
                if name.startswith('_ros2cli_'):
                    continue
                
                definition = ros_name_to_def.get(full_name)
                if definition:
                    display_name = definition['name']
                    category = definition.get('category', 'other')
                    icon = definition.get('icon', '⚙️')
                else:
                    display_name = full_name
                    icon = None
                    category = 'other'
                    if 'wake_word' in full_name.lower():
                        category = 'audio'
                    elif 'voice' in full_name.lower() or 'stt' in full_name.lower():
                        category = 'audio'
                    elif 'tts' in full_name.lower():
                        category = 'audio'
                    elif 'intent' in full_name.lower():
                        category = 'ai'
                    elif 'llm' in full_name.lower():
                        category = 'ai'
                    elif 'skill' in full_name.lower():
                        category = 'skills'
                    elif 'camera' in full_name.lower() or 'vision' in full_name.lower():
                        category = 'vision'
                    elif 'dashboard' in full_name.lower() or 'monitor' in full_name.lower():
                        category = 'tools'
                
                nodes[full_name] = {
                    'name': display_name,
                    'ros_name': full_name,
                    'category': category,
                    'icon': icon,
                    'status': 'running',
                    'last_seen': time.time()
                }
            
            _nodes_cache = nodes
            _nodes_last_update = time.time()
            _system_status['ros2_running'] = True
        else:
            _system_status['ros2_running'] = False
    except Exception as e:
        logger.warning(f"Failed to refresh nodes: {e}")
        _system_status['ros2_running'] = False


# ==================== ROS2 Node ====================

class MonitorNode(Node):
    """
    ROS2 Node that captures logs and provides monitoring data.
    """
    
    def __init__(self):
        super().__init__('ros2_monitor')
        
        # Subscribe to /rosout for log messages
        self._log_sub = self.create_subscription(
            Log,
            '/rosout',
            self._on_log_message,
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=100
            )
        )
        
        # Publishers for camera control, TTS testing, STT simulation, and cloud snapshot upload
        self._ptz_pub = self.create_publisher(Twist, '/camera/cmd_ptz', 10)
        self._tts_pub = self.create_publisher(String, '/tts/speak', 10)
        self._stt_pub = self.create_publisher(Transcription, '/voice/transcription', 10) if Transcription else None
        self._cloud_snapshot_pub = self.create_publisher(String, '/cloud/snapshot_manual_upload', 10)
        self._clear_session_pub = self.create_publisher(Bool, '/cloud/clear_session', 10)
        if AIMEE_MSGS_AVAILABLE:
            self._tracking_pub = self.create_publisher(TrackingCommand, '/camera/tracking', 10)
            self._snapshot_cli = self.create_client(CaptureSnapshot, '/camera/capture_snapshot')
        else:
            self._tracking_pub = None
            self._snapshot_cli = None
        
        # Camera frame cache for live view
        self._camera_frame_lock = threading.Lock()
        self._camera_frame_data = None
        self._camera_frame_sub = self.create_subscription(
            CompressedImage, '/camera/image_raw/compressed',
            self._on_camera_frame,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        )

        # Map / Odom cache for map console
        self._map_lock = threading.Lock()
        self._latest_map = None          # nav_msgs/OccupancyGrid
        self._latest_map_time = 0.0
        self._latest_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self._latest_pose_time = 0.0
        try:
            from nav_msgs.msg import OccupancyGrid, Odometry
            self._map_sub = self.create_subscription(
                OccupancyGrid, '/map',
                self._on_map_message,
                QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
            )
            self._odom_sub = self.create_subscription(
                Odometry, '/odom',
                self._on_odom_message,
                QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
            )
        except Exception as e:
            self.get_logger().warning(f"Map/odom subscription setup failed: {e}")
            self._map_sub = None
            self._odom_sub = None

        # Exploration state cache
        self._exploration_active = False

        # Map manager service clients (created lazily)
        self._map_mgr_clients = {}

        # Pipeline subscriptions
        self._voice_sub = self.create_subscription(
            (Transcription if Transcription else String), '/voice/transcription',
            self._on_voice_transcription,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        )
        self._intent_sub = self.create_subscription(
            String, '/intent/classified',
            self._on_intent_classified,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        )
        self._tts_sub = self.create_subscription(
            String, '/tts/speak',
            self._on_tts_speak,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        )
        self._cloud_sub = self.create_subscription(
            Bool, '/cloud/connected',
            self._on_cloud_connected,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        )
        
        # Timers for periodic updates
        self._nodes_timer = self.create_timer(10.0, self._update_nodes)
        self._topics_timer = self.create_timer(30.0, self._refresh_topics)
        
        # Immediate initial refresh
        self._initial_refresh_timer = self.create_timer(1.0, self._initial_refresh)
        
        self.get_logger().info("ROS2 Monitor Node started")
    
    def capture_snapshot(self, resolution: str = "", quality: int = 95, timeout_sec: float = 15.0) -> dict:
        """Call the /camera/capture_snapshot service and return result dict."""
        if not self._snapshot_cli:
            return {"success": False, "message": "aimee_msgs not available"}
        
        if not self._snapshot_cli.wait_for_service(timeout_sec=2.0):
            return {"success": False, "message": "Snapshot service not available"}
        
        req = CaptureSnapshot.Request()
        req.resolution = resolution
        req.quality = quality
        
        future = self._snapshot_cli.call_async(req)
        
        start = time.time()
        while not future.done() and time.time() - start < timeout_sec:
            time.sleep(0.1)
        
        if not future.done():
            return {"success": False, "message": "Snapshot service call timed out"}
        
        try:
            resp = future.result()
            import base64
            image_b64 = base64.b64encode(resp.image.data).decode('utf-8') if resp.success else ""
            return {
                "success": resp.success,
                "message": resp.message,
                "format": resp.image.format if resp.success else "",
                "image_base64": image_b64,
            }
        except Exception as e:
            return {"success": False, "message": f"Snapshot error: {e}"}
    
    def _get_map_mgr_client(self, service_name: str):
        """Lazy-create service clients for map manager."""
        if service_name not in self._map_mgr_clients:
            try:
                from aimee_msgs.srv import SaveMap, LoadMap, ListMaps, DeleteMap
                srv_map = {
                    'save_map': SaveMap,
                    'load_map': LoadMap,
                    'list_maps': ListMaps,
                    'delete_map': DeleteMap,
                }
                self._map_mgr_clients[service_name] = self.create_client(
                    srv_map[service_name], f'/map_manager/{service_name}'
                )
            except Exception as e:
                self.get_logger().warning(f"Failed to create map mgr client {service_name}: {e}")
        return self._map_mgr_clients.get(service_name)

    def render_map_png(self, max_size: int = 512) -> bytes:
        """Render latest occupancy grid as PNG bytes with robot pose overlay."""
        try:
            from PIL import Image, ImageDraw
        except ImportError:
            logger.error("PIL not available for map rendering")
            return b''

        with self._map_lock:
            msg = self._latest_map
            pose = dict(self._latest_pose)

        if msg is None:
            return b''

        w = msg.info.width
        h = msg.info.height
        if w == 0 or h == 0:
            return b''

        # Create grayscale image from occupancy data
        # ROS: -1=unknown, 0=free, 100=occupied
        data = list(msg.data)
        arr = bytearray(w * h)
        for i, v in enumerate(data):
            if v == -1:
                arr[i] = 205  # light gray
            elif v >= 50:
                arr[i] = 0    # black
            else:
                arr[i] = 254  # white

        img = Image.frombytes('L', (w, h), bytes(arr))

        # Draw robot pose if available and recent (<5s)
        if time.time() - self._latest_pose_time < 5.0:
            draw = ImageDraw.Draw(img)
            res = msg.info.resolution
            ox = msg.info.origin.position.x
            oy = msg.info.origin.position.y
            # Transform robot world -> grid
            gx = (pose['x'] - ox) / res
            gy = (pose['y'] - oy) / res
            # PIL y is top-down, grid y is bottom-up typically
            gy_img = h - 1 - gy
            r = max(2, int(0.15 / res))  # 15cm radius in pixels
            draw.ellipse([gx - r, gy_img - r, gx + r, gy_img + r], fill=255, outline=255)
            # Heading arrow
            theta = pose['theta']
            ax = gx + r * 2 * math.cos(theta)
            ay = gy_img - r * 2 * math.sin(theta)  # negate because image y is flipped
            draw.line([gx, gy_img, ax, ay], fill=255, width=max(1, r // 2))

        # Resize if too large (keep aspect ratio)
        if w > max_size or h > max_size:
            scale = max_size / max(w, h)
            new_w = int(w * scale)
            new_h = int(h * scale)
            img = img.resize((new_w, new_h), Image.NEAREST)

        import io
        buf = io.BytesIO()
        img.save(buf, format='PNG')
        return buf.getvalue()

    def _initial_refresh(self):
        """One-shot initial refresh after node is fully initialized."""
        self._update_nodes()
        self._refresh_topics()
        self.destroy_timer(self._initial_refresh_timer)
    
    def _on_voice_transcription(self, msg):
        global _pipeline_state
        text = getattr(msg, 'text', getattr(msg, 'data', ''))
        _pipeline_state['voice'] = {'text': text, 'timestamp': time.time(), 'is_partial': False}
    
    def _on_intent_classified(self, msg):
        global _pipeline_state
        _pipeline_state['intent'] = {
            'type': getattr(msg, 'intent_type', ''),
            'action': getattr(msg, 'action', ''),
            'confidence': getattr(msg, 'confidence', 0.0),
            'timestamp': time.time()
        }
    
    def _on_tts_speak(self, msg: String):
        global _pipeline_state
        _pipeline_state['tts'] = {'text': msg.data, 'timestamp': time.time()}
    
    def _on_camera_frame(self, msg: CompressedImage):
        """Cache latest compressed camera frame."""
        with self._camera_frame_lock:
            self._camera_frame_data = bytes(msg.data)
            self._camera_frame_time = time.time()
    
    def get_camera_frame(self) -> bytes:
        """Return latest camera frame bytes or empty bytes if stale (>2s)."""
        with self._camera_frame_lock:
            if self._camera_frame_data is None:
                return b''
            if getattr(self, '_camera_frame_time', 0) < time.time() - 2.0:
                return b''
            return self._camera_frame_data
    
    def _on_cloud_connected(self, msg: Bool):
        global _pipeline_state
        _pipeline_state['cloud'] = {'connected': msg.data, 'timestamp': time.time()}

    def _on_map_message(self, msg):
        """Cache latest occupancy grid."""
        with self._map_lock:
            self._latest_map = msg
            self._latest_map_time = time.time()

    def _on_odom_message(self, msg):
        """Cache latest odometry pose."""
        with self._map_lock:
            self._latest_pose['x'] = msg.pose.pose.position.x
            self._latest_pose['y'] = msg.pose.pose.position.y
            # Extract yaw from quaternion
            q = msg.pose.pose.orientation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self._latest_pose['theta'] = math.atan2(siny, cosy)
            self._latest_pose_time = time.time()

    def _on_log_message(self, msg: Log):
        """Handle incoming log message."""
        global _log_stats
        
        level_map = {
            10: 'debug',
            20: 'info',
            30: 'warn',
            40: 'error',
            50: 'fatal'
        }
        level_str = level_map.get(msg.level, 'unknown')
        
        if level_str in _log_stats:
            _log_stats[level_str] += 1
        
        log_entry = {
            'timestamp': datetime.fromtimestamp(msg.stamp.sec + msg.stamp.nanosec * 1e-9).isoformat(),
            'level': level_str,
            'node': msg.name,
            'msg': msg.msg,
            'file': msg.file,
            'line': msg.line,
            'function': msg.function
        }
        
        _logs_buffer.append(log_entry)
    
    def _update_nodes(self):
        """Periodic node list update using native ROS2 API."""
        try:
            _refresh_nodes(self.get_node_names_and_namespaces())
        except Exception as e:
            logger.warning(f"Node update error: {e}")
    
    # Message types to skip for auto echo (large/binary data)
    _ECHO_SKIP_TYPES = {
        'sensor_msgs/msg/Image',
        'sensor_msgs/msg/CompressedImage',
        'sensor_msgs/msg/PointCloud2',
        'sensor_msgs/msg/LaserScan',
        'sensor_msgs/msg/PointCloud',
        'nav_msgs/msg/OccupancyGrid',
        'visualization_msgs/msg/MarkerArray',
        'theora_image_transport/msg/Packet',
    }
    
    def _refresh_topics(self):
        """Refresh topic list using native ROS2 API and auto-subscribe to small topics."""
        global _topics_cache
        try:
            topics = []
            for topic_name, topic_types in self.get_topic_names_and_types():
                msg_type = topic_types[0] if topic_types else 'unknown'
                topics.append({
                    'name': topic_name,
                    'type': msg_type,
                    'hz': 0,
                    'bandwidth': '0 B',
                    'last_update': 0
                })
                # Auto-subscribe to lightweight topics for live inspection
                if (ROSIDL_PY_AVAILABLE and
                        msg_type not in self._ECHO_SKIP_TYPES and
                        msg_type != 'unknown'):
                    self.ensure_echo_subscription(topic_name, msg_type)
            _topics_cache = {t['name']: t for t in topics}
        except Exception as e:
            logger.warning(f"Failed to refresh topics: {e}")
    
    # ==================== Topic Echo Subscriptions ====================
    ECHO_TIMEOUT = 60.0
    
    def ensure_echo_subscription(self, topic_name: str, msg_type_str: str) -> bool:
        """Create a persistent subscription to cache the latest message."""
        if not ROSIDL_PY_AVAILABLE:
            return False
        with getattr(self, '_echo_lock', threading.Lock()):
            if not hasattr(self, '_echo_subs'):
                self._echo_subs = {}
                self._echo_cache = {}
                self._echo_last_access = {}
                self._echo_lock = threading.Lock()
            if topic_name in self._echo_subs:
                self._echo_last_access[topic_name] = time.time()
                return True
        try:
            msg_cls = get_message(msg_type_str)
            sub = self.create_subscription(
                msg_cls, topic_name,
                lambda msg, tn=topic_name: self._on_echo_message(tn, msg),
                QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1
                )
            )
            with self._echo_lock:
                self._echo_subs[topic_name] = sub
                self._echo_cache[topic_name] = None
                self._echo_last_access[topic_name] = time.time()
            # Start cleanup timer if not running
            if not hasattr(self, '_echo_cleanup_timer') or self._echo_cleanup_timer is None:
                self._echo_cleanup_timer = self.create_timer(10.0, self._cleanup_echo_subs)
            self.get_logger().debug(f"Echo subscription created for {topic_name}")
            return True
        except Exception as e:
            logger.warning(f"Failed to create echo sub for {topic_name}: {e}")
            return False
    
    def destroy_echo_subscription(self, topic_name: str):
        """Destroy a persistent echo subscription."""
        with getattr(self, '_echo_lock', threading.Lock()):
            if not hasattr(self, '_echo_subs'):
                return
            sub = self._echo_subs.pop(topic_name, None)
            self._echo_cache.pop(topic_name, None)
            self._echo_last_access.pop(topic_name, None)
        if sub:
            try:
                self.destroy_subscription(sub)
            except Exception as e:
                logger.warning(f"Failed to destroy echo sub for {topic_name}: {e}")
    
    def _cleanup_echo_subs(self):
        """Remove echo subscriptions idle for too long."""
        now = time.time()
        stale = []
        with getattr(self, '_echo_lock', threading.Lock()):
            if not hasattr(self, '_echo_last_access'):
                return
            for topic_name, last_access in list(self._echo_last_access.items()):
                if now - last_access > self.ECHO_TIMEOUT:
                    stale.append(topic_name)
        for topic_name in stale:
            self.destroy_echo_subscription(topic_name)
    
    def _on_echo_message(self, topic_name: str, msg):
        """Cache an incoming echoed message."""
        self.get_logger().debug(f"Echo received on {topic_name}")
        try:
            data = message_to_ordereddict(msg)
        except Exception as e:
            data = {'_raw': str(msg), '_error': str(e)}
        with getattr(self, '_echo_lock', threading.Lock()):
            if not hasattr(self, '_echo_cache'):
                self._echo_cache = {}
                self._echo_last_access = {}
            self._echo_cache[topic_name] = {
                'data': data,
                'timestamp': time.time(),
                'type': type(msg).__module__ + '/' + type(msg).__name__
            }
            self._echo_last_access[topic_name] = time.time()
    
    def get_echo_latest(self, topic_name: str):
        """Return the latest cached message dict or None."""
        with getattr(self, '_echo_lock', threading.Lock()):
            if not hasattr(self, '_echo_cache'):
                return None
            if topic_name in self._echo_last_access:
                self._echo_last_access[topic_name] = time.time()
            cache = self._echo_cache.get(topic_name)
            if cache is None:
                return None
            return dict(cache)

# ==================== Flask Thread ====================

def run_flask(host='0.0.0.0', port=8081):
    """Run Flask server in background thread."""
    import logging as flask_logging
    flask_logging.getLogger('werkzeug').setLevel(flask_logging.ERROR)
    
    app.run(host=host, port=port, debug=False, threaded=True)


# ==================== Main ====================

def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    _system_status['start_time'] = time.time()
    
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()
    
    global _ros_node
    node = MonitorNode()
    _ros_node = node
    
    print("\n" + "="*60)
    print("🔍 AIMEE ROS2 Monitor Dashboard")
    print("="*60)
    print(f"\n📊 Dashboard URL: http://localhost:8081")
    print("\n" + "="*60 + "\n")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

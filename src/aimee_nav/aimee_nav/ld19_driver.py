"""LD19 LiDAR direct serial driver.

Implements the LD19 data protocol based on the LudovaTech reference:
https://github.com/LudovaTech/lidar-LD19-tutorial

Protocol summary:
- 230400 baud, 8N1
- 47-byte packets, header 0x54, verlen 0x2C
- 12 measurement points per packet
- Angles in 0.01° units, wrap at 36000
- Timestamp wraps at 30000 ms
"""

import math
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Deque, List, Optional

import serial


@dataclass
class LD19Point:
    angle_deg: float  # [0, 360)
    distance_m: float  # meters
    intensity: int  # 0-255


@dataclass
class LD19Scan:
    timestamp: float  # ROS time when assembled
    points: List[LD19Point] = field(default_factory=list)
    spin_speed_dps: float = 0.0


class LD19Driver:
    """Threaded driver that reads LD19 packets and emits complete scans."""

    PACKET_SIZE = 47
    HEADER_BYTE = 0x54
    VER_LEN_BYTE = 0x2C
    POINTS_PER_PACKET = 12
    SCAN_TIMEOUT_S = 0.15  # emit scan if no new packet for this long

    # Correct CRC-8 table for LD19 (LudovaTech reference)
    CRC8_TABLE = bytes([
        0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae,
        0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
        0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
        0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
        0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1,
        0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
        0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18,
        0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea,
        0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
        0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62,
        0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39,
        0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
        0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f,
        0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d,
        0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
        0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4,
        0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2,
        0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
        0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b,
        0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89,
        0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
        0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f,
        0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64,
        0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
        0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec,
        0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e,
        0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
        0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
        0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3,
        0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
        0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a,
        0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8,
    ])

    def __init__(
        self,
        port: str = "/dev/ttyUSB1",
        baudrate: int = 230400,
        angle_offset_deg: float = 0.0,
                 distance_scale: float = 1.0,
        queue_size: int = 2,
    ) -> None:
        self._port = port
        self._baudrate = baudrate
        self._angle_offset_deg = angle_offset_deg
        self._distance_scale = distance_scale
        self._serial: Optional[serial.Serial] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._scan_queue: Deque[LD19Scan] = Deque(maxlen=queue_size)
        self._queue_lock = threading.Lock()
        self._crc_passed = 0
        self._crc_failed = 0

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        try:
            self._serial = serial.Serial(
                self._port,
                self._baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.01,
                write_timeout=0.5,
            )
            # Flush stale data
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()
            time.sleep(0.1)
            return True
        except Exception as exc:
            print(f"[LD19] Failed to open {self._port}: {exc}")
            return False

    def start(self) -> None:
        if not self._serial or not self._serial.is_open:
            if not self.connect():
                return
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def is_running(self) -> bool:
        return self._running and self._thread is not None and self._thread.is_alive()

    def _reconnect(self) -> None:
        """Close and reopen the serial port after an error."""
        try:
            if self._serial and self._serial.is_open:
                self._serial.close()
        except Exception:
            pass
        self._serial = None
        time.sleep(0.5)
        for attempt in range(5):
            if self.connect():
                print(f"[LD19] Reconnected on attempt {attempt + 1}")
                return
            time.sleep(0.5 + attempt * 0.5)
        print("[LD19] Reconnect failed after 5 attempts")

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._serial and self._serial.is_open:
            try:
                self._serial.close()
            except Exception:
                pass

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def get_scan(self, block: bool = False, timeout: Optional[float] = None) -> Optional[LD19Scan]:
        with self._queue_lock:
            if self._scan_queue:
                return self._scan_queue.popleft()
        if not block:
            return None
        # Simple blocking wait
        deadline = time.time() + (timeout or 1.0)
        while time.time() < deadline and self._running:
            with self._queue_lock:
                if self._scan_queue:
                    return self._scan_queue.popleft()
            time.sleep(0.005)
        return None

    def stats(self) -> dict:
        return {
            "crc_passed": self._crc_passed,
            "crc_failed": self._crc_failed,
            "queue_len": len(self._scan_queue),
        }

    # ------------------------------------------------------------------
    # Internal read loop
    # ------------------------------------------------------------------

    def _read_loop(self) -> None:
        buf = bytearray()
        current_points: List[LD19Point] = []
        current_speed_dps = 0.0
        last_start_angle: Optional[int] = None
        packets_since_emit = 0

        while self._running:
            try:
                # Read bytes directly with short timeout.
                # Avoid in_waiting/read race that causes "readiness but no data".
                if self._serial and self._serial.is_open:
                    try:
                        chunk = self._serial.read(4096)
                    except serial.SerialException:
                        chunk = b''
                    if chunk:
                        buf.extend(chunk)

                # Process complete packets from buffer
                packets_processed = 0
                while True:
                    idx = buf.find(self.HEADER_BYTE)
                    if idx == -1:
                        buf.clear()
                        break
                    if len(buf) - idx < self.PACKET_SIZE:
                        # Keep the partial packet at the front
                        buf = buf[idx:]
                        break

                    # Verify verlen byte before even copying
                    if buf[idx + 1] != self.VER_LEN_BYTE:
                        # Misaligned — discard just the false header
                        del buf[idx]
                        continue

                    packet = bytes(buf[idx:idx + self.PACKET_SIZE])

                    if not self._validate_packet(packet):
                        del buf[idx]
                        self._crc_failed += 1
                        continue

                    self._crc_passed += 1
                    points, speed_dps, start_angle_raw = self._parse_packet(packet)
                    packets_processed += 1
                    packets_since_emit += 1

                    # Detect scan boundary.
                    # LD19 emits ~45-50 packets per full rotation (≈600 pts).
                    # The angle-based wrap detection doesn't work for contiguous
                    # packets (delta is always small), so we just count packets.
                    scan_boundary = False
                    if packets_since_emit >= 50 and current_points:
                        scan_boundary = True

                    if scan_boundary and current_points:
                        self._emit_scan(current_points, current_speed_dps)
                        current_points = []
                        packets_since_emit = 0

                    current_points.extend(points)
                    current_speed_dps = speed_dps
                    del buf[:idx + self.PACKET_SIZE]

                if not packets_processed:
                    time.sleep(0.002)

            except serial.SerialException as exc:
                print(f"[LD19] Serial error: {exc} — attempting reconnect")
                self._reconnect()
            except Exception as exc:
                print(f"[LD19] Read loop error: {exc}")
                time.sleep(0.05)

        # Emit any remaining points on shutdown
        if current_points:
            self._emit_scan(current_points, current_speed_dps)

    # ------------------------------------------------------------------
    # Packet validation & parsing
    # ------------------------------------------------------------------

    def _validate_packet(self, packet: bytes) -> bool:
        """CRC-8 over the first 46 bytes (all except CRC byte itself).

        The reference pre-computes crc=0xD8 for header(0x54)+verlen(0x2C),
        then processes the remaining 45 bytes (speed..timestamp).
        We just CRC all 46 bytes which is mathematically equivalent.
        """
        expected_crc = packet[46]
        crc = 0
        for b in packet[:-1]:
            crc = self.CRC8_TABLE[crc ^ b]
        return crc == expected_crc

    def _parse_packet(self, packet: bytes):
        """Unpack a 47-byte packet into 12 points.

        Returns (points, speed_dps, start_angle_raw).
        """
        # Speed: bytes 2-3, LSB first, unit: degrees per second
        speed = struct.unpack_from("<H", packet, 2)[0]
        speed_dps = float(speed)

        # Start / end angles: bytes 4-5 and 42-43, LSB first, 0.01° units
        start_angle_raw = struct.unpack_from("<H", packet, 4)[0]
        end_angle_raw = struct.unpack_from("<H", packet, 42)[0]

        # Angle step between the 12 points (integer 0.01° units)
        if start_angle_raw <= end_angle_raw:
            step_raw = (end_angle_raw - start_angle_raw) // 11
        else:
            step_raw = (36000 + end_angle_raw - start_angle_raw) // 11

        points: List[LD19Point] = []
        offset = 6  # data starts at byte 6
        for i in range(self.POINTS_PER_PACKET):
            distance_mm = struct.unpack_from("<H", packet, offset)[0]
            intensity = packet[offset + 2]
            offset += 3

            # Angle in 0.01° units, modulo 36000
            angle_raw = (start_angle_raw + step_raw * i) % 36000
            angle_deg = angle_raw * 0.01 + self._angle_offset_deg
            angle_deg = angle_deg % 360.0

            points.append(LD19Point(
                angle_deg=angle_deg,
                distance_m=distance_mm / 1000.0 * self._distance_scale,
                intensity=intensity,
            ))

        return points, speed_dps, start_angle_raw

    # ------------------------------------------------------------------
    # Scan assembly
    # ------------------------------------------------------------------

    def _emit_scan(self, points: List[LD19Point], speed_dps: float) -> None:
        scan = LD19Scan(
            timestamp=time.time(),
            points=points,
            spin_speed_dps=speed_dps,
        )
        with self._queue_lock:
            self._scan_queue.append(scan)

#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (C) ARDUINO SRL
#
# SPDX-License-Identifier: MPL-2.0

"""
Yahboom IMU-Sensor driver (Yahboom serial protocol).

Protocol (from Yahboom documentation):
    Header:     0x7E 0x23
    Length:     total frame length in bytes
    Function:   packet type
    Data:       payload
    Checksum:   sum of all preceding bytes, lowest byte

Automatic output packets (cycle at configured Hz, default 25):
    0x04 — Raw sensor data (len=0x17=23)
           accel(x,y,z) + gyro(x,y,z) + mag(x,y,z) as int16 little-endian
           scale: accel=16/32767 g, gyro=(2000/32767)*(pi/180) rad/s, mag=800/32767
    0x16 — Quaternion (len=0x15=21)
           q0,q1,q2,q3 as IEEE-754 float32 little-endian
    0x26 — Euler angles (len=0x11=17)
           roll, pitch, yaw as IEEE-754 float32 little-endian (radians)
    0x32 — Barometer (len=0x15=21)  [10-axis only]
           height, temperature, pressure, pressure_contrast as float32
"""

import math
import struct
import threading
import time
from typing import Optional, Tuple

try:
    import serial
except ImportError:
    serial = None  # type: ignore


class YahboomIMUDriver:
    """
    Thread-safe driver for Yahboom IMU-Sensor via serial (USB/UART).
    """

    # Function codes
    FUNC_RAW = 0x04
    FUNC_QUAT = 0x16
    FUNC_EULER = 0x26
    FUNC_BARO = 0x32

    def __init__(
        self,
        port: str = '/dev/ttyCH341USB0',
        baudrate: int = 115200,
        timeout: float = 0.05,
    ) -> None:
        self._port = port
        self._baudrate = baudrate
        self._timeout = timeout

        self._serial: Optional[serial.Serial] = None
        self._serial_lock = threading.Lock()
        self._running = False
        self._read_thread: Optional[threading.Thread] = None

        # Latest parsed data (protected by _data_lock)
        self._data_lock = threading.Lock()
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._q0 = 1.0
        self._q1 = 0.0
        self._q2 = 0.0
        self._q3 = 0.0
        self._ax = 0.0
        self._ay = 0.0
        self._az = 0.0
        self._gx = 0.0
        self._gy = 0.0
        self._gz = 0.0
        self._mx = 0.0
        self._my = 0.0
        self._mz = 0.0
        self._altitude = 0.0
        self._temperature = 0.0
        self._pressure = 0.0
        self._last_data_time = 0.0

        # Stats
        self._packets_parsed = 0
        self._packets_bad = 0

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        """Open serial port and start reader thread."""
        if serial is None:
            raise RuntimeError("pyserial is not installed")

        try:
            self._serial = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                timeout=self._timeout,
            )
            self._serial.reset_input_buffer()
        except serial.SerialException as e:
            raise RuntimeError(f"Failed to open IMU port {self._port}: {e}")

        self._running = True
        self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._read_thread.start()
        return True

    def disconnect(self) -> None:
        self._running = False
        if self._read_thread is not None:
            self._read_thread.join(timeout=1.0)
            self._read_thread = None
        if self._serial is not None and self._serial.is_open:
            self._serial.close()
            self._serial = None

    def is_connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    # ------------------------------------------------------------------
    # Accessors
    # ------------------------------------------------------------------

    def get_yaw(self) -> Optional[float]:
        """Return latest yaw in radians, or None if no data yet."""
        with self._data_lock:
            if self._last_data_time == 0.0:
                return None
            return self._yaw

    def get_angles(self) -> Optional[Tuple[float, float, float]]:
        """Return (roll, pitch, yaw) in radians, or None."""
        with self._data_lock:
            if self._last_data_time == 0.0:
                return None
            return (self._roll, self._pitch, self._yaw)

    def get_quaternion(self) -> Optional[Tuple[float, float, float, float]]:
        """Return (q0, q1, q2, q3), or None."""
        with self._data_lock:
            if self._last_data_time == 0.0:
                return None
            return (self._q0, self._q1, self._q2, self._q3)

    def get_mag(self) -> Optional[Tuple[float, float, float]]:
        """Return (mx, my, mz), or None."""
        with self._data_lock:
            if self._last_data_time == 0.0:
                return None
            return (self._mx, self._my, self._mz)

    def get_gyro(self) -> Optional[Tuple[float, float, float]]:
        """Return (gx, gy, gz) in rad/s, or None."""
        with self._data_lock:
            if self._last_data_time == 0.0:
                return None
            return (self._gx, self._gy, self._gz)

    def get_accel(self) -> Optional[Tuple[float, float, float]]:
        """Return (ax, ay, az) in g, or None."""
        with self._data_lock:
            if self._last_data_time == 0.0:
                return None
            return (self._ax, self._ay, self._az)

    def get_pressure_altitude(self) -> Optional[Tuple[float, float, float, float]]:
        """Return (altitude_m, temp_C, pressure_Pa, pressure_ref_Pa), or None."""
        with self._data_lock:
            if self._last_data_time == 0.0:
                return None
            return (self._altitude, self._temperature, self._pressure, 0.0)

    def stats(self) -> Tuple[int, int]:
        """Return (packets_parsed, packets_bad)."""
        return (self._packets_parsed, self._packets_bad)

    # ------------------------------------------------------------------
    # Internal: serial read loop
    # ------------------------------------------------------------------

    def _read_loop(self) -> None:
        """Background thread: sync to 0x7E 0x23 header and parse variable-length frames."""
        buf = bytearray()
        while self._running and self._serial is not None and self._serial.is_open:
            try:
                chunk = self._serial.read(max(1, self._serial.in_waiting or 1))
                if chunk:
                    buf.extend(chunk)
            except Exception:
                time.sleep(0.01)
                continue

            # Parse all complete frames in buffer
            i = 0
            while i < len(buf) - 1:
                if buf[i] == 0x7E and buf[i + 1] == 0x23:
                    if i + 2 < len(buf):
                        length = buf[i + 2]
                        if length < 3 or length > 64:
                            # Invalid length, skip this false header
                            i += 1
                            continue
                        if i + length <= len(buf):
                            packet = bytes(buf[i:i + length])
                            if self._verify_checksum(packet):
                                self._parse_packet(packet)
                                self._packets_parsed += 1
                            else:
                                self._packets_bad += 1
                            i += length
                            continue
                        else:
                            # Need more data
                            break
                    else:
                        # Need length byte
                        break
                i += 1

            # Trim processed bytes, keep unprocessed tail
            if i > 0:
                buf = buf[i:]

            # Prevent unbounded growth
            if len(buf) > 512:
                buf = buf[-256:]

            time.sleep(0.001)

    @staticmethod
    def _verify_checksum(packet: bytes) -> bool:
        """Checksum = sum of all bytes except the last, take lowest byte."""
        if len(packet) < 3:
            return False
        expected = sum(packet[:-1]) & 0xFF
        return expected == packet[-1]

    def _parse_packet(self, data: bytes) -> None:
        """Parse a verified Yahboom frame."""
        func = data[3]
        payload = data[4:-1]

        with self._data_lock:
            self._last_data_time = time.time()

            if func == self.FUNC_EULER and len(payload) >= 12:
                self._roll = struct.unpack('<f', payload[0:4])[0]
                self._pitch = struct.unpack('<f', payload[4:8])[0]
                self._yaw = struct.unpack('<f', payload[8:12])[0]

            elif func == self.FUNC_QUAT and len(payload) >= 16:
                self._q0 = struct.unpack('<f', payload[0:4])[0]
                self._q1 = struct.unpack('<f', payload[4:8])[0]
                self._q2 = struct.unpack('<f', payload[8:12])[0]
                self._q3 = struct.unpack('<f', payload[12:16])[0]

            elif func == self.FUNC_RAW and len(payload) >= 18:
                # int16 little-endian
                vals = []
                for j in range(9):
                    v = struct.unpack('<h', payload[j * 2:j * 2 + 2])[0]
                    vals.append(v)
                # Scaling factors from Yahboom docs
                self._ax = vals[0] * 16.0 / 32767.0
                self._ay = vals[1] * 16.0 / 32767.0
                self._az = vals[2] * 16.0 / 32767.0
                self._gx = vals[3] * 2000.0 / 32767.0 * math.pi / 180.0
                self._gy = vals[4] * 2000.0 / 32767.0 * math.pi / 180.0
                self._gz = vals[5] * 2000.0 / 32767.0 * math.pi / 180.0
                self._mx = vals[6] * 800.0 / 32767.0
                self._my = vals[7] * 800.0 / 32767.0
                self._mz = vals[8] * 800.0 / 32767.0

            elif func == self.FUNC_BARO and len(payload) >= 16:
                self._altitude = struct.unpack('<f', payload[0:4])[0]
                self._temperature = struct.unpack('<f', payload[4:8])[0]
                self._pressure = struct.unpack('<f', payload[8:12])[0]
                # pressure_contrast at bytes 12:16, ignored for now

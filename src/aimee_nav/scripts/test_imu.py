#!/usr/bin/env python3
"""
Diagnostic script for Yahboom IMU-Sensor.

Scans common serial ports, tries to parse WitMotion frames,
and prints live yaw/angle data. Use this to verify wiring,
find the correct port, and check magnetometer health.

Usage:
    python3 test_imu.py /dev/ttyUSB2
    python3 test_imu.py          # auto-scan
"""

import sys
import time

# Allow importing from the package directly
sys.path.insert(0, '/home/arduino/aimee-robot-ws/src/aimee_nav')
from aimee_nav.yahboom_imu_driver import YahboomIMUDriver


PORTS = ['/dev/ttyUSB2', '/dev/ttyUSB3', '/dev/ttyACM0', '/dev/ttyACM1']


def main() -> None:
    port = sys.argv[1] if len(sys.argv) > 1 else None

    if port is None:
        print("Auto-scanning for Yahboom IMU...")
        for p in PORTS:
            print(f"  Trying {p} ...", end=' ', flush=True)
            imu = YahboomIMUDriver(port=p, baudrate=115200)
            try:
                imu.connect()
                time.sleep(1.5)  # Wait for a few packets
                yaw = imu.get_yaw()
                if yaw is not None:
                    print(f"SUCCESS  (yaw={yaw:.1f} rad)")
                    port = p
                    imu.disconnect()
                    break
                else:
                    print("no data")
            except Exception as e:
                print(f"failed ({e})")
            finally:
                try:
                    imu.disconnect()
                except Exception:
                    pass
        if port is None:
            print("No IMU found on scanned ports. Plug it in and try again.")
            sys.exit(1)

    print(f"\nReading IMU from {port} ... Press Ctrl+C to stop.\n")
    imu = YahboomIMUDriver(port=port, baudrate=115200)
    imu.connect()

    try:
        while True:
            time.sleep(0.2)
            angles = imu.get_angles()
            mag = imu.get_mag()
            stats = imu.stats()
            if angles:
                roll, pitch, yaw = angles
                line = (
                    f"Roll={roll:7.2f}°  Pitch={pitch:7.2f}°  Yaw={yaw:7.2f}°  "
                    f"(pkt_ok={stats[0]} pkt_bad={stats[1]})"
                )
                if mag:
                    line += f"  Mag=({mag[0]:.0f}, {mag[1]:.0f}, {mag[2]:.0f})"
                print(line)
            else:
                print("Waiting for data...")
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        imu.disconnect()


if __name__ == '__main__':
    main()

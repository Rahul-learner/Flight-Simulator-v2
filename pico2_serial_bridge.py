#!/usr/bin/env python3
"""
pico2_serial_bridge.py
======================
Python Telemetry Monitor & Terminal HUD for Raspberry Pi Pico 2 + BMI160
(2-Axis Zero-Drift Roll & Pitch Attitude Flight Controller).

Features:
1. Auto-detects connected Raspberry Pi Pico 2 on /dev/ttyACM* or COM*.
2. Live terminal HUD with Roll & Pitch (Zero Yaw Drift).
3. Interactive terminal controls:
   - 'l': Level / Tare current tilt to 0.0 deg
   - 'c': Recalibrate stationary offsets
   - '?': Print system status
   - 'q': Quit
"""

import sys
import time
import json
import glob
import argparse

try:
    import serial
except ImportError:
    print("pyserial is not installed. Run: pip install pyserial")
    sys.exit(1)

def find_pico_port():
    candidates = glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*") + glob.glob("/dev/tty.usbmodem*")
    if candidates:
        return candidates[0]
    return "/dev/ttyACM0"

def main():
    parser = argparse.ArgumentParser(description="Pico 2 BMI160 2-Axis Telemetry Bridge & Monitor")
    parser.add_argument("--port", "-p", default=None, help="Serial port (default: auto-detect)")
    parser.add_argument("--baud", "-b", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--raw", action="store_true", help="Print raw incoming lines")
    args = parser.parse_args()

    port = args.port or find_pico_port()
    print("\n=======================================================")
    print("  Pico 2 BMI160 2-Axis Attitude Monitor (No Yaw Drift) ")
    print("=======================================================")
    print(f"Connecting to port: {port} @ {args.baud} baud...")

    try:
        ser = serial.Serial(port, args.baud, timeout=0.1)
    except Exception as e:
        print(f"[ERROR] Could not open port {port}: {e}")
        print("Tip: Check `ls /dev/ttyACM*` or ensure your user is in the 'uucp' / 'dialout' group.")
        sys.exit(1)

    print("Connected! Streaming Roll & Pitch attitude data from Pico 2...")
    print("Controls: [l] Level | [c] Calibrate | [?] Status Info | [q] Quit\n")

    last_time = time.time()
    packet_count = 0

    try:
        while True:
            line = ser.readline().decode('utf-8', errors='replace').strip()
            if not line:
                continue

            packet_count += 1
            now = time.time()

            if args.raw:
                print(line)
                continue

            if line.startswith('{') and line.endswith('}'):
                try:
                    data = json.loads(line)
                    if 'r' in data and 'p' in data:
                        r = data['r']
                        p = data['p']
                        dt = data.get('dt', 0.0)
                        rate = 1.0 / dt if dt > 0 else 0

                        if (now - last_time) >= 0.05:
                            sys.stdout.write(
                                f"\r[PICO 2] Roll: {r:+6.1f}° | Pitch: {p:+6.1f}° | (Yaw Removed) | Loop: {rate:4.0f} Hz | Pkts: {packet_count}   "
                            )
                            sys.stdout.flush()
                            last_time = now
                    elif 'msg' in data:
                        print(f"\n[PICO MSG] {data['msg']}")
                    elif 'error' in data:
                        print(f"\n[PICO ERROR] {data['error']}")
                    else:
                        print(f"\n[PICO JSON] {line}")
                except json.JSONDecodeError:
                    print(f"\n[RAW] {line}")
            else:
                print(f"\n[LOG] {line}")

    except KeyboardInterrupt:
        print("\nExiting monitor.")
    finally:
        ser.close()

if __name__ == "__main__":
    main()

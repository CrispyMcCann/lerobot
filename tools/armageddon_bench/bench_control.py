#!/usr/bin/env python3
"""
ARMageddon Bench Test — Python Host Controller

Interactive CLI for controlling motors via the Teensy 4.1 firmware.
Sends text commands over USB serial, receives JSON responses.

Usage:
    python bench_control.py                    # auto-detect Teensy
    python bench_control.py --port /dev/ttyACM0
    python bench_control.py --port COM3        # Windows

Commands (type at the prompt):
    move j1 90        Move joint 1 to 90 degrees
    read j1           Read joint 1 position (+ load encoder if available)
    readall           Read all motors
    state j1 8        Set ODrive to closed-loop control (8) or idle (1)
    mode j1 torque    Set ODrive controller mode (pos | vel | torque)
    torque j1 0.05    Send torque command in Nm (requires mode torque + state 8)
    list              List registered motors
    ping              Health check
    help              Show all commands
    log start         Start logging positions to CSV
    log stop          Stop logging
    quit / exit       Exit the program

Requirements:
    pip install pyserial
"""

import argparse
import csv
import json
import os
import sys
import time
from datetime import datetime

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("Error: pyserial not installed. Run: pip install pyserial")
    sys.exit(1)


def find_teensy_port():
    """Auto-detect Teensy 4.1 USB serial port."""
    ports = serial.tools.list_ports.comports()
    for p in ports:
        # Teensy 4.1 USB vendor/product IDs
        if p.vid == 0x16C0 and p.pid == 0x0483:
            return p.device
        # Also check description
        if p.description and "teensy" in p.description.lower():
            return p.device
    return None


class BenchController:
    def __init__(self, port, baudrate=115200, timeout=2.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.logging = False
        self.log_file = None
        self.log_writer = None

    def connect(self):
        """Open serial connection to Teensy."""
        print(f"Connecting to {self.port} at {self.baudrate} baud...")
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.timeout,
        )
        # Wait for Teensy to boot and send startup messages
        time.sleep(2.0)

        # Read and display startup messages
        while self.ser.in_waiting:
            line = self.ser.readline().decode("utf-8", errors="replace").strip()
            if line:
                self._print_response(line)

        print(f"Connected to Teensy on {self.port}")
        print("Type 'help' for available commands, 'quit' to exit.\n")

    def disconnect(self):
        """Close serial connection."""
        if self.log_file:
            self.log_file.close()
            self.log_file = None
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Disconnected.")

    def send_command(self, cmd, timeout=None):
        """Send a command and return the parsed JSON response."""
        if not self.ser or not self.ser.is_open:
            print("Error: not connected")
            return None

        # Flush input buffer
        self.ser.reset_input_buffer()

        # Send command
        self.ser.write((cmd.strip() + "\n").encode("utf-8"))

        # Read response (may be multiple lines for HELP)
        response_lines = []
        deadline = time.time() + (timeout or self.timeout)
        while time.time() < deadline:
            if self.ser.in_waiting:
                line = self.ser.readline().decode("utf-8", errors="replace").strip()
                if line:
                    response_lines.append(line)
                    # Try to parse as JSON — if valid, we have our response
                    try:
                        data = json.loads(line)
                        return data
                    except json.JSONDecodeError:
                        pass
            else:
                time.sleep(0.01)

        # If we got lines but couldn't parse JSON, return raw
        if response_lines:
            return {"ok": True, "raw": "\n".join(response_lines)}

        return {"ok": False, "error": "timeout waiting for response"}

    def _print_response(self, line):
        """Pretty-print a response line."""
        try:
            data = json.loads(line)
            if data.get("ok"):
                if "position" in data:
                    motor = data.get("motor", "?")
                    pos = data["position"]
                    msg = f"  {motor}: {pos:.2f} deg"
                    if "encoder" in data:
                        msg += f"  (load encoder: {data['encoder']:.2f} deg)"
                    print(msg)
                elif "motors" in data and isinstance(data["motors"], dict):
                    print("  All positions:")
                    for name, pos in data["motors"].items():
                        print(f"    {name}: {pos:.2f} deg")
                elif "motors" in data and isinstance(data["motors"], list):
                    print("  Registered motors:")
                    for m in data["motors"]:
                        print(f"    {m['name']:10s}  type={m['type']:10s}  "
                              f"bus_id={m['bus_id']}  gear={m['gear_ratio']}")
                elif "torque" in data:
                    motor = data.get("motor", "?")
                    print(f"  {motor}: torque={data['torque']:.4f} Nm")
                elif "mode" in data:
                    motor = data.get("motor", "?")
                    print(f"  {motor}: controller mode = {data['mode']}")
                elif "state" in data:
                    states = {1: "IDLE", 8: "CLOSED_LOOP", 3: "CALIBRATION"}
                    state_name = states.get(data["state"], str(data["state"]))
                    print(f"  {data.get('motor', '?')}: state={state_name}  "
                          f"error={data.get('error', 0)}")
                elif "msg" in data:
                    print(f"  {data['msg']}")
                elif "servo_scan" in data:
                    results = data["servo_scan"]
                    if results:
                        print(f"  Found {len(results)} servo(s):")
                        for r in results:
                            print(f"    ID={r['id']}  baud={r['baud']}")
                    else:
                        print("  No servos found at any baud rate")
                elif "scan" in data:
                    results = data["scan"]
                    print(f"  Scan found {len(results)} endpoints:")
                    for r in results:
                        print(f"    ep={r['ep']:4d}  val={r['val']:.6f}")
                elif "commands" in data:
                    print("  Available commands:")
                    for c in data["commands"]:
                        print(f"    {c}")
                else:
                    print(f"  {data}")
            else:
                print(f"  ERROR: {data.get('error', 'unknown error')}")
        except json.JSONDecodeError:
            print(f"  {line}")

    def handle_command(self, user_input):
        """Process a user command."""
        parts = user_input.strip().split()
        if not parts:
            return True  # continue

        cmd = parts[0].lower()

        # Local commands
        if cmd in ("quit", "exit", "q"):
            return False

        if cmd == "stop":
            # Emergency stop — send immediately, don't wait for normal response cycle
            if self.ser and self.ser.is_open:
                self.ser.reset_input_buffer()
                self.ser.write(b"STOP\n")
                self.ser.flush()
                time.sleep(0.1)
                # Read response
                while self.ser.in_waiting:
                    line = self.ser.readline().decode("utf-8", errors="replace").strip()
                    if line:
                        self._print_response(line)
            print("  ** All motors stopped **")
            return True

        if cmd == "log":
            if len(parts) >= 2 and parts[1].lower() == "start":
                self._start_logging()
            elif len(parts) >= 2 and parts[1].lower() == "stop":
                self._stop_logging()
            else:
                print("  Usage: log start | log stop")
            return True

        # Use longer timeout for scan commands
        if cmd in ("scan", "servoscan"):
            response = self.send_command(user_input, timeout=10.0)
        else:
            response = self.send_command(user_input)
        if response:
            self._print_response(json.dumps(response))

            # Log if active
            if self.logging and self.log_writer and "position" in response:
                self.log_writer.writerow({
                    "timestamp": datetime.now().isoformat(),
                    "motor": response.get("motor", ""),
                    "position": response.get("position", ""),
                    "encoder": response.get("encoder", ""),
                })

        return True

    def _start_logging(self):
        """Start logging positions to CSV."""
        filename = f"bench_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.log_file = open(filename, "w", newline="")
        self.log_writer = csv.DictWriter(
            self.log_file,
            fieldnames=["timestamp", "motor", "position", "encoder"],
        )
        self.log_writer.writeheader()
        self.logging = True
        print(f"  Logging to {filename}")

    def _stop_logging(self):
        """Stop logging."""
        if self.log_file:
            self.log_file.close()
            self.log_file = None
            self.log_writer = None
        self.logging = False
        print("  Logging stopped.")

    def run_repl(self):
        """Run the interactive command loop."""
        try:
            while True:
                try:
                    user_input = input("arm> ")
                except EOFError:
                    break

                if not self.handle_command(user_input):
                    break
        except KeyboardInterrupt:
            print("\nInterrupted.")
        finally:
            self.disconnect()


def main():
    parser = argparse.ArgumentParser(
        description="ARMageddon Bench Test Controller",
    )
    parser.add_argument(
        "--port",
        help="Serial port (e.g., /dev/ttyACM0, COM3). Auto-detects Teensy if omitted.",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Baud rate (default: 115200, ignored for Teensy USB)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="Response timeout in seconds (default: 2.0)",
    )
    args = parser.parse_args()

    # Find port
    port = args.port
    if not port:
        port = find_teensy_port()
        if not port:
            print("Error: Could not auto-detect Teensy 4.1.")
            print("Available ports:")
            for p in serial.tools.list_ports.comports():
                print(f"  {p.device}: {p.description} [{p.hwid}]")
            print("\nSpecify manually with --port")
            sys.exit(1)

    controller = BenchController(port, args.baud, args.timeout)

    try:
        controller.connect()
        controller.run_repl()
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()

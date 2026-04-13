#!/usr/bin/env python3
"""
Scan for STS3215 servos using feetech-servo-sdk directly.
Tries multiple baud rates and IDs 0-15.

Usage:
    python servo_scan.py                          # auto-detect Waveshare USB
    python servo_scan.py --port /dev/ttyACM1
    python servo_scan.py --port COM5              # Windows
"""

import argparse
import sys

try:
    import scservo_sdk as scs
except ImportError:
    print("Error: feetech-servo-sdk not installed. Run: pip install feetech-servo-sdk")
    sys.exit(1)

BAUD_RATES = [1_000_000, 500_000, 250_000, 115_200, 57_600, 38_400]
MAX_ID = 15  # scan IDs 0 through 15


def find_waveshare_port():
    """Try to find the Waveshare USB-serial adapter."""
    try:
        import serial.tools.list_ports
        for p in serial.tools.list_ports.comports():
            desc = (p.description or "").lower()
            if "ch340" in desc or "waveshare" in desc or "usb serial" in desc:
                return p.device
        # Fall back to any ACM device that isn't the Teensy
        for p in serial.tools.list_ports.comports():
            if "ACM" in p.device or "ttyUSB" in p.device:
                # Skip Teensy (VID 0x16C0)
                if p.vid == 0x16C0:
                    continue
                return p.device
    except ImportError:
        pass
    return None


def scan(port_name):
    port = scs.PortHandler(port_name)
    if not port.openPort():
        print(f"Failed to open {port_name}")
        sys.exit(1)

    # Protocol version 0 is what STS3215 uses
    pkt = scs.PacketHandler(0)

    found = []

    for baud in BAUD_RATES:
        port.setBaudRate(baud)
        print(f"Scanning at {baud} baud...", end="", flush=True)
        count = 0

        for servo_id in range(MAX_ID + 1):
            model, comm, error = pkt.ping(port, servo_id)
            if comm == scs.COMM_SUCCESS:
                found.append((servo_id, baud, model))
                count += 1
                print(f" ID={servo_id}(model={model})", end="", flush=True)

        if count == 0:
            print(" none found")
        else:
            print()

    port.closePort()

    if found:
        print(f"\n=== Found {len(found)} servo(s) ===")
        for sid, baud, model in found:
            print(f"  ID={sid}  baud={baud}  model={model}")
    else:
        print("\nNo servos found at any baud rate.")
        print("Check:")
        print("  1. Servo power (red LED on Waveshare board?)")
        print("  2. Data wire connected to D on the Waveshare board")
        print("  3. USB cable from Waveshare B-port to computer")

    return found


def read_info(port_name, servo_id, baud):
    """Read key registers from a found servo."""
    port = scs.PortHandler(port_name)
    if not port.openPort():
        return
    port.setBaudRate(baud)
    pkt = scs.PacketHandler(0)

    print(f"\n--- Servo ID={servo_id} info ---")

    # STS3215 register addresses
    registers = {
        "Model Number": (3, 2),
        "Firmware Version": (6, 1),
        "ID": (5, 1),
        "Baud Rate": (4, 1),
        "Min Angle Limit": (9, 2),
        "Max Angle Limit": (11, 2),
        "Mode": (33, 1),
        "Present Position": (56, 2),
        "Present Speed": (58, 2),
        "Present Load": (60, 2),
        "Present Voltage": (62, 1),
        "Present Temperature": (63, 1),
    }

    for name, (addr, size) in registers.items():
        if size == 1:
            val, comm, err = pkt.read1ByteTxRx(port, servo_id, addr)
        else:
            val, comm, err = pkt.read2ByteTxRx(port, servo_id, addr)

        if comm == scs.COMM_SUCCESS:
            extra = ""
            if name == "Present Position":
                deg = val * 360.0 / 4096.0
                extra = f"  ({deg:.1f} deg)"
            elif name == "Baud Rate":
                baud_map = {0: 1000000, 1: 500000, 2: 250000, 3: 128000,
                            4: 115200, 5: 76800, 6: 57600, 7: 38400}
                extra = f"  ({baud_map.get(val, '?')} bps)"
            elif name == "Present Voltage":
                extra = f"  ({val / 10.0:.1f}V)"
            elif name == "Present Temperature":
                extra = f"  ({val}°C)"
            elif name == "Mode":
                modes = {0: "position", 1: "velocity", 2: "PWM", 3: "step"}
                extra = f"  ({modes.get(val, 'unknown')})"
            print(f"  {name:25s} = {val}{extra}")
        else:
            print(f"  {name:25s} = READ ERROR ({pkt.getTxRxResult(comm)})")

    port.closePort()


def main():
    parser = argparse.ArgumentParser(description="Scan for Feetech STS servos")
    parser.add_argument("--port", help="Serial port (auto-detects if omitted)")
    args = parser.parse_args()

    port_name = args.port
    if not port_name:
        port_name = find_waveshare_port()
        if not port_name:
            print("Could not auto-detect Waveshare adapter.")
            try:
                import serial.tools.list_ports
                print("Available ports:")
                for p in serial.tools.list_ports.comports():
                    print(f"  {p.device}: {p.description} [{p.hwid}]")
            except ImportError:
                pass
            print("\nSpecify with --port")
            sys.exit(1)

    print(f"Using port: {port_name}")
    results = scan(port_name)

    # If we found servos, read their details
    for sid, baud, model in results:
        read_info(port_name, sid, baud)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Configure a Feetech STS3215 servo with persistent EEPROM settings.

Process:
1. Find the servo (scan for it)
2. Disable torque
3. Unlock EEPROM
4. Write new ID, mode, angle limits
5. Lock EEPROM
6. Verify by reading back
7. Soft reboot the servo
8. Verify settings survived

Usage:
    python servo_configure.py <new_id>
    e.g., python servo_configure.py 1
"""

import sys
import time
import scservo_sdk as scs

PORT = "/dev/ttyACM0"
BAUD = 1000000

# Register addresses (STS3215)
# STS3215 memory table (verified against official Feetech SMS_STS SDK)
# NOTE: SCS-series servos have Lock at 48. STS/SMS-series have Torque_Limit at 48
# and Lock at 55. Using 48 silently clobbers torque limit without unlocking EEPROM.
REG_ID = 5
REG_BAUD = 6
REG_MIN_ANGLE = 9    # 2 bytes (9, 10)
REG_MAX_ANGLE = 11   # 2 bytes (11, 12)
REG_MODE = 33
REG_TORQUE_ENABLE = 40
REG_LOCK = 55        # was 48 — that's Torque_Limit, not Lock!


def find_servo(port, pkt):
    """Find any servo on the bus, return its current ID."""
    for sid in range(16):
        model, comm, _ = pkt.ping(port, sid)
        if comm == scs.COMM_SUCCESS:
            return sid
    return None


def check_write(pkt, comm, err, label):
    if comm != scs.COMM_SUCCESS:
        print(f"  !! COMM FAIL on {label}: {pkt.getTxRxResult(comm)}")
        return False
    if err != 0:
        print(f"  !! SERVO ERROR on {label}: {pkt.getRxPacketError(err)}")
        return False
    return True


def configure(new_id):
    port = scs.PortHandler(PORT)
    if not port.openPort():
        print(f"Failed to open {PORT}")
        return False
    port.setBaudRate(BAUD)
    pkt = scs.PacketHandler(0)

    # Find servo
    print("Scanning for servo...")
    cur_id = find_servo(port, pkt)
    if cur_id is None:
        print("No servo found!")
        port.closePort()
        return False
    print(f"  Found servo at ID={cur_id}")

    # 1. Disable torque (RAM register)
    comm, err = pkt.write1ByteTxRx(port, cur_id, REG_TORQUE_ENABLE, 0)
    check_write(pkt, comm, err, "torque disable")
    time.sleep(0.3)
    print("  Torque disabled")

    # 2. Unlock EEPROM — write 0 to register 48
    comm, err = pkt.write1ByteTxRx(port, cur_id, REG_LOCK, 0)
    if not check_write(pkt, comm, err, "EEPROM unlock"):
        port.closePort()
        return False
    time.sleep(0.5)

    # Verify unlock took effect
    lock_val, _, _ = pkt.read1ByteTxRx(port, cur_id, REG_LOCK)
    print(f"  EEPROM lock register after unlock = {lock_val} (should be 0)")
    if lock_val != 0:
        print("  !! UNLOCK FAILED — EEPROM writes will not persist")
        port.closePort()
        return False

    # 3. Write new ID (EEPROM)
    if cur_id != new_id:
        comm, err = pkt.write1ByteTxRx(port, cur_id, REG_ID, new_id)
        if not check_write(pkt, comm, err, "ID write"):
            port.closePort()
            return False
        print(f"  Wrote ID={new_id}")
        time.sleep(1.0)  # give flash time to commit

        # Immediate readback at new ID
        id_now, _, _ = pkt.read1ByteTxRx(port, new_id, REG_ID)
        print(f"  Immediate readback: ID={id_now}")
        if id_now != new_id:
            print("  !! ID write did not take effect")
            port.closePort()
            return False

    # 4. Mode = position (EEPROM)
    comm, err = pkt.write1ByteTxRx(port, new_id, REG_MODE, 0)
    check_write(pkt, comm, err, "mode")
    time.sleep(0.5)

    # 5. Angle limits (EEPROM)
    comm, err = pkt.write2ByteTxRx(port, new_id, REG_MIN_ANGLE, 0)
    check_write(pkt, comm, err, "min angle")
    time.sleep(0.5)
    comm, err = pkt.write2ByteTxRx(port, new_id, REG_MAX_ANGLE, 4095)
    check_write(pkt, comm, err, "max angle")
    time.sleep(0.5)
    print("  Mode + angle limits written")

    # 6. Long delay before locking — let flash commit
    print("  Waiting 2s for flash commit before lock...")
    time.sleep(2.0)

    # 7. Lock EEPROM
    comm, err = pkt.write1ByteTxRx(port, new_id, REG_LOCK, 1)
    check_write(pkt, comm, err, "EEPROM lock")
    time.sleep(0.5)

    # 8. Verify
    print("\nVerifying registers...")
    read_id, _, _ = pkt.read1ByteTxRx(port, new_id, REG_ID)
    read_mode, _, _ = pkt.read1ByteTxRx(port, new_id, REG_MODE)
    read_min, _, _ = pkt.read2ByteTxRx(port, new_id, REG_MIN_ANGLE)
    read_max, _, _ = pkt.read2ByteTxRx(port, new_id, REG_MAX_ANGLE)
    read_lock, _, _ = pkt.read1ByteTxRx(port, new_id, REG_LOCK)
    print(f"  ID={read_id}, mode={read_mode}, limits={read_min}-{read_max}, lock={read_lock}")

    # 9. Close the port then reopen — simulates a fresh connection (NOT a power cycle,
    # but catches any cases where stale RAM values would otherwise show up)
    port.closePort()
    time.sleep(0.5)

    port2 = scs.PortHandler(PORT)
    port2.openPort()
    port2.setBaudRate(BAUD)
    pkt2 = scs.PacketHandler(0)
    model, comm, _ = pkt2.ping(port2, new_id)
    if comm == scs.COMM_SUCCESS:
        print(f"  After reopen: servo ACKs at ID={new_id}  model={model}")
    else:
        print(f"  !! After reopen: servo NOT responding at ID={new_id}")
    port2.closePort()

    print("\n*** Configuration complete. ***")
    print("Now UNPLUG the Waveshare DC barrel jack, wait 5 seconds, plug back in,")
    print(f"and run: python3 servo_configure.py {new_id}")
    print(f"If the servo is found at ID={new_id}, EEPROM persisted.")
    print("If it's found at ID=8 (default), EEPROM did NOT persist.")
    return True


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python servo_configure.py <new_id>")
        sys.exit(1)
    new_id = int(sys.argv[1])
    if not configure(new_id):
        sys.exit(1)

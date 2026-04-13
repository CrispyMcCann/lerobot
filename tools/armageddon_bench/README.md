# ARMageddon Bench Test

Single-motor bench test for the ARMageddon leader arm. Controls an ODrive S1 + Eagle Power 8308 BLDC motor, reads an AS5048A load encoder, and drives an STS3215 servo — all through a Teensy 4.1.

## Quick Start

### 1. Flash the Teensy Firmware

Open `firmware/armageddon_bench.ino` in Arduino IDE (with Teensyduino installed) or set up a PlatformIO project.

**Required libraries** (install via Arduino Library Manager):
- `FlexCAN_T4` — included with Teensyduino
- `SCServo` — search "SCServo" or download from [GitHub](https://github.com/IS2511/SCServo)

**Board settings:**
- Board: Teensy 4.1
- USB Type: Serial
- CPU Speed: 600 MHz (default)

Flash to your Teensy 4.1.

### 2. Wire the Hardware

See [WIRING.md](WIRING.md) for pin assignments and diagrams.

### 3. Run the Python Controller

```bash
pip install pyserial
python bench_control.py
# or specify port:
python bench_control.py --port /dev/ttyACM0
```

### 4. Test Commands

```
arm> ping                    # verify connection
arm> list                    # show registered motors
arm> state j1 8              # enable ODrive closed-loop control
arm> move j1 90              # move motor to 90 degrees
arm> read j1                 # read position (+ load encoder)
arm> read j1_enc             # read load encoder directly
arm> move j4 180             # move STS servo to 180 degrees
arm> readall                 # read all positions
arm> state j1 1              # return ODrive to idle
arm> quit                    # exit
```

## Adding Motors

Edit `firmware/motor_registry.h` and add one line to `MOTOR_REGISTRY[]`:

```cpp
const MotorDef MOTOR_REGISTRY[] = {
    {"j1",      ODRIVE,        0,  1.0},
    {"j4",      STS_SERVO,     1,  1.0},
    {"j1_enc",  ENCODER_ONLY,  10, 1.0},
    // Add new motors here:
    {"j2",      ODRIVE,        1,  5.0},   // ODrive node 1, 5:1 belt ratio
    {"j5",      STS_SERVO,     2,  1.0},   // Another STS servo, ID 2
    {"j2_enc",  ENCODER_ONLY,  9,  1.0},   // Another encoder, CS pin 9
};
```

Re-flash the Teensy. The new motor is immediately available from the Python controller.

## File Structure

```
tools/armageddon_bench/
├── firmware/
│   ├── armageddon_bench.ino  — Main sketch (setup/loop)
│   ├── odrive_can.h/.cpp     — ODrive S1 CAN protocol driver
│   ├── as5048a_spi.h/.cpp    — AS5048A SPI encoder driver
│   ├── sts_servo.h/.cpp      — STS3215 UART servo driver
│   ├── motor_registry.h/.cpp — Motor registry + dispatch
│   └── serial_cmd.h/.cpp     — USB serial command parser
├── bench_control.py          — Python host controller
├── WIRING.md                 — Pin assignments and wiring diagram
└── README.md                 — This file
```

## ODrive Setup Reminder

The ODrive S1 must be pre-configured via USB (`odrivetool`) before CAN control works:
- Motor type, encoder type, and PID gains must be set
- CAN node ID must match the `bus_id` in the motor registry (default: 0)
- CAN baud rate must match the firmware (default: 250 kbps)
- The ODrive should be able to enter closed-loop control via USB before attempting CAN

## Troubleshooting

- **No response from ODrive**: Check CAN wiring, termination resistors, baud rate match
- **Encoder reads 0**: Check SPI wiring, verify CS pin matches motor registry
- **Servo doesn't move**: Check UART wiring, direction pin, servo ID, power supply (7V)
- **Python can't find Teensy**: Use `--port` flag, or check `ls /dev/ttyACM*`

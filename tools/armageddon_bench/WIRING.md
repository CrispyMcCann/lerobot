# ARMageddon Bench Test — Wiring Reference

## Teensy 4.1 Pin Assignments

### CAN Bus (ODrive S1) — via SN65HVD230 transceiver
| Teensy Pin | Function  | SN65HVD230 Pin | Notes                    |
|------------|-----------|----------------|--------------------------|
| 22         | CAN1_TX   | TXD            |                          |
| 23         | CAN1_RX   | RXD            |                          |
| 3.3V       | VCC       | VCC            | 3.3V supply              |
| GND        | GND       | GND            |                          |
| —          | CANH      | CANH           | To ODrive CANH           |
| —          | CANL      | CANL           | To ODrive CANL           |

**Important:** 120 ohm termination resistor between CANH and CANL at each end of the bus.

### SPI (AS5048A Encoder)
| Teensy Pin | Function | AS5048A Pin | Notes              |
|------------|----------|-------------|--------------------|
| 13         | SCK      | CLK         |                    |
| 11         | MOSI     | MOSI        |                    |
| 12         | MISO     | MISO        |                    |
| 10         | CS       | CSn         | Directly wired     |
| 3.3V       | VCC      | VDD         | 3.3V or 5V         |
| GND        | GND      | GND         |                    |

For additional encoders, use different CS pins (e.g., 9, 8, 7) and add entries to the motor registry.

### UART (STS3215 Servo) — Half-duplex
| Teensy Pin | Function   | Notes                              |
|------------|------------|------------------------------------|
| 8          | Serial2 TX | To STS3215 DATA via direction ctrl |
| 7          | Serial2 RX | From STS3215 DATA                  |
| 6          | DIR        | Direction control GPIO             |

**Half-duplex wiring options:**
- **Option A (recommended):** Use a tri-state buffer (74HC126) controlled by DIR pin
- **Option B:** Connect TX through a 100-ohm resistor to the servo DATA line, connect RX directly. DIR pin controls a MOSFET or buffer that disables TX when receiving.

### Power
| Rail   | Source          | Components                    |
|--------|-----------------|-------------------------------|
| 24V    | Battery/PSU     | ODrive S1 + 8308 motor        |
| 7V     | Buck converter  | STS3215 servo                 |
| 5V/3.3V| Teensy USB      | Teensy, SN65HVD230, AS5048A  |

## ODrive S1 CAN Wiring

| ODrive S1 Pin | Connection        |
|---------------|-------------------|
| CANH          | SN65HVD230 CANH   |
| CANL          | SN65HVD230 CANL   |
| GND           | Common ground     |

**CAN bus settings (must match firmware):**
- Baud rate: 250 kbps (ODrive default)
- Node ID: 0 (for J1 motor, configurable via odrivetool)

## Bench Test Setup Diagram

```
                    ┌──────────────────────────────┐
                    │         Teensy 4.1            │
USB to PC ────────▶│ USB                           │
                    │                               │
                    │ Pin 22 (CAN1_TX) ──┐          │
                    │ Pin 23 (CAN1_RX) ──┤          │
                    │                    │          │
                    │ Pin 13 (SCK)  ─────┤          │
                    │ Pin 11 (MOSI) ─────┤          │
                    │ Pin 12 (MISO) ─────┤          │
                    │ Pin 10 (CS)   ─────┤          │
                    │                    │          │
                    │ Pin 8 (TX2) ───────┤          │
                    │ Pin 7 (RX2) ───────┤          │
                    │ Pin 6 (DIR) ───────┤          │
                    └────────────────────┤──────────┘
                                         │
              ┌──────────────────────────┘
              │
    ┌─────────▼──────────┐
    │    SN65HVD230       │         ┌──────────────┐
    │  CAN Transceiver    │────────▶│  ODrive S1    │
    │  CANH ────── CANL   │  CAN    │  + 8308 Motor │
    └─────────────────────┘  Bus    └──────────────┘
              │
    ┌─────────▼──────────┐
    │    AS5048A          │
    │  SPI Encoder        │  (mounted on output shaft)
    └─────────────────────┘
              │
    ┌─────────▼──────────┐
    │    STS3215          │
    │  Feetech Servo      │  (half-duplex UART)
    └─────────────────────┘
```

#ifndef STS_SERVO_H
#define STS_SERVO_H

#include <Arduino.h>

// STS3215 protocol constants
#define STS_HEADER              0xFF
#define STS_BROADCAST_ID        0xFE

// Instruction set
#define STS_INST_PING           0x01
#define STS_INST_READ           0x02
#define STS_INST_WRITE          0x03
#define STS_INST_REG_WRITE      0x04
#define STS_INST_ACTION         0x05
#define STS_INST_SYNC_WRITE     0x83

// Key register addresses (STS3215 / SMS_STS series)
#define STS_REG_ID              0x05
#define STS_REG_BAUD_RATE       0x06
#define STS_REG_MIN_ANGLE       0x09
#define STS_REG_MAX_ANGLE       0x0B
#define STS_REG_TORQUE_ENABLE   0x28
#define STS_REG_GOAL_POSITION   0x2A
#define STS_REG_GOAL_TIME       0x2C
#define STS_REG_GOAL_SPEED      0x2E
#define STS_REG_LOCK            0x30
#define STS_REG_PRESENT_POS     0x38
#define STS_REG_PRESENT_SPEED   0x3A
#define STS_REG_PRESENT_LOAD    0x3C
#define STS_REG_PRESENT_VOLTAGE 0x3E
#define STS_REG_PRESENT_TEMP    0x3F
#define STS_REG_MODE            0x21

// Position range: 0-4095 for 0-360 degrees
#define STS_POS_MIN             0
#define STS_POS_MAX             4095
#define STS_POS_CENTER          2048

struct STSState {
    int16_t position;       // 0-4095
    int16_t speed;
    int16_t load;
    uint8_t voltage;        // 0.1V units
    uint8_t temperature;    // degrees C
    bool online;
    uint32_t last_read_ms;
};

// Initialize the STS servo bus
// serial_port: pointer to HardwareSerial (e.g., &Serial1)
// dir_pin: GPIO pin for TX/RX direction control (-1 if not used)
// baudrate: default 1000000
void sts_init(HardwareSerial* serial_port, int8_t dir_pin = -1,
              uint32_t baudrate = 1000000);

// Ping a servo, returns true if it responds
bool sts_ping(uint8_t id);

// Move servo to position (0-4095) over time_ms milliseconds
void sts_move(uint8_t id, uint16_t position, uint16_t time_ms = 0);

// Read current position (returns -1 on error)
int16_t sts_read_position(uint8_t id);

// Read current speed
int16_t sts_read_speed(uint8_t id);

// Enable/disable torque
void sts_set_torque(uint8_t id, bool enable);

// Get cached state
const STSState& sts_get_state(uint8_t id);

// Convert position (0-4095) to degrees (0-360)
inline float sts_pos_to_deg(int16_t pos) {
    return (float)pos / 4096.0f * 360.0f;
}

// Convert degrees (0-360) to position (0-4095)
inline uint16_t sts_deg_to_pos(float deg) {
    int32_t pos = (int32_t)(deg / 360.0f * 4096.0f);
    if (pos < 0) pos = 0;
    if (pos > 4095) pos = 4095;
    return (uint16_t)pos;
}

#endif // STS_SERVO_H

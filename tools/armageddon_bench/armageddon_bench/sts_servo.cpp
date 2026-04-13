#include "sts_servo.h"
#include <string.h>

static HardwareSerial* sts_serial = nullptr;
static int8_t sts_dir_pin = -1;

// State cache for up to 8 servos
#define STS_MAX_SERVOS 8
static STSState servo_states[STS_MAX_SERVOS];

// Direction control for half-duplex UART
static inline void set_tx_mode() {
    if (sts_dir_pin >= 0) {
        digitalWrite(sts_dir_pin, HIGH);
        delayMicroseconds(10);
    }
}

static inline void set_rx_mode() {
    if (sts_dir_pin >= 0) {
        // Wait for TX buffer to flush
        sts_serial->flush();
        delayMicroseconds(10);
        digitalWrite(sts_dir_pin, LOW);
    }
}

static uint8_t checksum(const uint8_t* buf, uint8_t len) {
    uint8_t sum = 0;
    for (uint8_t i = 2; i < len; i++) {  // skip 0xFF 0xFF header
        sum += buf[i];
    }
    return ~sum;
}

// Send a raw instruction packet
static void send_packet(uint8_t id, uint8_t instruction,
                        const uint8_t* params, uint8_t param_len) {
    uint8_t length = param_len + 2;  // instruction + params + checksum
    uint8_t packet[32];              // fixed buffer (max packet ~20 bytes)
    if (6 + param_len > sizeof(packet)) return;  // safety check

    packet[0] = STS_HEADER;
    packet[1] = STS_HEADER;
    packet[2] = id;
    packet[3] = length;
    packet[4] = instruction;
    if (params && param_len > 0) {
        memcpy(&packet[5], params, param_len);
    }
    packet[5 + param_len] = checksum(packet, 5 + param_len);

    set_tx_mode();
    sts_serial->write(packet, 6 + param_len);
    sts_serial->flush();
    set_rx_mode();
}

// Read response packet, returns number of parameter bytes (0 on timeout/error)
static uint8_t read_response(uint8_t* params, uint8_t max_params,
                             uint8_t* error_out, uint32_t timeout_ms = 50) {
    uint32_t start = millis();
    uint8_t buf[64];
    uint8_t idx = 0;

    // Wait for header bytes
    while (millis() - start < timeout_ms) {
        if (sts_serial->available()) {
            uint8_t b = sts_serial->read();
            buf[idx++] = b;
            if (idx >= 2 && buf[idx - 2] == STS_HEADER && buf[idx - 1] == STS_HEADER) {
                break;
            }
            if (idx >= sizeof(buf)) return 0;  // overflow
        }
    }

    // Read ID
    while (!sts_serial->available() && millis() - start < timeout_ms) {}
    if (millis() - start >= timeout_ms) return 0;
    uint8_t id = sts_serial->read();

    // Read length
    while (!sts_serial->available() && millis() - start < timeout_ms) {}
    if (millis() - start >= timeout_ms) return 0;
    uint8_t length = sts_serial->read();

    if (length < 2 || length > 32) return 0;

    // Read error + params + checksum
    uint8_t remaining = length;
    uint8_t resp[32];
    for (uint8_t i = 0; i < remaining && millis() - start < timeout_ms; ) {
        if (sts_serial->available()) {
            resp[i++] = sts_serial->read();
        }
    }

    if (error_out) *error_out = resp[0];

    uint8_t param_count = length - 2;  // subtract error byte and checksum
    if (param_count > max_params) param_count = max_params;
    if (params && param_count > 0) {
        memcpy(params, &resp[1], param_count);
    }

    return param_count;
}

void sts_init(HardwareSerial* serial_port, int8_t dir_pin, uint32_t baudrate) {
    sts_serial = serial_port;
    sts_dir_pin = dir_pin;

    if (sts_dir_pin >= 0) {
        pinMode(sts_dir_pin, OUTPUT);
        digitalWrite(sts_dir_pin, LOW);  // default RX mode
    }

    sts_serial->begin(baudrate);
    memset(servo_states, 0, sizeof(servo_states));
}

bool sts_ping(uint8_t id) {
    // Flush any leftover data
    while (sts_serial->available()) sts_serial->read();

    send_packet(id, STS_INST_PING, nullptr, 0);

    // Wait for echo of our own packet to pass (half-duplex)
    delay(2);
    while (sts_serial->available()) sts_serial->read();

    uint8_t error = 0xFF;  // default to error so timeout = no response
    uint8_t count = read_response(nullptr, 0, &error, 100);
    // Ping returns 0 params on success; we just check we got a response
    if (id < STS_MAX_SERVOS) {
        servo_states[id].online = (count > 0 || error == 0);
    }
    return (error == 0);
}

void sts_move(uint8_t id, uint16_t position, uint16_t time_ms) {
    // Write Goal Position (2 bytes) + Goal Time (2 bytes)
    // Start address: STS_REG_GOAL_POSITION (0x2A)
    uint8_t params[5];
    params[0] = STS_REG_GOAL_POSITION;     // start address
    params[1] = position & 0xFF;            // position low byte
    params[2] = (position >> 8) & 0xFF;     // position high byte
    params[3] = time_ms & 0xFF;             // time low byte
    params[4] = (time_ms >> 8) & 0xFF;      // time high byte

    send_packet(id, STS_INST_WRITE, params, 5);
    // No response expected for broadcast; for individual, we don't wait
}

int16_t sts_read_position(uint8_t id) {
    // Flush RX
    while (sts_serial->available()) sts_serial->read();

    // Read 2 bytes starting from STS_REG_PRESENT_POS (0x38)
    uint8_t params[2];
    params[0] = STS_REG_PRESENT_POS;  // start address
    params[1] = 2;                     // number of bytes to read

    send_packet(id, STS_INST_READ, params, 2);

    uint8_t resp[2];
    uint8_t error = 0;
    uint8_t count = read_response(resp, 2, &error, 50);

    if (count >= 2 && error == 0) {
        int16_t pos = resp[0] | ((int16_t)resp[1] << 8);
        if (id < STS_MAX_SERVOS) {
            servo_states[id].position = pos;
            servo_states[id].last_read_ms = millis();
        }
        return pos;
    }

    return -1;  // error
}

int16_t sts_read_speed(uint8_t id) {
    while (sts_serial->available()) sts_serial->read();

    uint8_t params[2];
    params[0] = STS_REG_PRESENT_SPEED;
    params[1] = 2;

    send_packet(id, STS_INST_READ, params, 2);

    uint8_t resp[2];
    uint8_t error = 0;
    uint8_t count = read_response(resp, 2, &error, 50);

    if (count >= 2 && error == 0) {
        int16_t speed = resp[0] | ((int16_t)resp[1] << 8);
        if (id < STS_MAX_SERVOS) {
            servo_states[id].speed = speed;
        }
        return speed;
    }
    return 0;
}

void sts_set_torque(uint8_t id, bool enable) {
    uint8_t params[2];
    params[0] = STS_REG_TORQUE_ENABLE;
    params[1] = enable ? 1 : 0;
    send_packet(id, STS_INST_WRITE, params, 2);
}

const STSState& sts_get_state(uint8_t id) {
    if (id >= STS_MAX_SERVOS) {
        static STSState empty = {};
        return empty;
    }
    return servo_states[id];
}

#include "motor_registry.h"
#include "odrive_can.h"
#include "as5048a_spi.h"
#include "sts_servo.h"
#include <string.h>

// Locally-tracked controller mode per ODrive node. 0 = unknown (never set by us
// this session), otherwise matches ODRIVE_CTRL_POSITION / VELOCITY / TORQUE.
// Used by the serial handlers to reject mismatched setpoint commands with a
// clear error instead of the ODrive silently ignoring them.
static uint8_t g_odrive_ctrl_mode[ODRIVE_MAX_NODES] = {0};

int motor_find(const char* name) {
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        if (strcmp(MOTOR_REGISTRY[i].name, name) == 0) {
            return i;
        }
    }
    return -1;
}

const MotorDef& motor_get(uint8_t index) {
    return MOTOR_REGISTRY[index];
}

void motor_init_all() {
    bool odrive_inited = false;
    bool sts_inited = false;

    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        const MotorDef& m = MOTOR_REGISTRY[i];

        switch (m.type) {
            case ODRIVE:
                if (!odrive_inited) {
                    odrive_init(250000);  // 250kbps default for ODrive S1
                    odrive_inited = true;
                }
                break;

            case STS_SERVO:
                if (!sts_inited) {
                    // Using Serial5 (TX5=20, RX5=21) for STS servos
                    // No direction pin — TX/RX wired directly
                    sts_init(&Serial5, -1, 1000000);
                    sts_inited = true;
                }
                break;

            case ENCODER_ONLY:
                as5048a_init(m.bus_id);  // bus_id is the CS pin
                break;

            case ODRIVE_LOAD_ENC:
                if (!odrive_inited) {
                    odrive_init(250000);
                    odrive_inited = true;
                }
                break;
        }
    }
}

bool motor_move(uint8_t index, float position) {
    if (index >= NUM_MOTORS) return false;
    const MotorDef& m = MOTOR_REGISTRY[index];

    switch (m.type) {
        case ODRIVE: {
            // Input is degrees, convert to turns, apply gear ratio
            float turns = deg_to_turns(position) * m.gear_ratio;
            odrive_set_position(m.bus_id, turns);
            return true;
        }

        case STS_SERVO: {
            // Input is degrees (0-360). Clamp to valid range.
            if (position < 0.0f) position = 0.0f;
            if (position > 360.0f) position = 360.0f;
            uint16_t pos = sts_deg_to_pos(position);
            sts_move(m.bus_id, pos, 500);  // 500ms move time
            return true;
        }

        case ENCODER_ONLY:
            // Can't move an encoder-only device
            return false;

        case ODRIVE_LOAD_ENC:
            return false;
    }
    return false;
}

float motor_read(uint8_t index) {
    if (index >= NUM_MOTORS) return 0.0f;
    const MotorDef& m = MOTOR_REGISTRY[index];

    switch (m.type) {
        case ODRIVE: {
            // ODrive broadcasts encoder estimates every 10ms by default
            // (encoder_msg_rate_ms = 10). The on_can_rx callback caches them.
            // Just pump the FIFO and read from cache — no blocking RTR needed.
            odrive_process_rx();
            const ODriveState& st = odrive_get_state(m.bus_id);
            return turns_to_deg(st.pos_estimate) / m.gear_ratio;
        }

        case STS_SERVO: {
            int16_t pos = sts_read_position(m.bus_id);
            if (pos < 0) return -1.0f;
            return sts_pos_to_deg(pos);
        }

        case ENCODER_ONLY: {
            float deg = as5048a_read_degrees(m.bus_id);
            return deg / m.gear_ratio;
        }

        case ODRIVE_LOAD_ENC: {
            float sdo_val = 0.0f;
            bool ok = odrive_read_sdo(m.bus_id,
                                      ODRIVE_ENDPOINT_SPI_ENC0_RAW,
                                      &sdo_val, 10);
            if (!ok) return -1.0f;
            // raw is in turns (0.0-1.0), convert to degrees
            return (sdo_val * 360.0f) / m.gear_ratio;
        }
    }
    return 0.0f;
}

bool motor_set_state(uint8_t index, uint32_t state) {
    if (index >= NUM_MOTORS) return false;
    const MotorDef& m = MOTOR_REGISTRY[index];
    if (m.type != ODRIVE) return false;

    odrive_set_state(m.bus_id, state);
    return true;
}

bool motor_set_velocity(uint8_t index, float velocity) {
    if (index >= NUM_MOTORS) return false;
    const MotorDef& m = MOTOR_REGISTRY[index];
    if (m.type != ODRIVE) return false;

    odrive_set_velocity(m.bus_id, velocity, 0.0f);
    return true;
}

bool motor_set_mode(uint8_t index, uint32_t ctrl_mode) {
    if (index >= NUM_MOTORS) return false;
    const MotorDef& m = MOTOR_REGISTRY[index];
    if (m.type != ODRIVE) return false;

    // Use trapezoidal trajectory for position mode so moves respect the
    // ODrive's trap_traj vel_limit / accel_limit instead of snapping
    // instantly. Velocity and torque modes stay on passthrough.
    uint32_t input_mode = (ctrl_mode == ODRIVE_CTRL_POSITION)
        ? ODRIVE_INPUT_TRAP_TRAJ
        : ODRIVE_INPUT_PASSTHROUGH;

    odrive_set_controller_mode(m.bus_id, ctrl_mode, input_mode);

    // Track locally so serial handlers can validate setpoint commands.
    if (m.bus_id < ODRIVE_MAX_NODES) {
        g_odrive_ctrl_mode[m.bus_id] = (uint8_t)ctrl_mode;
    }
    return true;
}

uint8_t motor_get_mode(uint8_t index) {
    if (index >= NUM_MOTORS) return 0;
    const MotorDef& m = MOTOR_REGISTRY[index];
    if (m.type != ODRIVE || m.bus_id >= ODRIVE_MAX_NODES) return 0;
    return g_odrive_ctrl_mode[m.bus_id];
}

bool motor_set_torque(uint8_t index, float torque_nm) {
    if (index >= NUM_MOTORS) return false;
    const MotorDef& m = MOTOR_REGISTRY[index];
    if (m.type != ODRIVE) return false;

    odrive_set_torque(m.bus_id, torque_nm);
    return true;
}

void motor_stop_all() {
    // First: zero velocity on all ODrives
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        const MotorDef& m = MOTOR_REGISTRY[i];
        if (m.type == ODRIVE) {
            odrive_set_velocity(m.bus_id, 0.0f, 0.0f);
        }
    }
    delay(10);
    // Then: set all ODrives to IDLE
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        const MotorDef& m = MOTOR_REGISTRY[i];
        if (m.type == ODRIVE) {
            odrive_set_state(m.bus_id, ODRIVE_STATE_IDLE);
        }
    }
}

void motor_read_all(float* positions) {
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        positions[i] = motor_read(i);
    }
}

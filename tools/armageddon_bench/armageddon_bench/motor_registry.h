#ifndef MOTOR_REGISTRY_H
#define MOTOR_REGISTRY_H

#include <Arduino.h>

// Motor types supported by the system
enum MotorType {
    ODRIVE,         // ODrive S1 via CAN bus (8308 BLDC motor)
    STS_SERVO,      // Feetech STS3215 via UART
    ENCODER_ONLY,   // AS5048A SPI encoder on Teensy (read-only, no actuation)
    ODRIVE_LOAD_ENC, // Load encoder on ODrive's SPI, read via CAN SDO
};

// Motor definition — add one entry per motor/encoder
struct MotorDef {
    const char* name;       // e.g., "j1", "j4", "j1_enc"
    MotorType type;
    uint8_t bus_id;         // CAN node_id, servo ID, or SPI CS pin
    float gear_ratio;       // output:input ratio (>1 means reduction)
};

// ============================================================
// MOTOR REGISTRY — Add your motors here!
// To add a new motor: just add one line to this array.
// ============================================================
const MotorDef MOTOR_REGISTRY[] = {
    // name       type           bus_id  gear_ratio
    {"j1",        ODRIVE,        0,      1.0},   // ODrive node 0, Eagle Power 8308
    {"j2",        ODRIVE,        1,      1.0},   // ODrive node 1, Eagle Power 8308
    {"j3",        ODRIVE,        2,      1.0},   // ODrive node 2, Eagle Power 8308
    {"j4",        STS_SERVO,     1,      1.0},   // STS3215 servo ID 1
    {"j5",        STS_SERVO,     2,      1.0},   // STS3215 servo ID 2
    {"j6",        STS_SERVO,     3,      1.0},   // STS3215 servo ID 3
    {"brake",     STS_SERVO,     4,      1.0},   // STS3215 brake servo ID 4
    // Load encoders: AS5048A on each ODrive's SPI port, read via CAN SDO.
    // j3's AS5048A isn't physically wired yet — expect ~0.00 until it is.
    {"j1_enc",    ODRIVE_LOAD_ENC, 0,    1.0},   // AS5048A on ODrive node 0
    {"j2_enc",    ODRIVE_LOAD_ENC, 1,    1.0},   // AS5048A on ODrive node 1
    {"j3_enc",    ODRIVE_LOAD_ENC, 2,    1.0},   // AS5048A on ODrive node 2 (not wired)
};

const uint8_t NUM_MOTORS = sizeof(MOTOR_REGISTRY) / sizeof(MOTOR_REGISTRY[0]);

// Find a motor by name (returns index, or -1 if not found)
int motor_find(const char* name);

// Get motor definition by index
const MotorDef& motor_get(uint8_t index);

// Initialize all motors in the registry
void motor_init_all();

// Move a motor to a position (degrees for ODRIVE, 0-4095 for STS)
// For ENCODER_ONLY, this is a no-op.
bool motor_move(uint8_t index, float position);

// Read a motor's current position (degrees)
// For ODRIVE: reads ODrive encoder estimate
// For STS_SERVO: reads present position register
// For ENCODER_ONLY: reads AS5048A angle
float motor_read(uint8_t index);

// Set ODrive axis state (only valid for ODRIVE type)
bool motor_set_state(uint8_t index, uint32_t state);

// Set ODrive velocity (only valid for ODRIVE type, turns/sec)
bool motor_set_velocity(uint8_t index, float velocity);

// Set ODrive controller mode (only valid for ODRIVE type)
// ctrl_mode: 1=TORQUE, 2=VELOCITY, 3=POSITION
// Position mode automatically uses TRAP_TRAJ input mode for smooth motion;
// velocity and torque modes use PASSTHROUGH.
bool motor_set_mode(uint8_t index, uint32_t ctrl_mode);

// Get the last-commanded controller mode for an ODrive motor.
// Returns 0 if unknown (never set by us this session) or not an ODrive motor.
uint8_t motor_get_mode(uint8_t index);

// Set ODrive torque in Nm at motor shaft (only valid for ODRIVE type)
// Requires controller mode = TORQUE and axis state = CLOSED_LOOP.
bool motor_set_torque(uint8_t index, float torque_nm);

// Emergency stop: zero velocity + IDLE on all ODrives
void motor_stop_all();

// Read all motors and fill position array (degrees)
void motor_read_all(float* positions);

#endif // MOTOR_REGISTRY_H

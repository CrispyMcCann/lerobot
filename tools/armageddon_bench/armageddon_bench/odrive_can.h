#ifndef ODRIVE_CAN_H
#define ODRIVE_CAN_H

#include <FlexCAN_T4.h>

// ODrive CAN Simple protocol command IDs
#define ODRIVE_CMD_HEARTBEAT            0x01
#define ODRIVE_CMD_ESTOP                0x02
#define ODRIVE_CMD_GET_ERROR            0x03
#define ODRIVE_CMD_SET_AXIS_STATE       0x07
#define ODRIVE_CMD_GET_ENCODER_EST      0x09
#define ODRIVE_CMD_SET_CONTROLLER_MODE  0x0B
#define ODRIVE_CMD_SET_INPUT_POS        0x0C
#define ODRIVE_CMD_SET_INPUT_VEL        0x0D
#define ODRIVE_CMD_SET_INPUT_TORQUE     0x0E
#define ODRIVE_CMD_RXSDO               0x04
#define ODRIVE_CMD_TXSDO               0x05
#define ODRIVE_CMD_CLEAR_ERRORS         0x18

// ODrive axis states
#define ODRIVE_STATE_IDLE               1
#define ODRIVE_STATE_CLOSED_LOOP        8
#define ODRIVE_STATE_FULL_CALIBRATION   3

// Controller modes
#define ODRIVE_CTRL_POSITION            3
#define ODRIVE_CTRL_VELOCITY            2
#define ODRIVE_CTRL_TORQUE              1

// Input modes
#define ODRIVE_INPUT_PASSTHROUGH         1
#define ODRIVE_INPUT_TRAP_TRAJ           5

// ODrive SDO endpoint IDs (firmware 0.6.10)
// IMPORTANT: Verify this value against your firmware's flat_endpoints.json.
// In odrivetool: look up endpoint ID for spi_encoder0.pos_abs
#define ODRIVE_ENDPOINT_SPI_ENC0_RAW       618  // spi_encoder0.raw (turns, 0.0-1.0)

// Max supported ODrive nodes on the bus
#define ODRIVE_MAX_NODES 8

// Cached state for each ODrive node
struct ODriveState {
    float pos_estimate;     // turns
    float vel_estimate;     // turns/s
    uint32_t axis_error;
    uint8_t axis_state;
    uint32_t last_heartbeat_ms;
    bool online;
    // SDO response cache
    float    sdo_value;
    uint16_t sdo_endpoint_id;
    bool     sdo_response_ready;
};

// Build CAN arbitration ID: (node_id << 5) | cmd_id
inline uint32_t odrive_arb_id(uint8_t node_id, uint8_t cmd_id) {
    return ((uint32_t)node_id << 5) | cmd_id;
}

// Extract node_id and cmd_id from arbitration ID
inline uint8_t odrive_node_from_id(uint32_t arb_id) { return arb_id >> 5; }
inline uint8_t odrive_cmd_from_id(uint32_t arb_id)  { return arb_id & 0x1F; }

// Initialize CAN bus for ODrive communication
void odrive_init(uint32_t baudrate = 250000);

// Process incoming CAN messages (call from loop or ISR)
void odrive_process_rx();

// Set axis state (IDLE=1, CLOSED_LOOP=8, etc.)
void odrive_set_state(uint8_t node_id, uint32_t state);

// Set controller mode and input mode
void odrive_set_controller_mode(uint8_t node_id, uint32_t ctrl_mode, uint32_t input_mode);

// Send position command (position in turns, feedforward optional)
void odrive_set_position(uint8_t node_id, float position_turns,
                         int16_t vel_ff = 0, int16_t torq_ff = 0);

// Send velocity command (turns/s)
void odrive_set_velocity(uint8_t node_id, float velocity, float torque_ff = 0.0f);

// Send torque command (Nm at motor shaft)
void odrive_set_torque(uint8_t node_id, float torque_nm);

// Request encoder estimates via RTR frame
void odrive_request_encoder(uint8_t node_id);

// Clear ODrive errors
void odrive_clear_errors(uint8_t node_id);

// Emergency stop
void odrive_estop(uint8_t node_id);

// Get cached state for a node
const ODriveState& odrive_get_state(uint8_t node_id);

// Check if node is online (received heartbeat within timeout)
bool odrive_is_online(uint8_t node_id, uint32_t timeout_ms = 1000);

// Read a float property via SDO (blocking, ~10ms timeout).
// Returns true if the response was received, with value written to *out_value.
bool odrive_read_sdo(uint8_t node_id, uint16_t endpoint_id, float* out_value,
                     uint32_t timeout_ms = 10);

// Dump raw CAN frames for debugging (prints to Serial for duration_ms)
void odrive_candump(uint32_t duration_ms);

// Convert turns to degrees and back
inline float turns_to_deg(float turns) { return turns * 360.0f; }
inline float deg_to_turns(float deg)   { return deg / 360.0f; }

#endif // ODRIVE_CAN_H

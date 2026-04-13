#include "odrive_can.h"
#include <string.h>

// CAN bus instance on CAN1 (pins 22 TX, 23 RX)
static FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

// Cached state per node
static ODriveState node_states[ODRIVE_MAX_NODES];

// Forward declaration for RX callback
static void on_can_rx(const CAN_message_t& msg);

void odrive_init(uint32_t baudrate) {
    memset(node_states, 0, sizeof(node_states));

    can1.begin();
    can1.setBaudRate(baudrate);
    can1.setMaxMB(16);
    can1.enableFIFO();
    can1.enableFIFOInterrupt();
    can1.setFIFOFilter(ACCEPT_ALL);
    can1.onReceive(on_can_rx);
}

void odrive_process_rx() {
    // FlexCAN_T4 events() pumps the interrupt-driven FIFO
    can1.events();
}

static void on_can_rx(const CAN_message_t& msg) {
    uint8_t node_id = odrive_node_from_id(msg.id);
    uint8_t cmd_id = odrive_cmd_from_id(msg.id);

    if (node_id >= ODRIVE_MAX_NODES) return;

    ODriveState& st = node_states[node_id];

    switch (cmd_id) {
        case ODRIVE_CMD_HEARTBEAT:
            if (msg.len >= 5) {
                memcpy(&st.axis_error, &msg.buf[0], 4);
                st.axis_state = msg.buf[4];
                st.last_heartbeat_ms = millis();
                st.online = true;
            }
            break;

        case ODRIVE_CMD_GET_ENCODER_EST:
            if (msg.len >= 8) {
                memcpy(&st.pos_estimate, &msg.buf[0], 4);
                memcpy(&st.vel_estimate, &msg.buf[4], 4);
            }
            break;

        case ODRIVE_CMD_TXSDO:
            if (msg.len >= 8) {
                uint16_t ep_id;
                memcpy(&ep_id, &msg.buf[1], 2);
                float val;
                memcpy(&val, &msg.buf[4], 4);
                st.sdo_endpoint_id = ep_id;
                st.sdo_value = val;
                st.sdo_response_ready = true;
            }
            break;

        default:
            break;
    }
}

static void send_can_msg(uint32_t arb_id, const uint8_t* data, uint8_t len,
                         bool rtr = false) {
    CAN_message_t msg;
    msg.id = arb_id;
    msg.len = len;
    msg.flags.extended = 0;
    msg.flags.remote = rtr ? 1 : 0;
    if (data && len > 0 && !rtr) {
        memcpy(msg.buf, data, len);
    }
    can1.write(msg);
}

void odrive_set_state(uint8_t node_id, uint32_t state) {
    uint8_t data[4];
    memcpy(data, &state, 4);
    send_can_msg(odrive_arb_id(node_id, ODRIVE_CMD_SET_AXIS_STATE), data, 4);
}

void odrive_set_controller_mode(uint8_t node_id, uint32_t ctrl_mode,
                                uint32_t input_mode) {
    uint8_t data[8];
    memcpy(&data[0], &ctrl_mode, 4);
    memcpy(&data[4], &input_mode, 4);
    send_can_msg(odrive_arb_id(node_id, ODRIVE_CMD_SET_CONTROLLER_MODE), data, 8);
}

void odrive_set_position(uint8_t node_id, float position_turns,
                         int16_t vel_ff, int16_t torq_ff) {
    uint8_t data[8];
    memcpy(&data[0], &position_turns, 4);
    memcpy(&data[4], &vel_ff, 2);
    memcpy(&data[6], &torq_ff, 2);
    send_can_msg(odrive_arb_id(node_id, ODRIVE_CMD_SET_INPUT_POS), data, 8);
}

void odrive_set_velocity(uint8_t node_id, float velocity, float torque_ff) {
    uint8_t data[8];
    memcpy(&data[0], &velocity, 4);
    memcpy(&data[4], &torque_ff, 4);
    send_can_msg(odrive_arb_id(node_id, ODRIVE_CMD_SET_INPUT_VEL), data, 8);
}

void odrive_set_torque(uint8_t node_id, float torque_nm) {
    uint8_t data[4];
    memcpy(data, &torque_nm, 4);
    send_can_msg(odrive_arb_id(node_id, ODRIVE_CMD_SET_INPUT_TORQUE), data, 4);
}

void odrive_request_encoder(uint8_t node_id) {
    // Send RTR frame to request encoder estimates
    send_can_msg(odrive_arb_id(node_id, ODRIVE_CMD_GET_ENCODER_EST),
                 nullptr, 8, true);
}

void odrive_clear_errors(uint8_t node_id) {
    send_can_msg(odrive_arb_id(node_id, ODRIVE_CMD_CLEAR_ERRORS), nullptr, 0);
}

void odrive_estop(uint8_t node_id) {
    send_can_msg(odrive_arb_id(node_id, ODRIVE_CMD_ESTOP), nullptr, 0);
}

const ODriveState& odrive_get_state(uint8_t node_id) {
    if (node_id >= ODRIVE_MAX_NODES) {
        static ODriveState empty = {};
        return empty;
    }
    return node_states[node_id];
}

bool odrive_is_online(uint8_t node_id, uint32_t timeout_ms) {
    if (node_id >= ODRIVE_MAX_NODES) return false;
    const ODriveState& st = node_states[node_id];
    return st.online && (millis() - st.last_heartbeat_ms < timeout_ms);
}

void odrive_candump(uint32_t duration_ms) {
    uint32_t start = millis();
    uint16_t count = 0;
    Serial.println("{\"ok\":true,\"candump\":[");
    while (millis() - start < duration_ms) {
        CAN_message_t msg;
        if (can1.read(msg)) {
            if (count > 0) Serial.print(",");
            uint8_t node = msg.id >> 5;
            uint8_t cmd = msg.id & 0x1F;
            Serial.print("{\"id\":\"0x");
            Serial.print(msg.id, HEX);
            Serial.print("\",\"node\":");
            Serial.print(node);
            Serial.print(",\"cmd\":\"0x");
            Serial.print(cmd, HEX);
            Serial.print("\",\"len\":");
            Serial.print(msg.len);
            Serial.print(",\"data\":\"");
            for (uint8_t i = 0; i < msg.len; i++) {
                if (msg.buf[i] < 0x10) Serial.print("0");
                Serial.print(msg.buf[i], HEX);
            }
            Serial.println("\"}");
            count++;
        }
        can1.events();
    }
    Serial.print("],\"count\":");
    Serial.print(count);
    Serial.println("}");
}

bool odrive_read_sdo(uint8_t node_id, uint16_t endpoint_id, float* out_value,
                     uint32_t timeout_ms) {
    if (node_id >= ODRIVE_MAX_NODES) return false;

    node_states[node_id].sdo_response_ready = false;

    // RxSdo frame: opcode=0x00 (read), endpoint_id (LE), reserved, unused
    uint8_t data[8] = {0};
    data[0] = 0x00;
    memcpy(&data[1], &endpoint_id, 2);

    send_can_msg(odrive_arb_id(node_id, ODRIVE_CMD_RXSDO), data, 8);

    // Poll for TxSdo response
    uint32_t start = millis();
    while (millis() - start < timeout_ms) {
        can1.events();
        if (node_states[node_id].sdo_response_ready &&
            node_states[node_id].sdo_endpoint_id == endpoint_id) {
            if (out_value) {
                *out_value = node_states[node_id].sdo_value;
            }
            return true;
        }
    }
    return false;
}

#include "serial_cmd.h"
#include "motor_registry.h"
#include "odrive_can.h"
#include "sts_servo.h"
#include <string.h>
#include <stdlib.h>

static char cmd_buf[CMD_MAX_LEN];
static uint8_t cmd_pos = 0;

// Forward declarations for command handlers
static void handle_move(char* args);
static void handle_vel(char* args);
static void handle_torque(char* args);
static void handle_mode(char* args);
static void handle_read(char* args);
static void handle_readall();
static void handle_state(char* args);
static void handle_stop();
static void handle_clear(char* args);
static void handle_list();
static void handle_ping();
static void handle_help();
static void handle_scan(char* args);
static void handle_servo_scan();

void serial_cmd_init(uint32_t baudrate) {
    Serial.begin(baudrate);
    cmd_pos = 0;
}

void serial_cmd_process() {
    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\n' || c == '\r') {
            if (cmd_pos == 0) continue;  // skip empty lines

            cmd_buf[cmd_pos] = '\0';
            cmd_pos = 0;

            // Parse command: first token is the command name
            char* cmd = strtok(cmd_buf, " ");
            char* args = strtok(nullptr, "");  // rest of the line

            if (cmd == nullptr) continue;

            // Convert command to uppercase for case-insensitive matching
            for (char* p = cmd; *p; p++) *p = toupper(*p);

            if (strcmp(cmd, "MOVE") == 0) {
                handle_move(args);
            } else if (strcmp(cmd, "VEL") == 0) {
                handle_vel(args);
            } else if (strcmp(cmd, "TORQUE") == 0) {
                handle_torque(args);
            } else if (strcmp(cmd, "MODE") == 0) {
                handle_mode(args);
            } else if (strcmp(cmd, "READ") == 0) {
                handle_read(args);
            } else if (strcmp(cmd, "READALL") == 0) {
                handle_readall();
            } else if (strcmp(cmd, "STATE") == 0) {
                handle_state(args);
            } else if (strcmp(cmd, "STOP") == 0) {
                handle_stop();
            } else if (strcmp(cmd, "CLEAR") == 0) {
                handle_clear(args);
            } else if (strcmp(cmd, "LIST") == 0) {
                handle_list();
            } else if (strcmp(cmd, "PING") == 0) {
                handle_ping();
            } else if (strcmp(cmd, "HELP") == 0) {
                handle_help();
            } else if (strcmp(cmd, "SCAN") == 0) {
                handle_scan(args);
            } else if (strcmp(cmd, "SERVOSCAN") == 0) {
                handle_servo_scan();
            } else if (strcmp(cmd, "CANDUMP") == 0) {
                odrive_candump(2000);
            } else {
                serial_respond_error("unknown command, try HELP");
            }
        } else if (cmd_pos < CMD_MAX_LEN - 1) {
            cmd_buf[cmd_pos++] = c;
        }
    }
}

// ---- JSON Response Helpers ----

void serial_respond_ok(const char* motor_name, float position) {
    Serial.print("{\"ok\":true,\"motor\":\"");
    Serial.print(motor_name);
    Serial.print("\",\"position\":");
    Serial.print(position, 2);
    Serial.println("}");
}

void serial_respond_ok_with_encoder(const char* motor_name, float position,
                                    float encoder_deg) {
    Serial.print("{\"ok\":true,\"motor\":\"");
    Serial.print(motor_name);
    Serial.print("\",\"position\":");
    Serial.print(position, 2);
    Serial.print(",\"encoder\":");
    Serial.print(encoder_deg, 2);
    Serial.println("}");
}

void serial_respond_error(const char* message) {
    Serial.print("{\"ok\":false,\"error\":\"");
    Serial.print(message);
    Serial.println("\"}");
}

void serial_respond_readall() {
    Serial.print("{\"ok\":true,\"motors\":{");
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        if (i > 0) Serial.print(",");
        const MotorDef& m = motor_get(i);
        float pos = motor_read(i);
        Serial.print("\"");
        Serial.print(m.name);
        Serial.print("\":");
        Serial.print(pos, 2);
    }
    Serial.println("}}");
}

void serial_respond_list() {
    Serial.print("{\"ok\":true,\"motors\":[");
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        if (i > 0) Serial.print(",");
        const MotorDef& m = motor_get(i);
        Serial.print("{\"name\":\"");
        Serial.print(m.name);
        Serial.print("\",\"type\":\"");
        switch (m.type) {
            case ODRIVE:       Serial.print("odrive"); break;
            case STS_SERVO:    Serial.print("sts_servo"); break;
            case ENCODER_ONLY: Serial.print("encoder"); break;
            case ODRIVE_LOAD_ENC: Serial.print("odrive_load_enc"); break;
        }
        Serial.print("\",\"bus_id\":");
        Serial.print(m.bus_id);
        Serial.print(",\"gear_ratio\":");
        Serial.print(m.gear_ratio, 2);
        Serial.print("}");
    }
    Serial.println("]}");
}

// ---- Command Handlers ----

static void handle_move(char* args) {
    if (!args) {
        serial_respond_error("usage: MOVE <motor> <position>");
        return;
    }

    char* name = strtok(args, " ");
    char* pos_str = strtok(nullptr, " ");

    if (!name || !pos_str) {
        serial_respond_error("usage: MOVE <motor> <position>");
        return;
    }

    int idx = motor_find(name);
    if (idx < 0) {
        char err[64];
        snprintf(err, sizeof(err), "unknown motor: %s", name);
        serial_respond_error(err);
        return;
    }

    float position = atof(pos_str);

    if (!motor_move(idx, position)) {
        serial_respond_error("cannot move this motor type");
        return;
    }

    // Brief delay then read back position
    delay(10);
    float current = motor_read(idx);
    serial_respond_ok(name, current);
}

static void handle_read(char* args) {
    if (!args) {
        serial_respond_error("usage: READ <motor>");
        return;
    }

    char* name = strtok(args, " ");
    if (!name) {
        serial_respond_error("usage: READ <motor>");
        return;
    }

    int idx = motor_find(name);
    if (idx < 0) {
        char err[64];
        snprintf(err, sizeof(err), "unknown motor: %s", name);
        serial_respond_error(err);
        return;
    }

    float pos = motor_read(idx);
    const MotorDef& m = motor_get(idx);

    // For ODrive motors, also report the load encoder if one exists
    // Look for a matching encoder (e.g., "j1" -> "j1_enc")
    if (m.type == ODRIVE) {
        char enc_name[32];
        snprintf(enc_name, sizeof(enc_name), "%s_enc", m.name);
        int enc_idx = motor_find(enc_name);
        if (enc_idx >= 0) {
            float enc_pos = motor_read(enc_idx);
            serial_respond_ok_with_encoder(name, pos, enc_pos);
            return;
        }
    }

    serial_respond_ok(name, pos);
}

static void handle_readall() {
    serial_respond_readall();
}

static void handle_state(char* args) {
    if (!args) {
        serial_respond_error("usage: STATE <motor> <state> (1=IDLE, 8=CLOSED_LOOP)");
        return;
    }

    char* name = strtok(args, " ");
    char* state_str = strtok(nullptr, " ");

    if (!name || !state_str) {
        serial_respond_error("usage: STATE <motor> <state>");
        return;
    }

    int idx = motor_find(name);
    if (idx < 0) {
        char err[64];
        snprintf(err, sizeof(err), "unknown motor: %s", name);
        serial_respond_error(err);
        return;
    }

    uint32_t state = atoi(state_str);

    if (!motor_set_state(idx, state)) {
        serial_respond_error("STATE only works for ODrive motors");
        return;
    }

    // Wait for state transition and check heartbeat
    delay(100);
    odrive_process_rx();
    const ODriveState& st = odrive_get_state(motor_get(idx).bus_id);

    Serial.print("{\"ok\":true,\"motor\":\"");
    Serial.print(name);
    Serial.print("\",\"state\":");
    Serial.print(st.axis_state);
    Serial.print(",\"error\":");
    Serial.print(st.axis_error);
    Serial.println("}");
}

static void handle_vel(char* args) {
    if (!args) {
        serial_respond_error("usage: VEL <motor> <turns_per_sec>");
        return;
    }

    char* name = strtok(args, " ");
    char* vel_str = strtok(nullptr, " ");

    if (!name || !vel_str) {
        serial_respond_error("usage: VEL <motor> <turns_per_sec>");
        return;
    }

    int idx = motor_find(name);
    if (idx < 0) {
        char err[64];
        snprintf(err, sizeof(err), "unknown motor: %s", name);
        serial_respond_error(err);
        return;
    }

    float velocity = atof(vel_str);

    if (!motor_set_velocity(idx, velocity)) {
        serial_respond_error("VEL only works for ODrive motors");
        return;
    }

    Serial.print("{\"ok\":true,\"motor\":\"");
    Serial.print(name);
    Serial.print("\",\"velocity\":");
    Serial.print(velocity, 3);
    Serial.println("}");
}

static void handle_torque(char* args) {
    if (!args) {
        serial_respond_error("usage: TORQUE <motor> <nm>");
        return;
    }

    char* name = strtok(args, " ");
    char* tq_str = strtok(nullptr, " ");

    if (!name || !tq_str) {
        serial_respond_error("usage: TORQUE <motor> <nm>");
        return;
    }

    int idx = motor_find(name);
    if (idx < 0) {
        char err[64];
        snprintf(err, sizeof(err), "unknown motor: %s", name);
        serial_respond_error(err);
        return;
    }

    float torque_nm = atof(tq_str);

    if (!motor_set_torque(idx, torque_nm)) {
        serial_respond_error("TORQUE only works for ODrive motors");
        return;
    }

    Serial.print("{\"ok\":true,\"motor\":\"");
    Serial.print(name);
    Serial.print("\",\"torque\":");
    Serial.print(torque_nm, 4);
    Serial.println("}");
}

static void handle_mode(char* args) {
    if (!args) {
        serial_respond_error("usage: MODE <motor> <pos|vel|torque>");
        return;
    }

    char* name = strtok(args, " ");
    char* mode_str = strtok(nullptr, " ");

    if (!name || !mode_str) {
        serial_respond_error("usage: MODE <motor> <pos|vel|torque>");
        return;
    }

    int idx = motor_find(name);
    if (idx < 0) {
        char err[64];
        snprintf(err, sizeof(err), "unknown motor: %s", name);
        serial_respond_error(err);
        return;
    }

    // Normalize mode string to lowercase
    for (char* p = mode_str; *p; p++) *p = tolower(*p);

    uint32_t ctrl_mode = 0;
    const char* mode_name = nullptr;
    if (strcmp(mode_str, "pos") == 0 || strcmp(mode_str, "position") == 0) {
        ctrl_mode = ODRIVE_CTRL_POSITION;
        mode_name = "position";
    } else if (strcmp(mode_str, "vel") == 0 || strcmp(mode_str, "velocity") == 0) {
        ctrl_mode = ODRIVE_CTRL_VELOCITY;
        mode_name = "velocity";
    } else if (strcmp(mode_str, "torque") == 0 || strcmp(mode_str, "tq") == 0) {
        ctrl_mode = ODRIVE_CTRL_TORQUE;
        mode_name = "torque";
    } else {
        serial_respond_error("mode must be pos, vel, or torque");
        return;
    }

    if (!motor_set_mode(idx, ctrl_mode)) {
        serial_respond_error("MODE only works for ODrive motors");
        return;
    }

    Serial.print("{\"ok\":true,\"motor\":\"");
    Serial.print(name);
    Serial.print("\",\"mode\":\"");
    Serial.print(mode_name);
    Serial.println("\"}");
}

static void handle_stop() {
    motor_stop_all();
    Serial.println("{\"ok\":true,\"msg\":\"EMERGENCY STOP - all motors idle\"}");
}

static void handle_clear(char* args) {
    if (!args) {
        serial_respond_error("usage: CLEAR <motor>");
        return;
    }

    char* name = strtok(args, " ");
    if (!name) {
        serial_respond_error("usage: CLEAR <motor>");
        return;
    }

    int idx = motor_find(name);
    if (idx < 0) {
        char err[64];
        snprintf(err, sizeof(err), "unknown motor: %s", name);
        serial_respond_error(err);
        return;
    }

    const MotorDef& m = motor_get(idx);
    if (m.type != ODRIVE) {
        serial_respond_error("CLEAR only works for ODrive motors");
        return;
    }

    odrive_clear_errors(m.bus_id);
    delay(50);
    odrive_process_rx();

    const ODriveState& st = odrive_get_state(m.bus_id);
    Serial.print("{\"ok\":true,\"motor\":\"");
    Serial.print(name);
    Serial.print("\",\"error\":");
    Serial.print(st.axis_error);
    Serial.println("}");
}

static void handle_list() {
    serial_respond_list();
}

static void handle_ping() {
    Serial.println("{\"ok\":true,\"msg\":\"pong\"}");
}

static void handle_help() {
    // Must be single-line JSON so the Python host can parse it
    Serial.println("{\"ok\":true,\"commands\":[\"MOVE <motor> <degrees>\",\"VEL <motor> <turns/s>\",\"TORQUE <motor> <nm>\",\"MODE <motor> <pos|vel|torque>\",\"READ <motor>\",\"READALL\",\"STATE <motor> <n> (1=IDLE,8=CLOSED_LOOP)\",\"STOP\",\"CLEAR <motor>\",\"LIST\",\"SCAN <start> <end>\",\"PING\",\"HELP\"]}");
}

static void handle_scan(char* args) {
    if (!args) {
        serial_respond_error("usage: SCAN <start_ep> <end_ep>");
        return;
    }
    char* start_str = strtok(args, " ");
    char* end_str = strtok(nullptr, " ");
    if (!start_str || !end_str) {
        serial_respond_error("usage: SCAN <start_ep> <end_ep>");
        return;
    }
    uint16_t start_ep = atoi(start_str);
    uint16_t end_ep = atoi(end_str);
    if (end_ep - start_ep > 200) {
        serial_respond_error("range too large, max 200");
        return;
    }

    // Collect results, then print as single JSON line
    uint16_t found_eps[50];
    float found_vals[50];
    uint8_t count = 0;

    for (uint16_t ep = start_ep; ep <= end_ep && count < 50; ep++) {
        float val = 0.0f;
        bool ok = odrive_read_sdo(0, ep, &val, 5);
        if (ok && val != 0.0f && !isnan(val) && !isinf(val)) {
            found_eps[count] = ep;
            found_vals[count] = val;
            count++;
        }
    }

    Serial.print("{\"ok\":true,\"scan\":[");
    for (uint8_t i = 0; i < count; i++) {
        if (i > 0) Serial.print(",");
        Serial.print("{\"ep\":");
        Serial.print(found_eps[i]);
        Serial.print(",\"val\":");
        Serial.print(found_vals[i], 6);
        Serial.print("}");
    }
    Serial.println("]}");
}

static void handle_servo_scan() {
    const uint32_t bauds[] = {1000000, 500000, 250000, 115200, 57600, 38400};
    const uint8_t num_bauds = sizeof(bauds) / sizeof(bauds[0]);

    Serial.print("{\"ok\":true,\"servo_scan\":[");
    bool first = true;

    for (uint8_t b = 0; b < num_bauds; b++) {
        sts_init(&Serial5, -1, bauds[b]);
        delay(10);

        for (uint8_t id = 0; id <= 10; id++) {
            if (sts_ping(id)) {
                if (!first) Serial.print(",");
                Serial.print("{\"baud\":");
                Serial.print(bauds[b]);
                Serial.print(",\"id\":");
                Serial.print(id);
                Serial.print("}");
                first = false;
            }
        }
    }

    Serial.println("]}");

    // Restore original baud rate
    sts_init(&Serial5, -1, 1000000);
}

#ifndef SERIAL_CMD_H
#define SERIAL_CMD_H

#include <Arduino.h>

// Maximum command line length
#define CMD_MAX_LEN 128

// Initialize the serial command interface
void serial_cmd_init(uint32_t baudrate = 115200);

// Check for and process incoming serial commands
// Call this from loop()
void serial_cmd_process();

// Send a JSON success response
void serial_respond_ok(const char* motor_name, float position);

// Send a JSON success response with extra encoder field
void serial_respond_ok_with_encoder(const char* motor_name, float position,
                                    float encoder_deg);

// Send a JSON error response
void serial_respond_error(const char* message);

// Send a JSON response for READALL
void serial_respond_readall();

// Send a JSON response for LIST
void serial_respond_list();

#endif // SERIAL_CMD_H

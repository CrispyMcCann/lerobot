#ifndef AS5048A_SPI_H
#define AS5048A_SPI_H

#include <Arduino.h>
#include <SPI.h>

// AS5048A register addresses (14-bit)
#define AS5048A_REG_NOP         0x0000
#define AS5048A_REG_CLEAR_ERR   0x0001
#define AS5048A_REG_DIAG_AGC    0x3FFD
#define AS5048A_REG_MAGNITUDE   0x3FFE
#define AS5048A_REG_ANGLE       0x3FFF

// 14-bit resolution: 16384 counts per revolution
#define AS5048A_CPR             16384

// SPI settings: Mode 1 (CPOL=0, CPHA=1), MSB first, max 10 MHz
#define AS5048A_SPI_SPEED       10000000
#define AS5048A_SPI_MODE        SPI_MODE1

struct AS5048AState {
    uint16_t raw_angle;     // 0-16383
    float degrees;          // 0-360
    bool error_flag;
    bool parity_ok;
    uint32_t last_read_ms;
};

// Initialize the AS5048A encoder on a given CS pin
void as5048a_init(uint8_t cs_pin);

// Read raw 14-bit angle (0-16383)
uint16_t as5048a_read_raw(uint8_t cs_pin);

// Read angle in degrees (0.0-360.0)
float as5048a_read_degrees(uint8_t cs_pin);

// Read angle in turns (0.0-1.0)
float as5048a_read_turns(uint8_t cs_pin);

// Get last read state (raw + error info)
const AS5048AState& as5048a_get_state(uint8_t cs_pin);

// Clear error flags on the sensor
void as5048a_clear_errors(uint8_t cs_pin);

#endif // AS5048A_SPI_H

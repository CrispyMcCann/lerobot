#include "as5048a_spi.h"

// Store state per CS pin (support up to 4 encoders)
#define MAX_ENCODERS 4
static struct {
    uint8_t cs_pin;
    AS5048AState state;
    bool initialized;
} encoders[MAX_ENCODERS];
static uint8_t num_encoders = 0;

static SPISettings spi_settings(AS5048A_SPI_SPEED, MSBFIRST, AS5048A_SPI_MODE);

// Even parity over bits 14:0
static bool check_parity(uint16_t value) {
    uint16_t v = value & 0x7FFF;
    v ^= v >> 8;
    v ^= v >> 4;
    v ^= v >> 2;
    v ^= v >> 1;
    bool calc_parity = v & 1;
    bool recv_parity = (value >> 15) & 1;
    return calc_parity == recv_parity;
}

// Add parity bit to a 15-bit command word
static uint16_t add_parity(uint16_t cmd) {
    cmd &= 0x7FFF;  // clear parity bit
    uint16_t v = cmd;
    v ^= v >> 8;
    v ^= v >> 4;
    v ^= v >> 2;
    v ^= v >> 1;
    if (v & 1) cmd |= 0x8000;
    return cmd;
}

static int find_encoder(uint8_t cs_pin) {
    for (uint8_t i = 0; i < num_encoders; i++) {
        if (encoders[i].cs_pin == cs_pin) return i;
    }
    return -1;
}

void as5048a_init(uint8_t cs_pin) {
    if (num_encoders >= MAX_ENCODERS) return;

    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);
    SPI.begin();

    int idx = find_encoder(cs_pin);
    if (idx < 0) {
        idx = num_encoders++;
    }
    encoders[idx].cs_pin = cs_pin;
    encoders[idx].initialized = true;
    memset(&encoders[idx].state, 0, sizeof(AS5048AState));

    // Do two dummy reads to flush the pipeline
    as5048a_read_raw(cs_pin);
    as5048a_read_raw(cs_pin);
}

static uint16_t spi_transfer16(uint8_t cs_pin, uint16_t tx_word) {
    SPI.beginTransaction(spi_settings);
    digitalWrite(cs_pin, LOW);
    uint16_t rx = SPI.transfer16(tx_word);
    digitalWrite(cs_pin, HIGH);
    SPI.endTransaction();
    delayMicroseconds(1);  // CS high time between frames
    return rx;
}

uint16_t as5048a_read_raw(uint8_t cs_pin) {
    int idx = find_encoder(cs_pin);
    if (idx < 0) return 0;

    // Build read command for ANGLE register: R/W=1 (read), addr=0x3FFF
    uint16_t cmd = add_parity(0x7FFF);  // bit14=1 (read) | 0x3FFF

    // Transaction 1: send read command (response is stale data from prior read)
    spi_transfer16(cs_pin, cmd);

    // Transaction 2: send NOP to clock out the angle
    uint16_t nop_cmd = add_parity(0x4000);  // bit14=1 (read) | 0x0000 (NOP)
    uint16_t response = spi_transfer16(cs_pin, nop_cmd);

    // Parse response
    bool parity_ok = check_parity(response);
    bool error_flag = (response >> 14) & 0x01;
    uint16_t raw_angle = response & 0x3FFF;

    // Update cached state
    AS5048AState& st = encoders[idx].state;
    st.raw_angle = raw_angle;
    st.degrees = (float)raw_angle / (float)AS5048A_CPR * 360.0f;
    st.error_flag = error_flag;
    st.parity_ok = parity_ok;
    st.last_read_ms = millis();

    return raw_angle;
}

float as5048a_read_degrees(uint8_t cs_pin) {
    as5048a_read_raw(cs_pin);
    int idx = find_encoder(cs_pin);
    if (idx < 0) return 0.0f;
    return encoders[idx].state.degrees;
}

float as5048a_read_turns(uint8_t cs_pin) {
    as5048a_read_raw(cs_pin);
    int idx = find_encoder(cs_pin);
    if (idx < 0) return 0.0f;
    return (float)encoders[idx].state.raw_angle / (float)AS5048A_CPR;
}

const AS5048AState& as5048a_get_state(uint8_t cs_pin) {
    int idx = find_encoder(cs_pin);
    if (idx < 0) {
        static AS5048AState empty = {};
        return empty;
    }
    return encoders[idx].state;
}

void as5048a_clear_errors(uint8_t cs_pin) {
    // Read the CLEAR_ERROR register (reading it clears the error)
    uint16_t cmd = add_parity(0x4001);  // read | 0x0001
    spi_transfer16(cs_pin, cmd);
    spi_transfer16(cs_pin, add_parity(0x4000));  // NOP to get response
}

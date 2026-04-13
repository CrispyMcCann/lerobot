/*
 * ARMageddon Bench Test Firmware
 * Teensy 4.1
 *
 * Controls:
 *   - ODrive S1 motor controller via CAN bus (Joint 1: Eagle Power 8308)
 *   - AS5048A absolute encoder via SPI (load encoder for J1)
 *   - STS3215 Feetech servo via UART (Joint 4)
 *
 * Receives text commands from Python host via USB serial.
 * Responds with JSON for easy parsing.
 *
 * Pin assignments:
 *   CAN1:  TX=22, RX=23  (via SN65HVD230 transceiver to ODrive)
 *   SPI:   SCK=13, MOSI=11, MISO=12, CS=10  (AS5048A encoder)
 *   UART:  Serial5 TX5=20, RX5=21, no DIR pin  (STS3215 direct)
 *   USB:   Serial (native USB to Python host)
 *
 * To add a new motor: edit motor_registry.h and add one line to MOTOR_REGISTRY[].
 */

#include "motor_registry.h"
#include "odrive_can.h"
#include "as5048a_spi.h"
#include "sts_servo.h"
#include "serial_cmd.h"

// Status LED — DO NOT use pin 13, it's SPI SCK for the AS5048A encoder!
// Pin 2 is a safe choice (not used by CAN, SPI, or UART)
#define LED_PIN 2

// Heartbeat interval for status indication
#define HEARTBEAT_INTERVAL_MS 1000
static uint32_t last_heartbeat_ms = 0;

void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    // Initialize USB serial for host communication
    serial_cmd_init(115200);

    // Wait for serial connection (with timeout for headless operation)
    uint32_t start = millis();
    while (!Serial && millis() - start < 3000) {
        // blink while waiting
        digitalWrite(LED_PIN, (millis() / 250) % 2);
    }

    Serial.println("{\"ok\":true,\"msg\":\"ARMageddon bench test starting...\"}");

    // Initialize all motors/encoders in the registry
    motor_init_all();

    Serial.print("{\"ok\":true,\"msg\":\"Initialized ");
    Serial.print(NUM_MOTORS);
    Serial.println(" motors. Type HELP for commands.\"}");

    // Print the motor list at startup
    serial_respond_list();

    digitalWrite(LED_PIN, HIGH);
}

void loop() {
    // Process incoming CAN messages (heartbeats, encoder responses)
    odrive_process_rx();

    // Process incoming serial commands from Python host
    serial_cmd_process();

    // Heartbeat LED blink
    uint32_t now = millis();
    if (now - last_heartbeat_ms >= HEARTBEAT_INTERVAL_MS) {
        last_heartbeat_ms = now;
        digitalToggle(LED_PIN);
    }
}

#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// Pin assignments — ESP32 38-pin dev board
// All GPIOs are 3.3V logic. Never connect a 5V signal directly.
// ─────────────────────────────────────────────────────────────────────────────

// I2C bus — shared by MPU-6050, MAX30102, and SSD1306
// GY-521 already has 4.7 kΩ pull-ups — do not add external pull-ups
#define PIN_I2C_SDA         21
#define PIN_I2C_SCL         22

// Pulse Sensor Amped — analog green-LED heart rate reference
// GPIO34 is ADC1-CH6, input-only: safe for ADC, no internal pull-up/down
// ADC1 is required — ADC2 conflicts with WiFi/BLE radio
#define PIN_PULSE_SENSOR    34

// Battery voltage monitor — resistor divider from LiPo terminal
// GPIO35 is ADC1-CH7, input-only
// Do not connect during early bring-up; wire only when LiPo is integrated
#define PIN_BATT_ADC        35

// Status LED — active HIGH, GPIO2 is built-in LED on most 38-pin ESP32 boards
#define PIN_STATUS_LED      2

// Future: INT pin from MPU-6050 (data-ready interrupt, currently unused)
// #define PIN_IMU_INT      4

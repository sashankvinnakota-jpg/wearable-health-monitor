#pragma once
#include <stdint.h>

// ─────────────────────────────────────────────────────────────────────────────
// App state machine
//
// BOOT → INIT_HW → IDLE → STREAM_RAW
//                       ↘ SENSOR_ERROR
//                       ↘ LOW_BATTERY   (TODO: Phase 3)
//                       ↘ CALIBRATION   (TODO: Phase 2)
// ─────────────────────────────────────────────────────────────────────────────
enum class AppState : uint8_t {
    BOOT,           // MCU just powered on, no peripherals initialized
    INIT_HW,        // I2C, sensors, display initializing
    IDLE,           // sensors running, no active streaming/processing
    STREAM_RAW,     // streaming raw IMU + PPG values to serial
    CALIBRATION,    // TODO: offset calibration routine
    SENSOR_ERROR,   // one or more sensors failed init or stopped responding
    LOW_BATTERY,    // TODO: battery below 3.5 V threshold
};

// ── Sensor health flags ───────────────────────────────────────────────────────
struct SensorHealth {
    bool imu_ok     : 1;    // MPU-6050 init and reading
    bool ppg_ok     : 1;    // MAX30102 init and reading
    bool display_ok : 1;    // SSD1306 found and init'd
    bool pulse_ok   : 1;    // Pulse Sensor Amped (always true if ADC pin is wired)
    uint8_t _pad    : 4;
};

// ── Shared system status ──────────────────────────────────────────────────────
// Written by sensor tasks, read by app loop and display task.
// Fields are primitive types — reads/writes are atomic on Xtensa LX6.
// If you add non-primitive fields, protect with a mutex.
struct AppStatus {
    AppState        state;
    SensorHealth    health;

    uint32_t        uptime_ms;

    // Latest sensor readings — updated every sample cycle
    float           last_accel_mag;     // g — accel vector magnitude
    uint32_t        last_ppg_red;       // raw 18-bit count
    uint32_t        last_ppg_ir;        // raw 18-bit count
    int             pulse_raw;          // 12-bit ADC reading (Pulse Sensor)

    // TODO (Phase 2): float heart_rate_bpm
    // TODO (Phase 2): float spo2_pct
    // TODO (Phase 3): bool fall_detected
    // TODO (Phase 3): uint8_t battery_pct
};

// Global singleton — access from any translation unit
AppStatus& getAppStatus();
const char* appStateString(AppState s);

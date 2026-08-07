#pragma once
#include "../app/app_state.h"
#include <stdint.h>
#include <stdbool.h>

// ─────────────────────────────────────────────────────────────────────────────
// SSD1306Display
//
// Responsibility:
//   Render AppStatus fields to the OLED. Nothing else.
//   Never reads sensor registers directly — always pulls from AppStatus.
//   All display calls hold the I2C mutex to prevent bus contention
//   with the sensor tasks (Adafruit library calls Wire internally).
//
// Why Adafruit_SSD1306 here but not for sensors:
//   SSD1306 init requires ~50 initialization commands plus a frame buffer.
//   The Adafruit library is the de facto standard abstraction for this display.
//   Sensor drivers are register-level because the algorithms depend on
//   understanding every configuration decision.
// ─────────────────────────────────────────────────────────────────────────────

class SSD1306Display {
public:
    SSD1306Display(uint8_t i2c_addr, uint8_t width, uint8_t height);

    bool    begin();

    // Structured screens — call from app loop, not from sensor tasks
    void    showBootScreen(const char *version);
    void    showSensorStatus(const SensorHealth &health);
    void    showRawData(const AppStatus &status);
    void    showError(const char *msg);
    void    clear();

    // TODO (Phase 2): showHeartRate(float bpm)
    // TODO (Phase 2): showSpO2(float pct)
    // TODO (Phase 3): showFallAlert(uint32_t timestamp_ms)
    // TODO (Phase 3): showBatteryBar(uint8_t pct)

private:
    uint8_t _addr;
    uint8_t _width;
    uint8_t _height;
    bool    _initialized;

    // Acquire I2C mutex, render, release — used for any multi-call display sequence
    void    _beginFrame();
    void    _endFrame();
};

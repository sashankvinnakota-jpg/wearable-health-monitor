#pragma once
#include <stdint.h>
#include <stdbool.h>

// ─────────────────────────────────────────────────────────────────────────────
// PowerManager
//
// Current scope (bring-up phase): USB power only, monitoring stubs.
// Do not connect LiPo until firmware is stable.
//
// Later phases:
//   - Read battery voltage via resistor divider on PIN_BATT_ADC
//   - Set LOW_BATTERY app state when voltage < 3.5 V
//   - Reduce LED current at low battery
//   - Implement light sleep between sensor reads
// ─────────────────────────────────────────────────────────────────────────────

// LiPo voltage thresholds
#define BATT_FULL_V         4.20f
#define BATT_LOW_V          3.50f   // warn threshold
#define BATT_CUTOFF_V       3.00f   // protection IC cuts off here anyway

// Resistor divider from LiPo terminal to PIN_BATT_ADC:
//   Vbatt → 100 kΩ → PIN_BATT_ADC → 100 kΩ → GND
//   Vmeasured = Vbatt / 2  →  multiply by 2 to recover Vbatt
#define BATT_DIVIDER_RATIO  2.0f
#define ADC_VREF            3.3f
#define ADC_MAX_COUNT       4095.0f

class PowerManager {
public:
    static void     begin();

    static uint16_t readBatteryRaw();       // raw 12-bit ADC count
    static float    readBatteryVoltage();   // estimated LiPo voltage in V
    static uint8_t  batteryPercent();       // 0–100 (linear approx between 3.0–4.2 V)
    static bool     isBatteryLow();

    // TODO (Phase 3): enterLightSleep(uint32_t duration_ms)
    // TODO (Phase 3): setWakeupPin(uint8_t gpio) — IMU interrupt wakes ESP32
    // TODO (Phase 3): reduceLedCurrent() — call when battery < BATT_LOW_V
};

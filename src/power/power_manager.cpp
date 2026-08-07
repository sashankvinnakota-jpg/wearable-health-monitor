#include "power_manager.h"
#include "../config/pins.h"
#include "../utils/logger.h"
#include <Arduino.h>
#include <algorithm>

void PowerManager::begin() {
    // Do not read battery ADC during USB-only bring-up.
    // Wire the voltage divider only when LiPo is integrated (Phase 3).
    LOG_I("PWR", "PowerManager ready. Running on USB. LiPo deferred to Phase 3.");
}

uint16_t PowerManager::readBatteryRaw() {
    return (uint16_t)analogRead(PIN_BATT_ADC);
}

float PowerManager::readBatteryVoltage() {
    float adc = (float)readBatteryRaw();
    return (adc / ADC_MAX_COUNT) * ADC_VREF * BATT_DIVIDER_RATIO;
}

uint8_t PowerManager::batteryPercent() {
    float v = readBatteryVoltage();
    float pct = (v - BATT_CUTOFF_V) / (BATT_FULL_V - BATT_CUTOFF_V) * 100.0f;
    return (uint8_t)std::max(0.0f, std::min(100.0f, pct));
}

bool PowerManager::isBatteryLow() {
    return readBatteryVoltage() < BATT_LOW_V;
}

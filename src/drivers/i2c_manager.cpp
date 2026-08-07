#include "i2c_manager.h"
#include "../utils/logger.h"
#include <Wire.h>
#include <Arduino.h>

SemaphoreHandle_t I2CManager::_mutex = nullptr;

// ─────────────────────────────────────────────────────────────────────────────
// Repeated-START vs STOP-START register reads
//
// An I2C register read is two transactions: write the register number, then
// read the data. They can be joined two ways:
//
//   Repeated START  — endTransmission(false). Holds the bus across both
//                     phases. Standard, faster, what most devices expect.
//   STOP then START — endTransmission(true), then a fresh requestFrom().
//                     Releases the bus in between. Slower but far more
//                     tolerant of slow rise times.
//
// Breakout boards that level-shift I2C (notably the purple MAX30102 modules,
// which shift down to a 1.8 V rail) often ACK their address but cannot meet
// repeated-START timing, so every register read fails with -1 while the bus
// scan still shows the device.
//
// Strategy: try repeated START; on failure fall back to STOP-START. If the
// fallback works, remember that address and go straight to STOP-START next
// time — otherwise every read pays for a failed attempt first, which at
// 100 Hz is a real cost.
// ─────────────────────────────────────────────────────────────────────────────

static uint8_t _stop_start_addrs[8] = {0};
static uint8_t _stop_start_count    = 0;

static bool needsStopStart(uint8_t addr) {
    for (uint8_t i = 0; i < _stop_start_count; i++) {
        if (_stop_start_addrs[i] == addr) return true;
    }
    return false;
}

static void markStopStart(uint8_t addr) {
    if (_stop_start_count < sizeof(_stop_start_addrs)) {
        _stop_start_addrs[_stop_start_count++] = addr;
    }
}

// Both helpers assume the caller already holds _mutex.
static bool rawReadRepeatedStart(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;

    if (Wire.requestFrom((int)addr, (int)len, true) != (int)len) return false;
    for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
    return true;
}

static bool rawReadStopStart(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(true) != 0) return false;

    delayMicroseconds(50);          // let the bus settle before the read phase
    if (Wire.requestFrom((int)addr, (int)len, true) != (int)len) return false;
    for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
    return true;
}

bool I2CManager::begin(int sda_pin, int scl_pin, uint32_t freq_hz) {
    if (!_mutex) {
        _mutex = xSemaphoreCreateMutex();
        if (!_mutex) return false;
    }

    Wire.begin(sda_pin, scl_pin);
    Wire.setClock(freq_hz);
    return true;
}

bool I2CManager::writeReg(uint8_t dev_addr, uint8_t reg, uint8_t data) {
    if (!_mutex) return false;

    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return false;
    }

    Wire.beginTransmission(dev_addr);
    Wire.write(reg);
    Wire.write(data);
    bool ok = (Wire.endTransmission() == 0);

    xSemaphoreGive(_mutex);
    return ok;
}

uint8_t I2CManager::readReg(uint8_t dev_addr, uint8_t reg) {
    uint8_t val = 0;
    readRegs(dev_addr, reg, &val, 1);
    return val;
}

bool I2CManager::readReg(uint8_t dev_addr, uint8_t reg, uint8_t *val) {
    if (!val) return false;
    return readRegs(dev_addr, reg, val, 1);
}

bool I2CManager::readRegs(uint8_t dev_addr, uint8_t reg, uint8_t *buf, size_t len) {
    if (!_mutex || !buf || len == 0 || len > 255) return false;

    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return false;
    }

    bool ok            = false;
    bool newly_flagged = false;

    // Preferred path, unless this address has already proven it can't do it.
    if (!needsStopStart(dev_addr)) {
        ok = rawReadRepeatedStart(dev_addr, reg, buf, len);
    }

    // Fallback — also the primary path for addresses already flagged.
    if (!ok) {
        ok = rawReadStopStart(dev_addr, reg, buf, len);
        if (ok && !needsStopStart(dev_addr)) {
            markStopStart(dev_addr);
            newly_flagged = true;
        }
    }

    xSemaphoreGive(_mutex);

    // Log outside the lock — Serial can block, and holding the I2C mutex while
    // it does would stall the sensor tasks.
    if (newly_flagged) {
        LOG_W("I2C",
              "0x%02X failed repeated-START; using STOP-then-read for this device.",
              dev_addr);
    }

    return ok;
}

bool I2CManager::devicePresent(uint8_t addr) {
    if (!_mutex) return false;

    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return false;
    }

    Wire.beginTransmission(addr);
    bool ok = (Wire.endTransmission() == 0);

    xSemaphoreGive(_mutex);
    return ok;
} 
bool I2CManager::lock(uint32_t timeout_ms) {
    if (!_mutex) return false;

    return (xSemaphoreTake(
                _mutex,
                pdMS_TO_TICKS(timeout_ms)
            ) == pdTRUE);
}

void I2CManager::unlock() {
    if (_mutex) {
        xSemaphoreGive(_mutex);
    }
}
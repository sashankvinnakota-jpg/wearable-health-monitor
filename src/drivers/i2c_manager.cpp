#include "i2c_manager.h"
#include <Wire.h>
#include <Arduino.h>

SemaphoreHandle_t I2CManager::_mutex = nullptr;

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

    Wire.beginTransmission(dev_addr);
    Wire.write(reg);

    bool ok = (Wire.endTransmission(false) == 0);

    if (ok) {
        size_t received = Wire.requestFrom((int)dev_addr, (int)len, true);
        ok = (received == len);

        if (ok) {
            for (size_t i = 0; i < len; i++) {
                buf[i] = Wire.read();
            }
        }
    }

    xSemaphoreGive(_mutex);
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
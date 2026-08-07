#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class I2CManager {
public:
    static bool     begin(int sda_pin, int scl_pin, uint32_t freq_hz);

    static bool     writeReg(uint8_t dev_addr, uint8_t reg, uint8_t data);

    // 2-arg convenience: returns the byte value (0 on error)
    static uint8_t  readReg(uint8_t dev_addr, uint8_t reg);
    // 3-arg form: returns true on success, writes value to *val
    static bool     readReg(uint8_t dev_addr, uint8_t reg, uint8_t *val);

    static bool     readRegs(uint8_t dev_addr, uint8_t reg, uint8_t *buf, size_t len);

    static bool     devicePresent(uint8_t addr);

    // lock/unlock for callers that need to hold the bus across multiple transactions
    static bool     lock(uint32_t timeout_ms = 100);
    static void     unlock();

    // Raw mutex handle — display layer uses this with xSemaphoreTake/Give directly
    static SemaphoreHandle_t getMutex() { return _mutex; }

private:
    static SemaphoreHandle_t _mutex;
};
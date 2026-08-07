#include "max30102_driver.h"
#include "../drivers/i2c_manager.h"
#include "../utils/logger.h"
#include <Arduino.h>

MAX30102Driver::MAX30102Driver(uint8_t i2c_addr)
    : _addr(i2c_addr), _initialized(false) {}

bool MAX30102Driver::isConnected() {
    uint8_t id = I2CManager::readReg(_addr, MAX_REG_PART_ID);
    if (id != MAX_PART_ID_EXPECTED) {
        LOG_E("MAX30102", "Part ID: expected 0x%02X, got 0x%02X", MAX_PART_ID_EXPECTED, id);
        return false;
    }
    return true;
}

bool MAX30102Driver::begin() {
    if (!isConnected()) {
        LOG_E("MAX30102", "Not found at I2C 0x%02X. Check wiring.", _addr);
        return false;
    }
    if (!_softwareReset()) {
        LOG_E("MAX30102", "Software reset timed out.");
        return false;
    }
    if (!_configure()) {
        LOG_E("MAX30102", "Configuration write failed.");
        return false;
    }
    _initialized = true;
    LOG_I("MAX30102", "OK — SpO2 mode | 100 sps | 18-bit | LED %.1f mA",
          MAX_LED_CURRENT_INIT * 0.2f);
    return true;
}

bool MAX30102Driver::_softwareReset() {
    // Set RESET bit (bit 6) in MODE_CONFIG. Device clears it when reset is complete.
    if (!I2CManager::writeReg(_addr, MAX_REG_MODE_CONFIG, MAX_RESET_BIT)) return false;

    // Poll until reset bit clears (typically < 1 ms, timeout at 100 ms)
    for (int i = 0; i < 100; i++) {
        delay(1);
        if (!(I2CManager::readReg(_addr, MAX_REG_MODE_CONFIG) & MAX_RESET_BIT)) return true;
    }
    return false;
}

bool MAX30102Driver::_configure() {
    // Clear FIFO pointers before configuring — avoids reading stale pre-reset data
    _clearFifo();

    // FIFO: no hardware averaging (filter in firmware), rollover enabled
    if (!I2CManager::writeReg(_addr, MAX_REG_FIFO_CONFIG, MAX_FIFO_CONFIG_VAL)) return false;

    // Mode: SpO2 — both Red (LED1) and IR (LED2) active
    if (!I2CManager::writeReg(_addr, MAX_REG_MODE_CONFIG, MAX_MODE_SPO2)) return false;

    // SpO2 config: 4096 nA ADC range | 100 sps | 411 µs pulse | 18-bit resolution
    // 18-bit gives maximum SNR — important for wrist where optical coupling is poor
    if (!I2CManager::writeReg(_addr, MAX_REG_SPO2_CONFIG, MAX_SPO2_CONFIG_VAL)) return false;

    // LED drive currents — conservative starting point
    // If red and ir FIFO values stay below ~50000, increase in steps of 0x10
    if (!I2CManager::writeReg(_addr, MAX_REG_LED1_PA, MAX_LED_CURRENT_INIT)) return false;
    if (!I2CManager::writeReg(_addr, MAX_REG_LED2_PA, MAX_LED_CURRENT_INIT)) return false;

    return true;
}

void MAX30102Driver::_clearFifo() {
    I2CManager::writeReg(_addr, MAX_REG_FIFO_WR_PTR, 0x00);
    I2CManager::writeReg(_addr, MAX_REG_OVF_COUNTER, 0x00);
    I2CManager::writeReg(_addr, MAX_REG_FIFO_RD_PTR, 0x00);
}

void MAX30102Driver::setLedCurrent(uint8_t red_pa, uint8_t ir_pa) {
    I2CManager::writeReg(_addr, MAX_REG_LED1_PA, red_pa);
    I2CManager::writeReg(_addr, MAX_REG_LED2_PA, ir_pa);
    LOG_I("MAX30102", "LED current updated: Red=%.1f mA IR=%.1f mA",
          red_pa * 0.2f, ir_pa * 0.2f);
}

uint8_t MAX30102Driver::getOverflowCount() {
    return I2CManager::readReg(_addr, MAX_REG_OVF_COUNTER);
}

int MAX30102Driver::readFifo(PpgSample *buf, int max_samples) {
    if (!_initialized || !buf || max_samples <= 0) return 0;

    uint8_t wr  = I2CManager::readReg(_addr, MAX_REG_FIFO_WR_PTR);
    uint8_t rd  = I2CManager::readReg(_addr, MAX_REG_FIFO_RD_PTR);
    uint8_t ovf = I2CManager::readReg(_addr, MAX_REG_OVF_COUNTER);

    if (ovf > 0) {
        LOG_W("MAX30102", "FIFO overflow: %d samples lost. Increase drain frequency.", ovf);
        _clearFifo();
        return 0;
    }

    // Number of unread samples in circular buffer
    int count = (int)((wr - rd + MAX_FIFO_DEPTH) % MAX_FIFO_DEPTH);
    if (count == 0) return 0;
    if (count > max_samples) count = max_samples;

    uint8_t  raw[MAX_BYTES_PER_SAMPLE];
    uint32_t now = millis();

    for (int i = 0; i < count; i++) {
        // Reading FIFO_DATA (0x07) returns 3 bytes per channel and auto-advances RD_PTR
        if (!I2CManager::readRegs(_addr, MAX_REG_FIFO_DATA, raw, MAX_BYTES_PER_SAMPLE)) {
            LOG_W("MAX30102", "FIFO read error at sample %d", i);
            return i;
        }

        // Each channel: 3 bytes big-endian, 18-bit data left-aligned in 24-bit field.
        // Upper 6 bits are unused (set to 0). Mask to extract 18-bit value.
        buf[i].red = ((uint32_t)raw[0] << 16 | (uint32_t)raw[1] << 8 | raw[2]) & MAX_RAW_MASK;
        buf[i].ir  = ((uint32_t)raw[3] << 16 | (uint32_t)raw[4] << 8 | raw[5]) & MAX_RAW_MASK;
        buf[i].timestamp_ms = now;
    }

    return count;
}

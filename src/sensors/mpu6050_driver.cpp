#include "mpu6050_driver.h"
#include "../drivers/i2c_manager.h"
#include "../utils/logger.h"
#include "../config/board_config.h"
#include <Arduino.h>

MPU6050Driver::MPU6050Driver(uint8_t i2c_addr)
    : _addr(i2c_addr), _initialized(false) {}

bool MPU6050Driver::isConnected() {
    uint8_t id = I2CManager::readReg(_addr, MPU_REG_WHO_AM_I);

    switch (id) {
        case 0x68:                  // genuine MPU-6050
            break;
        case 0x70:                  // MPU-6500 die — the usual "MPU6050" clone
        case 0x71:                  // MPU-9250
        case 0x73:                  // MPU-9255
            LOG_W("MPU6050",
                  "WHO_AM_I 0x%02X — clone die, register-compatible. Continuing.", id);
            break;
        default:
            LOG_E("MPU6050", "WHO_AM_I unrecognised — got 0x%02X", id);
            return false;
    }
    return true;
}

bool MPU6050Driver::begin() {
    if (!isConnected()) {
        LOG_E("MPU6050",
              "Not found at I2C 0x%02X. Check SDA/SCL wiring and AD0 pin.",
              _addr);
        return false;
    }

    _initialized = _wakeAndConfigure();

    if (!_initialized) {
        LOG_E("MPU6050", "Initialization failed.");
        return false;
    }

    // Dump configuration registers for verification
    uint8_t config      = I2CManager::readReg(_addr, MPU_REG_CONFIG);
    uint8_t smplrt_div  = I2CManager::readReg(_addr, MPU_REG_SMPLRT_DIV);
    uint8_t gyro_cfg    = I2CManager::readReg(_addr, MPU_REG_GYRO_CONFIG);
    uint8_t accel_cfg   = I2CManager::readReg(_addr, MPU_REG_ACCEL_CONFIG);

    LOG_I("MPU6050", "Initialization successful");
    LOG_I("MPU6050", "CONFIG       = 0x%02X", config);
    LOG_I("MPU6050", "SMPLRT_DIV   = 0x%02X", smplrt_div);
    LOG_I("MPU6050", "GYRO_CONFIG  = 0x%02X", gyro_cfg);
    LOG_I("MPU6050", "ACCEL_CONFIG = 0x%02X", accel_cfg);

    LOG_I("MPU6050",
          "Running at %d Hz | ±8 g | ±500 °/s | DLPF 44 Hz",
          IMU_SAMPLE_HZ);

    return true;
}

bool MPU6050Driver::_wakeAndConfigure() {

    // ------------------------------------------------------------------------
    // Step 0: Optional device reset
    // ------------------------------------------------------------------------
    // Writing DEVICE_RESET (bit 7) resets internal registers to defaults.
    // Useful during bring-up to guarantee known state.
    if (!I2CManager::writeReg(_addr, MPU_REG_PWR_MGMT_1, 0x80)) {
        LOG_E("MPU6050", "Failed to reset device");
        return false;
    }

    delay(100);

    // ------------------------------------------------------------------------
    // Step 1: Wake device and select PLL clock source
    // ------------------------------------------------------------------------
    // Default register value after reset is 0x40:
    // SLEEP bit set, internal oscillator selected.
    //
    // Writing 0x01:
    // - clears sleep bit
    // - selects PLL from X-axis gyro for improved clock stability
    if (!I2CManager::writeReg(_addr,
                              MPU_REG_PWR_MGMT_1,
                              MPU_CLK_PLL_XGYRO)) {
        LOG_E("MPU6050", "Failed to configure PWR_MGMT_1");
        return false;
    }

    delay(30);

    // Verify write
    uint8_t pwr = I2CManager::readReg(_addr, MPU_REG_PWR_MGMT_1);
    if (pwr != MPU_CLK_PLL_XGYRO) {
        LOG_E("MPU6050",
              "PWR_MGMT_1 verify failed — expected 0x%02X got 0x%02X",
              MPU_CLK_PLL_XGYRO,
              pwr);
        return false;
    }

    // ------------------------------------------------------------------------
    // Step 2: Configure Digital Low Pass Filter
    // ------------------------------------------------------------------------
    // DLPF = 44 Hz
    //
    // Benefits:
    // - removes high-frequency vibration noise
    // - preserves human motion / fall content
    // - changes gyro output rate from 8 kHz to 1 kHz
    if (!I2CManager::writeReg(_addr,
                              MPU_REG_CONFIG,
                              MPU_DLPF_44HZ)) {
        LOG_E("MPU6050", "Failed to configure DLPF");
        return false;
    }

    uint8_t config = I2CManager::readReg(_addr, MPU_REG_CONFIG);
    if (config != MPU_DLPF_44HZ) {
        LOG_E("MPU6050",
              "CONFIG verify failed — expected 0x%02X got 0x%02X",
              MPU_DLPF_44HZ,
              config);
        return false;
    }

    // ------------------------------------------------------------------------
    // Step 3: Configure sample rate divider
    // ------------------------------------------------------------------------
    // With DLPF enabled:
    // internal sample rate = 1000 Hz
    //
    // SampleRate = 1000 / (1 + SMPLRT_DIV)
    //
    // For 100 Hz:
    // SMPLRT_DIV = 9

    if (IMU_SAMPLE_HZ <= 0 || IMU_SAMPLE_HZ > 1000) {
        LOG_E("MPU6050",
              "Invalid IMU_SAMPLE_HZ = %d",
              IMU_SAMPLE_HZ);
        return false;
    }

    uint8_t smplrt_div = (uint8_t)((1000 / IMU_SAMPLE_HZ) - 1);

    if (!I2CManager::writeReg(_addr,
                              MPU_REG_SMPLRT_DIV,
                              smplrt_div)) {
        LOG_E("MPU6050", "Failed to configure sample rate divider");
        return false;
    }

    uint8_t actual_div = I2CManager::readReg(_addr,
                                             MPU_REG_SMPLRT_DIV);

    if (actual_div != smplrt_div) {
        LOG_E("MPU6050",
              "SMPLRT_DIV verify failed — expected 0x%02X got 0x%02X",
              smplrt_div,
              actual_div);
        return false;
    }

    // ------------------------------------------------------------------------
    // Step 4: Configure gyroscope full-scale range
    // ------------------------------------------------------------------------
    // ±500 °/s
    // Sensitivity = 65.5 LSB/°/s
    //
    // Good tradeoff:
    // - enough range for aggressive wrist movement
    // - better resolution than ±1000 or ±2000
    if (!I2CManager::writeReg(_addr,
                              MPU_REG_GYRO_CONFIG,
                              MPU_GYRO_FS_500)) {
        LOG_E("MPU6050", "Failed to configure gyro range");
        return false;
    }

    uint8_t gyro_cfg = I2CManager::readReg(_addr,
                                           MPU_REG_GYRO_CONFIG);

    if (gyro_cfg != MPU_GYRO_FS_500) {
        LOG_E("MPU6050",
              "GYRO_CONFIG verify failed — expected 0x%02X got 0x%02X",
              MPU_GYRO_FS_500,
              gyro_cfg);
        return false;
    }

    // ------------------------------------------------------------------------
    // Step 5: Configure accelerometer full-scale range
    // ------------------------------------------------------------------------
    // ±8 g
    // Sensitivity = 4096 LSB/g
    //
    // Chosen because wrist impacts can exceed ±4 g.
    if (!I2CManager::writeReg(_addr,
                              MPU_REG_ACCEL_CONFIG,
                              MPU_ACCEL_FS_8G)) {
        LOG_E("MPU6050", "Failed to configure accel range");
        return false;
    }

    uint8_t accel_cfg = I2CManager::readReg(_addr,
                                            MPU_REG_ACCEL_CONFIG);

    if (accel_cfg != MPU_ACCEL_FS_8G) {
        LOG_E("MPU6050",
              "ACCEL_CONFIG verify failed — expected 0x%02X got 0x%02X",
              MPU_ACCEL_FS_8G,
              accel_cfg);
        return false;
    }
// ------------------------------------------------------------------------
    // Step 6: Accelerometer DLPF — MPU-6500/9250 clone dies only
    // ------------------------------------------------------------------------
    // The genuine MPU-6050 filters accel and gyro together via CONFIG (0x1A).
    // The MPU-6500 die split them: its accel DLPF lives in ACCEL_CONFIG2 (0x1D),
    // which CONFIG does not touch. Left at its 0x00 default the accelerometer
    // runs at ~460 Hz bandwidth while the gyro sits at 44 Hz — a noisier accel
    // than intended, and a mismatched pair.
    //
    // On a real MPU-6050 0x1D is undocumented/reserved, so this write is inert.
    // Not fatal either way: never return false on this one.
    if (!I2CManager::writeReg(_addr,
                              MPU_REG_ACCEL_CONFIG2,
                              MPU_ACCEL_DLPF_44HZ)) {
        LOG_W("MPU6050", "ACCEL_CONFIG2 write failed — accel DLPF unchanged");
    } else {
        uint8_t accel_cfg2 = I2CManager::readReg(_addr, MPU_REG_ACCEL_CONFIG2);
        LOG_I("MPU6050", "ACCEL_CONFIG2 = 0x%02X %s",
              accel_cfg2,
              accel_cfg2 == MPU_ACCEL_DLPF_44HZ
                  ? "(accel DLPF 44 Hz — clone die)"
                  : "(register not implemented — genuine MPU-6050)");
    }
    // ------------------------------------------------------------------------
    // Interrupts intentionally disabled for polling-based bring-up.
    // ------------------------------------------------------------------------
    // Future:
    // writeReg(MPU_REG_INT_ENABLE, 0x01);
    // Enables data-ready interrupt.

    return true;
}

bool MPU6050Driver::readSample(ImuSample &out) {
    if (!_initialized) {
        LOG_E("MPU6050", "readSample() called before initialization");
        return false;
    }

    // ------------------------------------------------------------------------
    // Burst read 14 contiguous bytes starting at ACCEL_XOUT_H (0x3B)
    // ------------------------------------------------------------------------
    // Layout:
    // AX_H AX_L AY_H AY_L AZ_H AZ_L TEMP_H TEMP_L
    // GX_H GX_L GY_H GY_L GZ_H GZ_L
    //
    // Burst reads prevent axis skew between measurements.

    uint8_t buf[14];

    if (!I2CManager::readRegs(_addr,
                              MPU_REG_ACCEL_XOUT_H,
                              buf,
                              14)) {
        LOG_E("MPU6050", "Burst read failed");
        return false;
    }

    // ------------------------------------------------------------------------
    // Reconstruct signed 16-bit big-endian values
    // ------------------------------------------------------------------------
    int16_t raw_ax = (int16_t)((buf[0]  << 8) | buf[1]);
    int16_t raw_ay = (int16_t)((buf[2]  << 8) | buf[3]);
    int16_t raw_az = (int16_t)((buf[4]  << 8) | buf[5]);

    // Temperature currently unused

    int16_t raw_gx = (int16_t)((buf[8]  << 8) | buf[9]);
    int16_t raw_gy = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t raw_gz = (int16_t)((buf[12] << 8) | buf[13]);

    // ------------------------------------------------------------------------
    // Convert raw ADC counts to physical units
    // ------------------------------------------------------------------------
    // Accelerometer:
    // ±8 g => 4096 LSB/g
    //
    // Gyroscope:
    // ±500 °/s => 65.5 LSB/°/s

    out.ax = raw_ax / MPU_ACCEL_SENS;
    out.ay = raw_ay / MPU_ACCEL_SENS;
    out.az = raw_az / MPU_ACCEL_SENS;

    out.gx = raw_gx / MPU_GYRO_SENS;
    out.gy = raw_gy / MPU_GYRO_SENS;
    out.gz = raw_gz / MPU_GYRO_SENS;

    out.timestamp_ms = millis();

    // ------------------------------------------------------------------------
    // Optional debug logging
    // ------------------------------------------------------------------------
    // Uncomment during bring-up.
    /*
    LOG_D("MPU6050",
          "A[g] %.2f %.2f %.2f | G[dps] %.2f %.2f %.2f",
          out.ax,
          out.ay,
          out.az,
          out.gx,
          out.gy,
          out.gz);
    */

    return true;
}

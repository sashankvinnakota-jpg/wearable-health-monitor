#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// ─────────────────────────────────────────────────────────────────────────────
// MPU-6050 register map — InvenSense datasheet §4 (register map document)
// Only registers needed for current firmware are defined here.
// Add others as features are implemented.
// ─────────────────────────────────────────────────────────────────────────────

// Configuration registers
#define MPU_REG_SMPLRT_DIV      0x19    // sample rate divider
#define MPU_REG_CONFIG          0x1A    // DLPF and FSYNC config
#define MPU_REG_GYRO_CONFIG     0x1B    // gyro full-scale range
#define MPU_REG_ACCEL_CONFIG    0x1C    // accel full-scale range

// Interrupt registers (defer use until interrupt-driven sampling is needed)
#define MPU_REG_INT_PIN_CFG     0x37
#define MPU_REG_INT_ENABLE      0x38
#define MPU_REG_INT_STATUS      0x3A

// Data registers — 14 contiguous bytes starting here
// Layout: AX_H AX_L  AY_H AY_L  AZ_H AZ_L  TEMP_H TEMP_L  GX_H GX_L  GY_H GY_L  GZ_H GZ_L
#define MPU_REG_ACCEL_XOUT_H    0x3B

// Power management
#define MPU_REG_PWR_MGMT_1      0x6B
#define MPU_REG_PWR_MGMT_2      0x6C

// Device identity — read first to confirm the device is present and responding
#define MPU_REG_WHO_AM_I        0x75
#define MPU_WHO_AM_I_EXPECTED   0x68    // always 0x68 regardless of I2C address

// ── PWR_MGMT_1 values ─────────────────────────────────────────────────────────
// CLKSEL = 001: PLL from X-axis gyro — more stable than internal 8 MHz RC oscillator
// SLEEP bit (bit 6) = 0: device awake
#define MPU_CLK_PLL_XGYRO       0x01

// ── CONFIG: DLPF_CFG values ───────────────────────────────────────────────────
// Setting DLPF ≠ 0 sets gyro output rate to 1 kHz (denominator for SMPLRT_DIV).
// 44 Hz bandwidth passes all fall/motion content while rejecting high-freq noise.
#define MPU_DLPF_260HZ          0x00    // no DLPF — gyro output rate = 8 kHz
#define MPU_DLPF_184HZ          0x01    // gyro output rate = 1 kHz
#define MPU_DLPF_94HZ           0x02
#define MPU_DLPF_44HZ           0x03    // ← chosen: 44 Hz BW, 1 kHz gyro output rate
#define MPU_DLPF_21HZ           0x04

// ── GYRO_CONFIG: FS_SEL bits [4:3] ───────────────────────────────────────────
#define MPU_GYRO_FS_250         0x00    // ±250  °/s  → 131.0 LSB/°/s
#define MPU_GYRO_FS_500         0x08    // ±500  °/s  →  65.5 LSB/°/s  ← chosen
#define MPU_GYRO_FS_1000        0x10    // ±1000 °/s  →  32.8 LSB/°/s
#define MPU_GYRO_FS_2000        0x18    // ±2000 °/s  →  16.4 LSB/°/s

// ── ACCEL_CONFIG: AFS_SEL bits [4:3] ─────────────────────────────────────────
#define MPU_ACCEL_FS_2G         0x00    // ±2  g  → 16384 LSB/g
#define MPU_ACCEL_FS_4G         0x08    // ±4  g  →  8192 LSB/g
#define MPU_ACCEL_FS_8G         0x10    // ±8  g  →  4096 LSB/g  ← chosen (fall headroom)
#define MPU_ACCEL_FS_16G        0x18    // ±16 g  →  2048 LSB/g

// ── ACCEL_CONFIG2 (0x1D) — MPU-6500/9250 dies only ───────────────────────────
// The genuine MPU-6050 filters accel and gyro together via CONFIG (0x1A).
// The MPU-6500 die split them out: leaving this at its 0x00 default runs the
// accelerometer at ~460 Hz bandwidth while the gyro sits at 44 Hz.
// On a real MPU-6050 this register is undocumented and the write is inert.
#define MPU_REG_ACCEL_CONFIG2   0x1D
#define MPU_ACCEL_DLPF_44HZ     0x03

// Sensitivity denominators for chosen full-scale ranges
#define MPU_ACCEL_SENS          4096.0f     // LSB/g   for ±8 g
#define MPU_GYRO_SENS           65.5f       // LSB/°/s for ±500 °/s

// ─────────────────────────────────────────────────────────────────────────────
// ImuSample — one reading from the MPU-6050
// ─────────────────────────────────────────────────────────────────────────────
struct ImuSample {
    float    ax, ay, az;        // acceleration in g  (1 g ≈ 9.81 m/s²)
    float    gx, gy, gz;        // angular velocity in °/s
    uint32_t timestamp_ms;      // millis() at time of read

    // Euclidean magnitude of acceleration vector in g
    // At rest: should be ≈ 1.0 g (gravity). Use for fall threshold comparison.
    float accelMagnitude() const {
        return sqrtf(ax*ax + ay*ay + az*az);
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// MPU6050Driver
// ─────────────────────────────────────────────────────────────────────────────
class MPU6050Driver {
public:
    explicit MPU6050Driver(uint8_t i2c_addr);

    bool    begin();                        // init and configure; returns false if not found
    bool    isConnected();                  // WHO_AM_I check
    bool    readSample(ImuSample &out);     // burst read all axes

    // TODO (Phase 2): enableDataReadyInt(uint8_t gpio_pin) — interrupt-driven sampling
    // TODO (Phase 2): readTemperatureCelsius() — raw/340 + 36.53
    // TODO (Phase 3): setOffsets(int16_t ax, int16_t ay, int16_t az, ...)
    // TODO (Phase 3): motionDetectInterrupt(uint8_t threshold_mg, uint8_t duration_ms)

private:
    uint8_t _addr;
    bool    _initialized;

    bool    _wakeAndConfigure();
};

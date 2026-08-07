#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// selftest.h — boot-time hardware validation
//
// Place at: src/test/selftest.h
//
// Runs a fixed battery of checks against the live hardware and prints a
// PASS/FAIL table to serial. Call AFTER sensorMgr.begin() but BEFORE
// sensorMgr.startTasks() — the tests need exclusive, deterministic timing on
// the I2C bus.
//
// The device must be lying still on a flat surface for the IMU tests to mean
// anything. Tests that need a missing sensor are reported SKIP, not FAIL.
// ─────────────────────────────────────────────────────────────────────────────

#include <stdint.h>
#include "../sensors/mpu6050_driver.h"
#include "../sensors/max30102_driver.h"

// ── Pass/fail thresholds ──────────────────────────────────────────────────────
// Accel magnitude at rest must equal gravity within this tolerance.
// 0.05 g is loose on purpose: it catches a broken axis or wrong FS_SEL scaling,
// not the small offset error that calibration exists to remove.
#define ST_GRAVITY_TOL_G        0.05f

// Standard deviation of |a| at rest — the accelerometer noise floor.
// MPU-6050 at ±8 g with a 44 Hz DLPF should sit well under 0.02 g.
// A clone die with ACCEL_CONFIG2 left at default (460 Hz BW) will be noisier.
#define ST_NOISE_MAX_G          0.020f

// Gyro bias at rest. Anything past 5 °/s is a bad part; past 2 °/s just needs
// calibrating out before it integrates into meaningful drift.
#define ST_GYRO_BIAS_MAX_DPS    5.0f
#define ST_GYRO_BIAS_WARN_DPS   2.0f

// Pulse Sensor Amped idles at roughly mid-rail. Railed at 0 or 4095 means the
// signal pin is disconnected, shorted, or the module is unpowered.
#define ST_PULSE_MIN_COUNT      300
#define ST_PULSE_MAX_COUNT      3800

// I2C stress: fraction of reads allowed to fail. Anything above zero on a
// short breadboard run is a real signal-integrity problem, not bad luck.
#define ST_I2C_MAX_FAIL_PCT     0.0f

// Sample counts
#define ST_IMU_SAMPLES          300     // 3 s at 100 Hz
#define ST_PULSE_SAMPLES        300
#define ST_I2C_STRESS_READS     500

// ─────────────────────────────────────────────────────────────────────────────
struct SelfTestResults {
    // Accelerometer
    bool  imu_present;
    bool  gravity_ok;
    bool  noise_ok;
    float accel_mag_mean;       // g
    float accel_mag_std;        // g
    float ax_mean, ay_mean, az_mean;

    // Gyroscope
    bool  gyro_bias_ok;
    float gx_bias, gy_bias, gz_bias;    // °/s

    // Timing
    float imu_read_hz;          // achievable read rate, not the scheduled rate
    float imu_read_us;          // mean duration of one 14-byte burst read

    // I2C integrity
    int   i2c_imu_fails;
    int   i2c_ppg_fails;
    bool  i2c_ok;

    // Analog pulse sensor
    bool  pulse_ok;
    int   pulse_mean, pulse_min, pulse_max;
    int   pulse_mv;             // calibrated millivolts

    // MAX30102
    bool  ppg_present;
    bool  ppg_ok;
    int   ppg_sample_count;
    uint32_t ppg_red_mean, ppg_ir_mean;

    bool allCriticalPassed() const;
};

// Runs every test and prints the report. Returns the filled result struct so
// callers can gate startup on it if they want to.
SelfTestResults runSelfTest(MPU6050Driver &imu,
                            MAX30102Driver &ppg,
                            bool imu_present,
                            bool ppg_present);

#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// imu_calibration.h — six-position accelerometer calibration + gyro bias
//
// Place at: src/test/imu_calibration.h
//
// WHY SIX POSITIONS
// A single resting measurement cannot separate a zero offset from a scale
// error: both show up as "not quite 1.000 g". Measuring each axis pointing up
// AND down gives two equations per axis, which solves for both:
//
//     offset = (a_up + a_down) / 2        <- the midpoint that should be 0 g
//     scale  = 2 / (a_up - a_down)        <- the span that should be 2 g
//
// Corrected reading:  a = (a_raw - offset) * scale
//
// Run this once per physical board. The numbers are per-device, not per-model,
// so they do not transfer if you swap the module.
// ─────────────────────────────────────────────────────────────────────────────

#include "../sensors/mpu6050_driver.h"

#define CAL_SAMPLES_PER_POS     400     // 4 s at 100 Hz — averages out noise
#define CAL_SETTLE_MS           1000    // let the board stop wobbling after you place it

struct ImuCalibration {
    float accel_offset[3];      // g,     subtract before scaling
    float accel_scale[3];       // unitless multiplier, ~1.00
    float gyro_bias[3];         // °/s,   subtract

    bool  valid;
};

// Interactive routine — prompts over serial, waits for ENTER between positions.
// Blocks until finished. Call before startTasks(); nothing else may touch I2C.
ImuCalibration runImuCalibration(MPU6050Driver &imu);

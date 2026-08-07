// ─────────────────────────────────────────────────────────────────────────────
// selftest.cpp — boot-time hardware validation
// Place at: src/test/selftest.cpp
// ─────────────────────────────────────────────────────────────────────────────

#include "selftest.h"
#include "../drivers/i2c_manager.h"
#include "../config/pins.h"
#include "../config/board_config.h"
#include "../utils/logger.h"
#include <Arduino.h>
#include <math.h>

// ── local helpers ─────────────────────────────────────────────────────────────
namespace {

const char *mark(bool ok)      { return ok ? "PASS" : "FAIL"; }
const char *markSkip()         { return "SKIP"; }

// Hammer one register and count how many reads fail. This is the single most
// useful bus-health number you can get: it separates "wired wrong" from
// "wired right but the signal is marginal".
int i2cStress(uint8_t addr, uint8_t reg, int reads) {
    int fails = 0;
    uint8_t v = 0;
    for (int i = 0; i < reads; i++) {
        if (!I2CManager::readReg(addr, reg, &v)) fails++;
    }
    return fails;
}

void rule() {
    Serial.println(F("  ---------------------------------------------------------------"));
}

}  // namespace

bool SelfTestResults::allCriticalPassed() const {
    // A missing MAX30102 is not fatal — the rest of the system is still valid.
    if (!imu_present) return false;
    return gravity_ok && noise_ok && gyro_bias_ok && i2c_ok && pulse_ok;
}

// ─────────────────────────────────────────────────────────────────────────────
SelfTestResults runSelfTest(MPU6050Driver &imu,
                            MAX30102Driver &ppg,
                            bool imu_present,
                            bool ppg_present) {
    SelfTestResults r = {};
    r.imu_present = imu_present;
    r.ppg_present = ppg_present;

    Serial.println();
    Serial.println(F("  ==============================================================="));
    Serial.println(F("   SELF TEST — keep the device STILL on a flat surface"));
    Serial.println(F("  ==============================================================="));
    delay(1500);   // let the user stop touching it

    // ── 1. Accelerometer + gyro statistics at rest ────────────────────────────
    if (imu_present) {
        double sum_mag = 0, sum_mag_sq = 0;
        double sum_ax = 0, sum_ay = 0, sum_az = 0;
        double sum_gx = 0, sum_gy = 0, sum_gz = 0;
        uint32_t read_us_total = 0;
        int good = 0;

        for (int i = 0; i < ST_IMU_SAMPLES; i++) {
            ImuSample s;
            uint32_t t0 = micros();
            bool ok = imu.readSample(s);
            read_us_total += (micros() - t0);

            if (ok) {
                float m = s.accelMagnitude();
                sum_mag    += m;
                sum_mag_sq += (double)m * m;
                sum_ax += s.ax;  sum_ay += s.ay;  sum_az += s.az;
                sum_gx += s.gx;  sum_gy += s.gy;  sum_gz += s.gz;
                good++;
            }
            delay(10);   // ~100 Hz
        }

        if (good > 1) {
            double mean = sum_mag / good;
            double var  = (sum_mag_sq / good) - (mean * mean);
            if (var < 0) var = 0;               // guard float cancellation

            r.accel_mag_mean = (float)mean;
            r.accel_mag_std  = (float)sqrt(var);
            r.ax_mean = (float)(sum_ax / good);
            r.ay_mean = (float)(sum_ay / good);
            r.az_mean = (float)(sum_az / good);
            r.gx_bias = (float)(sum_gx / good);
            r.gy_bias = (float)(sum_gy / good);
            r.gz_bias = (float)(sum_gz / good);

            r.imu_read_us = (float)read_us_total / ST_IMU_SAMPLES;
            r.imu_read_hz = (r.imu_read_us > 0) ? (1000000.0f / r.imu_read_us) : 0;

            r.gravity_ok = fabsf(r.accel_mag_mean - 1.0f) <= ST_GRAVITY_TOL_G;
            r.noise_ok   = r.accel_mag_std <= ST_NOISE_MAX_G;
            r.gyro_bias_ok =
                fabsf(r.gx_bias) < ST_GYRO_BIAS_MAX_DPS &&
                fabsf(r.gy_bias) < ST_GYRO_BIAS_MAX_DPS &&
                fabsf(r.gz_bias) < ST_GYRO_BIAS_MAX_DPS;
        }
    }

    // ── 2. I2C bus integrity ──────────────────────────────────────────────────
    if (imu_present) {
        r.i2c_imu_fails = i2cStress(ADDR_MPU6050, MPU_REG_WHO_AM_I, ST_I2C_STRESS_READS);
    }
    // Always stress the PPG address — if the chip ACKs but reads fail, this is
    // the number that proves it, and it is exactly the bus-timing symptom.
    r.i2c_ppg_fails = i2cStress(ADDR_MAX30102, MAX_REG_PART_ID, ST_I2C_STRESS_READS);

    float imu_fail_pct = (r.i2c_imu_fails * 100.0f) / ST_I2C_STRESS_READS;
    r.i2c_ok = (imu_fail_pct <= ST_I2C_MAX_FAIL_PCT);

    // ── 3. Analog pulse sensor ────────────────────────────────────────────────
    {
        long sum = 0;
        int  mn = 4095, mx = 0;
        for (int i = 0; i < ST_PULSE_SAMPLES; i++) {
            int v = analogRead(PIN_PULSE_SENSOR);
            sum += v;
            if (v < mn) mn = v;
            if (v > mx) mx = v;
            delay(5);
        }
        r.pulse_mean = (int)(sum / ST_PULSE_SAMPLES);
        r.pulse_min  = mn;
        r.pulse_max  = mx;
        r.pulse_mv   = (int)analogReadMilliVolts(PIN_PULSE_SENSOR);
        r.pulse_ok   = (r.pulse_mean > ST_PULSE_MIN_COUNT &&
                        r.pulse_mean < ST_PULSE_MAX_COUNT);
    }

    // ── 4. MAX30102 FIFO ──────────────────────────────────────────────────────
    if (ppg_present) {
        uint64_t sum_red = 0, sum_ir = 0;
        int total = 0;
        for (int i = 0; i < 20; i++) {         // ~1 s of draining
            PpgSample batch[8];
            int n = ppg.readFifo(batch, 8);
            for (int j = 0; j < n; j++) {
                sum_red += batch[j].red;
                sum_ir  += batch[j].ir;
            }
            total += n;
            delay(50);
        }
        r.ppg_sample_count = total;
        if (total > 0) {
            r.ppg_red_mean = (uint32_t)(sum_red / total);
            r.ppg_ir_mean  = (uint32_t)(sum_ir / total);
        }
        // Expect ~100 samples in 1 s. Non-zero counts with a live LED is the bar.
        r.ppg_ok = (total > 50) && (r.ppg_ir_mean > 0);
    }

    // ── Report ────────────────────────────────────────────────────────────────
    Serial.println();
    Serial.println(F("  SELF TEST RESULTS"));
    rule();

    if (imu_present) {
        Serial.printf("   [%s] Gravity      |a| = %.4f g   (target 1.000 +/- %.3f)\n",
                      mark(r.gravity_ok), r.accel_mag_mean, ST_GRAVITY_TOL_G);
        Serial.printf("          axes       ax=%+.4f  ay=%+.4f  az=%+.4f g\n",
                      r.ax_mean, r.ay_mean, r.az_mean);
        Serial.printf("   [%s] Noise floor  sd(|a|) = %.4f g   (max %.3f)\n",
                      mark(r.noise_ok), r.accel_mag_std, ST_NOISE_MAX_G);
        Serial.printf("   [%s] Gyro bias    gx=%+.2f  gy=%+.2f  gz=%+.2f deg/s  (max %.1f)\n",
                      mark(r.gyro_bias_ok), r.gx_bias, r.gy_bias, r.gz_bias,
                      ST_GYRO_BIAS_MAX_DPS);
        Serial.printf("          read time  %.0f us/sample  ->  %.0f Hz ceiling (need %d Hz)\n",
                      r.imu_read_us, r.imu_read_hz, IMU_SAMPLE_HZ);
    } else {
        Serial.printf("   [%s] IMU tests    — MPU6050 not initialised\n", markSkip());
    }

    rule();
    Serial.printf("   [%s] I2C 0x%02X     %d/%d reads failed (%.1f%%)\n",
                  imu_present ? mark(r.i2c_ok) : markSkip(),
                  ADDR_MPU6050, r.i2c_imu_fails, ST_I2C_STRESS_READS, imu_fail_pct);
    Serial.printf("   [%s] I2C 0x%02X     %d/%d reads failed (%.1f%%)\n",
                  r.i2c_ppg_fails == 0 ? "PASS" : "FAIL",
                  ADDR_MAX30102, r.i2c_ppg_fails, ST_I2C_STRESS_READS,
                  (r.i2c_ppg_fails * 100.0f) / ST_I2C_STRESS_READS);

    rule();
    Serial.printf("   [%s] Pulse sensor mean=%d  min=%d  max=%d  (%d mV)\n",
                  mark(r.pulse_ok), r.pulse_mean, r.pulse_min, r.pulse_max, r.pulse_mv);
    Serial.printf("          swing      p-p = %d counts %s\n",
                  r.pulse_max - r.pulse_min,
                  (r.pulse_max - r.pulse_min) < 20
                      ? "<- flat: no finger, or sensor not powered"
                      : "");

    rule();
    if (ppg_present) {
        Serial.printf("   [%s] MAX30102     %d samples in 1 s   red=%lu  ir=%lu\n",
                      mark(r.ppg_ok), r.ppg_sample_count,
                      (unsigned long)r.ppg_red_mean, (unsigned long)r.ppg_ir_mean);
    } else {
        Serial.printf("   [%s] MAX30102     — driver init failed, see boot log\n", markSkip());
    }

    rule();
    Serial.printf("   OVERALL: %s\n",
                  r.allCriticalPassed() ? "PASS" : "ATTENTION NEEDED");
    Serial.println(F("  ==============================================================="));
    Serial.println();
    Serial.flush();

    return r;
}

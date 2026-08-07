#pragma once
#include <stdint.h>

// ─────────────────────────────────────────────────────────────────────────────
// Board-level constants — hardware facts derived from datasheets
// Change these if swapping sensors; do not scatter magic numbers in drivers
// ─────────────────────────────────────────────────────────────────────────────

// ── I2C bus ───────────────────────────────────────────────────────────────────
#define I2C_FREQ_HZ             400000UL    // 400 kHz Fast Mode

// ── Device I2C addresses ──────────────────────────────────────────────────────
#define ADDR_MPU6050            0x68        // AD0 tied to GND on GY-521
#define ADDR_MAX30102           0x57        // fixed, not configurable
#define ADDR_SSD1306            0x3C        // SA0 = GND; use 0x3D if SA0 = VCC

// ── Sampling rates ────────────────────────────────────────────────────────────
// IMU at 100 Hz: Bourke & Lyons fall detection validated at this rate
// PPG at 100 Hz: sufficient for HR (< 3 Hz signal) and SpO2 ratio
#define IMU_SAMPLE_HZ           100
#define PPG_SAMPLE_HZ           100
#define PULSE_SAMPLE_HZ         100

// ── Buffer sizing ──────────────────────────────────────────────────────────────
// 2 s of data at 100 Hz — enough for fall event capture window
// ImuSample  ≈ 28 bytes  → 200 × 28 =  5.6 KB
// PpgSample  ≈ 12 bytes  → 200 × 12 =  2.4 KB
// Total ring buffers: ~8 KB out of 520 KB SRAM — acceptable
#define IMU_RING_BUF_SIZE       200
#define PPG_RING_BUF_SIZE       200
#define PULSE_RING_BUF_SIZE     200

// ── FreeRTOS tasks ────────────────────────────────────────────────────────────
// All tasks pinned to Core 1 (Core 0 reserved for WiFi/BLE radio)
#define TASK_CORE               1

#define STACK_SENSOR_IMU        3072        // words
#define STACK_SENSOR_PPG        3072
#define STACK_APP               2048

// Priority: 1 = lowest, configMAX_PRIORITIES-1 = highest
// Sensor tasks higher than app so sampling is not delayed by serial output
#define PRIORITY_SENSOR         3
#define PRIORITY_APP            1

// ── Serial debug ──────────────────────────────────────────────────────────────
#define SERIAL_BAUD             115200
#define LOG_ENABLED             true
#define LOG_MIN_LEVEL           0           // 0=DEBUG, 1=INFO, 2=WARN, 3=ERROR

// ── Firmware version ──────────────────────────────────────────────────────────
#define FW_VERSION              "0.1.0-dev"

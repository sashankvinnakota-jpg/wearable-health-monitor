#pragma once
#include <stdint.h>
#include <stdbool.h>

// ─────────────────────────────────────────────────────────────────────────────
// MAX30102 register map — Maxim Integrated datasheet rev 3, Table 1
// ─────────────────────────────────────────────────────────────────────────────

// Interrupt status and enable
#define MAX_REG_INT_STATUS1     0x00    // read to clear; bit 7=A_FULL, bit 6=PPG_RDY
#define MAX_REG_INT_STATUS2     0x01
#define MAX_REG_INT_ENABLE1     0x02    // bit 7=A_FULL_EN, bit 6=PPG_RDY_EN
#define MAX_REG_INT_ENABLE2     0x03

// FIFO control
#define MAX_REG_FIFO_WR_PTR     0x04    // next write position (0–31)
#define MAX_REG_OVF_COUNTER     0x05    // samples lost to overflow since last clear
#define MAX_REG_FIFO_RD_PTR     0x06    // next read position (0–31)
#define MAX_REG_FIFO_DATA       0x07    // reading here auto-increments RD_PTR

// Configuration
#define MAX_REG_FIFO_CONFIG     0x08
#define MAX_REG_MODE_CONFIG     0x09
#define MAX_REG_SPO2_CONFIG     0x0A

// LED current (0x00 = 0 mA, 0xFF = 51 mA, step = 0.2 mA per LSB)
#define MAX_REG_LED1_PA         0x0C    // Red  LED pulse amplitude
#define MAX_REG_LED2_PA         0x0D    // IR   LED pulse amplitude

// Die temperature (optional; useful for SpO2 calibration compensation)
#define MAX_REG_TEMP_INT        0x1F
#define MAX_REG_TEMP_FRAC       0x20
#define MAX_REG_TEMP_CONFIG     0x21    // bit 0: trigger one-shot temp read

// Device identity — Part ID always 0x15; Revision ID varies
#define MAX_REG_PART_ID         0xFF
#define MAX_PART_ID_EXPECTED    0x15

// ── MODE_CONFIG values ────────────────────────────────────────────────────────
#define MAX_MODE_HR             0x02    // Heart-rate only: LED1 (Red) active
#define MAX_MODE_SPO2           0x03    // SpO2: LED1 (Red) + LED2 (IR) active
#define MAX_RESET_BIT           0x40    // bit 6: software reset, self-clearing
#define MAX_SHDN_BIT            0x80    // bit 7: shutdown (low-power standby)

// ── FIFO_CONFIG register breakdown ───────────────────────────────────────────
// Bits [7:5] SMP_AVE: hardware averaging before FIFO write
//   000=1 (no avg), 001=2, 010=4, 011=8, 100=16, 101/110/111=32
// Bit  [4]   FIFO_ROLLOVER_EN: 1 = overwrite oldest sample when FIFO full
// Bits [3:0] FIFO_A_FULL: almost-full interrupt fires when this many slots remain
// Chosen: no averaging (filter in software), rollover enabled, threshold=15
// Value: 0b000_1_1111 = 0x1F
#define MAX_FIFO_CONFIG_VAL     0x1F

// ── SPO2_CONFIG register breakdown ───────────────────────────────────────────
// Bits [6:5] SPO2_ADC_RGE: ADC full-scale range
//   00=2048 nA, 01=4096 nA, 10=8192 nA, 11=16384 nA
// Bits [4:2] SPO2_SR: sample rate
//   000=50 sps, 001=100 sps, 010=200 sps, 011=400 sps, 100=800 sps, 101=1000 sps
// Bits [1:0] LED_PW: LED pulse width → determines ADC resolution
//   00=69 µs/15-bit, 01=118 µs/16-bit, 10=215 µs/17-bit, 11=411 µs/18-bit
//
// Chosen: 4096 nA range | 100 sps | 411 µs (18-bit, maximum SNR)
// Value: 0b0_01_001_11 = 0x27
#define MAX_SPO2_CONFIG_VAL     0x27

// ── LED current ───────────────────────────────────────────────────────────────
// Start conservative at wrist. Increase in steps of 0x10 (~3 mA) if AC/DC ratio
// (AC component / DC mean) falls below ~0.5%.
// 0x1F = 31 × 0.2 mA = 6.2 mA  ← conservative start
// 0x7F = 127 × 0.2 mA = 25.4 mA ← strong; try if signal too weak
#define MAX_LED_CURRENT_INIT    0x1F

// FIFO geometry
#define MAX_FIFO_DEPTH          32
#define MAX_BYTES_PER_SAMPLE    6       // 3 bytes Red + 3 bytes IR in SpO2 mode
#define MAX_RAW_MASK            0x3FFFF // 18-bit data mask

// ─────────────────────────────────────────────────────────────────────────────
// PpgSample — one SpO2-mode reading (Red + IR channels)
// ─────────────────────────────────────────────────────────────────────────────
struct PpgSample {
    uint32_t red;           // 18-bit raw ADC count (max 262143)
    uint32_t ir;            // 18-bit raw ADC count
    uint32_t timestamp_ms;

    // Sanity check: counts should be > 50000 with a finger present.
    // If < 10000, LED current is too low or sensor placement is wrong.
    bool isValid() const { return red > 10000 && ir > 10000; }
};

// ─────────────────────────────────────────────────────────────────────────────
// MAX30102Driver
// ─────────────────────────────────────────────────────────────────────────────
class MAX30102Driver {
public:
    explicit MAX30102Driver(uint8_t i2c_addr);

    bool    begin();
    bool    isConnected();

    // Drain all available FIFO samples into buf[]. Returns count written.
    // Call at ≥20 Hz to prevent FIFO overflow (FIFO holds 32 samples at 100 sps → 320 ms).
    int     readFifo(PpgSample *buf, int max_samples);

    // Adjust LED drive current after initial bring-up signal quality check
    void    setLedCurrent(uint8_t red_pa, uint8_t ir_pa);

    uint8_t getOverflowCount();

    // TODO (Phase 2): enableFifoInterrupt() — A_FULL interrupt to GPIO
    // TODO (Phase 2): readDieTemperature() — for SpO2 calibration
    // TODO (Phase 3): setMode(uint8_t mode) — switch between HR and SpO2
    // TODO (Phase 3): setAveragingSamples(uint8_t n) — hardware averaging

private:
    uint8_t _addr;
    bool    _initialized;

    bool    _softwareReset();
    bool    _configure();
    void    _clearFifo();
};

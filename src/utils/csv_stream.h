#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// csv_stream.h — full-rate CSV telemetry over serial
//
// Place at: src/utils/csv_stream.h
//
// THE PROBLEM THIS SOLVES
// loop() prints once every 500 ms. A heartbeat is 1–2 Hz. Sampling a 1–2 Hz
// signal at 2 Hz is aliasing — the pulse waveform is being collected at 100 Hz
// in the sensor task and then thrown away before you ever see it.
//
// This emits every sample, tagged, at full rate, in a form a script can parse.
//
// LINE FORMAT — all data lines start with '$' so they never collide with the
// human-readable '[I][TAG]' log output:
//
//   $H,<header text>                      once at startup
//   $I,seq,t_ms,ax,ay,az,gx,gy,gz,pulse   one per IMU sample   (100 Hz)
//   $P,seq,t_ms,red,ir                    one per PPG sample   (100 Hz)
//
// BANDWIDTH
// An $I line is ~70 bytes. At 100 Hz that is 7 kB/s = 70 kbaud of the 115200
// available, before PPG. Raise monitor_speed AND SERIAL_BAUD to 921600 if you
// stream both sensors at once, or you will silently drop lines.
// ─────────────────────────────────────────────────────────────────────────────

#include <stdint.h>
#include "../sensors/mpu6050_driver.h"
#include "../sensors/max30102_driver.h"

// Master switch. Leave 0 for normal operation; set to 1 to capture a dataset.
// When enabled, raise LOG_MIN_LEVEL to 2 (WARN) in board_config.h so debug
// chatter does not interleave with the data.
#ifndef CSV_STREAM_ENABLED
#define CSV_STREAM_ENABLED   0
#endif

class CsvStream {
public:
    // Prints the header line. Call once, after Logger::begin().
    static void begin();

    // Call from SensorManager::_doImuRead(), once per sample.
    static void emitImu(const ImuSample &s, int pulse_raw);

    // Call from SensorManager::_doPpgRead(), once per sample in the batch.
    static void emitPpg(const PpgSample &s);

    // True while streaming — lets callers skip other printing.
    static bool active();

private:
    static uint32_t _imu_seq;
    static uint32_t _ppg_seq;
};

#if CSV_STREAM_ENABLED
  #define CSV_BEGIN()             CsvStream::begin()
  #define CSV_EMIT_IMU(s, p)      CsvStream::emitImu((s), (p))
  #define CSV_EMIT_PPG(s)         CsvStream::emitPpg((s))
#else
  #define CSV_BEGIN()             ((void)0)
  #define CSV_EMIT_IMU(s, p)      ((void)0)
  #define CSV_EMIT_PPG(s)         ((void)0)
#endif

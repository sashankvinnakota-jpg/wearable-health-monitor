// ─────────────────────────────────────────────────────────────────────────────
// csv_stream.cpp — full-rate CSV telemetry over serial
// Place at: src/utils/csv_stream.cpp
// ─────────────────────────────────────────────────────────────────────────────

#include "csv_stream.h"
#include <Arduino.h>

uint32_t CsvStream::_imu_seq = 0;
uint32_t CsvStream::_ppg_seq = 0;

bool CsvStream::active() {
    return CSV_STREAM_ENABLED != 0;
}

void CsvStream::begin() {
#if CSV_STREAM_ENABLED
    _imu_seq = 0;
    _ppg_seq = 0;
    // Two header lines so the capture script can self-describe each row type.
    Serial.println(F("$H,I,seq,t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,pulse_adc"));
    Serial.println(F("$H,P,seq,t_ms,red,ir"));
    Serial.flush();
#endif
}

void CsvStream::emitImu(const ImuSample &s, int pulse_raw) {
#if CSV_STREAM_ENABLED
    // Fixed 4 decimals: enough for 0.1 mg resolution, keeps the line short.
    // printf on ESP32 Arduino handles %f without the AVR dtostrf dance.
    Serial.printf("$I,%lu,%lu,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%d\n",
                  (unsigned long)_imu_seq++,
                  (unsigned long)s.timestamp_ms,
                  s.ax, s.ay, s.az,
                  s.gx, s.gy, s.gz,
                  pulse_raw);
#else
    (void)s; (void)pulse_raw;
#endif
}

void CsvStream::emitPpg(const PpgSample &s) {
#if CSV_STREAM_ENABLED
    Serial.printf("$P,%lu,%lu,%lu,%lu\n",
                  (unsigned long)_ppg_seq++,
                  (unsigned long)s.timestamp_ms,
                  (unsigned long)s.red,
                  (unsigned long)s.ir);
#else
    (void)s;
#endif
}

#include "sensor_manager.h"
#include "../app/app_state.h"
#include "../utils/logger.h"
#include "../utils/csv_stream.h"
#include "../config/pins.h"
#include <Arduino.h>

SensorManager::SensorManager()
    : _imu(ADDR_MPU6050),
      _ppg(ADDR_MAX30102),
      _imu_task_handle(nullptr),
      _ppg_task_handle(nullptr) {}

bool SensorManager::begin() {
    AppStatus &s = getAppStatus();

    s.health.imu_ok = _imu.begin();
    s.health.ppg_ok = _ppg.begin();

    // Pulse Sensor is purely analog — always available if ADC pin is wired
    analogSetPinAttenuation(PIN_PULSE_SENSOR, ADC_11db);  // 0–3.3 V input range
    s.health.pulse_ok = true;

    if (!s.health.imu_ok) LOG_E("SensorMgr", "MPU6050 failed — fall detection unavailable");
    if (!s.health.ppg_ok) LOG_E("SensorMgr", "MAX30102 failed — PPG/SpO2 unavailable");

    return s.health.imu_ok || s.health.ppg_ok;  // partial init is still useful
}

void SensorManager::startTasks() {
    xTaskCreatePinnedToCore(
        _imuTask, "IMU", STACK_SENSOR_IMU, this, PRIORITY_SENSOR, &_imu_task_handle, TASK_CORE);

    xTaskCreatePinnedToCore(
        _ppgTask, "PPG", STACK_SENSOR_PPG, this, PRIORITY_SENSOR, &_ppg_task_handle, TASK_CORE);

    LOG_I("SensorMgr", "Tasks started on core %d", TASK_CORE);
}

void SensorManager::stopTasks() {
    if (_imu_task_handle) { vTaskDelete(_imu_task_handle); _imu_task_handle = nullptr; }
    if (_ppg_task_handle) { vTaskDelete(_ppg_task_handle); _ppg_task_handle = nullptr; }
}

void SensorManager::_doImuRead() {
    ImuSample sample;
    if (!_imu.readSample(sample)) return;

    _imu_buf.push(sample);
    getAppStatus().last_accel_mag = sample.accelMagnitude();

    // Full-rate telemetry. pulse_raw is read earlier in the same task cycle,
    // so the two values in this row are from the same 10 ms window.
    CSV_EMIT_IMU(sample, getAppStatus().pulse_raw);

    // TODO (Phase 2): pass sample to fall detection state machine
}

void SensorManager::_doPpgRead() {
    PpgSample batch[8];
    int n = _ppg.readFifo(batch, 8);
    if (n <= 0) return;

    for (int i = 0; i < n; i++) {
        _ppg_buf.push(batch[i]);
        CSV_EMIT_PPG(batch[i]);
    }

    // Update display fields with most recent sample
    AppStatus &s = getAppStatus();
    s.last_ppg_red = batch[n - 1].red;
    s.last_ppg_ir  = batch[n - 1].ir;

    // TODO (Phase 2): push batch to HR / SpO2 processing window
}

void SensorManager::_doPulseSensorRead() {
    // 8-point running average reduces ESP32 ADC noise on the analog PPG signal
    static int avg_buf[8] = {};
    static int idx        = 0;
    static long sum       = 0;

    int raw  = analogRead(PIN_PULSE_SENSOR);
    sum     -= avg_buf[idx];
    avg_buf[idx] = raw;
    sum     += raw;
    idx      = (idx + 1) % 8;

    int smoothed = (int)(sum / 8);
    getAppStatus().pulse_raw = smoothed;
    _pulse_buf.push(smoothed);

    // TODO (Phase 2): peak detection on _pulse_buf window for HR estimate
}

void SensorManager::_imuTask(void *param) {
    SensorManager *self = static_cast<SensorManager *>(param);
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        // Pulse first, then IMU: _doImuRead() emits the CSV row containing both,
        // so the ADC value must already be current when it runs.
        self->_doPulseSensorRead();  // share task — both at 100 Hz
        self->_doImuRead();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000 / IMU_SAMPLE_HZ));
    }
}

void SensorManager::_ppgTask(void *param) {
    SensorManager *self = static_cast<SensorManager *>(param);
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        self->_doPpgRead();
        // Drain FIFO at 20 Hz — at 100 sps, FIFO fills in 320 ms; draining at 50 ms is safe
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(50));
    }
}

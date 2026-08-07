#pragma once
#include "mpu6050_driver.h"
#include "max30102_driver.h"
#include "../utils/ring_buffer.h"
#include "../config/board_config.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// ─────────────────────────────────────────────────────────────────────────────
// SensorManager
//
// Responsibility:
//   Owns both sensor driver instances and the FreeRTOS tasks that drive them.
//   Reads raw data, stores in ring buffers, updates AppStatus fields.
//   Does NOT run algorithms — algorithm layer reads ring buffers separately.
//
// Thread model:
//   _imuTask  runs at 100 Hz (vTaskDelayUntil) on Core 1
//   _ppgTask  drains FIFO at 20 Hz (FIFO holds 32 samples → 320 ms at 100 sps)
//   Pulse Sensor is read inside _imuTask (same timing, one ADC call)
// ─────────────────────────────────────────────────────────────────────────────

class SensorManager {
public:
    SensorManager();
    MPU6050Driver&  imu() { return _imu; }
    MAX30102Driver& ppg() { return _ppg; }
    bool    begin();            // init drivers, report health flags in AppStatus
    void    startTasks();       // launch FreeRTOS sensor tasks
    void    stopTasks();        // delete tasks (e.g., before sleep)

    // Ring buffer accessors — read by algorithm layer (Phase 2+).
    // THREAD SAFETY: sensor tasks write these buffers from Core 1.
    // Algorithm callers must suspend sensor tasks OR use a shared mutex before
    // calling copyWindow() — concurrent push()/copyWindow() is a data race.
    RingBuffer<ImuSample, IMU_RING_BUF_SIZE>&  imuBuffer()   { return _imu_buf; }
    RingBuffer<PpgSample, PPG_RING_BUF_SIZE>&  ppgBuffer()   { return _ppg_buf; }
    RingBuffer<int,       PULSE_RING_BUF_SIZE>& pulseBuffer() { return _pulse_buf; }

    // TODO (Phase 2): setImuSampleRate(uint16_t hz)
    // TODO (Phase 2): tunePpgLedCurrent(uint8_t red, uint8_t ir)
    // TODO (Phase 3): enableFallInterrupt()

private:
    MPU6050Driver   _imu;
    MAX30102Driver  _ppg;

    RingBuffer<ImuSample, IMU_RING_BUF_SIZE>  _imu_buf;
    RingBuffer<PpgSample, PPG_RING_BUF_SIZE>  _ppg_buf;
    RingBuffer<int,       PULSE_RING_BUF_SIZE> _pulse_buf;

    TaskHandle_t    _imu_task_handle;
    TaskHandle_t    _ppg_task_handle;

    // FreeRTOS task entry points must be static (no implicit 'this')
    static void     _imuTask(void *param);
    static void     _ppgTask(void *param);

    void            _doImuRead();
    void            _doPpgRead();
    void            _doPulseSensorRead();
};

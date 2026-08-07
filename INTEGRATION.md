# Validation suite — integration guide

Three tools, run in this order: **self-test → calibration → CSV capture**.
Each one depends on the previous one being clean.

## Where the files go

```
src/
├── test/
│   ├── selftest.h          selftest.cpp
│   └── imu_calibration.h   imu_calibration.cpp
└── utils/
    └── csv_stream.h        csv_stream.cpp

capture_and_plot.py         <- project root, runs on your PC
```

---

## Step 0 — three prerequisite patches

### 0a. Expose the drivers (`src/sensors/sensor_manager.h`)

The test code needs direct access to the driver objects. Add to the `public:`
section:

```cpp
    MPU6050Driver&  imu() { return _imu; }
    MAX30102Driver& ppg() { return _ppg; }
```

### 0b. Fix the clone accelerometer bandwidth (`src/sensors/mpu6050_driver.h`)

Your WHO_AM_I is 0x70 — an MPU-6500 die. Its accel DLPF lives in a register the
MPU-6050 doesn't have, so your accel is unfiltered at ~460 Hz while the gyro is
at 44 Hz. Add the register:

```cpp
#define MPU_REG_ACCEL_CONFIG2   0x1D    // MPU-6500/9250 only: accel DLPF
#define MPU_ACCEL_DLPF_44HZ     0x03
```

Then in `_wakeAndConfigure()`, after the ACCEL_CONFIG block, add:

```cpp
    // MPU-6500 clone dies only: match accel bandwidth to the gyro's 44 Hz.
    // Genuine MPU-6050 ignores this register, so the write is harmless.
    I2CManager::writeReg(_addr, MPU_REG_ACCEL_CONFIG2, MPU_ACCEL_DLPF_44HZ);
```

Do this **before** running the self-test — it directly changes the noise floor
the test measures.

### 0c. Build flags (`platformio.ini`)

```ini
build_flags =
    -DCORE_DEBUG_LEVEL=3
;   -DRUN_SELFTEST=1        ; uncomment for step 1
;   -DRUN_CALIBRATION=1     ; uncomment for step 2
;   -DCSV_STREAM_ENABLED=1  ; uncomment for step 3
```

Only ever enable **one** at a time.

---

## Step 1 — self-test

In `main.cpp`, add the includes:

```cpp
#include "test/selftest.h"
#include "test/imu_calibration.h"
#include "utils/csv_stream.h"
```

Then in `setup()`, between `sensorMgr.begin()` and `sensorMgr.startTasks()`:

```cpp
#ifdef RUN_SELFTEST
    runSelfTest(sensorMgr.imu(), sensorMgr.ppg(),
                getAppStatus().health.imu_ok,
                getAppStatus().health.ppg_ok);
#endif
```

Uncomment `-DRUN_SELFTEST=1`, flash, leave the board flat and still, open the
monitor. You get a PASS/FAIL table.

**The line that matters most for your current bug:**

```
[????] I2C 0x57     N/500 reads failed (x.x%)
```

You have a MAX30102 that ACKs its address but fails register reads. This
measures that failure rate directly. Run it at `I2C_FREQ_HZ = 400000`, note the
number, then re-run at `100000`. If the failure count drops to zero, it was bus
timing and you're done. If it stays at 500/500, it's the board's level shifting
and you need the STOP-then-read change in `readRegs()`.

Expected on a healthy board: gravity 1.00 ± 0.05 g, noise sd < 0.02 g, gyro bias
under 5 °/s, 0% I2C failures, pulse mean somewhere around 1900 counts.

---

## Step 2 — calibration

Turn off `RUN_SELFTEST`, turn on `RUN_CALIBRATION`, and add:

```cpp
#ifdef RUN_CALIBRATION
    runImuCalibration(sensorMgr.imu());
    while (true) delay(1000);   // calibration is terminal; reflash without the flag
#endif
```

It prompts for six orientations. Rest the board against a table edge or a book
corner for the vertical positions — held by hand, your numbers will be worse
than no calibration at all.

It prints a paste-ready block. Save it as `src/config/calibration.h`, then apply
it in `MPU6050Driver::readSample()`:

```cpp
    out.ax = (raw_ax / MPU_ACCEL_SENS - ACCEL_OFFSET_X) * ACCEL_SCALE_X;
    out.ay = (raw_ay / MPU_ACCEL_SENS - ACCEL_OFFSET_Y) * ACCEL_SCALE_Y;
    out.az = (raw_az / MPU_ACCEL_SENS - ACCEL_OFFSET_Z) * ACCEL_SCALE_Z;

    out.gx = raw_gx / MPU_GYRO_SENS - GYRO_BIAS_X;
    out.gy = raw_gy / MPU_GYRO_SENS - GYRO_BIAS_Y;
    out.gz = raw_gz / MPU_GYRO_SENS - GYRO_BIAS_Z;
```

Re-run the self-test afterwards. `|a|` should now read within a few mg of 1.000
and the gyro bias should be near zero. That is the proof the calibration worked.

---

## Step 3 — CSV capture and analysis

Add the hooks in `src/sensors/sensor_manager.cpp`.

In `_doImuRead()`, after `_imu_buf.push(sample)`:

```cpp
    CSV_EMIT_IMU(sample, getAppStatus().pulse_raw);
```

In `_doPpgRead()`, inside the batch loop:

```cpp
    for (int i = 0; i < n; i++) {
        _ppg_buf.push(batch[i]);
        CSV_EMIT_PPG(batch[i]);
    }
```

And `CSV_BEGIN();` in `setup()` right after `Logger::begin()`.

Then in `board_config.h` set `LOG_MIN_LEVEL` to `2` so debug chatter doesn't
interleave with the data. Enable `-DCSV_STREAM_ENABLED=1`, flash, and **close
the PlatformIO serial monitor** — only one program can hold the port.

```bash
pip install pyserial numpy matplotlib
python capture_and_plot.py --port COM5 --seconds 30
```

Keep a finger on the pulse sensor for the whole capture. You get a stats report,
a heart-rate estimate, and `capture_plot.png`.

### Reading the output

- **measured rate** should be ~100 Hz. Much lower means the sensor task is
  missing deadlines — most likely serial output blocking it, so raise the baud.
- **|a| sd** is your true noise floor. Under 0.02 g is healthy.
- **heart rate** is only meaningful when `peak/median` is above ~4. Below that
  the script says WEAK, and it means the spectrum has no dominant peak — you're
  looking at noise, not a pulse. Check finger pressure and sensor contact.
- Cross-check the bpm against a manual pulse count over 30 s, doubled. If those
  two agree within a few bpm, your signal chain is genuinely working.

The analysis runs offline too:

```bash
python capture_and_plot.py --replay session.csv
```

---

## Suggested order of attack

1. Patch 0b, run the self-test at 400 kHz — record the 0x57 failure rate.
2. Drop to 100 kHz, re-run — compare. This settles the MAX30102 question.
3. Calibrate the IMU once the bus is stable.
4. Capture 30 s with a finger on the sensor and confirm a plausible bpm.

Only after step 4 is it worth touching fall-detection thresholds — every
threshold in Bourke & Lyons is expressed in g and °/s, and those units don't
mean anything until the calibration above is applied.

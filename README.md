# Wearable Health Monitor


Firmware for an ESP32-based wrist-worn health monitor: accelerometer/gyro for
motion and fall detection, a PPG sensor for heart rate / SpO2, an analog pulse
sensor as a second heart-rate reference, and a small OLED for status.

The project is currently in the **hardware bring-up / validation phase** — the
sensor drivers, task scaffolding, and app state machine are in place, but the
signal-processing algorithms (heart rate, SpO2, fall detection) have not been
implemented yet. See [Status & Roadmap](#status--roadmap).

## Hardware

| Part | Role | Bus / Pin | I2C Address |
|---|---|---|---|
| ESP32 38-pin dev board | MCU | — | — |
| MPU-6050 (GY-521 breakout) | 6-axis IMU — motion / fall detection | I2C (GPIO21 SDA, GPIO22 SCL) | `0x68` |
| MAX30102 | PPG — heart rate / SpO2 | I2C (shared bus) | `0x57` |
| SSD1306 128×64 | OLED status display | I2C (shared bus) | `0x3C` |
| Pulse Sensor Amped | Analog green-LED HR reference | GPIO34 (ADC1‑CH6) | — |
| LiPo battery + divider | Power (Phase 3, not yet wired) | GPIO35 (ADC1‑CH7) | — |
| Status LED | Boot/error indicator | GPIO2 | — |

All three I2C devices share one bus. The GY-521 breakout already has 4.7 kΩ
pull-ups — don't add more. Full pin rationale is in
[src/config/pins.h](src/config/pins.h).

> **Note:** the MPU-6050 on this board identifies as `WHO_AM_I = 0x70`, which
> is actually an MPU-6500 die (common on cheap GY-521 clones). See
> [Known hardware quirks](#known-hardware-quirks) below.

## Architecture

```
setup()
 ├─ I2CManager::begin()         shared bus, mutex-protected
 ├─ i2c scan                    log every responding address
 ├─ SSD1306Display::begin()     boot screen
 ├─ PowerManager::begin()       USB-only stub for now
 └─ SensorManager::begin()      init MPU6050 + MAX30102, health flags in AppStatus
      └─ startTasks()           spawns two FreeRTOS tasks, pinned to Core 1

 IMU task  @ 100 Hz  — reads MPU6050 + analog pulse sensor, pushes ring buffers
 PPG task  @  20 Hz  — drains MAX30102 FIFO (32-sample depth @ 100 sps)

loop()  @ 2 Hz — serial heartbeat, display refresh, app-state transitions
```

- **`AppStatus`** ([src/app/app_state.h](src/app/app_state.h)) is a single
  global struct holding sensor health flags and the latest readings. Sensor
  tasks write it, `loop()` and the display read it. Fields are primitive
  types so writes are atomic on the Xtensa core — no mutex needed unless a
  non-primitive field is added.
- **`RingBuffer<T, N>`** ([src/utils/ring_buffer.h](src/utils/ring_buffer.h))
  holds 2 s of history (200 samples @ 100 Hz) for IMU, PPG, and pulse-sensor
  data. This is where the not-yet-written algorithm layer will read from.
  It is *not* thread-safe on its own — callers reading from outside the
  owning task must suspend the task or add a mutex.
- **Sensor drivers are register-level**, not vendor SDKs — the only external
  dependency is the Adafruit SSD1306/GFX display library. The IMU and PPG
  drivers talk directly to documented registers so the fall-detection and
  HR/SpO2 algorithms (Phase 2) can be built on a fully understood signal
  chain. See [src/sensors/mpu6050_driver.h](src/sensors/mpu6050_driver.h) and
  [src/sensors/max30102_driver.h](src/sensors/max30102_driver.h) for the
  register maps.
- **App state machine** (`AppState` in
  [src/app/app_state.h](src/app/app_state.h)):
  `BOOT → INIT_HW → STREAM_RAW`, with `SENSOR_ERROR` and (planned)
  `LOW_BATTERY` / `CALIBRATION` branches.

## Repository layout

```
src/
├── main.cpp                setup()/loop(), wires all modules together
├── config/
│   ├── board_config.h       sample rates, buffer sizes, task priorities, bench-tooling flags
│   └── pins.h                GPIO assignments
├── drivers/
│   └── i2c_manager.h/.cpp    mutex-guarded I2C wrapper shared by all sensors
├── sensors/
│   ├── mpu6050_driver.h/.cpp    register-level IMU driver
│   ├── max30102_driver.h/.cpp   register-level PPG driver
│   └── sensor_manager.h/.cpp    owns both drivers + the FreeRTOS sampling tasks
├── display/
│   └── ssd1306_display.h/.cpp   renders AppStatus to the OLED
├── power/
│   └── power_manager.h/.cpp     battery voltage/percent (stubbed until LiPo is wired)
├── app/
│   └── app_state.h/.cpp         global AppStatus + AppState machine
├── utils/
│   ├── logger.h/.cpp             leveled, tagged serial logging
│   ├── ring_buffer.h             fixed-size circular buffer template
│   └── csv_stream.h/.cpp         full-rate CSV telemetry (opt-in)
├── test/
│   ├── selftest.h/.cpp           boot-time PASS/FAIL hardware check
│   └── imu_calibration.h/.cpp    six-position accel/gyro calibration routine
└── probe/
    └── probe_max30102.cpp        standalone MAX30102 bus diagnostic (own env)

tools/
└── capture_and_plot.py       PC-side CSV capture + heart-rate/noise analysis

INTEGRATION.md                 step-by-step guide for wiring up the validation suite
platformio.ini                 esp32dev (firmware) and probe (diagnostic) environments
```

## Building and flashing

Requires [PlatformIO](https://platformio.org/) (CLI or the VS Code extension).

```bash
# build + flash the main firmware
pio run -e esp32dev -t upload

# open the serial monitor (115200 baud, with backtrace decoding)
pio device monitor
```

`esp32dev` is the default environment, so a bare `pio run` / `pio run -t upload`
also works. The `probe` environment (see below) is opt-in only.

## Bench tooling

The firmware has three opt-in diagnostic modes, controlled by build flags in
[platformio.ini](platformio.ini) — enable **only one at a time**, then
reflash. Full walkthrough with required code patches: [INTEGRATION.md](INTEGRATION.md).

| Flag | What it does |
|---|---|
| `-DRUN_SELFTEST=1` | Runs a PASS/FAIL battery of checks at boot (gravity magnitude, accel/gyro noise, I2C read reliability, pulse-sensor range) and prints a report table. Board must be flat and still. |
| `-DRUN_CALIBRATION=1` | Interactive six-position accelerometer + gyro bias calibration over serial. Prints a paste-ready offset/scale block for `src/config/calibration.h`. Halts after running — reflash without the flag to resume normal operation. |
| `-DCSV_STREAM_ENABLED=1` | Streams every IMU/PPG sample as CSV (`$I,...` / `$P,...` lines) at full 100 Hz rate for offline analysis. Also set `LOG_MIN_LEVEL` to `2` in `board_config.h` so debug logs don't interleave with the data. |

CSV capture pairs with the analysis script:

```bash
pip install pyserial numpy matplotlib

# capture 30 s from the board and plot
python tools/capture_and_plot.py --port COM5 --seconds 30

# or re-analyse a capture you already have, no board needed
python tools/capture_and_plot.py --replay session.csv
```

It reports measured sample rate, accelerometer noise floor, gyro bias, and a
heart-rate estimate (FFT peak in the 42–180 bpm band) from both the analog
pulse sensor and the MAX30102 IR channel, and saves `capture_plot.png`.

### MAX30102 bus probe

A separate, dependency-free diagnostic for isolating I2C read failures on the
MAX30102 (ACKs its address but fails register reads). Builds as its own
PlatformIO environment so it never gets mixed into the main firmware:

```bash
pio run -e probe -t upload
pio device monitor
```

It compares two I2C read strategies (repeated-START vs. STOP-then-START) at
both 100 kHz and 400 kHz, using the MPU6050 as a known-good control device, and
prints a verdict on whether the fault is bus timing, level shifting, or a
dead/counterfeit part.

## Known hardware quirks

- **MPU-6500 clone die.** This board's `WHO_AM_I` reads `0x70`, not the
  genuine MPU-6050's `0x68`. On the 6500 die, the accelerometer DLPF lives in
  a register (`0x1D`, undocumented on real MPU-6050s) that the current driver
  doesn't touch — so the accelerometer runs unfiltered at ~460 Hz while the
  gyro sits at the configured 44 Hz. This inflates the accel noise floor and
  affects self-test/calibration numbers. Fix is documented in
  [INTEGRATION.md](INTEGRATION.md#0b-fix-the-clone-accelerometer-bandwidth-srcsensorsmpu6050_driverh).
- **MAX30102 register reads failing.** Some breakout boards ACK the `0x57`
  address but fail every subsequent register read. This is a level-shifting /
  bus-timing issue on certain clone boards, not a code bug — use the `probe`
  environment above to characterize it before changing driver code.

## Status & roadmap

Implemented:
- I2C bus manager, register-level MPU6050 + MAX30102 drivers, analog pulse
  sensor reading
- FreeRTOS sampling tasks with ring-buffer history
- OLED status display, app state machine, tagged/leveled logging
- Self-test, IMU calibration, and CSV telemetry validation suite

Not yet implemented (see `TODO` markers throughout the source):
- **Phase 2** — heart-rate and SpO2 computation from the PPG/pulse buffers,
  fall detection from the IMU buffer (thresholds per Bourke & Lyons, in g and
  °/s — meaningless until IMU calibration is applied), corresponding OLED
  screens
- **Phase 3** — LiPo battery monitoring and `LOW_BATTERY` state, light-sleep
  power management, reduced LED current at low battery

## Configuration reference

Key tunables live in [src/config/board_config.h](src/config/board_config.h):
I2C frequency, device addresses, sample rates (100 Hz for IMU/PPG/pulse),
ring buffer depth (200 samples = 2 s), FreeRTOS task stack sizes/priorities
(sensor tasks pinned to Core 1, above app priority so serial output never
delays sampling), and `LOG_MIN_LEVEL`.

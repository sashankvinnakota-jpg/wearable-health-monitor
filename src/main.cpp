#include <Arduino.h>
#include "config/board_config.h"
#include "config/pins.h"
#include "drivers/i2c_manager.h"
#include "sensors/sensor_manager.h"
#include "display/ssd1306_display.h"
#include "power/power_manager.h"
#include "app/app_state.h"
#include "utils/logger.h"
#include "utils/csv_stream.h"
#include "test/selftest.h"
#include "test/imu_calibration.h"

// ─────────────────────────────────────────────────────────────────────────────
// Module instances
// ─────────────────────────────────────────────────────────────────────────────
static SensorManager   sensorMgr;
static SSD1306Display  display(ADDR_SSD1306, 128, 64);

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────
static void setState(AppState next) {
    AppStatus &s = getAppStatus();
    if (s.state != next) {
        LOG_I("APP", "State: %s → %s", appStateString(s.state), appStateString(next));
        s.state = next;
    }
}

// Run once at boot — lists all responding I2C devices
// Expected: 0x3C (SSD1306), 0x57 (MAX30102), 0x68 (MPU6050)
static void i2cScan() {
    LOG_I("I2C", "Bus scan —");
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        if (I2CManager::devicePresent(addr)) {
            LOG_I("I2C", "  0x%02X found", addr);
            found++;
        }
    }
    if (found == 0) LOG_W("I2C", "  No devices found — check wiring");
    else            LOG_I("I2C", "  Total: %d device(s)", found);
}

// ─────────────────────────────────────────────────────────────────────────────
// setup()
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    setState(AppState::BOOT);

    Logger::begin(SERIAL_BAUD);
    CSV_BEGIN();        // no-op unless CSV_STREAM_ENABLED=1

    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, HIGH);     // LED on = initializing

    setState(AppState::INIT_HW);

    // ── I2C bus ───────────────────────────────────────────────────────────────
    if (!I2CManager::begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQ_HZ)) {
        LOG_E("APP", "I2C init failed — halting");
        while (true) delay(1000);
    }
    LOG_I("APP", "I2C bus OK — SDA=%d SCL=%d @ %lu Hz", PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQ_HZ);

    // Scan before doing anything — if expected devices are missing, fix wiring first
    i2cScan();

    // ── Display ───────────────────────────────────────────────────────────────
    // Init display early so all subsequent steps are visible on screen
    getAppStatus().health.display_ok = display.begin();
    if (getAppStatus().health.display_ok) {
        display.showBootScreen("v" FW_VERSION);
    }

    // ── Power manager ─────────────────────────────────────────────────────────
    PowerManager::begin();

    // ── Sensors ───────────────────────────────────────────────────────────────
    bool sensors_ok = sensorMgr.begin();
    display.showSensorStatus(getAppStatus().health);

    // ── Bench tooling ─────────────────────────────────────────────────────────
    // Both run BEFORE startTasks() — they need deterministic, exclusive access
    // to the I2C bus. Deliberately placed above the !sensors_ok early return:
    // a failed sensor is exactly when you most want the diagnostics.
    // Enable one at a time via build_flags in platformio.ini.
#ifdef RUN_SELFTEST
    runSelfTest(sensorMgr.imu(), sensorMgr.ppg(),
                getAppStatus().health.imu_ok,
                getAppStatus().health.ppg_ok);
#endif

#ifdef RUN_CALIBRATION
    if (getAppStatus().health.imu_ok) {
        runImuCalibration(sensorMgr.imu());
    } else {
        LOG_E("APP", "Cannot calibrate — IMU did not initialise.");
    }
    LOG_I("APP", "Calibration build: halting. Reflash without RUN_CALIBRATION.");
    while (true) delay(1000);
#endif

    if (!sensors_ok) {
        LOG_E("APP", "No sensors initialized — check I2C wiring");
        display.showError("No sensors found");
        setState(AppState::SENSOR_ERROR);
        return;     // do not start tasks; loop() will blink LED
    }

    // ── Start sensor tasks ────────────────────────────────────────────────────
    sensorMgr.startTasks();

    setState(AppState::STREAM_RAW);
    LOG_I("APP", "Init complete. Streaming raw data.");
    digitalWrite(PIN_STATUS_LED, LOW);      // LED off = running
}

// ─────────────────────────────────────────────────────────────────────────────
// loop() — state machine + serial heartbeat at 2 Hz
//
// Sensor reads happen in FreeRTOS tasks. This loop handles:
//   - Serial debug output
//   - Display refresh
//   - State transitions (error handling, future: button events)
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    AppStatus &s = getAppStatus();
    s.uptime_ms = millis();

    switch (s.state) {

        case AppState::STREAM_RAW:
            // Heartbeat: print latest readings every 500 ms
            LOG_D("LOOP",
                  "ACC=%.3fg | RED=%-6lu | IR=%-6lu | PLS=%4d | UP=%lus",
                  s.last_accel_mag,
                  s.last_ppg_red,
                  s.last_ppg_ir,
                  s.pulse_raw,
                  s.uptime_ms / 1000UL);

            display.showRawData(s);

            // TODO (Phase 2): check fall_detected flag → setState(ALERT)
            // TODO (Phase 3): check PowerManager::isBatteryLow() → setState(LOW_BATTERY)
            break;

        case AppState::SENSOR_ERROR:
            // Blink LED to make error visible without serial monitor
            digitalWrite(PIN_STATUS_LED, !digitalRead(PIN_STATUS_LED));
            break;

        case AppState::LOW_BATTERY:
            // TODO (Phase 3): display battery warning, reduce LED current
            LOG_W("APP", "Low battery");
            break;

        case AppState::IDLE:
            // TODO: enter when no activity for N seconds
            break;

        default:
            break;
    }

    delay(500);  // 2 Hz loop — sensor tasks run independently at their own rates
}

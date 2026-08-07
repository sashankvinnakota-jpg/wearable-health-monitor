#include "app_state.h"

static AppStatus _status = {
    .state          = AppState::BOOT,
    .health         = { false, false, false, false, 0 },
    .uptime_ms      = 0,
    .last_accel_mag = 0.0f,
    .last_ppg_red   = 0,
    .last_ppg_ir    = 0,
    .pulse_raw      = 0,
};

AppStatus& getAppStatus() {
    return _status;
}

const char* appStateString(AppState s) {
    switch (s) {
        case AppState::BOOT:         return "BOOT";
        case AppState::INIT_HW:      return "INIT_HW";
        case AppState::IDLE:         return "IDLE";
        case AppState::STREAM_RAW:   return "STREAM_RAW";
        case AppState::CALIBRATION:  return "CALIBRATION";
        case AppState::SENSOR_ERROR: return "SENSOR_ERROR";
        case AppState::LOW_BATTERY:  return "LOW_BATTERY";
        default:                     return "UNKNOWN";
    }
}

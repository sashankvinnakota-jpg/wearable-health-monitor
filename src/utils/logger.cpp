#include "logger.h"
#include <stdarg.h>

LogLevel Logger::_min_level = (LogLevel)LOG_MIN_LEVEL;

void Logger::begin(uint32_t baud) {
    Serial.begin(baud);
    delay(200);
    Serial.printf("\n\n=== Wearable Health Monitor v%s ===\n", FW_VERSION);
    Serial.printf("Build: %s %s\n\n", __DATE__, __TIME__);
}

void Logger::setMinLevel(LogLevel level) {
    _min_level = level;
}

void Logger::log(LogLevel level, const char *tag, const char *fmt, ...) {
    if (level < _min_level) return;

    const char *prefix;
    switch (level) {
        case LogLevel::DEBUG: prefix = "D"; break;
        case LogLevel::INFO:  prefix = "I"; break;
        case LogLevel::WARN:  prefix = "W"; break;
        case LogLevel::ERROR: prefix = "E"; break;
        default:              prefix = "?"; break;
    }

    char msg[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(msg, sizeof(msg), fmt, args);
    va_end(args);

    // [L][TAG] message  — consistent format, easy to grep in serial monitor
    Serial.printf("[%s][%-10s] %s\n", prefix, tag, msg);
}

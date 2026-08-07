#pragma once
#include <Arduino.h>
#include "../config/board_config.h"

// ─────────────────────────────────────────────────────────────────────────────
// Logger — level-filtered, tagged serial output
// Format: [L][TAG] message
// Call Logger::begin() once in setup(). Use macros everywhere else.
// ─────────────────────────────────────────────────────────────────────────────

enum class LogLevel : uint8_t { DEBUG = 0, INFO = 1, WARN = 2, ERROR = 3 };

#if LOG_ENABLED
  #define LOG_D(tag, fmt, ...)  Logger::log(LogLevel::DEBUG, tag, fmt, ##__VA_ARGS__)
  #define LOG_I(tag, fmt, ...)  Logger::log(LogLevel::INFO,  tag, fmt, ##__VA_ARGS__)
  #define LOG_W(tag, fmt, ...)  Logger::log(LogLevel::WARN,  tag, fmt, ##__VA_ARGS__)
  #define LOG_E(tag, fmt, ...)  Logger::log(LogLevel::ERROR, tag, fmt, ##__VA_ARGS__)
#else
  #define LOG_D(tag, fmt, ...)  ((void)0)
  #define LOG_I(tag, fmt, ...)  ((void)0)
  #define LOG_W(tag, fmt, ...)  ((void)0)
  #define LOG_E(tag, fmt, ...)  ((void)0)
#endif

class Logger {
public:
    static void begin(uint32_t baud);
    static void setMinLevel(LogLevel level);
    static void log(LogLevel level, const char *tag, const char *fmt, ...);

private:
    static LogLevel _min_level;
};

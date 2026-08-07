#include "ssd1306_display.h"
#include "../drivers/i2c_manager.h"
#include "../utils/logger.h"
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Wire.h>

// Frame buffer lives here — 128×64 / 8 = 1024 bytes of RAM
static Adafruit_SSD1306 _oled(128, 64, &Wire, -1);  // -1 = no hardware reset pin

SSD1306Display::SSD1306Display(uint8_t i2c_addr, uint8_t width, uint8_t height)
    : _addr(i2c_addr), _width(width), _height(height), _initialized(false) {}

bool SSD1306Display::begin() {
    // Adafruit library calls Wire.begin() internally — I2C must already be init'd
    if (!_oled.begin(SSD1306_SWITCHCAPVCC, _addr)) {
        LOG_E("OLED", "SSD1306 not found at 0x%02X. Check SA0 pin for address.", _addr);
        return false;
    }
    _oled.setTextSize(1);
    _oled.setTextColor(SSD1306_WHITE);
    _oled.clearDisplay();
    _oled.display();
    _initialized = true;
    LOG_I("OLED", "OK at 0x%02X — 128x64", _addr);
    return true;
}

// Hold I2C mutex across the full display sequence.
// Prevents sensor tasks from interleaving Wire transactions with Adafruit library calls.
void SSD1306Display::_beginFrame() {
    xSemaphoreTake(I2CManager::getMutex(), portMAX_DELAY);
    _oled.clearDisplay();
    _oled.setCursor(0, 0);
}

void SSD1306Display::_endFrame() {
    _oled.display();
    xSemaphoreGive(I2CManager::getMutex());
}

void SSD1306Display::showBootScreen(const char *version) {
    if (!_initialized) return;
    _beginFrame();
    _oled.println("Wearable Monitor");
    _oled.println(version);
    _oled.println();
    _oled.println("Initializing...");
    _endFrame();
}

void SSD1306Display::showSensorStatus(const SensorHealth &h) {
    if (!_initialized) return;
    _beginFrame();
    _oled.println("Sensor Status:");
    _oled.println();
    _oled.printf("IMU  (MPU6050): %s\n", h.imu_ok     ? "OK"   : "FAIL");
    _oled.printf("PPG  (MAX30102):%s\n", h.ppg_ok     ? "OK"   : "FAIL");
    _oled.printf("DISP (SSD1306): OK\n");
    _oled.printf("Pulse Sensor:  %s\n", h.pulse_ok   ? "OK"   : "N/A");
    _endFrame();
    delay(2500);  // leave status on screen before transitioning
}

void SSD1306Display::showRawData(const AppStatus &s) {
    if (!_initialized) return;
    _beginFrame();
    _oled.printf("ACC  %.3fg\n",  s.last_accel_mag);
    _oled.printf("RED  %lu\n",    s.last_ppg_red);
    _oled.printf("IR   %lu\n",    s.last_ppg_ir);
    _oled.printf("PLS  %d\n",     s.pulse_raw);
    _oled.printf("UP   %lus\n",   s.uptime_ms / 1000UL);
    _endFrame();
}

void SSD1306Display::showError(const char *msg) {
    if (!_initialized) return;
    _beginFrame();
    _oled.println("!! ERROR !!");
    _oled.println();
    _oled.println(msg);
    _endFrame();
}

void SSD1306Display::clear() {
    if (!_initialized) return;
    xSemaphoreTake(I2CManager::getMutex(), portMAX_DELAY);
    _oled.clearDisplay();
    _oled.display();
    xSemaphoreGive(I2CManager::getMutex());
}

// ─────────────────────────────────────────────────────────────────────────────
// probe_max30102.ino — standalone MAX30102 bus diagnostic
//
// Depends on NOTHING from the project. Uses only Wire. Run this to find out
// exactly why 0x57 acknowledges its address but fails every register read.
//
// HOW TO RUN
//   PlatformIO : rename your src/main.cpp to src/main.cpp.bak, drop this in as
//                src/main.cpp, build, upload. Restore afterwards.
//   Arduino IDE: open directly, select "ESP32 Dev Module", upload.
//
// WHAT IT DISTINGUISHES
//   An I2C register read is really two transactions: write the register number,
//   then read the data. They can be joined two ways:
//
//     METHOD A (repeated START) — what your I2CManager::readRegs() does.
//       START addr+W, reg, repeated-START addr+R, data, STOP
//       Holds the bus the whole time. Some level-shifted breakout boards
//       cannot meet the timing and NACK the second address phase.
//
//     METHOD B (STOP then START)
//       START addr+W, reg, STOP ... START addr+R, data, STOP
//       Releases the bus between phases. Slower, far more forgiving.
//
//   If A fails and B works, the fix is a four-line change to readRegs().
//   If both fail at both speeds while the address still ACKs, the chip is
//   almost certainly counterfeit or internally damaged — no code will fix it.
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>
#include <Wire.h>

#define PIN_SDA         21
#define PIN_SCL         22

#define ADDR_MAX30102   0x57
#define ADDR_MPU6050    0x68      // known-good control device
#define ADDR_SSD1306    0x3C

#define REG_MAX_PARTID  0xFF      // MAX30102 -> 0x15,  MAX30100 -> 0x11
#define REG_MAX_REVID   0xFE
#define REG_MPU_WHOAMI  0x75

#define TRIALS          20

const uint32_t SPEEDS[] = { 100000UL, 400000UL };

// ── endTransmission() decoder ────────────────────────────────────────────────
const char *txErr(uint8_t c) {
    switch (c) {
        case 0: return "OK";
        case 1: return "data too long";
        case 2: return "NACK on ADDRESS  <- device not responding";
        case 3: return "NACK on DATA     <- device rejected the register number";
        case 4: return "other error";
        case 5: return "timeout          <- SCL held low, check pull-ups";
        default: return "unknown";
    }
}

// ── METHOD A: repeated START (what the project driver uses) ──────────────────
bool readRegRepeatedStart(uint8_t addr, uint8_t reg, uint8_t *out, uint8_t *tx_code) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    uint8_t code = Wire.endTransmission(false);   // false = repeated START
    if (tx_code) *tx_code = code;
    if (code != 0) return false;

    if (Wire.requestFrom((int)addr, 1, true) != 1) return false;
    *out = Wire.read();
    return true;
}

// ── METHOD B: full STOP between write and read ───────────────────────────────
bool readRegStopStart(uint8_t addr, uint8_t reg, uint8_t *out, uint8_t *tx_code) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    uint8_t code = Wire.endTransmission(true);    // true = send STOP
    if (tx_code) *tx_code = code;
    if (code != 0) return false;

    delayMicroseconds(50);                        // let the bus settle
    if (Wire.requestFrom((int)addr, 1, true) != 1) return false;
    *out = Wire.read();
    return true;
}

// ── bus scan ─────────────────────────────────────────────────────────────────
void scan() {
    Serial.print(F("  devices: "));
    int n = 0;
    for (uint8_t a = 1; a < 127; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) {
            Serial.printf("0x%02X ", a);
            n++;
        }
    }
    if (n == 0) Serial.print(F("NONE"));
    Serial.printf("  (%d total)\n", n);
}

// ── run TRIALS reads, report success rate and the last value seen ────────────
void testMethod(const char *name, uint8_t addr, uint8_t reg,
                bool (*fn)(uint8_t, uint8_t, uint8_t *, uint8_t *)) {
    int ok = 0;
    uint8_t val = 0, last_val = 0, code = 0, last_code = 0;

    for (int i = 0; i < TRIALS; i++) {
        if (fn(addr, reg, &val, &code)) { ok++; last_val = val; }
        else { last_code = code; }
        delay(5);
    }

    Serial.printf("    %-22s %2d/%d ok", name, ok, TRIALS);
    if (ok > 0) {
        Serial.printf("   value = 0x%02X\n", last_val);
    } else {
        Serial.printf("   endTransmission: %s\n", txErr(last_code));
    }
}

void identify(uint8_t part_id) {
    Serial.print(F("    -> "));
    switch (part_id) {
        case 0x15: Serial.println(F("MAX30102 confirmed. Chip is healthy.")); break;
        case 0x11: Serial.println(F("MAX30100! Board is mislabelled — different "
                                    "register map, driver will not work as-is.")); break;
        case 0x00: Serial.println(F("0x00 — read returned nothing meaningful.")); break;
        case 0xFF: Serial.println(F("0xFF — bus floating, SDA never pulled low.")); break;
        default:   Serial.printf ("unexpected part ID 0x%02X\n", part_id); break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(1500);

    Serial.println(F("\n\n==============================================="));
    Serial.println(F(" MAX30102 STANDALONE PROBE"));
    Serial.println(F("===============================================\n"));

    Wire.begin(PIN_SDA, PIN_SCL);

    for (uint8_t s = 0; s < 2; s++) {
        uint32_t hz = SPEEDS[s];
        Wire.setClock(hz);
        delay(50);

        Serial.printf("--- %lu Hz --------------------------------\n",
                      (unsigned long)hz);
        scan();

        // Control device first. If the MPU passes and the MAX fails, the bus
        // and the method are both fine and the problem is that one chip.
        Serial.println(F("  MPU6050 @ 0x68 (control):"));
        testMethod("repeated START", ADDR_MPU6050, REG_MPU_WHOAMI, readRegRepeatedStart);
        testMethod("STOP then START", ADDR_MPU6050, REG_MPU_WHOAMI, readRegStopStart);

        Serial.println(F("  MAX30102 @ 0x57:"));
        testMethod("repeated START", ADDR_MAX30102, REG_MAX_PARTID, readRegRepeatedStart);
        testMethod("STOP then START", ADDR_MAX30102, REG_MAX_PARTID, readRegStopStart);

        // Whichever method worked, try to identify the part.
        uint8_t id = 0, code = 0;
        if (readRegStopStart(ADDR_MAX30102, REG_MAX_PARTID, &id, &code) ||
            readRegRepeatedStart(ADDR_MAX30102, REG_MAX_PARTID, &id, &code)) {
            identify(id);
            uint8_t rev = 0;
            if (readRegStopStart(ADDR_MAX30102, REG_MAX_REVID, &rev, &code))
                Serial.printf("    revision ID = 0x%02X\n", rev);
        } else {
            Serial.println(F("    -> no read method succeeded at this speed"));
        }
        Serial.println();
    }

    // ── Verdict ──────────────────────────────────────────────────────────────
    Serial.println(F("==============================================="));
    Serial.println(F(" HOW TO READ THIS"));
    Serial.println(F("==============================================="));
    Serial.println(F(" 0x57 absent from scan"));
    Serial.println(F("   -> power or wiring. No code change helps."));
    Serial.println(F(" 0x57 in scan, STOP-START works, repeated START fails"));
    Serial.println(F("   -> patch I2CManager::readRegs() to use STOP-then-read."));
    Serial.println(F(" 0x57 in scan, both fail, but MPU passes both"));
    Serial.println(F("   -> the chip ACKs its address but has no working"));
    Serial.println(F("      registers: counterfeit or dead part. Replace it."));
    Serial.println(F(" Works at 100k but not 400k"));
    Serial.println(F("   -> signal integrity. Set I2C_FREQ_HZ to 100000."));
    Serial.println(F("===============================================\n"));
}

void loop() {
    delay(10000);
}

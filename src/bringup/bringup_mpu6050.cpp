// ─────────────────────────────────────────────────────────────────────────────
//  bringup_mpu6050.cpp
//
//  Stage 1 hardware bring-up: MPU-6050 (GY-521) on the ESP32 I2C bus.
//
//  STANDALONE. This file depends on nothing in src/ — not I2CManager, not
//  Logger, not board_config.h, not AppStatus. Every constant it needs is
//  redefined locally and marked BU_*. That is deliberate: if this program
//  works and the main firmware does not, the fault is in the firmware layers,
//  not the sensor or the wiring. Sharing headers would destroy that property.
//
//  Build:   pio run -e bringup_imu -t upload && pio device monitor
//
//  It runs six stages in order and stops at the first hard failure, so the
//  stage number in the output tells you exactly where the problem is:
//
//    STAGE 0  Bus scan            — is anything electrically alive on I2C?
//    STAGE 1  Identity            — is the device at 0x68 actually an IMU?
//    STAGE 2  Reset & wake        — does it accept writes and leave sleep?
//    STAGE 3  Configure + verify  — did every config register take the value?
//    STAGE 4  Bus reliability     — 200 back-to-back reads, count failures
//    STAGE 5  Signal sanity       — gravity magnitude, gyro bias, noise floor
//    then     Live stream         — scaled readings + tilt, 10 Hz, forever
//
//  Board must be FLAT and STILL from power-on until stage 5 prints.
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// ── Wiring ───────────────────────────────────────────────────────────────────
// Matches src/config/pins.h, redeclared locally so this file stands alone.
//   MPU VCC → ESP32 3V3      (NOT VIN/5V — the GY-521 regulator is fine either
//                             way, but its pull-ups then sit at 5V and will
//                             push 5V onto SDA/SCL, which the ESP32 hates)
//   MPU GND → ESP32 GND
//   MPU SCL → GPIO22
//   MPU SDA → GPIO21
//   MPU AD0 → GND or floating (GY-521 pulls it low) → address 0x68
static const uint8_t BU_PIN_SDA = 21;
static const uint8_t BU_PIN_SCL = 22;

static const uint8_t BU_ADDR_PRIMARY = 0x68;  // AD0 low
static const uint8_t BU_ADDR_ALT     = 0x69;  // AD0 high

static const uint32_t BU_BAUD = 115200;

// ── MPU register map (InvenSense RM-MPU-6000A-00) ────────────────────────────
static const uint8_t REG_SMPLRT_DIV   = 0x19;
static const uint8_t REG_CONFIG       = 0x1A;
static const uint8_t REG_GYRO_CONFIG  = 0x1B;
static const uint8_t REG_ACCEL_CONFIG = 0x1C;
static const uint8_t REG_ACCEL_CFG2   = 0x1D;  // MPU-6500/9250 die only
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;
static const uint8_t REG_PWR_MGMT_1   = 0x6B;
static const uint8_t REG_WHO_AM_I     = 0x75;

// Config values — identical to the main firmware so this validates the same
// signal chain the real driver will use.
static const uint8_t CFG_SMPLRT_DIV   = 9;     // 1 kHz / (1+9) = 100 Hz
static const uint8_t CFG_DLPF         = 0x03;  // 44 Hz BW, gyro output 1 kHz
static const uint8_t CFG_GYRO_FS      = 0x08;  // ±500 °/s
static const uint8_t CFG_ACCEL_FS     = 0x10;  // ±8 g
static const uint8_t CFG_PWR_CLK      = 0x01;  // wake, PLL from X gyro
static const uint8_t CFG_ACCEL_DLPF2  = 0x03;  // 6500 die: 41 Hz accel BW
                                               // (bit 3 = ACCEL_FCHOICE_B must
                                               //  stay 0 or the DLPF is bypassed)

static const float ACCEL_SENS = 4096.0f;  // LSB/g   at ±8 g
static const float GYRO_SENS  = 65.5f;    // LSB/°/s at ±500 °/s

// ── Pass/fail thresholds for stage 5 ─────────────────────────────────────────
// Deliberately loose. This is a "is the sensor alive and sane" check, not a
// calibration. Tight limits here would fail good hardware and teach you to
// ignore the output.
static const float LIM_GRAVITY_LO   = 0.90f;  // g — resting |a| lower bound
static const float LIM_GRAVITY_HI   = 1.10f;  // g — resting |a| upper bound
static const float LIM_GYRO_BIAS    = 8.0f;   // °/s — per-axis bias at rest
static const float LIM_ACCEL_NOISE  = 0.05f;  // g   — per-axis std dev
static const float LIM_TEMP_LO      = 5.0f;   // °C
static const float LIM_TEMP_HI      = 60.0f;  // °C

// ── State discovered during bring-up ─────────────────────────────────────────
static uint8_t  g_addr    = BU_ADDR_PRIMARY;
static uint8_t  g_whoami  = 0x00;
static bool     g_is6500  = false;  // clone die with ACCEL_CONFIG2
static uint32_t g_failed  = 0;      // count of failed stages (non-fatal ones)

// ═════════════════════════════════════════════════════════════════════════════
//  Low-level I2C helpers
//  Every one returns a status. Nothing here silently swallows an error — a
//  driver that returns 0 for "read failed" is how you spend a day debugging
//  an algorithm when the real problem is a loose jumper.
// ═════════════════════════════════════════════════════════════════════════════

// Wire.endTransmission() status codes, for readable diagnostics.
static const char *twiErrorName(uint8_t code) {
    switch (code) {
        case 0: return "OK";
        case 1: return "data too long for buffer";
        case 2: return "NACK on address (nothing at this address)";
        case 3: return "NACK on data (device rejected the byte)";
        case 4: return "other error (bus stuck / arbitration lost)";
        case 5: return "timeout (SDA or SCL held low)";
        default: return "unknown";
    }
}

static bool regWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(g_addr);
    Wire.write(reg);
    Wire.write(val);
    uint8_t err = Wire.endTransmission();
    if (err != 0) {
        Serial.printf("       ! write 0x%02X <- 0x%02X failed: %s\n",
                      reg, val, twiErrorName(err));
        return false;
    }
    return true;
}

// Repeated-START read. endTransmission(false) holds the bus between the
// register-address write and the data read. This is the correct sequence for
// InvenSense parts; some clone boards only work with a full STOP in between,
// which is what the MAX30102 probe environment exists to diagnose.
static bool regReadN(uint8_t reg, uint8_t *buf, uint8_t n) {
    Wire.beginTransmission(g_addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;

    uint8_t got = Wire.requestFrom((uint16_t) g_addr, (uint8_t) n, (uint8_t) true);
    if (got != n) return false;

    for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
    return true;
}

static bool regRead(uint8_t reg, uint8_t &val) {
    return regReadN(reg, &val, 1);
}

// Read a register and compare against what we wrote. A register that reads
// back wrong means the write never landed — usually a bus problem, sometimes
// a register that does not exist on this die.
static bool verifyReg(const char *name, uint8_t reg, uint8_t expect) {
    uint8_t got = 0;
    if (!regRead(reg, got)) {
        Serial.printf("       %-13s 0x%02X  READ FAILED\n", name, reg);
        return false;
    }
    bool ok = (got == expect);
    Serial.printf("       %-13s 0x%02X  wrote 0x%02X  read 0x%02X  %s\n",
                  name, reg, expect, got, ok ? "ok" : "MISMATCH");
    return ok;
}

static void banner(const char *text) {
    Serial.println();
    Serial.println(F("---------------------------------------------------------------"));
    Serial.printf("  %s\n", text);
    Serial.println(F("---------------------------------------------------------------"));
}

// Blink forever. Stops the program at an unrecoverable fault so the serial
// output stays on screen instead of scrolling away under a boot loop.
static void halt(const char *why) {
    Serial.println();
    Serial.printf("  BRING-UP HALTED: %s\n", why);
    Serial.println(F("  Fix the above, then reset the board."));
    Serial.flush();
    pinMode(2, OUTPUT);
    for (;;) {
        digitalWrite(2, HIGH); delay(120);
        digitalWrite(2, LOW);  delay(880);
    }
}

// ═════════════════════════════════════════════════════════════════════════════
//  STAGE 0 — Bus scan
// ═════════════════════════════════════════════════════════════════════════════
static void stage0_scan() {
    banner("STAGE 0  I2C bus scan @ 100 kHz");
    Serial.println(F("  Scanning at 100 kHz first: a marginal bus (long wires, weak"));
    Serial.println(F("  pull-ups) often works at 100 kHz and fails at 400 kHz."));
    Serial.println();

    Wire.setClock(100000UL);
    delay(50);

    uint8_t found = 0;
    bool sawPrimary = false, sawAlt = false;

    for (uint8_t a = 0x08; a <= 0x77; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) {
            found++;
            const char *guess = "unknown device";
            if (a == 0x68) guess = "MPU-6050 / MPU-6500 (AD0 low)";
            else if (a == 0x69) guess = "MPU-6050 (AD0 high)";
            else if (a == 0x57) guess = "MAX30102 PPG";
            else if (a == 0x3C || a == 0x3D) guess = "SSD1306 OLED";
            Serial.printf("       0x%02X  %s\n", a, guess);
            if (a == 0x68) sawPrimary = true;
            if (a == 0x69) sawAlt = true;
        }
    }

    Serial.println();
    if (found == 0) {
        Serial.println(F("       no devices responded."));
        Serial.println(F("       -> check 3V3 and GND to the GY-521"));
        Serial.println(F("       -> check SDA=GPIO21, SCL=GPIO22 (easy to swap)"));
        Serial.println(F("       -> measure SDA and SCL: both should idle near 3.3 V."));
        Serial.println(F("          If either sits at 0 V the bus is shorted or a"));
        Serial.println(F("          device is holding it low."));
        halt("no I2C devices found");
    }

    Serial.printf("       %u device(s) responding.\n", found);

    if (sawPrimary) {
        g_addr = BU_ADDR_PRIMARY;
    } else if (sawAlt) {
        g_addr = BU_ADDR_ALT;
        Serial.println(F("       note: IMU answered at 0x69, not 0x68 — AD0 is high."));
        Serial.println(F("             Tie AD0 to GND, or change ADDR_MPU6050 in"));
        Serial.println(F("             board_config.h to 0x69."));
    } else {
        Serial.println(F("       devices are present but none at 0x68 or 0x69."));
        halt("MPU-6050 not on the bus");
    }
    Serial.printf("       using address 0x%02X\n", g_addr);
}

// ═════════════════════════════════════════════════════════════════════════════
//  STAGE 1 — Identity
// ═════════════════════════════════════════════════════════════════════════════
static void stage1_identity() {
    banner("STAGE 1  WHO_AM_I identity");

    if (!regRead(REG_WHO_AM_I, g_whoami)) {
        Serial.println(F("       device ACKs its address but WHO_AM_I read failed."));
        Serial.println(F("       That combination means the address phase works and"));
        Serial.println(F("       the data phase does not — almost always bus timing"));
        Serial.println(F("       or level shifting, not firmware."));
        halt("WHO_AM_I read failed");
    }

    Serial.printf("       WHO_AM_I = 0x%02X  ", g_whoami);
    switch (g_whoami) {
        case 0x68:
            Serial.println(F("(genuine MPU-6050)"));
            break;
        case 0x70:
            Serial.println(F("(MPU-6500 die — common GY-521 clone)"));
            g_is6500 = true;
            break;
        case 0x71:
        case 0x73:
            Serial.println(F("(MPU-9250 die)"));
            g_is6500 = true;
            break;
        case 0x98:
            Serial.println(F("(ICM-20689 / relabelled clone)"));
            g_is6500 = true;
            break;
        case 0x00:
        case 0xFF:
            Serial.println(F("(INVALID)"));
            Serial.println(F("       0x00 or 0xFF is not a device ID — it is the bus"));
            Serial.println(F("       floating. The read returned nothing."));
            halt("invalid WHO_AM_I");
            break;
        default:
            Serial.println(F("(unrecognised)"));
            Serial.println(F("       Register-compatible clones usually still work."));
            Serial.println(F("       Continuing — later stages will tell you."));
            g_is6500 = true;  // assume the wider register map, verify in stage 3
            break;
    }

    if (g_is6500) {
        Serial.println();
        Serial.println(F("       This is NOT a genuine MPU-6050 die. It is register-"));
        Serial.println(F("       compatible for everything this project uses, with one"));
        Serial.println(F("       real difference: the accelerometer has its own DLPF in"));
        Serial.println(F("       register 0x1D, which does not exist on a real 6050."));
        Serial.println(F("       Leave it unset and the accel runs unfiltered at ~460 Hz"));
        Serial.println(F("       while the gyro is filtered to 44 Hz — that mismatch is"));
        Serial.println(F("       what inflates your accel noise floor. Stage 3 sets it."));
    }
}

// ═════════════════════════════════════════════════════════════════════════════
//  STAGE 2 — Reset and wake
// ═════════════════════════════════════════════════════════════════════════════
static void stage2_wake() {
    banner("STAGE 2  Reset and wake");

    // Full device reset. Without this, a warm reboot (ESP32 resets, MPU does
    // not) leaves stale config in the sensor and you debug a state that a
    // power cycle would have cleared.
    Serial.println(F("       device reset (PWR_MGMT_1 bit 7)..."));
    if (!regWrite(REG_PWR_MGMT_1, 0x80)) {
        halt("device does not accept writes");
    }
    delay(100);  // datasheet: 100 ms for internal registers to settle

    // Post-reset PWR_MGMT_1 is 0x40 (asleep) on a genuine 6050 and 0x01
    // (already awake) on a 6500 die. Both are normal — this line is printed
    // for information, not as a check.
    uint8_t pwr = 0;
    regRead(REG_PWR_MGMT_1, pwr);
    Serial.printf("       post-reset PWR_MGMT_1 = 0x%02X (sleep bit %s — %s)\n",
                  pwr, (pwr & 0x40) ? "set" : "clear",
                  (pwr & 0x40) ? "6050 default" : "6500 default; both are fine");

    Serial.println(F("       waking, clock source = PLL from X gyro..."));
    if (!regWrite(REG_PWR_MGMT_1, CFG_PWR_CLK)) halt("wake write failed");
    delay(50);  // PLL lock

    regRead(REG_PWR_MGMT_1, pwr);
    if (pwr & 0x40) {
        Serial.println(F("       sleep bit still set after wake write."));
        halt("device will not leave sleep");
    }
    Serial.printf("       PWR_MGMT_1 = 0x%02X — awake, PLL locked.\n", pwr);
}

// ═════════════════════════════════════════════════════════════════════════════
//  STAGE 3 — Configure and verify every register
// ═════════════════════════════════════════════════════════════════════════════
static void stage3_configure() {
    banner("STAGE 3  Configure and read back");
    Serial.println(F("       Writing the same config the main firmware uses, then"));
    Serial.println(F("       reading every register back. A write that does not stick"));
    Serial.println(F("       is invisible until your data looks wrong."));
    Serial.println();

    regWrite(REG_SMPLRT_DIV,   CFG_SMPLRT_DIV);
    regWrite(REG_CONFIG,       CFG_DLPF);
    regWrite(REG_GYRO_CONFIG,  CFG_GYRO_FS);
    regWrite(REG_ACCEL_CONFIG, CFG_ACCEL_FS);
    if (g_is6500) regWrite(REG_ACCEL_CFG2, CFG_ACCEL_DLPF2);
    delay(20);

    bool ok = true;
    ok &= verifyReg("SMPLRT_DIV",   REG_SMPLRT_DIV,   CFG_SMPLRT_DIV);
    ok &= verifyReg("CONFIG",       REG_CONFIG,       CFG_DLPF);
    ok &= verifyReg("GYRO_CONFIG",  REG_GYRO_CONFIG,  CFG_GYRO_FS);
    ok &= verifyReg("ACCEL_CONFIG", REG_ACCEL_CONFIG, CFG_ACCEL_FS);
    if (g_is6500) {
        ok &= verifyReg("ACCEL_CFG2", REG_ACCEL_CFG2, CFG_ACCEL_DLPF2);
    }

    Serial.println();
    Serial.printf("       sample rate  = 1000 / (1 + %u) = %u Hz\n",
                  CFG_SMPLRT_DIV, 1000 / (1 + CFG_SMPLRT_DIV));
    Serial.println(F("       gyro range   = +/- 500 deg/s   (65.5 LSB per deg/s)"));
    Serial.println(F("       accel range  = +/- 8 g         (4096 LSB per g)"));
    Serial.println(F("       gyro DLPF    = 44 Hz"));
    if (g_is6500) Serial.println(F("       accel DLPF   = 41 Hz (via 0x1D)"));
    else          Serial.println(F("       accel DLPF   = 44 Hz (shared, via CONFIG)"));

    if (!ok) {
        Serial.println();
        Serial.println(F("       one or more registers did not take. Everything"));
        Serial.println(F("       downstream is now suspect."));
        halt("configuration verification failed");
    }
    Serial.println(F("       all config registers verified."));
}

// ═════════════════════════════════════════════════════════════════════════════
//  STAGE 4 — Bus reliability
//  An IMU that works 99% of the time looks fine on a scope and wrecks a
//  100 Hz sampling task. Measure it now, at both bus speeds.
// ═════════════════════════════════════════════════════════════════════════════
static void reliabilityAt(uint32_t hz, const char *label) {
    Wire.setClock(hz);
    delay(20);

    const uint16_t N = 200;
    uint16_t fails = 0;
    uint8_t  buf[14];
    uint32_t t0 = micros();
    for (uint16_t i = 0; i < N; i++) {
        if (!regReadN(REG_ACCEL_XOUT_H, buf, 14)) fails++;
    }
    uint32_t dt = micros() - t0;

    Serial.printf("       %-9s %3u/%u failed   %5.0f us per 14-byte read\n",
                  label, fails, N, (float) dt / N);
    if (fails > 0) g_failed++;
}

static void stage4_reliability() {
    banner("STAGE 4  Bus reliability — 200 burst reads per speed");
    reliabilityAt(100000UL, "100 kHz");
    reliabilityAt(400000UL, "400 kHz");

    Serial.println();
    if (g_failed == 0) {
        Serial.println(F("       zero failures at both speeds. Bus is solid."));
    } else {
        Serial.println(F("       failures detected. If 100 kHz is clean and 400 kHz"));
        Serial.println(F("       is not, the bus cannot sustain Fast Mode: shorten the"));
        Serial.println(F("       wires, or drop I2C_FREQ_HZ to 100000 in board_config.h."));
        Serial.println(F("       100 kHz is ample for 100 Hz sampling of 14 bytes."));
    }

    Wire.setClock(400000UL);  // settle at the project's configured speed
}

// ═════════════════════════════════════════════════════════════════════════════
//  Sample read + scaling
// ═════════════════════════════════════════════════════════════════════════════
struct Sample {
    int16_t rax, ray, raz, rtemp, rgx, rgy, rgz;  // raw LSB
    float   ax, ay, az;                            // g
    float   gx, gy, gz;                            // deg/s
    float   tempC;
};

static bool readSample(Sample &s) {
    uint8_t b[14];
    if (!regReadN(REG_ACCEL_XOUT_H, b, 14)) return false;

    s.rax   = (int16_t)((b[0]  << 8) | b[1]);
    s.ray   = (int16_t)((b[2]  << 8) | b[3]);
    s.raz   = (int16_t)((b[4]  << 8) | b[5]);
    s.rtemp = (int16_t)((b[6]  << 8) | b[7]);
    s.rgx   = (int16_t)((b[8]  << 8) | b[9]);
    s.rgy   = (int16_t)((b[10] << 8) | b[11]);
    s.rgz   = (int16_t)((b[12] << 8) | b[13]);

    s.ax = s.rax / ACCEL_SENS;
    s.ay = s.ray / ACCEL_SENS;
    s.az = s.raz / ACCEL_SENS;
    s.gx = s.rgx / GYRO_SENS;
    s.gy = s.rgy / GYRO_SENS;
    s.gz = s.rgz / GYRO_SENS;

    // Temperature formula differs by die and is a useful cross-check on the
    // identity we decided in stage 1: if the reading is wildly off room
    // temperature, WHO_AM_I probably lied about which die this is.
    s.tempC = g_is6500 ? (s.rtemp / 333.87f + 21.0f)
                       : (s.rtemp / 340.0f  + 36.53f);
    return true;
}

// ═════════════════════════════════════════════════════════════════════════════
//  STAGE 5 — Signal sanity at rest
// ═════════════════════════════════════════════════════════════════════════════
static void stage5_sanity() {
    banner("STAGE 5  Signal sanity — keep the board FLAT and STILL");
    Serial.println(F("       Collecting 200 samples at 100 Hz (2 seconds)..."));
    delay(600);

    const uint16_t N = 200;
    double sax = 0, say = 0, saz = 0, sgx = 0, sgy = 0, sgz = 0, stemp = 0;
    double qax = 0, qay = 0, qaz = 0;  // sums of squares, for std dev
    uint16_t good = 0;
    Sample s;

    for (uint16_t i = 0; i < N; i++) {
        if (readSample(s)) {
            good++;
            sax += s.ax; say += s.ay; saz += s.az;
            qax += (double) s.ax * s.ax;
            qay += (double) s.ay * s.ay;
            qaz += (double) s.az * s.az;
            sgx += s.gx; sgy += s.gy; sgz += s.gz;
            stemp += s.tempC;
        }
        delay(10);
    }

    if (good < N / 2) halt("more than half the sample reads failed");

    float max = sax / good, may = say / good, maz = saz / good;
    float mgx = sgx / good, mgy = sgy / good, mgz = sgz / good;
    float mtemp = stemp / good;

    // Population std dev: sqrt(E[x^2] - E[x]^2), guarded against the tiny
    // negative that floating-point rounding can produce when variance ~ 0.
    float vax = (float)(qax / good) - max * max;
    float vay = (float)(qay / good) - may * may;
    float vaz = (float)(qaz / good) - maz * maz;
    float nax = sqrtf(vax > 0 ? vax : 0);
    float nay = sqrtf(vay > 0 ? vay : 0);
    float naz = sqrtf(vaz > 0 ? vaz : 0);

    float gmag = sqrtf(max * max + may * may + maz * maz);

    Serial.printf("       %u/%u reads succeeded\n\n", good, N);
    Serial.printf("       mean accel   %+7.3f %+7.3f %+7.3f  g\n", max, may, maz);
    Serial.printf("       accel noise   %6.4f  %6.4f  %6.4f  g (std dev)\n", nax, nay, naz);
    Serial.printf("       gyro  bias   %+7.2f %+7.2f %+7.2f  deg/s\n", mgx, mgy, mgz);
    Serial.printf("       |a|          %7.3f g\n", gmag);
    Serial.printf("       die temp     %7.1f C\n", mtemp);
    Serial.println();

    // ── Checks ───────────────────────────────────────────────────────────────
    struct { const char *name; bool pass; const char *hint; } checks[] = {
        { "gravity magnitude",
          gmag >= LIM_GRAVITY_LO && gmag <= LIM_GRAVITY_HI,
          "|a| should be ~1.00 g at rest. Far off means wrong full-scale "
          "constant, or the board was moving." },
        { "gravity on Z axis",
          fabsf(maz) > 0.8f,
          "board is not flat, or Z is not the axis you think it is." },
        { "gyro bias in range",
          fabsf(mgx) < LIM_GYRO_BIAS && fabsf(mgy) < LIM_GYRO_BIAS
              && fabsf(mgz) < LIM_GYRO_BIAS,
          "large bias at rest is normal-ish for cheap parts but must be "
          "calibrated out before any angle integration." },
        { "accel noise floor",
          nax < LIM_ACCEL_NOISE && nay < LIM_ACCEL_NOISE && naz < LIM_ACCEL_NOISE,
          "noisy accel: unfiltered clone die (see stage 1), a shared/noisy "
          "3V3 rail, or the board is not actually still." },
        { "temperature plausible",
          mtemp > LIM_TEMP_LO && mtemp < LIM_TEMP_HI,
          "implausible die temperature — the byte order or the die "
          "identification is wrong." },
        { "data is changing",
          nax > 0.0f || nay > 0.0f || naz > 0.0f,
          "every sample identical means you are reading a frozen register, "
          "not live data." },
    };

    uint8_t failures = 0;
    for (auto &c : checks) {
        Serial.printf("       [%s] %s\n", c.pass ? "PASS" : "FAIL", c.name);
        if (!c.pass) { Serial.printf("              %s\n", c.hint); failures++; }
    }

    Serial.println();
    if (failures == 0) {
        Serial.println(F("       ALL CHECKS PASSED — the MPU-6050 is working."));
    } else {
        Serial.printf("       %u check(s) failed. Readings follow anyway so you can\n",
                      failures);
        Serial.println(F("       watch the numbers move while you investigate."));
    }

    // Paste-ready gyro offsets. Not a calibration — a starting point, and a
    // reminder that raw gyro data is biased.
    Serial.println();
    Serial.println(F("       Measured gyro bias, for src/config/calibration.h:"));
    Serial.printf("         #define GYRO_OFFSET_X  %+.2ff   // deg/s\n", mgx);
    Serial.printf("         #define GYRO_OFFSET_Y  %+.2ff\n", mgy);
    Serial.printf("         #define GYRO_OFFSET_Z  %+.2ff\n", mgz);
    Serial.println(F("       (Accel needs the six-position routine, not this.)"));
}

// ═════════════════════════════════════════════════════════════════════════════
//  Live stream
// ═════════════════════════════════════════════════════════════════════════════
static void streamHeader() {
    banner("LIVE  Tilt the board and watch the numbers");
    Serial.println(F("       Expected behaviour, to confirm axis orientation:"));
    Serial.println(F("         flat, chip up ....... az = +1 g"));
    Serial.println(F("         flat, chip down ..... az = -1 g"));
    Serial.println(F("         on its long edge .... ax or ay = +/-1 g"));
    Serial.println(F("         rotating ............ matching gyro axis spikes,"));
    Serial.println(F("                               returns to ~0 when you stop"));
    Serial.println();
    Serial.println(F("     ax      ay      az   |  gx      gy      gz   | pitch  roll | temp"));
    Serial.println(F("     (g)     (g)     (g)  | (deg/s)(deg/s)(deg/s) | (deg) (deg) | (C)"));
    Serial.println(F("   ------------------------------------------------------------------"));
}

void setup() {
    Serial.begin(BU_BAUD);
    delay(1500);  // USB CDC / monitor attach

    Serial.println();
    Serial.println(F("==============================================================="));
    Serial.println(F("  MPU-6050 STANDALONE BRING-UP"));
    Serial.println(F("  Wearable Health Monitor — stage 1 of sensor validation"));
    Serial.printf("  SDA=GPIO%u  SCL=GPIO%u  %lu baud\n",
                  BU_PIN_SDA, BU_PIN_SCL, (unsigned long) BU_BAUD);
    Serial.println(F("==============================================================="));

    Wire.begin(BU_PIN_SDA, BU_PIN_SCL);
    Wire.setTimeOut(50);  // ms — do not hang forever on a stuck bus

    stage0_scan();
    stage1_identity();
    stage2_wake();
    stage3_configure();
    stage4_reliability();
    stage5_sanity();
    streamHeader();
}

void loop() {
    Sample s;
    if (!readSample(s)) {
        Serial.println(F("   read failed"));
        delay(200);
        return;
    }

    // Tilt from gravity. Valid only when the board is not accelerating —
    // gravity and linear acceleration are indistinguishable to an
    // accelerometer, which is exactly why the gyro exists.
    float pitch = atan2f(-s.ax, sqrtf(s.ay * s.ay + s.az * s.az)) * 57.29578f;
    float roll  = atan2f(s.ay, s.az) * 57.29578f;

    Serial.printf("  %+6.3f  %+6.3f  %+6.3f | %+7.1f %+7.1f %+7.1f | %+5.1f %+5.1f | %4.1f\n",
                  s.ax, s.ay, s.az, s.gx, s.gy, s.gz, pitch, roll, s.tempC);

    delay(100);  // 10 Hz — readable on screen; the sensor itself runs at 100 Hz
}

/*
 * Copyright 2026 Phuthiphong Wongchantib
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *
 * Original author / contributor:
 * Phuthiphong Wongchantib
 */
/**
 * @file imu_only.cpp
 * @brief The BNO085 on its own, with nothing else running. A bring-up tool.
 *
 *     pio run -e imu_only -t upload
 *     pio device monitor -e imu_only
 *
 * WHY THIS EXISTS
 *
 * The IMU has been failing to start, and on the real firmware it is impossible
 * to say why with any confidence, because a dozen other things are happening on
 * the same board at the same time: Wi-Fi, micro-ROS, the 100 Hz control loop,
 * four INA226s and a magnetometer sharing the I2C bus, motors switching amps a
 * few centimetres away. Any of those could be the reason, and none of them can
 * be ruled out from inside a system that is running all of them.
 *
 * So this runs the sensor and NOTHING else. No Wi-Fi, no micro-ROS, no motors,
 * no other I2C device touched. If the BNO085 comes up here and not in the
 * firmware, the sensor is fine and the fault is contention or timing. If it
 * fails here too, it is the sensor or the wiring, and no amount of firmware
 * work will fix it.
 *
 * WHAT IT PRINTS
 *
 *   - every I2C address that answers, so "the sensor is wired" is checked
 *     rather than assumed
 *   - the raw SHTP header, which separates "nothing there" from "answers but is
 *     not running" from "talking normally"
 *   - each begin() attempt and whether it worked
 *   - once running: quaternion, roll/pitch/yaw, gyro, linear acceleration and
 *     the calibration accuracy 0-3, twice a second
 *
 * The accuracy is the number to watch. Move the board in slow figure-of-eights,
 * away from motors and batteries, and it should climb 0 -> 1 -> 2 -> 3. If it
 * sits at 0 while everything else looks healthy, the magnetometer is being
 * disturbed rather than the sensor being broken.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

// Deliberately repeated here rather than included from config/. This tool has
// to be able to disagree with the firmware's idea of the wiring - if the two
// ever differ, that difference is itself the bug, and sharing a header would
// hide it.
static const int      SDA_PIN  = 21;
static const int      SCL_PIN  = 22;
static const uint8_t  BNO_ADDR = 0x4A;      // 0x4B if the ADR pad is bridged
static const uint32_t I2C_HZ   = 400000;

Adafruit_BNO08x   bno(-1);                  // -1: no reset line on this board
sh2_SensorValue_t evt;

/** What is known to live at each address on THIS robot.
 *
 * Worth naming rather than printing bare numbers: a scan that lists seven
 * devices when you can account for six is a question, and the answer is far
 * easier to reach when the six are labelled. That is exactly how 0x76 was
 * noticed - four INA226s, a compass and the IMU add up to six, and something
 * else was answering. */
static const char *knownAt(uint8_t a) {
  switch (a) {
    case 0x0D: return "QMC5883L compass (in the GPS module)";
    case 0x40: return "INA226 - motor rail B";
    case 0x41: return "INA226 - processor rail";
    case 0x44: return "INA226 - motor rail A";
    case 0x45: return "INA226 - fan rail";
    case 0x4A: return "BNO085 IMU";
    case 0x76: return "UNUSED by this firmware - see the chip id below";
    case 0x77: return "UNUSED by this firmware - see the chip id below";
    default:   return "not expected on this robot";
  }
}

/** Read a chip-identity register, the way most sensors expose one.
 *
 * 0xD0 is the near-universal 'who are you' register on Bosch parts and several
 * others. It costs one transaction and turns "something is at 0x76" into a part
 * number. */
static void identify(uint8_t addr) {
  for (uint8_t reg = 0; reg < 2; ++reg) {
    const uint8_t r = reg ? 0x00 : 0xD0;          // Bosch first, then generic
    Wire.beginTransmission(addr);
    Wire.write(r);
    if (Wire.endTransmission(false) != 0) continue;
    if (Wire.requestFrom((int)addr, 1) != 1) continue;
    const uint8_t id = Wire.read();
    const char *name =
        (r == 0xD0 && id == 0x58) ? "BMP280 barometer" :
        (r == 0xD0 && id == 0x60) ? "BME280 barometer + humidity" :
        (r == 0xD0 && id == 0x61) ? "BME680" :
        (r == 0x00 && id == 0x10) ? "SPL06-001 barometer" :
        (r == 0x00 && id == 0x11) ? "SPA06 barometer" : "unknown part";
    Serial.printf("       reg 0x%02X -> id 0x%02X  (%s)\n", r, id, name);
  }
}

static void scanBus() {
  Serial.println("[i2c] scanning...");
  uint8_t found = 0;
  for (uint8_t a = 0x08; a < 0x78; ++a) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() != 0) continue;
    ++found;
    Serial.printf("  0x%02X  %s\n", a, knownAt(a));
    // Anything we cannot account for gets asked what it is. An unexplained
    // device is either a sensor going to waste or a wiring mistake, and both
    // are worth knowing about.
    if (a == 0x76 || a == 0x77) identify(a);
  }
  if (!found) Serial.println("  NONE - check SDA, SCL, 3V3 and ground");
  Serial.printf("[i2c] %u device(s)\n", found);
}

/** The first four bytes of any SHTP packet: length low, length high, channel,
 *  sequence. This is the one question that separates a sensor which is absent
 *  from one that is powered but not running its firmware. */
static void printHeader() {
  uint8_t h[4] = {0xFF, 0xFF, 0xFF, 0xFF};
  const bool ok = Wire.requestFrom((int)BNO_ADDR, 4) == 4;
  if (ok) for (uint8_t i = 0; i < 4; ++i) h[i] = Wire.read();
  const uint16_t len = (uint16_t)h[0] | (uint16_t)((h[1] & 0x7F) << 8);
  Serial.printf("[shtp] header %s %02X %02X %02X %02X   length %u  channel %u  -> %s\n",
                ok ? "read:" : "UNREADABLE:", h[0], h[1], h[2], h[3], len, h[2],
                !ok  ? "nothing readable at the address"
                : len == 0 ? "answers, but produces no packets (powered, not running)"
                           : "talking SHTP normally");
}

/**
 * Ask the sensor to actually CALIBRATE, not merely to report.
 *
 * This was missing, and its absence looked exactly like a broken magnetometer:
 * the board was moved properly - gyro peaking at 1.4 rad/s, orientation
 * swinging through 135 degrees - and the accuracy sat at 0/3 for three minutes.
 * The sensor was doing precisely what it had been told, which was to report and
 * nothing else.
 *
 * The BNO085 only runs its dynamic calibration when it is switched on. Once it
 * has saved a calibration record it stops adjusting, and a record saved on a
 * different robot - or before a battery was fitted next to it - is never
 * revisited. Enabling accel, gyro and magnetometer together costs nothing: the
 * same motion teaches all three.
 */
static void enableCalibration() {
  const int rc = sh2_setCalConfig(SH2_CAL_ACCEL | SH2_CAL_GYRO | SH2_CAL_MAG);
  Serial.printf("[cal] enable accel+gyro+mag -> %s (%d)\n",
                rc == SH2_OK ? "on" : "REFUSED", rc);
}

static void enableReports() {
  bno.enableReport(SH2_ROTATION_VECTOR, 20000);       // 50 Hz, magnetometer referenced
  bno.enableReport(SH2_GYROSCOPE_CALIBRATED, 20000);
  bno.enableReport(SH2_LINEAR_ACCELERATION, 20000);
  // THE MEASUREMENT THAT SETTLES THE ARGUMENT.
  //
  // Accuracy stuck at 0 says the sensor does not trust its magnetometer, but
  // not why. The field strength does: the Earth's is 25-65 microtesla and
  // around 41 here in Thailand. A reading near that, holding steady as the
  // board is turned, means the environment is fine and the problem is the
  // motion or the procedure. A reading of hundreds, or one that swings about
  // while the board is still, means iron or current nearby - and no amount of
  // waving fixes that.
  //
  // Guessing at magnets has already cost two rounds: a steel bolt was the
  // right answer, the fans were the wrong one. This replaces the guessing.
  bno.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 50000);
}

static bool tryBegin(uint8_t attempts) {
  for (uint8_t i = 1; i <= attempts; ++i) {
    Serial.printf("[begin] attempt %u of %u ... ", i, attempts);
    if (bno.begin_I2C(BNO_ADDR, &Wire)) {
      Serial.println("OK");
      enableCalibration();
      enableReports();
      return true;
    }
    Serial.println("failed");
    printHeader();
    delay(500);
  }
  return false;
}

static float qw = 1, qx = 0, qy = 0, qz = 0;
static float gx = 0, gy = 0, gz = 0;
static float ax = 0, ay = 0, az = 0;
static float mx = 0, my = 0, mz = 0;
static bool  mag_seen = false;
static uint8_t accuracy = 0;
static float   head_acc = 0;
static bool running = false;

void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println();
  Serial.println("=== BNO085 on its own - no Wi-Fi, no micro-ROS, no motors ===");

  Wire.begin(SDA_PIN, SCL_PIN, I2C_HZ);
  // Generous on purpose: there is no control loop here to protect, and the
  // SHTP handshake clock-stretches through the sensor's reset.
  Wire.setTimeOut(250);

  scanBus();
  printHeader();

  running = tryBegin(6);
  if (!running) {
    Serial.println();
    Serial.println("[result] the sensor did not start.");
    Serial.println("         If the scan listed 0x4A the wiring is fine and the");
    Serial.println("         chip is powered - it is the SHTP handshake failing.");
    Serial.println("         Remove ALL power (battery AND USB) for ten seconds");
    Serial.println("         and run this again; that is the only thing that has");
    Serial.println("         ever cleared it.");
  } else {
    Serial.println();
    Serial.println("[result] running. Move it in slow figure-of-eights, away from");
    Serial.println("         motors and batteries, and watch accuracy climb to 3.");
    Serial.println();
    Serial.println("         Press 's' to sweep the I2C clock against the mag report");
    Serial.println("         rate and see what each combination really delivers.");
    Serial.println("         Takes about a minute and answers which of the two");
    Serial.println("         changed values starved the magnetometer.");
    Serial.println();
  }
}

/**
 * Measure what the bus can actually deliver, one variable at a time.
 *
 * WHY THIS EXISTS: the magnetometer-referenced heading went from missing 87% of
 * the time to missing 100%, and TWO things had been changed to get there - the
 * report dropped from 50 Hz to 10 Hz, and the I2C clock dropped from 400 kHz to
 * 100 kHz. Either one could explain it and neither was measured, so the honest
 * position was "we do not know which", and the next change would have been a
 * third guess on top of two.
 *
 * This sweeps both, separately, and reports the DELIVERED rate of every report
 * against the rate it was asked for. It reproduces the firmware's real bus load
 * while it does so - the game rotation vector and the gyro at 100 Hz and linear
 * acceleration at 50 Hz, which is what the mag-referenced report has to compete
 * with - because measuring it on an idle bus would answer a question nobody
 * asked.
 *
 * Read the table like this: a row where rv delivered matches rv asked is a
 * combination that works. A row where the fast reports are at their full rate
 * and only rv has collapsed is bus saturation, not a broken magnetometer. A row
 * where everything collapses together is the bus clock being too slow for the
 * total load.
 */
static void runSweep() {
  static const uint32_t CLOCKS[] = { 100000, 200000, 400000 };
  static const uint32_t RV_US[]  = { 100000, 40000, 20000, 10000 };  // 10, 25, 50, 100 Hz
  const uint32_t WINDOW_MS = 3000;

  Serial.println();
  Serial.println("=== bus sweep: what actually arrives, per report ===");
  Serial.println("Load matches the firmware: game RV 100 Hz + gyro 100 Hz + accel 50 Hz,");
  Serial.println("plus the magnetometer-referenced rotation vector at the rate shown.");
  Serial.println();
  Serial.println(" bus kHz | rv asked | rv got | game got | gyro got | accel got | resets");
  Serial.println("---------+----------+--------+----------+----------+-----------+-------");

  for (unsigned c = 0; c < sizeof(CLOCKS) / sizeof(CLOCKS[0]); c++) {
    Wire.setClock(CLOCKS[c]);
    for (unsigned r = 0; r < sizeof(RV_US) / sizeof(RV_US[0]); r++) {
      bno.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);
      bno.enableReport(SH2_ROTATION_VECTOR, RV_US[r]);
      bno.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);
      bno.enableReport(SH2_LINEAR_ACCELERATION, 20000);

      // Let the new rates settle before counting. The BNO085 takes a few
      // hundred milliseconds to act on enableReport, and counting through that
      // would blame the settling time on the combination being tested.
      const uint32_t settle_until = millis() + 600;
      while (millis() < settle_until) { while (bno.getSensorEvent(&evt)) {} }

      uint32_t n_rv = 0, n_game = 0, n_gyro = 0, n_acc = 0, n_reset = 0;
      const uint32_t started = millis();
      while (millis() - started < WINDOW_MS) {
        if (bno.wasReset()) {
          n_reset++;
          bno.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);
          bno.enableReport(SH2_ROTATION_VECTOR, RV_US[r]);
          bno.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);
          bno.enableReport(SH2_LINEAR_ACCELERATION, 20000);
        }
        while (bno.getSensorEvent(&evt)) {
          switch (evt.sensorId) {
            case SH2_ROTATION_VECTOR:      n_rv++;   break;
            case SH2_GAME_ROTATION_VECTOR: n_game++; break;
            case SH2_GYROSCOPE_CALIBRATED: n_gyro++; break;
            case SH2_LINEAR_ACCELERATION:  n_acc++;  break;
            default: break;
          }
        }
      }
      const float secs = (float)(millis() - started) / 1000.0f;
      Serial.printf("   %4lu  |  %5.1f Hz | %5.1f  |  %6.1f  |  %6.1f  |  %7.1f  | %5lu\n",
                    (unsigned long)(CLOCKS[c] / 1000),
                    1000000.0f / (float)RV_US[r],
                    (float)n_rv / secs, (float)n_game / secs,
                    (float)n_gyro / secs, (float)n_acc / secs,
                    (unsigned long)n_reset);
    }
  }

  Serial.println();
  Serial.println("Pick the SLOWEST bus clock whose rv row still delivers what it was");
  Serial.println("asked for - a slower clock is kinder to a long or noisy cable, and");
  Serial.println("there is nothing to gain from headroom that goes unused.");
  Serial.println();

  // Leave the tool the way it was found, so the live readout that follows is
  // still the one described at the top of this file.
  Wire.setClock(I2C_HZ);
  enableReports();
}

void loop() {
  // One key, because this runs on a robot with a phone in the other hand.
  if (Serial.available()) {
    const int key = Serial.read();
    if ((key == 's' || key == 'S') && running) runSweep();
  }

  if (!running) {
    // Keep trying, slowly, and say so - so the tool is still useful if the
    // sensor comes back after being unplugged while this is connected.
    static uint32_t next = 0;
    if (millis() > next) {
      next = millis() + 5000;
      Serial.println("[retry] still trying...");
      printHeader();
      running = tryBegin(2);
      if (running) Serial.println("[result] it came up.");
    }
    return;
  }

  if (bno.wasReset()) {
    // A reset clears the calibration setting too, so turn it back on. Without
    // this a mid-run reset silently ends the calibration while the numbers keep
    // scrolling and look perfectly healthy.
    Serial.println("[reset] the sensor reset itself - re-enabling reports and calibration");
    enableCalibration();
    enableReports();
  }

  while (bno.getSensorEvent(&evt)) {
    switch (evt.sensorId) {
      case SH2_ROTATION_VECTOR:
        qw = evt.un.rotationVector.real;
        qx = evt.un.rotationVector.i;
        qy = evt.un.rotationVector.j;
        qz = evt.un.rotationVector.k;
        // status bits 1-0 hold the 0-3 accuracy. rotationVector.accuracy is
        // the heading uncertainty in RADIANS and reads near zero when the
        // sensor is HAPPY - using it as the 0-3 status inverts the meaning.
        accuracy = (uint8_t)(evt.status & 0x03);
        head_acc = evt.un.rotationVector.accuracy;
        break;
      case SH2_GYROSCOPE_CALIBRATED:
        gx = evt.un.gyroscope.x; gy = evt.un.gyroscope.y; gz = evt.un.gyroscope.z;
        break;
      case SH2_LINEAR_ACCELERATION:
        ax = evt.un.linearAcceleration.x;
        ay = evt.un.linearAcceleration.y;
        az = evt.un.linearAcceleration.z;
        break;
      case SH2_MAGNETIC_FIELD_CALIBRATED:
        mx = evt.un.magneticField.x;
        my = evt.un.magneticField.y;
        mz = evt.un.magneticField.z;
        mag_seen = true;
        break;
      default: break;
    }
  }

  static uint32_t next_print = 0;
  if (millis() > next_print) {
    next_print = millis() + 500;

    const float sqw = qw * qw, sqx = qx * qx, sqy = qy * qy, sqz = qz * qz;
    const float yaw   = atan2f(2.0f * (qx * qy + qz * qw), sqx - sqy - sqz + sqw) * 57.29578f;
    float s = -2.0f * (qx * qz - qy * qw) / (sqx + sqy + sqz + sqw);
    if (s > 1.0f) s = 1.0f;
    if (s < -1.0f) s = -1.0f;
    const float pitch = asinf(s) * 57.29578f;
    const float roll  = atan2f(2.0f * (qy * qz + qx * qw), -sqx - sqy + sqz + sqw) * 57.29578f;

    const float field = sqrtf(mx * mx + my * my + mz * mz);
    // Earth's field is 25-65 uT, about 41 uT in Thailand. Anything far outside
    // that is iron or current, not a sensor that needs more waving.
    const char *verdict = !mag_seen  ? "no mag report"
                        : field < 15.0f ? "TOO WEAK - shielded or saturated"
                        : field > 80.0f ? "TOO STRONG - iron or current nearby"
                                        : "field looks like the Earth's";
    Serial.printf("acc %u/3 %-14s | rpy %7.1f %6.1f %6.1f | gyro %6.2f %6.2f %6.2f "
                  "| accel %6.2f %6.2f %6.2f | mag %6.1f uT | head +-%5.1f deg  %s\n",
                  accuracy,
                  accuracy >= 2 ? "(usable)" : "(move it more)",
                  roll, pitch, yaw, gx, gy, gz, ax, ay, az,
                  field, head_acc * 57.29578f, verdict);
  }
}

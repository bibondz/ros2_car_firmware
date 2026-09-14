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
 * @file imu_bno085.h
 * @brief BNO085 wrapper: fused orientation, yaw rate and gravity-free acceleration.
 *
 * Three reports are enabled:
 *   - rotation vector      : quaternion + heading (game RV = no magnetometer)
 *   - gyroscope calibrated : yaw rate for the heading PID and wheel-speed split
 *   - linear acceleration  : forward acceleration for the speed estimator
 *
 * The BNO085 can silently reset (brown-out, ESD). wasReset() is checked on
 * every update and the reports are re-enabled automatically; ok() goes false
 * while no event has arrived for stale_ms so the safety layer can react.
 */
#ifndef IMU_BNO085_H
#define IMU_BNO085_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

class ImuBno085 {
public:
  ImuBno085(int reset_pin = -1) : bno_(reset_pin) {}

  /** Which orientation reports to enable. BOTH costs a little I2C bandwidth and
   *  buys a second, independent absolute heading source for the fallback chain. */
  enum Mode { MODE_GAME = 0, MODE_MAG = 1, MODE_BOTH = 2 };

  bool begin(TwoWire *wire, uint8_t addr, Mode mode,
             uint32_t report_interval_us = 10000) {
    wire_        = wire;
    addr_        = addr;
    mode_        = mode;
    use_mag_     = (mode == MODE_MAG);
    interval_us_ = report_interval_us;

    // Boot may take several attempts; the control loop is not running yet, so
    // a couple of seconds here costs nothing.
    begun_ = false;
    ok_ = false;
    retries_ = 0;
    return retryBegin(BEGIN_ATTEMPTS);
  }

  /** Did begin() ever succeed? Everything else is gated on this. */
  bool begun() const { return begun_; }

  /**
   * Try begin() again when it has never worked. Safe to call at any cadence.
   *
   * WHY THIS EXISTS: BNO085_RESET_PIN is -1 on this board - the sensor's reset
   * line is not wired to anything. A power-on brings it up fine, but an ESP32
   * *software* reset does not touch it, so it carries whatever I2C state it was
   * left in and begin_I2C() fails. Once that happened the driver gave up for
   * good and the IMU stayed dead until someone pulled the power.
   *
   * That turned every Wi-Fi firmware update into a trip to the robot, which is
   * the one thing OTA exists to avoid. Found exactly that way: flash over
   * Wi-Fi, and the board comes back with imu_missing and a calibration that
   * reports "the IMU never started".
   *
   * DO NOT ADD A SOFT RESET HERE. It was tried, and it made things worse. The
   * Adafruit HAL's own open already writes the identical SHTP reset packet -
   * {5, 0, 1, 0, 1} on the executable channel - and then waits 300 ms for the
   * sensor to come back. Sending one first only meant the library's reset
   * arrived while the sensor was still rebooting from ours, so the handshake
   * ran against a device that was not ready. The sensor is not the problem
   * here: asked directly it ACKs its address AND returns a real SHTP packet.
   *
   * Retrying costs one failed I2C probe per interval, and the bus already has a
   * timeout, so a sensor that is genuinely absent is not expensive.
   */
  /**
   * @param attempts how many handshakes to try before giving up this round.
   *
   * ONE by default, and that default matters. Each attempt sends an SHTP reset
   * and the library waits 300 ms for the sensor, so a burst of four costs the
   * best part of three seconds - inside the control loop. Measured under motor
   * load with the IMU down: the loop stalled for 1.33 SECONDS, which is 133
   * missed control periods, and telemetry stalled with it. A recovery attempt
   * that stops the robot responding is not a recovery.
   *
   * So the loop retries once per tick and boot gets the burst, where there is
   * no loop to protect.
   */
  bool retryBegin(uint8_t attempts = 1) {
    if (begun_) return true;
    if (wire_ == nullptr) return false;         // begin() was never called at all
    retries_++;

    // The backlog is drained before each try so the handshake starts from an
    // empty pipe rather than the tail of whatever was in flight.
    for (uint8_t attempt = 0; attempt < attempts; ++attempt) {
      if (attempt) delay(BEGIN_RETRY_GAP_MS);
      drain();
      if (bno_.begin_I2C(addr_, wire_)) { cal_rc_ = 0; goto started; }
    }
    // Record WHY, so a manual retry from the web reports something true.
    // Without this the last calibration command's code was still sitting
    // there, and a failed retry cheerfully answered "ok".
    cal_rc_ = diagnose();
    return false;

  started:
    enableReports();
    begun_ = true;
    ok_ = false;                                // fresh data has still to arrive
    last_event_ms_ = millis();
    // The rate window starts when the reports do, so the first reading is not
    // diluted by however long the board spent booting before this point.
    resetReportCounts();
    return true;
  }

  uint16_t beginRetries() const { return retries_; }

  /** Let the automatic retry have another run of attempts.
   *
   * The cap exists so a permanently absent sensor does not hold the I2C bus
   * every thirty seconds forever. It has to be liftable, because the operator
   * can change the very thing the cap assumes is unchangeable: pulling USB and
   * the battery genuinely resets the sensor, and after that it deserves another
   * go without reflashing the board to clear a counter. */
  void resetRetries() { retries_ = 0; }

  /**
   * Read and throw away everything the sensor still has queued.
   *
   * WHY: the BNO085's reset line is not wired on this board, so an ESP32
   * software reset - which is what a USB flash and an over-the-air update both
   * are - leaves the sensor running, still streaming the reports the previous
   * session asked for. sh2_open() then tries to handshake against a stream
   * already in progress. Sometimes it syncs and sometimes it does not, which is
   * exactly the intermittent "IMU never started" we keep seeing: the sensor
   * answers its address AND returns a real SHTP packet, so it is plainly alive,
   * yet begin_I2C() fails.
   *
   * Clearing the backlog first means the library's own reset - it sends the
   * SHTP reset packet and waits 300 ms, so do NOT add another one here - starts
   * from an empty pipe.
   *
   * Bounded twice over: at most MAX_DRAIN packets, and each body read is capped,
   * so a sensor babbling nonsense cannot hold the loop.
   */
  void drain() {
    if (wire_ == nullptr) return;
    for (uint8_t packet = 0; packet < MAX_DRAIN; ++packet) {
      uint8_t hdr[4];
      if (!probeHeader(hdr)) return;
      // Bit 15 of the length is the continuation flag, not part of the length.
      uint16_t len = (uint16_t)hdr[0] | (uint16_t)((hdr[1] & 0x7F) << 8);
      if (len <= 4) return;                     // nothing left to read
      len -= 4;                                 // the header we already took
      while (len > 0) {
        const uint8_t chunk = (len > 32) ? 32 : (uint8_t)len;
        if (wire_->requestFrom((int)addr_, (int)chunk) != chunk) return;
        while (wire_->available()) (void)wire_->read();
        len -= chunk;
      }
    }
  }

  /**
   * Read one raw SHTP header straight off the bus, bypassing the library.
   *
   * "begin() failed" and "the address ACKs" are both true here and neither says
   * what is wrong. This asks the sensor the only question that separates the
   * cases: is it speaking SHTP at all? The first four bytes of any packet are
   * length LSB, length MSB, channel, sequence. A freshly reset BNO085 answers
   * with its advertisement - a couple of hundred bytes on channel 0. All ones,
   * or all zeros, or a length of zero mean it is not talking, whatever its
   * address does.
   *
   * @return true when four bytes were actually read.
   */
  bool probeHeader(uint8_t out[4]) {
    out[0] = out[1] = out[2] = out[3] = 0xFF;
    if (wire_ == nullptr) return false;
    if (wire_->requestFrom((int)addr_, 4) != 4) return false;
    for (uint8_t i = 0; i < 4; ++i) out[i] = (uint8_t)wire_->read();
    return true;
  }

  /**
   * Service the sensor. Bounded by BOTH a count and a time budget.
   *
   * The count alone was not a bound on anything that matters. Eight events
   * sounds small until you notice what one costs: SHTP delivers a whole cargo
   * per report, and a few hundred bytes on a 100 kHz bus is tens of
   * milliseconds. Measured on the robot with the sensor finally streaming, this
   * function held the 100 Hz control loop for 45 to 161 ms per tick - against
   * 0.56 ms on the same firmware while the sensor was dead and there was
   * nothing to read. The loop was running at roughly 10 Hz whenever the IMU
   * worked, which also starved the reports it was trying to collect: about 13%
   * of what had been asked for arrived.
   *
   * So the real bound is time. Whatever the bus does - a slow clock, a
   * stretched transfer, a backlog after a reset - a tick costs at most
   * UPDATE_BUDGET_US and the rest waits for the next one. Reports are not lost
   * by stopping early; they are queued in the sensor and read next time.
   *
   * @param max_events  still a cap, as a second belt on a sensor babbling.
   */
  bool update(uint8_t max_events = 8) {
    const uint32_t started_us = micros();
    // Never touch the driver if begin() failed. The Adafruit BNO08x library
    // leaves its SHTP state uninitialised in that case, so getSensorEvent()
    // dereferences a null pointer and the ESP32 dies with
    //     Guru Meditation Error: LoadProhibited
    // inside shtp_service.
    //
    // Found on real hardware with no IMU attached: the board crashed and
    // rebooted roughly every 15 s, forever. It matters with an IMU fitted
    // too - a loose connector or an I2C fault at boot turns a robot that
    // should degrade into one that crash-loops.
    // Guard on begun_, NOT on ok_. Conflating the two cost the IMU entirely:
    // ok_ means "data is fresh", so the first gap longer than stale_ms_ made it
    // false, and this early return then stopped the driver polling the sensor
    // ever again. One late report at start-up latched the IMU dead until reboot.
    //
    // The BNO085 takes well over 500 ms to start reporting after
    // enableReport(), so that gap happened on EVERY boot. Seen on hardware: the
    // sensor answering at 0x4A, begin() succeeding, and sensor_health still
    // saying imu_missing forever afterwards.
    //
    // begun_ is what the crash guard actually needs anyway - it only has to
    // know begin() succeeded, so the driver's SHTP state is initialised.
    if (!begun_) return false;
    // A reset means the reports have to be asked for again - but NOT all four
    // right here. Each enableReport() is a feature command that waits on the
    // sensor, and doing the set of them inline stalled the control loop for up
    // to 178 MILLISECONDS, measured on the robot: seventeen missed control
    // periods, every time the sensor reset. It was resetting five times a
    // minute, so the loop was being held for a sixth of a second, repeatedly,
    // while the robot was meant to be driving.
    //
    // This is the same lesson the begin() retry burst taught - a recovery
    // attempt must not stop the robot responding - and it was reintroduced
    // here by a path nobody was looking at. One command per tick: the reports
    // are all back within four control periods, which the sensor does not
    // notice, and no single tick is ever held.
    if (bno_.wasReset()) { resets_++; pending_enables_ = ENABLE_ALL; }
    servicePendingEnables();

    // Lifetime counts survived the 49.7-day millis() wrap while their elapsed
    // time did not, producing millions of Hz. Restart both together before
    // counting new events, even when no reports arrive, so a stopped or slowed
    // sensor is measured over recent time instead of its healthy lifetime.
    if ((uint32_t)(millis() - rate_since_ms_) >= RATE_WINDOW_MS) resetReportCounts();

    bool got = false;
    while (max_events--) {
      // Checked BEFORE the read, not after: the point is to avoid starting a
      // transfer there is no time left for, and a read already begun cannot be
      // taken back.
      if ((uint32_t)(micros() - started_us) >= UPDATE_BUDGET_US) {
        budget_hits_++;
        break;
      }
      if (!bno_.getSensorEvent(&evt_)) break;
      switch (evt_.sensorId) {
        case SH2_ROTATION_VECTOR: {
          // magnetometer-referenced: absolute, but the BNO085 sits near the motors
          float yaw = yawFromQuat(evt_.un.rotationVector.real, evt_.un.rotationVector.i,
                                  evt_.un.rotationVector.j, evt_.un.rotationVector.k);
          mag_yaw_deg_ = yaw;
          // status bits 1-0 are the 0-3 calibration accuracy. NOT
          // rotationVector.accuracy - that is the heading uncertainty in
          // RADIANS, a float, and assigning it here silently truncated it to an
          // integer. A well calibrated sensor reports about 0.05 rad, which
          // became 0; a badly calibrated one reports 1-2 rad, which became 1 or
          // 2. The scale was therefore INVERTED - "0/3" meant excellent and
          // "2/3" meant poor - and every conclusion drawn from it was upside
          // down. Cost hours of hunting magnets that were never the problem.
          accuracy_ = (uint8_t)(evt_.status & 0x03);
          heading_acc_rad_ = evt_.un.rotationVector.accuracy;
          mag_yaw_ms_ = millis();
          n_rv_++;
          if (use_mag_) setQuat(evt_.un.rotationVector.real, evt_.un.rotationVector.i,
                                evt_.un.rotationVector.j, evt_.un.rotationVector.k);
          got = true;
        } break;
        case SH2_GAME_ROTATION_VECTOR:
          // gyro/accel only: smooth and immune to magnets, but relative
          n_game_++;
          // The game vector reports its OWN 0-3 confidence in status bits 1-0,
          // for the accelerometer-and-gyro fusion it actually runs. That is not
          // magnetometer accuracy and must not be confused with it - but it IS
          // the number to judge this sensor by when the magnetometer-referenced
          // report is not being asked for at all.
          //
          // Without this, accuracy sat at 0 for ever the moment the report mode
          // changed to game-only: the only writer was the OTHER rotation
          // vector's handler. The dashboard then showed a permanently
          // uncalibrated sensor, the magnetometer was reported missing for ever,
          // and the automatic calibration save - which waits for 3/3 - became
          // unreachable code.
          accuracy_ = (uint8_t)(evt_.status & 0x03);
          if (!use_mag_) {
            setQuat(evt_.un.gameRotationVector.real, evt_.un.gameRotationVector.i,
                    evt_.un.gameRotationVector.j, evt_.un.gameRotationVector.k);
            got = true;
          }
          break;
        case SH2_GYROSCOPE_CALIBRATED:
          n_gyro_++;
          gyro_x_ = evt_.un.gyroscope.x;
          gyro_y_ = evt_.un.gyroscope.y;
          gyro_z_ = evt_.un.gyroscope.z;
          got = true;
          break;
        case SH2_LINEAR_ACCELERATION:
          n_acc_++;
          acc_x_ = evt_.un.linearAcceleration.x;
          acc_y_ = evt_.un.linearAcceleration.y;
          acc_z_ = evt_.un.linearAcceleration.z;
          got = true;
          break;
        default:
          break;
      }
    }
    if (got) last_event_ms_ = millis();
    ok_ = (millis() - last_event_ms_) < stale_ms_;
    return got;
  }

  bool  ok()        const { return ok_; }
  float yawDeg()    const { return yaw_deg_; }     // 0..360, CCW positive (ENU convention)
  float pitchDeg()  const { return pitch_deg_; }
  float rollDeg()   const { return roll_deg_; }
  float yawRateDps() const { return gyro_z_ * 57.29578f; }
  float gyroX()     const { return gyro_x_; }      // [rad/s]
  float gyroY()     const { return gyro_y_; }
  float gyroZ()     const { return gyro_z_; }
  float accX()      const { return acc_x_; }       // [m/s^2], gravity removed
  float accY()      const { return acc_y_; }
  float accZ()      const { return acc_z_; }
  float qw()        const { return qw_; }
  float qx()        const { return qx_; }
  float qy()        const { return qy_; }
  float qz()        const { return qz_; }
  uint8_t accuracy() const { return accuracy_; }   // 0 unreliable .. 3 high (rotation vector only)
  /** The sensor's estimate of how wrong its heading might be, in radians.
   *  Small is good - about 0.05 rad on a settled sensor. */
  float headingAccuracyRad() const { return heading_acc_rad_; }
  /** How long since a magnetometer-referenced report arrived. */
  uint32_t magYawAgeMs() const {
    return mag_yaw_ms_ == 0 ? 0xFFFFFFFFu : (millis() - mag_yaw_ms_);
  }
  uint16_t resets()  const { return resets_; }
  void setStaleMs(uint32_t ms) { stale_ms_ = ms; }

  //--------------------------- calibration ---------------------------------//
  /**
   * Turn the BNO085's own continuous calibration back on.
   *
   * The sensor calibrates itself from motion, but once it has saved a good
   * calibration record it stops adjusting. If the robot then changes - a magnet
   * moves, a battery is fitted next to it, the sensor is remounted - the saved
   * record is wrong and the accuracy it reports never recovers on its own.
   * This asks it to start learning again; figure-of-eight motion is what feeds
   * it. accuracy() says when it has got there: 2 or better is what the heading
   * chain requires before it will use this magnetometer at all.
   *
   * Accel and gyro are included because the same motion calibrates all three
   * and there is no reason to leave two of them stale.
   */
  bool startCalibration() {
    if (!begun_) { cal_rc_ = diagnose(); return false; }
    cal_rc_ = sh2_setCalConfig(SH2_CAL_ACCEL | SH2_CAL_GYRO | SH2_CAL_MAG);
    calibrating_ = (cal_rc_ == SH2_OK);
    return calibrating_;
  }

  /** Write the calibration into the sensor's own flash so it survives a reboot.
   *  Without this the figure-of-eight has to be repeated after every power cycle. */
  bool saveCalibration() {
    if (!begun_) { cal_rc_ = diagnose(); return false; }
    cal_rc_ = sh2_saveDcdNow();
    if (cal_rc_ == SH2_OK) calibrating_ = false;
    return cal_rc_ == SH2_OK;
  }

  /** Throw away the stored calibration and restart the sensor. The counterpart
   *  of the compass's clear: a saved calibration that is wrong stays wrong. */
  bool clearCalibration() {
    if (!begun_) { cal_rc_ = diagnose(); return false; }
    cal_rc_ = sh2_clearDcdAndReset();
    if (cal_rc_ == SH2_OK) { accuracy_ = 0; calibrating_ = false; enableReports(); }
    return cal_rc_ == SH2_OK;
  }

  bool calibrating() const { return calibrating_; }

  /** What the sensor said to the last calibration command.
   *
   * SH2_OK is 0; everything else is one of the negative codes in sh2_err.h. It
   * is kept because "refused" on its own is not a diagnosis - a sensor that is
   * not there, one that is busy, and one that rejected the command outright all
   * look the same from the web, and they need different actions. */
  int lastCalibrationResult() const { return cal_rc_; }

  // Our own codes, chosen outside the sh2_err.h range, for the cases sh2 never
  // gets to see. They are separate because they call for different actions and
  // used to be indistinguishable - "the IMU is missing" covered all three.
  static const uint8_t MAX_DRAIN = 8;      // packets discarded before giving up
  static const uint8_t BEGIN_ATTEMPTS = 4;  // tries per burst
  static const uint16_t BEGIN_RETRY_GAP_MS = 400;
  static const int NOT_BEGUN = -100;   // begin() never ran, or the cause is unclear
  static const int NO_DEVICE = -101;   // nothing ACKs at the address: wiring or address
  static const int NO_SHTP   = -102;   // it ACKs but produces no packets: powered, not running
  // The handshake worked and the sensor then refused to turn any report on.
  // This was invisible: enableReport() returns a bool and nothing looked at it,
  // so a sensor that opened cleanly and then delivered NOTHING reported "ok"
  // for its last operation and "missing" for its data, which are two true
  // statements that together explain nothing. Measured on the robot: rv, game,
  // gyro and accel all at 0.0 Hz against 10, 100, 100 and 50 asked for.
  static const int NO_REPORTS = -104;

  /**
   * Work out WHY the sensor never started, using the bus rather than the library.
   *
   * The three answers need three different actions, and until this existed they
   * all arrived as one: "imu_missing". Seen on this robot - the bus scan listed
   * 0x4A quite happily and begin() failed anyway, with the raw SHTP header
   * reading 00 00 00 00. That is a sensor that is powered and answering its
   * address while its firmware is not running, which no amount of retrying will
   * fix and a power cycle will. Nothing in the previous output said so.
   */
  int diagnose() {
    if (wire_ == nullptr) return NOT_BEGUN;
    wire_->beginTransmission(addr_);
    if (wire_->endTransmission() != 0) return NO_DEVICE;
    uint8_t hdr[4];
    if (!probeHeader(hdr)) return NO_SHTP;
    // Bit 15 of the length is the continuation flag, not part of the length.
    const uint16_t len = (uint16_t)hdr[0] | (uint16_t)((hdr[1] & 0x7F) << 8);
    return (len == 0) ? NO_SHTP : NOT_BEGUN;
  }

  /** Is the magnetometer-referenced rotation vector being asked for at all?
   *
   *  When it is not, its heading is absent by choice rather than by fault, and
   *  reporting it as a missing sensor is a lie that costs someone an afternoon.
   */
  bool magRequested() const { return mode_ != MODE_GAME; }

  /** Absolute heading from the BNO085's own magnetometer - third source in the
   *  fallback chain, used when the QMC5883L is missing or disturbed. */
  float magYawDeg() const { return mag_yaw_deg_; }
  bool  magYawOk() const {
    return mag_yaw_ms_ != 0 && (millis() - mag_yaw_ms_) < magStaleMs() && accuracy_ >= 2;
  }

  /**
   * How long a magnetometer-referenced heading may go unrefreshed.
   *
   * NOT stale_ms_. That number is 500 ms and it belongs to the FAST reports -
   * the game rotation vector and the gyro, both at 100 Hz, where 500 ms is
   * fifty missed reports and unambiguously a dead sensor.
   *
   * The mag-referenced rotation vector runs at 10 Hz. Judged against the same
   * 500 ms it was allowed to miss FIVE reports before being called missing, and
   * on a busy I2C bus five in a row is an ordinary hiccup rather than a fault.
   * That is what made imu_magnetometer_missing flap - "have and lost, have and
   * lost" - while the sensor sat at a steady 3/3 accuracy and nothing was
   * actually wrong with it. The flag was measuring bus contention and calling
   * it a broken magnetometer.
   *
   * Ten report periods, floored at stale_ms_ so a fast report rate can never
   * make this tighter than the rest of the driver. At 10 Hz that is one second.
   */
  uint32_t magStaleMs() const {
    const uint32_t ten_periods = (MAG_REPORT_US / 1000u) * 10u;
    return ten_periods > stale_ms_ ? ten_periods : stale_ms_;
  }

  //------------------------- report accounting -----------------------------//
  /**
   * How many of each report actually arrived, and how fast.
   *
   * WHY THIS EXISTS: the magnetometer was reported missing 100% of the time
   * after two things were changed together - the mag report rate dropped to
   * 10 Hz and the I2C bus dropped to 100 kHz. Either could explain it, and with
   * no per-report accounting there was no way to tell which, so the next step
   * would have been another guess.
   *
   * Four reports share this bus: the game rotation vector and the gyro at
   * 100 Hz, linear acceleration at 50 Hz, and the mag-referenced rotation
   * vector at 10 Hz. If the bus is saturated it is the 10 Hz one that starves
   * first and the fast ones that look perfectly healthy - which is exactly the
   * symptom. Counting each kind separately turns "the magnetometer is missing"
   * into "the magnetometer is asking for 10 reports a second and getting 1",
   * which names the cause.
   */
  uint32_t rvReports()   const { return n_rv_; }
  uint32_t gameReports() const { return n_game_; }
  uint32_t gyroReports() const { return n_gyro_; }
  uint32_t accReports()  const { return n_acc_; }

  /** Measured rate over the current window, restarted at least every ten seconds
   *  while update() runs. The first 100 ms is too short to measure a useful rate. */
  float rvHz()   const { return rateOf(n_rv_); }
  float gameHz() const { return rateOf(n_game_); }
  float gyroHz() const { return rateOf(n_gyro_); }
  float accHz()  const { return rateOf(n_acc_); }

  /** What each report was ASKED for, so measured can be compared with expected. */
  float rvWantHz()   const { return 1000000.0f / (float)MAG_REPORT_US; }
  float gameHzWant() const { return 1000000.0f / (float)interval_us_; }

  uint32_t rateWindowMs() const { return millis() - rate_since_ms_; }

  static const uint32_t RATE_WINDOW_MS = 10000;

  /** Start a fresh measurement window. Call before timing a change. */
  void resetReportCounts() {
    n_rv_ = n_game_ = n_gyro_ = n_acc_ = 0;
    rate_since_ms_ = millis();
  }

private:
  float rateOf(uint32_t count) const {
    const uint32_t ms = millis() - rate_since_ms_;
    // A caller may read diagnostics before update() resumes after a long gap.
    // That expired bucket no longer describes the current report rate.
    return ms < 100u || ms >= RATE_WINDOW_MS ? 0.0f : (float)count * 1000.0f / (float)ms;
  }

  void enableReports() {
    // Ask for only what each report is actually used for.
    //
    // Asking for the magnetometer-referenced rotation vector at 50 Hz alongside
    // the game rotation vector at 100 Hz was too much: MODE_BOTH runs two
    // separate fusions, and the mag-referenced one was then delivered in bursts
    // - a second of reports, then ten seconds of nothing. Its timestamp went
    // stale, magYawOk() went false, and imu_magnetometer_missing flapped 33
    // times in two minutes while the sensor reported a steady 3/3 accuracy and
    // only 4 of those flips were anywhere near a sensor reset.
    //
    // The mag-referenced heading is a SLOW reference. It is consulted when the
    // robot is standing still, eased in over a time constant of seconds, and
    // exists to stop the gyro drifting - nothing about it needs 50 Hz. At 10 Hz
    // it is delivered reliably, which is worth far more than a rate nothing
    // uses, and it leaves the bandwidth for the reports that are used fast.
    //
    // EVERY RETURN VALUE IS CHECKED, and that is not defensive habit - it is
    // the only thing that separates two failures which look identical from
    // outside. A sensor whose handshake succeeded and whose reports were all
    // refused reports "ok" for its last operation and "missing" for its data.
    // Both statements are true and neither one names the fault. It happened on
    // this robot: rv, game, gyro and accel all at 0.0 Hz against 10, 100, 100
    // and 50 asked for, with a clean open and zero resets.
    uint8_t asked = 0, got = 0;
    // The game rotation vector at HALF the loop rate.
    //
    // MODE_BOTH runs two independent fusions inside the sensor and the slow
    // magnetometer-referenced one loses: measured at 0.0 to 1.0 Hz against the
    // 10 it asks for, while the fast reports get 50 to 70. The sensor is
    // delivering about 150 reports a second against the 260 asked for, so
    // something has to give and it is always the same something.
    //
    // Halving this was tried once before and looked like it made things worse.
    // That measurement was worthless: the Wi-Fi radio was browning the sensor
    // out at the time, it was resetting every two seconds, and the reset storm
    // swamped any effect the rate had. Re-run properly with the radio turned
    // down, it helps the FAST reports and does nothing at all for the slow one:
    // gyro went from 48-72 Hz to 72-83, accel from 33-49 to 39-46, the loop
    // from 2.8-9.3 ms to 1.1-6.9 - and the magnetometer-referenced vector
    // stayed at 0.0-0.5 Hz, exactly where it was.
    //
    // So the starvation is NOT bandwidth, and no rate change will fix it. The
    // BNO085 will not stream its magnetometer-referenced rotation vector
    // reliably alongside the game vector; MODE_BOTH asks for two fusions and
    // delivers one. The halved rate is kept because it is better for the
    // reports that do arrive, not because it solved anything.
    //
    // This costs less than it sounds. The magnetometer-referenced vector is the
    // THIRD absolute heading source, behind GPS course and the QMC5883L, and
    // the QMC5883L works. If it is ever needed as the primary, MODE_MAG drops
    // the game vector entirely and should stream it - untested, because nothing
    // has needed it.
    //
    // Nothing reads orientation at 100 Hz. The heading PID takes its yaw RATE
    // from the gyroscope, which is a separate report still running at the full
    // rate; the game vector supplies orientation, and 50 Hz is finer than a
    // robot turning at 45 deg/s can use.
    if (mode_ != MODE_MAG) {
      asked++; if (bno_.enableReport(SH2_GAME_ROTATION_VECTOR, interval_us_ * 2)) got++;
    }
    if (mode_ != MODE_GAME) {
      asked++; if (bno_.enableReport(SH2_ROTATION_VECTOR, MAG_REPORT_US)) got++;
    }
    asked++; if (bno_.enableReport(SH2_GYROSCOPE_CALIBRATED, interval_us_)) got++;
    asked++; if (bno_.enableReport(SH2_LINEAR_ACCELERATION, interval_us_ * 2)) got++;

    reports_asked_ = asked;
    reports_on_    = got;
    // Only overwrite a success code. A real sh2 error from a calibration
    // command is more specific than this and must not be masked by it.
    if (got == 0 && cal_rc_ == 0) cal_rc_ = NO_REPORTS;
    pending_enables_ = 0;                 // all four asked for, inline
  }

  /**
   * Ask for at most ONE outstanding report per call.
   *
   * Called from update(), which runs inside the 100 Hz control loop, so the
   * cost of a tick has to be one sensor command at most. Re-enabling all four
   * inline after a reset held the loop for up to 178 ms - seventeen missed
   * control periods - and the sensor was resetting often enough for that to be
   * a repeated stall rather than a one-off.
   *
   * Four ticks to restore every report is 40 ms, which is shorter than the gap
   * the reset itself already caused.
   */
  void servicePendingEnables() {
    if (!pending_enables_) return;
    if (pending_enables_ & ENABLE_GAME) {
      pending_enables_ &= ~ENABLE_GAME;
      // Same halved rate as enableReports(). The two have to agree, or a sensor
      // reset would quietly restore the rate that starves the magnetometer.
      if (mode_ != MODE_MAG) bno_.enableReport(SH2_GAME_ROTATION_VECTOR, interval_us_ * 2);
      return;
    }
    if (pending_enables_ & ENABLE_RV) {
      pending_enables_ &= ~ENABLE_RV;
      if (mode_ != MODE_GAME) bno_.enableReport(SH2_ROTATION_VECTOR, MAG_REPORT_US);
      return;
    }
    if (pending_enables_ & ENABLE_GYRO) {
      pending_enables_ &= ~ENABLE_GYRO;
      bno_.enableReport(SH2_GYROSCOPE_CALIBRATED, interval_us_);
      return;
    }
    if (pending_enables_ & ENABLE_ACCEL) {
      pending_enables_ &= ~ENABLE_ACCEL;
      bno_.enableReport(SH2_LINEAR_ACCELERATION, interval_us_ * 2);
      return;
    }
  }

  static const uint8_t ENABLE_GAME  = 1 << 0;
  static const uint8_t ENABLE_RV    = 1 << 1;
  static const uint8_t ENABLE_GYRO  = 1 << 2;
  static const uint8_t ENABLE_ACCEL = 1 << 3;
  static const uint8_t ENABLE_ALL   = 0x0F;

  /* How long one tick may spend reading the sensor.
   *
   * The control period is 10 ms.
   *
   * It was two milliseconds, on the reasoning that a fifth of the period is
   * generous for one peripheral. Measured, it was not: the sensor was asked for
   * 200 reports a second and only 119 were being collected, and the shortfall
   * fell almost entirely on the rotation vector - 3 Hz of the 50 asked, while
   * the gyroscope got 70 of 100 and linear acceleration 46 of 50. Events are
   * drained in arrival order, so a budget that runs out part way through
   * consistently starves whatever sits at the back, and the queue then backs up
   * inside the sensor.
   *
   * Four milliseconds still leaves more than half the period, and the loop has
   * been measuring 3 to 8 ms with the old budget, so there is room. Reports not
   * read this tick are not lost - the sensor queues them. */
  static const uint32_t UPDATE_BUDGET_US = 4000;

  // 10 Hz for the magnetometer-referenced rotation vector. See enableReports().
  static const uint32_t MAG_REPORT_US = 100000;

  static float yawFromQuat(float w, float x, float y, float z) {
    float sqw = w * w, sqx = x * x, sqy = y * y, sqz = z * z;
    float yaw = atan2f(2.0f * (x * y + z * w), (sqx - sqy - sqz + sqw));
    return fmodf(yaw * 57.29578f + 360.0f, 360.0f);
  }

  void setQuat(float w, float x, float y, float z) {
    qw_ = w; qx_ = x; qy_ = y; qz_ = z;

    float sqw = w * w, sqx = x * x, sqy = y * y, sqz = z * z;
    float yaw   = atan2f(2.0f * (x * y + z * w), (sqx - sqy - sqz + sqw));
    float pitch = asinf(constrain(-2.0f * (x * z - y * w) / (sqx + sqy + sqz + sqw), -1.0f, 1.0f));
    float roll  = atan2f(2.0f * (y * z + x * w), (-sqx - sqy + sqz + sqw));

    yaw_deg_   = fmodf(yaw * 57.29578f + 360.0f, 360.0f);
    pitch_deg_ = pitch * 57.29578f;
    roll_deg_  = roll * 57.29578f;
  }

  Adafruit_BNO08x   bno_;
  sh2_SensorValue_t evt_;
  TwoWire *wire_ = nullptr;
  uint8_t  addr_ = 0x4A;
  Mode     mode_ = MODE_BOTH;
  bool     use_mag_ = false;
  float    mag_yaw_deg_ = 0.0f;
  uint32_t mag_yaw_ms_ = 0;
  uint32_t interval_us_ = 10000;
  uint32_t stale_ms_ = 500;
  uint32_t last_event_ms_ = 0;
  bool     begun_ = false;      // begin() succeeded; never cleared after
  bool     calibrating_ = false;
  int      cal_rc_ = 0;         // sh2 return code of the last calibration command
  uint16_t resets_ = 0;
  uint16_t retries_ = 0;        // how many times retryBegin() has tried
  bool     ok_ = false;

  // Per-report accounting. See rvReports() for why it exists.
  uint32_t n_rv_ = 0, n_game_ = 0, n_gyro_ = 0, n_acc_ = 0;
  uint8_t  reports_asked_ = 0, reports_on_ = 0;
  uint8_t  pending_enables_ = 0;   // reports still to re-ask for, one per tick
  uint32_t budget_hits_ = 0;       // ticks that ran out of time mid-drain
  uint32_t rate_since_ms_ = 0;

  float qw_ = 1.0f, qx_ = 0.0f, qy_ = 0.0f, qz_ = 0.0f;
  float yaw_deg_ = 0.0f, pitch_deg_ = 0.0f, roll_deg_ = 0.0f;
  float gyro_x_ = 0.0f, gyro_y_ = 0.0f, gyro_z_ = 0.0f;
  float acc_x_ = 0.0f, acc_y_ = 0.0f, acc_z_ = 0.0f;
  uint8_t accuracy_ = 0;
  float   heading_acc_rad_ = 0.0f;   // the sensor's own heading uncertainty
};

#endif // IMU_BNO085_H

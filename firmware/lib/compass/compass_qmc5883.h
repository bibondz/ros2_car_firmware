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
 * @file compass_qmc5883.h
 * @brief QMC5883L magnetometer (the one built into the GEP-M10-DQ GPS module).
 *
 * WHY IT IS HERE
 *   The GPS course over ground only exists while the robot is moving, and the
 *   BNO085 game rotation vector is relative. Without a magnetometer the robot
 *   does not know which way it is pointing until it has driven a few metres.
 *   This chip gives an ABSOLUTE heading while standing still, which is what
 *   the estimator uses to seed itself before the first move.
 *
 * WHY THIS ONE AND NOT THE BNO085 MAGNETOMETER
 *   It sits on the GPS antenna mast, far from the motors and the battery
 *   wires. The BNO085 lives next to both. You can still switch to the BNO085
 *   rotation vector with heading_source = 0.
 *
 * TILT COMPENSATION
 *   A flat compass reading is only correct when the robot is level. Roll and
 *   pitch from the BNO085 are used to project the field onto the horizontal
 *   plane, so a slope or a bumpy lawn does not swing the heading.
 *
 * INTERFERENCE CHECK
 *   The field strength is compared with the value learned during calibration.
 *   If it changes by more than the tolerance (motor current, a steel fence,
 *   a phone on the chassis) the reading is marked invalid instead of lying.
 *
 * No dynamic allocation, one I2C burst read per update.
 */
#ifndef COMPASS_QMC5883_H
#define COMPASS_QMC5883_H

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

class CompassQMC5883 {
public:
  static const uint8_t REG_DATA    = 0x00;   // X L/H, Y L/H, Z L/H
  static const uint8_t REG_STATUS  = 0x06;
  static const uint8_t REG_CTRL1   = 0x09;
  static const uint8_t REG_CTRL2   = 0x0A;
  static const uint8_t REG_SETRESET = 0x0B;
  static const uint8_t REG_CHIP_ID = 0x0D;   // reads 0xFF on a QMC5883L

  struct Calibration {
    float off_x = 0.0f, off_y = 0.0f, off_z = 0.0f;   // hard iron [counts]
    float scale_x = 1.0f, scale_y = 1.0f, scale_z = 1.0f;  // soft iron
    float field_norm = 0.0f;                          // expected |B| after correction
    bool  valid = false;
  };

  bool begin(TwoWire *wire, uint8_t addr = 0x0D) {
    wire_ = wire;
    addr_ = addr;

    uint8_t id = 0;
    if (!readReg(REG_CHIP_ID, id) || id != 0xFF) { present_ = false; return false; }

    writeReg(REG_CTRL2, 0x80);        // soft reset
    delay(10);
    writeReg(REG_SETRESET, 0x01);     // recommended by the datasheet
    // OSR 512 | range 8 G | ODR 100 Hz | continuous
    writeReg(REG_CTRL1, (0x00 << 6) | (0x01 << 4) | (0x02 << 2) | 0x01);
    delay(10);

    present_ = true;
    return true;
  }

  void setCalibration(const Calibration &cal) { cal_ = cal; }
  const Calibration &calibration() const { return cal_; }

  /** Mounting: angle added after the maths, and direction flip if the module is upside down. */
  void setMounting(float offset_deg, bool invert) { mount_offset_ = offset_deg; invert_ = invert; }
  void setDeclination(float deg) { declination_ = deg; }
  void setFieldTolerance(float fraction) { field_tol_ = fraction; }

  /**
   * Read the sensor and compute the tilt compensated heading.
   * @param roll_deg  roll from the IMU
   * @param pitch_deg pitch from the IMU
   * @return true when a fresh, trustworthy sample was produced
   */
  bool update(float roll_deg, float pitch_deg) {
    if (!present_) return false;

    int16_t rx, ry, rz;
    if (!readRaw(rx, ry, rz)) { fail(); return false; }

    raw_x_ = rx; raw_y_ = ry; raw_z_ = rz;
    if (calibrating_) captureMinMax(rx, ry, rz);

    const float mx = ((float)rx - cal_.off_x) * cal_.scale_x;
    const float my = ((float)ry - cal_.off_y) * cal_.scale_y;
    const float mz = ((float)rz - cal_.off_z) * cal_.scale_z;

    field_ = sqrtf(mx * mx + my * my + mz * mz);
    if (cal_.valid && cal_.field_norm > 1.0f) {
      const float ratio = field_ / cal_.field_norm;
      if (ratio < (1.0f - field_tol_) || ratio > (1.0f + field_tol_)) {
        disturbed_ = true;
        ok_ = false;
        return false;              // magnetic disturbance: better no reading than a wrong one
      }
    }
    disturbed_ = false;

    const float roll  = roll_deg  * 0.01745329f;
    const float pitch = pitch_deg * 0.01745329f;
    const float sr = sinf(roll),  cr = cosf(roll);
    const float sp = sinf(pitch), cp = cosf(pitch);

    const float xh = mx * cp + my * sr * sp + mz * cr * sp;
    const float yh = my * cr - mz * sr;

    float deg = atan2f(-yh, xh) * 57.29578f;          // compass sense: 0 = north, CW
    if (invert_) deg = -deg;
    deg += declination_ + mount_offset_;
    heading_deg_ = wrap(deg);

    last_ok_ms_ = millis();
    ok_ = true;
    errors_ = 0;
    return true;
  }

  bool  present() const { return present_; }
  bool  ok() const { return ok_ && (millis() - last_ok_ms_) < 500; }
  bool  disturbed() const { return disturbed_; }
  float headingDeg() const { return heading_deg_; }   // compass degrees, tilt compensated
  float field() const { return field_; }
  int16_t rawX() const { return raw_x_; }
  int16_t rawY() const { return raw_y_; }
  int16_t rawZ() const { return raw_z_; }

  //--------------------------- calibration ---------------------------------//
  /** Start capturing min/max. Turn the robot slowly through at least one full circle. */
  void startCalibration() {
    calibrating_ = true;
    min_x_ = min_y_ = min_z_ = 32767;
    max_x_ = max_y_ = max_z_ = -32768;
    samples_ = 0;
    n_pts_ = 0;
    gap_ = 1;
    turned_deg_ = 0.0f;
    turn_known_ = false;
    last_yaw_deg_ = 0.0f;
  }

  bool calibrating() const { return calibrating_; }

  /** Abandon a sweep without saving anything.
   *
   * Finish either stores a calibration or refuses; neither is what you want
   * when you simply started it by accident, or ran out of room to turn. Without
   * this the only ways out were to press Finish - which risks storing something
   * - or to leave the robot capturing for ever. */
  void cancelCalibration() {
    calibrating_ = false;
    samples_ = 0;
    n_pts_ = 0;
    gap_ = 1;
    turned_deg_ = 0.0f;
    turn_known_ = false;
  }

  /**
   * Tell the calibration how far the robot has actually turned, from the IMU's
   * own fused heading.
   *
   * PREFER THIS TO addTurnRate(). The BNO085 already integrates its gyro and
   * accelerometer into a rotation vector, and it does that better than we can
   * from rate samples: it runs at the sensor's own rate rather than the control
   * loop's, it corrects the gyro's zero drift against gravity, and it loses
   * nothing to a dropped or late sample. Integrating yawRateDps() by hand takes
   * a signal the sensor has already integrated properly, throws that work away,
   * and re-does it worse - every missed cycle is turn that silently never
   * happened.
   *
   * Fed an absolute heading; the wrapped difference between calls is the turn,
   * so 359 to 1 degrees counts as +2 and not -358.
   *
   * @param yaw_deg the IMU's fused heading, any 360-degree convention
   */
  void addTurnYaw(float yaw_deg) {
    if (!calibrating_) return;
    if (!turn_known_) {                    // first sample only sets the origin
      last_yaw_deg_ = yaw_deg;
      turn_known_ = true;
      return;
    }
    float d = yaw_deg - last_yaw_deg_;
    while (d > 180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    last_yaw_deg_ = yaw_deg;
    turned_deg_ += d;
  }

  /**
   * FALLBACK: turn measured by integrating a raw gyro rate.
   *
   * Only for a board whose IMU offers a rate and no fused orientation. Where
   * there is a fused heading, use addTurnYaw() instead - the sensor integrates
   * better than this loop can, and this one loses turn on every late cycle.
   *
   * This is the honest measurement of the thing the operator is being asked to
   * do. Everything else here infers the turn from the magnetometer's own data,
   * which is circular reasoning: the field readings are what is being
   * calibrated, so using their shape to decide whether the sweep was good means
   * a distorted field can both cause a bad calibration and vouch for it.
   *
   * The gyro is independent of all that. It does not care about iron, motors or
   * hard-iron offset - it measures rotation directly.
   *
   * Accumulated SIGNED, so a wobble back and forth cancels out and only real
   * progress round the circle counts. Turning one way then the other cannot
   * fake a full sweep.
   *
   * @param yaw_rate_dps rotation rate about the vertical axis [deg/s]
   * @param dt           seconds since the last call
   */
  void addTurnRate(float yaw_rate_dps, float dt) {
    if (!calibrating_ || dt <= 0.0f || dt > 1.0f) return;
    turned_deg_ += yaw_rate_dps * dt;
    turn_known_ = true;
  }

  /** Degrees turned since the sweep started, signed. Zero when no gyro fed it. */
  float turnedDeg() const { return turned_deg_; }
  bool  turnKnown() const { return turn_known_; }
  uint16_t calibrationSamples() const { return samples_; }

  /**
   * How much of the circle has actually been swept, 0.0 to 1.0.
   *
   * WHY SAMPLE COUNT WILL NOT DO
   *
   * Standing still with the sensor running piles up thousands of samples and
   * calibrates nothing. That is the failure people hit: press start, nudge the
   * robot, press finish. Any count-based progress bar fills up while the robot
   * has barely moved.
   *
   * WHY THE OBVIOUS FIX DOES NOT WORK EITHER
   *
   * The natural measure is "which of sixteen 22.5-degree sectors has the field
   * vector visited", about the centre of the sweep. But the centre is not known
   * until the sweep is over - it IS the thing being measured - and a running
   * estimate from min/max only settles once all four extremes have been seen,
   * which is three quarters of the way round. Marking sectors as they arrive
   * therefore mislabels the first three quarters of the turn, and a genuine
   * full circle came out as 0.81 rather than 1.0.
   *
   * WHAT THIS DOES
   *
   * Keeps a decimated trail of the sweep - at most CALIB_POINTS samples, spaced
   * out and halved whenever it fills, so a long slow turn costs the same as a
   * fast one - and buckets the WHOLE trail against the centre as it stands now.
   * Every reading is then classified with the best centre available, including
   * the ones taken before that centre was known.
   *
   * The sector count is multiplied by how square the swept box is. A short arc
   * has points spread around its own small bounding box and would otherwise
   * report half the circle covered; its box is a sliver, so the aspect ratio
   * pulls it back down. Measured over arcs, offsets, radii and noise: nothing
   * at 225 degrees or less reaches 0.75, and nothing at 300 or more falls below
   * it. That is the gate finishCalibration() uses.
   */
  float calibrationCoverage() const {
    if (n_pts_ < 12) return 0.0f;
    const float sx = (float)(max_x_ - min_x_);
    const float sy = (float)(max_y_ - min_y_);
    // Same floor finishCalibration() applies, so the bar and the gate never
    // disagree: a full-looking bar that is then refused is worse than no bar.
    if (sx < 200.0f || sy < 200.0f) return 0.0f;

    const float cx = (float)(max_x_ + min_x_) * 0.5f;
    const float cy = (float)(max_y_ + min_y_) * 0.5f;
    uint16_t mask = 0;
    float r_min = 1e9f, r_max = 0.0f;
    for (uint8_t i = 0; i < n_pts_; ++i) {
      const float dx = (float)pt_x_[i] - cx;
      const float dy = (float)pt_y_[i] - cy;
      float ang = atan2f(dy, dx) * 57.29578f;
      if (ang < 0.0f) ang += 360.0f;
      const uint8_t sector =
          (uint8_t)(ang / (360.0f / CALIB_SECTORS)) % CALIB_SECTORS;
      mask |= (uint16_t)(1u << sector);

      const float d = sqrtf(dx * dx + dy * dy);
      if (d < r_min) r_min = d;
      if (d > r_max) r_max = d;
    }
    uint8_t visited = 0;
    for (uint8_t i = 0; i < CALIB_SECTORS; ++i)
      if (mask & (uint16_t)(1u << i)) visited++;

    // Two sanity factors, both there to stop a small movement reading as a turn.
    //
    // ASPECT: how square the swept box is. A short arc has its points spread
    // around its own sliver of a box and would otherwise claim half the circle.
    //
    // ROUNDNESS: how close the trail is to a circle rather than a fat line. A
    // brief nudge with sensor noise on top produces a short, noisy segment whose
    // box can come out square by luck, and whose points then do scatter through
    // many sectors. On a real sweep every point sits at nearly the same distance
    // from the centre; on a segment the middle points sit almost on top of it.
    // Applied with a dead band so a genuine turn over uneven ground - where tilt
    // makes the horizontal projection breathe by 10% or so - is not punished for
    // it, only a trail that is nothing like a circle.
    const float aspect = (sx < sy) ? (sx / sy) : (sy / sx);
    const float roundness = (r_max > 0.0f) ? (2.0f * r_min) / (r_min + r_max) : 0.0f;
    const float shape = (roundness >= ROUND_FULL) ? 1.0f : (roundness / ROUND_FULL);
    const float field = ((float)visited / (float)CALIB_SECTORS) * aspect * shape;

    // With a gyro, take the SMALLER of what the field says and what the
    // rotation says. They answer different questions and both have to be
    // satisfied:
    //
    //   the gyro   proves the robot really went round - it is independent of
    //              the magnetometer, so a distorted field cannot vouch for
    //              itself, which is the weakness of judging the sweep purely
    //              from the data being calibrated;
    //   the field  proves the readings themselves swept a clean circle, which
    //              a gyro cannot see. Turning a full turn next to a steel bench
    //              gives 360 degrees of rotation and a badly deformed circle.
    //
    // Without a gyro this falls back to the field measure alone, which is what
    // it always was - a board with no IMU still calibrates, just less strictly.
    if (!turn_known_) return field;
    const float turned = (turned_deg_ < 0.0f) ? -turned_deg_ : turned_deg_;
    const float by_gyro = (turned >= 360.0f) ? 1.0f : (turned / 360.0f);
    return (by_gyro < field) ? by_gyro : field;
  }

  /** Forget the stored calibration and go back to raw readings.
   *
   * A saved calibration that is wrong is worse than none: the disturbance check
   * only runs once calibrated, so a bad field_norm rejects every reading and
   * the compass goes silently dead. This is the way back out. */
  void clearCalibration() {
    cal_ = Calibration();
    calibrating_ = false;
    samples_ = 0;
    n_pts_ = 0;
    gap_ = 1;
    turned_deg_ = 0.0f;
    turn_known_ = false;
  }

  bool calibrated() const { return cal_.valid; }

  /**
   * Feed one raw sample straight into the calibration.
   *
   * update() does this itself from the I2C read. This exists so the maths can
   * be exercised without a sensor: the calibration is the one part of this
   * driver that can silently produce a result which then rejects every later
   * reading, so it is the part most worth testing, and it was previously
   * reachable only through hardware.
   */
  void addCalibrationSample(int16_t x, int16_t y, int16_t z) {
    if (calibrating_) captureMinMax(x, y, z);
  }

  /** Stop capturing and apply the result. Returns false if the turn was too small. */
  bool finishCalibration() {
    calibrating_ = false;
    if (samples_ < 100) return false;

    const float span_x = (float)(max_x_ - min_x_);
    const float span_y = (float)(max_y_ - min_y_);
    const float span_z = (float)(max_z_ - min_z_);
    if (span_x < 200.0f || span_y < 200.0f) return false;   // barely turned

    // A wide span is not the same as a full turn. Sweeping a half circle
    // produces the full span in one axis and a centre that is off by the
    // hard-iron offset it was supposed to measure. Require most of the circle.
    if (calibrationCoverage() < MIN_COVERAGE) return false;

    Calibration cal;
    cal.off_x = (max_x_ + min_x_) * 0.5f;
    cal.off_y = (max_y_ + min_y_) * 0.5f;
    cal.off_z = (max_z_ + min_z_) * 0.5f;

    // THE Z AXIS IS NOT SWEPT BY THE CALIBRATION WE ASK PEOPLE TO DO.
    //
    // The instructions - and the web UI - say to turn the robot through a full
    // circle. That is a rotation about the vertical axis, so it sweeps X and Y
    // and barely moves Z. Averaging all three spans therefore mixed a real
    // measurement with an unswept axis, and did two harmful things at once:
    // field_norm came out far too small, and scale_z came out enormous because
    // it divided by a span that was only noise.
    //
    // Both errors then fed the disturbance check, which is only active once
    // calibrated. Measured on this robot: reference 1057.7 against a real field
    // of 1962, a ratio of 1.86 against a tolerance of 0.35 - so EVERY reading
    // was rejected and a working compass went completely dead the moment a
    // calibration was saved. A bad calibration was worse than none at all.
    //
    // So: normalise on the horizontal plane, which is what a yaw turn actually
    // measures, and leave an unswept Z alone rather than scaling it by noise.
    const float horiz = (span_x + span_y) * 0.5f;
    cal.scale_x = (span_x > 1.0f) ? horiz / span_x : 1.0f;
    cal.scale_y = (span_y > 1.0f) ? horiz / span_y : 1.0f;
    // Only trust Z if it was genuinely swept - a tilt or a figure-of-eight.
    // Otherwise 1.0 leaves it uncorrected, which is honest, where a scale
    // derived from noise is not.
    const bool z_swept = span_z > (horiz * 0.25f);
    cal.scale_z = (z_swept && span_z > 1.0f) ? horiz / span_z : 1.0f;
    // Half the horizontal span is the field magnitude a level robot sees.
    cal.field_norm = horiz * 0.5f;
    cal.valid = true;

    cal_ = cal;
    return true;
  }

private:
  void captureMinMax(int16_t x, int16_t y, int16_t z) {
    if (x < min_x_) min_x_ = x;
    if (x > max_x_) max_x_ = x;
    if (y < min_y_) min_y_ = y;
    if (y > max_y_) max_y_ = y;
    if (z < min_z_) min_z_ = z;
    if (z > max_z_) max_z_ = z;
    if (samples_ < 65535) samples_++;

    // Keep a decimated trail of the sweep so calibrationCoverage() can classify
    // the whole turn against the centre as it finally comes out, rather than
    // against the poor estimate that existed when each sample arrived.
    //
    // Bounded two ways: a sample is only kept when it is at least gap_ counts
    // from the last kept one, and when the trail fills, every second point is
    // dropped and the spacing doubles. So a slow turn and a fast one cost the
    // same, and the trail stays spread over whatever has been swept so far.
    const int32_t dx = (int32_t)x - (int32_t)pt_x_[n_pts_ ? n_pts_ - 1 : 0];
    const int32_t dy = (int32_t)y - (int32_t)pt_y_[n_pts_ ? n_pts_ - 1 : 0];
    const int32_t step = (dx < 0 ? -dx : dx) > (dy < 0 ? -dy : dy)
                             ? (dx < 0 ? -dx : dx) : (dy < 0 ? -dy : dy);
    if (n_pts_ != 0 && step < (int32_t)gap_) return;

    pt_x_[n_pts_] = x;
    pt_y_[n_pts_] = y;
    n_pts_++;
    if (n_pts_ >= CALIB_POINTS) {
      uint8_t kept = 0;
      for (uint8_t i = 0; i < n_pts_; i += 2) {
        pt_x_[kept] = pt_x_[i];
        pt_y_[kept] = pt_y_[i];
        kept++;
      }
      n_pts_ = kept;
      // Re-derive the spacing from what actually survived, rather than simply
      // doubling it. Doubling from 1 lags far behind the real sample spacing, so
      // the trail refills almost immediately and the OLDEST part gets halved
      // over and over while the newest is untouched. Measured: a full turn kept
      // samples 0, 64, 112, 144, then every 4th - two whole sectors of the start
      // of the turn had no point left in them, and a complete circle reported
      // 0.88. Setting the gap to the smallest step that survived makes the next
      // fill take twice as long and keeps the trail evenly spread.
      uint16_t smallest = 0xFFFF;
      for (uint8_t i = 1; i < n_pts_; ++i) {
        const int32_t sx = (int32_t)pt_x_[i] - (int32_t)pt_x_[i - 1];
        const int32_t sy = (int32_t)pt_y_[i] - (int32_t)pt_y_[i - 1];
        const int32_t s = (sx < 0 ? -sx : sx) > (sy < 0 ? -sy : sy)
                              ? (sx < 0 ? -sx : sx) : (sy < 0 ? -sy : sy);
        if (s < (int32_t)smallest) smallest = (uint16_t)s;
      }
      const uint16_t doubled = (gap_ < 16384) ? (uint16_t)(gap_ * 2) : gap_;
      gap_ = (smallest != 0xFFFF && smallest > doubled) ? smallest : doubled;
    }
  }

  static float wrap(float a) { float x = fmodf(a, 360.0f); return x < 0.0f ? x + 360.0f : x; }

  void fail() { if (++errors_ > 10) { ok_ = false; } }

  bool writeReg(uint8_t reg, uint8_t value) {
    wire_->beginTransmission(addr_);
    wire_->write(reg);
    wire_->write(value);
    return wire_->endTransmission() == 0;
  }

  bool readReg(uint8_t reg, uint8_t &out) {
    wire_->beginTransmission(addr_);
    wire_->write(reg);
    if (wire_->endTransmission(false) != 0) return false;
    if (wire_->requestFrom((int)addr_, 1) != 1) return false;
    out = wire_->read();
    return true;
  }

  bool readRaw(int16_t &x, int16_t &y, int16_t &z) {
    wire_->beginTransmission(addr_);
    wire_->write(REG_DATA);
    if (wire_->endTransmission(false) != 0) return false;
    if (wire_->requestFrom((int)addr_, 6) != 6) return false;
    uint8_t b[6];
    for (uint8_t i = 0; i < 6; ++i) b[i] = wire_->read();
    x = (int16_t)((uint16_t)b[1] << 8 | b[0]);        // little endian
    y = (int16_t)((uint16_t)b[3] << 8 | b[2]);
    z = (int16_t)((uint16_t)b[5] << 8 | b[4]);
    return !(x == 0 && y == 0 && z == 0);
  }

  TwoWire *wire_ = nullptr;
  uint8_t  addr_ = 0x0D;
  bool     present_ = false;
  bool     ok_ = false;
  bool     disturbed_ = false;
  uint8_t  errors_ = 0;
  uint32_t last_ok_ms_ = 0;

  Calibration cal_;
  float declination_  = 0.0f;
  float mount_offset_ = 0.0f;
  bool  invert_ = false;
  float field_tol_ = 0.35f;

  float heading_deg_ = 0.0f;
  float field_ = 0.0f;
  int16_t raw_x_ = 0, raw_y_ = 0, raw_z_ = 0;

  static const uint8_t CALIB_SECTORS = 16;   // 22.5 degrees each
  static const uint8_t CALIB_POINTS = 96;    // decimated sweep trail
  // 12 of 16 sectors. Not 16/16: the last sector or two are easy to miss on a
  // hand-turned robot, and refusing a good sweep for that would push people
  // back to not calibrating at all.
  static constexpr float MIN_COVERAGE = 0.75f;
  // Radial consistency at or above this counts as a circle with no penalty.
  // Below it the score is scaled down in proportion.
  static constexpr float ROUND_FULL = 0.8f;

  bool     calibrating_ = false;
  int16_t  min_x_ = 32767, min_y_ = 32767, min_z_ = 32767;
  int16_t  max_x_ = -32768, max_y_ = -32768, max_z_ = -32768;
  uint16_t samples_ = 0;

  // The decimated sweep trail. 96 points is 384 bytes and enough for 16 sectors
  // to be resolved several times over; the decimation keeps it bounded however
  // long the turn takes.
  int16_t  pt_x_[CALIB_POINTS];
  int16_t  pt_y_[CALIB_POINTS];
  uint8_t  n_pts_ = 0;
  uint16_t gap_ = 1;          // minimum spacing between kept points [counts]
  float    turned_deg_ = 0.0f;   // signed rotation since the sweep started
  bool     turn_known_ = false;  // the IMU has been feeding the turn
  float    last_yaw_deg_ = 0.0f; // previous fused heading, for the wrapped delta
};

#endif // COMPASS_QMC5883_H

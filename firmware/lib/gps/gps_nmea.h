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
 * @file gps_nmea.h
 * @brief NMEA 0183 reader for the GEP-M10-DQ (u-blox M10050).
 *
 * Parses GGA (fix quality / satellites / HDOP / altitude), RMC (position,
 * ground speed, course, date-time) and VTG (ground speed, course).
 *
 * Memory notes:
 *  - one fixed 96 byte line buffer, no String, no heap, no printf
 *  - the parser walks the sentence in place, it never copies fields
 *  - update() is bounded: it consumes at most GPS_MAX_BYTES_PER_CALL bytes per
 *    call so one noisy UART burst can never stall the 100 Hz control loop
 */
#ifndef GPS_NMEA_H
#define GPS_NMEA_H

#include <Arduino.h>

#define GPS_LINE_MAX             96
#define GPS_MAX_BYTES_PER_CALL   160

class GpsNmea {
public:
  GpsNmea() {}

  /** Open the port. If auto_baud is true the candidate list is probed until valid NMEA appears. */
  bool begin(HardwareSerial *serial, int rx_pin, int tx_pin,
             uint32_t baud, bool auto_baud = true,
             const uint32_t *candidates = nullptr, uint8_t n_candidates = 0,
             uint32_t probe_ms = 1200) {
    serial_ = serial;
    rx_pin_ = rx_pin;
    tx_pin_ = tx_pin;
    baud_   = baud;

    openPort(baud_);
    if (!auto_baud || candidates == nullptr || n_candidates == 0) return true;

    for (uint8_t i = 0; i < n_candidates; ++i) {
      if (i > 0 || candidates[0] != baud_) {
        serial_->end();
        openPort(candidates[i]);
      }
      baud_ = candidates[i];
      // Record what each baud actually produced. A receiver at the wrong baud
      // still yields a trickle of framing garbage, so "some bytes" is not
      // success - but the baud with FAR more bytes than the others is the real
      // one, and that is the only way to find it without a scope.
      const uint32_t before = bytes_seen_;
      if (probe_n_ < PROBE_MAX) probe_baud_[probe_n_] = candidates[i];
      uint32_t t0 = millis();
      bool ok = false;
      while (millis() - t0 < probe_ms) {
        if (update() || sentences_ok_ > 0) { ok = true; break; }
        delay(2);
      }
      if (probe_n_ < PROBE_MAX) {
        probe_bytes_[probe_n_] = bytes_seen_ - before;
        ++probe_n_;
      }
      if (ok) return true;
    }
    return sentences_ok_ > 0;
  }

  //--------------------------- probe report ---------------------------//
  /** How many candidate bauds were tried. */
  uint8_t probeCount() const { return probe_n_; }
  uint32_t probeBaud(uint8_t i)  const { return i < probe_n_ ? probe_baud_[i] : 0; }
  uint32_t probeBytes(uint8_t i) const { return i < probe_n_ ? probe_bytes_[i] : 0; }

  /** Open the UART with a buffer big enough for a burst of NMEA.
   *
   * setRxBufferSize() MUST come before begin(). The ESP32 core refuses to
   * resize a running port and reports it only as an error log line - the port
   * then works perfectly well at the default 256 bytes, so nothing looks wrong.
   * Seen on hardware as "RX Buffer can't be resized when Serial is already
   * running" on every boot.
   */
  void openPort(uint32_t baud) {
    if (!serial_) return;
    serial_->setRxBufferSize(512);
    serial_->begin(baud, SERIAL_8N1, rx_pin_, tx_pin_);
  }

  /** Feed the parser. Returns true when a sentence updated the solution. */
  bool update() {
    bool updated = false;
    
    if (!serial_) return false;
    int avail = serial_->available();
    if (avail <= 0) return false;
    
    // Process up to GPS_MAX_BYTES_PER_CALL bytes per control loop.
    if (avail > GPS_MAX_BYTES_PER_CALL) avail = GPS_MAX_BYTES_PER_CALL;
    
    uint8_t chunk[128];
    if (avail > (int)sizeof(chunk)) avail = sizeof(chunk);
    
    // HardwareSerial::read(uint8_t*, size_t) fetches from the IDF ring buffer
    // without blocking for timeouts since we only ask for available bytes.
    size_t read_bytes = serial_->read(chunk, avail);
    
    for (size_t i = 0; i < read_bytes; i++) {
      char c = (char)chunk[i];
      ++bytes_seen_;
      if (c == '$') { len_ = 0; buf_[len_++] = c; capture_ = true; continue; }
      if (!capture_) continue;
      if (c == '\r' || c == '\n') {
        if (len_ > 6) {
          buf_[len_] = '\0';
          if (parseSentence()) updated = true;
        }
        len_ = 0;
        capture_ = false;
        continue;
      }
      if (len_ < GPS_LINE_MAX - 1) buf_[len_++] = c;
      else { len_ = 0; capture_ = false; }   // overlong line -> drop it
    }
    
    return updated;
  }

  //--------------------------- solution ---------------------------//
  bool     valid()      const { return fix_quality_ > 0 && has_position_; }
  bool     fresh(uint32_t stale_ms) const {
    return valid() && (millis() - last_fix_ms_) < stale_ms;
  }
  double   latitude()   const { return lat_; }          // [deg] +N
  double   longitude()  const { return lon_; }          // [deg] +E
  float    altitude()   const { return alt_; }          // [m] MSL
  uint8_t  fixQuality() const { return fix_quality_; }  // 0 none, 1 GPS, 2 DGPS, 4/5 RTK
  uint8_t  satellites() const { return sats_; }
  float    hdop()       const { return hdop_; }
  float    speedMps()   const { return speed_mps_; }    // ground speed
  float    courseDeg()  const { return course_deg_; }   // course over ground, 0..360
  bool     courseValid() const { return course_valid_; }
  uint32_t lastFixMs()  const { return last_fix_ms_; }
  uint32_t baud()       const { return baud_; }
  /** Raw bytes read at any baud. Zero means nothing is transmitting. */
  uint32_t bytesSeen()  const { return bytes_seen_; }
  uint32_t sentencesOk()   const { return sentences_ok_; }
  uint32_t sentencesBad()  const { return sentences_bad_; }

private:
  //--------------------------- helpers ----------------------------//
  static float toFloat(const char *s) { return (s && *s) ? atof(s) : 0.0f; }

  static double nmeaToDegrees(const char *value, const char *hemi) {
    if (!value || !*value) return 0.0;
    double raw = atof(value);
    int deg = (int)(raw / 100.0);
    double minutes = raw - (deg * 100.0);
    double out = deg + minutes / 60.0;
    if (hemi && (*hemi == 'S' || *hemi == 'W')) out = -out;
    return out;
  }

  /** Split the buffer in place; fields_[] points into buf_. Returns field count. */
  uint8_t split() {
    uint8_t n = 0;
    fields_[n++] = buf_;
    for (uint16_t i = 0; i < len_ && n < GPS_MAX_FIELDS; ++i) {
      if (buf_[i] == ',') { buf_[i] = '\0'; fields_[n++] = &buf_[i + 1]; }
      else if (buf_[i] == '*') { buf_[i] = '\0'; break; }
    }
    return n;
  }

  bool checksumOk() {
    // buf_ = "$GNRMC,...*4F"
    int star = -1;
    for (int i = (int)len_ - 1; i >= 0 && i > (int)len_ - 6; --i) {
      if (buf_[i] == '*') { star = i; break; }
    }
    if (star < 1 || star + 2 >= (int)len_) return false;
    uint8_t sum = 0;
    for (int i = 1; i < star; ++i) sum ^= (uint8_t)buf_[i];
    uint8_t given = (uint8_t)((hexVal(buf_[star + 1]) << 4) | hexVal(buf_[star + 2]));
    return sum == given;
  }

  static uint8_t hexVal(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return 0;
  }

  bool parseSentence() {
    if (!checksumOk()) { sentences_bad_++; return false; }
    sentences_ok_++;

    // talker id is 2 chars (GP/GN/GA/...), message type is the next 3
    const char *type = &buf_[3];
    uint8_t n = split();
    if (n < 5) return false;

    if (!strncmp(type, "GGA", 3)) return parseGGA(n);
    if (!strncmp(type, "RMC", 3)) return parseRMC(n);
    if (!strncmp(type, "VTG", 3)) return parseVTG(n);
    return false;
  }

  bool parseGGA(uint8_t n) {
    if (n < 10) return false;
    uint8_t quality = (uint8_t)atoi(fields_[6]);
    fix_quality_ = quality;
    sats_ = (uint8_t)atoi(fields_[7]);
    hdop_ = toFloat(fields_[8]);
    if (quality > 0 && *fields_[2] && *fields_[4]) {
      lat_ = nmeaToDegrees(fields_[2], fields_[3]);
      lon_ = nmeaToDegrees(fields_[4], fields_[5]);
      alt_ = toFloat(fields_[9]);
      has_position_ = true;
      last_fix_ms_ = millis();
      return true;
    }
    if (quality == 0) has_position_ = false;
    return false;
  }

  bool parseRMC(uint8_t n) {
    if (n < 9) return false;
    bool active = (*fields_[2] == 'A');
    if (!active) { has_position_ = false; return false; }

    lat_ = nmeaToDegrees(fields_[3], fields_[4]);
    lon_ = nmeaToDegrees(fields_[5], fields_[6]);
    if (*fields_[7]) speed_mps_ = toFloat(fields_[7]) * 0.514444f;  // knots -> m/s
    if (*fields_[8]) { course_deg_ = toFloat(fields_[8]); course_valid_ = true; }
    if (fix_quality_ == 0) fix_quality_ = 1;
    has_position_ = true;
    last_fix_ms_  = millis();
    return true;
  }

  bool parseVTG(uint8_t n) {
    if (n < 8) return false;
    if (*fields_[1]) { course_deg_ = toFloat(fields_[1]); course_valid_ = true; }
    if (*fields_[7]) speed_mps_ = toFloat(fields_[7]) / 3.6f;       // km/h -> m/s
    return false;   // VTG alone is not a position update
  }

  static const uint8_t GPS_MAX_FIELDS = 20;

  HardwareSerial *serial_ = nullptr;
  int      rx_pin_ = -1, tx_pin_ = -1;
  uint32_t baud_   = 115200;
  uint32_t bytes_seen_ = 0;   // raw bytes off the wire, any baud
  static const uint8_t PROBE_MAX = 8;
  uint32_t probe_baud_[PROBE_MAX]  = {0};
  uint32_t probe_bytes_[PROBE_MAX] = {0};
  uint8_t  probe_n_ = 0;

  char     buf_[GPS_LINE_MAX];
  char    *fields_[GPS_MAX_FIELDS];
  uint16_t len_     = 0;
  bool     capture_ = false;

  double   lat_ = 0.0, lon_ = 0.0;
  float    alt_ = 0.0f;
  uint8_t  fix_quality_ = 0;
  uint8_t  sats_ = 0;
  float    hdop_ = 99.9f;
  float    speed_mps_ = 0.0f;
  float    course_deg_ = 0.0f;
  bool     course_valid_ = false;
  bool     has_position_ = false;
  uint32_t last_fix_ms_ = 0;
  uint32_t sentences_ok_ = 0;
  uint32_t sentences_bad_ = 0;
};

#endif // GPS_NMEA_H

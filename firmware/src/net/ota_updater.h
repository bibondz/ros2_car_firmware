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
 * Firmware update over Wi-Fi, so a board that is bolted into a robot and
 * running on battery can be reflashed without finding a cable.
 *
 * This exists because of a real dead end: the relay was mis-wired in firmware,
 * the fix was one line, and it could not be applied. The robot was on battery
 * with no USB attached, and the running image had no way to accept an update.
 * A cable was the only route, which is exactly the situation OTA removes.
 *
 * WHAT IT WILL NOT DO
 *
 * It refuses to start an update while the robot is MOVING. Rewriting the
 * program while wheels are turning means the motors keep their last duty cycle
 * for the seconds it takes to flash and reboot, with nothing running that could
 * stop them.
 *
 * A robot that is merely powered and standing still is fine: the caller cuts the
 * motor rails itself the moment an update is accepted, so no one has to walk
 * over and press the e-stop first. What is deliberately NOT automatic is cutting
 * power to a machine in motion - that stops being an update and becomes an
 * unannounced e-stop, so a moving robot simply does not answer.
 *
 * The refusal is silent by design: the port is not serviced, the uploader
 * reports no response, and a driving robot is never interrupted.
 *
 * The password is not decoration. Anything on the network can reach port 3232,
 * and an unauthenticated OTA port is a way to put arbitrary code on a machine
 * that drives itself around. It comes from network_secrets.h, which is
 * gitignored, and the build fails if it is missing.
 *
 * ROLLBACK
 *
 * ArduinoOTA writes to the inactive OTA partition and only switches over once
 * the whole image has arrived and its MD5 matches. A partial or corrupt upload
 * leaves the running image untouched, so a dropped Wi-Fi link mid-flash costs
 * nothing but a retry. That is why the update lands as a normal reboot rather
 * than a bricked board.
 */
#ifndef OTA_UPDATER_H
#define OTA_UPDATER_H

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>

#include "config.h"

class OtaUpdater {
 public:
  /**
   * Advertise the board for wireless flashing.
   *
   * @param hostname   what the board answers to, e.g. "esp32-gps-localize"
   * @param is_safe    consulted every loop. Return false whenever the robot
   *                   could move: the OTA port is then simply not serviced, so
   *                   the uploader reports that the device did not respond and
   *                   the robot is not disturbed in any way.
   * @param on_begin   called once an update has been accepted, to shut down
   *                   anything that must not run while flash is being rewritten.
   */
  void begin(const char *hostname, bool (*is_safe)(), void (*on_begin)()) {
    is_safe_  = is_safe;
    on_begin_ = on_begin;

    ArduinoOTA.setHostname(hostname);
    ArduinoOTA.setPassword(OTA_PASSWORD);

    ArduinoOTA.onStart([this]() {
      if (on_begin_) on_begin_();
      updating_ = true;
      Serial.printf("[OTA] update starting (%s)\n",
                    ArduinoOTA.getCommand() == U_FLASH ? "firmware"
                                                       : "filesystem");
    });

    ArduinoOTA.onProgress([](unsigned int done, unsigned int total) {
      static int last_pct = -1;
      const int pct = total ? (int)((done * 100) / total) : 0;
      if (pct != last_pct && pct % 10 == 0) {
        last_pct = pct;
        Serial.printf("[OTA] %d%%\n", pct);
      }
    });

    ArduinoOTA.onEnd([this]() {
      updating_ = false;
      Serial.println("[OTA] image verified, rebooting into it");
    });

    ArduinoOTA.onError([this](ota_error_t error) {
      updating_ = false;
      // The running image is untouched: ArduinoOTA writes the inactive
      // partition and only switches over on a verified image.
      const char *why = "unknown";
      switch (error) {
        case OTA_AUTH_ERROR:    why = "wrong password"; break;
        case OTA_BEGIN_ERROR:   why = "could not begin - no space or bad partition"; break;
        case OTA_CONNECT_ERROR: why = "connection lost"; break;
        case OTA_RECEIVE_ERROR: why = "receive failed"; break;
        case OTA_END_ERROR:     why = "image failed verification"; break;
      }
      Serial.printf("[OTA] failed: %s - the running firmware is unchanged\n", why);
    });

    ArduinoOTA.begin();
    ready_ = true;
    Serial.printf("[OTA] ready - flash over Wi-Fi with:\n"
                  "      pio run -e wifi_ota -t upload   (board: %s / %s)\n",
                  hostname, WiFi.localIP().toString().c_str());
  }

  /**
   * Service the OTA server, but ONLY while an update would be safe to accept.
   *
   * The gate lives here rather than in onStart because by the time onStart runs
   * the transfer has already been accepted, and ArduinoOTA offers no way to
   * decline it. The first version of this refused there and called
   * ESP.restart() to avoid flashing in an unsafe state - which meant anyone
   * running the upload command could reboot a live robot. Rebooting a machine
   * because someone asked it an unwelcome question is not a safety measure.
   *
   * Simply not servicing the port is a clean refusal: no packet is answered,
   * the uploader reports that the device did not respond, and the robot carries
   * on undisturbed.
   */
  void handle() {
    if (!ready_) return;
    if (!updating_ && is_safe_ && !is_safe_()) {
      // Say why, but rarely - this runs every loop.
      static uint32_t last_note_ms = 0;
      const uint32_t now = millis();
      if (now - last_note_ms > 30000) {
        last_note_ms = now;
        Serial.println("[OTA] not accepting updates while the robot is moving. "
                       "Stop it and try again.");
      }
      return;
    }
    ArduinoOTA.handle();
  }

  /** True while an image is being received, so the caller can stay out of the way. */
  bool updating() const { return updating_; }

 private:
  bool (*is_safe_)()  = nullptr;
  void (*on_begin_)() = nullptr;
  bool ready_    = false;
  bool updating_ = false;
};

#endif  // OTA_UPDATER_H

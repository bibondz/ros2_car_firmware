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
 * @file safety_manager.h
 * @brief Collects every stop condition into one bitfield and drives the red LED.
 *
 * The manager never touches the motors itself - main.cpp asks allowMotion()
 * on every control cycle. Keeping the decision in one place means a new stop
 * source only has to be added here.
 *
 * Red LED (IO2)   : solid = emergency latched, fast blink = link/command lost,
 *                   slow blink = battery cutoff, short blip = healthy.
 * Green LED       : wired straight to the 5 V rail after the ON MODULE switch.
 *                   It shows "module powered" and is intentionally NOT driven
 *                   by firmware, so it still lights if the ESP32 hangs.
 */
#ifndef SAFETY_MANAGER_H
#define SAFETY_MANAGER_H

#include <Arduino.h>
#include <config.h>

class SafetyManager {
public:
  struct Inputs {
    bool     sw_estop        = false;   // software e-stop latched by the web UI
    uint32_t heartbeat_age_ms = 0xFFFFFFFF;
    uint32_t cmd_age_ms       = 0xFFFFFFFF;
    bool     agent_ok        = false;
    float    battery_v       = 0.0f;
    bool     battery_valid   = false;
    bool     motion_requested = false;  // a non-zero command is pending
    // What the motor-rail INAs actually MEASURE, so the contactor can be
    // cross-checked against what we told it to do rather than trusted.
    bool     rails_live      = false;   // a motor rail is above PACK_RAIL_MIN_VALID_V
  };

  void begin() {
    // The pull MUST oppose the active level, or an unconnected pin reads as
    // permanently asserted. Both of these were wrong on the bench: the button
    // is active HIGH (ESP_EMER_SW_ACTIVE_LOW 0) but had a pull-UP, so with no
    // button fitted IO13 floated high, read as pressed, and held the board in
    // STOP_ESP_BUTTON forever - state ESTOP from boot, motors never enabled,
    // and nothing in the log to say why. The emergency line had no pull at all,
    // so it was reading "not tripped" only by luck of a floating input.
    //
    // Deriving the pull from the polarity is what stops the two disagreeing
    // again: change the ACTIVE_LOW define and the pull follows it.
#if E_EMER_ACTIVE_LOW
    pinMode(PIN_E_EMER, INPUT_PULLUP);
#else
    pinMode(PIN_E_EMER, INPUT_PULLDOWN);
#endif

    // Optional: this build has no separate on-board button, PIN_ESP_EMER_SW -1.
    if (PIN_ESP_EMER_SW >= 0) {
#if ESP_EMER_SW_ACTIVE_LOW
      pinMode(PIN_ESP_EMER_SW, INPUT_PULLUP);
#else
      pinMode(PIN_ESP_EMER_SW, INPUT_PULLDOWN);
#endif
    }

    // The contactor comes up OPEN and is driven closed only once this class has
    // decided the robot may be powered. Set low BEFORE the pin becomes an
    // output, so configuring it cannot produce a brief closed pulse.
    relay_closed_ = false;
    if (PIN_RELAY_MAIN >= 0) {
      digitalWrite(PIN_RELAY_MAIN, RELAY_MAIN_ACTIVE_LOW ? HIGH : LOW);
      pinMode(PIN_RELAY_MAIN, OUTPUT);
      digitalWrite(PIN_RELAY_MAIN, RELAY_MAIN_ACTIVE_LOW ? HIGH : LOW);
    }
    if (PIN_LED_RED >= 0) { pinMode(PIN_LED_RED, OUTPUT); digitalWrite(PIN_LED_RED, LOW); }
    if (PIN_LED_GREEN >= 0) { pinMode(PIN_LED_GREEN, OUTPUT); digitalWrite(PIN_LED_GREEN, HIGH); }
    if (PIN_FAN_CTRL >= 0) { pinMode(PIN_FAN_CTRL, OUTPUT); digitalWrite(PIN_FAN_CTRL, HIGH); }
    boot_ms_ = millis();
    last_clear_ms_ = 0;

    // A battery cutoff is cleared by starting the robot again and by nothing
    // else, so this is the one place it is released. Stated explicitly rather
    // than left to the member initialiser, because "off and on again" IS the
    // documented recovery and it should be visible in the code that performs it.
    batt_latched_   = false;
    batt_low_since_ = 0;
    batt_soft_stop_ = false;
    batt_soft_ok_since_ = 0;

    // Same rule for the update lockout: a reboot is what ends it, and finishing
    // a flash reboots the board. Cleared here rather than relying on the object
    // being reconstructed, so "restart" means the same thing to both latches.
    update_lockout_ = false;
  }

  void update(const Inputs &in) {
    uint32_t now = millis();
    uint32_t f = 0;

    // --- hardware inputs (debounced) ---
    bool hw_raw  = readEmergencyLine();
    bool btn_raw = readEspButton();
    hw_emer_  = debounce(hw_raw,  hw_state_,  hw_change_ms_,  now);
    esp_btn_  = debounce(btn_raw, btn_state_, btn_change_ms_, now);

    if (hw_emer_)  f |= STOP_HW_EMERGENCY;
    if (esp_btn_)  f |= STOP_ESP_BUTTON;
    if (in.sw_estop) f |= STOP_SW_ESTOP;

    // --- link / command supervision ---
    if (!in.agent_ok)                                    f |= STOP_AGENT_LOST;
    if (in.heartbeat_age_ms > HEARTBEAT_TIMEOUT_MS)      f |= STOP_HEARTBEAT_LOST;
    if (in.motion_requested && in.cmd_age_ms > CMD_TIMEOUT_MS) f |= STOP_CMD_TIMEOUT;

    // --- battery protection (must stay low, ignore short load dips) ---
    //
    // battery_valid false means the pack voltage could not be MEASURED - the
    // motor rail is unpowered because the mushroom switch or the relay is open.
    // That is not a flat battery, so nothing latches. Reading the ESP32 +5 V
    // rail here by mistake is exactly what latched the robot into e-stop three
    // seconds after every boot; see power.h.
    // The hard cut LATCHES UNTIL POWER OFF. It used to clear itself once the
    // pack came back above BATT_CUTOFF_CLEAR_V, and on a LiPo that is the wrong
    // behaviour: a pack at the cutoff sags under load and recovers the moment
    // the load comes off, so the robot would stop, un-latch a second later,
    // drive, sag, stop again - cycling while it drained the cells past the point
    // where they take damage. There is no voltage that means "this pack is fine
    // again"; only a human deciding to swap or charge it does.
    //
    // So the only way out is switching the robot off and starting it again. That
    // is deliberate: it forces someone to look at the battery instead of letting
    // the machine talk itself back into driving.
    if (in.battery_valid && in.battery_v < cutoff_v_) {
      if (batt_low_since_ == 0) batt_low_since_ = now;
      if (now - batt_low_since_ > BATT_CUTOFF_HOLD_MS) batt_latched_ = true;
    } else {
      batt_low_since_ = 0;
    }
    if (batt_latched_) f |= STOP_BATTERY_LOW;


    // Soft stop: stop driving but stay powered, so the pack is not run down to
    // the hard cut on every outing. Not latched - it follows the voltage back
    // up once the load comes off.
    // Entering is immediate; LEAVING needs the voltage clearly back up and
    // steady, or the robot chatters: sag under load, stop, recover, drive, sag.
    if (in.battery_valid && in.battery_v < soft_stop_v_) {
      batt_soft_stop_ = true;
      batt_soft_ok_since_ = 0;
    } else if (batt_soft_stop_) {
      const bool clearly_up = in.battery_valid
                              && in.battery_v > (soft_stop_v_ + BATT_SOFT_HYST_V);
      if (!clearly_up) {
        batt_soft_ok_since_ = 0;
      } else {
        if (batt_soft_ok_since_ == 0) batt_soft_ok_since_ = now ? now : 1;
        if (now - batt_soft_ok_since_ >= BATT_SOFT_RECOVER_MS) {
          batt_soft_stop_ = false;
          batt_soft_ok_since_ = 0;
        }
      }
    }
    if (batt_soft_stop_) f |= STOP_BATTERY_SOFT;

    // --- boot lockout ---
    if (now - boot_ms_ < STARTUP_LOCKOUT_MS) f |= STOP_STARTUP;

    // ---- main contactor -----------------------------------------------------
    //
    // The pack sits BEHIND this relay, so nothing can measure the battery until
    // the relay is closed. That forces the order:
    //
    //   1. hold the rails dead through the start-up lockout
    //   2. close the relay - only now does a pack voltage exist to read
    //   3. read it, and if it is genuinely flat, open the relay and stay open
    //
    // Step 3 is why the cutoff latches until a power cycle. Opening the relay
    // removes the very measurement that justified opening it: the pack reads 0
    // and unmeasurable the moment the contactor drops out. Anything that
    // re-closed on "the voltage looks fine now" would therefore close every
    // time, chatter the contactor, and drain the flat pack it was protecting.
    //
    // A heartbeat or agent loss does NOT open the contactor. Those stop the
    // robot driving through allowMotion(), and cutting rail power on a brief
    // network hiccup would only power-cycle the motor supply and make the
    // battery reading flap.
    // Read the emergency from THIS cycle's flags, not from flags_, which still
    // holds the previous cycle's answer. Using the stale copy delayed opening
    // the contactor by a full control period after the button went down - 10 ms
    // of powered rails that nothing needed to spend.
    const bool emergency_now =
        (f & (STOP_HW_EMERGENCY | STOP_ESP_BUTTON | STOP_SW_ESTOP)) != 0;
    const bool safe_to_power = !(f & STOP_STARTUP) && !emergency_now
                               && !batt_latched_ && !update_lockout_;
    setRelay(safe_to_power);

    // --- contactor cross-check -------------------------------------
    // Everything above trusts what we COMMANDED the contactor to do. This is
    // the one check that asks the rails instead. If we opened it and they are
    // still live past the settle time, the contact is welded and no amount of
    // firmware will remove power - so say so loudly rather than reporting a
    // safely stopped robot.
    //
    // It runs BEFORE the release below, and that ordering is the whole point.
    // It used to run after, and since f is rebuilt from zero every cycle the
    // stuck-contactor bit was never present when allow_ was decided - so the
    // one fault where power cannot be cut was reported and then ignored, and
    // the robot was still allowed to drive.
    if (relay_closed_) {
      relay_open_since_ = 0;
    } else {
      if (relay_open_since_ == 0) relay_open_since_ = now ? now : 1;
      if (in.rails_live && (now - relay_open_since_) >= CONTACTOR_SETTLE_MS) {
        f |= STOP_CONTACTOR_STUCK;
      }
    }

    // --- release needs the inputs to stay clear for a moment ---
    if (f == 0) {
      if (last_clear_ms_ == 0) last_clear_ms_ = now ? now : 1;
      allow_ = (now - last_clear_ms_) >= ESTOP_RELEASE_HOLD_MS;
    } else {
      last_clear_ms_ = 0;
      allow_ = false;
    }
    // An update lockout has dropped the rails on purpose. It carries no stop
    // bit of its own - adding one would change the telemetry contract - but it
    // must still refuse motion, or the loop keeps driving PWM into a driver
    // whose supply has just been taken away.
    if (update_lockout_) allow_ = false;

    flags_ = f;
    serviceLed(now);
  }

  /**
   * Battery tiers in volts, for THIS pack.
   *
   * They used to be compile-time constants sized for a 3S pack, which meant a
   * 2S or 4S battery had every tier wrong at once - and wrong in the dangerous
   * direction for 4S, where a 3S cutoff of 10.7 V is 2.68 V per cell and the
   * pack would be destroyed long before the robot stopped. The parameters
   * existed but nothing read them; these setters are what makes them real.
   */
  void setBatteryThresholds(float warn_v, float soft_stop_v, float cutoff_v) {
    warn_v_      = warn_v;
    soft_stop_v_ = soft_stop_v;
    cutoff_v_    = cutoff_v;
  }
  float warnVolts()     const { return warn_v_; }
  float softStopVolts() const { return soft_stop_v_; }
  float cutoffVolts()   const { return cutoff_v_; }

  uint32_t flags() const { return flags_; }
  bool allowMotion() const { return allow_; }
  bool emergencyLatched() const {
    return (flags_ & (STOP_HW_EMERGENCY | STOP_ESP_BUTTON | STOP_SW_ESTOP)) != 0;
  }
  bool linkLost() const {
    return (flags_ & (STOP_HEARTBEAT_LOST | STOP_CMD_TIMEOUT | STOP_AGENT_LOST)) != 0;
  }
  /** True when the main contactor is closed and the motor rails are live. */
  bool relayClosed() const { return relay_closed_; }

  /**
   * Cut motor power for a firmware update, immediately and for good.
   *
   * Requiring a person to walk over and press the e-stop before every wireless
   * flash defeats the point of flashing wirelessly. The firmware owns the
   * contactor, so it can do that part itself: this drops the rails at once and
   * holds them down, and only a reboot clears it - which is exactly what
   * finishing an update does anyway.
   *
   * The pin is written here rather than left to the next update(), because a
   * flash blocks the main loop: waiting for the next pass would mean the rails
   * were still live while the program was being rewritten.
   *
   * It is still refused while the robot is MOVING. Yanking power from a machine
   * in motion, remotely, on someone else's say-so is not something to do
   * quietly - and the caller checks that before getting here.
   */
  void beginUpdateLockout() {
    update_lockout_ = true;
    setRelay(false);
  }
  bool batteryLatched() const { return batt_latched_; }
  bool batterySoftStop() const { return batt_soft_stop_; }
  bool hwEmergency() const { return hw_emer_; }
  bool espButton() const { return esp_btn_; }

  RunState runState(bool auto_mode, bool manual_mode) const {
    if (emergencyLatched() || batt_latched_ ||
        (flags_ & STOP_CONTACTOR_STUCK)) return RUN_ESTOP;
    if (linkLost())                          return RUN_COMM_LOSS;
    if (auto_mode)                           return RUN_AUTO;
    if (manual_mode)                         return RUN_MANUAL;
    return RUN_IDLE;
  }

private:
  void setRelay(bool close) {
    if (relay_closed_ == close) return;         // only write on a real change
    relay_closed_ = close;
    if (PIN_RELAY_MAIN < 0) return;
    const int level = RELAY_MAIN_ACTIVE_LOW ? (close ? LOW : HIGH)
                                            : (close ? HIGH : LOW);
    digitalWrite(PIN_RELAY_MAIN, level);
  }

  static bool readEmergencyLine() {
    int v = digitalRead(PIN_E_EMER);
#if E_EMER_ACTIVE_LOW
    return v == LOW;
#else
    return v == HIGH;
#endif
  }

  static bool readEspButton() {
    if (PIN_ESP_EMER_SW < 0) return false;      // not fitted: never asserted
    int v = digitalRead(PIN_ESP_EMER_SW);
#if ESP_EMER_SW_ACTIVE_LOW
    return v == LOW;
#else
    return v == HIGH;
#endif
  }

  static bool debounce(bool raw, bool &state, uint32_t &change_ms, uint32_t now) {
    const uint32_t DEBOUNCE_MS = 25;
    if (raw != state) {
      if (change_ms == 0) change_ms = now;
      else if (now - change_ms >= DEBOUNCE_MS) { state = raw; change_ms = 0; }
    } else {
      change_ms = 0;
    }
    return state;
  }

  void serviceLed(uint32_t now) {
    if (PIN_LED_RED < 0) return;
    bool on;
    if (emergencyLatched() || (flags_ & STOP_CONTACTOR_STUCK)) {
      on = true;                                                    // solid
    } else if (linkLost()) {
      on = ((now / LED_BLINK_FAST_MS) & 1) != 0;                    // fast blink
    } else if (batt_latched_) {
      on = ((now / LED_BLINK_SLOW_MS) & 1) != 0;                    // slow blink
    } else {
      on = (now % LED_HEARTBEAT_MS) < 60;                           // healthy blip
    }
    if (on != led_on_) { led_on_ = on; digitalWrite(PIN_LED_RED, on ? HIGH : LOW); }
  }

  uint32_t flags_ = STOP_STARTUP;
  bool     allow_ = false;
  bool     hw_emer_ = false, esp_btn_ = false;
  bool     hw_state_ = false, btn_state_ = false;
  uint32_t hw_change_ms_ = 0, btn_change_ms_ = 0;
  uint32_t boot_ms_ = 0;
  uint32_t last_clear_ms_ = 0;
  uint32_t batt_low_since_ = 0;
  bool     batt_latched_ = false;
  bool     batt_soft_stop_ = false;
  uint32_t batt_soft_ok_since_ = 0;   // when the pack came back above the threshold
  bool     relay_closed_ = false;
  uint32_t relay_open_since_ = 0;     // when we last commanded it open
  // Default to the compiled-in 3S figures so a board is protected from the
  // first millisecond, before any parameter has been applied.
  float    warn_v_      = BATT_WARN_V;
  float    soft_stop_v_ = BATT_SOFT_STOP_V;
  float    cutoff_v_    = BATT_CUTOFF_V;
  bool     update_lockout_ = false;
  bool     led_on_ = false;
};

#endif // SAFETY_MANAGER_H

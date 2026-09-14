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
 * A line protocol on the USB console, so the robot can be configured with no
 * network at all.
 *
 * WHY THIS EXISTS WHEN THERE IS ALREADY A WEB UI
 *
 * The web UI needs Wi-Fi, and Wi-Fi is the thing you most often need to change.
 * A board that knows none of the networks where it currently is cannot be
 * reached over the network to be told about them - so without a wire there is
 * no way in. That is the plug-and-configure case: plug the board into a laptop,
 * set the credentials, unplug.
 *
 * It is also the recovery path when the radio is the broken part. Every command
 * here works with the antenna unplugged.
 *
 * DESIGN NOTES
 *
 * Templated on the stream so the native tests can drive it through FakeSerial
 * and assert on what it replies - the protocol is then covered without a board.
 * Everything is fixed buffers: no String, no heap, and an over-long line is
 * dropped rather than truncated into a half-command that might parse as
 * something else.
 *
 * PASSWORDS GO IN AND NEVER COME OUT. `wifi` lists SSIDs and whether each has a
 * password, never the password itself. A console is a thing people paste into
 * chat windows and bug reports.
 */
#ifndef SETTINGS_CONSOLE_H
#define SETTINGS_CONSOLE_H

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef CONSOLE_LINE_MAX
#define CONSOLE_LINE_MAX 96
#endif

/**
 * Everything the console needs from the rest of the firmware, passed in rather
 * than reached for, so the tests can supply their own.
 */
struct ConsoleHooks {
  /** True while the robot is moving - STOPPED_ONLY parameters are refused. */
  bool (*isMoving)() = nullptr;
  /** Show where the agent is being looked for, and how that was decided. */
  void (*showAgent)(void *ctx) = nullptr;
  /** Point the robot at an agent by name or IP. "" or "auto" clears it. */
  bool (*setAgent)(void *ctx, const char *host) = nullptr;
  /** Report the compass, and forget its calibration. */
  void (*showCompass)(void *ctx) = nullptr;
  void (*clearCompass)(void *ctx) = nullptr;
  /** Print a status block. Free-form; the console only calls it for `info`. */
  void (*printInfo)(void *out) = nullptr;
  void *info_ctx = nullptr;
};

template <typename Stream, typename Registry, typename WifiStoreT>
class SettingsConsole {
public:
  void begin(Stream *s, Registry *params, WifiStoreT *wifi, ConsoleHooks hooks) {
    io_ = s;
    params_ = params;
    wifi_ = wifi;
    hooks_ = hooks;
    len_ = 0;
    overlong_ = false;
  }

  /**
   * Read whatever has arrived and act on any complete line.
   *
   * Bounded per call: the console must never be able to starve the control
   * loop, however fast someone pastes into it.
   */
  void poll(uint16_t budget = 128) {
    if (!io_) return;
    while (budget-- && io_->available()) {
      int c = io_->read();
      if (c < 0) break;
      if (c == '\r') continue;
      if (c == '\n') {
        if (overlong_) {
          reply("error: line too long, ignored");
          overlong_ = false;
        } else if (len_ > 0) {
          line_[len_] = '\0';
          dispatch(line_);
        }
        len_ = 0;
        continue;
      }
      if (len_ < CONSOLE_LINE_MAX - 1) {
        line_[len_++] = (char)c;
      } else {
        // Do not silently truncate: a cut-off "set x 100" could arrive as
        // "set x 10", which is a different and entirely plausible command.
        overlong_ = true;
      }
    }
  }

private:
  //---------------------------------------------------------------- helpers
  void reply(const char *text) {
    if (io_) { io_->print(text); io_->print("\n"); }
  }

  void replyf(const char *fmt, ...) {
    char buf[CONSOLE_LINE_MAX + 64];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    reply(buf);
  }

  /** Split off the next whitespace-delimited token, in place. */
  static char *token(char **cursor) {
    char *p = *cursor;
    while (*p == ' ' || *p == '\t') ++p;
    if (!*p) { *cursor = p; return nullptr; }
    char *start = p;
    while (*p && *p != ' ' && *p != '\t') ++p;
    if (*p) { *p = '\0'; ++p; }
    *cursor = p;
    return start;
  }

  /** The rest of the line, leading blanks trimmed. Used for passwords. */
  static char *rest(char *cursor) {
    while (*cursor == ' ' || *cursor == '\t') ++cursor;
    return cursor;
  }

  bool moving() const { return hooks_.isMoving && hooks_.isMoving(); }

  //--------------------------------------------------------------- dispatch
  void dispatch(char *line) {
    char *cursor = line;
    char *cmd = token(&cursor);
    if (!cmd) return;

    if (!strcmp(cmd, "help") || !strcmp(cmd, "?"))      return cmdHelp();
    if (!strcmp(cmd, "info"))                           return cmdInfo();
    if (!strcmp(cmd, "params") || !strcmp(cmd, "list")) return cmdParams();
    if (!strcmp(cmd, "get"))                            return cmdGet(&cursor);
    if (!strcmp(cmd, "set"))                            return cmdSet(&cursor);
    if (!strcmp(cmd, "reset"))                          return cmdReset(&cursor);
    if (!strcmp(cmd, "wifi"))                           return cmdWifi(&cursor);
    if (!strcmp(cmd, "agent"))                          return cmdAgent(&cursor);
    if (!strcmp(cmd, "compass"))                        return cmdCompass(&cursor);

    replyf("error: unknown command \"%s\" - try help", cmd);
  }

  void cmdHelp() {
    reply("commands:");
    reply("  info                    version, uptime, link and network state");
    reply("  params                  every tunable: value, unit, default");
    reply("  get <key>               one value");
    reply("  set <key> <value>       change one, bounds enforced here");
    reply("  reset <key> | reset all restore the compiled-in default");
    reply("  wifi                    list known networks (never the passwords)");
    reply("  wifi add <ssid> <pass>  add or replace one");
    reply("  wifi del <ssid>         forget one");
    reply("  wifi first <ssid>       try this one first");
    reply("  agent                   where the micro-ROS agent is being looked for");
    reply("  agent <host|ip>         point the robot at one; agent auto = discover");
    reply("  compass                 heading, field strength and calibration state");
    reply("  compass clear           forget the calibration - the way back from a bad one");
    reply("");
    reply("  there is no save command - every change is written to NVS at once");
    reply("ok");
  }

  void cmdInfo() {
    if (hooks_.printInfo) hooks_.printInfo(hooks_.info_ctx);
    else reply("no info available");
    reply("ok");
  }

  void cmdParams() {
    if (!params_) { reply("error: no parameter registry"); return; }
    for (uint8_t i = 0; i < params_->count(); ++i) {
      const auto &d = params_->def(i);
      replyf("%-28s %12.4f %-6s default %.4f%s", d.key, (double)params_->value(i),
             d.unit ? d.unit : "", (double)d.def,
             params_->overridden(i) ? "  (overridden)" : "");
    }
    replyf("ok  %u parameter(s)", (unsigned)params_->count());
  }

  void cmdGet(char **cursor) {
    char *key = token(cursor);
    if (!key) { reply("error: usage: get <key>"); return; }
    if (!params_) { reply("error: no parameter registry"); return; }
    int16_t i = params_->indexOf(key);
    if (i < 0) { replyf("error: no such parameter \"%s\"", key); return; }
    const auto &d = params_->def((uint8_t)i);
    replyf("%s = %.4f %s (default %.4f, range %.4f..%.4f)%s", d.key,
           (double)params_->value((uint8_t)i), d.unit ? d.unit : "",
           (double)d.def, (double)d.min, (double)d.max,
           params_->overridden((uint8_t)i) ? "  (overridden)" : "");
    reply("ok");
  }

  void cmdSet(char **cursor) {
    char *key = token(cursor);
    char *val = token(cursor);
    if (!key || !val) { reply("error: usage: set <key> <value>"); return; }
    if (!params_) { reply("error: no parameter registry"); return; }

    // strtof reports failure by not advancing, which is the only way to tell
    // "0" from a word that is not a number at all - atof() returns 0.0 for
    // both, and would quietly set the value to zero.
    char *end = nullptr;
    float v = strtof(val, &end);
    if (end == val || (end && *end != '\0')) {
      replyf("error: \"%s\" is not a number", val);
      return;
    }
    ParamResult r = params_->set(key, v, moving());
    if (r == PARAM_OK) replyf("ok  %s = %.4f", key, (double)v);
    else               replyf("error: %s", paramResultText(r));
  }

  void cmdReset(char **cursor) {
    char *key = token(cursor);
    if (!key) { reply("error: usage: reset <key> | reset all"); return; }
    if (!params_) { reply("error: no parameter registry"); return; }
    if (!strcmp(key, "all")) {
      params_->resetAll(moving());
      reply("ok  every parameter back to its default");
      return;
    }
    ParamResult r = params_->reset(key, moving());
    if (r == PARAM_OK) replyf("ok  %s back to its default", key);
    else               replyf("error: %s", paramResultText(r));
  }

  //---------------------------------------------------------------- compass
  void cmdCompass(char **cursor) {
    char *sub = token(cursor);
    if (!sub) {
      if (hooks_.showCompass) hooks_.showCompass(hooks_.info_ctx);
      else reply("error: no compass information available");
      reply("ok");
      return;
    }
    if (strcmp(sub, "clear")) { replyf("error: unknown compass subcommand \"%s\"", sub); return; }
    if (!hooks_.clearCompass) { reply("error: cannot clear the calibration here"); return; }
    hooks_.clearCompass(hooks_.info_ctx);
    reply("ok  calibration forgotten - readings are raw again, and the");
    reply("    disturbance check is off until a new calibration is saved");
  }

  //------------------------------------------------------------------ agent
  void cmdAgent(char **cursor) {
    char *host = token(cursor);
    if (!host) {
      if (hooks_.showAgent) hooks_.showAgent(hooks_.info_ctx);
      else reply("error: no agent information available");
      reply("ok");
      return;
    }
    if (!hooks_.setAgent) { reply("error: cannot set the agent here"); return; }
    // "auto" is the way back to discovery. Without it, setting an address by
    // hand would be a one-way door needing a reflash to undo, which is exactly
    // the trap this command exists to remove.
    const bool clearing = !strcmp(host, "auto") || !strcmp(host, "none");
    if (!hooks_.setAgent(hooks_.info_ctx, clearing ? "" : host)) {
      reply("error: could not store that address");
      return;
    }
    if (clearing) reply("ok  back to discovery: mDNS, then DNS, then the address that worked last");
    else          replyf("ok  agent set to %s - it takes effect on the next reconnect", host);
  }

  //------------------------------------------------------------------- wifi
  void cmdWifi(char **cursor) {
    char *sub = token(cursor);
    if (!wifi_) { reply("error: no wifi store"); return; }

    if (!sub) {                                   // bare "wifi" = list
      for (uint8_t i = 0; i < wifi_->count(); ++i) {
        // Deliberately no password. This output gets pasted into bug reports.
        replyf("  %u  %-32s %s", (unsigned)i, wifi_->ssid(i),
               wifi_->hasPassword(i) ? "(has a password)" : "(open)");
      }
      replyf("ok  %u network(s), tried in this order", (unsigned)wifi_->count());
      return;
    }

    if (!strcmp(sub, "add")) {
      char *ssid = token(cursor);
      if (!ssid) { reply("error: usage: wifi add <ssid> <password>"); return; }
      // The password is the REST of the line: spaces are legal in one, and
      // tokenising it would silently store only the first word.
      char *pass = rest(*cursor);
      if (!wifi_->set(ssid, pass)) {
        reply("error: could not add - the list may be full");
        return;
      }
      replyf("ok  \"%s\" added (%s), saved", ssid,
             (pass && *pass) ? "with a password" : "open");
      return;
    }

    if (!strcmp(sub, "del") || !strcmp(sub, "forget")) {
      char *ssid = token(cursor);
      if (!ssid) { reply("error: usage: wifi del <ssid>"); return; }
      if (!wifi_->remove(ssid)) { replyf("error: \"%s\" is not in the list", ssid); return; }
      replyf("ok  \"%s\" forgotten, saved", ssid);
      return;
    }

    if (!strcmp(sub, "first")) {
      char *ssid = token(cursor);
      if (!ssid) { reply("error: usage: wifi first <ssid>"); return; }
      if (!wifi_->moveTo(ssid, 0)) { replyf("error: \"%s\" is not in the list", ssid); return; }
      replyf("ok  \"%s\" will be tried first, saved", ssid);
      return;
    }

    replyf("error: unknown wifi subcommand \"%s\"", sub);
  }

  Stream      *io_     = nullptr;
  Registry    *params_ = nullptr;
  WifiStoreT  *wifi_   = nullptr;
  ConsoleHooks hooks_;
  char         line_[CONSOLE_LINE_MAX] = {0};
  uint16_t     len_ = 0;
  bool         overlong_ = false;
};

#endif  // SETTINGS_CONSOLE_H

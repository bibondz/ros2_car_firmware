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
 * The USB settings console (T61).
 *
 * This is the way in when there is no network - and the network is the thing
 * you most often need to change. So every command is exercised here through
 * FakeSerial, replies included: a protocol tested only for "did not crash" is
 * tested for the half that never breaks.
 *
 * The cases that matter are the refusals. A console that accepts a bad value
 * quietly is worse than one that rejects a good one loudly.
 */

#include <unity.h>

#include <Preferences.h>

#include "../mock/fake_serial.h"
#include "../../src/params/param_registry.h"
#include "../../src/params/param_table.h"
#include "../../src/net/wifi_store.h"
#include "../../src/net/settings_console.h"

namespace {

bool g_moving = false;
bool movingHook() { return g_moving; }

// Stand-ins for the agent hooks, so the console's half of the contract can be
// tested without a radio: what it accepts, what it refuses, and what it stores.
std::string g_agent_set;
bool g_agent_store_ok = true;
bool g_agent_shown = false;
void showAgentHook(void *) { g_agent_shown = true; }
bool setAgentHook(void *, const char *host) {
  if (!g_agent_store_ok) return false;
  g_agent_set = host ? host : "";
  return true;
}

struct Rig {
  FakeSerial io;
  Preferences prefs_params;
  Preferences prefs_wifi;
  ParamRegistry<PARAM_COUNT> params;
  WifiStore wifi;
  SettingsConsole<FakeSerial, ParamRegistry<PARAM_COUNT>, WifiStore> console;

  Rig() {
    g_moving = false;
    prefs_params.begin("t_params", false);
    prefs_params.clear();
    prefs_wifi.begin("t_wifi", false);
    prefs_wifi.clear();
    params.begin(PARAM_TABLE, PARAM_COUNT, &prefs_params);
    WifiEntry seed[1];
    strncpy(seed[0].ssid, "seeded", WIFI_SSID_MAX - 1);
    seed[0].ssid[WIFI_SSID_MAX - 1] = '\0';
    strncpy(seed[0].pass, "seedpass", WIFI_PASS_MAX - 1);
    seed[0].pass[WIFI_PASS_MAX - 1] = '\0';
    wifi.begin(&prefs_wifi, seed, 1);

    g_agent_set.clear(); g_agent_store_ok = true; g_agent_shown = false;
    ConsoleHooks hooks;
    hooks.isMoving = movingHook;
    hooks.showAgent = showAgentHook;
    hooks.setAgent = setAgentHook;
    console.begin(&io, &params, &wifi, hooks);
  }

  /** Type a line and let the console act on it. Returns what it replied.
   *
   * Polls until the input is drained, because poll() is deliberately bounded
   * per call - it must never starve the control loop - so one call does not
   * necessarily reach the newline. The firmware calls it every loop; this does
   * the same thing.
   */
  const std::string &run(const char *line) {
    io.clearOut();
    io.push(line);
    io.push("\n");
    for (int i = 0; i < 64 && io.available(); ++i) console.poll();
    return io.out();
  }
};

/** A key that exists in the shipped table, whatever the table happens to hold. */
const char *anyKey() { return PARAM_TABLE[0].key; }

}  // namespace

void test_help_lists_the_commands(void) {
  Rig r;
  r.run("help");
  TEST_ASSERT_TRUE(r.io.said("set <key> <value>"));
  TEST_ASSERT_TRUE(r.io.said("wifi add"));
}

void test_unknown_command_says_so(void) {
  Rig r;
  r.run("frobnicate");
  TEST_ASSERT_TRUE(r.io.said("unknown command"));
  TEST_ASSERT_TRUE(r.io.said("frobnicate"));
}

void test_params_lists_every_parameter(void) {
  Rig r;
  r.run("params");
  TEST_ASSERT_TRUE(r.io.said(anyKey()));
  TEST_ASSERT_TRUE(r.io.said("ok"));
}

void test_get_and_set_round_trip(void) {
  Rig r;
  const auto &d = PARAM_TABLE[0];
  // A value guaranteed to be inside the declared range and not the default.
  float target = d.min + (d.max - d.min) * 0.5f;
  char cmd[128];
  snprintf(cmd, sizeof(cmd), "set %s %.4f", d.key, (double)target);
  r.run(cmd);
  TEST_ASSERT_TRUE(r.io.said("ok"));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, target, r.params.get(d.key));

  snprintf(cmd, sizeof(cmd), "get %s", d.key);
  r.run(cmd);
  TEST_ASSERT_TRUE(r.io.said(d.key));
  TEST_ASSERT_TRUE(r.io.said("overridden"));
}

void test_out_of_range_is_refused_with_a_reason(void) {
  Rig r;
  const auto &d = PARAM_TABLE[0];
  char cmd[128];
  snprintf(cmd, sizeof(cmd), "set %s %.4f", d.key, (double)(d.max + 1000.0f));
  r.run(cmd);
  TEST_ASSERT_TRUE(r.io.said("error"));
  // and the stored value did not move
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, d.def, r.params.get(d.key));
}

void test_a_word_is_not_a_number(void) {
  // atof() would turn "banana" into 0.0 and set the parameter to zero, which
  // is a legal value for many of them - so the refusal has to be explicit.
  Rig r;
  const auto &d = PARAM_TABLE[0];
  char cmd[128];
  snprintf(cmd, sizeof(cmd), "set %s banana", d.key);
  r.run(cmd);
  TEST_ASSERT_TRUE(r.io.said("is not a number"));
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, d.def, r.params.get(d.key));
}

void test_unknown_key_is_named(void) {
  Rig r;
  r.run("get no.such.key");
  TEST_ASSERT_TRUE(r.io.said("no such parameter"));
}

void test_reset_restores_the_default(void) {
  Rig r;
  const auto &d = PARAM_TABLE[0];
  char cmd[128];
  snprintf(cmd, sizeof(cmd), "set %s %.4f", d.key, (double)(d.min));
  r.run(cmd);
  snprintf(cmd, sizeof(cmd), "reset %s", d.key);
  r.run(cmd);
  TEST_ASSERT_TRUE(r.io.said("ok"));
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, d.def, r.params.get(d.key));
}

void test_wifi_list_never_shows_a_password(void) {
  // The whole reason the store has no password getter. A console transcript is
  // the sort of thing people paste into a bug report.
  Rig r;
  r.run("wifi add homenet hunter2");
  r.run("wifi");
  TEST_ASSERT_TRUE(r.io.said("homenet"));
  TEST_ASSERT_TRUE(r.io.said("has a password"));
  TEST_ASSERT_FALSE(r.io.said("hunter2"));
  TEST_ASSERT_FALSE(r.io.said("seedpass"));
}

void test_wifi_password_may_contain_spaces(void) {
  // Tokenising the password would store only "correct" and the board would
  // then fail to join with no hint as to why.
  Rig r;
  r.run("wifi add cafe correct horse battery staple");
  TEST_ASSERT_TRUE(r.io.said("with a password"));
  TEST_ASSERT_TRUE(r.wifi.hasPassword((uint8_t)r.wifi.indexOf("cafe")));
}

void test_wifi_add_delete_and_reorder(void) {
  Rig r;
  r.run("wifi add alpha apass");
  r.run("wifi add beta bpass");
  TEST_ASSERT_TRUE(r.wifi.indexOf("alpha") >= 0);
  r.run("wifi first beta");
  TEST_ASSERT_EQUAL(0, r.wifi.indexOf("beta"));
  r.run("wifi del alpha");
  TEST_ASSERT_TRUE(r.wifi.indexOf("alpha") < 0);
  r.run("wifi del alpha");
  TEST_ASSERT_TRUE(r.io.said("not in the list"));
}

void test_stopped_only_parameter_is_refused_while_moving(void) {
  // T70's guard, reached through the console rather than the ROS path.
  Rig r;
  int idx = -1;
  for (uint8_t i = 0; i < PARAM_COUNT; ++i) {
    if (PARAM_TABLE[i].when == PARAM_STOPPED) { idx = (int)i; break; }
  }
  if (idx < 0) { TEST_IGNORE_MESSAGE("no STOPPED_ONLY parameter in the table"); return; }
  const auto &d = PARAM_TABLE[idx];
  g_moving = true;
  char cmd[128];
  snprintf(cmd, sizeof(cmd), "set %s %.4f", d.key, (double)(d.min));
  r.run(cmd);
  TEST_ASSERT_TRUE(r.io.said("error"));
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, d.def, r.params.get(d.key));
  g_moving = false;
}

void test_an_overlong_line_is_dropped_not_truncated(void) {
  // A cut-off "set x 100" arriving as "set x 10" is a different, entirely
  // plausible command - so the line has to be refused outright.
  Rig r;
  char big[CONSOLE_LINE_MAX + 40];
  memset(big, 'a', sizeof(big) - 1);
  big[sizeof(big) - 1] = '\0';
  r.run(big);
  TEST_ASSERT_TRUE(r.io.said("too long"));
}

void test_commands_split_across_reads_still_work(void) {
  // Serial arrives in whatever chunks the USB stack feels like.
  Rig r;
  r.io.clearOut();
  r.io.push("he");
  r.console.poll();
  r.io.push("lp\n");
  r.console.poll();
  TEST_ASSERT_TRUE(r.io.said("commands:"));
}

void test_blank_lines_are_ignored(void) {
  Rig r;
  r.run("");
  TEST_ASSERT_EQUAL_STRING("", r.io.out().c_str());
}

/**
 * Pointing the robot at an agent, without a reflash.
 *
 * The agent address used to be compile-time only, so when the PC changed
 * network the board could only be corrected over a cable - on a robot that by
 * definition could no longer be reached over the network. That is the trap this
 * command exists to remove.
 */
void test_agent_can_be_set_by_hand(void) {
  Rig r;
  r.run("agent 192.168.1.50");
  TEST_ASSERT_TRUE(r.io.said("ok"));
  TEST_ASSERT_EQUAL_STRING("192.168.1.50", g_agent_set.c_str());
}

void test_agent_accepts_a_hostname_too(void) {
  Rig r;
  r.run("agent mypc.local");
  TEST_ASSERT_EQUAL_STRING("mypc.local", g_agent_set.c_str());
}

/** "auto" must exist, or setting an address by hand is a one-way door. */
void test_agent_auto_returns_to_discovery(void) {
  Rig r;
  r.run("agent 10.0.0.9");
  TEST_ASSERT_EQUAL_STRING("10.0.0.9", g_agent_set.c_str());
  r.run("agent auto");
  TEST_ASSERT_EQUAL_STRING("", g_agent_set.c_str());
  TEST_ASSERT_TRUE(r.io.said("discovery"));
}

void test_agent_with_no_argument_reports_where_it_is_looking(void) {
  Rig r;
  r.run("agent");
  TEST_ASSERT_TRUE(g_agent_shown);
  TEST_ASSERT_TRUE(r.io.said("ok"));
}

void test_agent_says_so_when_it_cannot_store(void) {
  Rig r;
  g_agent_store_ok = false;
  r.run("agent 192.168.1.50");
  TEST_ASSERT_TRUE(r.io.said("error"));
}

int run_console_tests(void) {
  RUN_TEST(test_help_lists_the_commands);
  RUN_TEST(test_unknown_command_says_so);
  RUN_TEST(test_params_lists_every_parameter);
  RUN_TEST(test_get_and_set_round_trip);
  RUN_TEST(test_out_of_range_is_refused_with_a_reason);
  RUN_TEST(test_a_word_is_not_a_number);
  RUN_TEST(test_unknown_key_is_named);
  RUN_TEST(test_reset_restores_the_default);
  RUN_TEST(test_wifi_list_never_shows_a_password);
  RUN_TEST(test_wifi_password_may_contain_spaces);
  RUN_TEST(test_wifi_add_delete_and_reorder);
  RUN_TEST(test_stopped_only_parameter_is_refused_while_moving);
  RUN_TEST(test_an_overlong_line_is_dropped_not_truncated);
  RUN_TEST(test_commands_split_across_reads_still_work);
  RUN_TEST(test_blank_lines_are_ignored);
  RUN_TEST(test_agent_can_be_set_by_hand);
  RUN_TEST(test_agent_accepts_a_hostname_too);
  RUN_TEST(test_agent_auto_returns_to_discovery);
  RUN_TEST(test_agent_with_no_argument_reports_where_it_is_looking);
  RUN_TEST(test_agent_says_so_when_it_cannot_store);
  return 0;
}

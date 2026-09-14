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
 * The Wi-Fi list that lives on the robot.
 *
 * The behaviour that matters is the seeding rule: the compiled-in header is a
 * FIRST-BOOT seed and nothing more. Get that wrong in the obvious direction -
 * seed on every boot - and a network added in the field is silently reverted
 * the next time the robot is switched on, which is the kind of fault that looks
 * like "it just stopped connecting" and has no trace anywhere.
 */
#include <unity.h>
#include <cstring>
#include "Arduino.h"
#include "Preferences.h"
#include "wifi_store.h"

static WifiEntry SEED[2];

static void makeSeed() {
    strcpy(SEED[0].ssid, "seed-one");
    strcpy(SEED[0].pass, "pass-one");
    strcpy(SEED[1].ssid, "seed-two");
    strcpy(SEED[1].pass, "");
}

void test_seeds_from_the_header_when_storage_is_empty(void) {
    makeSeed();
    Preferences nvs;
    WifiStore store;
    store.begin(&nvs, SEED, 2);
    TEST_ASSERT_EQUAL_UINT8(2, store.count());
    TEST_ASSERT_EQUAL_STRING("seed-one", store.ssid(0));
    TEST_ASSERT_TRUE(store.seededFromHeader());
    TEST_ASSERT_TRUE(store.hasPassword(0));
    TEST_ASSERT_FALSE(store.hasPassword(1));      // open network
}

void test_storage_beats_the_header_on_every_later_boot(void) {
    // The rule the whole design rests on. A network added in the field must
    // survive a reboot, and the header must not quietly undo it.
    makeSeed();
    Preferences nvs;
    {
        WifiStore first;
        first.begin(&nvs, SEED, 2);
        first.set("added-in-the-field", "fieldpass");
        first.remove("seed-one");
    }
    WifiStore second;
    second.begin(&nvs, SEED, 2);           // same seed offered again
    TEST_ASSERT_FALSE(second.seededFromHeader());
    TEST_ASSERT_EQUAL_UINT8(2, second.count());
    TEST_ASSERT_EQUAL_INT16(-1, second.indexOf("seed-one"));       // stayed deleted
    TEST_ASSERT_TRUE(second.indexOf("added-in-the-field") >= 0);   // stayed added
}

void test_setting_a_known_ssid_replaces_rather_than_duplicates(void) {
    // A duplicate SSID with a stale password would be tried first and fail,
    // making the working entry further down look like the broken one.
    Preferences nvs;
    WifiStore store;
    store.begin(&nvs, nullptr, 0);
    store.set("home", "old");
    store.set("home", "new");
    TEST_ASSERT_EQUAL_UINT8(1, store.count());
    TEST_ASSERT_EQUAL_STRING("new", store.pass(0));
}

void test_order_is_priority_and_can_be_changed(void) {
    // Order is the only way to say "prefer the hotspot I brought over the
    // building Wi-Fi that happens to be in range".
    Preferences nvs;
    WifiStore store;
    store.begin(&nvs, nullptr, 0);
    store.set("a", "1");
    store.set("b", "2");
    store.set("c", "3");
    TEST_ASSERT_EQUAL_STRING("a", store.ssid(0));

    TEST_ASSERT_TRUE(store.moveTo("c", 0));
    TEST_ASSERT_EQUAL_STRING("c", store.ssid(0));
    TEST_ASSERT_EQUAL_STRING("a", store.ssid(1));
    TEST_ASSERT_EQUAL_STRING("b", store.ssid(2));

    TEST_ASSERT_TRUE(store.moveTo("c", 2));
    TEST_ASSERT_EQUAL_STRING("a", store.ssid(0));
    TEST_ASSERT_EQUAL_STRING("b", store.ssid(1));
    TEST_ASSERT_EQUAL_STRING("c", store.ssid(2));
}

void test_removing_keeps_the_rest_in_order(void) {
    Preferences nvs;
    WifiStore store;
    store.begin(&nvs, nullptr, 0);
    store.set("a", "1");
    store.set("b", "2");
    store.set("c", "3");
    TEST_ASSERT_TRUE(store.remove("b"));
    TEST_ASSERT_EQUAL_UINT8(2, store.count());
    TEST_ASSERT_EQUAL_STRING("a", store.ssid(0));
    TEST_ASSERT_EQUAL_STRING("c", store.ssid(1));
}

void test_the_list_is_bounded(void) {
    // Bounded on purpose: this is a fixed array on a board that must not
    // fragment its heap after hours of running.
    Preferences nvs;
    WifiStore store;
    store.begin(&nvs, nullptr, 0);
    char name[8];
    for (int i = 0; i < WIFI_STORE_MAX; ++i) {
        snprintf(name, sizeof(name), "n%d", i);
        TEST_ASSERT_TRUE(store.set(name, "p"));
    }
    TEST_ASSERT_EQUAL_UINT8(WIFI_STORE_MAX, store.count());
    TEST_ASSERT_FALSE(store.set("one-too-many", "p"));
    TEST_ASSERT_EQUAL_UINT8(WIFI_STORE_MAX, store.count());
}

void test_an_over_long_ssid_is_truncated_not_overflowed(void) {
    // 802.11 caps an SSID at 32 bytes. Anything longer must be cut rather than
    // run off the end of the buffer.
    Preferences nvs;
    WifiStore store;
    store.begin(&nvs, nullptr, 0);
    char huge[128];
    memset(huge, 'x', sizeof(huge) - 1);
    huge[sizeof(huge) - 1] = '\0';
    store.set(huge, huge);
    TEST_ASSERT_EQUAL_UINT8(1, store.count());
    TEST_ASSERT_EQUAL_UINT32(WIFI_SSID_MAX - 1, strlen(store.ssid(0)));
    TEST_ASSERT_EQUAL_UINT32(WIFI_PASS_MAX - 1, strlen(store.pass(0)));
}

void test_an_empty_ssid_is_refused(void) {
    Preferences nvs;
    WifiStore store;
    store.begin(&nvs, nullptr, 0);
    TEST_ASSERT_FALSE(store.set("", "p"));
    TEST_ASSERT_FALSE(store.set(nullptr, "p"));
    TEST_ASSERT_EQUAL_UINT8(0, store.count());
}

int run_wifi_store_tests(void) {
    RUN_TEST(test_seeds_from_the_header_when_storage_is_empty);
    RUN_TEST(test_storage_beats_the_header_on_every_later_boot);
    RUN_TEST(test_setting_a_known_ssid_replaces_rather_than_duplicates);
    RUN_TEST(test_order_is_priority_and_can_be_changed);
    RUN_TEST(test_removing_keeps_the_rest_in_order);
    RUN_TEST(test_the_list_is_bounded);
    RUN_TEST(test_an_over_long_ssid_is_truncated_not_overflowed);
    RUN_TEST(test_an_empty_ssid_is_refused);
    return 0;
}

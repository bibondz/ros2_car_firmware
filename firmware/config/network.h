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
 * @file network.h
 * @brief Wi-Fi and micro-ROS agent settings.
 *
 * The Ubuntu 22.04 machine that runs ROS 2 Humble also runs the micro-ROS
 * agent and the web UI. Put its IP address in AGENT_IP.
 */
#ifndef CONF_NETWORK_H
#define CONF_NETWORK_H

#include <Arduino.h>

//---- Wi-Fi ----
// LIST EVERY NETWORK THE ROBOT MAY EVER JOIN, here, once.
//
// On boot the firmware scans the air, keeps only the networks that are both
// visible and listed below, and connects to the STRONGEST one. Move the robot
// from the workshop to the field, or swap the phone hotspot for the house
// router, and it joins whatever is in range - no editing, no reflashing.
//
// Order in this list does not matter; signal strength decides. Add a new
// network by appending one line. An open network takes "" as the password.
struct WifiNetwork { const char* ssid; const char* pass; };

// The network list itself lives in network_secrets.h, which is GITIGNORED, so
// real passwords never reach the repository. Copy the template once:
//
//     cp firmware/config/network_secrets.example.h \
//        firmware/config/network_secrets.h
//
// That list is only the first-boot seed. The board keeps its own list in NVS,
// editable from the web UI, and once NVS holds anything it wins over the seed.
#if defined(__has_include)
#  if !__has_include("network_secrets.h")
#    error "firmware/config/network_secrets.h is missing. Copy network_secrets.example.h to network_secrets.h and put your Wi-Fi name and password in it. It is gitignored on purpose."
#  endif
#endif
#include "network_secrets.h"

static const size_t WIFI_NETWORK_COUNT =
    sizeof(WIFI_NETWORKS) / sizeof(WIFI_NETWORKS[0]);

// The name the ESP32 announces over mDNS. The Ubuntu PC can then reach the
// robot as "esp32-gps-localize.local" no matter which network it joined or
// what address DHCP handed out:
//     ping esp32-gps-localize.local
//     avahi-resolve-host-name -4 esp32-gps-localize.local
static const char* WIFI_HOSTNAME = "esp32-gps-localize";

// Radio transmit power. Lowered from the default 20 dBm because the radio is
// the best suspect for the BNO085 resetting itself: the standalone IMU tool,
// which has no Wi-Fi at all, runs for minutes on the same board and bus with
// zero resets, while every Wi-Fi firmware resets the sensor repeatedly. An
// ESP32 transmit burst at full power is a few hundred milliamps, and on USB
// alone - no battery holding the rail up - that is a plausible brown-out of a
// sensor sharing 3V3.
//
// 13 dBm is roughly a fifth of the transmit current for about half the range,
// which is affordable here: the robot works within a house or a car park and
// the link is monitored continuously. If the radio turns out to be the cause,
// the proper fix is a battery or bulk capacitance on the sensor supply, and
// this goes back up.
// Default transmit power in dBm. The live value is the parameter
// net.tx_power_dbm, which starts from this and can be changed from the web
// without a reflash - range and supply current pull in opposite directions and
// which one matters depends on where the robot is working that day.
#define WIFI_TX_POWER_DBM  13.0f

//---- micro-ROS agent (udp4) ----
// The agent address can be found by NAME instead of a fixed IP, so a router
// handing out a different address after a reboot does not break the robot.
//
// Resolution order (firmware retries the whole chain on every reconnect):
//   1. mDNS  "<AGENT_HOSTNAME>.local"   - needs avahi-daemon on the Ubuntu PC
//                                         (sudo apt install avahi-daemon)
//   2. plain DNS lookup of AGENT_HOSTNAME - works if your router registers
//                                         DHCP client names
//   3. AGENT_IP below                   - last resort / fixed installations
// WHAT TO PUT IN AGENT_HOSTNAME
//   Your Ubuntu terminal shows something like   mannaja@mannaja:~$
//                                               ^user   ^hostname
//   The part AFTER the @ is the hostname. So for "mannaja@mannaja" write:
//
//       static const char* AGENT_HOSTNAME = "mannaja";
//
//   All of these also work, the firmware cleans them up for you:
//       "mannaja"            plain hostname            <- preferred
//       "mannaja.local"      mDNS name
//       "mannaja@mannaja.local"  copied straight out of the terminal prompt
//
//   Check what your PC is called:   hostname        (prints just the name)
//   Check it answers on the network: avahi-resolve -n mannaja.local
// NOTE on this machine: step 1 works. Plain "avahi-resolve -n mannaja.local"
// prints a link-local IPv6 address, which looks like a failure, but the
// firmware asks for an IPv4 A record specifically and that resolves:
//     avahi-resolve -4 -n mannaja.local     ->  192.168.100.16
// Check the IPv4 form, not the default one, before believing mDNS is broken.
//
// The fallback below is a last resort and goes stale on its own: it is a DHCP
// address, so it changes whenever the lease does. It was 192.168.100.64 and the
// machine had already moved to .16, which would have left the board dialling an
// address nobody answers if mDNS ever missed. Re-check it with `hostname -I`,
// or give the PC a reserved lease so it stops moving.
// The agent announces itself, so the board does not have to guess.
//
// Every other way of finding the agent breaks the moment the network changes:
// a hand-set address goes stale, mDNS cannot cross a subnet, and the gateway
// guess is only right when the PC happens to be the gateway. So the host
// broadcasts a small "the agent is at <ip>:<port>" datagram on this port a few
// times a second, and a board that is NOT currently linked adopts it. Change
// Wi-Fi, move the PC, hand out new DHCP leases - the board hears the new
// address within a couple of seconds and reconnects with nothing to configure.
//
// A broadcast is only trusted while the board has no session, has to carry the
// magic below, and has to come from the board's own subnet - and the agent
// still has to accept the session afterwards, so a bogus beacon costs a failed
// connection attempt and nothing more.
#define AGENT_BEACON_PORT 8889
#define AGENT_BEACON_MAGIC "GPSLOC-AGENT"

#define USE_AGENT_HOSTNAME 1
static const char*     AGENT_HOSTNAME = "mannaja";      // hostname of the Ubuntu PC
static const IPAddress AGENT_IP(192, 168, 100, 16);     // fallback, DHCP - verify
static const uint16_t  AGENT_PORT = 8888;
// Where the board fetches firmware from when told to pull an update. The
// host's web server, which is on the same machine as the agent - so the
// board already knows the address and it is already proven reachable.
#define PULL_UPDATE_PORT       8080
#define AGENT_RESOLVE_TIMEOUT_MS 3000

//---- static IP (optional) ----
// Set to 1 to skip DHCP: joins the network 1-2 seconds sooner and the address
// never changes, which also means the web UI link stays valid.
// Pick an address outside your router's DHCP pool.
#define USE_STATIC_IP 0
static const IPAddress STATIC_IP     (192, 168, 100, 60);
static const IPAddress STATIC_GATEWAY(192, 168, 100, 1);
static const IPAddress STATIC_SUBNET (255, 255, 255, 0);
static const IPAddress STATIC_DNS    (192, 168, 100, 1);

//---- ROS graph ----
#define ROS_DOMAIN_ID   10
#define ROS_NODE_NAME   "gps_localize_firmware"
#define ROS_NS          "/gps_localize"

#endif // CONF_NETWORK_H
    
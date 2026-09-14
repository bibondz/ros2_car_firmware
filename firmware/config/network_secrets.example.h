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
 * @file network_secrets.example.h
 * @brief Template for network_secrets.h — copy this file, then fill it in.
 *
 *     cp firmware/config/network_secrets.example.h \
 *        firmware/config/network_secrets.h
 *
 * network_secrets.h is gitignored and never leaves this machine, so real Wi-Fi
 * passwords stay out of the repository.
 *
 * WHAT THIS LIST IS FOR
 *   It is only the FIRST-BOOT SEED. The board keeps its own network list in
 *   NVS, editable from the Wi-Fi firmware USB settings console. On a board whose NVS is empty the seed below
 *   is copied in once; after that the board's own list wins and this file is
 *   ignored. So you normally edit Wi-Fi from the web, not here — this exists so
 *   a freshly flashed board can reach the network the very first time.
 *
 * On boot the firmware scans the air, keeps the networks that are both visible
 * and known, and joins the STRONGEST one. Order in this list does not matter.
 * An open network takes "" as the password.
 */
#ifndef NETWORK_SECRETS_H
#define NETWORK_SECRETS_H

static const WifiNetwork WIFI_NETWORKS[] = {
    { "YOUR_WIFI_NAME", "YOUR_WIFI_PASSWORD" },
    // { "my-phone-hotspot", "hotspotpass" },
    // { "field-router",     "fieldpass"   },
    // { "OpenGuestWiFi",    ""            },
};


// Password for wireless firmware updates. Anything on the network can reach the
// OTA port, and an unauthenticated one is a way to put arbitrary code on a
// machine that drives itself around - so this is not decoration. Change it.
#ifndef OTA_PASSWORD
#define OTA_PASSWORD "CHANGE_ME"
#endif

#endif  // NETWORK_SECRETS_H

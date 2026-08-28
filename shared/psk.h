#pragma once
// Pre-shared AES-128 key for all Hiker's Companion radio links.
//
// Baked directly into firmware by design: every node (T-Beam senders, the
// eventual main/gateway node) is firmware we build and flash ourselves, so
// there's no field key-exchange problem to solve yet. Revisit this if the
// project ever moves to per-device keys or over-the-air provisioning.
//
// This file is the single source of truth for the key -- every firmware
// project includes it via the `-I ../shared` build flag in its
// platformio.ini rather than keeping its own copy, so sender and receiver
// can't drift out of sync.

#include <cstdint>

static const uint8_t HC_PSK[16] = {
    0x47, 0xfd, 0xa7, 0x37, 0x63, 0x65, 0x7b, 0xd3,
    0x12, 0xc8, 0xe6, 0xde, 0x35, 0x45, 0x5f, 0x55,
};

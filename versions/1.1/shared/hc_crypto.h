#pragma once
// AES-128-GCM helpers for Hiker's Companion radio links.
//
// GCM (not plain CBC) so a receiver gets both confidentiality and an
// authentication tag -- decrypting a packet also proves it was built with
// our key and wasn't altered in transit. That matters once this feeds an
// emergency-services pipeline: a corrupted or spoofed "location"/"SOS"
// packet needs to fail loudly, not silently decrypt into garbage or a
// forged message.
//
// Header-only so each PlatformIO project can just #include it (via the
// `-I ../shared` build flag) without wiring up an extra build_src_filter
// entry for a shared .cpp.
//
// Wire format produced by hcEncrypt() / consumed by hcDecrypt():
//   [nonce (12 bytes)] [ciphertext (N bytes)] [tag (16 bytes)]

#include <Arduino.h>
#include <esp_system.h>
#include <mbedtls/gcm.h>

#include "psk.h"

static constexpr size_t HC_GCM_NONCE_LEN = 12;
static constexpr size_t HC_GCM_TAG_LEN = 16;

inline void hcFillRandom(uint8_t *buf, size_t len) {
    size_t i = 0;
    while (i < len) {
        uint32_t r = esp_random();
        size_t chunk = (len - i < sizeof(r)) ? (len - i) : sizeof(r);
        memcpy(buf + i, &r, chunk);
        i += chunk;
    }
}

// Encrypts plaintext into outBuf as [nonce][ciphertext][tag]. outBuf must be
// at least `len + HC_GCM_NONCE_LEN + HC_GCM_TAG_LEN` bytes. Returns the total
// bytes written, or 0 on failure.
//
// Uses a fresh random 96-bit nonce every call (via the ESP32's hardware
// RNG) rather than a persistent counter -- a counter that resets to 0 on
// reboot (e.g. if not saved to flash) would reuse a nonce with the same key,
// which breaks GCM's security guarantees. Collision odds for a random
// 96-bit nonce are negligible at any message volume this device will ever
// send.
inline size_t hcEncrypt(const uint8_t *plaintext, size_t len, uint8_t *outBuf) {
    uint8_t *nonce = outBuf;
    hcFillRandom(nonce, HC_GCM_NONCE_LEN);

    uint8_t *ciphertext = outBuf + HC_GCM_NONCE_LEN;
    uint8_t *tag = ciphertext + len;

    mbedtls_gcm_context ctx;
    mbedtls_gcm_init(&ctx);
    int ret = mbedtls_gcm_setkey(&ctx, MBEDTLS_CIPHER_ID_AES, HC_PSK, 128);
    if (ret == 0) {
        ret = mbedtls_gcm_crypt_and_tag(&ctx, MBEDTLS_GCM_ENCRYPT, len,
                                         nonce, HC_GCM_NONCE_LEN,
                                         nullptr, 0,
                                         plaintext, ciphertext,
                                         HC_GCM_TAG_LEN, tag);
    }
    mbedtls_gcm_free(&ctx);

    return (ret == 0) ? (HC_GCM_NONCE_LEN + len + HC_GCM_TAG_LEN) : 0;
}

// Decrypts + authenticates a buffer produced by hcEncrypt(). plaintextOut
// must be at least `inLen - HC_GCM_NONCE_LEN - HC_GCM_TAG_LEN` bytes.
// Returns the plaintext length, or 0 if the buffer is too short or the
// auth tag doesn't match (corrupted, tampered, or wrong key).
inline size_t hcDecrypt(const uint8_t *inBuf, size_t inLen, uint8_t *plaintextOut) {
    if (inLen < HC_GCM_NONCE_LEN + HC_GCM_TAG_LEN) {
        return 0;
    }

    const uint8_t *nonce = inBuf;
    const uint8_t *ciphertext = inBuf + HC_GCM_NONCE_LEN;
    size_t ciphertextLen = inLen - HC_GCM_NONCE_LEN - HC_GCM_TAG_LEN;
    const uint8_t *tag = ciphertext + ciphertextLen;

    mbedtls_gcm_context ctx;
    mbedtls_gcm_init(&ctx);
    int ret = mbedtls_gcm_setkey(&ctx, MBEDTLS_CIPHER_ID_AES, HC_PSK, 128);
    if (ret == 0) {
        ret = mbedtls_gcm_auth_decrypt(&ctx, ciphertextLen,
                                        nonce, HC_GCM_NONCE_LEN,
                                        nullptr, 0,
                                        tag, HC_GCM_TAG_LEN,
                                        ciphertext, plaintextOut);
    }
    mbedtls_gcm_free(&ctx);

    return (ret == 0) ? ciphertextLen : 0;
}

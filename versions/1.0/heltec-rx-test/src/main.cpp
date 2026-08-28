// Hiker's Companion — LoRa RX bench test (Heltec WiFi LoRa 32 V3)
//
// Listens continuously and prints whatever it receives, plus RSSI/SNR, so we
// can confirm the T-Beam Supreme's TX is actually reaching a second radio
// before trusting "sent ok" on the TX side alone.
//
// Radio params below must match the T-Beam's main.cpp exactly (frequency,
// bandwidth, spreading factor, coding rate) or the two sides won't hear each
// other even though both radios are working.

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>

#include "hc_crypto.h"

// Heltec WiFi LoRa 32 V3 SX1262 pinout (from Heltec's official pin_config.h).
static constexpr int LORA_CS_PIN = 8;
static constexpr int LORA_SCK_PIN = 9;
static constexpr int LORA_MOSI_PIN = 10;
static constexpr int LORA_MISO_PIN = 11;
static constexpr int LORA_RST_PIN = 12;
static constexpr int LORA_BUSY_PIN = 13;
static constexpr int LORA_DIO1_PIN = 14;

// Must match LORA_* constants in firmware/src/main.cpp.
static constexpr float LORA_FREQUENCY_MHZ = 915.0;
static constexpr float LORA_BANDWIDTH_KHZ = 125.0;
static constexpr uint8_t LORA_SPREADING_FACTOR = 10;
static constexpr uint8_t LORA_CODING_RATE = 5;

SX1262 radio = new Module(LORA_CS_PIN, LORA_DIO1_PIN, LORA_RST_PIN, LORA_BUSY_PIN);

// readData() must only be called after DIO1 actually signals RX-done -- calling
// it on a bare polling loop reads stale FIFO contents and drops the radio out
// of continuous receive. This ISR flag is RadioLib's documented pattern for
// SX126x continuous receive.
volatile bool packetReceived = false;

static void onPacketReceived() {
    packetReceived = true;
}

static void printHex(const char *label, const uint8_t *data, size_t len) {
    Serial.print(label);
    for (size_t i = 0; i < len; i++) {
        Serial.printf("%02x", data[i]);
    }
    Serial.println();
}

static void haltWithError(const char *message) {
    Serial.println(message);
    while (true) {
        delay(1000);
    }
}

void setup() {
    Serial.begin(115200);
    delay(1500);

    SPI.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_CS_PIN);

    int state = radio.begin();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("Radio init failed, code %d\n", state);
        haltWithError("Halting.");
    }

    radio.setFrequency(LORA_FREQUENCY_MHZ);
    radio.setBandwidth(LORA_BANDWIDTH_KHZ);
    radio.setSpreadingFactor(LORA_SPREADING_FACTOR);
    radio.setCodingRate(LORA_CODING_RATE);

    radio.setDio1Action(onPacketReceived);

    state = radio.startReceive();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("startReceive failed, code %d\n", state);
        haltWithError("Halting.");
    }

    Serial.println("Heltec LoRa RX test ready - listening for T-Beam packets");
}

void loop() {
    if (!packetReceived) {
        return;
    }
    packetReceived = false;

    static constexpr size_t kMaxCipherLen = 128;
    size_t cipherLen = radio.getPacketLength();

    if (cipherLen == 0 || cipherLen > kMaxCipherLen) {
        Serial.printf("RX: bad packet length %u, dropped\n", (unsigned)cipherLen);
        radio.startReceive();
        return;
    }

    uint8_t cipherBuf[kMaxCipherLen];
    int state = radio.readData(cipherBuf, cipherLen);

    if (state == RADIOLIB_ERR_NONE) {
        printHex("RX (over-the-air bytes): ", cipherBuf, cipherLen);

        uint8_t plainBuf[kMaxCipherLen];
        size_t plainLen = hcDecrypt(cipherBuf, cipherLen, plainBuf);

        if (plainLen == 0) {
            Serial.println("RX: decrypt/auth failed - corrupted, tampered, or wrong key");
        } else {
            Serial.print("RX: ");
            Serial.write(plainBuf, plainLen);
            Serial.printf("  (RSSI %.1f dBm, SNR %.1f dB)\n", radio.getRSSI(), radio.getSNR());
        }
    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
        Serial.println("RX: CRC mismatch, dropped");
    } else {
        Serial.printf("readData failed, code %d\n", state);
    }

    radio.startReceive();
}

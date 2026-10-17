// Hiker's Companion LoRa RX bench test (Heltec WiFi LoRa 32 V3).
// Listens continuously, prints what it receives plus RSSI/SNR, and ACKs each
// packet. Radio params must match the T-Beam's main.cpp exactly.

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>

#include "hc_crypto.h"

// Heltec WiFi LoRa 32 V3 SX1262 pinout (Heltec's pin_config.h).
static constexpr int LORA_CS_PIN = 8;
static constexpr int LORA_SCK_PIN = 9;
static constexpr int LORA_MOSI_PIN = 10;
static constexpr int LORA_MISO_PIN = 11;
static constexpr int LORA_RST_PIN = 12;
static constexpr int LORA_BUSY_PIN = 13;
static constexpr int LORA_DIO1_PIN = 14;

// Onboard SSD1306 OLED. Vext gates its power, off at boot; active LOW.
static constexpr int OLED_SDA_PIN = 17;
static constexpr int OLED_SCL_PIN = 18;
static constexpr int OLED_RST_PIN = 21;
static constexpr int VEXT_PIN = 36;
static constexpr uint8_t OLED_WIDTH = 128;
static constexpr uint8_t OLED_HEIGHT = 64;
static constexpr uint8_t OLED_I2C_ADDR = 0x3C;

// Must match LORA_* constants in firmware/src/main.cpp. SF and bandwidth are set
// per build with -DLORA_SF=<7-12> and -DLORA_BW=<kHz> (see the envs in
// platformio.ini); default SF12 at 500 kHz.
#ifndef LORA_SF
#define LORA_SF 12
#endif
#ifndef LORA_BW
#define LORA_BW 500.0
#endif
static constexpr float LORA_FREQUENCY_MHZ = 915.0;
static constexpr float LORA_BANDWIDTH_KHZ = LORA_BW;
static constexpr uint8_t LORA_SPREADING_FACTOR = LORA_SF;
static constexpr uint8_t LORA_CODING_RATE = 5;
// ACK power, set per build like LORA_SF. Unset in RadioLib it would default to
// 10 dBm, and ACKs dropped out first at range.
#ifndef LORA_TX_POWER
#define LORA_TX_POWER 22
#endif
static constexpr int8_t LORA_TX_POWER_DBM = LORA_TX_POWER;

SX1262 radio = new Module(LORA_CS_PIN, LORA_DIO1_PIN, LORA_RST_PIN, LORA_BUSY_PIN);
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RST_PIN);

// readData() must only run after DIO1 signals RX-done; polling reads stale FIFO
// data and drops the radio out of continuous receive.
volatile bool packetReceived = false;

static void onPacketReceived() {
    packetReceived = true;
}

// Prefixes receive-path log lines with millis() so the receiver and transmitter
// logs from a test run can be lined up.
static void logStamp() {
    Serial.printf("[%lu] ", (unsigned long)millis());
}

static void printHex(const char *label, const uint8_t *data, size_t len) {
    logStamp();
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

static void initDisplay() {
    pinMode(VEXT_PIN, OUTPUT);
    digitalWrite(VEXT_PIN, LOW); // enable power to the OLED
    delay(100);

    pinMode(OLED_RST_PIN, OUTPUT);
    digitalWrite(OLED_RST_PIN, LOW);
    delay(20);
    digitalWrite(OLED_RST_PIN, HIGH);

    Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR)) {
        haltWithError("Display init failed - check I2C wiring to SSD1306");
    }
    display.clearDisplay();
    display.display();
}

// The panel wakes on each packet and sleeps on a timer checked in loop(); no
// delay(), which would drop packets during continuous receive.
static constexpr uint32_t DISPLAY_HOLD_MS = 5000;
bool displayOn = false;
bool displayOffScheduled = false;
uint32_t displayOffAtMs = 0;

static void displayWake() {
    if (!displayOn) {
        display.ssd1306_command(SSD1306_DISPLAYON);
        displayOn = true;
    }
    displayOffScheduled = false;
}

static void displaySleep() {
    if (displayOn) {
        display.ssd1306_command(SSD1306_DISPLAYOFF);
        displayOn = false;
    }
    displayOffScheduled = false;
}

static void scheduleDisplayOff(uint32_t holdMs) {
    displayOffAtMs = millis() + holdMs;
    displayOffScheduled = true;
}

static void showBanner(const char *line1, const char *line2) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print(line1);

    display.setTextSize(2);
    display.setCursor(0, 20);
    display.print(line2);

    display.display();
}

// Copies one comma-separated field of a decrypted payload into out. The payload
// isn't null-terminated, so it's bounded by len.
static void splitField(const uint8_t *data, size_t len, int fieldIndex, char *out, size_t outSize) {
    size_t fieldStart = 0;
    int currentField = 0;
    for (size_t pos = 0; pos <= len; pos++) {
        if (pos == len || data[pos] == ',') {
            if (currentField == fieldIndex) {
                size_t fieldLen = pos - fieldStart;
                if (fieldLen > outSize - 1) {
                    fieldLen = outSize - 1;
                }
                memcpy(out, data + fieldStart, fieldLen);
                out[fieldLen] = '\0';
                return;
            }
            fieldStart = pos + 1;
            currentField++;
        }
    }
    out[0] = '\0';
}

// Sends an encrypted "ACK,<node>,<id>" so only the sending node's waitForAck()
// matches it. loop() re-arms receive afterward. The DIO1 callback fires on any
// rising edge, including this transmit's own TX-done, so packetReceived is
// cleared after to avoid reading our own ACK as an incoming packet.
static void sendAck(const char *node, const char *id) {
    String payload = String("ACK,") + node + "," + id;

    static constexpr size_t kMaxPlaintextLen = 32;
    uint8_t plainBuf[kMaxPlaintextLen];
    size_t plainLen = payload.length();
    if (plainLen > kMaxPlaintextLen) {
        logStamp();
        Serial.println("  ACK payload too long, dropping");
        return;
    }
    memcpy(plainBuf, payload.c_str(), plainLen);

    uint8_t cipherBuf[kMaxPlaintextLen + HC_GCM_NONCE_LEN + HC_GCM_TAG_LEN];
    size_t cipherLen = hcEncrypt(plainBuf, plainLen, cipherBuf);
    if (cipherLen == 0) {
        logStamp();
        Serial.println("  ACK encrypt failed");
        return;
    }

    int state = radio.transmit(cipherBuf, cipherLen);
    packetReceived = false; // discard any spurious flag set by this transmit's own TX-done IRQ
    logStamp();
    if (state == RADIOLIB_ERR_NONE) {
        Serial.printf("  ACK sent for node %s id %s\n", node, id);
    } else {
        Serial.printf("  ACK send failed, code %d\n", state);
    }
}

// The message type is large; RSSI/SNR are secondary detail for range testing.
static void showReceived(const char *type, float rssi, float snr) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print(type);

    display.setTextSize(1);
    display.setCursor(0, 24);
    display.printf("RSSI %.1f dBm", rssi);
    display.setCursor(0, 36);
    display.printf("SNR %.1f dB", snr);

    display.display();
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
    radio.setOutputPower(LORA_TX_POWER_DBM);

    radio.setDio1Action(onPacketReceived);

    state = radio.startReceive();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("startReceive failed, code %d\n", state);
        haltWithError("Halting.");
    }

    initDisplay();
    displayWake();
    showBanner("Heltec RX", "Listening...");
    scheduleDisplayOff(DISPLAY_HOLD_MS);

    Serial.printf("Heltec LoRa RX test ready - listening for T-Beam packets (SF%d, %.0f kHz, %d dBm ACKs)\n",
                  LORA_SPREADING_FACTOR, LORA_BANDWIDTH_KHZ, LORA_TX_POWER_DBM);
}

void loop() {
    if (displayOffScheduled && millis() >= displayOffAtMs) {
        displaySleep();
    }

    if (!packetReceived) {
        return;
    }
    packetReceived = false;

    displayWake();
    scheduleDisplayOff(DISPLAY_HOLD_MS);

    // Must stay >= the T-Beam's kMaxPlaintextLen (160) + nonce + tag (188 total);
    // SOS packets carry the contact list.
    static constexpr size_t kMaxCipherLen = 192;
    size_t cipherLen = radio.getPacketLength();

    if (cipherLen == 0 || cipherLen > kMaxCipherLen) {
        logStamp();
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
            logStamp();
            Serial.println("RX: decrypt/auth failed - corrupted, tampered, or wrong key");
            showBanner("Heltec RX", "AUTH FAIL");
        } else {
            logStamp();
            Serial.print("RX: ");
            Serial.write(plainBuf, plainLen);
            Serial.printf("  (RSSI %.1f dBm, SNR %.1f dB)\n", radio.getRSSI(), radio.getSNR());

            // Payload is "<type>,<node>,<id>,<location>[,<contacts>]".
            char type[16];
            char node[8];
            char id[16];
            splitField(plainBuf, plainLen, 0, type, sizeof(type));
            splitField(plainBuf, plainLen, 1, node, sizeof(node));
            splitField(plainBuf, plainLen, 2, id, sizeof(id));

            // Sent before the display update to keep ACK latency low.
            sendAck(node, id);
            showReceived(type, radio.getRSSI(), radio.getSNR());
        }
    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
        logStamp();
        Serial.println("RX: CRC mismatch, dropped");
        showBanner("Heltec RX", "CRC FAIL");
    } else {
        logStamp();
        Serial.printf("readData failed, code %d\n", state);
        showBanner("Heltec RX", "RX ERROR");
    }

    radio.startReceive();
}

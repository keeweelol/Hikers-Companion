// Hiker's Companion — LoRa RX bench test (Heltec WiFi LoRa 32 V3)
//
// Listens continuously and prints whatever it receives, plus RSSI/SNR, so we
// can confirm the T-Beam Supreme's TX is actually reaching a second radio
// before trusting "sent ok" on the TX side alone.
//
// Radio params below must match the T-Beam's main.cpp exactly (frequency,
// bandwidth, spreading factor, coding rate) or the two sides won't hear each
// other even though both radios are working.

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
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

// Heltec V3's onboard SSD1306 OLED (also from pin_config.h). Vext gates
// power to the OLED (and a couple other peripherals) and is off by default
// at boot -- active LOW to enable.
static constexpr int OLED_SDA_PIN = 17;
static constexpr int OLED_SCL_PIN = 18;
static constexpr int OLED_RST_PIN = 21;
static constexpr int VEXT_PIN = 36;
static constexpr uint8_t OLED_WIDTH = 128;
static constexpr uint8_t OLED_HEIGHT = 64;
static constexpr uint8_t OLED_I2C_ADDR = 0x3C;

// Must match LORA_* constants in firmware/src/main.cpp.
static constexpr float LORA_FREQUENCY_MHZ = 915.0;
static constexpr float LORA_BANDWIDTH_KHZ = 125.0;
static constexpr uint8_t LORA_SPREADING_FACTOR = 10;
static constexpr uint8_t LORA_CODING_RATE = 5;

SX1262 radio = new Module(LORA_CS_PIN, LORA_DIO1_PIN, LORA_RST_PIN, LORA_BUSY_PIN);
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RST_PIN);

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

// Panel stays off between packets to save power -- it wakes on any received
// signal and sleeps again on a timer checked in loop() (no delay() here,
// since that would drop packets while the radio is in continuous receive).
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

// Pulls one comma-separated field out of a decrypted payload (e.g. field 0
// = "SOS", field 1 = "42" from "SOS,42,37.774900,-122.419400,1.2,340")
// without assuming it's null-terminated -- hcDecrypt() only guarantees
// plainLen valid bytes.
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

// Encrypts and transmits "ACK,<id>" back to whoever sent the message we
// just received -- id lets the sender's waitForAck() match this reply to
// its specific send. Interrupts the ongoing continuous receive, same as
// any other radio.transmit() call; loop() re-arms it with startReceive()
// right after, same as it already does for every other branch here.
//
// setDio1Action()'s callback fires on ANY rising edge on that pin, with no
// idea whether the chip currently means RX-done or TX-done by it -- so this
// transmit's own TX-done pulse sets packetReceived = true too. Left alone,
// the next loop() iteration would treat that as a real receive and read
// whatever's left in the shared TX/RX buffer (part fresh ACK ciphertext,
// part leftover tail from the packet we just processed), which reliably
// fails the GCM auth check but is still a spurious, printed failure. Clear
// the flag afterward so our own ACKs can't be mistaken for incoming packets.
static void sendAck(const char *id) {
    String payload = String("ACK,") + id;

    static constexpr size_t kMaxPlaintextLen = 32;
    uint8_t plainBuf[kMaxPlaintextLen];
    size_t plainLen = payload.length();
    if (plainLen > kMaxPlaintextLen) {
        Serial.println("  ACK payload too long, dropping");
        return;
    }
    memcpy(plainBuf, payload.c_str(), plainLen);

    uint8_t cipherBuf[kMaxPlaintextLen + HC_GCM_NONCE_LEN + HC_GCM_TAG_LEN];
    size_t cipherLen = hcEncrypt(plainBuf, plainLen, cipherBuf);
    if (cipherLen == 0) {
        Serial.println("  ACK encrypt failed");
        return;
    }

    int state = radio.transmit(cipherBuf, cipherLen);
    packetReceived = false; // discard any spurious flag set by this transmit's own TX-done IRQ
    if (state == RADIOLIB_ERR_NONE) {
        Serial.printf("  ACK sent for id %s\n", id);
    } else {
        Serial.printf("  ACK send failed, code %d\n", state);
    }
}

// SOS gets top billing (large text) since that's the one thing worth seeing
// at a glance; RSSI/SNR are secondary detail for range testing.
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

    Serial.println("Heltec LoRa RX test ready - listening for T-Beam packets");
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

    // Must stay >= the T-Beam's kMaxPlaintextLen (160) + HC_GCM_NONCE_LEN +
    // HC_GCM_TAG_LEN (188 total) -- the first SOS transmission after
    // activation rides a stored-contacts field alongside the location (see
    // sosContactsPending in firmware/src/main.cpp), which needs more room
    // than the plain location-only packets this was originally sized for.
    static constexpr size_t kMaxCipherLen = 192;
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
            showBanner("Heltec RX", "AUTH FAIL");
        } else {
            Serial.print("RX: ");
            Serial.write(plainBuf, plainLen);
            Serial.printf("  (RSSI %.1f dBm, SNR %.1f dB)\n", radio.getRSSI(), radio.getSNR());

            char type[16];
            char id[16];
            splitField(plainBuf, plainLen, 0, type, sizeof(type));
            splitField(plainBuf, plainLen, 1, id, sizeof(id));

            // Sent before the display update to keep ACK latency low against
            // the sender's fixed listen window.
            sendAck(id);
            showReceived(type, radio.getRSSI(), radio.getSNR());
        }
    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
        Serial.println("RX: CRC mismatch, dropped");
        showBanner("Heltec RX", "CRC FAIL");
    } else {
        Serial.printf("readData failed, code %d\n", state);
        showBanner("Heltec RX", "RX ERROR");
    }

    radio.startReceive();
}

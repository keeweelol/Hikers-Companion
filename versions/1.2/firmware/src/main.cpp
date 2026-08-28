// Hiker's Companion — Milestone 1 firmware
//
// Reads GPS location and transmits it over LoRa on a fixed interval.
// No button/SOS logic or display yet — this exists to prove the GPS -> LoRa
// path works end to end on real hardware before building the full SOS flow.
//
// Target: LILYGO T-Beam Supreme (ESP32-S3, SX1262, AXP2101, GNSS)

#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>
#include <TinyGPS++.h>
#include <XPowersLib.h>

#include "hc_crypto.h"
#include "pins.h"

// ---- Radio configuration -------------------------------------------------
// 915 MHz sits in the US902-928 MHz ISM band the report identifies as our
// target. Spreading factor is not finalized in the report (SF7-SF12
// range/rate tradeoff) -- SF10 is a reasonable starting point that favors
// range over throughput; revisit once we have real range-test data.
static constexpr float LORA_FREQUENCY_MHZ = 915.0;
static constexpr float LORA_BANDWIDTH_KHZ = 125.0;
static constexpr uint8_t LORA_SPREADING_FACTOR = 10;
static constexpr uint8_t LORA_CODING_RATE = 5;
static constexpr int8_t LORA_TX_POWER_DBM = 17;

// Report's flow chart calls for polling roughly once a minute; kept as a
// constant here so it's easy to tune once battery-life testing informs it.
static constexpr uint32_t SEND_INTERVAL_MS = 60000;

// How long to listen for an ACK after transmitting before giving up. This
// blocks loop() (GPS/button polling included) for up to this long right
// after a send -- acceptable since it only happens once per send, not
// continuously.
static constexpr uint32_t ACK_TIMEOUT_MS = 2000;

// GPIO0 has no external debounce hardware on this board -- a plain
// digitalRead() chatters across several transitions on every press/release.
static constexpr uint32_t BUTTON_DEBOUNCE_MS = 50;

// ---- Globals ---------------------------------------------------------
// XPowersAXP2101's power-control methods are only public through the
// XPowersLibInterface base -- the concrete class re-declares them protected,
// so the pointer must be typed as the interface (this matches XPowersLib's
// own usage pattern, not a workaround).
TwoWire PMUWire = TwoWire(1);
XPowersLibInterface *PMU = nullptr;

HardwareSerial &SerialGPS = Serial1;
TinyGPSPlus gps;

SX1262 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);

static constexpr uint8_t DISPLAY_WIDTH = 128;
static constexpr uint8_t DISPLAY_HEIGHT = 64;
static constexpr uint8_t DISPLAY_I2C_ADDR = 0x3C;
Adafruit_SH1106G display(DISPLAY_WIDTH, DISPLAY_HEIGHT, &Wire, -1);

uint32_t lastSendMs = 0;
uint16_t nextMessageId = 0;

// BUTTON_PIN uses INPUT_PULLUP, so idle-high/pressed-low.
bool lastButtonReading = HIGH;
bool buttonState = HIGH;
uint32_t lastButtonChangeMs = 0;

// A press starts a standing SOS beacon (every periodic send goes out flagged
// "SOS" instead of "OK") rather than sending just one SOS packet and falling
// back to routine sends. A second press cancels it.
bool sosActive = false;

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

// The T-Beam Supreme gates LoRa and GPS power behind the AXP2101 PMU's ALDO3
// and ALDO4 rails -- they are off by default at boot. Without this, the
// radio and GPS modules never power up even though the wiring is correct.
static void initPower() {
    PMUWire.begin(PMU_SDA, PMU_SCL);

    PMU = new XPowersAXP2101(PMUWire);
    if (!PMU->init()) {
        haltWithError("PMU init failed - check I2C wiring to AXP2101");
    }

    PMU->setPowerChannelVoltage(XPOWERS_ALDO3, 3300); // LoRa rail
    PMU->enablePowerOutput(XPOWERS_ALDO3);

    PMU->setPowerChannelVoltage(XPOWERS_ALDO4, 3300); // GPS rail
    PMU->enablePowerOutput(XPOWERS_ALDO4);
}

static void initGPS() {
    pinMode(GPS_EN_PIN, OUTPUT);
    digitalWrite(GPS_EN_PIN, HIGH);
    SerialGPS.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
}

static void initDisplay() {
    Wire.begin(I2C_SDA, I2C_SCL);
    if (!display.begin(DISPLAY_I2C_ADDR, true)) {
        haltWithError("Display init failed - check I2C wiring to SH1106");
    }
    display.clearDisplay();
    display.display();
}

// Panel stays off between transmissions to save power -- it wakes for a
// send, holds through the result, then sleeps again on a timer checked in
// loop() (no delay() here, since that would stall GPS/button/radio polling).
static constexpr uint32_t DISPLAY_HOLD_MS = 5000;
bool displayOn = false;
bool displayOffScheduled = false;
uint32_t displayOffAtMs = 0;

static void displayWake() {
    if (!displayOn) {
        display.oled_command(SH110X_DISPLAYON);
        displayOn = true;
    }
    displayOffScheduled = false;
}

static void displaySleep() {
    if (displayOn) {
        display.oled_command(SH110X_DISPLAYOFF);
        displayOn = false;
    }
    displayOffScheduled = false;
}

static void scheduleDisplayOff(uint32_t holdMs) {
    displayOffAtMs = millis() + holdMs;
    displayOffScheduled = true;
}

// line2 is the big/important word ("Sending", "Delivered", "Send failed");
// line1 is the small context above it (message type, or a boot message).
// No ACK yet (see FW-06 in the backlog), so "Delivered" here means "the
// radio call returned success," not "a human received this."
static void showStatus(const char *line1, const char *line2) {
    display.clearDisplay();
    display.setTextColor(SH110X_WHITE);

    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print(line1);

    display.setTextSize(2);
    display.setCursor(0, 20);
    display.print(line2);

    display.display();
}

static void initRadio() {
    SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN, RADIO_CS_PIN);

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
}

// Builds "lat,lng,hdop,age_ms" or "NOFIX,age_ms" if we don't have a valid fix yet.
static String buildLocationMessage() {
    if (!gps.location.isValid()) {
        return "NOFIX," + String(gps.location.age());
    }

    String message;
    message += String(gps.location.lat(), 6);
    message += ",";
    message += String(gps.location.lng(), 6);
    message += ",";
    message += String(gps.hdop.hdop(), 1);
    message += ",";
    message += String(gps.location.age());
    return message;
}

// Listens for up to ACK_TIMEOUT_MS for a reply matching "ACK,<expectedId>".
// radio.receive() blocks (with the given timeout) and internally calls
// readData() for us, so no separate startReceive()/interrupt dance is
// needed here -- that pattern is only for continuous listening.
static bool waitForAck(uint16_t expectedId) {
    static constexpr size_t kMaxAckCipherLen = 64;
    uint8_t ackCipherBuf[kMaxAckCipherLen];

    int state = radio.receive(ackCipherBuf, kMaxAckCipherLen, ACK_TIMEOUT_MS);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.println("  no ACK received");
        return false;
    }

    size_t ackCipherLen = radio.getPacketLength();
    uint8_t ackPlainBuf[kMaxAckCipherLen];
    size_t ackPlainLen = hcDecrypt(ackCipherBuf, ackCipherLen, ackPlainBuf);
    if (ackPlainLen == 0) {
        Serial.println("  ACK decrypt/auth failed");
        return false;
    }

    char ackStr[32];
    size_t copyLen = ackPlainLen < sizeof(ackStr) - 1 ? ackPlainLen : sizeof(ackStr) - 1;
    memcpy(ackStr, ackPlainBuf, copyLen);
    ackStr[copyLen] = '\0';

    int ackId = -1;
    if (sscanf(ackStr, "ACK,%d", &ackId) == 1 && ackId == expectedId) {
        Serial.printf("  ACK received for id %d\n", ackId);
        return true;
    }

    Serial.printf("  ACK mismatch or malformed: %s\n", ackStr);
    return false;
}

// Encrypts and transmits a "<type>,<id>,<location>" packet, where type is
// "OK" for a routine poll or "SOS" for a button-triggered emergency send,
// and id lets waitForAck() match a reply to this specific send.
static void sendLocationPacket(const char *type) {
    displayWake();
    showStatus(type, "Sending...");

    uint16_t msgId = nextMessageId++;
    String payload = String(type) + "," + String(msgId) + "," + buildLocationMessage();
    Serial.print("TX (plaintext): ");
    Serial.println(payload);

    static constexpr size_t kMaxPlaintextLen = 64;
    uint8_t plainBuf[kMaxPlaintextLen];
    size_t plainLen = payload.length();
    if (plainLen > kMaxPlaintextLen) {
        Serial.println("  payload too long for crypto buffer, dropping");
        showStatus(type, "Send failed");
        scheduleDisplayOff(DISPLAY_HOLD_MS);
        return;
    }
    memcpy(plainBuf, payload.c_str(), plainLen);

    uint8_t cipherBuf[kMaxPlaintextLen + HC_GCM_NONCE_LEN + HC_GCM_TAG_LEN];
    size_t cipherLen = hcEncrypt(plainBuf, plainLen, cipherBuf);
    if (cipherLen == 0) {
        Serial.println("  encrypt failed");
        showStatus(type, "Send failed");
        scheduleDisplayOff(DISPLAY_HOLD_MS);
        return;
    }
    printHex("TX (over-the-air bytes): ", cipherBuf, cipherLen);

    int state = radio.transmit(cipherBuf, cipherLen);
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println("  sent ok, waiting for ACK...");
        if (waitForAck(msgId)) {
            showStatus(type, "Delivered");
        } else {
            showStatus(type, "No ACK");
        }
    } else {
        Serial.printf("  send failed, code %d\n", state);
        showStatus(type, "Send failed");
    }
    scheduleDisplayOff(DISPLAY_HOLD_MS);
}

// Debounces BUTTON_PIN and fires one SOS send per confirmed press. Runs
// every loop() iteration rather than off an interrupt -- at this poll rate
// missing a press by a millisecond or two doesn't matter, and it keeps the
// button on the same simple polling model as everything else in loop().
static void pollButton() {
    bool reading = digitalRead(BUTTON_PIN);
    if (reading != lastButtonReading) {
        lastButtonChangeMs = millis();
        lastButtonReading = reading;
    }

    if (millis() - lastButtonChangeMs > BUTTON_DEBOUNCE_MS && reading != buttonState) {
        buttonState = reading;
        if (buttonState == LOW) {
            sosActive = !sosActive;
            Serial.println(sosActive ? "SOS button pressed - beacon started" : "SOS button pressed - beacon canceled");
            sendLocationPacket(sosActive ? "SOS" : "OK");
            lastSendMs = millis(); // don't also fire the routine send right after
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(1500); // let USB CDC come up before first prints

    pinMode(BUTTON_PIN, INPUT_PULLUP);

    initPower();
    initGPS();
    initRadio();
    initDisplay();

    displayWake();
    showStatus("Hiker's Companion", "Ready");
    scheduleDisplayOff(DISPLAY_HOLD_MS);
    Serial.println("Hiker's Companion - GPS/LoRa send loop ready");
}

void loop() {
    if (displayOffScheduled && millis() >= displayOffAtMs) {
        displaySleep();
    }

    while (SerialGPS.available() > 0) {
        gps.encode(SerialGPS.read());
    }

    pollButton();

    uint32_t now = millis();
    if (now - lastSendMs >= SEND_INTERVAL_MS) {
        lastSendMs = now;
        sendLocationPacket(sosActive ? "SOS" : "OK");
    }
}

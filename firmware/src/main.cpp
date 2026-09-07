// Hiker's Companion firmware
//
// Reads GPS location and transmits it over LoRa on a fixed interval, with an
// SOS button, OLED status display, and delivery ACK. Between sends the
// GPS/LoRa PMU rails are gated off and the ESP32 light-sleeps (see loop()) —
// that duty cycle is what the report's average current draw is based on.
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
#include <driver/gpio.h>
#include <esp_sleep.h>

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

// GPS and the LoRa radio are only powered for the acquire+send window each
// cycle (see gpsRailUp()/radioRailUp() and the light sleep in loop()) --
// unlike milestone 1's always-on loop, this is a cold GPS acquisition every
// cycle, not a running fix. 25s is a placeholder budget pending real TTFF
// measurements on this module, same caveat as the other timing constants.
static constexpr uint32_t GPS_ACQUIRE_BUDGET_MS = 25000;

// An SOS press gets a much shorter acquisition window than a routine poll --
// getting the packet on the air fast matters more than a precise fix, and a
// hiker who just hit the button shouldn't wait out the full budget above.
// buildLocationMessage() still reports whatever fix (possibly stale, or
// none) TinyGPSPlus has on hand either way.
static constexpr uint32_t GPS_ACQUIRE_QUICK_MS = 2000;

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
// send and holds through the result via holdDisplayThenSleep() below, which
// light-sleeps for the hold instead of delay()ing so it isn't a needless
// full-power stretch either.
static constexpr uint32_t DISPLAY_HOLD_MS = 5000;
bool displayOn = false;

static void displayWake() {
    if (!displayOn) {
        display.oled_command(SH110X_DISPLAYON);
        displayOn = true;
    }
}

static void displaySleep() {
    if (displayOn) {
        display.oled_command(SH110X_DISPLAYOFF);
        displayOn = false;
    }
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

// LoRa rail (ALDO3) and GPS rail (ALDO4) are the same PMU outputs initPower()
// turns on at boot -- cut them here whenever GPS/radio have nothing to do
// (the light sleep in loop()) instead of leaving both powered for the full
// ~60s interval the way milestone 1 did.
static void gpsRailDown() {
    digitalWrite(GPS_EN_PIN, LOW); // avoid driving EN into an otherwise-unpowered GPS chip
    PMU->disablePowerOutput(XPOWERS_ALDO4);
}

static void radioRailDown() {
    PMU->disablePowerOutput(XPOWERS_ALDO3);
}

// The SX1262 and the GPS module both lose all internal state when their
// rail is cut, so bringing a rail back up means a full re-init, not just
// flipping the PMU output back on -- reuses the same init*() setup() calls.
// The delay() after each enablePowerOutput() is a conservative placeholder
// for AXP2101 rail ramp-up before touching the chip on it over SPI/I2C --
// initPower() gets this for free at boot from the setup steps that run
// after it, but here initGPS()/initRadio() would otherwise run immediately
// on a rail that was just re-enabled. Not measured against real hardware.
static void gpsRailUp() {
    PMU->setPowerChannelVoltage(XPOWERS_ALDO4, 3300);
    PMU->enablePowerOutput(XPOWERS_ALDO4);
    delay(10);
    initGPS();
}

static void radioRailUp() {
    PMU->setPowerChannelVoltage(XPOWERS_ALDO3, 3300);
    PMU->enablePowerOutput(XPOWERS_ALDO3);
    delay(10);
    initRadio();
}

// Light-sleeps the ESP32 for durationMs, or until BUTTON_PIN is pressed,
// whichever comes first -- SOS needs to interrupt the wait immediately
// rather than sit out the rest of a ~60s sleep. esp_light_sleep_start()
// preserves RAM and peripheral state (unlike deep sleep), so there's nothing
// else here that needs saving/restoring around it.
static void lightSleepMs(uint32_t durationMs) {
    esp_sleep_enable_timer_wakeup((uint64_t)durationMs * 1000ULL);

    gpio_wakeup_enable((gpio_num_t)BUTTON_PIN, GPIO_INTR_LOW_LEVEL);
    esp_sleep_enable_gpio_wakeup();

    esp_light_sleep_start();

    gpio_wakeup_disable((gpio_num_t)BUTTON_PIN);
}

static void holdDisplayThenSleep() {
    lightSleepMs(DISPLAY_HOLD_MS);
    displaySleep();
}

// Debounces BUTTON_PIN and reports one confirmed press per press/release
// cycle (BUTTON_PIN uses INPUT_PULLUP, so idle-high/pressed-low). Polled
// rather than interrupt-driven so it fits the same simple model as GPS
// acquisition below; separated from acting on the press so both the
// acquisition loop and the top of loop() can check for one without
// duplicating the toggle logic.
static bool buttonPressed() {
    bool reading = digitalRead(BUTTON_PIN);
    if (reading != lastButtonReading) {
        lastButtonChangeMs = millis();
        lastButtonReading = reading;
    }

    if (millis() - lastButtonChangeMs > BUTTON_DEBOUNCE_MS && reading != buttonState) {
        buttonState = reading;
        return buttonState == LOW;
    }
    return false;
}

static void handleButtonPress() {
    sosActive = !sosActive;
    Serial.println(sosActive ? "SOS button pressed - beacon started" : "SOS button pressed - beacon canceled");
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
        holdDisplayThenSleep();
        return;
    }
    memcpy(plainBuf, payload.c_str(), plainLen);

    uint8_t cipherBuf[kMaxPlaintextLen + HC_GCM_NONCE_LEN + HC_GCM_TAG_LEN];
    size_t cipherLen = hcEncrypt(plainBuf, plainLen, cipherBuf);
    if (cipherLen == 0) {
        Serial.println("  encrypt failed");
        showStatus(type, "Send failed");
        holdDisplayThenSleep();
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
    holdDisplayThenSleep();
}

// Feeds GPS UART bytes to the parser until a fresh fix shows up or budgetMs
// runs out, whichever is first. GPS is powered down between cycles (see
// gpsRailDown()), so this is a cold acquisition every time, not a running
// fix -- buildLocationMessage() falls back to whatever fix (possibly stale,
// or none) TinyGPSPlus still has on hand if this times out.
static void acquireGpsFix(uint32_t budgetMs) {
    uint32_t start = millis();
    while (millis() - start < budgetMs) {
        while (SerialGPS.available() > 0) {
            gps.encode(SerialGPS.read());
        }
        if (gps.location.isValid() && gps.location.age() < 2000) {
            return;
        }
        if (buttonPressed()) {
            handleButtonPress();
            return; // don't make an SOS press wait out the rest of the budget
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
    Serial.println("Hiker's Companion - GPS/LoRa send loop ready");
    holdDisplayThenSleep();
}

// One cycle is: check for a pending SOS press, acquire a GPS fix (bounded),
// send, then gate the GPS/LoRa rails off and light-sleep for whatever's left
// of SEND_INTERVAL_MS -- not a fixed sleep after a fixed acquire, since the
// acquire+send stretch is itself variable and this keeps the overall cycle
// close to SEND_INTERVAL_MS regardless. This is the duty cycle the report's
// 46.93 mA average assumes; the milestone-1 loop this replaced kept the
// CPU, GPS, and radio all fully awake for the entire interval instead.
void loop() {
    uint32_t cycleStartMs = millis();

    if (buttonPressed()) {
        handleButtonPress();
    }

    acquireGpsFix(sosActive ? GPS_ACQUIRE_QUICK_MS : GPS_ACQUIRE_BUDGET_MS);
    sendLocationPacket(sosActive ? "SOS" : "OK");

    gpsRailDown();
    radioRailDown();

    uint32_t elapsedMs = millis() - cycleStartMs;
    uint32_t sleepMs = elapsedMs < SEND_INTERVAL_MS ? SEND_INTERVAL_MS - elapsedMs : 0;
    lightSleepMs(sleepMs);

    gpsRailUp();
    radioRailUp();
}

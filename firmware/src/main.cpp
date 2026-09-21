// Hiker's Companion firmware for the LILYGO T-Beam Supreme (ESP32-S3, SX1262, AXP2101, GNSS).
// Sends GPS location over LoRa on an interval, with SOS button, OLED status, and delivery ACK.
// Between sends the GPS/LoRa rails are cut and the ESP32 light-sleeps (see loop()).

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

#include "ble_provisioning.h"
#include "hc_crypto.h"
#include "pins.h"

// ---- Radio configuration ----
// US902-928 MHz ISM band. SF must match the Heltec receiver.
static constexpr float LORA_FREQUENCY_MHZ = 915.0;
static constexpr float LORA_BANDWIDTH_KHZ = 125.0;
static constexpr uint8_t LORA_SPREADING_FACTOR = 7;
static constexpr uint8_t LORA_CODING_RATE = 5;
static constexpr int8_t LORA_TX_POWER_DBM = 22; // SX1262 max

static constexpr uint32_t SEND_INTERVAL_MS = 60000;

// ACK listen window = ACK airtime at the current SF (computed in waitForAck())
// plus this margin for the receiver's decrypt/print/turnaround.
static constexpr uint32_t ACK_TURNAROUND_MARGIN_MS = 750;

// Longest ACK plaintext is "ACK,65535" (message ids are uint16_t).
static constexpr size_t kMaxAckPlainLen = 9;

// GPIO0 has no hardware debounce.
static constexpr uint32_t BUTTON_DEBOUNCE_MS = 50;

// GPS is cold-started every cycle; 25s is a placeholder pending real TTFF data.
static constexpr uint32_t GPS_ACQUIRE_BUDGET_MS = 25000;

// SOS gets a short window: getting the packet out beats a precise fix.
static constexpr uint32_t GPS_ACQUIRE_QUICK_MS = 2000;

// ---- Globals ----
// Must be typed as the interface: the concrete class makes power control protected.
TwoWire PMUWire = TwoWire(1);
XPowersLibInterface *PMU = nullptr;

HardwareSerial &SerialGPS = Serial1;
TinyGPSPlus gps;

SX1262 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);

static constexpr uint8_t DISPLAY_WIDTH = 128;
static constexpr uint8_t DISPLAY_HEIGHT = 64;

// Two board sub-variants put the OLED at 0x3C or 0x3D, so initDisplay() probes both.
static constexpr uint8_t DISPLAY_I2C_ADDR_CANDIDATES[] = {0x3C, 0x3D};
Adafruit_SH1106G display(DISPLAY_WIDTH, DISPLAY_HEIGHT, &Wire, -1);

// If no display ever responds, every display function is a no-op.
bool displayAvailable = false;
uint8_t displayI2cAddr = DISPLAY_I2C_ADDR_CANDIDATES[0];

uint16_t nextMessageId = 0;

// BUTTON_PIN is INPUT_PULLUP: idle-high, pressed-low.
bool lastButtonReading = HIGH;
bool buttonState = HIGH;
uint32_t lastButtonChangeMs = 0;

// A press starts a standing SOS beacon (every send flagged "SOS"); a second press cancels it.
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

// LoRa, GPS, and the display sit behind AXP2101 rails ALDO3/ALDO4/ALDO1, all off at boot.
// Missing ALDO1 was the cause of a "dead" display: the controller still ACKs I2C
// unpowered, so display.begin() succeeded but nothing lit up.
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

    PMU->setPowerChannelVoltage(XPOWERS_ALDO1, 3300); // Display (+ BME280 + magnetometer) rail
    PMU->enablePowerOutput(XPOWERS_ALDO1);

    // The PWR button is the AXP2101 power key, reported by PMU_IRQ_PIN going low.
    // Only its IRQs are enabled. A long press starts a hardware power-off
    // countdown (4s) that firmware can't abort.
    PMU->disableIRQ(XPOWERS_AXP2101_ALL_IRQ);
    PMU->clearIrqStatus();
    PMU->enableIRQ(XPOWERS_AXP2101_PKEY_SHORT_IRQ | XPOWERS_AXP2101_PKEY_LONG_IRQ);
    PMU->setPowerKeyPressOffTime(XPOWERS_POWEROFF_4S);
    pinMode(PMU_IRQ_PIN, INPUT_PULLUP);
}

static void initGPS() {
    pinMode(GPS_EN_PIN, OUTPUT);
    digitalWrite(GPS_EN_PIN, HIGH);
    SerialGPS.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
}

// Non-fatal: GPS/LoRa/SOS must keep working with a dead display.
static void initDisplay() {
    Wire.begin(I2C_SDA, I2C_SCL);
    for (uint8_t addr : DISPLAY_I2C_ADDR_CANDIDATES) {
        if (display.begin(addr, true)) {
            displayI2cAddr = addr;
            displayAvailable = true;
            display.clearDisplay();
            display.display();
            return;
        }
    }
    Serial.println("Display init failed at 0x3C and 0x3D - continuing without it");
}

// The panel stays off between sends; it wakes for a send and holds through the result.
static constexpr uint32_t DISPLAY_HOLD_MS = 5000;
bool displayOn = false;

static void displayWake() {
    if (!displayAvailable) {
        return;
    }
    if (!displayOn) {
        display.oled_command(SH110X_DISPLAYON);
        displayOn = true;
    }
}

// The ESP32's I2C state doesn't reliably survive light sleep, so the bus and
// controller are re-initialized before each cycle's draw. Non-fatal on failure.
static void reinitDisplay() {
    if (!displayAvailable) {
        return;
    }
    Wire.begin(I2C_SDA, I2C_SCL);
    if (display.begin(displayI2cAddr, true)) {
        displayOn = true; // begin() leaves the OLED on
    } else {
        Serial.println("  display re-init failed, continuing without it");
    }
}

static void displaySleep() {
    if (!displayAvailable) {
        return;
    }
    if (displayOn) {
        display.oled_command(SH110X_DISPLAYOFF);
        displayOn = false;
    }
}

// line1 is small context (message type); line2 is the big status word.
static void showStatus(const char *line1, const char *line2) {
    if (!displayAvailable) {
        return;
    }
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

// The GPS and LoRa rails are cut whenever they have nothing to do.
static void gpsRailDown() {
    digitalWrite(GPS_EN_PIN, LOW); // don't drive EN into an unpowered chip
    PMU->disablePowerOutput(XPOWERS_ALDO4);
}

static void radioRailDown() {
    PMU->disablePowerOutput(XPOWERS_ALDO3);
}

// Both chips lose all state when their rail is cut, so bringing a rail up means a
// full re-init. The 10ms delay is an unmeasured placeholder for rail ramp-up.
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

// Light-sleeps for durationMs, or until the SOS or PWR button is pressed.
static void lightSleepMs(uint32_t durationMs) {
#ifdef DEBUG_NO_SLEEP
    // Debug build: light sleep suspends USB serial, so stay awake and poll the
    // same wake pins instead. Draws more power; not for power measurement.
    uint32_t start = millis();
    while (millis() - start < durationMs) {
        if (digitalRead(BUTTON_PIN) == LOW || digitalRead(PMU_IRQ_PIN) == LOW) {
            return;
        }
        delay(10);
    }
    return;
#endif
    esp_sleep_enable_timer_wakeup((uint64_t)durationMs * 1000ULL);

    gpio_wakeup_enable((gpio_num_t)BUTTON_PIN, GPIO_INTR_LOW_LEVEL);
    gpio_wakeup_enable((gpio_num_t)PMU_IRQ_PIN, GPIO_INTR_LOW_LEVEL);
    esp_sleep_enable_gpio_wakeup();

    esp_light_sleep_start();

    gpio_wakeup_disable((gpio_num_t)BUTTON_PIN);
    gpio_wakeup_disable((gpio_num_t)PMU_IRQ_PIN);
}

static void holdDisplayThenSleep() {
    lightSleepMs(DISPLAY_HOLD_MS);
    displaySleep();
}

// Debounced SOS button: true once per confirmed press.
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

// PMU_IRQ_PIN only goes low for a PWR press (see initPower()), already qualified
// by the AXP2101, so no debounce. clearIrqStatus() must run or the pin stays low.
// A long press can't be aborted, so it shows "Power Off" and parks until the PMU
// cuts power; that applies even while SOS is active.
static bool bluetoothButtonPressed() {
    if (digitalRead(PMU_IRQ_PIN) != LOW) {
        return false;
    }
    PMU->getIrqStatus();
    bool shortPress = PMU->isPekeyShortPressIrq();
    bool longPress = PMU->isPekeyLongPressIrq();
    PMU->clearIrqStatus();

    if (longPress) {
        Serial.println("PWR long-press - shutting down");
        reinitDisplay();
        showStatus("Power", "Off");
        while (true) {
            delay(100);
        }
    }
    return shortPress;
}

// PWR short press enters BLE provisioning, on a different button from SOS on
// purpose. It's ignored while SOS is active: provisioning can block for minutes
// and the beacon must keep sending. Never returns if it enters.
static void checkBluetoothButton() {
    if (!bluetoothButtonPressed()) {
        return;
    }
    if (sosActive) {
        Serial.println("PWR button pressed - ignored, SOS beacon is active");
        return;
    }
    Serial.println("PWR button pressed - entering BLE provisioning");
    runProvisioningMode();
}

// "lat,lng,hdop,age_ms", or "NOFIX,age_ms" if there's no fix yet.
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

// Waits for "ACK,<expectedId>" for the longest ACK's airtime at this SF plus
// ACK_TURNAROUND_MARGIN_MS. radio.receive() blocks and reads the packet itself.
static bool waitForAck(uint16_t expectedId) {
    static constexpr size_t kMaxAckCipherLen = 64;
    uint8_t ackCipherBuf[kMaxAckCipherLen];

    size_t maxAckOnAirBytes = kMaxAckPlainLen + HC_GCM_NONCE_LEN + HC_GCM_TAG_LEN;
    uint32_t ackAirtimeMs = (uint32_t)(radio.getTimeOnAir(maxAckOnAirBytes) / 1000);
    uint32_t ackTimeoutMs = ackAirtimeMs + ACK_TURNAROUND_MARGIN_MS;
    Serial.printf("  ACK window %u ms (ACK airtime %u ms + %u ms margin)\n",
                  (unsigned)ackTimeoutMs, (unsigned)ackAirtimeMs, (unsigned)ACK_TURNAROUND_MARGIN_MS);

    int state = radio.receive(ackCipherBuf, kMaxAckCipherLen, ackTimeoutMs);
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

// Sends an encrypted "<type>,<id>,<location>[,<contacts>]" packet. Type is "OK"
// or "SOS"; id lets waitForAck() match the reply. Contacts ride on every SOS send
// so any single packet is enough; they're dropped if too long, since the
// location must never fail to send.
static void sendLocationPacket(const char *type) {
    reinitDisplay();
    showStatus(type, "Sending...");

    uint16_t msgId = nextMessageId++;
    String payload = String(type) + "," + String(msgId) + "," + buildLocationMessage();

    static constexpr size_t kMaxPlaintextLen = 160;

    if (sosActive) {
        String withContacts = payload + "," + buildContactsForSos();
        if (withContacts.length() <= kMaxPlaintextLen) {
            payload = withContacts;
        } else {
            Serial.println("  contacts too long to fit with location, sending location only");
        }
    }

    Serial.print("TX (plaintext): ");
    Serial.println(payload);

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

// Feeds GPS bytes to the parser until a fresh fix or budgetMs. On timeout,
// buildLocationMessage() uses whatever fix TinyGPSPlus still has.
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
            return; // don't make SOS wait out the budget
        }
        checkBluetoothButton(); // never returns if it enters provisioning
    }
}

void setup() {
    Serial.begin(115200);
    delay(1500); // let USB CDC come up

    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // Must run before the provisioning check: it enables the display rail (ALDO1).
    initPower();

    // Holding BOOT through power-on enters BLE provisioning (never returns).
    if (bootButtonHeldForProvisioning()) {
        runProvisioningMode();
    }

    initGPS();
    initRadio();
    initDisplay();

    displayWake();
    showStatus("Hiker's Companion", "Ready");
    Serial.println("Hiker's Companion - GPS/LoRa send loop ready");
    holdDisplayThenSleep();
}

// One cycle: handle button presses, acquire a fix, send, cut the GPS/LoRa rails,
// then light-sleep for the rest of SEND_INTERVAL_MS. Sleeping only the remainder
// keeps the cycle near that interval despite variable acquire/send time.
void loop() {
    uint32_t cycleStartMs = millis();

    if (buttonPressed()) {
        handleButtonPress();
    }
    checkBluetoothButton(); // never returns if it enters provisioning

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

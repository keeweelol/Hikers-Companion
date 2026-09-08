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

#include "ble_provisioning.h"
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

// LilyGO ships this exact board with two magnetometer sub-variants
// (QMC6310U vs QMC6310N) that put the SH1106 OLED at different I2C
// addresses -- 0x3C or 0x3D respectively (see the T-Beam Supreme hardware
// doc). Two physical units on the bench can genuinely need different
// addresses here; initDisplay() probes both rather than assuming one.
static constexpr uint8_t DISPLAY_I2C_ADDR_CANDIDATES[] = {0x3C, 0x3D};
Adafruit_SH1106G display(DISPLAY_WIDTH, DISPLAY_HEIGHT, &Wire, -1);

// Set once initDisplay() finds a display that actually responds, at
// whichever of the two candidate addresses worked -- reinitDisplay() then
// reuses this instead of re-probing both every cycle. False for the whole
// run if neither address ever responded (no display present or a genuinely
// dead one), in which case every display* function below becomes a no-op
// rather than touching hardware that isn't there.
bool displayAvailable = false;
uint8_t displayI2cAddr = DISPLAY_I2C_ADDR_CANDIDATES[0];

uint16_t nextMessageId = 0;

// BUTTON_PIN uses INPUT_PULLUP, so idle-high/pressed-low.
bool lastButtonReading = HIGH;
bool buttonState = HIGH;
uint32_t lastButtonChangeMs = 0;

// A press starts a standing SOS beacon (every periodic send goes out flagged
// "SOS" instead of "OK") rather than sending just one SOS packet and falling
// back to routine sends. A second press cancels it.
bool sosActive = false;

// Set the moment SOS activates, consumed by the very next SOS send (see
// sendLocationPacket()), then cleared -- so the emergency contact list rides
// along on just that first packet, not every repeat of the standing beacon.
// Left alone (not cleared) by a routine "OK" send, so if SOS gets canceled
// before it ever actually transmits, the contacts are still attached to
// whichever SOS send eventually happens next.
bool sosContactsPending = false;

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

    // The board's physical PWR button isn't a GPIO at all -- it's wired
    // into the AXP2101 as its "power key," which reports presses by
    // pulling PMU_IRQ_PIN low. Disabling every other IRQ source means that
    // pin can only go low for one reason, so bluetoothButtonPressed() can
    // treat "pin is low" as "PWR was short-pressed" without decoding the
    // status register further.
    PMU->disableIRQ(XPOWERS_AXP2101_ALL_IRQ);
    PMU->clearIrqStatus();
    PMU->enableIRQ(XPOWERS_AXP2101_PKEY_SHORT_IRQ);
    pinMode(PMU_IRQ_PIN, INPUT_PULLUP);
}

static void initGPS() {
    pinMode(GPS_EN_PIN, OUTPUT);
    digitalWrite(GPS_EN_PIN, HIGH);
    SerialGPS.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
}

// Non-fatal on failure, unlike initPower()/initRadio() -- GPS/LoRa/SOS must
// keep working even with a dead or undetectable display, since this is a
// safety device first and a status screen second. Tries both candidate I2C
// addresses (see DISPLAY_I2C_ADDR_CANDIDATES above) before giving up.
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

// Panel stays off between transmissions to save power -- it wakes for a
// send and holds through the result via holdDisplayThenSleep() below, which
// light-sleeps for the hold instead of delay()ing so it isn't a needless
// full-power stretch either.
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

// Re-runs the I2C bus + SH1106 controller init before each cycle's display
// writes, reusing whichever address initDisplay() found working at boot
// (displayI2cAddr) rather than re-probing both every cycle. Unlike
// initDisplay() at boot, a failure here doesn't disable anything further --
// it was already non-fatal at boot too, so this just means the display
// stays unavailable for this cycle, same as if it had never been found at
// all. Skipped entirely if no display was ever found (displayAvailable),
// since there's no known address to retry and no point probing hardware
// that was already confirmed not to be there. This exists because the
// display's rail is never gated (only ALDO3/ALDO4 are), but the ESP32's own
// I2C peripheral state isn't guaranteed to survive esp_light_sleep_start()
// unrestored -- plain displayWake() alone left the screen dead after the
// first light sleep even though the controller itself was never powered
// down.
static void reinitDisplay() {
    if (!displayAvailable) {
        return;
    }
    Wire.begin(I2C_SDA, I2C_SCL);
    if (display.begin(displayI2cAddr, true)) {
        displayOn = true; // begin() unconditionally leaves the OLED powered on
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

// line2 is the big/important word ("Sending", "Delivered", "Send failed");
// line1 is the small context above it (message type, or a boot message).
// No ACK yet (see FW-06 in the backlog), so "Delivered" here means "the
// radio call returned success," not "a human received this." A no-op if no
// display was ever found -- see displayAvailable.
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

// Light-sleeps the ESP32 for durationMs, or until BUTTON_PIN (SOS) or
// PMU_IRQ_PIN (the PWR/Bluetooth button) is pressed, whichever comes first --
// both need to interrupt the wait immediately rather than sit out the rest
// of a ~60s sleep. esp_light_sleep_start() preserves RAM and peripheral
// state (unlike deep sleep), so there's nothing else here that needs
// saving/restoring around it.
static void lightSleepMs(uint32_t durationMs) {
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
    if (sosActive) {
        sosContactsPending = true; // only the next SOS send carries the stored contacts
    }
    Serial.println(sosActive ? "SOS button pressed - beacon started" : "SOS button pressed - beacon canceled");
}

// PMU_IRQ_PIN only ever goes low for a PWR short-press -- see initPower(),
// which disables every other AXP2101 IRQ source -- so this is a plain level
// read, no debounce needed the way BUTTON_PIN needs one: the AXP2101 has
// already qualified the press itself before raising the IRQ. clearIrqStatus()
// releases the line back high; skipping it would leave PMU_IRQ_PIN stuck low
// and this function permanently "pressed."
static bool bluetoothButtonPressed() {
    if (digitalRead(PMU_IRQ_PIN) != LOW) {
        return false;
    }
    PMU->getIrqStatus();
    bool shortPress = PMU->isPekeyShortPressIrq();
    PMU->clearIrqStatus();
    return shortPress;
}

// A separate physical button from SOS (BUTTON_PIN) on purpose, so pulling up
// contacts on the fly can never be mistaken for -- or interfere with -- the
// emergency button. Entering provisioning blocks the loop for potentially
// minutes (until a phone connects and finishes, or the idle timeout), so a
// press is flatly ignored whenever the SOS beacon is active rather than
// queued for later -- an active SOS beacon must keep beaconing every cycle,
// not go quiet while someone edits contacts. Never returns if it does enter,
// since runProvisioningMode() itself only returns via esp_restart().
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

// Encrypts and transmits a "<type>,<id>,<location>[,<contacts>]" packet,
// where type is "OK" for a routine poll or "SOS" for a button-triggered
// emergency send, and id lets waitForAck() match a reply to this specific
// send. The trailing <contacts> field (see buildContactsForSos() in
// ble_provisioning.cpp) is appended only once -- on the first send after SOS
// activates (sosContactsPending) -- not on every repeat of a standing SOS
// beacon, so responders learn who to notify without re-sending that data on
// every ~60s heartbeat. If it wouldn't fit alongside the location, it's
// dropped and the location-only packet still goes out: the location itself
// must never fail to send just because someone's stored contact names ran
// long.
static void sendLocationPacket(const char *type) {
    reinitDisplay();
    showStatus(type, "Sending...");

    uint16_t msgId = nextMessageId++;
    String payload = String(type) + "," + String(msgId) + "," + buildLocationMessage();

    static constexpr size_t kMaxPlaintextLen = 160;

    if (sosActive && sosContactsPending) {
        String withContacts = payload + "," + buildContactsForSos();
        if (withContacts.length() <= kMaxPlaintextLen) {
            payload = withContacts;
        } else {
            Serial.println("  contacts too long to fit with location, sending location only");
        }
        sosContactsPending = false;
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
        checkBluetoothButton(); // never returns if it enters provisioning
    }
}

void setup() {
    Serial.begin(115200);
    delay(1500); // let USB CDC come up before first prints

    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // Checked before anything else touches power rails or radios: holding
    // BOOT through power-on is the only way into BLE provisioning (FW-10),
    // so normal operation never has BLE running and doesn't touch the
    // sleep/power budget FW-12 was built around. runProvisioningMode()
    // itself never returns -- it esp_restart()s when done.
    if (bootButtonHeldForProvisioning()) {
        runProvisioningMode();
    }

    initPower();
    initGPS();
    initRadio();
    initDisplay();

    displayWake();
    showStatus("Hiker's Companion", "Ready");
    Serial.println("Hiker's Companion - GPS/LoRa send loop ready");
    holdDisplayThenSleep();
}

// One cycle is: check for a pending SOS press, check for a pending
// Bluetooth-button press (see checkBluetoothButton() -- ignored outright
// while SOS is active, otherwise this is where a routine "OK" cycle gets
// interrupted for on-the-fly contact editing), acquire a GPS fix (bounded),
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

// Hiker's Companion — Milestone 1 firmware
//
// Reads GPS location and transmits it over LoRa on a fixed interval.
// No button/SOS logic or display yet — this exists to prove the GPS -> LoRa
// path works end to end on real hardware before building the full SOS flow.
//
// Target: LILYGO T-Beam Supreme (ESP32-S3, SX1262, AXP2101, GNSS)

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>
#include <TinyGPS++.h>
#include <XPowersLib.h>

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

uint32_t lastSendMs = 0;

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

void setup() {
    Serial.begin(115200);
    delay(1500); // let USB CDC come up before first prints

    initPower();
    initGPS();
    initRadio();

    Serial.println("Hiker's Companion - GPS/LoRa send loop ready");
}

void loop() {
    while (SerialGPS.available() > 0) {
        gps.encode(SerialGPS.read());
    }

    uint32_t now = millis();
    if (now - lastSendMs >= SEND_INTERVAL_MS) {
        lastSendMs = now;

        String payload = buildLocationMessage();
        Serial.print("TX: ");
        Serial.println(payload);

        int state = radio.transmit(payload);
        if (state == RADIOLIB_ERR_NONE) {
            Serial.println("  sent ok");
        } else {
            Serial.printf("  send failed, code %d\n", state);
        }
    }
}

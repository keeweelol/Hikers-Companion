#include "ble_provisioning.h"

#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <Wire.h>
#include <esp_system.h>

#include "pins.h"

namespace {

constexpr uint32_t PROVISION_HOLD_MS = 2000;

// Stop advertising if no phone ever connects (e.g. BOOT held by accident).
constexpr uint32_t PROVISION_IDLE_TIMEOUT_MS = 180000;

constexpr uint8_t MAX_CONTACTS = 2;

// Built from MAX_CONTACTS so prompts can't go stale when the count changes.
String slotRange() {
    return "0-" + String(MAX_CONTACTS - 1);
}

// Nordic UART Service UUIDs: terminal-style BLE apps auto-detect it as a serial
// link. RX is what the phone writes (commands in), TX is what it receives.
constexpr char SERVICE_UUID[] = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr char RX_CHAR_UUID[] = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr char TX_CHAR_UUID[] = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

constexpr uint8_t DISPLAY_WIDTH = 128;
constexpr uint8_t DISPLAY_HEIGHT = 64;

// Two board sub-variants put the OLED at 0x3C or 0x3D; probe both.
constexpr uint8_t DISPLAY_I2C_ADDR_CANDIDATES[] = {0x3C, 0x3D};

// Pairing is unauthenticated, like the baked-in LoRa PSK: fine for team-only
// bench hardware. Anyone in BLE range during provisioning could read or overwrite
// contacts. Add NimBLE passkey/bonding before this leaves the bench.
Preferences prefs;
NimBLECharacteristic *txChar = nullptr;
volatile bool clientConnected = false;
volatile bool provisioningDone = false;

// Separate NVS keys per slot, so a write to one can't corrupt another on power loss.
String contactKey(const char *field, uint8_t slot) {
    return String(field) + String(slot);
}

String contactName(uint8_t slot) {
    return prefs.getString(contactKey("name", slot).c_str(), "");
}

String contactPhone(uint8_t slot) {
    return prefs.getString(contactKey("phone", slot).c_str(), "");
}

void storeContact(uint8_t slot, const String &name, const String &phone) {
    prefs.putString(contactKey("name", slot).c_str(), name);
    prefs.putString(contactKey("phone", slot).c_str(), phone);
}

void clearContacts() {
    for (uint8_t i = 0; i < MAX_CONTACTS; i++) {
        storeContact(i, "", "");
    }
}

// "<slot>:<name>:<phone>" per line, one line per slot.
String buildContactsSummary() {
    String out;
    for (uint8_t i = 0; i < MAX_CONTACTS; i++) {
        out += String(i) + ":" + contactName(i) + ":" + contactPhone(i) + "\n";
    }
    return out;
}

// notify() is a no-op until a client subscribes to TX.
void sendLine(const String &line) {
    if (txChar == nullptr) {
        return;
    }
    // Pass the String itself, not .c_str(): NimBLE's setValue() template would
    // treat a bare const char* as a pointer and send the pointer's bytes instead
    // of the text.
    String withNewline = line + "\n";
    txChar->setValue(withNewline);
    txChar->notify();
}

// Commands arrive one per write over RX:
//   <slot 0-1>|<name>|<phone>   store a contact
//   LIST                        reply with all slots
//   CLEAR                       wipe all slots
//   DONE                        end the session
// Anything else gets an ERR reply, since a human types these by hand.
class RxCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *chr) override {
        String value = String(chr->getValue().c_str());
        // Logged for every command, since LIST and errors only reply over BLE.
        Serial.printf("  provisioning: RX %d bytes: \"%s\"\n", value.length(), value.c_str());
        while (value.endsWith("\r") || value.endsWith("\n")) {
            value.remove(value.length() - 1);
        }
        if (value.length() == 0) {
            return;
        }

        if (value == "LIST") {
            sendLine(buildContactsSummary());
            return;
        }
        if (value == "CLEAR") {
            clearContacts();
            sendLine("OK: cleared all contacts");
            return;
        }
        if (value == "DONE") {
            Serial.println("  provisioning: client signaled done");
            sendLine("Bye");
            provisioningDone = true;
            return;
        }

        int firstBar = value.indexOf('|');
        int secondBar = firstBar < 0 ? -1 : value.indexOf('|', firstBar + 1);
        if (firstBar < 0 || secondBar < 0) {
            sendLine("ERR: expected <slot " + slotRange() + ">|<name>|<phone>, LIST, CLEAR, or DONE");
            return;
        }

        int slot = value.substring(0, firstBar).toInt();
        if (slot < 0 || slot >= MAX_CONTACTS) {
            sendLine("ERR: slot must be " + slotRange());
            return;
        }

        String name = value.substring(firstBar + 1, secondBar);
        String phone = value.substring(secondBar + 1);
        storeContact((uint8_t)slot, name, phone);
        Serial.printf("  provisioning: stored contact %d\n", slot);
        sendLine("OK: stored slot " + String(slot));
    }
};

// One session per boot-hold: any disconnect ends it rather than re-advertising.
class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer *server, ble_gap_conn_desc *desc) override {
        (void)server;
        (void)desc;
        clientConnected = true;
        Serial.println("  provisioning: phone connected");
        // The greeting is sent from TxCallbacks::onSubscribe(): subscribing is a
        // separate GATT write that happens after this event.
    }
    void onDisconnect(NimBLEServer *server, ble_gap_conn_desc *desc) override {
        (void)server;
        (void)desc;
        Serial.println("  provisioning: phone disconnected");
        provisioningDone = true;
    }
};

// The greeting must go out here: notify() drops silently until a client subscribes.
class TxCallbacks : public NimBLECharacteristicCallbacks {
    void onSubscribe(NimBLECharacteristic *chr, ble_gap_conn_desc *desc, uint16_t subValue) override {
        (void)chr;
        (void)desc;
        if (subValue == 0) {
            return; // client unsubscribed
        }
        sendLine("Hiker's Companion provisioning. Commands: <slot " + slotRange() + ">|<name>|<phone>, LIST, CLEAR, DONE");
    }
};

void showProvisioningStatus(Adafruit_SH1106G &display, const char *line1, const char *line2) {
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

} // namespace

bool bootButtonHeldForProvisioning() {
    uint32_t start = millis();
    while (digitalRead(BUTTON_PIN) == LOW) {
        if (millis() - start >= PROVISION_HOLD_MS) {
            return true;
        }
    }
    return false;
}

void runProvisioningMode() {
    Serial.println("Entering BLE provisioning mode (FW-10)");

    // This mode has its own display init, separate from main.cpp's sleep-cycle one.
    Wire.begin(I2C_SDA, I2C_SCL);
    Adafruit_SH1106G display(DISPLAY_WIDTH, DISPLAY_HEIGHT, &Wire, -1);
    bool hasDisplay = false;
    for (uint8_t addr : DISPLAY_I2C_ADDR_CANDIDATES) {
        if (display.begin(addr, true)) {
            hasDisplay = true;
            break;
        }
    }
    if (hasDisplay) {
        display.clearDisplay();
        display.display();
        showProvisioningStatus(display, "Bluetooth", "No Device");
    }

    prefs.begin("hc_contacts", false);

    char deviceName[32];
    uint64_t mac = ESP.getEfuseMac();
    snprintf(deviceName, sizeof(deviceName), "HikerComp-%04X", (unsigned)(mac & 0xFFFF));

    NimBLEDevice::init(deviceName);
    NimBLEServer *server = NimBLEDevice::createServer();
    server->setCallbacks(new ServerCallbacks());

    NimBLEService *service = server->createService(SERVICE_UUID);

    NimBLECharacteristic *rxChar = service->createCharacteristic(
        RX_CHAR_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    rxChar->setCallbacks(new RxCallbacks());

    // READ as well as NOTIFY so a raw GATT explorer can pull the last reply.
    txChar = service->createCharacteristic(TX_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    txChar->setCallbacks(new TxCallbacks());

    service->start();

    NimBLEAdvertising *advertising = NimBLEDevice::getAdvertising();
    advertising->addServiceUUID(SERVICE_UUID);
    advertising->start();

    Serial.printf("  advertising as %s (Nordic UART Service)\n", deviceName);
    Serial.printf("  commands: \"<slot %s>|<name>|<phone>\", \"LIST\", \"CLEAR\", \"DONE\"\n", slotRange().c_str());

    // Connect events fire on NimBLE's task; poll here so display writes stay on one task.
    bool shownConnected = false;
    uint32_t idleStartMs = millis();
    while (!provisioningDone) {
        if (hasDisplay && clientConnected != shownConnected) {
            shownConnected = clientConnected;
            showProvisioningStatus(display, "Bluetooth", shownConnected ? "Connected" : "No Device");
        }
        if (!clientConnected && millis() - idleStartMs > PROVISION_IDLE_TIMEOUT_MS) {
            Serial.println("  provisioning: idle timeout, no phone ever connected");
            break;
        }
        if (digitalRead(BUTTON_PIN) == LOW) {
            Serial.println("  provisioning: BOOT pressed, exiting");
            break;
        }
        delay(50);
    }

    if (hasDisplay) {
        showProvisioningStatus(display, "Bluetooth", "Done");
        delay(1000);
    }

    Serial.println("Exiting provisioning mode, restarting into normal firmware");
    Serial.flush();
    esp_restart();
}

String buildContactsForSos() {
    // Opens and closes the namespace itself: callable mid-SOS with no session running.
    prefs.begin("hc_contacts", true); // read-only
    String out;
    for (uint8_t i = 0; i < MAX_CONTACTS; i++) {
        String name = contactName(i);
        String phone = contactPhone(i);
        if (name.length() == 0 && phone.length() == 0) {
            continue; // skip empty slots to keep the packet small
        }
        if (out.length() > 0) {
            out += ";";
        }
        out += name + ":" + phone;
    }
    prefs.end();
    return out;
}

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

// No phone ever connecting shouldn't leave the device stuck advertising
// (and burning battery) indefinitely just because someone held BOOT by
// accident on power-up.
constexpr uint32_t PROVISION_IDLE_TIMEOUT_MS = 180000;

constexpr uint8_t MAX_CONTACTS = 3;

// Nordic UART Service (NUS) UUIDs -- a de facto standard that terminal-style
// BLE apps (Serial Bluetooth Terminal, nRF Toolbox/Connect's UART preset,
// Adafruit Bluefruit Connect, etc.) auto-detect and treat as a plain serial
// link, rather than requiring the user to browse a raw GATT table for a
// one-off custom service. RX is what the phone writes to (commands in);
// TX is what the device notifies on (responses out) -- named from the
// phone's perspective, matching the spec.
constexpr char SERVICE_UUID[] = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr char RX_CHAR_UUID[] = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr char TX_CHAR_UUID[] = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

constexpr uint8_t DISPLAY_WIDTH = 128;
constexpr uint8_t DISPLAY_HEIGHT = 64;
constexpr uint8_t DISPLAY_I2C_ADDR = 0x3C;

// Unauthenticated on purpose for now, same tradeoff as the LoRa PSK in
// shared/hc_crypto.h being baked into firmware: this is a feasibility MVP
// with hardware only the team controls, not a fielded product. Anyone in
// BLE range while a unit is in provisioning mode (BOOT held at power-on
// only, not the normal operating mode) could read or overwrite its contact
// list. Revisit with NimBLE's passkey/bonding support before this ever
// leaves the bench.
Preferences prefs;
NimBLECharacteristic *txChar = nullptr;
volatile bool clientConnected = false;
volatile bool provisioningDone = false;

// Contacts are stored as separate "name<N>"/"phone<N>" NVS keys rather than
// one blob, so a write to one slot can't corrupt the others if power is
// lost mid-write.
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

// notify() is a safe no-op if nobody's subscribed to TX yet (checked
// internally by NimBLE), so callers don't need to track subscription state
// themselves -- worst case an early reply is silently dropped.
void sendLine(const String &line) {
    if (txChar == nullptr) {
        return;
    }
    // Must pass the String itself, not withNewline.c_str() -- NimBLE's
    // setValue() is a template that dispatches on whether the argument type
    // has c_str()/length() methods. A bare `const char*` doesn't (it's just
    // a pointer), so that call silently took the *other* branch and copied
    // sizeof(pointer) bytes of the pointer's own address as the notify
    // payload instead of the string's contents -- a phone would see a
    // handful of garbage-looking bytes on every reply, never the actual
    // text. Passing the String directly lets it correctly resolve to the
    // c_str()/length() branch.
    String withNewline = line + "\n";
    txChar->setValue(withNewline);
    txChar->notify();
}

// Command protocol over RX, one command per write (matches how terminal
// apps send a line at a time):
//   <slot 0-2>|<name>|<phone>   store a contact
//   LIST                        reply with all 3 slots
//   CLEAR                       wipe all 3 slots
//   DONE                        end the provisioning session
// Anything else gets an ERR reply on TX rather than being silently dropped,
// since a human is typing these by hand.
class RxCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *chr) override {
        String value = String(chr->getValue().c_str());
        // Unconditional, regardless of which command this turns out to be --
        // LIST and error replies only ever answer over BLE (see sendLine()
        // calls below), so without this line there was no serial evidence at
        // all that a write reached the device for those commands.
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
            sendLine("ERR: expected <slot 0-2>|<name>|<phone>, LIST, CLEAR, or DONE");
            return;
        }

        int slot = value.substring(0, firstBar).toInt();
        if (slot < 0 || slot >= MAX_CONTACTS) {
            sendLine("ERR: slot must be 0-" + String(MAX_CONTACTS - 1));
            return;
        }

        String name = value.substring(firstBar + 1, secondBar);
        String phone = value.substring(secondBar + 1);
        storeContact((uint8_t)slot, name, phone);
        Serial.printf("  provisioning: stored contact %d\n", slot);
        sendLine("OK: stored slot " + String(slot));
    }
};

// One provisioning session per boot-hold: a disconnect (whether the phone
// finished on purpose or just dropped the link) ends the session rather
// than going back to advertising, so the device doesn't sit around
// re-advertising indefinitely after whoever provisioned it walks away.
class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer *server, ble_gap_conn_desc *desc) override {
        (void)server;
        (void)desc;
        clientConnected = true;
        Serial.println("  provisioning: phone connected");
        // The greeting itself is sent from TxCallbacks::onSubscribe() below,
        // not here -- notify() silently drops if nobody's subscribed yet,
        // and subscribing to TX is a separate GATT write the client only
        // sends after this connect event, not part of it.
    }
    void onDisconnect(NimBLEServer *server, ble_gap_conn_desc *desc) override {
        (void)server;
        (void)desc;
        Serial.println("  provisioning: phone disconnected");
        provisioningDone = true;
    }
};

// notify() silently no-ops until a client subscribes to TX, so the greeting
// has to be sent from here -- the moment a subscription actually lands --
// rather than from ServerCallbacks::onConnect(), which fires before that
// subscription exists and would otherwise drop the greeting every time.
class TxCallbacks : public NimBLECharacteristicCallbacks {
    void onSubscribe(NimBLECharacteristic *chr, ble_gap_conn_desc *desc, uint16_t subValue) override {
        (void)chr;
        (void)desc;
        if (subValue == 0) {
            return; // client unsubscribed, nothing to greet
        }
        sendLine("Hiker's Companion provisioning. Commands: <slot 0-2>|<name>|<phone>, LIST, CLEAR, DONE");
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

    // This mode never touches GPS/LoRa, so it gets its own tiny display
    // init rather than sharing main.cpp's initDisplay()/reinitDisplay(),
    // which are wired into the normal-operation sleep cycle.
    Wire.begin(I2C_SDA, I2C_SCL);
    Adafruit_SH1106G display(DISPLAY_WIDTH, DISPLAY_HEIGHT, &Wire, -1);
    bool hasDisplay = display.begin(DISPLAY_I2C_ADDR, true);
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

    // READ alongside NOTIFY so a raw GATT explorer (nRF Connect, LightBlue)
    // can still manually pull the last response without subscribing, even
    // though the intended clients are terminal apps that auto-subscribe.
    txChar = service->createCharacteristic(TX_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    txChar->setCallbacks(new TxCallbacks());

    service->start();

    NimBLEAdvertising *advertising = NimBLEDevice::getAdvertising();
    advertising->addServiceUUID(SERVICE_UUID);
    advertising->start();

    Serial.printf("  advertising as %s (Nordic UART Service)\n", deviceName);
    Serial.println("  commands: \"<slot 0-2>|<name>|<phone>\", \"LIST\", \"CLEAR\", \"DONE\"");

    // The connect/disconnect events themselves fire from NimBLE's own host
    // task, not this loop -- polling clientConnected here and drawing from
    // this task keeps every display write on one thread, same as the rest
    // of the firmware.
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
    // Preferences::begin()/end() bracket a single open/read/close, since
    // nothing else keeps this namespace open outside of an active
    // provisioning session -- this can be called mid-SOS with no session
    // running at all.
    prefs.begin("hc_contacts", true); // read-only
    String out;
    for (uint8_t i = 0; i < MAX_CONTACTS; i++) {
        String name = contactName(i);
        String phone = contactPhone(i);
        if (name.length() == 0 && phone.length() == 0) {
            continue; // skip empty slots -- keep the packet small when few/no contacts are set
        }
        if (out.length() > 0) {
            out += ";";
        }
        out += name + ":" + phone;
    }
    prefs.end();
    return out;
}

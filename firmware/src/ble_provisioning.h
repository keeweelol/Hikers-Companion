#pragma once

#include <WString.h>

// Emergency-contact BLE provisioning (FW-10). Entered by holding BOOT through
// power-on, or by a short PWR press during normal operation (ignored while SOS
// is active). BLE only runs inside runProvisioningMode().

// True if BUTTON_PIN is held down for PROVISION_HOLD_MS. Call at the top of
// setup(), while the pin still just reads the physical button.
bool bootButtonHeldForProvisioning();

// Runs a BLE server for editing the contact list in NVS, then esp_restart()s.
// Ends on "DONE", a disconnect, a BOOT press, or the idle timeout. Never returns.
[[noreturn]] void runProvisioningMode();

// "name:phone;name:phone;..." of stored contacts (empty slots skipped), sent in
// every SOS packet. Empty string if none are stored. Safe to call outside a session.
String buildContactsForSos();

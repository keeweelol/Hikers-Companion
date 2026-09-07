#pragma once

#include <WString.h>

// Emergency-contact BLE provisioning mode (FW-10). Two ways in: holding
// BOOT through power-on (bootButtonHeldForProvisioning(), checked once at
// the top of setup()), or short-pressing the separate PWR button during
// normal operation (checkBluetoothButton() in main.cpp, checked every cycle
// but ignored outright while the SOS beacon is active). Either way, the BLE
// radio only ever runs inside runProvisioningMode() itself -- normal
// operation doesn't touch the sleep/power budget FW-12 was built around.

// True if BUTTON_PIN has been held down continuously for PROVISION_HOLD_MS.
// Must be called at the very top of setup(), before initPower()/initGPS()/
// initRadio() touch anything else, while the pin still just reads the
// physical button with nothing else driving it.
bool bootButtonHeldForProvisioning();

// Runs a BLE GATT server that lets a phone read/write the emergency contact
// list stored in NVS, blocking until provisioning ends -- the client writes
// "DONE", the phone disconnects after a session, BOOT is pressed again, or
// PROVISION_IDLE_TIMEOUT_MS passes with no connection ever made -- then
// esp_restart()s back into normal firmware. Never returns.
[[noreturn]] void runProvisioningMode();

// Compact "name:phone;name:phone;..." summary of the stored emergency
// contacts, empty slots skipped, for embedding in an SOS packet's first
// transmission (see sosContactsPending in main.cpp). Opens, reads, and
// closes NVS itself in one call, so it's safe to call from normal
// operation with no provisioning session active. Empty string if no
// contacts have ever been stored.
String buildContactsForSos();

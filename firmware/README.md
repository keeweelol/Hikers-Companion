# Hiker's Companion Firmware

Custom PlatformIO/Arduino firmware for the LILYGO T-Beam Supreme
(ESP32-S3 + SX1262 LoRa + AXP2101 PMU + GNSS).

This is separate from `../Lora Source/`, which is LilyGO's own vendored
example repo — kept only as pinout/API reference, not built directly.

Also separate: `../heltec-rx-test/`, a throwaway RX bench rig on a Heltec
WiFi LoRa 32 V3 used to confirm this board's LoRa TX is actually reaching a
second radio — decrypted payload + RSSI/SNR go to both serial and the
Heltec's onboard SSD1306 OLED, which wakes for 5s on each received signal
and sleeps otherwise. It also replies with an `ACK,<id>` for every valid
packet it decrypts, which is what makes the T-Beam's "Delivered" status
real (see FW-06 below). Not part of the product firmware.

## Test hardware on hand

2x LILYGO T-Beam Supreme, 2x Heltec WiFi LoRa 32 V3. Enough to test beyond a
single sender/receiver pair once we get there — e.g. two T-Beam senders into
one gateway, or a relay hop — not just the current one-to-one link test.

## Status: Milestone 1 — GPS + LoRa send loop, with SOS button and status display

Reads GPS fixes and transmits location over LoRa on a timer. The onboard
BOOT button (GPIO0) toggles a standing SOS beacon: pressing it sends an
immediate "SOS"-flagged packet and switches every subsequent periodic send
to "SOS" too (instead of the routine "OK") until pressed again to cancel —
see `sendLocationPacket()` / `buttonPressed()` in `src/main.cpp`.

An SH1106 OLED (I2C, pins 17/18) shows live status via `showStatus()`:
message type on top, "Sending" / "Delivered" / "No ACK" / "Send failed"
below. "Delivered" is now a real ACK, not an assumption: each packet carries
a small message id (`<type>,<id>,<location>`), and after transmitting, the
T-Beam listens for up to `ACK_TIMEOUT_MS` (2s) for a matching `ACK,<id>`
reply from the Heltec RX rig (`waitForAck()`/`sendAck()`). A timeout with no
reply shows "No ACK" — distinct from "Send failed," which means the radio
call itself errored, not that nobody answered.

The panel stays off between transmissions to save power — it wakes for a
send and holds for `DISPLAY_HOLD_MS` (5s) after the result, then sleeps
again (`displayWake()`/`displaySleep()`).

## Power

Between sends, the LoRa and GPS PMU rails (ALDO3/ALDO4) are cut and the
ESP32 goes into light sleep, instead of both rails and the CPU staying fully
awake for the whole `SEND_INTERVAL_MS` interval — this duty cycle is what
the report's 46.93 mA average current draw is based on. Each cycle in
`loop()`:

1. Check for a pending SOS press (`buttonPressed()`/`handleButtonPress()`).
2. Power the GPS/LoRa rails back on and acquire a fix (`acquireGpsFix()`,
   bounded by `GPS_ACQUIRE_BUDGET_MS` for a routine poll or the much shorter
   `GPS_ACQUIRE_QUICK_MS` for an SOS press — a fresh fix isn't worth making
   an emergency send wait).
3. Send (`sendLocationPacket()`).
4. Gate the rails back off (`gpsRailDown()`/`radioRailDown()`) and light-sleep
   for whatever's left of the ~60s interval (`lightSleepMs()`).

The SX1262 and the GPS module both lose all internal state when their rail
is cut, so `gpsRailUp()`/`radioRailUp()` do a full re-init (`initGPS()`/
`initRadio()`), not just a PMU output toggle. The sleep also arms a GPIO
wakeup on the SOS button (level-triggered, since ESP32-S3 light sleep GPIO
wakeup only supports level triggers) so a press doesn't have to wait out the
rest of a ~60s sleep.

## BLE provisioning (FW-10)

Two ways in:

- **At power-on**: holding BOOT through boot (2s, `PROVISION_HOLD_MS`) skips
  the normal GPS/LoRa loop entirely and boots straight into a standalone
  provisioning mode (`bootButtonHeldForProvisioning()` in `main.cpp`).
- **On the fly, during normal operation**: short-pressing the board's
  physical PWR button (`checkBluetoothButton()`/`bluetoothButtonPressed()`
  in `main.cpp`) drops straight into provisioning mid-loop, without a
  reboot. PWR isn't a GPIO on this board — it's wired into the AXP2101 PMU
  as its "power key," reported via `PMU_IRQ_PIN` going low — so it's a
  physically separate button from BOOT/`BUTTON_PIN` (SOS) on purpose: a
  contact-editing press can never be mistaken for, or interfere with, the
  emergency button. **Ignored outright whenever the SOS beacon is active**
  (`sosActive`) rather than queued — entering provisioning can block the
  loop for minutes, and an active SOS beacon has to keep beaconing every
  cycle, not go quiet while someone edits contacts. When SOS isn't active,
  a PWR press is checked at the top of `loop()`, inside `acquireGpsFix()`'s
  polling loop, and as a light-sleep wakeup source, so it can interrupt a
  routine "OK" cycle at any point except the ~2s send/ACK-wait window
  itself.

Either way, `runProvisioningMode()` never returns — normal operation never
runs BLE, so it doesn't touch the sleep/power budget the Power section
above depends on.

In provisioning mode the device advertises as `HikerComp-XXXX` (last 4 hex
digits of its MAC, so multiple units on the bench are distinguishable)
implementing the Nordic UART Service (NUS) — the de facto standard a
terminal-style BLE app (Serial Bluetooth Terminal, nRF Connect's UART
preset, Adafruit Bluefruit Connect, etc.) auto-detects and treats as a
plain serial link, so provisioning is "type a command, see a reply" rather
than hand-browsing a raw GATT table. No dedicated companion app exists yet
(see FW-11).

The OLED reflects connection state throughout — "Bluetooth / No Device"
while advertising, flipping to "Bluetooth / Connected" as soon as a phone
connects (polled from `clientConnected` in `runProvisioningMode()`'s loop
rather than drawn from inside the NimBLE connect/disconnect callbacks
themselves, so every display write stays on one task). Unlike normal
operation, the display is never put to sleep in this mode — it stays lit
for the whole session, since provisioning is a short, deliberate, plugged-in
bench activity rather than something to optimize for battery life.

One command per line on the RX characteristic, one text reply per line back
over TX (`sendLine()`/`RxCallbacks::onWrite()` in `src/ble_provisioning.cpp`):

- `<slot 0-2>|<name>|<phone>` — store one of up to 3 emergency contacts.
  Replies `OK: stored slot <n>`.
- `LIST` — replies with all 3 slots as `<slot>:<name>:<phone>` lines,
  confirming what's actually stored.
- `CLEAR` — wipes all 3 slots.
- `DONE` — ends the session immediately instead of waiting on a disconnect.
- Anything else gets an `ERR: ...` reply rather than being silently dropped,
  since a human is typing these by hand.

Contacts are backed by NVS (`Preferences`, namespace `hc_contacts`) so they
survive a reboot; each slot is its own pair of keys so a write to one can't
corrupt another.

A session also ends on a phone disconnect (one provisioning session per
boot-hold — walking away doesn't leave the device re-advertising), on a
second BOOT press, or after `PROVISION_IDLE_TIMEOUT_MS` (3 min) if no phone
ever connects. Either way `runProvisioningMode()` finishes with
`esp_restart()` back into normal firmware — there's no in-place return path.

**Not implemented:** pairing is unauthenticated (BLE "Just Works", no
passkey/bonding) — same simple-for-now tradeoff as the LoRa PSK being baked
into firmware, acceptable for bench hardware the team controls but not
before this leaves the bench.

### Contacts over LoRa

The stored contacts do go out over LoRa now, but only once per SOS
activation, not on every repeat. Pressing SOS sets `sosContactsPending`
(`main.cpp`); the very next `sendLocationPacket()` call appends
`,<buildContactsForSos()>` — a compact `name:phone;name:phone;...` summary,
empty slots skipped — to that one packet and clears the flag, so every
later repeat of a standing SOS beacon goes back to being location-only.
Canceling SOS before it ever actually transmits leaves the flag set rather
than clearing it, so contacts still ride along on whichever SOS send
eventually happens next.

This is a deliberate one-shot, not a bug: sending the full contact list on
every ~60s beacon repeat would waste airtime on data that essentially never
changes mid-emergency, when responders only need to learn it once. If the
combined location+contacts string would overflow the crypto buffer
(`kMaxPlaintextLen`, now 160 bytes — raised from the location-only packet's
64 specifically to fit this), the contacts are dropped for that send and
the location goes out alone instead — the location must never fail to send
just because someone's stored contact names ran long.

Raising `kMaxPlaintextLen` also raises the ciphertext size on the air
(`+ HC_GCM_NONCE_LEN + HC_GCM_TAG_LEN` = up to 188 bytes now, versus 92
before), which is why `heltec-rx-test/src/main.cpp`'s own receive buffer
(`kMaxCipherLen`) had to move from 128 to 192 alongside it — a packet larger
than the receiver's buffer gets silently dropped as "bad packet length"
rather than received. The two constants have to be kept in sync by hand;
there's no shared source of truth between the two PlatformIO projects for
this.

## Build / flash

```sh
python3 -m venv .venv
.venv/bin/pip install platformio
.venv/bin/pio run                 # compile
.venv/bin/pio run -t upload       # flash (board connected via USB-C)
.venv/bin/pio device monitor      # serial log at 115200 baud
```

## Config knobs (top of `src/main.cpp`)

- `LORA_FREQUENCY_MHZ` — 915 MHz (US915 ISM band)
- `LORA_SPREADING_FACTOR` — currently SF10, placeholder until range testing
  picks a value from the report's SF7-SF12 tradeoff space
- `SEND_INTERVAL_MS` — currently 60s, matches the flow chart's "poll every
  minute" behavior

## Security

LoRa payloads are encrypted with AES-128-GCM before transmission — GCM gives
both confidentiality and an authentication tag, so a receiver can tell a
tampered/spoofed packet from a real one, not just decrypt it. This matters
once the main/gateway node forwards messages to emergency services: a forged
SOS or forged "I'm fine" location needs to fail the auth check, not be
trusted just because it decrypts.

The 128-bit key is pre-shared and baked directly into firmware — see
`../shared/psk.h`, included by every project via the `-I ../shared` build
flag so sender and receiver firmware can't drift out of sync. This is
deliberately simple for now, since every node is firmware we build and flash
ourselves; revisit if the project ever needs per-device keys or field
provisioning. Crypto helpers (`hcEncrypt`/`hcDecrypt`) live in
`../shared/hc_crypto.h`.

Wire format on the air: `[12-byte nonce][ciphertext][16-byte tag]`.

## Known gotcha

The T-Beam Supreme's AXP2101 PMU gates power to the LoRa, GPS, and display
modules behind the ALDO3, ALDO4, and ALDO1 rails respectively, all off at
boot. `initPower()` in `main.cpp` turns them on — skip any of these and
that module silently never powers up even with correct wiring. ALDO1 is
the one that actually bit us: per LilyGO's own hardware doc it's shared
across the display, the BME280 sensor, and the magnetometer, and our
`initPower()` originally only ever enabled ALDO3/ALDO4, never ALDO1. One of
our two units had a screen that stayed completely black through boot even
though `display.begin()` reported success and the rest of the device ran
fine — flashing Meshtastic on the same unit proved the OLED hardware itself
was fine, which pointed straight at power rather than the display or its
I2C address. The SH1106 controller could apparently still ACK basic I2C
reads/writes with no ALDO1 power at all (enough for `begin()` to report
success), but never had the power to actually drive the panel. `initPower()`
now enables ALDO1 alongside ALDO3/ALDO4, and — because `runProvisioningMode()`
can be entered via the boot-hold path *before* `initPower()` would otherwise
run (it's `[[noreturn]]`, so the rest of `setup()` never executes in that
boot) — `initPower()` was moved ahead of the `bootButtonHeldForProvisioning()`
check in `setup()`, so both entry paths into provisioning always have a
powered display too.

LilyGO also ships this board with two magnetometer sub-variants (QMC6310U
vs QMC6310N) that, per their docs, put the SH1106 OLED at different I2C
addresses — 0x3C or 0x3D respectively. We haven't actually confirmed this
varies between our own two units (the ALDO1 gap above was the real bug on
the unit we tested), but `initDisplay()`/`runProvisioningMode()` both probe
`0x3C` then `0x3D` rather than assuming one anyway (`DISPLAY_I2C_ADDR_CANDIDATES`),
since it's cheap insurance against a documented hardware variance for this
exact board. Display failure is non-fatal everywhere, including at boot —
`initDisplay()` used to `haltWithError()` if the display didn't respond,
which took the *entire* device down (no GPS, no LoRa, no SOS) over what
looked like just a dead screen. It now logs and continues with
`displayAvailable = false`; every `display*` function in `main.cpp` checks
that flag and no-ops rather than touching hardware that was never found.

The board's PWR button isn't a GPIO at all — it's wired into the AXP2101 as
its power key, reported by pulling `PMU_IRQ_PIN` low. `initPower()`
disables every other AXP2101 IRQ source and enables only
`XPOWERS_AXP2101_PKEY_SHORT_IRQ`, so `bluetoothButtonPressed()` can treat
"pin is low" as "PWR was pressed" without decoding the status register —
if another IRQ source ever gets enabled here for something else, that
assumption breaks and the pin could go low for an unrelated reason.
`PMU->clearIrqStatus()` must be called after reading a press or the pin
stays stuck low and every subsequent check reads as a phantom press.

The display's I2C bus (`Wire`) needs a full re-init (`reinitDisplay()`)
before every cycle's status draw, not just an ON command — unlike the
GPS/LoRa rails, the display's own power is never cut, but the ESP32's I2C
peripheral state doesn't reliably survive `esp_light_sleep_start()` either
way. Without this the screen goes dark after the first sleep and stays dark
even though GPS/LoRa/SOS keep working fine, since those get a full re-init
already (their rails actually are cut). `reinitDisplay()` is non-fatal on
failure, unlike `initDisplay()` at boot — a flaky display shouldn't halt the
whole device the way a missing one at boot should.

Confirmed, not just suspected: native USB CDC serial (`ARDUINO_USB_CDC_ON_BOOT=1`)
is not reliable across sleep on this chip — RadioLib itself emits `"Use of
USB CDC for debug output is not recommended (might stop on first sleep).
Use hardware UART instead."` at compile time whenever that flag is set,
sleep loop or not. If `pio device monitor` (or opening the COM port
directly) stops working partway through testing, replug the USB-C cable to
force Windows to re-enumerate it — don't rely on one continuous serial
session surviving a full sleep cycle. A hardware UART-to-USB adapter would
sidestep this if continuous logging becomes necessary.

RadioLib's `setDio1Action()` (used for interrupt-driven receive, see
`heltec-rx-test/`) fires on any rising edge on that pin — it doesn't know
whether the chip currently means RX-done or TX-done by it. Any
`radio.transmit()` call made while that interrupt is attached (e.g. sending
an ACK from within the RX loop) will trip the same callback via its own
TX-done pulse, so the flag it sets must be explicitly cleared after your own
transmits or the next loop iteration will treat it as a real received
packet — see the comment on `sendAck()` in `heltec-rx-test/src/main.cpp`.

## Next milestones (not yet implemented)

- Power measurement against the report's theoretical current-draw numbers
  now that the sleep/rail-gating duty cycle (see Power, above) is in place
  to measure

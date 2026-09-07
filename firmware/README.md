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

The T-Beam Supreme's AXP2101 PMU gates power to the LoRa and GPS modules
behind the ALDO3/ALDO4 rails, which are off at boot. `initPower()` in
`main.cpp` turns them on — skip that step and the radio/GPS silently never
power up even with correct wiring.

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

# Hiker's Companion Firmware

Custom PlatformIO/Arduino firmware for the LILYGO T-Beam Supreme
(ESP32-S3 + SX1262 LoRa + AXP2101 PMU + GNSS).

This is separate from `../Lora Source/`, which is LilyGO's own vendored
example repo — kept only as pinout/API reference, not built directly.

Also separate: `../heltec-rx-test/`, a throwaway RX bench rig on a Heltec
WiFi LoRa 32 V3 used to confirm this board's LoRa TX is actually reaching a
second radio — decrypted payload + RSSI/SNR go to both serial and the
Heltec's onboard SSD1306 OLED, which wakes for 5s on each received signal
and sleeps otherwise. Not part of the product firmware.

## Test hardware on hand

2x LILYGO T-Beam Supreme, 2x Heltec WiFi LoRa 32 V3. Enough to test beyond a
single sender/receiver pair once we get there — e.g. two T-Beam senders into
one gateway, or a relay hop — not just the current one-to-one link test.

## Status: Milestone 1 — GPS + LoRa send loop, with SOS button and status display

Reads GPS fixes and transmits location over LoRa on a timer. The onboard
BOOT button (GPIO0) toggles a standing SOS beacon: pressing it sends an
immediate "SOS"-flagged packet and switches every subsequent periodic send
to "SOS" too (instead of the routine "OK") until pressed again to cancel —
see `sendLocationPacket()` / `pollButton()` in `src/main.cpp`.

An SH1106 OLED (I2C, pins 17/18) shows live status via `showStatus()`:
message type on top, "Sending" / "Delivered" / "Send failed" below. No ACK
yet, so "Delivered" means "the radio call returned success," not "a human
received this" — see FW-06 in the backlog.

The panel stays off between transmissions to save power — it wakes for a
send and holds for `DISPLAY_HOLD_MS` (5s) after the result, then sleeps
again (`displayWake()`/`displaySleep()`, timer checked in `loop()`).

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

## Next milestones (not yet implemented)

- LoRa RX side / ACK handling so "Delivered" status is real, not assumed
- Power measurement against the report's theoretical current-draw numbers

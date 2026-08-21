# Hiker's Companion Firmware

Custom PlatformIO/Arduino firmware for the LILYGO T-Beam Supreme
(ESP32-S3 + SX1262 LoRa + AXP2101 PMU + GNSS).

This is separate from `../Lora Source/`, which is LilyGO's own vendored
example repo — kept only as pinout/API reference, not built directly.

## Status: Milestone 1 — GPS + LoRa send loop

Reads GPS fixes and transmits location over LoRa on a timer. No button/SOS
logic or display yet.

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

## Known gotcha

The T-Beam Supreme's AXP2101 PMU gates power to the LoRa and GPS modules
behind the ALDO3/ALDO4 rails, which are off at boot. `initPower()` in
`main.cpp` turns them on — skip that step and the radio/GPS silently never
power up even with correct wiring.

## Next milestones (not yet implemented)

- Button input + SOS message framing
- LCD status display ("Sending" / "Delivered")
- LoRa RX side / ACK handling so "Delivered" status is real, not assumed
- Power measurement against the report's theoretical current-draw numbers
- Encryption (AES-128, required by the report's standards section)

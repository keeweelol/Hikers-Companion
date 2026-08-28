# Versions

Snapshots of the firmware source (`firmware/`, `heltec-rx-test/`, `shared/`)
at each major change, so we can always go back to a known-working state
without digging through git history.

Numbering: start at 1.0, bump by 0.1 for each major change (a new feature or
milestone, not every small tweak). Each `versions/X.Y/` folder is a snapshot
of source files only — no build artifacts (`.pio/`, `.venv/`, `.cache/`) — plus
a `NOTES.md` describing what that version contains.

To go back to an old version: copy its `firmware/`, `heltec-rx-test/`, and
`shared/` subfolders back over the top-level ones of the same name.

| Version | What it is |
|---|---|
| 1.0 | Milestone 1 (GPS + LoRa send loop) + AES-128-GCM encryption + SOS button toggle. Baseline, hardware-verified. |
| 1.1 | Adds status OLED displays to both the T-Beam (Sending/Delivered/Send failed) and the Heltec RX rig (received type + RSSI/SNR), both with power-saving sleep/wake. |
| 1.2 | Adds a real delivery ACK: Heltec replies `ACK,<id>` on every valid packet, T-Beam waits up to 2s and now distinguishes Delivered / No ACK / Send failed. |

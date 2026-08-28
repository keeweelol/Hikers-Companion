# Version 1.0

Baseline snapshot: everything working and hardware-verified as of this session.

- GPS + LoRa send loop (Milestone 1), hardware-verified on the T-Beam Supreme.
- AES-128-GCM encryption on all LoRa payloads (pre-shared key in `shared/psk.h`).
- SOS button (GPIO0 / BOOT button): press toggles a standing SOS beacon —
  every send goes out flagged "SOS" instead of "OK" until pressed again to
  cancel.
- `heltec-rx-test/`: RX bench rig on a Heltec WiFi LoRa 32 V3, confirmed
  receiving and decrypting real packets from the T-Beam with RSSI/SNR logged.

Not yet implemented: LCD status display, LoRa RX/ACK on the gateway side,
sleep/wake power management, final GPS polling interval.

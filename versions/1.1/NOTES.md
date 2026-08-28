# Version 1.1

Built on 1.0 — adds status displays to both boards (FW-05).

- T-Beam: SH1106 OLED (I2C, pins 17/18) shows "Sending" / "Delivered" /
  "Send failed" per send, plus a "Ready" boot screen. See `showStatus()` in
  `firmware/src/main.cpp`. No ACK yet, so "Delivered" means the radio call
  returned success, not that anyone received it (FW-06).
- Heltec RX rig: onboard SSD1306 OLED shows the received message type
  (OK/SOS, large) with RSSI/SNR below, plus distinct banners for
  decrypt/auth failure, CRC mismatch, and radio errors. Needed enabling the
  Vext power rail (GPIO36, active-low) before the display responds.
- Both displays power down between events rather than staying lit: the
  T-Beam's wakes for a send and holds 5s past the result before sleeping;
  the Heltec's wakes for 5s on any received signal. Non-blocking timer
  (`displayWake()`/`displaySleep()`/`scheduleDisplayOff()`, checked once per
  `loop()`) — no `delay()`, since that would stall GPS/button/radio polling.

Everything from 1.0's NOTES.md still applies (GPS + LoRa send loop, AES-128-GCM
encryption, SOS button toggle).

# Version 1.2

Built on 1.1 — adds a real delivery ACK (FW-06).

- Outgoing packets now carry a message id: `<type>,<id>,<location>` instead
  of just `<type>,<location>`.
- Heltec RX rig replies with an encrypted `ACK,<id>` for every packet it
  successfully decrypts (`sendAck()`), sent before its own display update to
  keep latency low.
- T-Beam listens for up to `ACK_TIMEOUT_MS` (2s) after transmitting
  (`waitForAck()`, using RadioLib's blocking `receive()` with a timeout) and
  now shows three distinct outcomes instead of two:
  - "Delivered" — a matching ACK came back (a real confirmation now, not an
    assumption)
  - "No ACK" — transmit succeeded but nothing came back in time
  - "Send failed" — the radio call itself errored

Trade-off: `waitForAck()` blocks `loop()` (GPS/button polling included) for
up to 2s after every send. Acceptable for now since it's once per send, not
continuous — revisit if button responsiveness during that window becomes a
problem.

**Bug fix folded into this snapshot**: `sendAck()`'s own `radio.transmit()`
call was tripping the Heltec's `onPacketReceived()` interrupt via its
TX-done pulse on the same DIO1 pin `setDio1Action()` watches for RX-done —
RadioLib's callback doesn't distinguish the two. The next `loop()` iteration
then read a corrupted mix of the ACK's own bytes and leftover tail from the
real packet (SX1262's TX/RX buffers share memory), which reliably failed
the GCM auth check and printed a spurious "decrypt/auth failed" right after
every successful exchange. Fixed by clearing `packetReceived = false`
immediately after `sendAck()`'s transmit call. See the "Known gotcha"
section of `firmware/README.md` for the full mechanism.

Everything from 1.1's NOTES.md still applies (status displays with
sleep/wake power saving) and 1.0's (GPS + LoRa send loop, AES-128-GCM
encryption — now covering the ACK too — and the SOS button toggle).

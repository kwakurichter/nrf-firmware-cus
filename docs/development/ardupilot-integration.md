---
title: ArduPilot integration
page_id: ardupilot_integration
---

Specification for the ArduPilot side of the MAVLink-over-ESB link. Written to
be read standalone — no knowledge of this firmware is assumed.

## The system

A Crazyflie 2.x carries two MCUs on one board:

```
  GCS ── USB ── Crazyradio 2.0 ──── 2.4 GHz ESB ──── nRF51822 ── UART ── STM32
                  (nRF52840)                        (radio)     1Mbaud   (ArduPilot)
                                                        │
                                          also: power, button, charging
```

ArduPilot runs on the **STM32**. It has no radio of its own. The **nRF51822**
owns the radio and is reached over a UART running a framed protocol called
**syslink**. To ArduPilot, the nRF51 should be treated as a dumb, lossy,
packet-oriented serial link.

Two destinations exist, and they are **not** modes — they are concurrent:

- **Unicast** — to a ground station via a Crazyradio 2.0. Hardware ack and
  automatic retry apply.
- **Broadcast** — to any peer on the shared address. Unacked, no retry.

The radio receives on both addresses simultaneously and picks the transmit
address per packet, so there is no mode to set, nothing to reconfigure, and no
restart. Telemetry and peer traffic interleave freely in both directions. The
destination is carried by the syslink packet type.

## What the nRF51 does and does not do

It does **not** parse MAVLink, track frame boundaries, reassemble anything, or
retry above what the radio hardware provides. One syslink packet becomes
exactly one radio packet and vice versa. Treat it as a pipe that moves opaque
byte chunks and occasionally drops one.

It does own the radio channel, address and datarate, and it handles battery,
button and power-management traffic on other syslink packet types that are
unrelated to MAVLink.

## Syslink framing

Serial, **1 Mbaud, 8N1**.

The nRF51 drives RTS (`NRF_FLOW_CTRL` in the ArduPilot hwdef) from its UART
receive FIFO, so the STM32 should gate transmission on it to avoid overrunning
the nRF51's UART. There is no CTS in the other direction. This line has
nothing to do with radio queue depth — see requirement 4.

```
+-----------+------+-----+=============+-----+-----+
|   START   | TYPE | LEN | DATA        |   CKSUM   |
+-----------+------+-----+=============+-----+-----+
```

- `START` — two constant bytes, `0xBC 0xCF`
- `TYPE` — one byte, packet type
- `LEN` — one byte, length of `DATA`
- `CKSUM` — two-byte Fletcher-8 checksum ([RFC 1146](https://tools.ietf.org/html/rfc1146))
  computed over `TYPE`, `LEN` and `DATA`

## MAVLink packet types

### `SYSLINK_RADIO_MAVLINK` — `0x0C`

An opaque chunk of the MAVLink byte stream, **1 to 251 bytes**, unicast to the
ground station. Both directions: STM32→nRF51 transmits, nRF51→STM32 delivers
what arrived on the unicast address.

Queued, not immediate — see requirement 4.

### `SYSLINK_RADIO_MAVLINK_BROADCAST` — `0x0D`

Identical payload, but broadcast to peers instead. On receive, this type means
the chunk arrived on the broadcast address.

Sent immediately rather than queued, so it consumes no transmit slot and
cannot fail for lack of room.

### `SYSLINK_RADIO_MAVLINK_SPACE` — `0x0E`

One byte: free unicast transmit slots, peaking at 5. Sent **unsolicited by the
nRF51 whenever the count changes**. This is the backpressure signal — see
requirement 4.

## Operating requirements

These are the constraints that will cause silent, hard-to-diagnose failures if
missed.

**1. Chunks cap at 251 bytes, not 252.** One byte of the 252-byte radio
payload is a marker that keeps MAVLink traffic from being mistaken for the
radio's own control packets. A chunk over 251 bytes is **dropped, not
truncated** — losing the tail silently would corrupt a frame in a way the
receiver could not detect.

**2. A MAVLink v2 frame can exceed one chunk.** Maximum frame size is 267
bytes unsigned, 280 signed. `FILE_TRANSFER_PROTOCOL` lands near 261, and FTP
is how a GCS downloads parameters and logs — this is not a rare edge case.
Split oversized frames across chunks and accept that losing either half costs
the frame.

**3. Send one whole frame per chunk where it fits.** Nothing enforces frame
alignment, but a chunk spanning two frames means one lost packet damages both.

**4. Unicast transmission can fail; use the space report for backpressure.**
The Crazyflie is a receiver from the radio's point of view, so downlink chunks
only leave in ack payloads when the ground station polls. If it stops polling,
the queue fills and further chunks are silently dropped. Usable depth is **5**.

Track free slots from `SYSLINK_RADIO_MAVLINK_SPACE`: decrement locally on each
unicast send, and refresh from the report. That value is what `txspace()`
should be derived from. Broadcasts do not consume slots.

**Do not rely on the `NRF_FLOW_CTRL` pin for this.** That line is the nRF51's
UART RTS and reflects its UART receive FIFO only. The nRF51 keeps draining
syslink when the radio queue is full — it just discards chunks — so the line
never asserts for this condition. Both mechanisms are needed and they guard
different things: the pin prevents UART overrun, the space report prevents
radio-queue overflow. Note the nRF51 drives RTS but has no CTS input, so
hardware flow control is one-directional.

**5. The link is lossy.** Unicast retries in hardware but eventually gives up;
broadcast has no retry at all. MAVLink tolerates this by design — the parser
resyncs on `STX`. Do not build anything that assumes reliable delivery.

**6. ArduPilot owns the radio configuration.** The nRF51's compiled-in
defaults (channel 80, address `E7E7E7E7E7`) are **not** what the radio ends up
using. The STM32 pushes the stored channel, datarate and address over syslink
at boot, and those win. A mismatch between the vehicle and the ground station
looks identical to a packet-format failure, so verify both ends agree before
debugging anything else.

**7. The radio is gated off for the first 3 seconds.** After boot the nRF51
does not receive until either the STM32 sends `SYSLINK_RADIO_READY` (`0x0B`)
or a 3-second timeout expires. Sending that packet early shortens startup.

## Throughput

The UART is the bottleneck, not the radio. At 1 Mbaud the theoretical ceiling
is ~100 kB/s, but the nRF51's UART has no DMA — it is a per-byte interrupt on
a 16 MHz Cortex-M0, and transmission busy-waits. **Budget ~50 kB/s sustained
duplex**, and expect the main loop to stall for ~2.6 ms while forwarding a
full-size chunk.

Ample for telemetry. Not a bulk data pipe.

## Host side

A ground station cannot use `cflib` for this. Its `Crazyradio.send_packet()`
reads the dongle's bulk IN endpoint with a hardcoded 64-byte length and
truncates anything larger. A host driver must request the full transfer, and
must send a zero-length USB packet after any write whose length is an exact
multiple of 64 or the dongle will wait for a transfer end that never comes.

A working minimal driver is in `tools/crazyradio2_large.py`. The dongle also
needs its large packet mode enabled with vendor request `0x27`, which is
opt-in because the legacy protocol reads a >32-byte broadcast transfer as two
packets rather than one.

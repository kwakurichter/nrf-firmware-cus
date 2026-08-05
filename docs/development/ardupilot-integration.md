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

## Bring-up: the link starts silent

**The nRF51 sends nothing over the UART until the STM32 speaks first.** This is
the designed behaviour and it is the first thing to get right — until it is
satisfied, a perfectly working nRF51 is indistinguishable from a dead one.

There are three independent gates.

### Gate 1 — syslink transmit is disabled until a valid packet arrives

`syslinkSend()` is a no-op until an inbound syslink packet has passed **both**
checksum bytes. Until then the nRF51 emits no battery data, no RSSI, no MAVLink
and no handshake. Any valid packet lifts the gate. It re-arms only when the nRF51
powers the STM32 down (the `SYSOFF` radio bootloader command), so in normal
operation it is a one-time handshake per boot.

Send this — `SYSLINK_RADIO_READY`, zero length:

```
BC CF 0B 00 0B 16
```

**The nRF51 echoes the identical frame back.** That echo is the definitive
proof that the serial link, baud rate, framing and checksum are all correct. It
also satisfies gate 3 immediately.

### Gate 2 — battery *and* RSSI both need enabling

```
BC CF 14 00 14 28      SYSLINK_PM_BATTERY_AUTOUPDATE, zero length
```

Note that the periodic RSSI report is emitted from inside the same
`enableBatteryAutoupdate` check as the battery packet, despite being unrelated
to it. Without this packet there is **no RSSI either**, which reads like a
broken link rather than a disabled feature.

### Gate 3 — the radio is deaf for the first 3 seconds

Covered in requirement 7. It gates radio reception only, not the UART, so it is
not what keeps the link quiet at boot.

### Recommended boot sequence

1. `BC CF 0B 00 0B 16` — activate syslink and the radio. **Expect the echo.**
2. `BC CF 14 00 14 28` — enable battery and RSSI reporting.
3. Radio configuration if the defaults are wrong: channel (`0x01`), datarate
   (`0x02`), address (`0x05`). Each is echoed back as confirmation.

After step 1 the nRF51 also sends an unsolicited `SYSLINK_RADIO_MAVLINK_SPACE`
(`0x0E`, one byte, value 5). It reports on change and starts from an impossible
value, so in practice it is the first packet the nRF51 ever sends.

### Checksum, precisely

Both bytes start at zero and cover `TYPE`, `LEN` and `DATA` — **not** the two
start bytes:

```
for each byte b in (TYPE, LEN, DATA...):
    cksum_a = (cksum_a + b)        & 0xFF
    cksum_b = (cksum_b + cksum_a)  & 0xFF
```

Wire order is `START1 START2 TYPE LEN DATA CKSUM_A CKSUM_B` — **TYPE before
LEN**. Swapping those two produces a well-formed frame that will never lift
gate 1, with no error reported anywhere.

### There is no debug output to look for

`DEBUG_PRINT` on the nRF51 expands to nothing unless the firmware is built with
`DEBUG_PRINT_ON_SEGGER_RTT`, and even then it goes to SEGGER RTT over SWD, not
to the UART. There is no printf-to-serial path, and adding one would corrupt
syslink because they share the port. The echo in gate 1 is the diagnostic to
use instead.

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

## Battery telemetry

Unrelated to the MAVLink types, but the STM32 will want it.

`SYSLINK_PM_BATTERY_STATE` (`0x13`) is sent by the nRF51 at 100 Hz once the
STM32 enables it with `SYSLINK_PM_BATTERY_AUTOUPDATE` (`0x14`).

```
+-------+------+------+------+
| FLAGS | VBAT | ISET | TEMP |
+-------+------+------+------+
 1 byte   4       4      4
```

- `FLAGS` — bit0 charging, bit1 USB powered, bit2 can charge
- `VBAT` — float, battery volts
- `ISET` — float, charge current in mA
- `TEMP` — float, nRF51 **die** temperature in degrees C

**This build is 13 bytes**, with `TEMP` present (`PM_SYSLINK_INCLUDE_TEMP` is
enabled). Upstream defaults to 9 bytes without it. Accepting both lengths is
still the robust choice, since the field is a compile-time option and someone
will eventually build without it — but 13 is what you will see here.

`TEMP` is the die sensor, not the battery or ambient, so it reads above room
temperature. It is sampled anyway for charge temperature control.

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
does not receive on the air until either the STM32 sends `SYSLINK_RADIO_READY`
(`0x0B`) or a 3-second timeout expires. Sending that packet early shortens
startup.

This gate affects the radio only. It is **not** what stops the nRF51 talking
over the UART at boot, and waiting out the timeout will not start the flow —
see "Bring-up: the link starts silent" above, which is the gate that matters.

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

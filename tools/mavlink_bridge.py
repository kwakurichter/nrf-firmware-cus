#!/usr/bin/env python3
"""Bridge MAVLink between a Crazyflie over ESB and a local GCS.

    # watch what the vehicle is sending, decode it, print a summary
    .venv/bin/python tools/mavlink_bridge.py

    # same, but also forward to QGroundControl
    .venv/bin/python tools/mavlink_bridge.py --udp 127.0.0.1:14550

    # non-default radio settings
    .venv/bin/python tools/mavlink_bridge.py --uri radio://0/100/2M/E7E7E7E706

Polling is not optional. The Crazyflie is a PRX, so it can only transmit inside
an ack — it never speaks unprompted. Every downlink byte arrives as the payload
of an ack to something this script sent, which is why the loop transmits
continuously even with nothing to say. When there is no uplink to carry, it
sends a bare marker byte as a poll.

Requires the Crazyradio 2.0 large-packet firmware. cflib cannot be used: its
Crazyradio.send_packet() reads the bulk IN endpoint with a hardcoded 64 byte
length, so it truncates any ack payload above that.
"""

import argparse
import collections
import socket
import sys
import time

sys.path.insert(0, __file__.rsplit("/", 1)[0])

from crazyradio2_large import Crazyradio2, DR_250K, DR_1M, DR_2M  # noqa: E402

# Must match MAVLINK_AIR_MARKER in the Crazyflie's mavlink_transport.h. Present
# on every MAVLink packet in both directions; acks that do not start with it are
# the radio's own empty acks and carry nothing.
MAVLINK_AIR_MARKER = 0xE0

# 252 byte ESB payload less the marker.
MAVLINK_CHUNK_MAX = 251

DEFAULT_URI = "radio://0/80/2M/E7E7E7E7E7"

RATES = {"250K": DR_250K, "1M": DR_1M, "2M": DR_2M}

# MAVLink framing constants, used instead of a dialect so this works with any
# message set and needs no XML.
STX_V1 = 0xFE
STX_V2 = 0xFD
MAVLINK_IFLAG_SIGNED = 0x01


def parse_uri(uri):
    """radio://<index>/<channel>/<rate>/<address> -> (index, ch, rate, addr)."""
    if not uri.startswith("radio://"):
        raise ValueError("URI must start with radio://")

    parts = uri[len("radio://"):].split("/")
    if len(parts) != 4:
        raise ValueError("URI must be radio://<index>/<channel>/<rate>/<address>")

    index, channel, rate, address = parts

    if rate.upper() not in RATES:
        raise ValueError(f"rate must be one of {', '.join(RATES)}")

    addr = bytes.fromhex(address)
    if len(addr) != 5:
        raise ValueError("address must be 5 bytes / 10 hex digits")

    return int(index), int(channel), RATES[rate.upper()], addr


def split_frames(buf):
    """Pull complete MAVLink frames out of `buf`, consuming what it returns.

    Deliberately dialect-free: frame length comes from the header alone, so no
    message definitions are needed and unknown messages pass through intact.
    """
    frames = []

    while True:
        # Discard anything before a plausible start byte.
        start = 0
        while start < len(buf) and buf[start] not in (STX_V1, STX_V2):
            start += 1
        if start:
            del buf[:start]

        if len(buf) < 3:
            return frames

        if buf[0] == STX_V1:
            # STX, LEN, SEQ, SYSID, COMPID, MSGID = 6, plus payload, plus CRC.
            total = 8 + buf[1]
        else:
            # STX, LEN, INCOMPAT, COMPAT, SEQ, SYSID, COMPID, MSGID(3) = 10,
            # plus payload, plus CRC, plus signature when the signed flag is set.
            total = 12 + buf[1]
            if buf[2] & MAVLINK_IFLAG_SIGNED:
                total += 13

        if len(buf) < total:
            return frames

        frames.append(bytes(buf[:total]))
        del buf[:total]


def chunk_frame(frame):
    """Split one frame into radio-sized pieces.

    Almost every MAVLink frame fits in a single 251 byte chunk. The exceptions
    are worth knowing about: a v2 frame reaches 267 bytes unsigned and 280
    signed, and FILE_TRANSFER_PROTOCOL lands near 261 -- which is what a GCS
    uses to fetch parameters and logs. Those get split, and losing either half
    costs the frame.
    """
    return [frame[i:i + MAVLINK_CHUNK_MAX]
            for i in range(0, len(frame), MAVLINK_CHUNK_MAX)]


class Observer:
    """Decodes the downlink for display. Optional; falls back to raw counts."""

    def __init__(self):
        self.counts = collections.Counter()
        self.total_frames = 0
        self.heartbeat = None
        self._mav = None

        try:
            import io
            from pymavlink.dialects.v20 import ardupilotmega as dialect
            self._mav = dialect.MAVLink(io.BytesIO())
            # A lossy link delivers damaged frames as a matter of course; the
            # parser must resync rather than raise.
            self._mav.robust_parsing = True
        except Exception:  # noqa: BLE001
            self._mav = None

    def feed(self, data):
        if self._mav is None:
            return

        try:
            for msg in self._mav.parse_buffer(data) or []:
                name = msg.get_type()
                if name == "BAD_DATA":
                    self.counts["BAD_DATA"] += 1
                    continue
                self.counts[name] += 1
                self.total_frames += 1
                if name == "HEARTBEAT":
                    self.heartbeat = msg
        except Exception:  # noqa: BLE001
            pass

    def summary(self):
        if self._mav is None:
            return "pymavlink unavailable, showing byte counts only"

        if not self.counts:
            return "no MAVLink frames decoded yet"

        top = ", ".join(f"{n}x{c}" for n, c in self.counts.most_common(6))
        line = f"{self.total_frames} frames | {top}"

        if self.heartbeat is not None:
            hb = self.heartbeat
            armed = bool(getattr(hb, "base_mode", 0) & 0x80)
            line += (f"\n           HEARTBEAT sys{hb.get_srcSystem()} "
                     f"type={hb.type} autopilot={hb.autopilot} "
                     f"mode={hb.custom_mode} armed={armed}")

        return line


def main():
    ap = argparse.ArgumentParser(
        description="Bridge MAVLink between a Crazyflie over ESB and a GCS.")
    ap.add_argument("--uri", default=DEFAULT_URI,
                    help=f"radio URI (default {DEFAULT_URI})")
    ap.add_argument("--udp", metavar="HOST:PORT",
                    help="forward MAVLink to this UDP endpoint, e.g. "
                         "127.0.0.1:14550 for QGroundControl")
    ap.add_argument("--idle-poll-ms", type=float, default=2.0,
                    help="pause between polls when the link is idle "
                         "(default 2.0, lower means lower latency and more USB "
                         "traffic)")
    ap.add_argument("--status-sec", type=float, default=2.0,
                    help="seconds between status lines (default 2.0)")
    ap.add_argument("--quiet", action="store_true",
                    help="suppress the periodic status line")
    args = ap.parse_args()

    try:
        index, channel, rate, address = parse_uri(args.uri)
    except ValueError as exc:
        print(f"Bad URI: {exc}")
        return 2

    try:
        radio = Crazyradio2(index)
    except Exception as exc:  # noqa: BLE001
        print(f"Could not open a Crazyradio: {exc}")
        print("Check it is plugged in and running the large-packet firmware.")
        return 1

    radio.set_channel(channel)
    radio.set_data_rate(rate)
    radio.set_address(address)
    radio.set_ack_enabled(True)
    radio.set_large_packet_mode(True)

    print(f"Radio open: {args.uri}")

    sock = None
    peer = None
    if args.udp:
        host, _, port = args.udp.partition(":")
        peer = (host, int(port))
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setblocking(False)
        # Bind so the GCS has somewhere to reply to.
        sock.bind(("0.0.0.0", 0))
        print(f"Forwarding to {peer[0]}:{peer[1]} "
              f"(local port {sock.getsockname()[1]})")
        print("Point QGroundControl at a UDP link on that port, or let it "
              "auto-connect on 14550.")
    else:
        print("Observe only. Pass --udp 127.0.0.1:14550 to feed a GCS.")

    observer = Observer()
    uplink_bytes = bytearray()
    pending = collections.deque()

    stats = {"polls": 0, "down_pkts": 0, "down_bytes": 0,
             "up_chunks": 0, "up_bytes": 0, "no_ack": 0, "usb_err": 0}

    last_status = time.time()
    poll_marker = bytes([MAVLINK_AIR_MARKER])

    print("Polling. Ctrl-C to stop.\n")

    try:
        while True:
            # --- collect uplink from the GCS ---------------------------------
            if sock is not None:
                while True:
                    try:
                        data, src = sock.recvfrom(4096)
                    except BlockingIOError:
                        break
                    except OSError:
                        break
                    # Reply to wherever the GCS actually is.
                    peer = src
                    uplink_bytes += data

                for frame in split_frames(uplink_bytes):
                    pending.extend(chunk_frame(frame))

            # --- one poll ----------------------------------------------------
            if pending:
                payload = poll_marker + pending.popleft()
                stats["up_chunks"] += 1
                stats["up_bytes"] += len(payload) - 1
            else:
                payload = poll_marker

            ack = radio.send_packet(payload)
            stats["polls"] += 1

            if ack is None:
                stats["usb_err"] += 1
                time.sleep(0.01)
                continue

            if not ack.ack:
                # Vehicle out of range, powered down, or on another channel.
                stats["no_ack"] += 1
                time.sleep(args.idle_poll_ms / 1000.0)
            elif ack.data and ack.data[0] == MAVLINK_AIR_MARKER:
                down = ack.data[1:]
                if down:
                    stats["down_pkts"] += 1
                    stats["down_bytes"] += len(down)
                    observer.feed(down)
                    if sock is not None and peer is not None:
                        try:
                            sock.sendto(down, peer)
                        except OSError:
                            pass
                else:
                    # Marker with no payload: the vehicle had nothing queued.
                    time.sleep(args.idle_poll_ms / 1000.0)
            else:
                # An empty radio ack, or CRTP traffic that is not ours.
                time.sleep(args.idle_poll_ms / 1000.0)

            # --- status ------------------------------------------------------
            now = time.time()
            if not args.quiet and now - last_status >= args.status_sec:
                elapsed = now - last_status
                print(f"[{time.strftime('%H:%M:%S')}] "
                      f"down {stats['down_bytes']}B/{stats['down_pkts']}pkt "
                      f"up {stats['up_bytes']}B/{stats['up_chunks']}chunk "
                      f"polls {stats['polls']} "
                      f"({stats['polls'] / elapsed:.0f}/s) "
                      f"no-ack {stats['no_ack']} usb-err {stats['usb_err']}")
                print(f"           {observer.summary()}")
                for k in ("polls", "down_pkts", "down_bytes",
                          "up_chunks", "up_bytes", "no_ack", "usb_err"):
                    stats[k] = 0
                last_status = now

    except KeyboardInterrupt:
        print("\nStopped.")

    return 0


if __name__ == "__main__":
    sys.exit(main())

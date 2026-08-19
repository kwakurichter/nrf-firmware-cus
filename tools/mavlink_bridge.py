#!/usr/bin/env python3
"""Bridge MAVLink between one or more Crazyflies over ESB and a local GCS.

    # one vehicle, watch and decode
    .venv/bin/python tools/mavlink_bridge.py

    # one vehicle, feed QGroundControl
    .venv/bin/python tools/mavlink_bridge.py --udp 127.0.0.1:14550

    # two vehicles, one Crazyradio each, both into the same GCS
    .venv/bin/python tools/mavlink_bridge.py \
        --uri radio://0/80/2M/E7E7E7E7E7 \
        --uri radio://1/90/2M/E7E7E7E706 \
        --udp 127.0.0.1:14550

Polling is not optional. The Crazyflie is a PRX, so it can only transmit inside
an ack -- it never speaks unprompted. Every downlink byte arrives as the payload
of an ack to something this script sent, which is why each link transmits
continuously even with nothing to say. When there is no uplink to carry, it
sends a bare marker byte as a poll.

One Crazyradio serves exactly one vehicle at a time: in normal operation the
dongle enables receive pipe 0 only, so there is no way to poll two addresses at
once. Multiple vehicles therefore mean multiple dongles, one thread each.

PUT EACH DONGLE ON A DIFFERENT CHANNEL. ESB has no carrier sense, so two
dongles sharing a frequency will collide and retries will only partly hide it.

Requires the Crazyradio 2.0 large-packet firmware. cflib cannot be used: its
Crazyradio.send_packet() reads the bulk IN endpoint with a hardcoded 64 byte
length, so it truncates any ack payload above that.
"""

import argparse
import collections
import socket
import sys
import threading
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


def frame_src_system(frame):
    """System id of the sender, straight out of the header."""
    if len(frame) < 6:
        return None
    return frame[3] if frame[0] == STX_V1 else frame[5]


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


def make_parser():
    """A standalone MAVLink parser, or None when pymavlink is unavailable."""
    try:
        import io
        from pymavlink.dialects.v20 import ardupilotmega as dialect
        mav = dialect.MAVLink(io.BytesIO())
        # A lossy link delivers damaged frames as a matter of course; the
        # parser must resync rather than raise.
        mav.robust_parsing = True
        return mav
    except Exception:  # noqa: BLE001
        return None


class Router:
    """Decides which link an uplink frame belongs to.

    The mapping is learned from downlink: whichever link a system id was last
    heard on is where its commands go. Anything targeted at a system we have
    not heard from, or not targeted at all, goes to every link -- vehicles
    ignore what is not addressed to them, so the cost is bandwidth rather than
    confusion.
    """

    def __init__(self):
        self._sysid_link = {}
        self._parser = make_parser()
        self._lock = threading.Lock()

    def learn(self, sysid, link):
        if sysid is None or sysid == 0:
            return
        with self._lock:
            if self._sysid_link.get(sysid) is not link:
                self._sysid_link[sysid] = link

    def target_of(self, frame):
        """Target system id, or None when the frame is not addressed."""
        if self._parser is None:
            return None
        try:
            for msg in self._parser.parse_buffer(frame) or []:
                if msg.get_type() == "BAD_DATA":
                    continue
                target = getattr(msg, "target_system", None)
                if target:
                    return target
        except Exception:  # noqa: BLE001
            pass
        return None

    def links_for(self, frame, all_links):
        target = self.target_of(frame)
        if target:
            with self._lock:
                link = self._sysid_link.get(target)
            if link is not None:
                return [link]
        return all_links

    def known(self):
        with self._lock:
            return dict(self._sysid_link)


class RadioLink(threading.Thread):
    """One Crazyradio polling one vehicle."""

    def __init__(self, uri, on_downlink, idle_poll_ms):
        super().__init__(daemon=True)

        index, channel, rate, address = parse_uri(uri)

        self.uri = uri
        self.channel = channel
        self.address = address
        self.on_downlink = on_downlink
        self.idle_poll_s = idle_poll_ms / 1000.0

        # Which physical dongle answers to which index is not stable across
        # replug, but it does not matter: the channel and address below decide
        # which vehicle this link talks to, so identical dongles are
        # interchangeable. The bus/address is reported anyway for correlation.
        self.radio = Crazyradio2(index, warn_ambiguous=False)
        self.radio.set_channel(channel)
        self.radio.set_data_rate(rate)
        self.radio.set_address(address)
        self.radio.set_ack_enabled(True)
        self.radio.set_large_packet_mode(True)

        dev = self.radio.dev
        self.usb_id = f"bus{dev.bus}.addr{dev.address}"

        self.sysid = None
        self.parser = make_parser()
        self.counts = collections.Counter()

        self._pending = collections.deque()
        self._lock = threading.Lock()
        self._running = True

        self.stats = collections.Counter()

    def queue_frame(self, frame):
        chunks = chunk_frame(frame)
        with self._lock:
            self._pending.extend(chunks)

    def stop(self):
        self._running = False

    def _observe(self, data):
        if self.parser is None:
            return
        try:
            for msg in self.parser.parse_buffer(data) or []:
                name = msg.get_type()
                if name == "BAD_DATA":
                    self.counts["BAD_DATA"] += 1
                    continue
                self.counts[name] += 1
                if self.sysid is None:
                    self.sysid = msg.get_srcSystem()
        except Exception:  # noqa: BLE001
            pass

    def run(self):
        poll_marker = bytes([MAVLINK_AIR_MARKER])

        while self._running:
            with self._lock:
                chunk = self._pending.popleft() if self._pending else None

            if chunk is not None:
                payload = poll_marker + chunk
                self.stats["up_chunks"] += 1
                self.stats["up_bytes"] += len(chunk)
            else:
                payload = poll_marker

            ack = self.radio.send_packet(payload)
            self.stats["polls"] += 1

            if ack is None:
                self.stats["usb_err"] += 1
                time.sleep(0.01)
                continue

            if not ack.ack:
                # Vehicle out of range, powered down, or on another channel.
                self.stats["no_ack"] += 1
                time.sleep(self.idle_poll_s)
                continue

            if ack.data and ack.data[0] == MAVLINK_AIR_MARKER:
                down = ack.data[1:]
                if down:
                    self.stats["down_pkts"] += 1
                    self.stats["down_bytes"] += len(down)
                    self._observe(down)
                    self.on_downlink(self, down)
                    continue

            # Empty ack, or CRTP traffic that is not ours.
            time.sleep(self.idle_poll_s)

    def summary(self):
        who = f"sys{self.sysid}" if self.sysid is not None else "sys?"
        if not self.counts:
            return f"{who} no frames decoded yet"
        top = ", ".join(f"{n}x{c}" for n, c in self.counts.most_common(4))
        return f"{who} {top}"


def main():
    ap = argparse.ArgumentParser(
        description="Bridge MAVLink between Crazyflies over ESB and a GCS.")
    ap.add_argument("--uri", action="append", metavar="URI",
                    help=f"radio URI, repeatable for multiple vehicles "
                         f"(default {DEFAULT_URI}). Give each dongle its own "
                         f"channel: ESB has no carrier sense, so dongles "
                         f"sharing a frequency will collide.")
    ap.add_argument("--udp", metavar="HOST:PORT",
                    help="forward MAVLink to this UDP endpoint, e.g. "
                         "127.0.0.1:14550 for QGroundControl. All vehicles "
                         "share it; the GCS separates them by system id.")
    ap.add_argument("--idle-poll-ms", type=float, default=2.0,
                    help="pause between polls when a link is idle "
                         "(default 2.0, lower means lower latency and more USB "
                         "traffic)")
    ap.add_argument("--status-sec", type=float, default=2.0,
                    help="seconds between status lines (default 2.0)")
    ap.add_argument("--quiet", action="store_true",
                    help="suppress the periodic status line")
    args = ap.parse_args()

    uris = args.uri or [DEFAULT_URI]

    sock = None
    peer = None
    if args.udp:
        host, _, port = args.udp.partition(":")
        peer = (host, int(port))
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setblocking(False)
        sock.bind(("0.0.0.0", 0))

    router = Router()

    def on_downlink(link, data):
        router.learn(frame_src_system(data), link)
        if sock is not None and peer is not None:
            try:
                sock.sendto(data, peer)
            except OSError:
                pass

    links = []
    for uri in uris:
        try:
            link = RadioLink(uri, on_downlink, args.idle_poll_ms)
        except ValueError as exc:
            print(f"Bad URI {uri}: {exc}")
            return 2
        except Exception as exc:  # noqa: BLE001
            print(f"Could not open {uri}: {exc}")
            print("Check the dongle is plugged in and running the "
                  "large-packet firmware.")
            return 1
        links.append(link)
        print(f"Link {len(links) - 1}: {uri}  [{link.usb_id}]")

    channels = [l.channel for l in links]
    if len(channels) != len(set(channels)) and len(links) > 1:
        print("\nWARNING: two dongles share a channel. ESB has no carrier "
              "sense, so they will collide. Give each its own channel.")

    addresses = [l.address for l in links]
    if len(addresses) != len(set(addresses)) and len(links) > 1:
        print("\nWARNING: two links use the same radio address. They are "
              "talking to the same vehicle.")

    if sock is not None:
        print(f"\nForwarding to {peer[0]}:{peer[1]} "
              f"(local port {sock.getsockname()[1]})")
        if len(links) > 1:
            print("Vehicles must have distinct SYSID_THISMAV or the GCS will "
                  "merge them into one.")
    else:
        print("\nObserve only. Pass --udp 127.0.0.1:14550 to feed a GCS.")

    for link in links:
        link.start()

    uplink_bytes = bytearray()
    last_status = time.time()

    print("\nPolling. Ctrl-C to stop.\n")

    try:
        while True:
            if sock is not None:
                while True:
                    try:
                        data, src = sock.recvfrom(4096)
                    except BlockingIOError:
                        break
                    except OSError:
                        break
                    peer = src
                    uplink_bytes += data

                for frame in split_frames(uplink_bytes):
                    for link in router.links_for(frame, links):
                        link.queue_frame(frame)
            else:
                time.sleep(0.01)

            now = time.time()
            if not args.quiet and now - last_status >= args.status_sec:
                elapsed = now - last_status
                print(f"[{time.strftime('%H:%M:%S')}]")
                for i, link in enumerate(links):
                    s = link.stats
                    print(f"  link{i} ch{link.channel:<3} "
                          f"down {s['down_bytes']:5d}B/{s['down_pkts']:<4d} "
                          f"up {s['up_bytes']:4d}B/{s['up_chunks']:<3d} "
                          f"polls {s['polls']:4d} "
                          f"({s['polls'] / elapsed:4.0f}/s) "
                          f"no-ack {s['no_ack']:<4d} err {s['usb_err']}")
                    print(f"         {link.summary()}")
                    link.stats.clear()
                routed = router.known()
                if len(links) > 1 and routed:
                    mapping = ", ".join(
                        f"sys{sid}->link{links.index(l)}"
                        for sid, l in sorted(routed.items()))
                    print(f"  uplink routing: {mapping}")
                last_status = now

            if sock is None:
                continue

    except KeyboardInterrupt:
        print("\nStopping...")
        for link in links:
            link.stop()
        for link in links:
            link.join(timeout=1.0)
        print("Stopped.")

    return 0


if __name__ == "__main__":
    sys.exit(main())

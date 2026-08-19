#!/usr/bin/env python3
"""Bridge MAVLink between one or more Crazyflies over ESB and a local GCS.

    # one vehicle, watch and decode
    .venv/bin/python tools/mavlink_bridge.py

    # one vehicle, feed QGroundControl
    .venv/bin/python tools/mavlink_bridge.py --udp 127.0.0.1:14550

    # two vehicles sharing one dongle, round-robin (required if the drones
    # also talk to each other over P2P, which forces a common channel)
    .venv/bin/python tools/mavlink_bridge.py \
        --uri radio://0/80/2M/E7E7E7E7E7 \
        --uri radio://0/80/2M/E7E7E7E706 \
        --udp 127.0.0.1:14550

    # two vehicles, one dongle each, separate channels (no P2P between them)
    .venv/bin/python tools/mavlink_bridge.py \
        --uri radio://0/80/2M/E7E7E7E7E7 \
        --uri radio://1/90/2M/E7E7E7E706

URIs sharing a dongle index are served by that one dongle, which retunes
between them. URIs with different indices get a dongle each.

Which to choose depends on P2P. A Crazyflie has one radio frequency for
everything, so peers can only hear each other on a common channel -- and two
dongles sharing a channel collide, because ESB has no carrier sense and the
dongle retries in a tight loop with no backoff. Multiplexing one dongle removes
that collision by construction, at the cost of dividing the poll rate.

Polling is not optional. The Crazyflie is a PRX, so it can only transmit inside
an ack -- it never speaks unprompted. Every downlink byte arrives as the payload
of an ack to something this script sent.

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

# Must match MAVLINK_AIR_MARKER in the Crazyflie's mavlink_transport.h.
MAVLINK_AIR_MARKER = 0xE0

# 252 byte ESB payload less the marker.
MAVLINK_CHUNK_MAX = 251

DEFAULT_URI = "radio://0/80/2M/E7E7E7E7E7"

RATES = {"250K": DR_250K, "1M": DR_1M, "2M": DR_2M}
RATE_NAMES = {DR_250K: "250K", DR_1M: "1M", DR_2M: "2M"}

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
    matter though: a v2 frame reaches 267 bytes unsigned and 280 signed, and
    FILE_TRANSFER_PROTOCOL lands near 261 -- which is what a GCS uses to fetch
    parameters and logs. Those get split, and losing either half costs the frame.
    """
    return [frame[i:i + MAVLINK_CHUNK_MAX]
            for i in range(0, len(frame), MAVLINK_CHUNK_MAX)]


def make_parser():
    """A standalone MAVLink parser, or None when pymavlink is unavailable."""
    try:
        import io
        from pymavlink.dialects.v20 import ardupilotmega as dialect
        mav = dialect.MAVLink(io.BytesIO())
        # A lossy link delivers damaged frames as a matter of course.
        mav.robust_parsing = True
        return mav
    except Exception:  # noqa: BLE001
        return None


class Vehicle:
    """One drone: its radio settings, uplink queue, statistics and decoder."""

    def __init__(self, uri):
        self.index, self.channel, self.rate, self.address = parse_uri(uri)
        self.uri = uri

        self.sysid = None
        self.parser = make_parser()
        self.counts = collections.Counter()
        self.stats = collections.Counter()

        self._pending = collections.deque()
        self._lock = threading.Lock()

    def queue_frame(self, frame):
        chunks = chunk_frame(frame)
        with self._lock:
            self._pending.extend(chunks)

    def next_chunk(self):
        with self._lock:
            return self._pending.popleft() if self._pending else None

    def observe(self, data):
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

    def label(self):
        return f"{self.address.hex()}@ch{self.channel}"

    def summary(self):
        who = f"sys{self.sysid}" if self.sysid is not None else "sys?"
        if not self.counts:
            return f"{who} no frames decoded yet"
        top = ", ".join(f"{n}x{c}" for n, c in self.counts.most_common(4))
        return f"{who} {top}"


class RadioGroup(threading.Thread):
    """One Crazyradio serving one or more vehicles by time-multiplexing.

    With a single vehicle the radio is tuned once and never touched again, so
    this costs nothing over a dedicated link. With several, the dongle retunes
    between polls -- one USB control transfer for the address, plus another for
    the channel if they differ, which is why sharing a channel is cheaper.
    """

    def __init__(self, index, vehicles, on_downlink, idle_poll_ms):
        super().__init__(daemon=True)

        self.index = index
        self.vehicles = vehicles
        self.on_downlink = on_downlink
        self.idle_poll_s = idle_poll_ms / 1000.0

        self.radio = Crazyradio2(index, warn_ambiguous=False)
        self.radio.set_ack_enabled(True)
        self.radio.set_large_packet_mode(True)

        dev = self.radio.dev
        self.usb_id = f"bus{dev.bus}.addr{dev.address}"

        self._tuned_channel = None
        self._tuned_rate = None
        self._tuned_address = None

        self.retunes = 0
        self._running = True

    def stop(self):
        self._running = False

    def _tune(self, vehicle):
        """Point the radio at one vehicle, touching only what changed."""
        changed = False

        if vehicle.channel != self._tuned_channel:
            self.radio.set_channel(vehicle.channel)
            self._tuned_channel = vehicle.channel
            changed = True

        if vehicle.rate != self._tuned_rate:
            self.radio.set_data_rate(vehicle.rate)
            self._tuned_rate = vehicle.rate
            changed = True

        if vehicle.address != self._tuned_address:
            self.radio.set_address(vehicle.address)
            self._tuned_address = vehicle.address
            changed = True

        if changed and len(self.vehicles) > 1:
            self.retunes += 1

    def run(self):
        poll_marker = bytes([MAVLINK_AIR_MARKER])

        while self._running:
            round_had_traffic = False

            for vehicle in self.vehicles:
                if not self._running:
                    break

                self._tune(vehicle)

                chunk = vehicle.next_chunk()
                if chunk is not None:
                    payload = poll_marker + chunk
                    vehicle.stats["up_chunks"] += 1
                    vehicle.stats["up_bytes"] += len(chunk)
                    round_had_traffic = True
                else:
                    payload = poll_marker

                ack = self.radio.send_packet(payload)
                vehicle.stats["polls"] += 1

                if ack is None:
                    vehicle.stats["usb_err"] += 1
                    time.sleep(0.01)
                    continue

                if not ack.ack:
                    # Out of range, powered down, or on another channel.
                    vehicle.stats["no_ack"] += 1
                    continue

                if ack.data and ack.data[0] == MAVLINK_AIR_MARKER:
                    down = ack.data[1:]
                    if down:
                        vehicle.stats["down_pkts"] += 1
                        vehicle.stats["down_bytes"] += len(down)
                        vehicle.observe(down)
                        self.on_downlink(vehicle, down)
                        round_had_traffic = True

            # Back off only when the whole round was quiet. Sleeping per
            # vehicle would make each idle drone delay the others.
            if not round_had_traffic:
                time.sleep(self.idle_poll_s)


class Router:
    """Decides which vehicle an uplink frame belongs to.

    The mapping is learned from downlink: whichever vehicle a system id was
    last heard from is where its commands go. Anything targeted at a system we
    have not heard from, or not targeted at all, goes to every vehicle --
    they ignore what is not addressed to them, so the cost is bandwidth rather
    than confusion. Duplicating everything unconditionally would instead make
    two vehicles answer the same parameter or FTP request.
    """

    def __init__(self):
        self._sysid_vehicle = {}
        self._parser = make_parser()
        self._lock = threading.Lock()

    def learn(self, sysid, vehicle):
        if sysid is None or sysid == 0:
            return
        with self._lock:
            self._sysid_vehicle[sysid] = vehicle

    def target_of(self, frame):
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

    def vehicles_for(self, frame, all_vehicles):
        target = self.target_of(frame)
        if target:
            with self._lock:
                vehicle = self._sysid_vehicle.get(target)
            if vehicle is not None:
                return [vehicle]
        return all_vehicles

    def known(self):
        with self._lock:
            return dict(self._sysid_vehicle)


def main():
    ap = argparse.ArgumentParser(
        description="Bridge MAVLink between Crazyflies over ESB and a GCS.")
    ap.add_argument("--uri", action="append", metavar="URI",
                    help=f"radio URI, repeatable for multiple vehicles "
                         f"(default {DEFAULT_URI}). URIs sharing a dongle "
                         f"index are round-robined on that dongle; different "
                         f"indices use a dongle each.")
    ap.add_argument("--udp", metavar="HOST:PORT",
                    help="forward MAVLink to this UDP endpoint, e.g. "
                         "127.0.0.1:14550 for QGroundControl. All vehicles "
                         "share it; the GCS separates them by system id.")
    ap.add_argument("--idle-poll-ms", type=float, default=2.0,
                    help="pause after a polling round in which no vehicle had "
                         "traffic (default 2.0)")
    ap.add_argument("--status-sec", type=float, default=2.0,
                    help="seconds between status lines (default 2.0)")
    ap.add_argument("--quiet", action="store_true",
                    help="suppress the periodic status line")
    args = ap.parse_args()

    uris = args.uri or [DEFAULT_URI]

    try:
        vehicles = [Vehicle(u) for u in uris]
    except ValueError as exc:
        print(f"Bad URI: {exc}")
        return 2

    # Group vehicles by the dongle that serves them, preserving URI order.
    by_index = collections.OrderedDict()
    for v in vehicles:
        by_index.setdefault(v.index, []).append(v)

    sock = None
    peer = None
    if args.udp:
        host, _, port = args.udp.partition(":")
        peer = (host, int(port))
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setblocking(False)
        sock.bind(("0.0.0.0", 0))

    router = Router()

    def on_downlink(vehicle, data):
        router.learn(frame_src_system(data), vehicle)
        if sock is not None and peer is not None:
            try:
                sock.sendto(data, peer)
            except OSError:
                pass

    groups = []
    for index, members in by_index.items():
        try:
            group = RadioGroup(index, members, on_downlink, args.idle_poll_ms)
        except Exception as exc:  # noqa: BLE001
            print(f"Could not open dongle index {index}: {exc}")
            print("Check it is plugged in and running the large-packet "
                  "firmware.")
            return 1
        groups.append(group)

        shared = " (round-robin)" if len(members) > 1 else ""
        print(f"Dongle {index} [{group.usb_id}]{shared}")
        for v in members:
            print(f"    {v.uri}")

    warnings = []

    # Two dongles on one frequency collide: no carrier sense, and the dongle
    # retries in a tight loop with no backoff, so a long ack can swallow a
    # whole retry burst.
    groups_per_channel = collections.Counter()
    for g in groups:
        for ch in {v.channel for v in g.vehicles}:
            groups_per_channel[ch] += 1
    shared = sorted(ch for ch, n in groups_per_channel.items() if n > 1)
    if shared:
        warnings.append(
            f"channel {shared[0]} is used by more than one dongle. ESB has no "
            f"carrier sense and no retry backoff, so they will collide. "
            f"Either give them separate channels, or put both vehicles on one "
            f"dongle by giving their URIs the same index.")

    # Retuning the channel costs a second control transfer per switch.
    for g in groups:
        if len({v.channel for v in g.vehicles}) > 1:
            warnings.append(
                f"dongle {g.index} round-robins across different channels, "
                f"which adds a control transfer per switch. Keeping shared "
                f"vehicles on one channel is cheaper -- and P2P between them "
                f"requires it anyway.")

    addrs = [v.address for v in vehicles]
    if len(addrs) != len(set(addrs)):
        warnings.append("two URIs use the same radio address, so they are "
                        "talking to the same vehicle.")

    for w in warnings:
        print(f"\nWARNING: {w}")

    if sock is not None:
        print(f"\nForwarding to {peer[0]}:{peer[1]} "
              f"(local port {sock.getsockname()[1]})")
        if len(vehicles) > 1:
            print("Vehicles must have distinct SYSID_THISMAV or the GCS will "
                  "merge them into one.")
    else:
        print("\nObserve only. Pass --udp 127.0.0.1:14550 to feed a GCS.")

    for g in groups:
        g.start()

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
                    for v in router.vehicles_for(frame, vehicles):
                        v.queue_frame(frame)
            else:
                time.sleep(0.01)

            now = time.time()
            if not args.quiet and now - last_status >= args.status_sec:
                elapsed = now - last_status
                print(f"[{time.strftime('%H:%M:%S')}]")
                for g in groups:
                    tag = ""
                    if len(g.vehicles) > 1:
                        tag = f"  retunes {g.retunes}"
                        g.retunes = 0
                    print(f"  dongle{g.index} [{g.usb_id}]{tag}")
                    for v in g.vehicles:
                        s = v.stats
                        print(f"    {v.label():<18} "
                              f"down {s['down_bytes']:5d}B/{s['down_pkts']:<4d} "
                              f"up {s['up_bytes']:4d}B/{s['up_chunks']:<3d} "
                              f"polls {s['polls']:4d} "
                              f"({s['polls'] / elapsed:4.0f}/s) "
                              f"no-ack {s['no_ack']:<4d} err {s['usb_err']}")
                        print(f"    {'':<18} {v.summary()}")
                        v.stats.clear()
                routed = router.known()
                if len(vehicles) > 1 and routed:
                    mapping = ", ".join(
                        f"sys{sid}->{v.address.hex()}"
                        for sid, v in sorted(routed.items()))
                    print(f"  uplink routing: {mapping}")
                last_status = now

    except KeyboardInterrupt:
        print("\nStopping...")
        for g in groups:
            g.stop()
        for g in groups:
            g.join(timeout=1.0)
        print("Stopped.")

    return 0


if __name__ == "__main__":
    sys.exit(main())

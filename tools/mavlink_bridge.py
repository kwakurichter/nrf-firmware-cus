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
import json
import select
import socket
import sys
import threading
import time

sys.path.insert(0, __file__.rsplit("/", 1)[0])

from crazyradio2_large import Crazyradio2, DR_250K, DR_1M, DR_2M, find_devices  # noqa: E402

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

        self._downlink_bytes = bytearray()
        self._pending = collections.deque()
        self._lock = threading.Lock()

        # When this address last acknowledged a poll; None until it first does.
        # A RadioGroup polls a silent address slowly (see SILENT_AFTER_S).
        self.last_ack = None
        self.silent = False
        self.next_slow_poll = 0.0

    def queue_frame(self, frame):
        chunks = chunk_frame(frame)
        with self._lock:
            queued = time.monotonic()
            self._pending.extend((queued, chunk) for chunk in chunks)

    def next_chunk(self):
        with self._lock:
            if not self._pending:
                return None
            queued, chunk = self._pending.popleft()
            self.stats["queue_wait_max_ms"] = max(
                self.stats["queue_wait_max_ms"], (time.monotonic() - queued) * 1000)
            return chunk

    def drop_stale(self, max_age):
        """Discard queued chunks older than max_age seconds.

        Chunks of one frame share a timestamp, so frames go whole.
        """
        with self._lock:
            cutoff = time.monotonic() - max_age
            dropped = 0
            while self._pending and self._pending[0][0] < cutoff:
                self._pending.popleft()
                dropped += 1
        if dropped:
            self.stats["dropped_stale_chunks"] += dropped

    def pending_status(self):
        with self._lock:
            age = time.monotonic() - self._pending[0][0] if self._pending else 0
            return {"chunks": len(self._pending), "oldest_ms": age * 1000}

    def downlink_frames(self, data):
        # Each aircraft owns a byte stream. Never interleave oversized-frame
        # fragments from separate aircraft in the GCS's single UDP parser.
        self.observe(data)
        self._downlink_bytes.extend(data)
        return split_frames(self._downlink_bytes)

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
                if (name == "HEARTBEAT" and msg.get_srcComponent() == 1
                        and msg.autopilot != 8 and msg.get_srcSystem() != 0):
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


class RadioFailure(RuntimeError):
    """The dongle stopped answering; the worker reopens it."""


# Consecutive failed bulk transfers before the dongle is treated as gone.
# Each failure sleeps 10 ms, so this is roughly half a second of silence.
USB_ERRORS_BEFORE_REOPEN = 50
REOPEN_BACKOFF_MAX_S = 2.0

# A vehicle sharing a dongle that has not acknowledged a poll for this long is
# treated as absent (switched off, out of range, wrong address) and polled only
# every SILENT_POLL_INTERVAL_S. A failed poll costs the dongle's whole retry
# burst, so polling an absent address at full rate starves the drones sharing
# its dongle. Its queued uplink older than SILENT_QUEUE_MAX_AGE_S is dropped.
SILENT_AFTER_S = 1.0
SILENT_POLL_INTERVAL_S = 0.25
SILENT_QUEUE_MAX_AGE_S = 2.0

# Dongles held by a RadioGroup in this process, keyed by USB bus and address,
# so a group reopening after a replug never grabs another group's dongle.
_claimed_lock = threading.Lock()
_claimed = set()


def _usb_key(dev):
    return (dev.bus, dev.address)


class RadioGroup(threading.Thread):
    """One Crazyradio serving one or more vehicles by time-multiplexing.

    With a single vehicle the radio is tuned once and never touched again, so
    this costs nothing over a dedicated link. With several, the dongle retunes
    between polls -- one USB control transfer for the address, plus another for
    the channel if they differ, which is why sharing a channel is cheaper.

    A USB failure never ends the thread. A control-transfer exception or a run
    of failed bulk transfers closes the dongle and reopens the first Crazyradio
    no other group holds, retrying with backoff until one appears. Tuning lives
    here rather than in the dongle, so which physical dongle comes back does
    not matter. `on_event` receives a one-line description of each failure and
    recovery.
    """

    def __init__(self, index, vehicles, on_downlink, idle_poll_ms, on_event=None):
        super().__init__(daemon=True)

        self.index = index
        self.vehicles = vehicles
        self.on_downlink = on_downlink
        self.on_event = on_event or (lambda text: None)
        self.idle_poll_s = idle_poll_ms / 1000.0

        self.radio = None
        self.usb_id = None
        self._open(Crazyradio2(index, warn_ambiguous=False))

        self.retunes = 0
        self.failures = 0
        self.reopens = 0
        self.last_failure = None
        self._usb_error_streak = 0
        self._created_at = time.monotonic()
        self._running = True

    def _open(self, radio):
        radio.set_ack_enabled(True)
        radio.set_large_packet_mode(True)
        self.radio = radio
        dev = radio.dev
        self.usb_id = f"bus{dev.bus}.addr{dev.address}"
        with _claimed_lock:
            _claimed.add(_usb_key(dev))
        # A fresh or replugged dongle holds none of our settings.
        self._tuned_channel = None
        self._tuned_rate = None
        self._tuned_address = None

    def _close(self):
        if self.radio is None:
            return
        with _claimed_lock:
            _claimed.discard(_usb_key(self.radio.dev))
        close = getattr(self.radio, "close", None)
        if close is not None:
            close()

    def _reopen(self):
        """Open the first unclaimed Crazyradio; return True on success."""
        with _claimed_lock:
            held = set(_claimed)
        for dev in find_devices():
            if _usb_key(dev) in held:
                continue
            try:
                self._open(Crazyradio2(device=dev))
                return True
            except Exception:  # noqa: BLE001
                continue
        return False

    def _recover(self, exc):
        self.failures += 1
        self.last_failure = f"{type(exc).__name__}: {exc}"
        self.on_event(f"dongle {self.index} [{self.usb_id}] failed ({self.last_failure}); reopening")
        self._close()
        self.radio = None
        delay = 0.25
        while self._running:
            time.sleep(delay)
            try:
                if self._reopen():
                    self.reopens += 1
                    self._usb_error_streak = 0
                    self.on_event(f"dongle {self.index} reopened as [{self.usb_id}]")
                    return
            except Exception:  # noqa: BLE001
                pass
            delay = min(delay * 2, REOPEN_BACKOFF_MAX_S)

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
        while self._running:
            try:
                self._poll_round()
            except Exception as exc:  # noqa: BLE001
                if not self._running:
                    break
                self._recover(exc)

    def _deliver(self, vehicle, frame):
        # A consumer bug must not be mistaken for a dongle failure.
        try:
            self.on_downlink(vehicle, frame)
        except Exception as exc:  # noqa: BLE001
            vehicle.stats["downlink_handler_errors"] += 1
            self.on_event(f"downlink handler error for {vehicle.uri}: {type(exc).__name__}: {exc}")

    def _skip_silent(self, vehicle, now):
        """True when an absent vehicle on a shared dongle is not due a poll."""
        if len(self.vehicles) < 2:
            return False
        reference = vehicle.last_ack if vehicle.last_ack is not None else self._created_at
        silent = now - reference > SILENT_AFTER_S
        if silent != vehicle.silent:
            vehicle.silent = silent
            if silent:
                self.on_event(f"{vehicle.uri}: no ack for {now - reference:.1f} s; polling it "
                              f"{1 / SILENT_POLL_INTERVAL_S:.0f}x/s until it answers")
            else:
                self.on_event(f"{vehicle.uri}: answering again; full polling rate")
        if not silent:
            return False
        vehicle.drop_stale(SILENT_QUEUE_MAX_AGE_S)
        if now < vehicle.next_slow_poll:
            vehicle.stats["skipped_polls"] += 1
            return True
        vehicle.next_slow_poll = now + SILENT_POLL_INTERVAL_S
        return False

    def _poll_round(self):
        poll_marker = bytes([MAVLINK_AIR_MARKER])
        round_had_traffic = False

        for vehicle in self.vehicles:
            if not self._running:
                break

            poll_started = time.monotonic()
            if self._skip_silent(vehicle, poll_started):
                continue
            self._tune(vehicle)
            vehicle.stats["tune_max_ms"] = max(
                vehicle.stats["tune_max_ms"], (time.monotonic() - poll_started) * 1000)

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
            vehicle.stats["poll_max_ms"] = max(
                vehicle.stats["poll_max_ms"], (time.monotonic() - poll_started) * 1000)

            if ack is None:
                vehicle.stats["usb_err"] += 1
                self._usb_error_streak += 1
                if self._usb_error_streak >= USB_ERRORS_BEFORE_REOPEN:
                    raise RadioFailure(f"{self._usb_error_streak} consecutive USB errors, "
                                       f"last {self.radio.last_error}")
                time.sleep(0.01)
                continue

            self._usb_error_streak = 0
            vehicle.stats["retries"] += ack.retry
            if self.radio.read_duration_ms >= 900:
                vehicle.stats["slow_usb_reads"] += 1
                if self.radio.answer_length % 64 == 0:
                    vehicle.stats["slow_usb_reads_multiple_64"] += 1

            if not ack.ack:
                # Out of range, powered down, or on another channel.
                vehicle.stats["no_ack"] += 1
                continue

            vehicle.last_ack = time.monotonic()

            if ack.data and ack.data[0] == MAVLINK_AIR_MARKER:
                down = ack.data[1:]
                if down:
                    vehicle.stats["down_pkts"] += 1
                    vehicle.stats["down_bytes"] += len(down)
                    for frame in vehicle.downlink_frames(down):
                        self._deliver(vehicle, frame)
                        vehicle.stats["down_frames"] += 1
                    round_had_traffic = True

        # Back off only when the whole round was quiet. Sleeping per
        # vehicle would make each idle drone delay the others.
        if not round_had_traffic:
            time.sleep(self.idle_poll_s)


class Router:
    """Decides which vehicle an uplink frame belongs to.

    The mapping is learned from CRC-validated autopilot heartbeats. Conflicting
    system IDs are not eligible for targeted commands; restart after fixing IDs. Anything targeted at a system we
    have not heard from, or not targeted at all, goes to every vehicle --
    they ignore what is not addressed to them, so the cost is bandwidth rather
    than confusion. Duplicating everything unconditionally would instead make
    two vehicles answer the same parameter or FTP request.
    """

    def __init__(self):
        self._sysid_vehicle = {}
        self._parser = make_parser()
        self._conflicts = set()
        self._lock = threading.Lock()

    def learn(self, sysid, vehicle):
        if sysid is None or sysid == 0:
            return
        with self._lock:
            previous = self._sysid_vehicle.get(sysid)
            if previous is not None and previous is not vehicle:
                self._conflicts.add(sysid)
                return
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
                if target in self._conflicts:
                    return []
                vehicle = self._sysid_vehicle.get(target)
            if vehicle is not None:
                return [vehicle]
        return all_vehicles

    def known(self):
        with self._lock:
            return dict(self._sysid_vehicle)


def configuration_warnings(by_index):
    """Radio-plan problems for {dongle index: [Vehicle, ...]}, as sentences."""
    warnings = []

    # Two dongles on one frequency collide: no carrier sense, and the dongle
    # retries in a tight loop with no backoff, so a long ack can swallow a
    # whole retry burst.
    groups_per_channel = collections.Counter()
    for members in by_index.values():
        for ch in {v.channel for v in members}:
            groups_per_channel[ch] += 1
    shared = sorted(ch for ch, n in groups_per_channel.items() if n > 1)
    if shared:
        warnings.append(
            f"channel {shared[0]} is used by more than one dongle. ESB has no "
            f"carrier sense and no retry backoff, so they will collide. "
            f"Either give them separate channels, or put both vehicles on one "
            f"dongle by giving their URIs the same index.")

    # Retuning the channel costs a second control transfer per switch.
    for index, members in by_index.items():
        if len({v.channel for v in members}) > 1:
            warnings.append(
                f"dongle {index} round-robins across different channels, "
                f"which adds a control transfer per switch. Keeping shared "
                f"vehicles on one channel is cheaper -- and P2P between them "
                f"requires it anyway.")

    addrs = [v.address for members in by_index.values() for v in members]
    if len(addrs) != len(set(addrs)):
        warnings.append("two URIs use the same radio address, so they are "
                        "talking to the same vehicle.")
    return warnings


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
    ap.add_argument("--stats-jsonl", metavar="PATH",
                    help="write cumulative per-vehicle counters and queue/timing diagnostics to a new JSONL file")
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
        router.learn(vehicle.sysid, vehicle)
        if sock is not None and peer is not None:
            try:
                sock.sendto(data, peer)
            except OSError:
                pass

    groups = []
    for index, members in by_index.items():
        try:
            group = RadioGroup(index, members, on_downlink, args.idle_poll_ms,
                               on_event=lambda text: print(f"\n{text}", flush=True))
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

    warnings = configuration_warnings(by_index)

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

    stats_file = open(args.stats_jsonl, "x", buffering=1) if args.stats_jsonl else None
    for g in groups:
        g.start()

    uplink_bytes = bytearray()
    last_status = time.monotonic()
    previous_stats = {}

    print("\nPolling. Ctrl-C to stop.\n")

    try:
        while True:
            if sock is not None:
                # Sleep when UDP is idle and bound each burst so diagnostics
                # and radio threads still run under sustained GCS traffic.
                select.select([sock], [], [], 0.01)
                for _ in range(64):
                    try:
                        data, src = sock.recvfrom(65535)
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

            now = time.monotonic()
            if now - last_status >= args.status_sec:
                elapsed = now - last_status
                for g in groups:
                    for v in g.vehicles:
                        # Cumulative counters avoid clearing a worker's updates.
                        totals = dict(v.stats)
                        old = previous_stats.get(v.uri, {})
                        delta = {key: value - old.get(key, 0) for key, value in totals.items()}
                        previous_stats[v.uri] = totals
                        pending = v.pending_status()
                        if stats_file:
                            stats_file.write(json.dumps({
                                "monotonic": now, "dongle": g.index, "usb_id": g.usb_id,
                                "uri": v.uri, "sysid": v.sysid, "counters": totals,
                                "queue": pending, "last_usb_error": g.radio.last_error,
                                "last_slow_usb_read": g.radio.last_slow_read, "worker_alive": g.is_alive(),
                                "routing_conflicts": sorted(router._conflicts),
                            }) + "\n")
                        if not args.quiet:
                            print(f"dongle{g.index} {v.label()} sys{v.sysid} "
                                  f"polls {delta.get('polls', 0) / elapsed:.0f}/s "
                                  f"down {delta.get('down_bytes', 0) / elapsed:.0f}B/s "
                                  f"no-ack {delta.get('no_ack', 0)} USB errors {delta.get('usb_err', 0)} "
                                  f"queue {pending['chunks']} ({pending['oldest_ms']:.0f}ms) "
                                  f"max poll {totals.get('poll_max_ms', 0):.1f}ms "
                                  f"max tune {totals.get('tune_max_ms', 0):.1f}ms "
                                  f"slow USB reads {delta.get('slow_usb_reads', 0)}", flush=True)
                            if delta.get('usb_err', 0):
                                print(f"  last USB error: {g.radio.last_error}", flush=True)
                last_status = now

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        for g in groups:
            g.stop()
        for g in groups:
            g.join(timeout=1.0)
        if stats_file:
            stats_file.close()
        if sock:
            sock.close()
        print("Stopped.")

    return 0


if __name__ == "__main__":
    sys.exit(main())

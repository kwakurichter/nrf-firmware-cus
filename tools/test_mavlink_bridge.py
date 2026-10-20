#!/usr/bin/env python3
"""Offline bridge regression tests. No radio or flight connection is opened."""
# AP_FLAKE8_CLEAN
import time
import unittest
from unittest.mock import Mock, patch

from pymavlink.dialects.v20 import ardupilotmega as mav
import usb.core

import crazyradio2_large as driver
import mavlink_bridge as bridge


def pack(message, sid=1):
    return message.pack(mav.MAVLink(None, srcSystem=sid, srcComponent=1))


def heartbeat(sid):
    return pack(mav.MAVLink_heartbeat_message(2, 3, 0, 4, 3, 3), sid)


def vehicle(sid):
    return bridge.Vehicle(f'radio://{(sid - 1) // 2}/{80 if sid < 3 else 90}/2M/E7E7E7E7{sid:02X}')


class BridgeTests(unittest.TestCase):
    def test_four_interleaved_large_frames_preserve_bytes(self):
        vs = [vehicle(sid) for sid in range(1, 5)]
        frames = [pack(mav.MAVLink_file_transfer_protocol_message(0, 255, 190, list(range(251))), sid)
                  for sid in range(1, 5)]
        output = []
        for i in range(2):
            for v, frame in zip(vs, frames):
                output.extend(v.downlink_frames(frame[i * 251:(i + 1) * 251]))
        self.assertEqual(output, frames)
        decoded = bridge.make_parser().parse_buffer(b''.join(output))
        self.assertEqual([m.get_srcSystem() for m in decoded], [1, 2, 3, 4])
        self.assertTrue(all(m.get_type() == 'FILE_TRANSFER_PROTOCOL' for m in decoded))

    def test_fragment_does_not_invent_system_id(self):
        v = vehicle(1)
        frame = pack(mav.MAVLink_file_transfer_protocol_message(0, 255, 190, list(range(251))))
        self.assertEqual(v.downlink_frames(frame[:251]), [])
        self.assertIsNone(v.sysid)
        self.assertEqual(v.downlink_frames(frame[251:]), [frame])
        self.assertIsNone(v.sysid)
        self.assertEqual(v.downlink_frames(heartbeat(1)), [heartbeat(1)])
        self.assertEqual(v.sysid, 1)

    def test_bad_crc_does_not_learn_id(self):
        v = vehicle(1)
        damaged = bytearray(heartbeat(208))
        damaged[-1] ^= 1
        v.downlink_frames(damaged)
        self.assertIsNone(v.sysid)
        v.downlink_frames(heartbeat(1))
        self.assertEqual(v.sysid, 1)

    def test_coalesced_frames_and_unknown_message_are_preserved(self):
        v = vehicle(1)
        unknown = bytes([253, 1, 0, 0, 1, 1, 1, 254, 254, 254, 0, 0, 0])
        frames = [heartbeat(1), unknown, heartbeat(1)]
        self.assertEqual(v.downlink_frames(b''.join(frames)), frames)

    def test_routing_four_systems_and_broadcast(self):
        router = bridge.Router()
        vs = [vehicle(sid) for sid in range(1, 5)]
        self.assertEqual([v.index for v in vs], [0, 0, 1, 1])
        for sid, v in enumerate(vs, 1):
            v.downlink_frames(heartbeat(sid))
            router.learn(v.sysid, v)
        for sid, v in enumerate(vs, 1):
            command = pack(mav.MAVLink_command_long_message(sid, 1, 512, 0, 0, 0, 0, 0, 0, 0, 0), 255)
            self.assertEqual(router.vehicles_for(command, vs), [v])
        self.assertEqual(router.vehicles_for(heartbeat(255), vs), vs)

    def test_conflicting_system_ids_do_not_route_commands(self):
        router = bridge.Router()
        vs = [vehicle(1), vehicle(2)]
        for v in vs:
            router.learn(1, v)
        command = pack(mav.MAVLink_command_long_message(1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0), 255)
        self.assertEqual(router.vehicles_for(command, vs), [])

    def test_queue_preserves_fragment_order_and_reports_age(self):
        v = vehicle(1)
        frame = bytes(range(256))
        with patch.object(bridge.time, 'monotonic', return_value=10):
            v.queue_frame(frame)
        with patch.object(bridge.time, 'monotonic', return_value=10.5):
            self.assertEqual(v.pending_status(), {'chunks': 2, 'oldest_ms': 500})
            self.assertEqual(v.next_chunk(), frame[:251])
            self.assertEqual(v.next_chunk(), frame[251:])
            self.assertIsNone(v.next_chunk())
            self.assertEqual(v.pending_status(), {'chunks': 0, 'oldest_ms': 0})
        self.assertEqual(v.stats['queue_wait_max_ms'], 500)

    def test_four_vehicle_workers_forward_complete_frames(self):
        output = []
        vs = [vehicle(sid) for sid in range(1, 5)]
        groups = []
        for index in (0, 1):
            radio = Mock()
            radio.dev.bus, radio.dev.address = 1, index
            radio.read_duration_ms = 0
            group = None
            count = [0]

            def send(data):
                sid = 2 * index + count[0] % 2 + 1
                count[0] += 1
                if count[0] == 4:
                    group.stop()
                return driver.Ack(bytes([1, 0xE0]) + heartbeat(sid))

            radio.send_packet.side_effect = send
            with patch.object(bridge, 'Crazyradio2', return_value=radio):
                group = bridge.RadioGroup(index, vs[index * 2:index * 2 + 2],
                                          lambda v, frame: output.append((v.index, frame)), 0)
            group.run()
            groups.append(group)
        self.assertEqual(len(output), 8)
        self.assertEqual([v.sysid for v in vs], [1, 2, 3, 4])
        self.assertEqual([v.stats['polls'] for v in vs], [2, 2, 2, 2])


class DriverTests(unittest.TestCase):
    def make_radio(self):
        radio = driver.Crazyradio2.__new__(driver.Crazyradio2)
        radio.dev = Mock()
        radio.last_error = radio.last_slow_read = None
        return radio

    def test_partial_timeout_without_exception_is_visible(self):
        radio = self.make_radio()
        radio.dev.read.return_value = bytes([1]) + bytes(63)
        with patch.object(driver.time, 'monotonic', side_effect=[10, 11.001]):
            ack = radio.send_packet(b'\xe0')
        self.assertTrue(ack.ack)
        self.assertEqual(radio.last_slow_read['length'], 64)
        self.assertGreater(radio.last_slow_read['elapsed_ms'], 1000)

    def test_outgoing_64_byte_boundary_gets_terminator(self):
        radio = self.make_radio()
        radio.dev.read.return_value = b'\x01'
        radio.send_packet(bytes(64))
        self.assertEqual(radio.dev.write.call_count, 2)
        self.assertEqual(radio.dev.write.call_args.args[1], b'')

    def test_usb_error_reports_stage(self):
        radio = self.make_radio()
        radio.dev.read.side_effect = usb.core.USBError('timeout', errno=110)
        self.assertIsNone(radio.send_packet(b'\xe0'))
        self.assertEqual(radio.last_error['stage'], 'bulk IN')


class RecoveryTests(unittest.TestCase):
    """The polling thread survives USB failures and reopens a free dongle."""

    def setUp(self):
        bridge._claimed.clear()

    def make_radio(self, bus, address, send):
        radio = Mock()
        radio.dev.bus, radio.dev.address = bus, address
        radio.read_duration_ms = 0
        radio.last_error = None
        radio.send_packet.side_effect = send
        return radio

    def run_group(self, radios, devices, stop_after, on_downlink=None):
        events = []
        opened = iter(radios)
        with patch.object(bridge, 'Crazyradio2', side_effect=lambda *a, **k: next(opened)), \
                patch.object(bridge, 'find_devices', return_value=devices), \
                patch.object(bridge.time, 'sleep'):
            group = bridge.RadioGroup(0, [vehicle(1)], on_downlink or (lambda v, f: None), 0,
                                      on_event=events.append)
            stop_after.append(group)
            group.run()
        return group, events

    def test_control_transfer_error_reopens_unclaimed_dongle(self):
        holder = []
        good_sends = [0]

        def good(data):
            good_sends[0] += 1
            if good_sends[0] == 3:
                holder[0].stop()
            return driver.Ack(bytes([1, 0xE0]) + heartbeat(1))

        first = self.make_radio(1, 5, good)
        first.set_channel.side_effect = usb.core.USBError('pipe', errno=32)
        second = self.make_radio(1, 9, good)
        other_group_dev = Mock(bus=1, address=7)
        bridge._claimed.add((1, 7))
        replugged = Mock(bus=1, address=9)
        group, events = self.run_group([first, second], [other_group_dev, replugged], holder)
        self.assertEqual(group.usb_id, 'bus1.addr9')
        self.assertEqual((group.failures, group.reopens), (1, 1))
        self.assertIn('USBError', events[0])
        self.assertIn('reopened', events[1])
        self.assertEqual(good_sends[0], 3)
        self.assertNotIn((1, 5), bridge._claimed)

    def test_run_of_usb_errors_reopens(self):
        holder = []
        calls = [0]

        def failing(data):
            calls[0] += 1
            return None

        def good(data):
            holder[0].stop()
            return driver.Ack(bytes([1, 0xE0]))

        first = self.make_radio(1, 5, failing)
        second = self.make_radio(1, 6, good)
        group, events = self.run_group([first, second], [Mock(bus=1, address=6)], holder)
        self.assertEqual(calls[0], bridge.USB_ERRORS_BEFORE_REOPEN)
        self.assertEqual(group.reopens, 1)
        self.assertIn('consecutive USB errors', events[0])

    def test_downlink_handler_error_does_not_reopen(self):
        holder = []
        sends = [0]

        def good(data):
            sends[0] += 1
            if sends[0] == 2:
                holder[0].stop()
            return driver.Ack(bytes([1, 0xE0]) + heartbeat(1))

        def broken(v, frame):
            raise KeyError('consumer bug')

        group, events = self.run_group([self.make_radio(1, 5, good)], [], holder, broken)
        self.assertEqual(group.failures, 0)
        self.assertEqual(group.vehicles[0].stats['downlink_handler_errors'], 2)
        self.assertIn('downlink handler error', events[0])


class SilentVehicleTests(unittest.TestCase):
    """Field case 2026-10-20: an unpowered fourth address halved its dongle partner's polls."""

    def run_shared(self, duration, answers):
        """Two vehicles on one mock dongle. answers(address, now) -> bool."""
        clock = [0.0]
        tuned = [None]
        events = []
        live = vehicle(3)
        absent = vehicle(4)
        absent.index = live.index = 0
        radio = Mock()
        radio.dev.bus, radio.dev.address = 1, 1
        radio.read_duration_ms = 0
        radio.last_error = None
        radio.set_address.side_effect = lambda address: tuned.__setitem__(0, bytes(address))
        group = None

        def send(payload):
            if clock[0] >= duration:
                group.stop()
            if answers(tuned[0], clock[0]):
                clock[0] += 0.004
                return driver.Ack(bytes([1, 0xE0]))
            clock[0] += 0.012  # a failed poll costs the whole retry burst
            return driver.Ack(bytes([0]))

        radio.send_packet.side_effect = send
        with patch.object(bridge, 'Crazyradio2', return_value=radio), \
                patch.object(bridge.time, 'monotonic', side_effect=lambda: clock[0]), \
                patch.object(bridge.time, 'sleep', side_effect=lambda s: clock.__setitem__(0, clock[0] + s)):
            group = bridge.RadioGroup(0, [live, absent], lambda v, f: None, 2, on_event=events.append)
            group.run()
        return live, absent, events, clock

    def test_absent_address_is_polled_slowly_and_partner_gets_the_link(self):
        live, absent, events, _ = self.run_shared(10, lambda address, now: address == live_address)
        # After the first second, four polls a second for the absent address.
        self.assertLess(absent.stats['polls'], 1 / 0.016 + 4 * 9 + 5)
        # Polling the absent address at full rate would leave ~555 (4 + 12 + 2 ms rounds).
        self.assertGreater(live.stats['polls'], 1400)
        self.assertTrue(absent.silent)
        self.assertFalse(live.silent)
        self.assertEqual(len(events), 1)
        self.assertIn('E7E7E7E704: NO ACK', events[0].upper())

    def test_returning_vehicle_gets_full_rate_within_a_slow_poll(self):
        live, absent, events, _ = self.run_shared(
            6, lambda address, now: address == live_address or now >= 3)
        self.assertFalse(absent.silent)
        self.assertIn('answering again', events[-1])
        # Back at full rate for the last ~2.75 s: shares the dongle evenly with its partner.
        self.assertGreater(absent.stats['polls'], 300)

    def test_absent_vehicle_queue_does_not_grow_without_bound(self):
        clock = [100.0]
        v = vehicle(4)
        with patch.object(bridge.time, 'monotonic', side_effect=lambda: clock[0]):
            for _ in range(10):
                v.queue_frame(bytes(300))  # two chunks each
                clock[0] += 0.5
            v.drop_stale(bridge.SILENT_QUEUE_MAX_AGE_S)
            self.assertEqual(v.pending_status()['chunks'], 2 * 4)
        self.assertEqual(v.stats['dropped_stale_chunks'], 12)

    def test_single_vehicle_dongle_is_never_slowed(self):
        clock = [0.0]
        v = vehicle(1)
        radio = Mock()
        radio.dev.bus, radio.dev.address = 1, 2
        radio.read_duration_ms = 0
        group = None

        def send(payload):
            clock[0] += 0.012
            if clock[0] > 5:
                group.stop()
            return driver.Ack(bytes([0]))

        radio.send_packet.side_effect = send
        with patch.object(bridge, 'Crazyradio2', return_value=radio), \
                patch.object(bridge.time, 'monotonic', side_effect=lambda: clock[0]), \
                patch.object(bridge.time, 'sleep', side_effect=lambda s: clock.__setitem__(0, clock[0] + s)):
            group = bridge.RadioGroup(0, [v], lambda vv, f: None, 2)
            group.run()
        self.assertGreater(v.stats['polls'], 300)
        self.assertFalse(v.silent)

    def test_groups_start_as_real_threads(self):
        """Regression: an attribute named like a Thread internal broke start()."""
        radio = Mock()
        radio.dev.bus, radio.dev.address = 1, 3
        radio.read_duration_ms = 0
        radio.send_packet.return_value = driver.Ack(bytes([1, 0xE0]))
        with patch.object(bridge, 'Crazyradio2', return_value=radio):
            groups = [bridge.RadioGroup(0, [vehicle(1), vehicle(2)], lambda v, f: None, 2),
                      bridge.RadioGroup(1, [vehicle(3)], lambda v, f: None, 2)]
        for group in groups:
            group.start()
        time.sleep(0.05)
        for group in groups:
            self.assertTrue(group.is_alive())
            group.stop()
            group.join(timeout=1)
            self.assertFalse(group.is_alive())
        self.assertGreater(groups[0].vehicles[0].stats['polls'], 0)


live_address = bytes.fromhex('E7E7E7E703')


if __name__ == '__main__':
    unittest.main()

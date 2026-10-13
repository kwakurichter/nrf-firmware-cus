#!/usr/bin/env python3
"""Offline bridge regression tests. No radio or flight connection is opened."""
# AP_FLAKE8_CLEAN
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


if __name__ == '__main__':
    unittest.main()

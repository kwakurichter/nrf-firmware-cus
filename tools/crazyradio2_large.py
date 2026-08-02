#!/usr/bin/env python3
"""Minimal Crazyradio 2.0 driver with large packet support.

cflib cannot be used for this. Its Crazyradio.send_packet() reads the bulk IN
endpoint with a hardcoded 64 byte length, so it truncates anything larger no
matter what the dongle sends. This driver requests the full transfer instead
and lets libusb reassemble it from 64 byte USB packets.

Large packet mode is an opt-in vendor request (0x27). Without it the dongle
behaves exactly like a legacy Crazyradio: 32 byte cap, and a >32 byte transfer
with acks disabled means "two packets back to back" rather than one long one.
Those two behaviours are mutually exclusive, which is why the mode exists
rather than the cap simply being raised.

Run directly for a self-test:

    .venv/bin/python tools/crazyradio2_large.py [channel] [address]
"""

import sys

import usb.core
import usb.util

CRAZYRADIO_VID = 0x1915
CRAZYRADIO_PID = 0x7777

EP_OUT = 0x01
EP_IN = 0x81

# Bulk endpoint max packet size, from the dongle's descriptor.
EP_MAX_PACKET = 64

# Vendor requests
SET_RADIO_CHANNEL = 0x01
SET_RADIO_ADDRESS = 0x02
SET_DATA_RATE = 0x03
ACK_ENABLE = 0x10
SET_LARGE_PACKET_MODE = 0x27

DR_250K, DR_1M, DR_2M = 0, 1, 2

# 252 byte ESB payload plus the one byte answer header.
MAX_ANSWER = 253


class Ack:
    """Result of a transmission. `data` is the ack payload, if any."""

    def __init__(self, raw):
        self.ack = bool(raw[0] & 0x01)
        self.power_detector = bool(raw[0] & 0x02)
        self.retry = raw[0] >> 4
        self.data = bytes(raw[1:])

    def __repr__(self):
        return (f"Ack(ack={self.ack}, retry={self.retry}, "
                f"len={len(self.data)})")


class Crazyradio2:
    def __init__(self, index=0):
        devices = list(usb.core.find(find_all=True,
                                     idVendor=CRAZYRADIO_VID,
                                     idProduct=CRAZYRADIO_PID))
        if not devices:
            raise RuntimeError("No Crazyradio found")
        if index >= len(devices):
            raise RuntimeError(
                f"Crazyradio index {index} requested, {len(devices)} present")
        if len(devices) > 1:
            print(f"warning: {len(devices)} dongles present, using index "
                  f"{index}. Which one that is depends on enumeration order, "
                  f"which is not stable across replug.")

        self.dev = devices[index]
        self.dev.set_configuration()

    def _vendor_out(self, request, value=0, index=0, data=None):
        # 0x40 = host-to-device | vendor | device
        self.dev.ctrl_transfer(0x40, request, value, index, data or b"")

    def set_channel(self, channel):
        self._vendor_out(SET_RADIO_CHANNEL, channel)

    def set_data_rate(self, rate):
        self._vendor_out(SET_DATA_RATE, rate)

    def set_address(self, address):
        if len(address) != 5:
            raise ValueError("Address must be 5 bytes")
        self._vendor_out(SET_RADIO_ADDRESS, 0, 0, bytes(address))

    def set_ack_enabled(self, enabled):
        self._vendor_out(ACK_ENABLE, 1 if enabled else 0)

    def set_large_packet_mode(self, enabled):
        """Lift the legacy 32 byte cap to the full 252 byte ESB payload.

        Also disables the legacy two-packets-per-transfer broadcast quirk,
        which cannot coexist with single large packets.
        """
        self._vendor_out(SET_LARGE_PACKET_MODE, 1 if enabled else 0)

    def send_packet(self, data, timeout=1000):
        """Send a packet and return the Ack, or None on USB timeout."""
        data = bytes(data)
        try:
            self.dev.write(EP_OUT, data, timeout)

            # A bulk transfer whose length is an exact multiple of the
            # endpoint packet size contains no short packet, so the dongle
            # cannot tell it has ended and keeps accumulating forever. A zero
            # length packet terminates it. libusb does not do this for us.
            # Never came up under the legacy protocol, where packets capped at
            # 32 bytes; at 252 it bites on 64, 128 and 192.
            if data and len(data) % EP_MAX_PACKET == 0:
                self.dev.write(EP_OUT, b"", timeout)

            # Request the full length so libusb reassembles multi-packet
            # transfers. This is the line cflib gets wrong.
            raw = self.dev.read(EP_IN, MAX_ANSWER, timeout)
        except usb.core.USBError:
            return None

        if raw is None or len(raw) < 1:
            return None

        return Ack(raw)


def _self_test(channel, address):
    radio = Crazyradio2()
    radio.set_channel(channel)
    radio.set_data_rate(DR_2M)
    radio.set_address(address)
    radio.set_ack_enabled(True)
    radio.set_large_packet_mode(True)

    print(f"Crazyradio open. channel={channel} rate=2M "
          f"address={bytes(address).hex()} large_packet_mode=on\n")

    # 0xE0 is the MAVLink transport's on-air marker, so these land in the
    # Crazyflie's MAVLink path and get forwarded to the STM32 over syslink.
    sizes = [4, 32, 33, 64, 128, 251, 252]
    failures = 0

    for size in sizes:
        payload = bytes([0xE0] + [(0x40 + ((i * 7) & 0x7F))
                                  for i in range(1, size)])
        ack = radio.send_packet(payload)

        if ack is None:
            print(f"size {size:3d}: USB timeout -- no answer from the dongle")
            failures += 1
        elif not ack.ack:
            print(f"size {size:3d}: NOT ACKED -- the dongle transmitted, but "
                  f"the Crazyflie did not acknowledge")
            failures += 1
        else:
            print(f"size {size:3d}: acked after {ack.retry} retries, "
                  f"ack payload {len(ack.data)} bytes")

    print()
    if failures:
        print(f"{failures}/{len(sizes)} sizes failed.")
    else:
        print("All sizes acked. Host -> USB -> dongle -> air -> Crazyflie "
              "works at up to 252 bytes.")
        print("Note this only proves the uplink. The ack payloads are "
              "whatever the Crazyflie had queued, so the >32 byte DOWNLINK "
              "path is still unproven until the STM32 sends MAVLink.")

    return 0 if failures == 0 else 2


if __name__ == "__main__":
    ch = int(sys.argv[1]) if len(sys.argv) > 1 else 100
    addr = bytes.fromhex(sys.argv[2]) if len(sys.argv) > 2 \
        else bytes.fromhex("E7E7E7E706")
    sys.exit(_self_test(ch, addr))

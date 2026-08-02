/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * mavlink_transport.c - Carries a MAVLink byte stream over ESB
 *
 * Deliberately does not parse MAVLink. With a 252 byte radio payload almost
 * every frame fits in a single packet, and the far end runs a byte-stream
 * parser that resyncs on STX anyway, so there is nothing for the nRF to gain
 * by understanding the contents. It stays a pipe.
 */

#include <string.h>

#include "mavlink_transport.h"

static MavlinkMode mode = mavlinkModeTelemetry;

void mavlinkTransportSetMode(MavlinkMode newMode)
{
  mode = newMode;
}

MavlinkMode mavlinkTransportGetMode(void)
{
  return mode;
}

bool mavlinkTransportSend(const uint8_t *data, uint8_t length)
{
  if (length > MAVLINK_TRANSPORT_MTU) {
    return false;
  }

  if (mode == mavlinkModeP2P) {
    // Broadcasts are unacked and go out immediately, so there is no queue to
    // fill and nothing to report back other than the length check above.
    esbSendBroadcast(MAVLINK_AIR_MARKER, data, length);
    return true;
  }

  // Telemetry mode. The Crazyflie is a PRX, so this only queues the chunk --
  // it goes out as an ack payload the next time the ground station polls.
  if (!esbCanTxPacket()) {
    return false;
  }

  EsbPacket *packet = esbGetTxPacket();
  if (packet == NULL) {
    return false;
  }

  packet->data[0] = MAVLINK_AIR_MARKER;
  memcpy(&packet->data[1], data, length);
  packet->size = length + 1;

  esbSendTxPacket();

  return true;
}

bool mavlinkTransportReceive(const EsbPacket *packet,
                             const uint8_t **payload,
                             uint8_t *payloadLength)
{
  if (packet->size < 1 || packet->data[0] != MAVLINK_AIR_MARKER) {
    return false;
  }

  *payload = &packet->data[1];
  *payloadLength = packet->size - 1;

  return true;
}

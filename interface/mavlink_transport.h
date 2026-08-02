/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * mavlink_transport.h - Carries a MAVLink byte stream over ESB
 */
#ifndef __MAVLINK_TRANSPORT_H__
#define __MAVLINK_TRANSPORT_H__

#include <stdbool.h>
#include <stdint.h>

#include "esb.h"

/* On-air marker.
 *
 * MAVLink chunks share the air with the CRTP-derived control packets this
 * firmware still answers locally (null packets, bootloader commands, radio
 * commands), all of which are matched on (data[0] & 0xf3) == 0xf3. A chunk of
 * an opaque byte stream can easily start with such a byte, so MAVLink packets
 * carry a marker in data[0] to keep them from being mistaken for one.
 *
 * 0xE0 was chosen because 0xE0 & 0xf3 == 0xe0, which no local handler matches.
 */
#define MAVLINK_AIR_MARKER 0xE0

/* Usable payload per radio packet, after the marker byte. */
#define MAVLINK_TRANSPORT_MTU (ESB_MAX_PAYLOAD - 1)

typedef enum {
  /* Unicast to the ground station, with ESB's hardware ack and retry.
   * The Crazyflie is a PRX, so downlink rides in ack payloads and is only
   * sent when the ground station polls -- the same way CRTP downlink works. */
  mavlinkModeTelemetry = 0,
  /* Broadcast to any peer on the shared address. Unacked, which is the
   * correct semantic for peer-to-peer: there is no single receiver to ack. */
  mavlinkModeP2P = 1,
} MavlinkMode;

/* Select the transport mode. Takes effect on the next transmission. */
void mavlinkTransportSetMode(MavlinkMode mode);

MavlinkMode mavlinkTransportGetMode(void);

/**
 * Queue a chunk of the MAVLink byte stream for transmission.
 *
 * The chunk is sent verbatim, so the caller decides where frame boundaries
 * fall. Keeping one MAVLink frame per call avoids a lost radio packet
 * corrupting two frames instead of one.
 *
 * @param data Bytes to send.
 * @param length Number of bytes, at most MAVLINK_TRANSPORT_MTU.
 * @return false if the chunk is too long, or if the transmit queue is full
 *         (telemetry mode only -- broadcasts are sent immediately).
 */
bool mavlinkTransportSend(const uint8_t *data, uint8_t length);

/**
 * Test whether a received radio packet belongs to the MAVLink transport.
 *
 * @return true if it carries the marker, in which case payload and
 *         payloadLength describe the bytes after it.
 */
bool mavlinkTransportReceive(const EsbPacket *packet,
                             const uint8_t **payload,
                             uint8_t *payloadLength);

#endif //__MAVLINK_TRANSPORT_H__

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

/**
 * Queue a chunk for unicast transmission to the ground station.
 *
 * ESB's hardware ack and retry apply. The Crazyflie is a PRX, so this only
 * queues -- the chunk leaves in an ack payload the next time the ground
 * station polls, the same way CRTP downlink works. It therefore fails when
 * the queue is full, and the caller must handle that rather than assume the
 * write succeeded. mavlinkTransportTxFreeSlots() reports the room available.
 *
 * @return false if the chunk is too long or the transmit queue is full.
 */
bool mavlinkTransportSendUnicast(const uint8_t *data, uint8_t length);

/**
 * Broadcast a chunk to any peer on the shared address.
 *
 * Unacked, which is the correct semantic for peer-to-peer: there is no single
 * receiver to acknowledge. Sent immediately rather than queued, so it cannot
 * fail for lack of room.
 *
 * @return false only if the chunk is too long.
 */
bool mavlinkTransportSendBroadcast(const uint8_t *data, uint8_t length);

/**
 * Number of chunks that can still be queued for unicast transmission.
 *
 * Broadcasts do not consume queue slots and are not counted.
 */
uint8_t mavlinkTransportTxFreeSlots(void);

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

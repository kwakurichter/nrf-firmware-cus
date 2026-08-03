/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie 2.0 NRF Firmware
 * Copyright (c) 2014, Bitcraze AB, All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library.
 */
#ifndef __PACKET_H__
#define __PACKET_H__

#include <stdbool.h>
#include <stdint.h>

#define SYSLINK_STARTUP_DELAY_TIME_MS     1000
#define SYSLINK_SEND_PERIOD_MS            10
#define SYSLINK_RADIO_DISABLED_TIMEOUT_MS 3000

/* Must be able to carry a full ESB payload, otherwise main.c overruns
 * slTxPacket when forwarding a radio packet to the STM32. The on-wire length
 * field is a single byte, capping a frame at 255 bytes -- still comfortably
 * above ESB_MAX_PAYLOAD. The buffer is rounded up to 256 for alignment.
 */
#define SYSLINK_MTU 256

struct syslinkPacket {
  uint8_t type;
  uint8_t length;
  char data[SYSLINK_MTU];
};

/**
 * Receive syslink packet.
 *
 * @param packet  Syslink packet to receive data.
 */
bool syslinkReceive(struct syslinkPacket *packet);

/**
 * Send syslink packet.
 * Will only send if link is first activated by a packet beeing received.
 *
 * @param packet  Syslink packet containing data to send.
 */
bool syslinkSend(struct syslinkPacket *packet);

/**
 * Reset syslink state machine.
 */
void syslinkReset();

/**
 * Deactivate syslink. Meaning no packets will be sent
 * until a packet first has been received.
 */
void syslinkDeactivateUntilPacketReceived();

/**
 * @brief Get the number of times checksum 1 has failed for packets on syslink from the STM
 *
 * @return uint8_t Number of failures
 */
uint8_t syslinkGetRxCheckSum1ErrorCnt();

/**
 * @brief Get the number of times checksum 2 has failed for packets on syslink from the STM
 *
 * @return uint8_t Number of failures
 */
uint8_t syslinkGetRxCheckSum2ErrorCnt();


// Defined packet types
#define SYSLINK_RADIO_RAW           0x00
#define SYSLINK_RADIO_CHANNEL       0x01
#define SYSLINK_RADIO_DATARATE      0x02
#define SYSLINK_RADIO_CONTWAVE      0x03
#define SYSLINK_RADIO_RSSI          0x04
#define SYSLINK_RADIO_ADDRESS       0x05
#define SYSLINK_RADIO_RAW_BROADCAST 0x06
#define SYSLINK_RADIO_POWER         0x07
#define SYSLINK_RADIO_P2P           0x08
#define SYSLINK_RADIO_P2P_ACK       0x09
#define SYSLINK_RADIO_P2P_BROADCAST 0x0A
#define SYSLINK_RADIO_READY         0x0B

/* MAVLink transport.
 *
 * Both directions carry an opaque chunk of the MAVLink byte stream. The nRF
 * does not parse MAVLink -- it does not need frame boundaries, because the far
 * end feeds a byte-stream parser that resyncs on STX by design. One syslink
 * packet becomes exactly one radio packet, so the payload must not exceed
 * MAVLINK_TRANSPORT_MTU.
 *
 * The destination is encoded in the packet type rather than a mode setting,
 * matching SYSLINK_RADIO_RAW / _RAW_BROADCAST. The radio listens on the
 * unicast and broadcast addresses simultaneously and picks the transmit
 * address per packet, so telemetry and peer-to-peer traffic can be
 * interleaved freely with no mode to track on either side.
 *
 * SYSLINK_RADIO_MAVLINK_SPACE reports how many transmit slots are free, so
 * the STM32 can apply backpressure. The nRF sends it unsolicited whenever the
 * count changes. This is NOT the same thing as the UART flow control line,
 * which reflects the nRF's UART receive FIFO and says nothing about the radio
 * queue -- the nRF keeps reading syslink when that queue is full and simply
 * drops chunks.
 */
#define SYSLINK_RADIO_MAVLINK           0x0C
#define SYSLINK_RADIO_MAVLINK_BROADCAST 0x0D
#define SYSLINK_RADIO_MAVLINK_SPACE     0x0E


#define SYSLINK_PM_SOURCE             0x10
#define SYSLINK_PM_ONOFF_SWITCHOFF    0x11
#define SYSLINK_PM_BATTERY_VOLTAGE    0x12
#define SYSLINK_PM_BATTERY_STATE      0x13
#define SYSLINK_PM_BATTERY_AUTOUPDATE 0x14
#define SYSLINK_PM_SHUTDOWN_REQUEST   0x15
#define SYSLINK_PM_SHUTDOWN_ACK       0x16
#define SYSLINK_PM_LED_ON             0x17
#define SYSLINK_PM_LED_OFF            0x18
#define SYSLINK_PM_DECKCTRL_DFU       0x19
#define SYSLINK_PM_ONOFF_STM_OFF      0x1A

#define SYSLINK_OW_SCAN       0x20
#define SYSLINK_OW_GETINFO    0x21
#define SYSLINK_OW_READ       0x22
#define SYSLINK_OW_WRITE      0x23

#define SYSLINK_SYS_NRF_VERSION 0x30

#define SYSLINK_DEBUG_PROBE 0xF0

#endif

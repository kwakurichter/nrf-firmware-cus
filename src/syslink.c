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
#include "syslink.h"
#include "uart.h"

#include <nrf.h>
#include "pinout.h"

#include <stdbool.h>
#include <stdint.h>

/* Frame format:
 * +----+-----+------+-----+=============+-----+-----+
 * |  START   | TYPE | LEN | DATA        |   CKSUM   |
 * +----+-----+------+-----+=============+-----+-----+
 *
 * - Start is 2 bytes constant, defined bellow
 * - Length and type are uint8_t
 * - Length define the data length
 * - CKSUM is 2 bytes Fletcher 8 bit checksum. See rfc1146.
 *   Checksum is calculated with TYPE, LEN and DATA
 */


#define START "\xbc\xcf"
#define START_BYTE1 0xBC
#define START_BYTE2 0xCF

static enum {state_first_start, state_second_start, state_length, state_type, state_data, state_cksum1, state_cksum2, state_done} state = state_first_start;

static bool isSyslinkActive = false;

static uint8_t syslinkRxCheckSum1ErrorCnt;
static uint8_t syslinkRxCheckSum2ErrorCnt;

void syslinkReset() {
  state = state_first_start;
}

bool syslinkReceive(struct syslinkPacket *packet)
{
  static int step=0;
  static int length=0;
  static uint8_t cksum_a=0, cksum_b=0;
  char c;

  packet->length = 0;

  if (state == state_done)
  {
    state = state_first_start;
    step = 0;
  }

  while (uartIsDataReceived() && (state != state_done))
  {
    c = uartGetc();

    switch(state)
    {
      case state_first_start:
        state = (c == START_BYTE1) ? state_second_start : state_first_start;
        break;
      case state_second_start:
        state = (c == START_BYTE2) ? state_type : state_first_start;
        break;
      case state_type:
        packet->type = c;
        cksum_a = c;
        cksum_b = cksum_a;
        state = state_length;
        break;
      case state_length:
        length = c;
        cksum_a += c;
        cksum_b += cksum_a;
        step = 0;
        if (length > 0 && length <= SYSLINK_MTU)
          state = state_data;
        else if (length > SYSLINK_MTU)
          state = state_first_start;
        else
          state = state_cksum1;
        break;
      case state_data:
        if (step < SYSLINK_MTU)
        {
          packet->data[step] = c;
          cksum_a += c;
          cksum_b += cksum_a;
        }
        step++;
        if(step >= length) {
          state = state_cksum1;
        }
        break;
      case state_cksum1:
        if (c == cksum_a)
        {
          state = state_cksum2;
        }
        else
        {  // Wrong checksum
          syslinkRxCheckSum1ErrorCnt++;
          state = state_first_start;
#ifdef SYSLINK_CKSUM_MON
          if (NRF_GPIO->OUT & (1<<LED_PIN))
            NRF_GPIO->OUTCLR = 1<<LED_PIN;
          else
            NRF_GPIO->OUTSET = 1<<LED_PIN;
#endif
        }
        break;
      case state_cksum2:
        if (c == cksum_b)
        {
          packet->length = length;
          isSyslinkActive = true;
          state = state_done;
        }
        else
        {  // Wrong checksum
          syslinkRxCheckSum2ErrorCnt++;
          state = state_first_start;
          step = 0;
#ifdef SYSLINK_CKSUM_MON
          if (NRF_GPIO->OUT & (1<<LED_PIN))
            NRF_GPIO->OUTCLR = 1<<LED_PIN;
          else
            NRF_GPIO->OUTSET = 1<<LED_PIN;
#endif
        }
        break;
      case state_done:
        break;
    }
  }

  return (state == state_done);
}

bool syslinkSend(struct syslinkPacket *packet)
{
  uint8_t cksum_a=0;
  uint8_t cksum_b=0;
  int i;

  if (isSyslinkActive)
  {

    uartPuts(START);

    uartPutc((unsigned char)packet->type);
    cksum_a += packet->type;
    cksum_b += cksum_a;

    uartPutc((unsigned char)packet->length);
    cksum_a += packet->length;
    cksum_b += cksum_a;

    for (i=0; i < packet->length; i++)
    {
      uartPutc(packet->data[i]);
      cksum_a += packet->data[i];
      cksum_b += cksum_a;
    }

    uartPutc(cksum_a);
    uartPutc(cksum_b);

    return true;
  }
  else
  {
    return false;
  }
}

// For ArduPilot Implementation (uses the buffered UART send)
bool syslinkSend_buffered(struct syslinkPacket *packet)
{
  // This is part of the original logic to wait for the STM32 to talk first.
  if (!isSyslinkActive)
  {
    return false;
  }

  // 1. Create a temporary buffer to hold the entire packet frame.
  //    Size = 2 (START) + 1 (TYPE) + 1 (LEN) + max_data + 2 (CKSUM)
  uint8_t frame_buffer[SYSLINK_MTU + 6];
  uint16_t frame_len = 0;
  
  // 2. Initialize checksum variables
  uint8_t cksum_a=0;
  uint8_t cksum_b=0;
  
  // 3. Assemble the packet header
  frame_buffer[frame_len++] = START_BYTE1;
  frame_buffer[frame_len++] = START_BYTE2;

  frame_buffer[frame_len++] = packet->type;
  cksum_a += packet->type;
  cksum_b += cksum_a;

  frame_buffer[frame_len++] = packet->length;
  cksum_a += packet->length;
  cksum_b += cksum_a;

  // 4. Copy the data payload and update checksum on-the-fly
  for (int i = 0; i < packet->length; i++)
  {
    frame_buffer[frame_len++] = packet->data[i];
    cksum_a += packet->data[i];
    cksum_b += cksum_a;
  }

  // 5. Add the calculated checksum to the end of the frame
  frame_buffer[frame_len++] = cksum_a;
  frame_buffer[frame_len++] = cksum_b;

  // 6. Send the entire assembled frame in one non-blocking call
  uart_buffered_send(frame_buffer, frame_len);

  return true;
}

// For ArduPilot Implementation (uses the buffered UART send)
bool syslinkMAVSend_buffered(uint8_t *buffer, uint16_t len)
{
  // This is part of the original logic to wait for the STM32 to talk first.
  if (!isSyslinkActive)
  {
    return false;
  }

  // 6. Send the entire assembled frame in one non-blocking call
  uart_buffered_send(buffer, len);

  return true;
}

void syslinkDeactivateUntilPacketReceived()
{
  isSyslinkActive = false;
}


uint8_t syslinkGetRxCheckSum1ErrorCnt() {
  return syslinkRxCheckSum1ErrorCnt;
}

uint8_t syslinkGetRxCheckSum2ErrorCnt() {
  return syslinkRxCheckSum2ErrorCnt;
}

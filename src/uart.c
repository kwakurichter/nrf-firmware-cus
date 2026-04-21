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
#include <stdbool.h>

#include <nrf.h>

#include "pinout.h"

#include "uart.h"

#include <nrf_gpio.h>

static bool isInit = false;

#define Q_LENGTH 128

// -- For ArduPilot Implementation --
#define UART_TX_BUFFER_SIZE 256

static volatile uint8_t uart_tx_buffer[UART_TX_BUFFER_SIZE];
static volatile uint16_t uart_tx_head = 0;
static volatile uint16_t uart_tx_tail = 0;
// -- For ArduPilot Implementation --

static volatile char rxq[Q_LENGTH];
static volatile int head = 0;
static volatile int tail = 0;

static volatile int dropped = 0;
static volatile char dummy;

static volatile uint8_t uartError = 0;
static volatile uint8_t uartErrorCount = 0;

void UART0_IRQHandler()
{
  // --- Existing RX handling code ---
  if (NRF_UART0->EVENTS_RXDRDY) {
    int nhead = head+1;

    if (NRF_UART0->ERRORSRC) {
      uartError = NRF_UART0->ERRORSRC;
      NRF_UART0->ERRORSRC = 0xFF;

      uartErrorCount++;
    }

    NRF_UART0->EVENTS_RXDRDY = 0;

    // Check if the queue is not full
    if (nhead >= Q_LENGTH) nhead = 0;
    if (nhead == tail) {
      dummy = NRF_UART0->RXD; //Read anyway to avoid hw overflow
      dropped++;
      return;
    }

    // Push data in queue
    rxq[head++] = NRF_UART0->RXD;
    if (head >= Q_LENGTH) head = 0;
  }
  // --- New TX handling code ---
  if (NRF_UART0->EVENTS_TXDRDY) {
    NRF_UART0->EVENTS_TXDRDY = 0;

    if (uart_tx_head != uart_tx_tail) {
      // If there is data in our new buffer, send the next byte
      NRF_UART0->TXD = uart_tx_buffer[uart_tx_tail];
      uart_tx_tail = (uart_tx_tail + 1) % UART_TX_BUFFER_SIZE;
    } else {
      // Buffer is empty, disable the TX interrupt until more data is added
      NRF_UART0->INTENCLR = UART_INTENCLR_TXDRDY_Msk;
    }
  }
}

void uartInit()
{
  NRF_GPIO->PIN_CNF[8] &= ~GPIO_PIN_CNF_PULL_Msk;
  NRF_GPIO->PIN_CNF[8] |= (GPIO_PIN_CNF_PULL_Pulldown<<GPIO_PIN_CNF_PULL_Pos);

  NRF_GPIO->PIN_CNF[UART_TX_PIN] = (NRF_GPIO->PIN_CNF[UART_TX_PIN] & (~GPIO_PIN_CNF_DRIVE_Msk)) | (GPIO_PIN_CNF_DRIVE_S0S1<<GPIO_PIN_CNF_DRIVE_Pos);

  NRF_GPIO->DIRSET = 1<<UART_TX_PIN;
  NRF_GPIO->OUTSET = 1<<UART_TX_PIN;
  NRF_UART0->PSELTXD = UART_TX_PIN;

  NRF_GPIO->DIRCLR = 1<<UART_RX_PIN;
  NRF_UART0->PSELRXD = UART_RX_PIN;

  NRF_GPIO->DIRSET = 1<<UART_RTS_PIN;
  NRF_GPIO->OUTSET = 1<<UART_RTS_PIN;
  NRF_UART0->PSELRTS = UART_RTS_PIN;

  NRF_UART0->CONFIG = UART_CONFIG_HWFC_Msk;

#ifndef DEBUG_UART
  NRF_UART0->BAUDRATE = UART_BAUDRATE_BAUDRATE_Baud1M;
#else
  NRF_UART0->BAUDRATE = UART_BAUDRATE_BAUDRATE_Baud460800;
#endif

  NRF_UART0->ENABLE = UART_ENABLE_ENABLE_Enabled;

  NRF_UART0->TASKS_STARTTX = 1;

  // Enable interrupt on receive
  NVIC_SetPriority(UART0_IRQn, 3);
  NVIC_EnableIRQ(UART0_IRQn);
  NRF_UART0->INTENSET = UART_INTENSET_RXDRDY_Msk;

  NRF_UART0->TASKS_STARTRX = 1;

  isInit = true;
}

void uartDeinit()
{
  NRF_UART0->TASKS_STOPRX = 1;
  NRF_UART0->TASKS_STOPTX = 1;
  NRF_UART0->ENABLE = 0;

  nrf_gpio_cfg_input(UART_TX_PIN, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(UART_RTS_PIN, NRF_GPIO_PIN_NOPULL);

  isInit = false;
}

void uartPuts(char* string)
{
  if (!isInit)
      return;

  while(*string)
  {
    uartPutc(*string++);
  }
}

void uartSend(char* data, int len)
{
  if (!isInit)
      return;

  while(len--)
  {
    uartPutc(*data++);
  }
}

// For Ardupilot Implementation (Non-Blocking send)
void uart_buffered_send(const uint8_t *data, uint16_t length) {
  // 1) Snapshot head/tail & compute free space
  uint16_t head = uart_tx_head;
  uint16_t tail = uart_tx_tail;
  uint16_t free_space;

  if (head >= tail) {
      free_space = UART_TX_BUFFER_SIZE - (head - tail) - 1;
  } else {
      free_space = (tail - head) - 1;
  }

  // Drop whole packet if it won't fit
  if (length > free_space) {
      return;
  }

  // Remember if we were idle (so we know to prime the pump)
  bool was_idle = (head == tail);

  // 2) Disable all IRQs to protect head/tail
  __disable_irq();

  // 3) Queue the data
  for (uint16_t i = 0; i < length; i++) {
      uart_tx_buffer[head] = data[i];
      head = (head + 1) % UART_TX_BUFFER_SIZE;
  }
  uart_tx_head = head;

  // 4) If buffer was empty before, kick off the first byte immediately
  if (was_idle) {
      NRF_UART0->EVENTS_TXDRDY = 0;
      NRF_UART0->TXD = uart_tx_buffer[uart_tx_tail];
      uart_tx_tail = (uart_tx_tail + 1) % UART_TX_BUFFER_SIZE;
  }

  // 5) Enable the TXDRDY interrupt so the ISR will send the rest
  NRF_UART0->INTENSET = UART_INTENSET_TXDRDY_Msk;

  // 6) Re-enable IRQs
  __enable_irq();
}

void uartPutc(char c)
{
  if (!isInit)
      return;

  NRF_UART0->TXD = c;
  while(!NRF_UART0->EVENTS_TXDRDY);
  NRF_UART0->EVENTS_TXDRDY=0;
}

bool uartIsDataReceived()
{
  if (!isInit)
      return false;

  return head!=tail;
}

char uartGetc()
{
  char c=0;

  if (!isInit)
      return c;

  // TODO: Add overrun check

  if (head!=tail) {
    c = rxq[tail++];
    if (tail >= Q_LENGTH) tail = 0;
  }

  return c;
}

int uartDropped() {
  return dropped;
}

uint8_t uartGetError() {
  return uartError;
}

uint8_t uartGetErrorCount() {
  return uartErrorCount;
}

/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie 2.0 NRF Firmware
 * Copyright (c) 2024, Bitcraze AB, All rights reserved.
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
#include <nrf.h>



#include <stdio.h>
#include <string.h>

#include "debug.h"
#include "platform.h"

#include "uart.h"
#include "esb.h"
#include "syslink.h"
#include "led.h"
#include "button.h"
#include "pm.h"
#include "pinout.h"
#include "systick.h"
#include "nrf_sdm.h"
#include "nrf_soc.h"
#include "nrf_nvic.h"
#include "version.h"
#include "shutdown.h"
#include "platform.h"

#include "memory.h"
#include "ownet.h"

#include "ble_int.h"
#include "ble_crazyflies.h"

extern void  initialise_monitor_handles(void);

#ifndef SEMIHOSTING
#define printf(...)
#endif

#ifndef DEFAULT_RADIO_RATE
  #define DEFAULT_RADIO_RATE  esbDatarate2M
#endif
#ifndef DEFAULT_RADIO_CHANNEL
  #define DEFAULT_RADIO_CHANNEL 80
#endif

static void mainloop(void);

#if BLE==0
#undef BLE
#endif

#define MEMORY_BITCRAZE_VID 0xBC
#define MEMORY_AIDECK_PID 0x12
#define MEMORY_AIDECK_BOARDNAME "bcAI"
#define DEBUG_PORT 0x0E // any unused CRTP port 0–15
#define DEBUG_PORT_2 0x09 // Port for the second debug print

#ifdef BLE
int volatile bleEnabled = 1;
#else
int volatile bleEnabled = 0;
#endif

// New struct to store received packets to avoid race conditions
typedef struct {
  uint8_t size;
  uint8_t data[63];
  uint8_t rssi;
  bool broadcast;
} SafeRxPacket;

static SafeRxPacket safePacket; // Define a static instance of our safe buffer

static struct syslinkPacket slRxPacket;
static struct syslinkPacket slTxPacket;
static int radioRSSISendTime = SYSLINK_STARTUP_DELAY_TIME_MS;
static int vbatSendTime = SYSLINK_STARTUP_DELAY_TIME_MS;
static uint8_t rssi;
static bool bootedFromBootloader;
static bool enableBatteryAutoupdate = false;

static void syslinkEnableBatteryMessages();
static void sendDataToStmOverSyslink();
static void handleButtonEvents();
static void handleSyslinkEvents(bool slReceived);

static void handleRadioCmd(const SafeRxPacket* packet);
static void handleBootloaderCmd(const SafeRxPacket* packet);
static void disableBle();

static bool debugProbeReceivedChan = false;
static bool debugProbeReceivedAddress = false;
static bool debugProbeReceivedRate = false;
static bool bleIsDisabled = false;  // Added flag

int main()
{
  // Stop early if the platform is not supported
  if (platformInit() != 0) {
    while(1);
  }

  debugInit();
  systickInit();
  memoryInit();

  // The GAP8 can enter an undefined state if the system is powered off/on too quickly
  // or transitions directly from the bootloader to firmware. If this happens, the GAP8
  // might freeze and require a prolonged power-off period to recover.
  // To prevent this, we introduce a delay when a specific deck (bcAI) is detected.

  // Check if the bcAI deck is attached
  bool bcAiPresent = memoryHasDeck(MEMORY_BITCRAZE_VID, MEMORY_AIDECK_PID, MEMORY_AIDECK_BOARDNAME);

  // Apply a delay based on deck presence:
  // - If bcAI is connected, wait 10s to ensure stable operation.
  // - Otherwise, skip or use a minimal delay.
  if (bcAiPresent) {
    msDelay(5000);
  } else {
    // Short boot delay needed for stability, reason unknown
    msDelay(100);
  }

  if (bleEnabled) {
#ifdef BLE
    ble_init();
#endif
  } else {
    NRF_CLOCK->TASKS_HFCLKSTART = 1UL;
    while(!NRF_CLOCK->EVENTS_HFCLKSTARTED);
  }

#ifdef SEMIHOSTING
  initialise_monitor_handles();
#endif

  NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSTAT_SRC_Synth;

  NRF_CLOCK->TASKS_LFCLKSTART = 1UL;
  while(!NRF_CLOCK->EVENTS_LFCLKSTARTED);

  LED_INIT();
  if ((NRF_POWER->GPREGRET & 0x80) && ((NRF_POWER->GPREGRET&(0x3<<1))==0)) {
    buttonInit(buttonShortPress);
  } else {
    buttonInit(buttonIdle);
  }

  if  (NRF_POWER->GPREGRET & 0x20) {
    bootedFromBootloader = true;
    NRF_POWER->GPREGRET &= ~0x20;
  }

  pmInit();

  if ((NRF_POWER->GPREGRET&0x01) == 0) {
		  pmSetState(pmSysRunning);
  }

  LED_ON();


  NRF_GPIO->PIN_CNF[RADIO_PAEN_PIN] |= GPIO_PIN_CNF_DIR_Output | (GPIO_PIN_CNF_DRIVE_S0H1<<GPIO_PIN_CNF_DRIVE_Pos);

  if (!bleEnabled) {
    esbInit();

    esbSetDatarate(DEFAULT_RADIO_RATE);
    esbSetChannel(DEFAULT_RADIO_CHANNEL);
  }

  DEBUG_PRINT("Started\n");

  mainloop();

  // The main loop should never end
  // TODO see if we should shut-off the system there?
  while(1);

  return 0;
}

void mainloop()
{
  //static EsbPacket esbRxPacket;
  //bool esbReceived = false;

  while(1)
  {
#ifdef BLE
    if (bleEnabled) {
      //if ((esbReceived == false) && ble_receive_packet(&esbRxPacket)) {
      //  esbReceived = true;
      //}
    }
#endif

#ifndef CONT_WAVE_TEST

if (esbIsRxPacket())
{
  // A packet is available in the radio's receive queue.
  // We must immediately copy it to our safe, local buffer before the interrupt handler or radio DMA can overwrite it with an ACK packet.
  // To prevent the RADIO_IRQHandler from corrupting our data during the copy, we disable all interrupts, perform the copy, and then immediately re-enable them. This makes the copy "atomic".
  
  __disable_irq(); // Disable all interrupts
  
  // Step 1: Get the packet from the radio's receive queue.
  EsbPacket* packet = esbGetRxPacket();

  // 2. Perform a deep copy into our safe buffer.
  safePacket.size = packet->size;
  if (safePacket.size > 0) {
    memcpy(safePacket.data, packet->data, safePacket.size);
  }
  safePacket.rssi = packet->rssi;
  safePacket.broadcast = (packet->match == ESB_MULTICAST_ADDRESS_MATCH);  // The received packet was a broadcast, if received on local address 1

  // 3. Immediately release the radio's buffer. All subsequent logic will use the 'safePacket' object, not the volatile 'packet'.
  esbReleaseRxPacket();

  __enable_irq(); // Re-enable all interrupts

  // Only disable BLE once when the first ESB packet is received.
  if (!bleIsDisabled) {
    disableBle();
    bleIsDisabled = true;
  }

  // -- DEBUG --
  uint8_t dbg[3];
  dbg[0] = safePacket.size;        // how many bytes the nRF saw
  dbg[1] = safePacket.data[0];     // first byte
  dbg[2] = safePacket.data[1];     // second byte
  // Queue a debug packet on DEBUG_PORT (0x0E), channel 0
  esbSendDebugPacket(DEBUG_PORT, 0, (char*)dbg, sizeof(dbg));
  // -- DEBUG --

  //Store RSSI here so that we can send it to STM later
  // Todo investigate if we can not just simply link this to the packet itself or find a way to separate this due to P2P
  rssi = safePacket.rssi;

  // Now, inspect the packet and decide what to do with it.
  // Is it a special radio command packet?
  if ((safePacket.size >= 4) && (safePacket.data[0]&0xf3) == 0xf3 && (safePacket.data[1]==0x03))
  {
    // This logic needs to be updated to pass the safePacket
    handleRadioCmd(&safePacket);
  }
  // Is it a special bootloader command packet?
  else if ((safePacket.size > 2) && (safePacket.data[0]&0xf3) == 0xf3 && (safePacket.data[1]==0xfe))
  {
    // This logic needs to be updated to pass the safePacket
    handleBootloaderCmd(&safePacket);
  }
  // Is it a peer-to-peer (P2P) packet?
  else if (safePacket.size >= 2 && (safePacket.data[0] & 0xf3) == 0xf3 && (safePacket.data[1] & 0xF0) == 0x80)
  {
    // Handle P2P logic (forward to STM with SYSLINK_RADIO_P2P type)

    slTxPacket.data[0] = safePacket.data[1] & 0x0F;  // The first byte sent is the P2P port
    slTxPacket.data[1] = safePacket.rssi; // Save RSSI between drones in packet
    memcpy(&slTxPacket.data[2], &safePacket.data[2], safePacket.size - 2);
    slTxPacket.length = safePacket.size;
    if (safePacket.broadcast) {
      slTxPacket.type = SYSLINK_RADIO_P2P_BROADCAST;
    } else {
      slTxPacket.type = SYSLINK_RADIO_P2P;
    }

    syslinkSend_buffered(&slTxPacket);
  }
  else if ((safePacket.data[0] & 0xf3) == 0xf3)
  {
    // This is a low-level radio packet (like an ACK or handshake) that is not
    // a command. We should do nothing and discard it, preventing it from
    // being forwarded to the STM.
  }
  else if (safePacket.data[0] == 0xff)
  {
    // This is a low-level radio packet (like an ACK or handshake) that is not
    // a command. We should do nothing and discard it, preventing it from
    // being forwarded to the STM.
  }
  // If it's none of the above, assume it's general data from the GCS.
  else
  {
    // Set the Syslink payload length to the radio packet size
    slTxPacket.length = safePacket.size;

    // Copy the radio packet's data into the Syslink packet
    memcpy(slTxPacket.data, &safePacket.data, slTxPacket.length);

    // Set the Syslink packet type.
    if (safePacket.broadcast) {
      slTxPacket.type = SYSLINK_RADIO_RAW_BROADCAST;
    } else {
      // --- THIS IS NEW GCS->STM FORWARDING LOGIC ---
      slTxPacket.type = SYSLINK_RADIO_MAVLINK;
      
      // -- DEBUG --
      uint8_t dbg2[3];
      dbg2[0] = slTxPacket.length;      // how many bytes the nRF saw
      dbg2[1] = slTxPacket.data[0];     // first byte
      dbg2[2] = slTxPacket.data[1];     // second byte
      // Queue a debug packet on DEBUG_PORT_2 (0x09), channel 0
      esbSendDebugPacket(DEBUG_PORT_2, 0, (char*)dbg2, sizeof(dbg2));
      // -- DEBUG --
    }
    // Use the buffered send for all transmissions
    syslinkSend_buffered(&slTxPacket);
  }
}

handleSyslinkEvents(syslinkReceive(&slRxPacket));
sendDataToStmOverSyslink();

#endif

    handleButtonEvents();

    bootedFromBootloader = false;

    // processes loop
    buttonProcess();
    pmProcess();
    shutdownProcess();
    //owRun();       //TODO!
  }
}

static void handleSyslinkEvents(bool slReceived)
{
  if (slReceived)
  {
    switch (slRxPacket.type)
    {
      case SYSLINK_RADIO_RAW:
        if (esbCanTxPacket() && (slRxPacket.length < SYSLINK_MTU))
        {
          EsbPacket* packet = esbGetTxPacket();

          if (packet) {
            memcpy(packet->data, slRxPacket.data, slRxPacket.length);
            packet->size = slRxPacket.length;

            esbSendTxPacket(packet);
          }
          bzero(slRxPacket.data, SYSLINK_MTU);
        }

#ifdef BLE
        if (bleEnabled) {
          if (slRxPacket.length < SYSLINK_MTU) {
            static EsbPacket pk;
            memcpy(pk.data,  slRxPacket.data, slRxPacket.length);
            pk.size = slRxPacket.length;
            ble_send_packet(&pk);
          }
        }
#endif

        break;
      case SYSLINK_RADIO_CHANNEL:
        if(slRxPacket.length == 1)
        {
          esbSetChannel(slRxPacket.data[0]);

          slTxPacket.type = SYSLINK_RADIO_CHANNEL;
          slTxPacket.data[0] = slRxPacket.data[0];
          slTxPacket.length = 1;
          //syslinkSend(&slTxPacket);
          syslinkSend_buffered(&slTxPacket);  // to avoid weird interactions between buffered send and non-buffered send

          debugProbeReceivedChan = true;
        }
        break;
      case SYSLINK_RADIO_DATARATE:
        if(slRxPacket.length == 1)
        {
          esbSetDatarate(slRxPacket.data[0]);

          slTxPacket.type = SYSLINK_RADIO_DATARATE;
          slTxPacket.data[0] = slRxPacket.data[0];
          slTxPacket.length = 1;
          //syslinkSend(&slTxPacket);
          syslinkSend_buffered(&slTxPacket);  // to avoid weird interactions between buffered send and non-buffered send

          debugProbeReceivedRate = true;
        }
        break;
      case SYSLINK_RADIO_CONTWAVE:
        if(slRxPacket.length == 1) {
          esbSetContwave(slRxPacket.data[0]);

          slTxPacket.type = SYSLINK_RADIO_CONTWAVE;
          slTxPacket.data[0] = slRxPacket.data[0];
          slTxPacket.length = 1;
          //syslinkSend(&slTxPacket);
          syslinkSend_buffered(&slTxPacket);  // to avoid weird interactions between buffered send and non-buffered send
        }
        break;
      case SYSLINK_RADIO_ADDRESS:
        if(slRxPacket.length == 5)
        {
          uint64_t address = 0;
          memcpy(&address, &slRxPacket.data[0], 5);
          esbSetAddress(address);

          slTxPacket.type = SYSLINK_RADIO_ADDRESS;
          memcpy(slTxPacket.data, slRxPacket.data, 5);
          slTxPacket.length = 5;
          //syslinkSend(&slTxPacket);
          syslinkSend_buffered(&slTxPacket);  // to avoid weird interactions between buffered send and non-buffered send

          debugProbeReceivedAddress = true;
        }
        break;
      case SYSLINK_RADIO_POWER:
        if(slRxPacket.length == 1)
        {
          esbSetTxPowerDbm((int8_t)slRxPacket.data[0]);

          slTxPacket.type = SYSLINK_RADIO_POWER;
          slTxPacket.data[0] = slRxPacket.data[0];
          slTxPacket.length = 1;
          //syslinkSend(&slTxPacket);
          syslinkSend_buffered(&slTxPacket);  // to avoid weird interactions between buffered send and non-buffered send
        }
        break;
      case SYSLINK_RADIO_MAVLINK:
        // Check if the radio TX queue has space and the packet isn't too large
        if (esbCanTxPacket() && (slRxPacket.length < SYSLINK_MTU))
        {
          // Get a free radio packet buffer
          EsbPacket* packet = esbGetTxPacket();

          if (packet) {
            // Copy the data from the Syslink packet into the radio packet
            memcpy(packet->data, slRxPacket.data, slRxPacket.length);
            packet->size = slRxPacket.length;

            // Queue the radio packet for transmission
            esbSendTxPacket();
          }
          
          // Clear the received syslink packet data buffer for safety
          bzero(slRxPacket.data, SYSLINK_MTU);
        }
        break;
      case SYSLINK_PM_ONOFF_SWITCHOFF:
        pmSetState(pmAllOff);
        break;
      case SYSLINK_OW_GETINFO:
      case SYSLINK_OW_READ:
      case SYSLINK_OW_SCAN:
      case SYSLINK_OW_WRITE:
        if (memorySyslink(&slRxPacket)) {
          //syslinkSend(&slRxPacket);
          syslinkSend_buffered(&slTxPacket);  // to avoid weird interactions between buffered send and non-buffered send
        }
        break;
      case SYSLINK_RADIO_P2P_BROADCAST:
        // Check that bluetooth is disabled and disable it if not
        if (bleEnabled) {
          disableBle();
        }
        // Send the P2P packet immediately without buffer
        esbSendP2PPacket(slRxPacket.data[0],&slRxPacket.data[1],slRxPacket.length-1);
        break;
      case SYSLINK_SYS_NRF_VERSION:{
        size_t len = strlen(V_STAG);
        slTxPacket.type = SYSLINK_SYS_NRF_VERSION;

        memcpy(&slTxPacket.data[0], V_STAG, len);

        if (V_MODIFIED) {
          slTxPacket.data[len] = '*';
          len += 1;
        }
        // Add platform identifier
        slTxPacket.data[len++] = ' ';
        slTxPacket.data[len++] = '(';
        memcpy(&slTxPacket.data[len], (void*)PLATFORM_DEVICE_DATA_FLASH_POS + 2, 4);
        len += 4;
        slTxPacket.data[len++] = ')';
        slTxPacket.data[len++] = '\0';

        slTxPacket.length = len;
        //syslinkSend(&slTxPacket);
        syslinkSend_buffered(&slTxPacket);  // to avoid weird interactions between buffered send and non-buffered send
      } break;
      case SYSLINK_PM_BATTERY_AUTOUPDATE:
        syslinkEnableBatteryMessages();
        break;
      case SYSLINK_PM_SHUTDOWN_ACK:
        shutdownReceivedAck();
        break;
      case SYSLINK_PM_LED_ON:
        LED_ON();
        break;
      case SYSLINK_PM_LED_OFF:
        LED_OFF();
        break;
      case SYSLINK_DEBUG_PROBE:
      {
        slTxPacket.type = SYSLINK_DEBUG_PROBE;
        slTxPacket.data[0] = debugProbeReceivedAddress;
        slTxPacket.data[1] = debugProbeReceivedChan;
        slTxPacket.data[2] = debugProbeReceivedRate;
        slTxPacket.data[3] = (uint8_t)uartDropped();
        slTxPacket.data[4] = uartGetError();
        slTxPacket.data[5] = uartGetErrorCount();
        slTxPacket.data[6] = syslinkGetRxCheckSum1ErrorCnt();
        slTxPacket.data[7] = syslinkGetRxCheckSum2ErrorCnt();

        slTxPacket.length = 8;
        //syslinkSend(&slTxPacket);
        syslinkSend_buffered(&slTxPacket);  // to avoid weird interactions between buffered send and non-buffered send
      }
        break;
    }
  }
}

static void syslinkEnableBatteryMessages()
{
  enableBatteryAutoupdate = true;
}


static void sendDataToStmOverSyslink()
{
  if (enableBatteryAutoupdate)
  {
    // Send the battery voltage and state to the STM every SYSLINK_SEND_PERIOD_MS
    if ((systickGetTick() >= vbatSendTime + SYSLINK_SEND_PERIOD_MS))
    {
      float fdata;

      vbatSendTime = systickGetTick();
      slTxPacket.type = SYSLINK_PM_BATTERY_STATE;
      slTxPacket.length = 9;

      // Set flags that if we're plugged in to USB, currently charging and if we can charge.
      uint8_t flags = getPowerStatusFlags();
      slTxPacket.data[0] = flags;

      fdata = pmGetVBAT();
      memcpy(slTxPacket.data + 1, &fdata, sizeof(float));

      fdata = pmGetISET();
      memcpy(slTxPacket.data + 1 + 4, &fdata, sizeof(float));

    #ifdef PM_SYSLINK_INCLUDE_TEMP
      fdata = pmGetTemp();
      slTxPacket.length += 4;
      memcpy(slTxPacket.data + 1 + 8, &fdata, sizeof(float));
    #endif
      //syslinkSend(&slTxPacket);
      syslinkSend_buffered(&slTxPacket);  // to avoid weird interactions between buffered send and non-buffered send
    }

    //Send an RSSI sample to the STM every 10ms(100Hz)
    if (systickGetTick() >= radioRSSISendTime + 10)
    {
      radioRSSISendTime = systickGetTick();
      slTxPacket.type = SYSLINK_RADIO_RSSI;
      //This message contains only the RSSI measurement which consist
      //of a single uint8_t
      slTxPacket.length = sizeof(uint8_t);
      memcpy(slTxPacket.data, &rssi, sizeof(uint8_t));

      //syslinkSend(&slTxPacket);
      syslinkSend_buffered(&slTxPacket);  // to avoid weird interactions between buffered send and non-buffered send
    }
  }
}

static void handleButtonEvents()
{
  // Button event handling
  ButtonEvent be = buttonGetState();
  if ((pmGetState() != pmSysOff) && (be == buttonShortPress))
  {
    // Request graceful shutdown from STM32, will timeout and shutdown
    // if no response is received.
    shutdownSendRequest();
  }
  else if ((pmGetState() == pmSysOff) && (be == buttonShortPress))
  {
    //Normal boot
    pmSysBootloader(false);
    pmSetState(pmSysRunning);
  }
  else if ((pmGetState() == pmSysOff) && bootedFromBootloader)
  {
    //Normal boot after bootloader
    pmSysBootloader(false);
    pmSetState(pmSysRunning);
  }
  else if ((pmGetState() == pmSysOff) && (be == buttonLongPress))
  {
    //stm bootloader
    pmSysBootloader(true);
    pmSetState(pmSysRunning);
  }
}

#define RADIO_CTRL_SET_CHANNEL 1
#define RADIO_CTRL_SET_DATARATE 2
#define RADIO_CTRL_SET_POWER 3

static void handleRadioCmd(const SafeRxPacket *packet)
{
  switch (packet->data[2]) {
    case RADIO_CTRL_SET_CHANNEL:
      esbSetChannel(packet->data[3]);
      break;
    case RADIO_CTRL_SET_DATARATE:
      esbSetDatarate(packet->data[3]);
      break;
    case RADIO_CTRL_SET_POWER:
      esbSetTxPower(packet->data[3]);
      break;
    default:
      break;
  }
}

#define BOOTLOADER_CMD_RESET_INIT 0xFF
#define BOOTLOADER_CMD_RESET      0xF0
#define BOOTLOADER_CMD_ALLOFF     0x01
#define BOOTLOADER_CMD_SYSOFF     0x02
#define BOOTLOADER_CMD_SYSON      0x03
#define BOOTLOADER_CMD_GETVBAT    0x04
#define BOOTLOADER_CMD_LED_ON     0x05
#define BOOTLOADER_CMD_LED_OFF    0x06

static void handleBootloaderCmd(const SafeRxPacket *packet)
{
  static bool resetInit = false;
  static struct esbPacket_s txpk;

  switch (packet->data[2]) {
    case BOOTLOADER_CMD_RESET_INIT:

      resetInit = true;

      txpk.data[0] = 0xff;
      txpk.data[1] = 0xfe;
      txpk.data[2] = BOOTLOADER_CMD_RESET_INIT;

      memcpy(&(txpk.data[3]), (uint32_t*)NRF_FICR->DEVICEADDR, 6);

      txpk.size = 9;
#ifdef BLE
      if (bleEnabled) {
        ble_send_packet(&txpk);
      }
#endif

      if (esbCanTxPacket()) {
        struct esbPacket_s *pk = esbGetTxPacket();
        memcpy(pk, &txpk, sizeof(struct esbPacket_s));
        esbSendTxPacket(pk);
      }

      break;
    case BOOTLOADER_CMD_RESET:
      if (resetInit && (packet->size == 4)) {
        msDelay(100);
        if (packet->data[3] == 0) {
          NRF_POWER->GPREGRET |= 0x40;
        } else {
          //Set bit 0x20 forces boot to firmware
          NRF_POWER->GPREGRET |= 0x20U;
        }
        if (bleEnabled) {
#ifdef BLE
          sd_nvic_SystemReset();
#endif
        } else {
          NVIC_SystemReset();
        }
      }
      break;
    case BOOTLOADER_CMD_ALLOFF:
      pmSetState(pmAllOff);
      break;
    case BOOTLOADER_CMD_SYSOFF:
      syslinkDeactivateUntilPacketReceived();
      pmSetState(pmSysOff);
      break;
    case BOOTLOADER_CMD_SYSON:
      pmSysBootloader(false);
      pmSetState(pmSysRunning);
      syslinkReset();

      break;
    case BOOTLOADER_CMD_GETVBAT:
      if (esbCanTxPacket()) {
        float vbat = pmGetVBAT();
        struct esbPacket_s *pk = esbGetTxPacket();

        pk->data[0] = 0xff;
        pk->data[1] = 0xfe;
        pk->data[2] = BOOTLOADER_CMD_GETVBAT;

        memcpy(&(pk->data[3]), &vbat, sizeof(float));
        pk->size = 3 + sizeof(float);

        esbSendTxPacket(pk);
      }
      break;
    case BOOTLOADER_CMD_LED_ON:
      LED_ON();
      break;
    case BOOTLOADER_CMD_LED_OFF:
      LED_OFF();
      break;
    default:
      break;
  }
}

static void disableBle() {
#ifdef BLE
  if (bleEnabled) {
    sd_softdevice_disable();
    bleEnabled = 0;
    esbInit();
  }
#endif
  bleEnabled = 0;
}

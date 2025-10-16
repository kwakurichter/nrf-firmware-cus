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

#include "mavlink/ardupilotmega/mavlink.h"  // For VBAT message

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
#define DEBUG_PORT_3 0x0A // Port for the third debug print
#define MAV_PORT     0x09
#define MAV_CHANNEL  0x00

// Define a system and component ID for the NRF chip.
// This identifies the NRF as the source of the MAVLink message.
#define MAV_SYSTEM_ID_NRF 2
#define MAV_COMP_ID_NRF_PMU 191 // MAV_COMP_ID_POWER_MANAGEMENT_UNIT

#ifdef BLE
int volatile bleEnabled = 1;
#else
int volatile bleEnabled = 0;
#endif

// New struct to store received packets to avoid race conditions
typedef struct {
  uint8_t size;
  uint8_t data[64];
  uint8_t rssi;
  bool broadcast;
} SafeRxPacket;

static SafeRxPacket safePacket; // Define a static instance of our safe buffer

static struct syslinkPacket slRxPacket;
static struct syslinkPacket slTxPacket;
static uint16_t syslink_message_id_counter = 0; // For Crazyflie Syslink Packet ID
static int radioRSSISendTime = SYSLINK_STARTUP_DELAY_TIME_MS;
static int vbatSendTime = SYSLINK_STARTUP_DELAY_TIME_MS;
static int vbatMAVSendTime = SYSLINK_STARTUP_DELAY_TIME_MS;
static int heartbeatSendTime = SYSLINK_STARTUP_DELAY_TIME_MS;
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
  //uint8_t dbg[3];
  //dbg[0] = safePacket.size;        // how many bytes the nRF saw
  //dbg[1] = safePacket.data[0];     // first byte
  //dbg[2] = safePacket.data[1];     // second byte
  // Queue a debug packet on DEBUG_PORT (0x0E), channel 0
  //esbSendDebugPacket(DEBUG_PORT, 0, (char*)dbg, sizeof(dbg));
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
      //uint8_t len = slTxPacket.length;
      //uint8_t dbg2[len + 1];
      //dbg2[0] = len;      // how many bytes the nRF saw  
      //for (int i = 0; i < len; i++) {
      //  dbg2[i + 1] = slTxPacket.data[i];
      //}
      //dbg2[1] = slTxPacket.data[0];     // first byte
      //dbg2[2] = slTxPacket.data[1];     // second byte
      // Queue a debug packet on DEBUG_PORT_2 (0x09), channel 0
      //esbSendDebugPacket(DEBUG_PORT_2, 0, (char*)dbg2, sizeof(dbg2));
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
          syslinkSend_buffered(&slTxPacket);  // to avoid weird interactions between buffered send and non-buffered send
        }
        break;
      case SYSLINK_RADIO_MAVLINK:
        // --- STM->GCS DEBUG ---
        //{ // Use braces to create a local scope for the debug variable
        //  uint8_t dbg_stm[3];
        //  dbg_stm[0] = slRxPacket.length;
        //  dbg_stm[1] = slRxPacket.data[0];
        //  dbg_stm[2] = slRxPacket.data[1];
          // Use DEBUG_PORT_3 (0x0A) to make this print distinct
        //  esbSendDebugPacket(DEBUG_PORT_3, 0, (char*)dbg_stm, sizeof(dbg_stm));
        //}
        // --- END OF DEBUG ---

        // The slRxPacket contains a MAVLink message from the STM.
        // Pass it to our new helper function which will wrap it in a CRTP header and queue it for radio transmission.
        esbSendSyslinkMavlinkPacket(&slRxPacket);
        
        break;
      case SYSLINK_PM_ONOFF_SWITCHOFF:
        pmSetState(pmAllOff);
        break;
      case SYSLINK_OW_GETINFO:
      case SYSLINK_OW_READ:
      case SYSLINK_OW_SCAN:
      case SYSLINK_OW_WRITE:
        if (memorySyslink(&slRxPacket)) {
          syslinkSend_buffered(&slTxPacket);  // to avoid weird interactions between buffered send and non-buffered send
        }
        break;
      case SYSLINK_RADIO_P2P_BROADCAST:
        // Check that bluetooth is disabled and disable it if not
        if (bleEnabled) {
          disableBle();
        }

        // --- STM->GCS DEBUG ---
        //{ // Use braces to create a local scope for the debug variable
        //  uint8_t dbg_stm[3];
        //  dbg_stm[0] = slRxPacket.length;
        //  dbg_stm[1] = slRxPacket.data[0];
        //  dbg_stm[2] = slRxPacket.data[1];
          // Use DEBUG_PORT_3 (0x0A) to make this print distinct
        //  esbSendDebugPacket(DEBUG_PORT_3, 0, (char*)dbg_stm, sizeof(dbg_stm));
        //}
        // --- END OF DEBUG ---

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

static bool syslink_MAVLink(const uint8_t *buffer, uint16_t len) 
{
  // Define the chunk size for fragmentation (64 [SYSLINK MTU] - 1 [CRTP HEADER] - 6 [MAVLINK HEADER])
  static const int MAV_CHUNK = 57;

  // Calculate how many fragments this MAVLink message will be split into
  uint8_t total_syslink_fragments = (len + MAV_CHUNK - 1) / MAV_CHUNK;  

  uint16_t syslink_fragmentation_full_id = syslink_message_id_counter++;
  uint8_t offset = 0;  

  while (offset < len)
  {
    uint8_t this_len = ((len - offset < MAV_CHUNK) ? (len - offset) : MAV_CHUNK);
    uint8_t length_field = 7 + this_len; // 1B CRTP HEADER + 6B fragment header + data
    uint8_t packet[SYSLINK_MTU + 7]; // Buffer for one fragment
    uint8_t idx = 0;    

    // 1) Syslink header
    packet[idx++] = 0xBC;
    packet[idx++] = 0xCF;
    packet[idx++] = SYSLINK_SYS_MAVLINK;
    packet[idx++] = length_field;    

    // 1) CRTP Header
    packet[idx++] = ((MAV_PORT & 0x0f) << 4 | 3 << 2 | (MAV_CHANNEL & 0x03));
    
    // 2) Fragment header
    packet[idx++] = (uint8_t)(syslink_fragmentation_full_id & 0xFF);
    packet[idx++] = (uint8_t)(syslink_fragmentation_full_id >> 8);
    packet[idx++] = (uint8_t)(len & 0xFF);
    packet[idx++] = (uint8_t)(len >> 8);
    packet[idx++] = total_syslink_fragments;
    packet[idx++] = (uint8_t)(offset / MAV_CHUNK);  
    
    // 3) Payload slice
    if (this_len > 0) {
      memcpy(&packet[idx], &buffer[offset], this_len);
    }
    
    idx += this_len;

    // 4) Fletcher-8 checksum
    uint8_t c0=0, c1=0;
    for (uint8_t j = 2; j < idx; j++) {
        c0 += packet[j];
        c1 += c0;
    }
    packet[idx++] = c0;
    packet[idx++] = c1;    

    // --- STM->GCS DEBUG ---
    //{ // Use braces to create a local scope for the debug variable
    //  uint8_t len = slTxPacket.length;
    //  uint8_t dbg_stm[len];
    //  dbg_stm[0] = len;      // how many bytes the nRF saw  
    //  for (int i = 0; i < len; i++) {
    //    dbg_stm[i + 1] = slTxPacket.data[i];
    //  }      
      // Use DEBUG_PORT_3 (0x0A)
    //  esbSendDebugPacket(DEBUG_PORT_3, 0, (char*)dbg_stm, sizeof(dbg_stm));
    //}
    // --- END OF DEBUG ---    

    // 5) Send the message over Syslink
    syslinkMAVSend_buffered(packet, idx);

    offset += this_len;    
  }

  return true;
}

static void syslinkMAVLinkHeartbeat()
{
  // Send the heartbeat to the STM at 1Hz
  if ((systickGetTick() >= heartbeatSendTime + 1000)) {
    heartbeatSendTime = systickGetTick();
    
    // The MAVLink message struct
    mavlink_message_t msg;
    // A buffer to hold the serialized message
    uint8_t mavPacket[SYSLINK_MTU];
    
    // Step 1: Pack the HEARTBEAT message
    // This tells QGC that a battery component is alive and on the network
    mavlink_msg_heartbeat_pack(
        MAV_SYSTEM_ID_NRF,              // System ID of the NRF
        MAV_COMP_ID_NRF_PMU,            // Component ID of the NRF's PMU
        &msg,                           // MAVLink message to pack into
        MAV_TYPE_BATTERY,               // Identify this component as a battery
        MAV_AUTOPILOT_INVALID,          // Not a full autopilot
        0,                              // base_mode (not applicable)
        0,                              // custom_mode (not applicable)
        MAV_STATE_ACTIVE                // System status
    );

    // Step 2: Serialize the message into a byte buffer
    uint16_t len = mavlink_msg_to_send_buffer(mavPacket, &msg);

    // Step 3: Send the message over Syslink
    syslink_MAVLink(mavPacket, len);
  }
}

static void syslinkMAVLinkBatteryMessages()
{
  // Send the battery voltage and state to the STM AT 10Hz
  if ((systickGetTick() >= vbatMAVSendTime + 1500)) {
    float vdata; 
    float tdata;
    int is_charging = MAV_BATTERY_CHARGE_STATE_OK;

    uint8_t flags = getPowerStatusFlags();

    if (flags & 0x01) {   // Check if the 'isCharging' bit (the first bit) is set
      is_charging = MAV_BATTERY_CHARGE_STATE_CHARGING;
    }
    
    // The MAVLink message struct
    mavlink_message_t msg;

    // A buffer to hold the serialized message
    uint8_t mavPacket[SYSLINK_MTU];
    
    vbatMAVSendTime = systickGetTick();

    vdata = pmGetVBAT();
    tdata = pmGetTemp();

    // Convert temperature from degrees to centidegrees Celsius
    int16_t temperature = (int16_t)(tdata * 100);

    // Step 1: Prepare the voltage array as per the definition.
    // The total voltage goes into the first element.
    uint16_t voltages[10];
    voltages[0] = (uint16_t)(vdata * 1000); // Overall voltage in mV

    // Fill the rest of the cell voltages with UINT16_MAX as they are unknown.
    for (int i = 1; i < 10; i++) {
        voltages[i] = UINT16_MAX;
    }

    // This array is for cells 11-14. Zero indicates they are not supported.
    uint16_t voltages_ext[4] = {0};

    // Step 2: Pack the message using the function signature.
    mavlink_msg_battery_status_pack(
        MAV_SYSTEM_ID_NRF,                  // System ID
        MAV_COMP_ID_NRF_PMU,                // Component ID
        &msg,                               // MAVLink message to pack into
        0,                                  // Battery ID
        MAV_BATTERY_FUNCTION_ALL,           // Battery function
        MAV_BATTERY_TYPE_LIPO,              // Type (chemistry)
        temperature,                        // Temperature: INT16_MAX for unknown
        voltages,                           // The voltages array
        -1,                                 // Current: -1 for unknown
        -1,                                 // Consumed charge: -1 for unknown
        -1,                                 // Consumed energy: -1 for unknown
        -1,                                 // Remaining percentage: -1 for unknown
        0,                                  // Time remaining: 0 for unknown
        is_charging,                        // Charge state
        voltages_ext,                       // Extended voltages array (cells 11-14)
        0,                                  // Mode: 0 for default/unsupported
        0                                   // Fault bitmask
    );

    // Step 3: Serialize the message into a byte buffer
    // This converts the C struct into a byte array that can be transmitted.
    // The function returns the final length of the message.
    uint16_t len = mavlink_msg_to_send_buffer(mavPacket, &msg);    

    // Step 4: Send the message over Syslink
    syslink_MAVLink(mavPacket, len);
  }
}

static void sendDataToStmOverSyslink()
{
  // Call the heartbeat function
  syslinkMAVLinkHeartbeat();
  
  // battery message function
  syslinkMAVLinkBatteryMessages();
  
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

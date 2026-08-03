---
title: Home
page_id: home 
---

Source code of the firmware running in the Crazyflie 2.X nRF51822.
This microcontroller has a couple of roles:
 - Power management (ON/OFF logic and battery handling)
 - Radio communication
   - Enhanced Shockburst, with a 252 byte payload rather than the legacy 32
   - MAVLink transport, in either broadcast-to-peers or telemetry mode
 - One-wire memory access
 - Output a 8mhz high precision clock to STM32.

This is a fork carrying the Crazyflie towards running ArduPilot on the STM32.
Bluetooth low energy has been removed, along with the S130 softdevice's RAM
reservation, to make room for the larger radio payload. See
[ArduPilot integration](development/ardupilot-integration.md) for the contract
the STM32 side has to implement.

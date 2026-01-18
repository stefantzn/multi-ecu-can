# Multi-ECU CAN Control & Diagnostics System

A multi-ECU CAN bus control and diagnostics system implemented using ESP32 (ESP-IDF) and a Raspberry Pi 5 verification node.  
The project models an automotive-style CAN network with independent ECUs exchanging heartbeat, status, and command messages, with fault detection and bus monitoring.

## Overview

This repository contains firmware for two independent ESP32-based ECUs connected via a CAN bus, along with a Raspberry Pi 5 used as a Linux-based CAN analysis and verification node.

**ECU A**
  - Periodic heartbeat transmission
  - Command handling (e.g., LED control)
  - Monitoring ECU B alive messages with CRC validation and timeout detection
  - CAN bus state monitoring and bus-off recovery

**ECU B**
  - Alive/status message transmission
  - State reporting (OK / DEGRADED / SAFE)
  - CAN error counter reporting and health monitoring

**Raspberry Pi 5**
  - SocketCAN interface
  - CAN traffic monitoring and injection using `can-utils`
  - Independent verification of bus behavior and message timing

## To-do

- Cleanup and add to README.md
- Shared `ecu_common` ESP-IDF component for protocol logic
- UDS-style diagnostic services

## Gallery

Physical bus mockup

![alt text](/photos/bus_mockup.png)

ECU_A diagnostic messages

![alt text](/photos/ecu_a.png)

ECU_B diagnostic messages

![alt text](/photos/ecu_b.png)

cangaroo bus diagnostic data

![alt text](/photos/cangaroo.png)
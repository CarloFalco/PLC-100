# STEP 01 — Project Setup (Corrected for 2MB Flash)

## Overview
Initial setup of a XIAO ESP32-C6 Zigbee End Device using ESP-IDF with an explicit 2MB flash configuration.

This step fixes flash size assumptions and provides a valid partition table compatible with ESP-Zigbee.

## Hardware
- Board: Seeed Studio XIAO ESP32-C6
- Flash: 2MB
- PSRAM: None

## Toolchain
- PlatformIO
- ESP-IDF
- C++17

## Flash Configuration
- Flash size explicitly set to 2MB
- Custom partition table used
- sdkconfig.defaults provided

## Partitions
- NVS
- PHY Init
- Factory application (single-slot)

OTA is intentionally disabled due to flash constraints.

## Files Added
- partitions.csv
- sdkconfig.defaults

## Commands
```bash
pio run
pio run -t menuconfig

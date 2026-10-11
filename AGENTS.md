# TMCStepper Library (MarlinFirmware Fork)

This is the **MarlinFirmware fork** of the TMCStepper Arduino library, a comprehensive library for configuring Trinamic stepper motor drivers. This document serves as a guide for both humans and agents working with this codebase to understand its structure, purpose, and usage patterns.

## About This Fork

This repository is **not** the original TMCStepper library. It is a fork maintained by the Marlin Firmware project.

| | Original | MarlinFirmware Fork (this repo) |
| --- | --- | --- |
| Repository | [teemuatlut/TMCStepper](https://github.com/teemuatlut/TMCStepper) | [MarlinFirmware/TMCStepper](https://github.com/MarlinFirmware/TMCStepper) |
| Author / Maintainer | Teemu Mäntykallio (@teemuatlut) | Marlin Firmware team |
| Versions | up to `v0.7.3` (2021) | `v0.8.0` (Nov 2024) and later |
| Primary consumer | General Arduino / embedded users | [Marlin Firmware](https://github.com/MarlinFirmware/Marlin) |

Key points:

- The fork branched from upstream `v0.7.3`. All `v0.8.x` and later tags belong to this fork only and do not exist upstream.
- The fork changes freely to suit Marlin Firmware: new drivers (e.g., TMC2240), bug fixes, refactors, and API changes. It does not need to stay compatible with upstream, and changes are not expected to be sent back upstream.
- Marlin's PlatformIO configuration pulls the library from this repository by version tag, so a release means bumping the version in **both** `library.json` and `library.properties` and creating a matching `vX.Y.Z` tag.
- Changes should be judged by how they affect Marlin builds across its many supported platforms (AVR, STM32, ESP32, LPC, RP2040, etc.). Watch out for flash/RAM usage on small AVR boards.
- Credit for the original library stays with @teemuatlut. Keep the existing attribution in file headers and the README.

### Git Remotes

In a typical development checkout:

- `origin` → `MarlinFirmware/TMCStepper` (this fork; PRs target `master`)
- `upstream` → `teemuatlut/TMCStepper` (the original, for reference only)

### Notable Fork Changes (since v0.7.3)

- TMC2240 driver support (SPI), including StallGuard4, temperature reading, and correct current scaling
- The single large header split into per-class headers under `src/source/`
- Register initialization cleanup and `REG` macro usage
- Microsteps lookup table
- TMC2208/2209: longer timeout, configurable baud rate
- Software/hardware serial can be forced with `-DTMCSTEPPER_SW_SERIAL=1` or `=0`
- RP2040 single-wire UART write path for TMC2209
- Memory leak fix for TMC2130

See `git log upstream/master..HEAD` for the full list.

## Overview

The TMCStepper library is designed to provide easy and consistent configuration of Trinamic stepper motor drivers via SPI or UART interfaces. It supports TMC2130, TMC2160, TMC2208, TMC2209, TMC2224, TMC2240, TMC2660, TMC5130, and TMC5160/5161.

## Project Structure

```
TMCStepper/
├── src/
│   ├── TMCStepper.h           # Main header; includes all per-class headers
│   ├── TMCStepper_UTILITY.h   # Debug/utility helpers
│   └── source/
│       ├── TMCStepper.cpp     # Base class implementation, current calculation
│       ├── TMCStepperBase.h   # Base class declaration
│       ├── TMCxxxxStepper.h/.cpp   # Per-driver class declaration and implementation
│       ├── TMCxxxx_bitfields.h     # Per-driver register bitfield definitions
│       ├── <REGISTER>.cpp     # Register accessors (CHOPCONF, COOLCONF, PWMCONF, ...)
│       ├── SW_SPI.cpp/h       # Software SPI
│       ├── SERIAL_SWITCH.cpp/h
│       ├── bcm2835_*.cpp/h    # Raspberry Pi support
│       ├── TMC_MACROS.h
│       └── TMC_platforms.h
├── examples/                  # Example sketches
├── docs/                      # Doxygen-generated documentation
├── library.json               # PlatformIO metadata (version)
└── library.properties         # Arduino IDE metadata (version)
```

## Supported Drivers

* TMC2130 - SPI driver with StallGuard2 and CoolStep
* TMC2160 - TMC2130-style SPI driver with external MOSFETs
* TMC2208 - UART driver with StealthChop2
* TMC2209 - TMC2208 with StallGuard4 and CoolStep
* TMC2224 - TMC2208 variant
* TMC2240 - SPI driver with StallGuard4, temperature/voltage ADC (fork addition)
* TMC2660 - SPI driver with StallGuard2 and CoolStep, no StealthChop
* TMC5130 - SPI driver with integrated motion controller
* TMC5160/5161 - SPI drivers with integrated motion controller (5160 uses external MOSFETs)

## Key Design Patterns

### Class Hierarchy

```
TMCStepper
  TMC2130Stepper
    TMC2160Stepper
      TMC5130Stepper
        TMC5160Stepper
          TMC5161Stepper
  TMC2208Stepper
    TMC2209Stepper
    TMC2224Stepper
TMC2660Stepper   (standalone)
TMC2240Stepper   (standalone)
```

### Communication Methods

The library supports multiple communication methods:
- Hardware SPI for most drivers
- Software SPI for platforms that don't support hardware SPI
- Hardware UART for TMC2208 and TMC2209 drivers
- Software UART (via `SoftwareSerial`) for platforms that support it

### Configuration System

Driver configuration is done through register access methods:
- Read/Write register methods (e.g., `CHOPCONF()`)
- Convenience methods for configuration (e.g., `rms_current()`, `microsteps()`)
- Bitfield-based bit manipulation for fine-grained control
- Factory defaults and push/pull capabilities

### Configuration Parameters

Key parameters that can be configured:
- Current settings (`rms_current()`, `cs2rms()`)
- Microstepping (`microsteps()`)
- Driver behavior (`en_pwm_mode()`, `en_spreadCycle()`)
- Chopper configuration (`CHOPCONF`)
- PWM configuration (`PWMCONF`)
- StallGuard settings (`COOLCONF`, `SGTHRS`)

## Key Files

### src/TMCStepper.h
- Main header that users include
- Includes the bitfield and class headers for every driver from `src/source/`
- Documents the class inheritance hierarchy

### src/source/TMCStepper.cpp
- Implementation of base class functions
- Utility functions for current calculation

## Examples

Example sketches in the `examples/` directory:
- **Simple** - Basic usage for different driver types
- **StallGuard** - Using StallGuard (SG) functionality
- **StallGuard_TMC2209** - TMC2209-specific StallGuard example
- **StallGuard_TMC2660** - TMC2660-specific StallGuard example
- **TMC2240_SPI** / **TMC2240_UART** - TMC2240 examples
- **TMC_AccelStepper** - Integration with AccelStepper library
- **Calibrate_spreadCycle** - How to calibrate spreadCycle settings
- **Live_tune** - Real-time tuning of driver parameters
- **MoveByUART** - Driving motors via UART commands
- **Software_SPI** - Example of using software SPI
- **stealthChop** - StealthChop configuration

## Version Information

- Current fork version: 0.8.9 (see `library.json` / `library.properties`)
- Fork versions start at 0.8.0; upstream ended at 0.7.3
- Compatible with Arduino IDE and PlatformIO
- Supports various platforms (Arduino AVR, ESP32, Raspberry Pi, STM32, RP2040, etc.)

## Documentation

Doxygen output for this fork is generated into `docs/`. The original library's documentation is at https://teemuatlut.github.io/TMCStepper/index.html. It does not cover fork-only additions such as TMC2240.

## Key Concepts

### Current Configuration
- Current is typically set in RMS mA
- Requires proper sensing resistor value (R_SENSE) in the circuit (TMC2240 uses an `Rref` resistor instead)
- Uses `rms_current()` function for setting current level
- `cs2rms()` function converts raw current sense code to mA

### Microstepping Support
- Configurable from full step to 1/256 microsteps
- Uses the `microsteps()` function
- Implemented differently for different driver types (SPI vs UART)

### Configuration Persistence
- The library keeps shadow copies of write-only registers
- Use `push()` to write the stored register configuration back to the driver
- `defaults()` resets the shadow registers to factory default values

### Communication Protocols
- SPI: Hardware or software implementation
- UART: For communication with TMC2208/2209 drivers
- Uses CRC validation for UART communications

## Troubleshooting

- If your driver doesn't respond, check:
  - SPI connections
  - Chip select wiring
  - UART connection (baud rate, correct pins selected)
- Verify R_SENSE resistor matches the hardware
- Use `test_connection()` to verify driver communication

The original library was in development for many years by @teemuatlut and is widely used by hobbyists and professionals working with Trinamic drivers. This fork continues that work for Marlin Firmware while keeping a clean and consistent API across drivers and platforms.

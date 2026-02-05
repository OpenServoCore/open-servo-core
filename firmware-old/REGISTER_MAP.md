# Open Servo Core Register Map

This document defines the register map for the Open Servo Core, following Dynamixel Protocol 2.0 conventions for compatibility with existing tools and libraries.

## Overview

The register map is divided into two areas following Dynamixel standards:
- **EEPROM Area (0x00-0x3F)**: Non-volatile configuration, persists across power cycles
- **RAM Area (0x40-0xFB)**: Runtime state and control registers, reset on power cycle

All multi-byte values use **little-endian** byte order as per Dynamixel Protocol 2.0.

## Unit Translation and Quantization

The servo maintains high-precision internal units for accuracy while providing Dynamixel-compatible protocol units for external communication. Translation occurs transparently at the register interface.

**Important**: Standard registers (0-251) are compatibility views that quantize internal values. For full resolution access, use vendor high-resolution registers (0x0200+) when available. Quantization can introduce small errors - use high-res registers for precision control applications.

| Data Type | Internal Unit | Protocol Unit | Translation Formula |
|-----------|--------------|---------------|---------------------|
| Position | Centidegrees (0.01°) | Pulses (0.088°/pulse) | `protocol = round((internal * 4096) / 36000)` nearest, single-turn only |
| Velocity | 0.1°/s (signed) | 0.229 rpm/unit (signed) | `protocol = trunc((internal * 1000) / 13740)` toward zero |
| Current | mA | mA | No conversion |
| Temperature | CentiC (0.01°C) | 1°C | `protocol = floor(internal / 100)` |
| Voltage | mV | 0.1V | `protocol = floor(internal / 100)` |
| PWM | ±32767 (internal) | ±885 (±100%) | `protocol = round((internal * 885) / 32767)` nearest |
| PID Gains | - | 0-16383 | Registers store Dynamixel table values. Internal controller uses scaled gains per ROBOTIS formulas |

## EEPROM Area (Non-Volatile Memory)

**Note**: EEPROM can only be written when Torque Enable (64) is 0.

| Address | Size | Name | Access | Default | Range | Unit | Description |
|---------|------|------|--------|---------|-------|------|-------------|
| 0 | 2 | Model Number | R | 1200 | - | - | Servo model identifier |
| 2 | 4 | Model Information | R | - | - | - | Additional model info |
| 6 | 1 | Firmware Version | R | - | - | - | Firmware version |
| 7 | 1 | ID | RW | 1 | 1~252 | - | Servo network ID (254/0xFE reserved for broadcast) |
| 8 | 1 | Baud Rate | RW | 1 | 0~6 | - | 0:9600, 1:57600, 2:115200, 3:1M, 4:2M, 5:3M, 6:4M |
| 9 | 1 | Return Delay Time | RW | 250 | 0~254 | 2μs | Response delay |
| 10 | 1 | Drive Mode | RW | 0 | 0~13 | - | Motor/profile configuration |
| 11 | 1 | Operating Mode | RW | 3 | 0,1,3,16 | - | 0:Current, 1:Velocity, 3:Position (single-turn), 16:PWM. Modes 4,5 not implemented |
| 12 | 1 | Secondary ID | RW | 255 | 0~252 | - | Secondary (shadow) ID |
| 13 | 1 | Protocol Type | R | 2 | 2 | - | Fixed at 2 (Dynamixel 2.0 only) |
| 20 | 4 | Homing Offset | RW | 0 | -1044479~1044479 | pulse | Zero position offset |
| 24 | 4 | Moving Threshold | RW | 10 | 0~1023 | 0.229rpm | Velocity threshold for moving detection |
| 31 | 1 | Temperature Limit | RW | 70 | 0~100 | °C | Maximum operating temperature |
| 32 | 2 | Max Voltage Limit | RW | 84 | 31~84 | 0.1V | Maximum input voltage (8.4V for 2S LiPo) |
| 34 | 2 | Min Voltage Limit | RW | 35 | 31~84 | 0.1V | Minimum input voltage (3.5V for 1S LiPo cutoff) |
| 36 | 2 | PWM Limit | RW | 885 | 0~885 | 0.113% | Maximum PWM (100%) |
| 38 | 2 | Current Limit | RW | 1750 | 0~1750 | mA | Maximum current |
| 44 | 4 | Velocity Limit | RW | 445 | 0~1023 | 0.229rpm | Maximum velocity |
| 48 | 4 | Max Position Limit | RW | 4095 | 0~4095 | pulse | Maximum position (360°) |
| 52 | 4 | Min Position Limit | RW | 0 | 0~4095 | pulse | Minimum position (0°) |
| 60 | 1 | Startup Configuration | RW | 0 | 0~3 | - | Startup behavior |
| 62 | 1 | PWM Slope | RW | 140 | 1~255 | - | PWM acceleration |
| 63 | 1 | Reserved | - | - | - | - | Reserved |

## RAM Area (Volatile Memory)

| Address | Size | Name | Access | Range | Unit | Description |
|---------|------|------|--------|-------|------|-------------|
| **Control Registers** |
| 64 | 1 | Torque Enable | RW | 0~1 | - | 0:Disabled, 1:Enabled |
| 65 | 1 | LED | RW | 0~1 | - | 0:Off, 1:On |
| 68 | 1 | Status Return Level | RW | 0~2 | - | 0:PING only, 1:PING/READ, 2:All |
| 69 | 1 | Registered Instruction | R | 0~1 | - | 0:No, 1:Yes |
| 70 | 1 | Hardware Error Status | R | - | - | Bit field of hardware errors |
| **PID Gains** |
| 76 | 2 | Velocity I Gain | RW | 0~16383 | - | I Gain (KVI = value/65536) Initial: 1600 |
| 78 | 2 | Velocity P Gain | RW | 0~16383 | - | P Gain (KVP = value/128) Initial: 180 |
| 80 | 2 | Position D Gain | RW | 0~16383 | - | D Gain (KPD = value/16) Initial: 0 |
| 82 | 2 | Position I Gain | RW | 0~16383 | - | I Gain (KPI = value/65536) Initial: 0 |
| 84 | 2 | Position P Gain | RW | 0~16383 | - | P Gain (KPP = value/128) Initial: 400 |
| 88 | 2 | Feedforward 2nd Gain | RW | 0~16383 | - | Acceleration FF (KFF2 = value/4) Initial: 0 |
| 90 | 2 | Feedforward 1st Gain | RW | 0~16383 | - | Velocity FF (KFF1 = value/4) Initial: 0 |
| 98 | 1 | Bus Watchdog | RW | 0~127 | 20ms | 0:Disabled, 1~127:timeout |
| **Goal Values** |
| 100 | 2 | Goal PWM | RW | -885~885 | 0.113% | Target PWM (-885=100% reverse, 885=100% forward) |
| 102 | 2 | Goal Current | RW | -Current Limit~Current Limit | mA | Target current |
| 104 | 4 | Goal Velocity | RW | -Velocity Limit~Velocity Limit | 0.229rpm | Target velocity |
| 108 | 4 | Profile Acceleration | RW | 0~32767 | 214.577rev/min² | Profile acceleration |
| 112 | 4 | Profile Velocity | RW | 0~32767 | 0.229rpm | Profile max velocity |
| 116 | 4 | Goal Position | RW | Min Position~Max Position | pulse | Target position |
| **Feedback Values** |
| 120 | 2 | Realtime Tick | R | - | ms | System time in milliseconds |
| 122 | 1 | Moving | R | 0~1 | - | 0:Stopped, 1:Moving |
| 123 | 1 | Moving Status | R | - | - | Bit0:In-Position, Bit1:Profile Ongoing, Bit2:Following Error, Bit3:Velocity Profile, Bit4:Position Trajectory |
| 124 | 2 | Present PWM | R | - | 0.113% | Current PWM output |
| 126 | 2 | Present Current | R | - | mA | Current draw |
| 128 | 4 | Present Velocity | R | - | 0.229rpm | Current velocity |
| 132 | 4 | Present Position | R | - | pulse | Current position |
| 136 | 4 | Velocity Trajectory | R | - | 0.229rpm | Velocity trajectory value |
| 140 | 4 | Position Trajectory | R | - | pulse | Position trajectory value |
| 144 | 2 | Present Input Voltage | R | - | 0.1V | Input voltage |
| 146 | 1 | Present Temperature | R | - | °C | Internal temperature |
| 147 | 1 | Backup Ready | R | 0~1 | - | 0:Backup in progress, 1:Ready |
| **Indirect Access** |
| 168 | 2 | Indirect Address 1 | RW | 64~251 | - | First indirect address (RAM only*) |
| 170 | 2 | Indirect Address 2 | RW | 64~251 | - | Second indirect address (RAM only*) |
| ... | ... | ... | ... | ... | ... | ... |
| 222 | 2 | Indirect Address 28 | RW | 64~251 | - | 28th indirect address (RAM only*) |
| 224 | 1 | Indirect Data 1 | RW | - | - | Data at Indirect Address 1 |
| 225 | 1 | Indirect Data 2 | RW | - | - | Data at Indirect Address 2 |
| ... | ... | ... | ... | ... | ... | ... |
| 251 | 1 | Indirect Data 28 | RW | - | - | Data at Indirect Address 28 |

*Note: Indirect Address range (64-251) restricts mapping to standard RAM area only. This is an implementation choice - some Dynamixel models allow broader ranges. Vendor region (0x0200+) cannot be mapped through indirect access by design.

## Extended Registers (Custom Implementation)

**Important Note**: Addresses 252-255 (0xFC-0xFF) are not part of the ROBOTIS standard control table. 

### Tradeoffs of Using 252+:
- **Pros**: 
  - Stays within u8 address range for older SDK compatibility
  - Doesn't collide with Indirect Data (ends at 251)
  - Convenient for quick custom features
- **Cons**: 
  - Not part of ROBOTIS standard - some tools may assume table ends at 251
  - Limited to 4 addresses before hitting 256 boundary
  - May conflict with future Dynamixel extensions

### Alternative Approach:
For better future-proofing, consider using address 0x0200+ (512+) as a vendor-specific region, though this requires u16 addressing support. A hybrid approach could mirror critical values in both locations.

These registers extend the standard Dynamixel protocol with servo-specific features:

| Address | Size | Name | Access | Unit | Description |
|---------|------|------|--------|------|-------------|
| **Quick Access (252-255)** |
| 252 | 1 | Compliance Mode | R | - | 0:Move, 1:Hold, 2:Yield |
| 253 | 1 | Compliance Limited | R | - | 0:No, 1:Yes |
| 254 | 2 | Fault Status Extended | R | - | Extended fault info |
| **Vendor Specific Region (0x0200+)** |
| **High-Resolution Access** |
| 512-515 | 4 | Goal Position CDEG | RW | cdeg | Goal position (I32 centidegrees, future-proof) |
| 516-519 | 4 | Present Position CDEG | R | cdeg | Current position (I32 centidegrees) |
| 520-523 | 4 | Goal Velocity DPS10 | RW | 0.1°/s | Goal velocity (I32, future-proof for profiles) |
| 524-525 | 2 | Present Temperature | R | centiC | Temperature (I16 centiC, allows negative) |
| 526-527 | 2 | Present Voltage | R | mV | Voltage (U16 millivolts, 0-65535) |
| 528-529 | 2 | Present Current | R | mA | Current (I16 milliamps, signed) |
| 530-533 | 4 | Position Min Limit CDEG | RW | cdeg | Minimum position limit (I32 centidegrees) |
| 534-537 | 4 | Position Max Limit CDEG | RW | cdeg | Maximum position limit (I32 centidegrees) |
| 538-539 | 2 | Control Flags | R | - | Status/control source flags (see below) |
| **Compliance Control** |
| 540 | 1 | Compliance Mode | R | - | 0:Move, 1:Hold, 2:Yield (mirrors 252) |
| 541 | 1 | Compliance Limited | R | - | 0:No, 1:Yes (mirrors 253) |
| 542 | 2 | Move Current Limit | RW | mA | Current limit in Move mode |
| 544 | 2 | Hold Current Limit | RW | mA | Current limit in Hold mode |
| 546 | 2 | Compliance Hysteresis | RW | mA | Recovery hysteresis |
| 548 | 1 | Compliance Deglitch | RW | samples | Samples before limiting |
| 549 | 2 | Backoff Factor | RW | Q8 | Duty backoff factor |
| 551 | 2 | Recovery Rate | RW | duty/s | Compliance recovery rate |
| **Mode Thresholds** |
| 553 | 2 | Hold Enter Error | RW | pulse | Error to enter Hold mode |
| 555 | 2 | Hold Exit Error | RW | pulse | Error to exit Hold mode |
| 557 | 2 | Hold Enter Velocity | RW | 0.229rpm | Velocity to enter Hold |
| 559 | 2 | Hold Exit Velocity | RW | 0.229rpm | Velocity to exit Hold |
| 561 | 2 | Backdrive Velocity | RW | 0.229rpm | Backdrive detection threshold |
| **Debug/Telemetry** |
| 563 | 4 | Control Loop Counter | R | ticks | Control loop tick count |
| 567 | 2 | Control Output | R | - | Raw controller output |
| 569 | 2 | Integral Term | R | - | PID integral accumulator |
| 571 | 2 | Derivative Term | R | - | PID derivative term |
| 573 | 2 | Loop Time | R | μs | Control loop execution time |
| 575 | 1 | Fault Status | R | - | Current fault code |
| 576 | 1 | Fault Count | R | - | Total fault count |
| 577 | 4 | Fault History | R | - | Last 4 fault codes |

## Control Flags Register (Address 538)

| Bit | Name | Description |
|-----|------|-------------|
| 0 | Position Source | 0: Using standard Goal Position (116), 1: Using vendor Goal Position CDEG (512) |
| 1 | Velocity Source | 0: Using standard Goal Velocity (104), 1: Using vendor Goal Velocity DPS10 (520) |
| 2 | Multi-Turn Capable | 0: Single-turn only, 1: Multi-turn supported |
| 3 | Internal Storage | 0: Internal uses i16, 1: Internal uses i32 |
| 4-15 | Reserved | Reserved for future use |

## Operating Modes

| Value | Mode | Status | Description |
|-------|------|--------|-------------|
| 0 | Current Control | Implemented | Direct current/torque control |
| 1 | Velocity Control | Implemented | Continuous rotation with velocity control |
| 3 | Position Control | Implemented | Single-turn position control (0-4095 = 0-360°) |
| 4 | Extended Position Control | Not Implemented | Would allow multi-turn, returns error if selected |
| 5 | Current-based Position | Not Implemented | Would combine position + current limit, returns error |
| 16 | PWM Control | Implemented | Direct PWM control (-885 to 885) |

## Hardware Error Status Bits (Address 70)

| Bit | Name | Description |
|-----|------|-------------|
| 0 | Input Voltage Error | Input voltage outside configured range |
| 1 | - | Unused (always 0) |
| 2 | Overheating Error | Internal temperature exceeded limit |
| 3 | - | Unused (always 0) |
| 4 | Electrical Shock Error | Overcurrent or insufficient power detected |
| 5 | Overload Error | Persistent load exceeding maximum output |
| 6 | - | Unused (always 0) |
| 7 | - | Unused (always 0) |

## Status Return Level

| Value | Description |
|-------|-------------|
| 0 | No return (except PING) |
| 1 | Return only for READ commands |
| 2 | Return for all commands |

## Baud Rate Index

| Value | Baud Rate |
|-------|-----------|
| 0 | 9,600 bps |
| 1 | 57,600 bps |
| 2 | 115,200 bps |
| 3 | 1 Mbps |
| 4 | 2 Mbps |
| 5 | 3 Mbps |
| 6 | 4 Mbps |

## Implementation Notes

1. **Compatibility**: This register map follows common X-series control table conventions for core registers. Standard Dynamixel tools can read/write by address, though model-specific UI features require a custom profile.

2. **Unit Translation**: Internal high-precision units are automatically converted to/from protocol units at the register interface layer. Exception: PID gains are stored as raw Dynamixel values.

3. **PID Gain Handling**: 
   - Registers store raw Dynamixel table values (0-16383)
   - Control loop converts to actual gains using ROBOTIS formulas:
     - P gains: Kp = register_value / 128
     - I gains: Ki = register_value / 65536
     - D gains: Kd = register_value / 16
     - Feedforward gains: Kff = register_value / 4
   - This ensures compatibility with Dynamixel tuning tools

4. **Custom Extensions**: Addresses 252+ contain servo-specific features not part of standard Dynamixel protocol.

5. **Atomic Operations**: Multi-byte registers must be read/written atomically to prevent data tearing.

6. **EEPROM Protection**: EEPROM registers can only be modified when Torque Enable (64) is 0.

7. **Position Range**: Position uses 12-bit resolution (0-4095) representing 0-360° for single-turn mode.

8. **Voltage Range**: Default limits (3.5-8.4V) are designed for 1S-2S LiPo battery operation. 1S LiPo: 3.7V nominal (3.5V cutoff, 4.2V max). 2S LiPo: 7.4V nominal (7.0V cutoff, 8.4V max).

9. **Indirect Access**: Allows efficient bulk read/write of non-contiguous registers by mapping them to contiguous indirect data registers.

10. **Quantization and Rounding**: 
    - Position: Round to nearest (minimizes average error)
    - Velocity: Truncate toward zero (prevents oscillation)
    - Temperature/Voltage: Floor (conservative for safety)
    - PWM: Round to nearest (balanced output)
    - High-resolution vendor registers (0x0200+) provide unquantized access

11. **Vendor Register Future-Proofing**:
    - Position and velocity vendor registers are i32 for future compatibility
    - Current firmware uses i16 internally (Control Flags bit 3 = 0)
    - Writes: i32 values are clamped to i16 range internally
    - Reads: i16 values are sign-extended to i32
    - Future firmware can upgrade to i32 internal storage without protocol changes
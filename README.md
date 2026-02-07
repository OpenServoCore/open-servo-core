# open-servo-core

Democratize Robotics For Everyone

## Overview

This project aims to create a drop-in replacement control board for SG90 & MG90-class servos that adds cascade control, current sensing, and a DYNAMIXEL-style serial protocol. The goal is to make robotics more accessible by turning a $2.50 servo into a $5 smart actuator (vs $30+ for commercial alternatives). Built with bare-metal Rust on CH32V microcontrollers.

## Project Status

🚧 **Early Development** - Not ready for general use

### ✅ **Working**

- **Hardware:** STM32 development board with motor control and sensing
- **Firmware:** Basic PID control, PWM motor drive, current sensing, ADC readings (see `firmware-old/`)

### 🔄 **In Progress**

- **Hardware:** Create CH32V006-based development board
- **Firmware:** System bringup with new development board, architecture redesign to simplify current design

## MVP Goals

**Protocol Compatibility:**

- Single-wire, half-duplex UART communication using Dynamixel Protocol 2.0
- Four core commands: Ping, Read, Write, Reset
- Basic control table functionality with standard register addresses
- Error Code 2 (Instruction Error) for unimplemented features

**Control System:**

- Basic PID position control
- Torque Enable/Disable functionality
- PID tuning registers: Position P/I/D Gains (Address 80, 82, 84)
- Homing Offset register (Address 20) for position calibration

**Safety Features:**

- Current limiting (Current Limit register, Address 38)
- Temperature protection (Temperature Limit register, Address 31)
- Voltage protection (Min/Max Voltage Limit registers, Address 32/34)
- Position boundary enforcement (Angle Limit Error detection)
- Configurable shutdown conditions (Shutdown register, Address 63)
- Error status reporting (Hardware Error Status register, Address 70)

**Target Cost:** Under $5 per modified servo while maintaining drop-in compatibility with SG90 & MG90 servos.

## Hardware

**CH32V006-based Control Boards:**

**Development Board:**

- Designed for firmware development with easy pin access and test points
- No size constraints for convenient debugging and prototyping
- Current sensing via shunt resistor
- Utilizes existing servo potentiometer for position feedback
- Motor terminal voltage sensing
- Battery/Vin voltage monitoring

**Production Board:**

- Compact form factor designed to replace servo internal control boards
- Drop-in replacement maintaining mechanical compatibility
- Same sensing capabilities as development board

**Power System:**

- Input: 1-2S LiPo battery (3.7V - 8.4V)
- Motor drive: Direct Vin voltage
- Logic: 3.3V regulated

**Sensing Capabilities:**

- Motor current via shunt resistor
- Position via existing potentiometer
- Motor terminal voltages
- Input voltage monitoring

## Contributing

This is experimental research code in active development. Issues and discussions are welcome. Check the hardware folders for KiCad designs and the firmware folder for the Rust implementation.

## License

MIT License - See [LICENSE](LICENSE) file for details

# SG90 Servo Controller Board - CH32V006

A compact, high-precision servo controller designed to replace the control circuitry in SG90/MG90 servo clones. This single-sided board fits within a 10x12.5mm footprint while providing advanced features like back-EMF sensing, overcurrent protection, and 0.1-0.2° position precision.

## Features

- **MCU**: CH32V006F8U6 RISC-V microcontroller (48MHz, 62KB flash, 8KB RAM)
- **Motor Driver**: Integrated DRV8837 H-bridge (1.8A peak, IN1/IN2 dual PWM control)
- **Position Sensing**: 12-bit ADC with RC filtering for 0.1-0.2° precision
- **Current Sensing**: 0.1Ω shunt resistor with ADC monitoring
- **Voltage Sensing**: Motor phase and input voltage monitoring via resistor dividers
- **Communication**: 2Mbps half-duplex UART over single DATA line
- **Power**: 3.3-8.4V input (1-2S LiPo compatible), 3.3V logic via LDO
- **Size**: 10x12.5mm, single-sided assembly
- **Debug**: 4-pin SWD interface for programming with WCH-LinkE

## Specifications

- Input Voltage (VIN): 3.3-8.4V (1S-2S LiPo compatible)
- Logic Voltage: 3.3V (regulated via TPAP2210K LDO)
- Continuous Current: 200mA (typical servo operation)
- Peak Current: 1.8A (motor driver limit)
- Position Resolution: 0.044°/count (4096 counts over 180°)
- PWM Frequency: 10kHz center-aligned
- Communication: 2Mbps half-duplex UART
- Temperature Range: -40°C to +85°C

## Bill of Materials (BOM)

| Ref | Description | Package | LCSC # | Qty | Unit Price @5 | Unit Price @150 | Unit Price @500 |
|-----|-------------|---------|--------|-----|---------------|-----------------|-----------------|
| U1 | TPAP2210K-3.3TRG1 3.3V LDO | SOT-23-5 | C49190789 | 1 | $0.0767 | $0.052 | $0.0458 |
| U2 | SN74LVC1G07DRLR Buffer | SOT-553 1.6x1.2mm | C19829624 | 1 | $0.0509 | $0.0447 | $0.0393 |
| U3 | DRV8837DSGR-JSM Motor Driver | WSON-8 2x2mm | C22447132 | 1 | $0.0982 | $0.0662 | $0.0582 |
| U4 | CH32V006F8U6 MCU | QFN-20 3x3mm | C42431288 | 1 | $0.2643 | $0.1838 | $0.1537 |
| D1 | PESD5V0L1UL ESD Protection | SOD-882 | C3001948 | 1 | $0.02 | $0.01 | $0.008 |
| R1,R2 | 1kΩ (DATA pullup, buffer pullup) | 0603 | Generic | 2 | $0.01 | $0.002 | $0.002 |
| R3,R6,R8,R10,R12 | 10kΩ (nSLEEP pulldown, voltage divider low) | 0603 | Generic | 5 | $0.01 | $0.002 | $0.002 |
| R4 | 0.1Ω 1% (current shunt) | 1206 | Generic | 1 | $0.02 | $0.003 | $0.003 |
| R5 | 330Ω (series resistor) | 0603 | Generic | 1 | $0.01 | $0.002 | $0.002 |
| R7,R9,R11 | 20kΩ (voltage divider high) | 0603 | Generic | 3 | $0.01 | $0.002 | $0.002 |
| C1a,C1b | 10µF 25V X5R (bulk, parallel) | 0603 | Generic | 2 | $0.02 | $0.005 | $0.004 |
| C2,C3 | 1µF 16V X7R (LDO output) | 0603 | Generic | 2 | $0.01 | $0.003 | $0.003 |
| C4,C6,C7,C8 | 0.1µF 16V X7R (decoupling) | 0603 | Generic | 4 | $0.01 | $0.002 | $0.002 |
| C5 | 100nF (RC filter) | 0603 | Generic | 1 | $0.01 | $0.002 | $0.002 |
| NT1,NT2 | Net tie (ground connections) | - | - | 2 | - | - | - |
| **Components Total** | | | | | **~$0.73** | **~$0.40** | **~$0.36** |
| **PCB** | 10x12.5mm 8-layer | - | - | 1 | **$2.00** | **$0.30** | **$0.15** |
| **TOTAL per board** | | | | | **~$2.73** | **~$0.70** | **~$0.51** |

*Prices shown are from LCSC in USD. Passive component prices are estimates for generic parts. 8-layer PCB pricing estimated for JLCPCB.*

## Pinout

### Servo Input Connector (J1) - 3 pins
1. VIN (3.3-8.4V)
2. DATA (Half-duplex UART, 2Mbps)
3. GND

### Motor Connector (J2) - 2 wire pads
1. OUTA - Motor terminal A
2. OUTB - Motor terminal B

### Potentiometer Connector (J3) - 3 pads
1. +3.3V - Potentiometer power
2. POT - Position feedback (wiper)
3. GND - Potentiometer ground

### Debug Interface (J4) - 4 pads, 1.27mm pitch
1. +3.3V - Power from debugger
2. GND - Ground
3. SWDIO - Single-wire debug
4. NC - Not connected

## Design Notes

### Power Architecture
- **Dual-rail design:** VIN (3.3-8.4V) for motor, 3.3V regulated for logic
- **LDO:** TPAP2210K-3.3 (20V max input, 250mA) provides 3.3V rail for MCU and analog circuits
- **Motor power:** Direct from VIN through DRV8837 H-bridge
- **Ground separation:** PGND (power) and GND (signal) joined via net tie for star grounding
- **Current sensing:** 0.1Ω shunt between motor driver and PGND
- **Bulk capacitance:** 2× 10µF (20µF total) on VIN handles motor transients

### ADC Precision
- 12-bit ADC provides 22.8 counts per degree (0.044°/count theoretical)
- RC filter (10kΩ + 100nF) on POT input reduces wiper noise (fc ≈ 160Hz)
- 4-16x oversampling in firmware improves effective resolution
- Center-aligned PWM with mid-duty ADC sampling avoids switching noise

### Voltage Sensing
All voltage sensing uses 20kΩ/10kΩ dividers (ratio = 0.333):
- **VINS:** Input voltage monitoring (8.4V max → 2.8V at ADC)
- **VSNA/VSNB:** Motor phase voltage sensing for back-EMF detection
- **VPOS:** Potentiometer position (direct, no divider needed - powered from 3.3V)

### Current Sensing
- 0.1Ω shunt resistor between motor driver GND and PGND
- At 1A: Vshunt = 100mV (ADC reads ~124 counts at 3.3V reference)
- Software overcurrent protection via ADC threshold
- Resolution: ~8mA per ADC count

### Communication
- Half-duplex UART at 2Mbps over single DATA line
- 74LVC1G07 open-drain buffer enables bidirectional communication
- 1kΩ pullup (R1) sized for 2Mbps operation
- ESD protection via PESD5V0L1UL TVS diode

## Assembly

This board is designed for single-sided assembly, making it ideal for:
- Hot plate reflow
- Hot air station
- IR oven
- JLCPCB assembly service

All components are on the top side with no vias in pads.

## Programming

Program using WCH-LinkE debugger/programmer:
- Supports 3.3V target voltage
- Single-wire debug (SWD) interface
- Compatible with MounRiver Studio IDE
- printf debugging available over SWD

## Mechanical Integration

The board is designed to fit inside standard SG90 servo cases:
- Motor wires solder directly to MOT_A/MOT_B pads
- Potentiometer wires connect to J2
- Servo cable connects to J1
- Debug pads accessible through case opening

## Performance Targets

- Position accuracy: ±0.1-0.2°
- Position resolution: 0.044°
- Update rate: 10kHz
- Current limit: 1A (configurable via firmware)
- Continuous operation: 200mA
- Response time: <1ms
- Backdrive detection: Yes (via back-EMF)

## Manufacturing

Optimized for low-cost production:
- Total BOM cost: ~$0.35 at 500 qty
- Single-sided (front) assembly
- Standard 0603 passives (except current shunt)
- All components available from LCSC
- 8-layer PCB for signal integrity and compact routing
- **Cheapest option:** PCB + stencil only, manual reflow assembly

### JLCPCB Pricing Options

**Option 1: PCB + Stencil Only (manual assembly)**
| Item | Cost |
|------|------|
| PCB (5 boards) | $2.00 |
| Stencil (top only) | $10.29 |
| Engrave text | $0.48 |
| Shipping | ~$5.00 |
| **Total** | **~$18** |

*Best for prototyping. Order components separately from LCSC.*

**Option 2: JLCPCB Assembly (5 boards, front side only)**
| Item | Cost |
|------|------|
| PCB Price | $2.00 |
| Setup Fee | $8.00 |
| Stencil | $1.50 |
| Components (12 items) | $3.20 |
| Extended Components Fee | $12.00 |
| SMT Assembly | $0.72 |
| Nitrogen Reflow | $0.88 |
| **Total** | **$28.30** |

*~$5.66/board. Build time: 5-6 days PCB + 1-2 days assembly.*

**Back side (hand solder):** 3 easy components
- 0.1Ω sense resistor (1206)
- AO3401A P-FET (SOT-23-3)
- TPAP2210K-3.3TRG1 LDO (SOT-23-5)

## TODO / Future Improvements

### Critical Values to Verify
- [x] **Voltage dividers (20k/10k):** 8.4V × 0.333 = 2.8V (safe for 3.3V ADC) ✓
- [x] **Current shunt (0.1Ω):** 1A × 0.1Ω = 100mV → ~124 ADC counts ✓
- [x] **DATA pullup (1kΩ):** Suitable for 2Mbps with open-drain buffer ✓
- [x] **RC filter (10kΩ + 100nF):** fc = 160Hz, good for pot noise filtering ✓
- [x] **ESD protection:** PESD5V0L1UL on DATA line ✓
- [x] **LDO thermal:** TPAP2210K handles (8.4V-3.3V) × 50mA = 255mW in SOT-23-5 ✓
- [ ] **Motor pad spacing:** Confirm fits SG90 motor wires
- [ ] **Debug pad pitch:** Verify 1.27mm for pogo pins

### Firmware Features (Planned)
- Backlash compensation
- Temperature compensation
- Position calibration LUT
- Multi-drop addressing
- Synchronized motion commands
- Input voltage monitoring and low-battery warning
- Back-EMF based stall detection

## License

[Add your license here]

## Author

[Add author information here]
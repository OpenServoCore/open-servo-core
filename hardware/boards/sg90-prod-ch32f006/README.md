# SG90 Servo Controller Board - CH32V006

A compact, high-precision servo controller designed to replace the control circuitry in SG90/MG90 servo clones. This single-sided board fits within a 10x12.5mm footprint while providing advanced features like back-EMF sensing, overcurrent protection, and 0.1-0.2° position precision.

## Features

- **MCU**: CH32V006F8U6 RISC-V microcontroller (48MHz, 32KB flash, 4KB RAM)
- **Motor Driver**: Integrated DRV8837C H-bridge (1.8A peak)
- **Position Sensing**: 12-bit ADC with RC filtering for 0.1-0.2° precision
- **Current Sensing**: 0.1Ω shunt with internal op-amp
- **Protection**: P-channel MOSFET reverse polarity protection (17mΩ RDS(on))
- **Communication**: 2Mbps half-duplex UART over single DATA line
- **Power**: 5V operation, no LDO required
- **Size**: 10x12.5mm, single-sided assembly
- **Debug**: 4-pin SWD interface for programming with WCH-LinkE

## Specifications

- Input Voltage: 4.5-5.5V
- Continuous Current: 200mA (typical servo operation)
- Peak Current: 1A (stall condition)
- Position Resolution: 0.044°/count (4096 counts over 180°)
- PWM Frequency: 10kHz center-aligned
- Communication: 2Mbps half-duplex UART
- Temperature Range: -40°C to +85°C

## Bill of Materials (BOM)

| Ref | Description | Package | LCSC # | Qty | Unit Price @5 | Unit Price @150 | Unit Price @500 |
|-----|-------------|---------|--------|-----|---------------|-----------------|-----------------|
| U1 | CH32V006F8U6 MCU | QFN-20 3x3mm | C42431288 | 1 | $0.2643 | $0.1838 | $0.1537 |
| U2 | DRV8837CDSGR Motor Driver | WSON-8 | C191000 | 1 | $0.098 | $0.0666 | $0.0587 |
| U3 | 74LVC1G07DSFR Buffer | TDFN-6 1x1mm | C42445855 | 1 | $0.0466 | $0.0317 | $0.0280 |
| Q1 | BRCS150P02ZJ P-MOSFET | DFN-6 2x2mm | C22448985 | 1 | $0.0747 | $0.0522 | $0.0466 |
| R1 | 10kΩ 1% | 0603 | Generic | 1 | $0.01 | $0.002 | $0.002 |
| R2 | 0.1Ω 1% 1/4W | 0603/0805 | Generic | 1 | $0.02 | $0.003 | $0.003 |
| R3 | 10kΩ 1% | 0603 | Generic | 1 | $0.01 | $0.002 | $0.002 |
| R4,R5 | 10kΩ 1% | 0603 | Generic | 2 | $0.01 | $0.002 | $0.002 |
| R6,R7 | 0Ω | 0603 | Generic | 2 | $0.01 | $0.001 | $0.001 |
| R8 | 0Ω | 0603 | Generic | 1 | $0.01 | $0.001 | $0.001 |
| R9 | 10kΩ 1% | 0603 | Generic | 1 | $0.01 | $0.002 | $0.002 |
| R10 | 1kΩ 1% | 0603 | Generic | 1 | $0.01 | $0.002 | $0.002 |
| C1,C2 | 100nF 16V X7R | 0603 | Generic | 2 | $0.01 | $0.002 | $0.002 |
| C3 | 1µF 16V X7R | 0603 | Generic | 1 | $0.01 | $0.003 | $0.003 |
| C4,C5 | 10µF 10V X5R | 0603/0805 | Generic | 2 | $0.02 | $0.005 | $0.004 |
| C6 | 100nF 16V X7R | 0603 | Generic | 1 | $0.01 | $0.002 | $0.002 |
| **Components Total** | | | | | **$0.664** | **~$0.35** | **$0.326** |
| **PCB** | 10x12.5mm 4-layer | - | - | 1 | **$1.40** | **$0.25** | **$0.11** |
| **TOTAL per board** | | | | | **$2.064** | **$0.60** | **$0.436** |
| **TOTAL** | | | | | **$10.32** | **$90.00** | **$218.00** |

### Cost Breakdown by Quantity

#### Prototype Run (5 boards)
- Components (5 sets): $0.664 × 5 = $3.32
- PCB (5 boards): $2.00 + $5.00 shipping = $7.00
- **Total for 5 boards: $10.32**
- **Cost per board: $2.06**
- **Selling at $3/board: $15 revenue - Profit: $4.68 (45% margin)**
- **Selling at $5/board: $25 revenue - Profit: $14.68 (142% margin)**

#### Small Production (150 boards)
- Components (150 sets): ~$0.35 × 150 = $52.50
- PCB (150 boards): $32.00 + $5.00 shipping = $37.00
- **Total for 150 boards: $89.50**
- **Cost per board: $0.60**
- **Selling at $2-3/board: $300-450 revenue**
- **Gross profit: $210.50-360.50**

#### Medium Production (500 boards)
- Components (500 sets): $0.326 × 500 = $163.00
- PCB (500 boards): $50.20 + $5.00 shipping = $55.20
- **Total for 500 boards: $218.20**
- **Cost per board: $0.44**
- **Selling at $2/board: $1,000 revenue**
- **Gross profit: $781.80 (358% markup!)**

*Prices shown are from LCSC in USD. Passive component prices are estimates for generic parts.*

## Pinout

### Power Connector (J1) - 3 pins
1. V_IN (5V)
2. DATA (Half-duplex UART, 2Mbps)
3. GND

### Motor Connector (J3) - 2 wire pads
1. MOT_A - Motor terminal A (7.4mm spacing)
2. MOT_B - Motor terminal B

### Potentiometer Connector (J2) - 3 pads
1. POT_VCC - Potentiometer power (5V)
2. POT_WIPER - Position feedback
3. POT_GND - Potentiometer ground

### Debug Interface (J4) - 4 pads, 1.27mm pitch
1. 5V - Power from debugger
2. GND - Ground
3. SWDIO - Single-wire debug
4. NC - Not connected

## Design Notes

### Power Architecture
- Single 5V rail operation eliminates need for LDO
- P-channel MOSFET provides reverse polarity protection with only 17mV drop at 1A
- Bulk capacitance (2x10µF) handles motor transients
- No ferrite bead needed due to synchronized ADC sampling

### ADC Precision
- 12-bit ADC provides 22.8 counts per degree (0.044°/count theoretical)
- RC filter (1kΩ + 100nF) on POT input reduces wiper noise
- 4-16x oversampling in firmware improves effective resolution
- Center-aligned PWM with mid-duty ADC sampling avoids switching noise

### Current Sensing
- 0.1Ω shunt resistor in series with motor ground
- Internal op-amp amplifies voltage drop
- ADC watchdog enables hardware overcurrent protection
- Kelvin connections (0Ω resistors R6,R7) for accurate sensing

### Communication
- Half-duplex UART at 2Mbps over single DATA line
- 74LVC1G07 open-drain buffer enables bidirectional communication
- 10kΩ pullup (R1) sized for 2Mbps operation
- Compatible with standard servo protocols

## Assembly

This board is designed for single-sided assembly, making it ideal for:
- Hot plate reflow
- Hot air station
- IR oven
- JLCPCB assembly service

All components are on the top side with no vias in pads.

## Programming

Program using WCH-LinkE debugger/programmer:
- Supports 5V target voltage
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
- Total BOM cost: ~$0.43 at 500 qty
- Single-sided assembly reduces assembly cost
- Standard 0603 passives (except current shunt)
- All components available from LCSC
- 4-layer PCB for better thermal management

## TODO / Future Improvements

### Rev B Changes
1. **Add test points on back side**
   - Add second 1x4 array at 1.27mm pitch (same as debug interface)
   - Uses same pogo pin clip for both programming and testing
   - **Debug array (1x4):** 5V, GND, SWDIO, NC (existing)
   - **Test array (1x4):** SEN_I, SEN_POT, PWM_A, V_MCU
   - Both arrays accessible from back when installed in servo
   - *Rationale: Single pogo clip tool works for both debug and test*

2. **Add ESD protection**
   - TVS diode on DATA line (SOD-523 or 0402 package)
   - Protects against ESD during cable plug/unplug
   - Suggested part: PESD5V0S1BL or similar 5.5V TVS (~$0.05)

3. **Consider fiducial marker**
   - Single 1mm fiducial for JLCPCB assembly alignment
   - Only if assembly house requires it

### Firmware Features (Planned)
- Backlash compensation
- Temperature compensation
- Position calibration LUT
- Multi-drop addressing
- Synchronized motion commands

## License

[Add your license here]

## Author

[Add author information here]
# open-servo-kernel-api

Pure trait-level API boundary for the open-servo kernel. Defines interfaces for nodes, controllers, faults, telemetry, reset, mode switching, and signal flow.

## Sensors

The kernel expects certain sensor inputs from the board. Sensors are categorized as **invariant** (always present) or **optional** (board-dependent).

### Invariant Sensors

These sensors must be provided by every board implementation:

| Sensor | Description |
|--------|-------------|
| **Board/MCU Temperature** | NTC thermistor or MCU's internal temperature sensor. Can be combined with an I²R or IV model to estimate motor temperature when a dedicated motor temperature sensor is absent. |
| **Servo Position** | Absolute position feedback via potentiometer or Hall-effect sensor. |
| **Motor Current** | Phase current measurement via shunt resistor(s) or driver current mirroring. Feature-gated by motor topology (BDC vs BLDC). |
| **MCU Voltage** | MCU internal ADC measuring VDD or VDDA. Used for supply monitoring and ADC reference compensation. |

### Optional Sensors

These sensors may or may not be present depending on board capabilities:

| Sensor | Description |
|--------|-------------|
| **Motor Voltage** | Per-node voltage via resistor divider. BDC: 2 readings (one per terminal). BLDC: TBD (possibly 6 readings for phase voltages). |
| **Motor Temperature** | Dedicated NTC thermistor on motor winding or case. |
| **VSYS/VBAT Voltage** | System or battery supply voltage via resistor divider. |
| **Motor Position** | High-resolution incremental feedback via custom dual optical encoder. |

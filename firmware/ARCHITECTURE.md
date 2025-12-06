# **ARCHITECTURE**

OpenServo-Core Firmware Architecture
**Status:** Draft 1
**Author:** Aaron Q + ChatGPT
**Purpose:** Document the firmware architecture for a portable, deterministic smart-servo controller capable of running on STM32F301 (dev rig) and CH32V006/007 (production MCU).

---

## **1. Design Goals**

* Deterministic, jitter-free control loop (hard real-time)
* Portable across MCU families (PAC-only STM32 + WCH)
* Clean separation of core control logic vs. hardware details
* Support for:

  * cascaded control loops (current → velocity → position)
  * UART servo-bus protocol (Dynamixel-like)
  * debug UART
  * logging / telemetry
* Robust safety and fault-handling
* Support incremental development from V0 → V3

---

## **2. High-Level Architecture**

The firmware is built around **three execution tiers**:

### **Tier 1 — Hard Real-Time (Deterministic)**

* Runs at fixed frequency (e.g. 10 kHz)
* Triggered by ADC-DMA completion or Timer ISR
* Executes:

  * current loop
  * velocity loop
  * position loop
  * PWM update
* Must never block, allocate, or wait
* Not routed through event queue

### **Tier 2 — Critical Safety Path**

* Triggered by hardware comparator / timer break input
* Firmware safety checks (over-current / over-temp)
* Executes immediately and forces safe mode:

  * disables driver
  * sets PWM = 0
  * latches fault state
* Does **not** rely on event queue

### **Tier 3 — Soft Events**

* UART RX (debug + servo bus)
* Logging / telemetry
* Shell commands
* Non-critical tasks
* Routed through a small SPSC event queue

---

## **3. Module Layout**

```
firmware/                    # Workspace root
│
├─ Cargo.toml               # Workspace manifest
│
├─ open-servo-core/         # Core application logic
│   ├─ src/
│   │   ├─ lib.rs          # Module exports
│   │   ├─ app.rs          # App<C: ControlLoop> state machine
│   │   ├─ event.rs        # Event system (queue, types)
│   │   └─ fault.rs        # Fault handling + safety
│   └─ Cargo.toml
│
├─ open-servo-control/      # Control algorithms & sensors
│   ├─ src/
│   │   ├─ lib.rs          # Module exports
│   │   ├─ traits.rs       # Sensor & control traits
│   │   └─ pid.rs          # PID controller implementation
│   └─ Cargo.toml
│
├─ open-servo-hw/           # Hardware abstraction layer
│   ├─ src/
│   │   ├─ lib.rs          # Module exports
│   │   ├─ traits.rs       # HW driver traits
│   │   ├─ types.rs        # Common types (UartPort, etc)
│   │   └─ adc_dma.rs      # ADC/DMA abstractions
│   └─ Cargo.toml
│
├─ open-servo-stm32f301/    # STM32F301 implementation
│   ├─ src/
│   │   ├─ main.rs         # Entry point
│   │   ├─ hw_impl.rs      # Trait implementations
│   │   ├─ board.rs        # Board-specific config
│   │   └─ init/           # Hardware initialization
│   │       ├─ mod.rs
│   │       ├─ rcc.rs      # Clock configuration
│   │       ├─ gpio.rs     # Pin setup
│   │       ├─ tim.rs      # Timer/PWM setup
│   │       └─ adc.rs      # ADC/DMA setup
│   ├─ memory.x            # Memory layout
│   └─ Cargo.toml
│
└─ open-servo-ch32v00x/     # CH32V006/007 (future)
    └─ (similar structure)
```

---

## **4. Hardware Abstraction Layer — Trait Architecture**

The hardware abstraction is split across multiple trait boundaries for better modularity:

### **Hardware Driver Traits** (`open-servo-hw/traits.rs`)

```rust
pub enum UartPort { Bus, Debug }

/// Brushed DC motor driver control (H-bridge)
pub trait BdcMotorDriver {
    fn set_pwm(&mut self, duty: i32);     // PWM duty (-max to +max)
    fn set_enable(&mut self, enabled: bool);
    fn coast(&mut self);                  // High impedance mode
    fn brake(&mut self);                  // Short motor terminals
}

/// UART communication
pub trait UartDriver {
    fn uart_write(&mut self, port: UartPort, data: &[u8]);
    fn uart_read_byte(&mut self, port: UartPort) -> Option<u8>;
}

/// System time
pub trait SystemTime {
    fn now_us(&self) -> u32;
}
```

### **Sensor Traits** (`open-servo-control/traits.rs`)

```rust
pub trait PositionSensor {
    fn read_position(&self) -> u16;      // 0-4095 for 12-bit ADC
}

pub trait CurrentSensor {
    fn read_current(&self) -> u16;       // milliamps
}

pub trait VoltageSensor {
    fn read_voltage(&self) -> u16;       // millivolts
}

pub trait TemperatureSensor {
    fn read_temperature(&self) -> Option<u16>;  // decikelvin
}
```

### **Control Loop Trait** (`open-servo-control/traits.rs`)

```rust
pub trait ControlLoop {
    fn compute(&mut self, setpoint: u16, position: u16, 
               current: Option<u16>) -> i32;
    fn reset(&mut self);
    fn set_setpoint(&mut self, setpoint: u16);
    fn get_setpoint(&self) -> u16;
}
```

MCU-specific crates implement these traits. The **core logic is 100% independent of PAC/HAL code**.

---

## **5. `App` Structure (Core Brain)**

`App` is generic over the control algorithm:

```rust
pub struct App<C: ControlLoop> {
    controller: C,              // Control algorithm (PID, cascade, etc)
    fault_state: FaultState,    // Safety state machine
    system_state: SystemState,  // Telemetry snapshot
}

pub struct SystemState {
    pub setpoint: u16,
    pub position: u16,
    pub pwm_duty: i32,
    pub current_ma: u16,
    pub bus_voltage_mv: u16,
    pub temperature_dk: Option<u16>,
}
```

### Core entry points:

```rust
impl<C: ControlLoop> App<C> {
    // Hard real-time control tick (from ISR)
    pub fn on_control_tick<H>(&mut self, hw: &mut H)
    where H: BdcMotorDriver + PositionSensor + CurrentSensor 
           + VoltageSensor + TemperatureSensor;
    
    // Soft event handling (from main loop)
    pub fn handle_event<H: BdcMotorDriver>(&mut self, hw: &mut H, ev: Event);
    
    // Safety management
    pub fn raise_fault<H: BdcMotorDriver>(&mut self, hw: &mut H, kind: FaultKind);
    pub fn clear_fault<H: BdcMotorDriver>(&mut self, hw: &mut H);
}
```

Future additions:
* `BusProtoState` (servo network protocol) - V1
* `DebugShell` - V1
* Telemetry buffer - V1

---

## **6. Event System**

Soft events are routed through a small lock-free SPSC queue (e.g. `heapless::spsc::Queue`).

### Event enum:

```rust
pub enum Event {
    UartRx { port: UartPort, byte: u8 },
    SlowTick,
    Fault(FaultKind), // monitoring/logging only
}
```

### ISR → Event Queue

* UART1 ISR → enqueue `Event::UartRx { port: Bus }`
* UART2 ISR → enqueue `Event::UartRx { port: Debug }`
* Slow timer ISR (100 Hz) → enqueue `Event::SlowTick`

### Main Loop (Event Reactor)

In main:

```rust
loop {
    while let Some(ev) = EVENT_Q.dequeue() {
        app.handle_event(&mut hw, ev);
    }
    cortex_m::asm::wfi();
}
```

This keeps “slow” and “parsey” tasks off ISRs.

---

## **7. Safety & Fault System**

Faults must be handled *outside* the event system:

### Fault Types

```rust
pub enum FaultKind {
    OverCurrent,
    OverTemp,
    UnderVoltage,
    QueuePressure,
    EncoderFault,
}
```

### Fault State

```rust
pub enum FaultState {
    Ok,
    Latched(FaultKind),
}
```

### Fault Handling Rules

* Faults **immediately disable the motor**:

  * `set_enable(false)`
  * `set_pwm(0)`
* `Servo::on_control_tick()` exits early if a fault is latched.
* Faults require manual clearing via:

  * Debug shell command
  * Servo bus command

### Hardware First

Comparator / timer break input → hardware shuts down PWM instantly.

### Firmware Second

Break ISR → `app.raise_fault(FaultKind::OverCurrent)`.

This decouples **safety** from **logging** and from the event queue.

---

## **8. Control Loop Timing**

### Hard Real-Time Control Loop (Tier 1)

Called from ADC-DMA completion or Timer interrupt:

```rust
pub fn on_control_tick<H>(&mut self, hw: &mut H)
where
    H: BdcMotorDriver + PositionSensor + CurrentSensor 
     + VoltageSensor + TemperatureSensor
{
    // Safety-first: exit if faulted
    if self.fault_state.is_faulted() {
        self.fault_state.apply_safety(hw);
        return;
    }

    // Read sensors
    let current = hw.read_current();      // mA
    let position = hw.read_position();    // 0-4095
    let bus_voltage = hw.read_voltage();  // mV
    let temperature = hw.read_temperature(); // dK

    // Run control algorithm
    let pwm_command = self.controller.compute(
        self.controller.get_setpoint(), 
        position, 
        Some(current)
    );

    // Apply actuation
    hw.set_pwm(pwm_command);

    // Update telemetry
    self.system_state.position = position;
    self.system_state.pwm_duty = pwm_command;
    // ... etc
}
```

* Must never block
* Must never wait on a queue
* Must avoid heap allocation
* Must be predictable in execution time

---

## **9. UART Architecture (Debug + Servo Bus)**

Two independent UART contexts:

### **UART A — Servo Bus (Dynamixel-like)**

* For external host/controller → servo communication
* Tight state machine: command → action → status reply
* Bridges to servo control parameters, telemetry, configuration

### **UART B — Debug Shell**

* For development and tuning
* Not time-sensitive
* Can emit logs
* Accepts line-oriented debug commands

Each UART ISR simply enqueues bytes into an event queue.

---

## **10. MCU Portability Strategy**

Because all PAC code is fenced inside the `Hw` implementation and ISR wiring:

### To port to another MCU (e.g., CH32V007):

1. Create new workspace member `open-servo-ch32v007/`
2. Implement hardware traits:
   * `impl BdcMotorDriver for Ch32Hw`
   * `impl UartDriver for Ch32Hw`
   * `impl SystemTime for Ch32Hw`
   * `impl PositionSensor for Ch32Hw`
   * `impl CurrentSensor for Ch32Hw`
   * etc.
3. Provide:

   * timers
   * ADC + DMA
   * comparator / overcurrent pin
   * UART A + UART B
4. Map interrupts:

   * control tick ISR → `app.on_control_tick()`
   * UART ISRs → queue events
   * safety ISR → `app.raise_fault()`

No changes to `core` crate required.

---

## **11. Version Roadmap**

### **V0 — Bring-up** *(Current Implementation)*

* ✅ Workspace-based Rust architecture
* ✅ Hardware abstraction via traits
* ✅ Basic PID control loop
* ✅ PWM control + sensor reading
* ✅ Fault handling system
* ✅ Event-driven architecture
* 🚧 Debug UART (partial)
* Platform: STM32F301 dev board

### **V1 — Servo Controller**

* Full cascaded control loop
* Servo bus protocol
* Fault system
* Logging / telemetry
* Target MCU: **CH32V006 or CH32V007**

### **V2 — High Performance**

* Observers (friction, backlash, disturbance)
* Kalman filters or simplified observers
* Motion planner integration
* Target MCU: **CH32V007** (better analog front-end)

### **V3 — Advanced**

* Higher-loop bandwidth
* BLDC variant
* CAN or RS485 bus
* Target MCU: **CH32V208/307 (V2C/V3B cores)**

---

## **12. Summary**

This architecture ensures:

* **Hard real-time correctness**
* **Safety independence**
* **Scalable complexity**
* **MCU portability**
* **Clean dual-UART design**
* **PAC-only deterministic control**
* **Minimal ISR work**

The structure balances simplicity (good for hobby servos) and headroom for advanced versions (observer-based, Dynamixel-class servos).

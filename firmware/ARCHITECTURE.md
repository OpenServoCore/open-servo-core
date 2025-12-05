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
firmware/
│
├─ core/               # MCU-agnostic servo core
│   ├─ servo.rs        # control loops (PID/cascade)
│   ├─ protocol_bus.rs # dynamixel-like bus protocol
│   ├─ protocol_dbg.rs # debug shell
│   ├─ fault.rs        # fault model + safe mode
│   ├─ hw_trait.rs     # Hw trait (abstraction layer)
│   ├─ app.rs          # App { servo, proto, dbg, fault }
│   └─ telemetry.rs
│
├─ hw_stm32/           # STM32F301 backend
│   ├─ init/*.rs       # clocks, timers, adc, dma, uart
│   ├─ isr.rs          # ISR wiring
│   └─ hw_impl.rs      # impl Hw for StmHw
│
├─ hw_ch32v006/        # CH32V006 or 007 backend
│   ├─ init/*.rs
│   ├─ isr.rs
│   └─ hw_impl.rs
│
└─ main.rs             # entry point
```

---

## **4. `Hw` Trait — Hardware Abstraction Layer**

The `Hw` trait defines all MCU-specific interactions:

```rust
pub enum UartPort { Bus, Debug }

pub trait Hw {
    // Sensors (scaled into physical units)
    fn phase_current(&self) -> f32;
    fn position(&self) -> f32;
    fn bus_voltage(&self) -> f32;

    // Actuators
    fn set_pwm(&mut self, duty: f32);
    fn set_enable(&mut self, en: bool);

    // Time
    fn now_us(&self) -> u32;

    // Multi-UART
    fn uart_write(&mut self, port: UartPort, buf: &[u8]);
    fn uart_read_byte(&mut self, port: UartPort) -> Option<u8>;
}
```

MCU-specific crates (STM32 / CH32) implement this trait.

The **core logic is 100% independent of PAC/HAL code**.

---

## **5. `App` Structure (Core Brain)**

`App` owns:

* `Servo` (control loops)
* `BusProtoState` (servo network protocol)
* `DebugShell`
* `FaultState`
* telemetry buffer (optional)

### Core entry points:

```rust
impl<Cfg> App<Cfg> {
    pub fn on_control_tick<H: Hw>(&mut self, hw: &mut H);
    pub fn handle_event<H: Hw>(&mut self, hw: &mut H, ev: Event);
    pub fn raise_fault<H: Hw>(&mut self, hw: &mut H, kind: FaultKind);
    pub fn clear_fault<H: Hw>(&mut self, hw: &mut H);
}
```

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

Called from DMA or Timer interrupt:

```rust
pub fn on_control_tick<H: Hw>(&mut self, hw: &mut H) {
    if fault latched → disable motor + return;

    let i   = hw.phase_current();
    let pos = hw.position();
    let v   = hw.bus_voltage();

    let cmd = self.servo.step(i, pos, v);
    hw.set_pwm(cmd);
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

1. Implement `struct Ch32Hw`.
2. Implement `impl Hw for Ch32Hw`.
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

### **V0 — Bring-up**

* Minimal firmware
* PWM control + read sensors + debug UART
* Uses STM32F301 for early development
* CH32V003 optional for experiments

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

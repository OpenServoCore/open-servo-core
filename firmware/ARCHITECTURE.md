# **ARCHITECTURE**

OpenServo-Core Firmware Architecture
**Status:** Draft 5
**Purpose:** Document the firmware architecture for a portable, deterministic smart-servo controller capable of running on STM32F301 (dev rig) and CH32V006/007 (production MCU).

---

## **1. Design Goals**

- Deterministic, jitter-free control loop (hard real-time)
- Portable across MCU families (PAC-only STM32 + WCH)
- Clean separation of core control logic vs. hardware details
- Support for:
  - cascaded control loops (current → velocity → position)
  - UART servo-bus protocol (Dynamixel-like)
  - debug **channel** (UART or RTT, depending on board)
  - logging / telemetry
- Robust safety and fault-handling
- Support incremental development from V0 → V3
- On dev boards with only one UART (e.g. STM32F301 rig), keep that UART free for the servo bus and use **RTT over SWD** for debug REPL + logging.

---

## **2. High-Level Architecture**

The firmware is built around **three execution tiers**:

### **Tier 1 — Hard Real-Time (Deterministic)**

- Runs at fixed frequency (e.g. 10 kHz)
- Triggered by ADC-DMA completion or Timer ISR
- Executes:
  - current loop
  - velocity loop
  - position loop
  - PWM update
- Must never block, allocate, or wait
- Not routed through event queue

### **Tier 2 — Critical Safety Path**

- Triggered by hardware comparator / timer break input
- Firmware safety checks (over-current / over-temp)
- Executes immediately and forces safe mode:
  - disables driver
  - sets PWM = 0
  - latches fault state
- Does **not** rely on event queue

### **Tier 3 — Soft Events & Debug Shell**

- UART RX (servo bus)
- Logging / telemetry
- Debug shell / REPL (via **generic debug I/O**):
  - RTT-based REPL on boards with SWD/RTT (e.g. STM32F301).
  - UART-based REPL on boards with a spare UART (e.g. CH32V006/007).
- Non-critical tasks and configuration
- Routed through:
  - A small SPSC event queue (for UART + system events), **and**
  - A polled debug shell that consumes bytes from a `DebugIo` abstraction.

Tier 1 and Tier 2 **never** depend on RTT, defmt, or debug facilities.

---

## **3. Module Layout**

```txt
firmware/                    # Workspace root
│
├─ Cargo.toml               # Workspace manifest
│
├─ open-servo-core/         # Core application logic
│   ├─ src/
│   │   ├─ lib.rs           # Module exports
│   │   ├─ app.rs           # App<C: ControlLoop> state machine
│   │   ├─ event.rs         # Event system (queue, types)
│   │   ├─ fault.rs         # Fault handling + safety
│   │   ├─ debug_shell.rs   # Debug shell / REPL (generic over DebugIo)
│   │   └─ log.rs           # Optional defmt-based logging helpers
│   └─ Cargo.toml
│
├─ open-servo-control/      # Control algorithms & sensors
│   ├─ src/
│   │   ├─ lib.rs           # Module exports
│   │   ├─ traits.rs        # Sensor & control traits
│   │   └─ pid.rs           # PID controller implementation
│   └─ Cargo.toml
│
├─ open-servo-hw/           # Hardware abstraction layer
│   ├─ src/
│   │   ├─ lib.rs           # Module exports
│   │   ├─ traits.rs        # HW driver traits
│   │   ├─ types.rs         # Common types (UartPort, etc)
│   │   ├─ debug_io.rs      # Debug I/O abstraction (RTT / UART / USB-CDC)
│   │   └─ adc_dma.rs       # ADC/DMA abstractions
│   └─ Cargo.toml
│
├─ open-servo-stm32f301/    # STM32F301 implementation
│   ├─ src/
│   │   ├─ main.rs          # Entry point
│   │   ├─ hw_impl.rs       # Trait implementations
│   │   ├─ board.rs         # Board-specific config
│   │   ├─ debug_rtt.rs     # RTT-based DebugIo implementation
│   │   └─ init/            # Hardware initialization
│   │       ├─ mod.rs
│   │       ├─ rcc.rs       # Clock configuration
│   │       ├─ gpio.rs      # Pin setup
│   │       ├─ tim.rs       # Timer/PWM setup
│   │       └─ adc.rs       # ADC/DMA setup
│   ├─ memory.x             # Memory layout
│   └─ Cargo.toml
│
└─ open-servo-ch32v00x/     # CH32V006/007 (future)
    ├─ src/
    │   ├─ main.rs
    │   ├─ hw_impl.rs
    │   ├─ board.rs
    │   ├─ debug_uart.rs    # (Optionally) UART-based DebugIo implementation
    │   └─ init/
    └─ Cargo.toml
```

On STM32F301, the **single hardware UART** is reserved for the servo bus; debug traffic goes over RTT/SWD via `debug_rtt.rs`.

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

### **Debug I/O Trait** (`open-servo-hw/debug_io.rs`)

`DebugIo` abstracts “a debug byte stream” (REPL + logs) so the core doesn’t care whether it’s RTT, UART, or USB-CDC:

```rust
/// Non-blocking byte-oriented debug channel.
/// Implemented by board crates using RTT, UART, USB-CDC, etc.
pub trait DebugIo {
    /// Try to read a single byte; returns None if no data is available.
    fn try_read(&mut self) -> Option<u8>;

    /// Best-effort write. Should not block or allocate.
    fn write(&mut self, data: &[u8]);

    /// Optionally flush buffered data (can be a no-op).
    fn flush(&mut self) {}
}
```

Examples:

- **STM32F301 dev board**: `RttDebugIo` (backed by `rtt-target` channels).
- **CH32V006/007**: `UartDebugIo` (backed by a spare UART), or also `RttDebugIo` if code size allows.

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

MCU-specific crates implement these traits. The **core logic is 100% independent of PAC/HAL code and debug backend**.

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

- `BusProtoState` (servo network protocol) - V1
- `DebugShell` (RTT/UART REPL) - V1
- Telemetry buffer - V1

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

Debug REPL input is **not** queued as `Event`; instead, it is handled via the `DebugShell` using the `DebugIo` abstraction:

- For UART-based debug: the UART ISR may still enqueue `Event::UartRx { port: Debug, byte }`, and the board-specific code can feed those bytes into its `DebugIo` implementation.
- For RTT-based debug: there is no ISR; the `DebugShell` polls `DebugIo::try_read()` in a **bounded loop** in the main thread.

### ISR → Event Queue

- UART (servo bus) ISR → enqueue `Event::UartRx { port: Bus }`
- (Optional) UART debug ISR → enqueue `Event::UartRx { port: Debug }`
- Slow timer ISR (100 Hz) → enqueue `Event::SlowTick`

### Main Loop (Event Reactor + Debug Shell)

In main:

```rust
loop {
    // 1. Drain event queue
    while let Some(ev) = EVENT_Q.dequeue() {
        app.handle_event(&mut hw, ev);
    }

    // 2. Pump debug shell (RTT or UART-backed)
    debug_shell.poll(&mut app, &mut hw);

    // 3. Sleep until next interrupt
    cortex_m::asm::wfi();
}
```

This keeps “slow” tasks (parsing, logging, REPL) off ISRs and decouples debug backend from the core, while the shell’s internal loop bounds the amount of work per iteration.

---

## **7. Safety & Fault System**

Faults must be handled _outside_ the event system:

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

- Faults **immediately disable the motor**:
  - `set_enable(false)`
  - `set_pwm(0)`
- `App::on_control_tick()` exits early if a fault is latched.
- Faults require manual clearing via:
  - Debug shell command (RTT or UART REPL)
  - Servo bus command

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

- Must never block
- Must never wait on a queue
- Must avoid heap allocation
- Must be predictable in execution time
- **Must never call RTT, defmt, or DebugShell directly**

---

## **9. Servo Bus & Debug Channel Architecture**

At the logical level we have two communication channels:

### **Channel A — Servo Bus (Dynamixel-like)**

- For external host/controller → servo communication
- Tight state machine: command → action → status reply
- Bridges to servo control parameters, telemetry, configuration
- On STM32F301, this uses the **single hardware UART**.
- **High-rate binary streaming** (system-ID waveforms, dense sensor logs, continuous kSPS data) lives on this channel via extended bus instructions. The debug channel is not used for sustained high-rate streams.

### **Channel B — Debug Channel (RTT or UART)**

- For development and tuning
- Not time-sensitive
- Accepts line-oriented debug commands
- Emits logs and structured text/binary snapshots
- Implemented via `DebugIo`:
  - **STM32F301 dev board**: `RttDebugIo` using `rtt-target` over SWD.
  - **CH32V006/007**: initially `UartDebugIo` using a spare UART; can later also support `RttDebugIo` if code size allows.

On STM32F301, the **RTT layout** in the control block is:

- **Up channel 0** – defmt log frames (binary), used exclusively by `defmt`:
  - `rtt_init_defmt!()` or `set_defmt_channel()` points defmt at this channel.
  - Host tools (e.g. `defmt-print`, `cargo-embed`, `probe-rs` plugins) attach here and expect only defmt-encoded data.
- **Up channel 1** – text REPL output:
  - Used by `RttDebugIo` / `DebugShell` for human-readable debug text.
- **Up channel 2** – optional **low-rate** binary diagnostics (e.g. one-shot snapshots).
- **Down channel 1** – REPL input:
  - Paired with up channel 1 so RTT frontends (e.g. cargo-embed, VS Code) can present a “terminal with input” for the REPL.

RTT channel buffer modes for debug output:

- RTT `Up` channels used by debug (`up1` REPL, `up2` diag) should be configured as **non-blocking**, e.g. `ChannelMode::NoBlockSkip`.
- The firmware must never block on debug output; if the host is not consuming logs fast enough, messages may be dropped and that is acceptable for debug-only data.
- The defmt channel (up0) can use the mode recommended by `rtt-target` defaults for defmt; from the firmware’s perspective it is still treated as **debug-only** and never used from Tier 1/2.

The **core** only sees a `DebugShell<impl DebugIo>` and does not depend on physical transport, RTT layout, or defmt specifics.

---

## **10. MCU Portability Strategy**

Because all PAC code and debug transport are fenced inside the `Hw` + `DebugIo` implementations and ISR wiring:

### To port to another MCU (e.g., CH32V007):

1. Create new workspace member `open-servo-ch32v007/`.
2. Implement hardware traits:
   - `impl BdcMotorDriver for Ch32Hw`
   - `impl UartDriver for Ch32Hw`
   - `impl SystemTime for Ch32Hw`
   - `impl PositionSensor for Ch32Hw`
   - `impl CurrentSensor for Ch32Hw`
   - etc.
3. Implement `DebugIo`:
   - `impl DebugIo for UartDebugIo` (UART-based REPL), or
   - `impl DebugIo for RttDebugIo` (RTT-based REPL) when RTT is available and affordable.
4. Provide:
   - timers
   - ADC + DMA
   - comparator / overcurrent pin
   - UART A for servo bus
   - Optional UART B for debug (if not using RTT)
5. Map interrupts:
   - control tick ISR → `app.on_control_tick()`
   - UART ISR(s) → queue events and/or feed `DebugIo`
   - safety ISR → `app.raise_fault()`

No changes to `open-servo-core` are required.

---

## **11. Version Roadmap**

### **V0 — Bring-up** _(Current Implementation)_

- ✅ Workspace-based Rust architecture
- ✅ Hardware abstraction via traits
- ✅ Basic PID control loop
- ✅ PWM control + sensor reading
- ✅ Fault handling system
- ✅ Event-driven architecture
- 🚧 RTT debug REPL skeleton (STM32F301 via `RttDebugIo`)
- 🚧 defmt logging over RTT (STM32F301)
- Platform: STM32F301 dev board

### **V1 — Servo Controller**

- Full cascaded control loop
- Servo bus protocol
- Fault system
- Logging / telemetry (defmt-based where enabled)
- DebugShell with a useful command set:
  - view state
  - clear faults
  - adjust setpoints / gains
  - trigger simple system-ID routines
- Initial definition of **system-ID / streaming instructions** on the servo bus (Channel A).
- Target MCU: **CH32V006 or CH32V007**

### **V2 — High Performance**

- Observers (friction, backlash, disturbance)
- Kalman filters or simplified observers
- Motion planner integration
- Optional binary debug snapshots via RTT up-channel 2
- Target MCU: **CH32V007** (better analog front-end)

### **V3 — Advanced**

- Higher-loop bandwidth
- BLDC variant
- CAN or RS485 bus
- Target MCU: **CH32V208/307 (V2C/V3B cores)**

---

## **12. Logging & defmt Integration**

Logging is built around [`defmt`](https://github.com/knurling-rs/defmt) to keep **target-side code size small** while still providing leveled logging (`debug!`, `info!`, `warn!`, `error!`).

### **12.1 Core Logging Model**

- All code that needs logs uses `defmt` macros:
  - `defmt::debug!(...)`
  - `defmt::info!(...)`
  - `defmt::warn!(...)`
  - `defmt::error!(...)`
- `defmt` encodes log frames into a compact binary format:
  - Format strings and metadata live on the **host**, not in MCU flash.
  - This keeps firmware size down compared to `core::fmt`-based logging.
- The transport (RTT, UART, etc.) is provided by a **global logger** implementation and is MCU-specific.

Logging is used only from **Tier 3** (main loop / soft-path code). Tier 1 and Tier 2 never call `defmt` or touch logging to preserve real-time guarantees.

### **12.2 STM32F301 (Dev Rig) — defmt over RTT**

On the STM32F301 dev board, logging uses `defmt` + `rtt-target`:

- Dependencies (example):

  ```toml
  [dependencies]
  defmt = "0.3"
  rtt-target = { version = "0.6", features = ["defmt"] }
  panic-rtt-target = { version = "0.2", features = ["defmt"] } # optional
  ```

- In the F301 crate:

  ```rust
  use defmt::*;
  use panic_rtt_target as _;
  use rtt_target::rtt_init_defmt;

  #[entry]
  fn main() -> ! {
      // Sets up RTT and registers defmt's global logger on an RTT up-channel.
      rtt_init_defmt!();

      info!("boot ok, sp={} pos={}", 0u16, 0u16);

      loop {
          // ...
      }
  }
  ```

- `rtt_init_defmt!()` configures an RTT up channel (up0 in the suggested layout) as the **defmt channel**:
  - Host tools (e.g. `defmt-print`, `cargo-embed`, `probe-rs` integrations) attach to this channel and expect only defmt-encoded frames.
  - REPL and other debug traffic use separate channels (up1, up2) and do not mix with defmt frames.

### **12.3 CH32V003/006/007 — defmt with Custom Transport**

On CH32V00x devices, `defmt` remains **architecture-agnostic**; only the transport changes.

Two primary options:

1. **defmt over UART** (simplest and most portable):
   - Implement a small UART TX buffer + interrupt-driven sender.
   - Provide a `defmt::Logger` implementation that writes encoded frames into this buffer:

     ```rust
     #[defmt::global_logger]
     struct V003Logger;

     unsafe impl defmt::Logger for V003Logger {
         fn acquire() {
             // optional: mask interrupts if needed
         }

         unsafe fn release() {
             // optional: unmask interrupts
         }

         unsafe fn write(bytes: &[u8]) {
             // enqueue bytes into a debug UART TX buffer (Tier 3)
         }
     }
     ```

   - On the host, `defmt-print` (or similar tooling) consumes the UART stream and pretty-prints logs.

2. **defmt over RTT-like transport** (advanced / optional):
   - If `rtt-target` or a similar RTT implementation is proven to work reliably on CH32V00x:
     - Initialize RTT channels in board-specific code.
     - Implement `defmt::Logger` that writes frames into an RTT **up** channel (e.g., up0), mirroring the STM32F301 pattern.

In either case, the **core servo logic does not know or care** whether logs go over UART, RTT, or another backend — it just uses `defmt::info!` and friends.

### **12.4 Feature Gating and Code Size**

Because CH32V003 in particular has tight flash constraints:

- `defmt` is guarded by a feature flag in MCU crates, e.g.:

  ```toml
  [features]
  debug_defmt = []
  ```

- Dev builds on CH32 can enable logging:

  ```bash
  cargo build --features debug_defmt
  ```

- Production firmware can disable the feature, removing `defmt` logging entirely.
- On STM32F301 dev rig, `defmt` + RTT is expected to be enabled in development builds, and may be left on in production if flash allows.

Logging remains a **soft dev tool**:

- Used heavily during bring-up, tuning, and debugging.
- Optional (and removable) for production images on resource-constrained MCUs.

---

## **13. DebugShell & RTT Integration (V0 Detail)**

For V0, the `DebugShell` is a small, **line-oriented** REPL implemented in `open-servo-core`:

- Generic over `DebugIo` so it works on:
  - STM32F301 with `RttDebugIo`.
  - CH32V00x with `UartDebugIo` later.
- Lives purely in **Tier 3** (main loop).
- Maintains a small, fixed-capacity line buffer (e.g. `heapless::String<128>`):
  - Bytes from `DebugIo::try_read()` are appended to this buffer.
  - On receiving `
`, the buffer is treated as a complete command line and passed to the parser.
  - This bounds memory usage, keeps parsing simple, and avoids allocations.
- To bound the work per main-loop iteration, `DebugShell::poll` reads at most `MAX_BYTES_PER_POLL` bytes per call (e.g. 32):
  - Internally, it uses a simple bounded loop:

    ```rust
    const MAX_BYTES_PER_POLL: usize = 32;

    pub fn poll<C, H>(&mut self, app: &mut App<C>, hw: &mut H)
    where
        C: ControlLoop,
        H: BdcMotorDriver,
    {
        for _ in 0..MAX_BYTES_PER_POLL {
            let Some(b) = self.io.try_read() else { break };

            if self.push_byte(b) {
                let line = self.take_line();
                self.handle_line(app, hw, &line);
            }
        }
    }
    ```

  - This prevents a burst of incoming data from starving other Tier 3 work while still draining the debug input quickly under normal use.

- Common initial commands:
  - `help`
  - `state`
  - `fault clear`
  - `set sp <u16>`

Boards wire it up like:

```rust
let controller = /* ControlLoop */;
let mut app = App::new(controller);

// On STM32F301 dev board (when built with `--features debug_rtt`):
let rtt_io = RttDebugIo::new();
let mut debug_shell = DebugShell::new(rtt_io);

loop {
    while let Some(ev) = EVENT_Q.dequeue() {
        app.handle_event(&mut hw, ev);
    }
    debug_shell.poll(&mut app, &mut hw);
    cortex_m::asm::wfi();
}
```

RTT is **feature-gated** (e.g. `debug_rtt` feature) so production builds can exclude it if necessary, or swap to a different `DebugIo` backend.

---

## **14. Summary**

This architecture ensures:

- **Hard real-time correctness** (Tier 1 isolated from logging and debug)
- **Safety independence** (Tier 2 cannot be blocked by debug / REPL)
- **Scalable complexity** (DebugShell and DebugIo are optional but composable)
- **MCU portability** (core unaware of PAC, UART vs RTT, or logging backend)
- **Clean split between servo bus and debug channel**
- **High-rate / system-ID data** goes through the servo bus; RTT is reserved for low-rate diagnostics and dev-only snapshots.
- **PAC-only deterministic control** with **RTT-based REPL on dev boards** where UARTs are scarce
- **defmt-based logging** provides leveled logs with small target-side code size, and can be feature-gated off on tiny MCUs.
- **Minimal ISR work**, with all parsing, logging, and RTT interaction done in the main loop, and bounded per-iteration work in the debug shell.

The RTT-based debug REPL plus defmt logging act as a "soft dev console" that can be expanded over time (more commands, binary snapshots) without touching the hard real-time control loop or safety-critical paths.

---

## **15. Capability Features & Compile-Time Enforcement**

Hardware capabilities are expressed as Cargo features that propagate from board crates through the core logic. This enables:

- **Compile-time guarantees**: Control loops requiring specific sensors fail to compile if the sensor feature is missing
- **Code size optimization**: Unused capability code is excluded from the binary
- **Clean call sites**: No `#[cfg]` scattered throughout business logic

### **15.1 Feature Propagation Chain**

Features flow from board crates (which know what hardware exists) down to logic crates:

```
open-servo-stm32f301               open-servo-core                  open-servo-control
┌────────────────────┐             ┌────────────────────┐           ┌────────────────────┐
│ current-sense-bus ─┼────────────→│ current-sense-bus ─┼──────────→│ current-sense-bus  │
│ debug-shell ───────┼────────────→│ debug-shell        │           │                    │
└────────────────────┘             └────────────────────┘           └────────────────────┘
```

In `Cargo.toml`:

```toml
# Board crate (open-servo-stm32f301)
[features]
current-sense-bus = ["open-servo-core/current-sense-bus"]
debug-shell = ["open-servo-core/debug-shell"]

# Core crate (open-servo-core)
[features]
current-sense-bus = ["open-servo-control/current-sense-bus"]
debug-shell = []

# Control crate (open-servo-control)
[features]
current-sense-bus = []               # Bus current (BDC)
cascade = ["current-sense-bus"]      # Requires bus current sensing
foc = ["open-servo-hw/motor-bldc", "open-servo-hw/current-sense-phase"]  # BLDC path
```

### **15.2 Required vs Optional Features**

**Required capabilities** are assumed to always exist—code compiles unconditionally:

| Capability | Description | Notes |
|------------|-------------|-------|
| Position sensor | Servo output angle feedback | Required for any servo. Board implements `PositionSensor` trait. |
| `pid` | Basic position PID | Default control loop, always compiled. |

The specific position sensor technology (potentiometer, magnetic encoder, etc.) is a **board-level implementation detail**. Core only sees `PositionSensor::read_position()`.

**Optional capabilities** may or may not exist on a given board:

| Feature | Description | When Missing |
|---------|-------------|--------------|
| `current-sense-bus` | Bus current measurement (BDC) | Overcurrent protection disabled, cascade unavailable |
| `debug-shell` | RTT/UART debug REPL | No interactive debugging (~12KB flash saved) |
| `cascade` | Cascaded control loops | Must use position-only PID |
| `motor-encoder` | Motor-side encoder | No backlash compensation (future feature) |

**BLDC/FOC features** (future - different hardware stack):

| Feature | Description | Required Hardware |
|---------|-------------|-------------------|
| `motor-bldc` | 3-phase BLDC driver traits | 3-phase inverter, hall sensors or encoder |
| `current-sense-phase` | Per-phase current measurement | 2-3 current shunts |
| `foc` | Field-oriented control | Both `motor-bldc` + `current-sense-phase` |

The key distinction:
- **Required**: Code assumes it exists. No `#[cfg]` gates. Board must implement the trait.
- **Optional**: Code must handle absence gracefully via stub pattern or `#[cfg]` gates.
- **Motor-specific**: BDC and BLDC are separate hardware stacks with different traits.

### **15.3 Always-Callable Stub Pattern**

Safety and input methods always exist but become no-ops when their capability is missing. This keeps call sites clean:

```rust
// SafetyManager - method always exists
pub fn check_current(&self, current: Option<MilliAmp>) -> Option<FaultKind> {
    #[cfg(feature = "current-sense-bus")]
    {
        current.and_then(|c| {
            if c.abs() > self.thresholds.current_limit {
                Some(FaultKind::OverCurrent)
            } else {
                None
            }
        })
    }
    #[cfg(not(feature = "current-sense-bus"))]
    {
        let _ = current;
        None  // No-op when current sensing unavailable
    }
}

// FastInputs - accessor always exists
impl FastInputs {
    pub fn current(&self) -> Option<MilliAmp> {
        #[cfg(feature = "current-sense-bus")]
        { self.current }
        #[cfg(not(feature = "current-sense-bus"))]
        { None }
    }
}
```

Call sites are unconditional:

```rust
// No #[cfg] needed - check_current is always callable
if let Some(fault) = self.safety.check_current(inputs.current()) {
    self.raise_fault(fault);
    return FastOutputs::fault(fault);
}
```

### **15.4 Compile-Time Enforcement**

Control loops that require specific sensors declare feature dependencies:

```toml
# In open-servo-control/Cargo.toml
cascade = ["current-sense-bus"]  # Cascade control needs bus current feedback
```

Attempting to enable `cascade` without `current-sense-bus` produces a compile error:

```
error: feature `cascade` requires feature `current-sense-bus`
```

This prevents configuration mistakes that would result in runtime failures.

### **15.5 Debug Shell Command Visibility**

Commands for unavailable capabilities are completely hidden—not shown in help, not parsed:

```rust
pub enum LimitCmd {
    #[cfg(feature = "current-sense-bus")]
    Current(Option<i16>),  // Only exists when bus current sensing available
    Temp(Option<i16>),
    // ...
}

fn cmd_help(&mut self) {
    self.println("commands:");
    #[cfg(feature = "current-sense-bus")]
    self.println("  limit current [mA]  - get/set current limit");
    self.println("  limit temp [dC]     - get/set temp limit");
    // ...
}
```

This provides a clean UX: users only see commands that actually work on their hardware.

### **15.6 Adding New Capabilities**

To add a new capability (e.g., `encoder`):

1. **Define feature in control crate** with any dependencies:
   ```toml
   # open-servo-control/Cargo.toml
   encoder = []
   velocity-pid = ["encoder"]  # Velocity control needs encoder
   ```

2. **Propagate through core**:
   ```toml
   # open-servo-core/Cargo.toml
   encoder = ["open-servo-control/encoder"]
   ```

3. **Enable in board crate**:
   ```toml
   # open-servo-stm32f301/Cargo.toml
   encoder = ["open-servo-core/encoder"]
   ```

4. **Gate related code** using the stub pattern (methods always exist) or `#[cfg]` (types/fields).

5. **Hide debug commands** for the capability when feature is disabled.

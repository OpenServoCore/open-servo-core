# Kernel Refactor: Node/Graph/Wired Patterns

## Goal

Refactor `open-servo-kernel` to use the `Node<M>`, `Graph<M, S>`, `Wired<M, S>` patterns from `kernel-api`, enabling future feature migration from `open-servo-core` as composable nodes.

## Key Design Decisions

1. **Clean crate separation** - firmware = pure hardware, runtime = application framework, kernel = monolithic control logic
2. **Policies are nodes** - kernel-api uses Role system (Limiter, Monitor, etc.) instead of separate policy abstraction
3. **Two state layers** - node-internal (algorithm state) vs kernel shared (wire bus)
4. **Feature-based modules** - organize by feature (gate/, position/, etc.) not flat nodes/
5. **Controller trait** - wrap PidControllerI16 to formalize intent

---

## Implementation Steps

### Step 1: Clean Firmware/Runtime/Kernel Separation

**Goal:** Establish clean boundaries where firmware is pure hardware, runtime is the application framework.

#### Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│ open-servo-firmware-stm32f301 (binary crate)                        │
│                                                                     │
│ • Hardware init (configure_*, start_*)                             │
│ • Implements Board + Timebase traits with Stm32f301Board           │
│ • ISR handlers call into Runtime                                     │
│ • Feature flags for board variants                                  │
│                                                                     │
│ main():      runtime.run_main_loop()                               │
│ ADC ISR:     runtime.run_fast_tick()                               │
│ UART ISR:    runtime.run_comms()                                    │
└─────────────────────────────────────────────────────────────────────┘
         │
         ▼
┌─────────────────────────────────────────────────────────────────────┐
│ open-servo-runtime (library crate)                                  │
│                                                                     │
│ • Runtime<B: Board + Timebase> struct                              │
│ • Owns: board, kernel, shadow, queues, embassy executor            │
│ • Decimation logic (fast → medium → slow)                          │
│ • Computes domain_rates() from board.base_dt_us()                  │
│ • Feature flags for optional services (rpc, persist)               │
└─────────────────────────────────────────────────────────────────────┘
         │
         ▼
┌─────────────────────────────────────────────────────────────────────┐
│ open-servo-kernel (library crate)                                   │
│                                                                     │
│ • Monolithic, NO feature flags                                      │
│ • Pure control logic, rate-agnostic                                │
└─────────────────────────────────────────────────────────────────────┘
```

#### Trait Definitions (open-servo-hw)

```rust
pub trait Board {
    fn servo_pos_kind(&self) -> ServoPosKind;
    fn motor_type(&self) -> MotorType;
    fn sensor_capabilities(&self) -> SensorCapabilities;
    fn read_sensors(&mut self) -> SensorFrame;
    fn write_motor(&mut self, cmd: MotorCommand);
}

pub trait Timebase {
    fn now_us(&self) -> TimeStampUs;
    fn base_dt_us(&self) -> MicroSecond;  // Hardware's base tick period
}
```

#### Firmware Crate (Ultra-Thin)

```rust
// open-servo-firmware-stm32f301/src/main.rs

pub struct Stm32f301Board { /* peripheral handles */ }

impl Board for Stm32f301Board {
    fn read_sensors(&mut self) -> SensorFrame { /* read ADC DMA */ }
    fn write_motor(&mut self, cmd: MotorCommand) { /* set PWM */ }
    // ...
}

impl Timebase for Stm32f301Board {
    fn now_us(&self) -> TimeStampUs { /* read TIM2 */ }
    fn base_dt_us(&self) -> MicroSecond { MicroSecond(100) }  // 10kHz
}

static RUNTIME: StaticCell<Runtime<Stm32f301Board>> = StaticCell::new();

#[entry]
fn main() -> ! {
    init_peripherals();
    let board = Stm32f301Board::new(/* ... */);
    let runtime = RUNTIME.init(Runtime::new(board, kernel_config()));
    runtime.run_main_loop()
}

#[interrupt]
fn DMA1_CH1() {  // ADC complete
    RUNTIME.get().run_fast_tick();
}

#[interrupt]
fn USART1() {
    RUNTIME.get().run_comms();
}
```

#### Runtime Crate

```rust
// open-servo-runtime/src/runtime.rs

pub struct Runtime<B: Board + Timebase> {
    board: B,
    kernel: ServoKernel,
    kctx: KernelCtx,
    shadow: ShadowStorage<1024>,

    // Decimation
    medium_counter: u8,
    slow_counter: u8,

    // Queues, embassy executor, etc.
}

impl<B: Board + Timebase> Runtime<B> {
    const MEDIUM_DECIMATE: u8 = 10;
    const SLOW_DECIMATE: u8 = 5;

    pub fn new(board: B, config: KernelConfig) -> Self { ... }

    /// Called from main() - runs embassy executor
    pub fn run_main_loop(&'static self) -> ! { ... }

    /// Called from ADC ISR @ base rate
    pub fn run_fast_tick(&mut self) {
        let frame = self.board.read_sensors();
        let now = self.board.now_us();
        let dt = self.board.base_dt_us();

        // Fast tick + decimation for medium/slow
        let cmd = self.run_tick_with_decimation(frame, now, dt);

        self.board.write_motor(cmd);
    }

    /// Called from UART ISR
    pub fn run_comms(&mut self) { ... }

    /// Compute rates from base_dt_us
    pub fn domain_rates(&self) -> DomainRatesHz {
        let base_dt = self.board.base_dt_us().0;
        let fast_hz = 1_000_000 / base_dt;
        DomainRatesHz::from_u32(
            fast_hz,
            fast_hz / Self::MEDIUM_DECIMATE as u32,
            fast_hz / (Self::MEDIUM_DECIMATE * Self::SLOW_DECIMATE) as u32,
            100,
        )
    }
}
```

#### What Moves Where

| From | To | What |
|------|-----|------|
| board | runtime | `ControlExecutor` (becomes part of Runtime) |
| board | runtime | Queue creation |
| board | runtime | Embassy executor creation |
| board | runtime | Task spawning |
| board | runtime | Decimation constants |
| **stays** | firmware | Hardware init |
| **stays** | firmware | ISR entry points (thin) |
| **stays** | firmware | `impl Board` + `impl Timebase` |
| **stays** | firmware | Board-specific feature flags |

---

### Step 2: Define Node Specifications

| Node            | Role      | Config   | State            | In                               | Out          |
| --------------- | --------- | -------- | ---------------- | -------------------------------- | ------------ |
| GateNode        | Limiter   | (empty)  | None             | (engaged, driver_ok, fault_mask) | GateReason   |
| PositionPidNode | Control   | PidGains | PosPidController | (sp, pos, limit)                 | Effort       |
| OpenLoopNode    | Control   | (empty)  | None             | Effort                           | Effort       |
| OutputNode      | Actuation | deadband | None             | (effort, gate, pos, pos_sp)      | MotorCommand |

### Step 3: Define State Bus (KernelState)

Namespaced wire bus:

```
KernelState
├── host: HostWires
│   ├── engaged, mode, fault_mask
│   ├── pos_sp, open_loop_effort
├── sensors: SensorWires
│   ├── frame, pos
├── gate: GateWires
│   └── reason
├── control: ControlWires
│   ├── effort
│   └── effort_limit
└── output: OutputWires
    └── cmd
```

### Step 4: Create Feature-Based Module Structure

```
firmware/open-servo-kernel/src/
├── features/
│   ├── mod.rs                  // Re-exports all features
│   ├── gate/
│   │   ├── mod.rs
│   │   └── node.rs             // GateNode
│   ├── position/
│   │   ├── mod.rs
│   │   ├── config.rs           // PidNodeConfig { gains }
│   │   ├── controller.rs       // PosPidController
│   │   └── node.rs             // PositionPidNode
│   ├── open_loop/
│   │   ├── mod.rs
│   │   └── node.rs             // OpenLoopNode
│   └── output/
│       ├── mod.rs
│       ├── config.rs           // OutputNodeConfig { deadband }
│       └── node.rs             // OutputNode
├── state.rs                    // KernelState (wire bus)
├── kernel.rs                   // ServoKernel
└── lib.rs
```

### Step 5: Implement Controller Wrapper

Create `PosPidController` in `features/position/controller.rs`:

- Wraps `PidControllerI16` from open-servo-math
- Implements `Controller` trait from kernel-api
- Handles runtime limit via `set_limit()`

### Step 6: Implement Features

Order: gate → position → open_loop → output

For each feature:
1. Create mod.rs with re-exports
2. Create node.rs with:
   - `impl HasRole`
   - `impl Resettable`
   - `impl Node<CtlFast>`
   - `impl Wired<CtlFast, KernelState>`
3. Create config.rs if tunable parameters exist
4. Add unit tests

### Step 7: Compose Graph and Refactor Kernel

```rust
type FastGraph = Chain<
    GateNode,
    Chain<
        SwitchByMode<PositionPidNode, OpenLoopNode>,
        OutputNode
    >
>;
```

Update kernel:
- Add `fast_graph: FastGraph` field
- `tick_fast()` calls `self.fast_graph.run(&mut self.st, ctx)`
- Route config updates to features
- Propagate reset to graph

### Step 8: Tests and Validation

- Unit tests for each node
- Unit tests for controller wrapper
- Integration test for full graph
- Verify telemetry, reset propagation, config updates

---

## Files to Modify

### Step 1: Firmware/Runtime Separation

| File | Changes |
|------|---------|
| `open-servo-hw/src/v2/board.rs` | Add `Timebase` trait with `base_dt_us()` |
| `open-servo-runtime/src/lib.rs` | Add `mod runtime;` |
| `open-servo-runtime/src/runtime.rs` | New: `Runtime<B>` struct with run methods |
| `open-servo-runtime/src/executor.rs` | Refactor into Runtime or remove |
| Rename `open-servo-board-stm32f301` → `open-servo-firmware-stm32f301` | Crate rename (DONE) |
| `firmware-stm32f301/src/board.rs` | New: `Stm32f301Board` impl Board + Timebase |
| `firmware-stm32f301/src/main.rs` | Thin: init hw, create Runtime, call run methods |

### Steps 2-7: Kernel Refactor

| File | Changes |
|------|---------|
| `open-servo-kernel/src/lib.rs` | Add `mod features;` |
| `open-servo-kernel/src/state.rs` | Namespaced wire bus |
| `open-servo-kernel/src/kernel.rs` | Use graph, new constructor |
| `open-servo-kernel/src/features/mod.rs` | Re-export features |
| `open-servo-kernel/src/features/gate/node.rs` | GateNode |
| `open-servo-kernel/src/features/position/config.rs` | PidNodeConfig |
| `open-servo-kernel/src/features/position/controller.rs` | PosPidController |
| `open-servo-kernel/src/features/position/node.rs` | PositionPidNode |
| `open-servo-kernel/src/features/open_loop/node.rs` | OpenLoopNode |
| `open-servo-kernel/src/features/output/config.rs` | OutputNodeConfig |
| `open-servo-kernel/src/features/output/node.rs` | OutputNode |

---

## Acceptance Checks

### Step 1: Firmware/Runtime Separation

```bash
# Firmware crate is thin
wc -l firmware/open-servo-firmware-stm32f301/src/main.rs  # < 100 lines

# Board trait has Timebase
rg "trait Timebase" firmware/open-servo-hw/src

# Runtime owns the tick logic
rg "fn run_fast_tick" firmware/open-servo-runtime/src/runtime.rs

# Decimation is in runtime, not firmware
rg "MEDIUM_DECIMATE" firmware/open-servo-runtime/src  # matches
rg "MEDIUM_DECIMATE" firmware/open-servo-firmware-stm32f301/src  # 0 matches
```

### Steps 2-7: Kernel Refactor

```bash
cargo test -p open-servo-kernel
cargo check -p open-servo-kernel --no-default-features

# Feature modules exist
ls firmware/open-servo-kernel/src/features/*/

# Nodes implement required traits
rg "impl Node<CtlFast>" firmware/open-servo-kernel/src/features
rg "impl HasRole for" firmware/open-servo-kernel/src/features

# No algorithm state in KernelState
rg "PidControllerI16" firmware/open-servo-kernel/src/state.rs  # 0 matches
```

---

## Future Work

- Add nodes for Medium/Slow/System domains
- Migrate from open-servo-core:
  - Safety monitors → `Node<CtlMedium>` with `Role::Monitor`
  - Thermal model → `Node<CtlSlow>` with `Role::Model`
  - Compliance FSM → graph composition

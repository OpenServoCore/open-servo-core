# Hierarchical Driver Pattern

A design convention for organizing firmware that has more than two or three peripherals interacting with a protocol stack. This doc captures the convention in generic terms so it can be applied to any project — examples are illustrative rather than tied to a specific codebase.

---

## 1. The problem it solves

Most embedded firmware starts the same way: each peripheral gets a struct or a handful of globals, ISRs read and write those globals directly, and inter-peripheral coordination happens through shared atomics or ad-hoc flags. This works for a few peripherals. It collapses around the time the firmware has:

- Multiple ISRs reading and writing the same state.
- State machines whose transitions are scattered across ISRs and main-loop code.
- Cross-peripheral coordination (timer X drives DMA Y based on UART Z's state).
- A protocol layer above the peripherals that needs to issue commands and read status.
- A testing ceiling: integration tests cover what fits on the bench, but the FSM and policy logic inside drivers can't be exercised in isolation because every method touches registers directly.

The symptoms look like:

- A module of "statics" with dozens of `AtomicU32` / `SyncUnsafeCell` items at the top level. Each is referenced from three or four places. Ownership is implicit. Adding a new feature requires reading every file that touches the related atomics to understand the invariants.
- Hand-rolled seqlocks built out of multiple atomics that are conceptually one struct.
- ISRs that take several screens to read, alternating between hardware register polls, state machine transitions, and statics manipulation.
- Multiple implicit state machines encoded as the *combination* of several boolean flags and counter values, with no single place that names the states or transitions.
- A protocol-level adapter ("this peripheral is a USB CDC", "this UART implements a custom protocol") that ends up holding peripheral state instead of just adapting it.
- Driver logic that can only be exercised by running the firmware end-to-end. A subtle FSM bug requires a hardware repro, a logic analyzer, and an afternoon — not a one-line unit test.

The pattern below is one way to keep the structure under control as the firmware grows past the "couple of peripherals" complexity ceiling.

---

## 2. The four-layer chip lib

The chip-specific firmware crate splits into four layers. Each layer has one reason to change, and dependencies run strictly downward.

| Layer | Lives in | Responsibility | Varies with |
| --- | --- | --- | --- |
| **Service** | `services/<name>.rs` | Adapt a protocol-side trait (defined in a chip-agnostic core) to driver method calls | The protocol stack above |
| **Driver** | `drivers/<name>/` | Own hardware state; expose a pure-method API; generic over its interface dependencies | Features and FSM logic |
| **Adapter** | `adapters/<name>.rs` | Implement driver-defined interfaces over HAL primitives (or record calls for tests) | Rarely; ripples from chip-variant changes |
| **HAL** | `hal/<name>.rs` | Register access; chip-specific resource identifiers (pin enums, peripheral handles, configuration structs) | Chip variant (cfg-gated) |

The dependency direction is strictly downward: services depend on drivers, drivers depend on interfaces they themselves define, adapters implement those interfaces and consume HAL, HAL talks to the silicon. **Nothing skips a layer.** A driver never imports HAL types or calls HAL functions directly; a service never reaches past its bound driver.

A small additional file sits *outside* the stack: the **IRQ vector binding**. It's a thin dispatcher — reads an interrupt-source flag, classifies into a logical event, and calls a driver method. It is structurally part of the driver-access surface, not a layer of its own. §8 covers it in detail.

### 2.1 Why the layers exist

Each layer corresponds to a thing that varies independently:

- **Chip variant** changes ripple into HAL. Adapters may need light `cfg` gating if the underlying HAL types change names, but driver interfaces and driver logic stay pinned.
- **Protocol changes** ripple into services. Everything below stays pinned.
- **Feature and FSM work** ripples into drivers. Adapters and HAL stay pinned.
- **Test substitution** ripples into adapters only. Drivers are generic over their interfaces, so a test substitutes a fake adapter without touching production code.

If a single change requires editing two layers at once, the layering is leaking. The typical culprit is a chip-specific type appearing in a driver method signature — the fix is to define a domain type at the driver layer (or hoist a shared primitive to a sibling location) so the interface carries no HAL types.

### 2.2 Reading order

To understand a system built this way, read top-down for orchestration or bottom-up for hardware:

- **Top-down**: service → top-level driver → composite routing → sub-drivers → interfaces → adapters. This tells the story of "what the protocol asked for and how it became register writes."
- **Bottom-up**: HAL → adapters → interfaces → driver methods → composite routing → service. This tells the story of "this register exists; what consumes it?"

Each file is independently legible. The IRQ vector binding file is read after the driver — it tells you which hardware sources route to which driver methods.

---

## 3. The driver convention

A driver is a Rust type that owns hardware state. Its public surface is pure methods — no `handle(Command)` indirection, no per-driver `Event`/`Effect` enums. Operations are named for what they do; signatures carry only the inputs and outputs they actually need.

The convention is in *naming and role separation*, not in shape:

- **Hardware events**: methods prefixed `on_*` (e.g. `on_idle`, `on_deadline`). They mutate state and may return an outcome value when something downstream needs to act on the transition. They never return `Result` — a hardware event already happened and can't be refused.
- **Software commands**: methods named for the operation (`arm`, `stage_baud`, `set`). They mutate state. They return `Result<T, E>` *when the operation has a runtime failure mode* a caller might reasonably encounter and handle. Programmer-error invariants (call-order violations, double-install) become `debug_assert!`s, not `Error` variants.
- **Accessors**: methods named for what they expose (`window`, `last_tick`, `is_armed`). `&self` (or `&mut self` when truly necessary), returning primitives or small typed values. No transition coupling.

The driver is always fully valid after `new(...)`. Uninit state — the time between static allocation and the first install — is encoded *at the static cell*, not inside the type:

```rust
static TOP: SyncUnsafeCell<Option<Top>> = SyncUnsafeCell::new(None);
```

Two questions, two layers. The cell answers "is this hardware installed?"; the methods on the driver answer "what does it do?", and assume the answer to the first is yes. §9 covers the access discipline.

Three categories, three reasons to call into a driver. Keep them distinct.

### 3.1 Events vs commands

The distinction is *direction*, not shape:

- **Event** — something the hardware made happen. ISR-originated. Already true by the time the method runs. The method records the new state and may return an outcome for the composite/ISR to route elsewhere. Methods are named `on_*`.

  The `on_*` name describes the *logical event*, not the *peripheral that delivered it*. `on_tx_complete` is a logical event ("the transmission finished"); `on_uart_tc` names the peripheral flag that signaled it. The same logical event may be delivered by a different mechanism on another chip variant — e.g. a timer compare-match instead of a USART status bit — and the driver's method should not change name when that happens. Naming events by the peripheral leaks chip variation into the driver's API.

- **Command** — something software wants. Main-loop or service-originated. May be rejected if it conflicts with current state; in that case the method returns `Result::Err`. Methods are named descriptively.

The same conceptual transition may originate from either direction. "Start transmitting" from a hardware compare-match is an event; "start transmitting" from main-loop intent is a command. The driver's response often differs (the event has *already started* the transmission and the driver records that; the command needs to verify state permits it before committing). Naming them differently keeps the distinction visible at every call site.

### 3.2 Accessors

Accessors are restricted to **stable observations** — values that don't depend on a pending transition. Examples that qualify:

- A configuration scalar published by an earlier command.
- A monotonic counter incremented by an event.
- A bool capturing a long-lived state ("is the bus armed?").

Examples that *don't* qualify (return them from the relevant event method instead):

- "The most recent decoded packet" — that's a transition output.
- "What happened during the last IRQ" — also a transition output.
- "Internal byte buffer" — that's leaking implementation; prefer a command-style enqueue method or a named operation.

If the urge to add an accessor comes up, ask whether the value is being *polled* (stable observation, fine) or *consumed* (transition output, return it from the event method).

### 3.3 Return values across drivers

When a method's return value flows into another driver — particularly within a composite — it carries **primitives**, not driver-internal types. If two drivers need to exchange a value that's conceptually a struct, the struct gets flattened into primitive fields at the source and reconstructed (or consumed field-by-field) at the receiver. This is the encapsulation rule that keeps drivers independent: a downstream driver should never need to import an upstream driver's types.

---

## 4. Composite drivers

Drivers compose recursively. A top-level driver may contain sub-drivers as fields; its method bodies route events to those sub-drivers and compose their return values:

```rust
pub struct Top {
    sub_a: SubA,
    sub_b: SubB,
    sub_c: SubC,
}

impl Top {
    pub fn on_external_trigger(&mut self) {
        // Decompose into the sub-driver event(s) this trigger affects.
        if let Some(value) = self.sub_a.on_triggered() {
            // Route a's outcome into b as an event.
            self.sub_b.on_data(value);
        }
    }

    pub fn do_work(&mut self, payload: Payload) -> Result<(), TopError> {
        self.sub_b.work(payload).map_err(TopError::SubB)
    }
}
```

The composite follows the same convention as its sub-drivers — `on_*` for hardware events, descriptive names for commands, accessors for observations — but its bodies route between children rather than mutating leaf state.

### 4.1 The composition rule

> **Sub-drivers are only reachable through their parent.** Sibling sub-drivers do not call each other directly. The parent owns the routing.

This is the rule that keeps the soup out. Without it, two sub-drivers acquire a hidden coupling that doesn't show up in either of their public surfaces; with it, all coupling is visible in the parent's method bodies.

### 4.2 The no-peer-to-peer rule for top-level drivers

> **Top-level drivers do not communicate with each other.**

If two top-level drivers would need to coordinate — for example, a motor driver consulting a sensor driver for thermal protection — they are not actually top-level. They become sub-drivers of a higher composite driver that coordinates them:

```
drivers/
  control/         ← new top-level
    mod.rs          composite: routes between motor and sensors
    motor.rs        was top-level, now sub-driver
    sensors.rs      was top-level, now sub-driver
```

The composite's method bodies are where the coordination lives. From the outside, `Control` exposes the same pure-method surface as any other driver.

This rule has teeth: if you find yourself wanting driver-to-driver calls, the architecture is telling you that two concerns belong together. Either compose them, or move the orchestration up to the protocol layer (see §7).

---

## 5. Interfaces and adapters

A driver doesn't talk to hardware directly. It talks to one or more **interfaces** — Rust traits that describe what the driver needs done — and is generic over the concrete type that implements each interface. The interfaces are defined alongside the drivers; the implementations (called **adapters**) live one layer below.

### 5.1 Drivers own their interfaces

The interface is a contract *written by the consumer*. It expresses what the driver wants done — set this output low, give me the current tick count, apply this trim step to the clock — not what the peripheral exposes. This direction matters:

- **Driver-shaped methods, not peripheral-shaped.** An interface to "set a digital output" reads `fn set(&mut self, level: Level)`. The implementor is already bound to one specific output; the driver issues a command. Compare to a peripheral-shaped equivalent `fn set_level(pin: Pin, level: Level)` where the driver has to thread a pin identifier on every call and import a chip-specific `Pin` type just to compile. The driver-shaped form lets the driver stay free of chip types entirely.
- **Narrow per-driver granularity.** Each interface declares only what one driver actually calls. A clock driver's "apply trim" interface doesn't expose the rest of the clock-controller registers. Reading the trait bounds on a driver tells you exactly what hardware operations it depends on.
- **Domain types at the interface boundary.** A `Level` enum (high/low) is a domain concept — it has meaning at the driver layer and above. It lives with the interface that uses it, not with the HAL. The HAL itself can take primitives (a `bool`) and the adapter does the one-line translation.

The reverse direction — HAL exposes traits, drivers consume them — is the conventional layered-HAL design. It works, but the trait surface ends up shaped by what the HAL chose to offer rather than by what the driver needs, which tends to produce wider traits than the consumer actually uses.

### 5.2 Adapters implement interfaces over HAL or fakes

Each interface has at least two implementations:

- **Production adapter.** A small struct that holds whatever per-instance state is needed (typically a chip-side resource identifier) and dispatches each interface method into HAL functions. Lives in `adapters/`. The struct is often zero-sized when the interface targets a singleton peripheral, so the production driver pays no size or speed cost for the indirection — monomorphization plus `#[inline]` collapses the dispatch to a direct HAL call.
- **Fake adapter.** A test-only implementation that records calls into a log instead of touching hardware. Used by host-side unit tests. Lives alongside the production adapters, gated `cfg(test)`.

Both satisfy the same interface, so they're interchangeable from the driver's perspective. Production code constructs the driver with the real adapter; tests construct it with the fake.

### 5.3 Naming

By convention:

- **Interface** — a capability-flavored name (`DigitalOut`, `TimeSource`, `ClockTrim`). Reads as something the driver wants done.
- **Production adapter** — names the specific resource it adapts (e.g. `gpio::Output`, `serial::Baud`, `clock::Trim`). The module path establishes "this is the register-backed implementation"; the struct name describes what specific resource it is.
- **Fake adapter** — `Fake` prefix (`FakeDigitalOut`, `FakeBaud`, `FakeTrim`). Tells the reader at a glance that this is the test implementation.

The pairing is intentional: production adapter names answer "which hardware?", fake names answer "which interface?".

### 5.4 Composition under interfaces

When a composite driver owns multiple sub-drivers, each sub-driver is generic over its own interfaces. The composite is generic over the union:

```rust
pub struct Top<I1, I2, I3>
where I1: InterfaceA, I2: InterfaceB, I3: InterfaceC,
{
    sub_a: SubA<I1>,
    sub_b: SubB<I2, I3>,
}

pub type ProdTop = Top<ProdA, ProdB, ProdC>;
```

The `pub type` alias keeps production call sites short — they say `ProdTop`, not the generic form. Tests instantiate `Top<FakeA, FakeB, FakeC>` directly.

When the generic-parameter list grows past about four, a **super-interface** collapses them. The composite takes a single type parameter that's bound to satisfy every leaf interface:

```rust
pub trait Chip: InterfaceA + InterfaceB + InterfaceC {}
impl<T> Chip for T where T: InterfaceA + InterfaceB + InterfaceC {}

pub struct Top<C: Chip> {
    sub_a: SubA<C>,
    sub_b: SubB<C, C>,
}
```

Leaves stay narrowly typed (so reading the leaf still shows exactly what hardware it depends on); the composite trades documentation-via-bounds for ergonomic call sites. Introduce the super-interface only when the parameter list actually hurts — it's an optimization, not the default shape.

---

## 6. Unit testing via adapter swap

The interface/adapter split exists primarily for one reason: **driver logic is unit-testable on the host without an MCU, without a simulator, and without writing integration-style harnesses.** Tests construct a driver with fake adapters, drive it through a scenario, and assert on the recorded calls and the driver's resulting state.

### 6.1 What this looks like

A leaf-driver test:

```rust
#[test]
fn arming_drives_output_low_then_high() {
    let fake = FakeDigitalOut::default();
    let mut driver = SomeDriver::new(fake);
    driver.arm();
    assert_eq!(driver.adapter().ops(), &[Op::Set(Level::Low), Op::Set(Level::High)]);
}
```

The fake records every method call in order. The test asserts on the call sequence, the call arguments, or the resulting state of the driver — whichever the driver under test is responsible for.

For composite drivers, a single fake-host struct hands out per-resource fake adapters that share an underlying log. The composite test constructs one host, asks it for each sub-driver's fake, and inspects the combined log:

```rust
#[test]
fn workflow_dispatches_to_both_sub_drivers() {
    let host = FakeHost::default();
    let mut top = Top::new(
        host.interface_a(),
        host.interface_b(),
    );
    top.do_workflow();
    assert_eq!(host.log(), &[expected_op_1, expected_op_2, expected_op_3]);
}
```

The host pattern keeps test ergonomics manageable even when a composite consumes several interfaces — there's still one fake to construct, one log to inspect.

### 6.2 What's testable at the driver layer

Most of what drivers actually do:

- **State machine transitions.** Every `on_*` method moves the driver through its FSM; tests cover the transition graph and its edges.
- **Command rejection.** Methods that return `Result::Err` when state forbids them; tests assert the rejection under each disallowed state.
- **Integrators, filters, schedulers.** Driver-owned math (drift correction, scheduling deadlines, retry counters, hysteresis) is pure given a fake for the time source.
- **Composition routing.** A composite's `on_*` methods dispatch to children; tests verify the routing with fakes that record receipt.

### 6.3 What's not testable at the driver layer

- **Actual peripheral behavior.** Register-write side effects, hardware-bounded timing, electrical signaling. These need integration tests on real hardware.
- **Adapter correctness.** The production adapter's translation from interface call to HAL function. Tested at the adapter layer (small surface, mostly mechanical) or by running the firmware on hardware.
- **ISR-priority races.** Any concurrency between ISRs sharing a driver. Out of scope for host tests; needs hardware or a model checker.

The split is intentional. Driver-layer host tests catch the bugs that show up in FSM and policy logic — by far the largest class of bugs in firmware of this shape. Hardware-shaped bugs are caught by hardware-shaped tests; the host-test layer isn't trying to replace them.

### 6.4 Fakes are adapters too

Fake adapters live in the same `adapters/` layer as production adapters, gated `cfg(test)`. They satisfy the same interfaces. From the driver's perspective there's no distinction; from the layering's perspective, **the adapter layer is the testability seam by design**. The fakes aren't a separate "mock layer" bolted on — they're a second flavor of adapter, parallel to the production one. That's why the interface/adapter split is in the architecture rather than added as an afterthought when tests get hard to write.

---

## 7. Services

Services are **adapters** between protocol-side traits (defined in a chip-agnostic core) and driver methods. They are thin:

```rust
// Protocol-side trait, defined in the chip-agnostic core:
pub trait SomeBus {
    fn send(&mut self, payload: Payload, schedule: Schedule);
    fn read_window(&mut self) -> Option<&[u8]>;
}

// Chip-specific service:
pub struct SomeBusAdapter;

impl SomeBus for SomeBusAdapter {
    fn send(&mut self, payload: Payload, schedule: Schedule) {
        let _ = unsafe { Top::get() }.send(
            |buf| serialize(payload, buf),
            schedule,
        );
    }

    fn read_window(&mut self) -> Option<&[u8]> {
        unsafe { Top::get() }.read_window()
    }
}
```

That's the whole service. Each method:

1. Acquires access to the driver.
2. Translates protocol-level arguments to driver method calls.
3. Translates the driver result back to the protocol-level return type.

(The "adapter" name here predates the interface/adapter terminology of §5 and refers to the same general pattern — adapting between two type universes — applied at the top of the stack. The two adapter roles don't conflict: §5 adapters bridge driver interfaces to HAL; services bridge protocol traits to driver methods.)

### 7.1 The 1:1 rule

> **Each service binds to exactly one top-level driver.**

If a service needs to call methods on two different top-level drivers to fulfill its trait, that's the no-peer-to-peer rule (§4.2) telling you to compose those drivers into a higher one. The 1:1 rule keeps services trivial: every method has exactly one driver to delegate to.

### 7.2 Why services aren't drivers

The temptation to skip the service layer and have drivers implement the protocol traits directly is real. The reason not to:

- Drivers live in the chip-specific tree (knows about registers, DMA channels, IRQ priorities).
- Protocol traits live in the chip-agnostic tree (knows about packet formats, schedule semantics).
- Mixing them couples the driver to the protocol, defeating the chip-agnostic split.

The service layer is the *anti-corruption layer* between two type universes. It's thin because the only thing it does is type translation; the work is in the driver, the policy is in the protocol stack. Both ends stay clean.

### 7.3 Cross-domain orchestration

> **Orchestration that spans multiple devices lives at the protocol layer (or main loop), not in any driver.**

Example: a control-table Write touches a configuration field that should both be persisted to non-volatile storage *and* trigger a runtime reconfiguration. The protocol layer's Write handler does:

```rust
self.persist.stage(addr, bytes)?;
if addr == BAUD_ADDR { self.events.send(Event::SetBaud(decode(bytes))); }
self.bus.send(Status::Empty { /* ack */ }, schedule);
```

Three service calls. None of the underlying drivers know the other exists. The orchestration knowledge — "this address means persist and also signal" — is protocol-level, so it lives at the protocol layer.

This is the same rule as §4.2 stated from the other direction: drivers don't reach across; the layer that knows about both concerns is responsible for the coordination.

---

## 8. Hardware demultiplexing

The IRQ vector binding file is structurally minimal: each ISR is a small dispatcher that reads the hardware status register, classifies into logical events, and calls into one or more drivers.

```rust
// irq.rs — vector binding
pub fn on_some_peripheral() {
    let status = peripheral_status();
    let driver = unsafe { Top::get() };

    if status.condition_a() { driver.on_condition_a(); }
    if status.condition_b() { driver.on_condition_b(); }
    if status.condition_c() { driver.on_condition_c(status.detail()); }
}
```

Two things this file does:
- Resolves the hardware vector to a driver.
- Splits a multi-source IRQ into discrete logical events.

It is **not** the place to:
- Mutate driver state.
- Make routing decisions between drivers.
- Hold its own state across invocations.

If an ISR is doing more than the dispatcher shape above, the work probably belongs inside the relevant `on_*` method on the driver. If routing complexity is creeping into the ISR (e.g., "if driver A's method returns X, then call driver B"), it belongs in the composite driver's `on_*` method, not in the ISR.

### 8.1 Single-source IRQs

When an IRQ has a single logical source (a dedicated timer compare match, a dedicated DMA channel), the ISR collapses to one line:

```rust
pub fn on_dedicated_timer() {
    unsafe { Top::get() }.on_deadline();
}
```

In hot-path cases where the dispatcher's overhead matters, the ISR can call the relevant sub-driver directly instead of going through the composite. This is a deliberate exception to the composition rule and should be commented as such — it's an optimization, not the default shape.

---

## 9. Static instances and access discipline

Each top-level driver gets a single static cell holding `Option<Driver>` so the static can be valid before bringup runs. `Driver::new(...)` is total — it always returns a fully-initialized driver — and uninit state lives only at the cell:

```rust
static TOP: SyncUnsafeCell<Option<Top>> = SyncUnsafeCell::new(None);

impl Top {
    /// SAFETY: bringup-only, pre-IRQ; sole writer. Must be called exactly once.
    pub unsafe fn install(/* construction args */) {
        let cell = unsafe { &mut *TOP.get() };
        debug_assert!(cell.is_none(), "Top: already installed");
        *cell = Some(Top::new(/* args */));
    }

    /// SAFETY: bringup installs Top before any ISR runs; access follows the
    /// driver's IRQ-priority contract documented at the call site.
    #[inline(always)]
    pub unsafe fn get() -> &'static mut Self {
        let cell = unsafe { &mut *TOP.get() };
        debug_assert!(cell.is_some(), "Top: accessed before install");
        // SAFETY: bringup ensures Some before any ISR fires.
        unsafe { cell.as_mut().unwrap_unchecked() }
    }
}
```

The two-layer Option keeps the driver type itself honest: `Top::new(...)` never returns a half-constructed object. The "is it installed?" question is asked once per access, at the accessor, `debug_assert`-checked in dev / `unwrap_unchecked` in release (sound because bringup must run before any IRQ).

The access discipline beyond install depends on the runtime model. For a no-RTOS firmware where ISRs share a single priority level and don't preempt each other (or use cooperative scheduling), the discipline is:

- **From an ISR at the shared priority**: `Top::get()` is safe because no other code at the same priority can preempt.
- **From the main loop**: `Top::get()` is safe only after disabling the relevant ISRs around the access, OR if the access only touches state the ISRs never mutate (read-only accessors are usually safe without IRQ masking; mutating commands typically need a masked critical section).

Every `unsafe { Top::get() }` site is making a contract claim. The discipline is:

1. State the contract once, in the type's `SAFETY:` doc comment.
2. Site-level comments only when the call site deviates from the default discipline.
3. Restrict all mutation to driver methods (no raw field access from outside).

If the project uses RTIC, an async executor, or another framework that provides safer ownership patterns, those should be preferred. The `SyncUnsafeCell<Option<Driver>>` + `unsafe fn install`/`unsafe fn get` pattern is the floor — it works without runtime support but requires discipline.

### 9.1 Cross-cell coordination

Most coordination happens through the composite-driver routing in §4. The remaining cases are where two genuinely independent top-level drivers need to exchange a value — and per §4.2, that should be rare. When it does happen, the channel is one of:

- An atomic flag/counter, when the value is a scalar.
- A small lock-free queue or `SyncUnsafeCell<Option<T>>` with seqlock semantics, when the value is a struct.

These cross-cell channels are exceptional. If you find yourself adding more than one or two, the architecture is telling you to reconsider whether the two drivers should be composed.

---

## 10. Worked example — abstract UART bridge

Hypothetical: a chip implements a serial protocol over a UART, with a half-duplex direction-control GPIO, hardware-timed transmit triggering from a timer, and clock-drift calibration based on observed receive timing.

The hardware concerns:
- UART RX and TX framing.
- A timer that schedules TX firing.
- A GPIO that gates the half-duplex direction.
- A clock-rate calibration loop based on RX byte arrival timing.

The protocol concerns (in a chip-agnostic core):
- Decode incoming packets, dispatch to handlers, emit replies.
- Decide when to persist configuration changes.

### 10.1 Mapping to drivers

The hardware concerns compose into one top-level driver because they share the UART's timing model:

```
drivers/
  bus/
    mod.rs       Bus composite — routes events between sub-drivers
    rx.rs        BusRx — RX byte buffer, IDLE detection, drift observation
    tx.rs        BusTx — TX buffer, fire scheduling, direction GPIO
    clock.rs     BusClock — rate constants, pending re-tune values
```

Each sub-driver exposes pure methods. The composite (`Bus`) has them too, but its bodies route between children rather than mutating leaf state:

```rust
impl Bus {
    pub fn on_uart_idle(&mut self) {
        if let Some(wire_end_tick) = self.rx.on_uart_idle() {
            // Wire-end is the input to TX's fire scheduling.
            self.tx.on_wire_end(wire_end_tick);
        }
    }

    pub fn on_timer_match(&mut self) {
        self.tx.on_fire_deadline();
    }

    pub fn on_uart_tc_completed(&mut self) {
        if self.tx.on_tc_completed().released() {
            self.clock.apply_pending();
        }
    }

    pub fn on_uart_rx_error(&mut self, flags: RxErrorFlags) {
        self.rx.on_error(flags);
    }

    pub fn arm_reply<F>(&mut self, writer: F, schedule: Schedule) -> Result<(), BusError>
    where
        F: FnOnce(&mut [u8]) -> usize,
    {
        self.tx.arm(writer, schedule).map_err(BusError::Tx)
    }

    pub fn stage_baud(&mut self, rate: BaudRate) -> Result<(), BusError> {
        self.clock.stage_baud(rate).map_err(BusError::Clock)
    }

    pub fn rx_window(&self) -> Option<&[u8]> {
        self.rx.window()
    }
}
```

The wire diagram is one file's worth of method bodies. Reading the `Bus` impl tells you every cross-component interaction in the system. Sub-drivers stay independent and unaware of each other.

### 10.2 Mapping to ISRs

```rust
// irq.rs
pub fn on_uart() {
    let status = uart_status();
    // SAFETY: see Bus::get.
    let bus = unsafe { Bus::get() };
    if status.has_rx_error() { bus.on_uart_rx_error(status.rx_flags()); }
    if status.idle()         { bus.on_uart_idle(); }
    if status.tc()           { bus.on_uart_tc_completed(); }
}

pub fn on_timer_compare() {
    unsafe { Bus::get() }.on_timer_match();
}
```

Two ISRs. Combined: about a dozen lines. Each does one thing and reads as one thing.

### 10.3 Mapping to a service

```rust
// services/bus.rs
pub struct BusAdapter;

impl ProtocolBus for BusAdapter {
    fn rx_window(&mut self) -> Option<&[u8]> {
        unsafe { Bus::get() }.rx_window()
    }

    fn send(&mut self, packet: Packet<'_>, schedule: Schedule) {
        let _ = unsafe { Bus::get() }.arm_reply(
            |buf| serialize(packet, buf),
            schedule,
        );
    }
}
```

The protocol layer's dispatcher consumes `ProtocolBus`, knowing nothing about UARTs, timers, or GPIOs. The adapter is purely type translation. If the same protocol runs on a different chip with different peripherals, only the service and driver change — the protocol stack is identical.

---

## 11. Related patterns and origins

This convention is a synthesis of established patterns rather than a novel design. The components map roughly as:

- **Command Query Responsibility Segregation (CQRS).** The mutating-command / read-only-query split. Commands here are pure methods rather than reified message objects, but the read/write separation is the same. Long established in distributed systems and DDD.
- **Domain events.** `on_*` methods are the *intake* of domain events; their return values are the *output*. Mealy-machine outputs from event-sourced systems map directly.
- **Hexagonal architecture / Ports and Adapters.** Drivers are the domain core; the interface/adapter split in §5 is a direct application of Cockburn's ports-and-adapters at the module level. The "interface" in this doc corresponds to a "port" in hexagonal vocabulary; "adapter" is the same term in both. Services play the analogous role one layer up, adapting protocol traits to driver methods.
- **DDD aggregates.** A composite driver is an aggregate root; sub-drivers are entities reachable only through the aggregate. The "no sibling access" rule is the aggregate-boundary rule.
- **Active Object** (Samek, *Practical Statecharts in C/C++*). The most embedded-specific analog: each active object is a state machine with an event queue, composed hierarchically, with no direct peer access. Used heavily in safety-critical embedded.
- **Four-layer chip stack.** The services / drivers / adapters / HAL split is more structured than the typical drivers-on-top-of-HAL embedded layering. The adapter layer is the explicit dependency-inversion seam: drivers depend on interfaces they themselves define, not on HAL, which is what makes driver logic testable without hardware.

In actor-based frameworks (Erlang/OTP, Akka, Drogue Device), the same shape appears with asynchronous message passing instead of synchronous method calls. The shape is the same; the runtime is different. This pattern is the *synchronous* application of actor-style discipline, which fits firmware where the cost of queues and an executor outweighs their decoupling benefit.

If a single reference best captures the embedded version of this pattern: Miro Samek's *Practical UML Statecharts in C/C++*. The active-object + hierarchical-state-machine + no-peer-access combination is articulated there, in C, with deep treatment of the embedded constraints (memory, latency, determinism).

---

## 12. When this pattern fits, and when it doesn't

### 12.1 Use it when

- The firmware has three or more peripherals that interact non-trivially.
- A protocol stack lives above the peripherals and needs to remain chip-agnostic.
- Multiple ISRs share state and the coordination is already painful or about to become so.
- The codebase has a multi-year lifetime — discipline upfront amortizes well.
- You want driver logic covered by host-side unit tests rather than relying solely on hardware integration tests.
- The team will grow, or onboarding new contributors is a concern. The pattern is unusually self-documenting because reading any one file gives a complete picture of one concern.

### 12.2 Don't use it when

- The firmware is small enough that a single struct with methods would do. A blinker, a thermostat, a one-peripheral data logger — the ceremony costs more than it saves.
- The project uses a framework (RTIC, Embassy with async, Zephyr) that provides better ownership and scheduling primitives. Use those instead. The pattern is the floor for "no framework available"; frameworks raise the floor.
- The hot path is so tight that even the dispatch overhead of `Top::on_event_x()` matters. (For most ISR work this isn't true — the dispatch is a tail call. But cycle-budget-bound ISRs may need to inline directly into sub-drivers, breaking the composite-routing rule as a deliberate optimization.)
- The protocol layer is trivial or absent. The service-as-adapter benefit only pays off when there's a chip-agnostic protocol on the other side. Without one, services are bureaucracy.

### 12.3 Trade-offs to accept

- **Translation boilerplate.** Return values flow as primitives between drivers, which means flattening structs at one boundary and rebuilding (or consuming field-by-field) at the next. Most of this is a few lines per transition.
- **Discipline-by-convention, not by compiler.** The contract is a naming convention. Rust can't enforce "no sibling sub-driver access," "services are 1:1 with top-level drivers," or "use `on_*` for hardware events." A doc-comment per module stating the convention plus code review is the enforcement mechanism. A team that doesn't read the convention will erode it; one that does will find the pattern self-reinforcing.
- **Generic-parameter propagation.** Drivers carry one type parameter per interface they consume; composites carry the union. A `pub type` alias hides the verbosity at production call sites, but compile errors and IDE hovers show the long form. The cost is paid in compile-time verbosity, not runtime: production adapters are typically zero-sized, so monomorphization plus `#[inline]` makes the adapter dispatch identical to a direct HAL call.
- **One more layer of files.** The adapter layer adds one file per peripheral concept (plus the fake adapter alongside). The cost is fixed — adapters don't grow per driver — and it buys the testability seam, but it's a real addition to the file tree.

---

## 13. Summary

Six rules:

1. **Drivers own hardware state.** Expose pure methods: `on_*` for hardware events (named for the logical event, not the peripheral that delivered it), descriptive names for commands, accessors for stable observations. Programmer-error invariants are `debug_assert!`s; runtime failures return `Result`.
2. **Composite drivers route between sub-drivers.** Sub-drivers are reachable only through the parent.
3. **Top-level drivers do not communicate with each other.** If two would need to, compose them into a higher driver.
4. **Services adapt protocol traits to drivers.** Each service binds to exactly one top-level driver. Services are thin.
5. **Cross-domain orchestration lives at the protocol layer**, never in drivers.
6. **Drivers are generic over driver-defined interfaces; adapters implement them.** Production adapters wrap HAL calls; fake adapters record calls for tests. Driver logic is unit-testable on the host without hardware.

Four layers, top to bottom:

- **Services** — adapt protocol traits to driver method calls.
- **Drivers** — hierarchical types with pure-method APIs, generic over their interfaces, with cell-level `Option<Driver>` storage.
- **Adapters** — implement driver interfaces over HAL primitives (production) or recording stubs (test).
- **HAL** — register access and chip-specific resource identifiers, cfg-gated per chip variant.

Plus one orthogonal file:

- **IRQ vector binding** — hardware demux that routes interrupt sources to driver methods.

The result is firmware where reading any one file gives you a complete picture of one concern, the wire diagram between concerns lives in composite-driver method bodies, the protocol layer stays unaware of any specific peripheral, and driver logic is exercisable by host-side unit tests.

# Hierarchical Driver Pattern

A design convention for organizing firmware that has more than two or three peripherals interacting with a protocol stack. This doc captures the convention in generic terms so it can be applied to any project — examples are illustrative rather than tied to a specific codebase.

---

## 1. The problem it solves

Most embedded firmware starts the same way: each peripheral gets a struct or a handful of globals, ISRs read and write those globals directly, and inter-peripheral coordination happens through shared atomics or ad-hoc flags. This works for a few peripherals. It collapses around the time the firmware has:

- Multiple ISRs reading and writing the same state.
- State machines whose transitions are scattered across ISRs and main-loop code.
- Cross-peripheral coordination (timer X drives DMA Y based on UART Z's state).
- A protocol layer above the peripherals that needs to issue commands and read status.

The symptoms look like:

- A module of "statics" with dozens of `AtomicU32` / `SyncUnsafeCell` items at the top level. Each is referenced from three or four places. Ownership is implicit. Adding a new feature requires reading every file that touches the related atomics to understand the invariants.
- Hand-rolled seqlocks built out of multiple atomics that are conceptually one struct.
- ISRs that take several screens to read, alternating between hardware register polls, state machine transitions, and statics manipulation.
- Multiple implicit state machines encoded as the *combination* of several boolean flags and counter values, with no single place that names the states or transitions.
- A protocol-level adapter ("this peripheral is a USB CDC", "this UART implements a custom protocol") that ends up holding peripheral state instead of just adapting it.

The pattern below is one way to keep the structure under control as the firmware grows past the "couple of peripherals" complexity ceiling.

---

## 2. Three roles, three files

The pattern splits the firmware into three distinct kinds of code, each living in its own location and having one reason to change:

| Role | Lives in | Responsibility | Stateful? |
| --- | --- | --- | --- |
| **Hardware demux** | The IRQ vector binding file | Read interrupt-source flags; classify into logical events | No |
| **Driver** | `drivers/<name>/` | Own hardware state; expose a strict contract for events / commands / observations | Yes |
| **Service** | `services/<name>.rs` | Adapt between a protocol-side trait and a driver's contract | No (or trivially so) |

Reading order to understand a system built this way:

1. Open the IRQ vector binding file. It tells you which hardware sources route to which drivers.
2. Open the driver. It tells you what state the device has and how it transitions.
3. Open the service. It tells you how the protocol stack above plugs into the device.

Each file is independently legible. No file requires understanding the others to read.

---

## 3. The driver contract

Every driver exposes the same surface:

```rust
impl SomeDriver {
    /// Hardware event arrived. Cannot be refused (it already happened).
    /// May produce an effect that callers route elsewhere.
    pub fn process(&mut self, event: Event) -> Effect;

    /// Software intent arrived. Can be rejected if the driver is in a
    /// state that doesn't permit the requested transition.
    pub fn handle(&mut self, command: Command) -> Result<Effect, Error>;

    /// Stable-state observation. No transition coupling; safe to call
    /// any time. Returns primitives or small typed values.
    pub fn <accessor>(&self) -> T;
}
```

Three categories, three reasons to call into a driver. Keep them distinct.

### 3.1 Events vs commands

The distinction is *direction*, not *content*:

- **Event** — something the hardware made happen. ISR-originated. Already true by the time the driver hears about it. Returns an effect describing what to do next (often `Nothing`).
- **Command** — something software wants. Main-loop or service-originated. May be rejected if it conflicts with current state. Returns a result so callers know whether to retry or report the rejection.

The same conceptual transition may be expressible as either an event or a command depending on origin. "Start transmitting" from a hardware-trigger compare match is an event; "start transmitting" from main-loop intent is a command. Naming them differently is intentional: the driver's response will often differ (the event has *already started* the transmission and the driver records that; the command needs to verify state permits it before committing).

### 3.2 Accessors

Accessors are restricted to **stable observations** — values that don't depend on a pending transition. Examples that qualify:

- A configuration scalar published by an earlier `handle()` call.
- A monotonic counter incremented by an event.
- A bool capturing a long-lived state ("is the bus armed?").

Examples that *don't* qualify (and should be returned as effects from `process()` instead):

- "The most recent decoded packet" — that's a transition output.
- "What happened during the last IRQ" — also a transition output.
- "Internal byte buffer" — that's leaking implementation; prefer command-style enqueue or named operations.

If the urge to add an accessor comes up, ask whether the value is being *polled* (stable observation, fine) or *consumed* (transition output, return it from an effect).

### 3.3 Effects

`Effect` is a small enum specific to each driver, always with a `Nothing` variant for the no-op case:

```rust
pub enum SomeDriverEffect {
    Nothing,
    SomethingHappened { /* primitives describing what */ },
    // …
}
```

Effects carry **primitives**, not driver-internal types. If two drivers need to exchange a value that's conceptually a struct, the struct gets flattened into primitive fields in the effect and reconstructed (or just consumed field-by-field) at the receiver. This is the encapsulation rule that keeps drivers independent: a downstream driver should never need to import an upstream driver's types.

---

## 4. Composite drivers

Drivers compose recursively. A top-level driver may contain sub-drivers as fields; its `process()` body routes events to those sub-drivers and composes their effects:

```rust
pub struct Top {
    sub_a: SubA,
    sub_b: SubB,
    sub_c: SubC,
}

impl Top {
    pub fn process(&mut self, event: TopEvent) -> TopEffect {
        match event {
            TopEvent::ExternalTrigger => {
                // Decompose into the sub-driver event(s) this trigger affects.
                let a_eff = self.sub_a.process(SubAEvent::Triggered);
                if let SubAEffect::DataReady { value } = a_eff {
                    // Route a's effect into b as an event.
                    self.sub_b.process(SubBEvent::AcceptData { value });
                }
                TopEffect::Nothing
            }
            // …
        }
    }

    pub fn handle(&mut self, cmd: TopCommand) -> Result<TopEffect, TopError> {
        match cmd {
            TopCommand::DoWork(payload) =>
                self.sub_b.handle(SubBCommand::Work(payload))
                    .map(|_| TopEffect::Nothing)
                    .map_err(TopError::SubB),
            // …
        }
    }
}
```

The top-level driver has the same `process` / `handle` / `accessor` surface as the sub-drivers. The contract is fractal: at every level you see the same shape. The only thing that distinguishes a composite driver from a leaf one is that its body routes between children rather than mutating leaf state.

### 4.1 The composition rule

> **Sub-drivers are only reachable through their parent.** Sibling sub-drivers do not call each other directly. The parent owns the routing.

This is the rule that keeps the soup out. Without it, two sub-drivers acquire a hidden coupling that doesn't show up in either of their public surfaces; with it, all coupling is visible in the parent's `process()` body.

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

The composite's `process()` is where the coordination lives. From the outside, `Control` exposes the same `process / handle / accessor` surface as any other driver.

This rule has teeth: if you find yourself wanting driver-to-driver calls, the architecture is telling you that two concerns belong together. Either compose them, or move the orchestration up to the protocol layer (see §5).

---

## 5. Services

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
        let driver = unsafe { Top::get() };
        let _ = driver.handle(TopCommand::Send {
            writer: |buf| serialize(payload, buf),
            schedule,
        });
    }

    fn read_window(&mut self) -> Option<&[u8]> {
        unsafe { Top::get() }.read_window()
    }
}
```

That's the whole service. Each method:

1. Acquires access to the driver.
2. Translates protocol-level arguments to driver `handle` commands or accessor calls.
3. Translates the driver result back to the protocol-level return type.

### 5.1 The 1:1 rule

> **Each service binds to exactly one top-level driver.**

If a service needs to call methods on two different top-level drivers to fulfill its trait, that's the no-peer-to-peer rule (§4.2) telling you to compose those drivers into a higher one. The 1:1 rule keeps services trivial: every method has exactly one driver to delegate to.

### 5.2 Why services aren't drivers

The temptation to skip the service layer and have drivers implement the protocol traits directly is real. The reason not to:

- Drivers live in the chip-specific tree (knows about registers, DMA channels, IRQ priorities).
- Protocol traits live in the chip-agnostic tree (knows about packet formats, schedule semantics).
- Mixing them couples the driver to the protocol, defeating the chip-agnostic split.

The service layer is the *anti-corruption layer* between two type universes. It's thin because the only thing it does is type translation; the work is in the driver, the policy is in the protocol stack. Both ends stay clean.

### 5.3 Cross-domain orchestration

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

## 6. Hardware demultiplexing

The IRQ vector binding file is structurally minimal: each ISR is a small dispatcher that reads the hardware status register, classifies into logical events, and calls into one or more drivers.

```rust
// irq.rs — vector binding
pub fn on_some_peripheral() {
    let status = peripheral_status();
    let driver = unsafe { Top::get() };

    if status.condition_a() { driver.process(TopEvent::ConditionA); }
    if status.condition_b() { driver.process(TopEvent::ConditionB); }
    if status.condition_c() { driver.process(TopEvent::ConditionC(status.detail())); }
}
```

Two things this file does:
- Resolves the hardware vector to a driver.
- Splits a multi-source IRQ into discrete logical events.

It is **not** the place to:
- Mutate driver state.
- Make routing decisions between drivers.
- Hold its own state across invocations.

If an ISR is doing more than the dispatcher shape above, the work probably belongs inside the driver's `process()` body. If routing complexity is creeping into the ISR (e.g., "if driver A's process returns X, then call driver B"), it belongs in a composite driver's `process()` body, not in the ISR.

### 6.1 Single-source IRQs

When an IRQ has a single logical source (a dedicated timer compare match, a dedicated DMA channel), the ISR collapses to one line:

```rust
pub fn on_dedicated_timer() {
    unsafe { Top::get() }.process(TopEvent::Deadline);
}
```

In hot-path cases where the dispatcher's overhead matters, the ISR can call the relevant sub-driver directly instead of going through the composite. This is a deliberate exception to the composition rule and should be commented as such — it's an optimization, not the default shape.

---

## 7. Static instances and access discipline

Each top-level driver gets a single static instance, typically in a `SyncUnsafeCell`:

```rust
static TOP: SyncUnsafeCell<Top> = SyncUnsafeCell::new(Top::new());

impl Top {
    /// SAFETY: caller holds the access discipline (see below).
    #[inline(always)]
    pub unsafe fn get() -> &'static mut Self {
        unsafe { &mut *TOP.get() }
    }
}
```

The access discipline depends on the runtime model. For a no-RTOS firmware where ISRs share a single priority level and don't preempt each other (or use cooperative scheduling), the discipline is:

- **From an ISR at the shared priority**: `Top::get()` is safe because no other code at the same priority can preempt.
- **From the main loop**: `Top::get()` is safe only after disabling the relevant ISRs around the access, OR if the access only touches state the ISRs never mutate (read-only accessors are usually safe without IRQ masking; mutating commands typically need a masked critical section).

This isn't free of footguns — every `unsafe { Top::get() }` site is making a contract claim. The discipline is:

1. State the contract once, in the type's `SAFETY:` doc comment.
2. Site-level comments only when the call site deviates from the default discipline.
3. Restrict all mutation to driver methods (no raw field access from outside).

If the project uses RTIC, an async executor, or another framework that provides safer ownership patterns, those should be preferred. The `SyncUnsafeCell` + `unsafe fn get()` pattern is the floor — it works without runtime support but requires discipline.

### 7.1 Cross-cell coordination

Most coordination happens through the composite-driver routing in §4. The remaining cases are where two genuinely independent top-level drivers need to exchange a value — and per §4.2, that should be rare. When it does happen, the channel is one of:

- An atomic flag/counter, when the value is a scalar.
- A small lock-free queue or `SyncUnsafeCell<Option<T>>` with seqlock semantics, when the value is a struct.

These cross-cell channels are exceptional. If you find yourself adding more than one or two, the architecture is telling you to reconsider whether the two drivers should be composed.

---

## 8. Worked example — abstract UART bridge

Hypothetical: a chip implements a serial protocol over a UART, with a half-duplex direction-control GPIO, hardware-timed transmit triggering from a timer, and clock-drift calibration based on observed receive timing.

The hardware concerns:
- UART RX and TX framing.
- A timer that schedules TX firing.
- A GPIO that gates the half-duplex direction.
- A clock-rate calibration loop based on RX byte arrival timing.

The protocol concerns (in a chip-agnostic core):
- Decode incoming packets, dispatch to handlers, emit replies.
- Decide when to persist configuration changes.

### 8.1 Mapping to drivers

The hardware concerns compose into one top-level driver because they share the UART's timing model:

```
drivers/
  bus/
    mod.rs       Bus composite — routes events between sub-drivers
    rx.rs        BusRx — RX byte buffer, IDLE detection, drift observation
    tx.rs        BusTx — TX buffer, fire scheduling, direction GPIO
    clock.rs     BusClock — rate constants, pending re-tune values
```

Each sub-driver has `process / handle / accessors`. The composite (`Bus`) has them too, but its body routes:

```rust
impl Bus {
    pub fn process(&mut self, event: BusEvent) -> BusEffect {
        match event {
            BusEvent::UartIdle => {
                if let RxEffect::PacketBoundary { wire_end_tick } =
                    self.rx.process(RxEvent::UartIdle)
                {
                    // Wire-end is the input to TX's fire scheduling.
                    self.tx.process(TxEvent::WireEnd { tick: wire_end_tick });
                }
                BusEffect::Nothing
            }
            BusEvent::TimerMatch => {
                self.tx.process(TxEvent::FireDeadline);
                BusEffect::Nothing
            }
            BusEvent::UartTcCompleted => {
                if matches!(self.tx.process(TxEvent::TcCompleted), TxEffect::Released) {
                    self.clock.process(ClockEvent::ApplyPending);
                }
                BusEffect::Nothing
            }
            BusEvent::UartRxError(flags) => {
                self.rx.process(RxEvent::Error(flags));
                BusEffect::Nothing
            }
        }
    }

    pub fn handle(&mut self, cmd: BusCommand) -> Result<BusEffect, BusError> {
        match cmd {
            BusCommand::ArmReply { writer, schedule } =>
                self.tx.handle(TxCommand::Arm { writer, schedule })
                    .map(|_| BusEffect::Nothing)
                    .map_err(BusError::Tx),
            BusCommand::StageBaud(rate) =>
                self.clock.handle(ClockCommand::StageBaud(rate))
                    .map(|_| BusEffect::Nothing)
                    .map_err(BusError::Clock),
        }
    }

    pub fn rx_window(&self) -> Option<&[u8]> { self.rx.window() }
}
```

The wire diagram is in one file. Reading `Bus::process()` tells you every cross-component interaction in the system. Sub-drivers stay independent and unaware of each other.

### 8.2 Mapping to ISRs

```rust
// irq.rs
pub fn on_uart() {
    let status = uart_status();
    let bus = unsafe { Bus::get() };
    if status.has_rx_error() { bus.process(BusEvent::UartRxError(status.rx_flags())); }
    if status.idle()         { bus.process(BusEvent::UartIdle); }
    if status.tc()           { bus.process(BusEvent::UartTcCompleted); }
}

pub fn on_timer_compare() {
    unsafe { Bus::get() }.process(BusEvent::TimerMatch);
}
```

Two ISRs. Combined: about a dozen lines. Each does one thing and reads as one thing.

### 8.3 Mapping to a service

```rust
// services/bus.rs
pub struct BusAdapter;

impl ProtocolBus for BusAdapter {
    fn rx_window(&mut self) -> Option<&[u8]> {
        unsafe { Bus::get() }.rx_window()
    }

    fn send(&mut self, packet: Packet<'_>, schedule: Schedule) {
        let _ = unsafe { Bus::get() }.handle(BusCommand::ArmReply {
            writer: |buf| serialize(packet, buf),
            schedule,
        });
    }
}
```

The protocol layer's dispatcher consumes `ProtocolBus`, knowing nothing about UARTs, timers, or GPIOs. The adapter is purely type translation. If the same protocol runs on a different chip with different peripherals, only the service and driver change — the protocol stack is identical.

---

## 9. Related patterns and origins

This convention is a synthesis of established patterns rather than a novel design. The components map roughly as:

- **Command Query Responsibility Segregation (CQRS).** The `handle` (commands mutate, may emit) / accessor (queries read) split. Long established in distributed systems and DDD.
- **Domain events.** `process(Event)` is the *intake* of domain events; `Effect` is the *output*. Mealy-machine outputs from event-sourced systems map directly.
- **Hexagonal architecture / Ports and Adapters.** Drivers are the domain core; services are adapters to outer protocol traits. Cockburn's framing applied at the module level instead of the application level.
- **DDD aggregates.** A composite driver is an aggregate root; sub-drivers are entities reachable only through the aggregate. The "no sibling access" rule is the aggregate-boundary rule.
- **Active Object** (Samek, *Practical Statecharts in C/C++*). The most embedded-specific analog: each active object is a state machine with an event queue, composed hierarchically, with no direct peer access. Used heavily in safety-critical embedded.
- **Layered HAL.** The drivers-on-top-of-HAL stack is standard embedded layering; the strict service-as-adapter step on top of drivers is what this pattern adds.

In actor-based frameworks (Erlang/OTP, Akka, Drogue Device), the same shape appears with asynchronous message passing instead of synchronous method calls. The shape is the same; the runtime is different. This pattern is the *synchronous* application of actor-style discipline, which fits firmware where the cost of queues and an executor outweighs their decoupling benefit.

If a single reference best captures the embedded version of this pattern: Miro Samek's *Practical UML Statecharts in C/C++*. The active-object + hierarchical-state-machine + no-peer-access combination is articulated there, in C, with deep treatment of the embedded constraints (memory, latency, determinism).

---

## 10. When this pattern fits, and when it doesn't

### 10.1 Use it when

- The firmware has three or more peripherals that interact non-trivially.
- A protocol stack lives above the peripherals and needs to remain chip-agnostic.
- Multiple ISRs share state and the coordination is already painful or about to become so.
- The codebase has a multi-year lifetime — discipline upfront amortizes well.
- The team will grow, or onboarding new contributors is a concern. The pattern is unusually self-documenting because reading any one file gives a complete picture of one concern.

### 10.2 Don't use it when

- The firmware is small enough that a single struct with methods would do. A blinker, a thermostat, a one-peripheral data logger — the ceremony costs more than it saves.
- The project uses a framework (RTIC, Embassy with async, Zephyr) that provides better ownership and scheduling primitives. Use those instead. The pattern is the floor for "no framework available"; frameworks raise the floor.
- The hot path is so tight that even the dispatch overhead of `Top::process(Event::X)` matters. (For most ISR work this isn't true — the dispatch is a tail call. But cycle-budget-bound ISRs may need to inline directly into sub-drivers, breaking the composite-routing rule as a deliberate optimization.)
- The protocol layer is trivial or absent. The service-as-adapter benefit only pays off when there's a chip-agnostic protocol on the other side. Without one, services are bureaucracy.

### 10.3 Trade-offs to accept

- **More files, more types.** Each driver has its `Event`, `Command`, `Effect`, `Error` enums and its `process` / `handle` / accessor methods. For a small project, this is bookkeeping that doesn't pay back.
- **Translation boilerplate.** Effects flow as primitives between drivers, which means flattening structs at one boundary and rebuilding (or consuming field-by-field) at the next. Most of this is a few lines per transition.
- **Discipline-by-convention, not by compiler.** The contract is a convention. Rust can't enforce "no sibling sub-driver access" or "services are 1:1 with top-level drivers." A doc-comment per module stating the convention plus code review is the enforcement mechanism. A team that doesn't read the convention will erode it; one that does will find the pattern self-reinforcing.

---

## 11. Summary

Five rules:

1. **Drivers own hardware state.** Expose `process(Event)` / `handle(Command)` / stable accessors. Nothing else is public.
2. **Composite drivers route between sub-drivers.** Sub-drivers are reachable only through the parent.
3. **Top-level drivers do not communicate with each other.** If two would need to, compose them into a higher driver.
4. **Services adapt protocol traits to drivers.** Each service binds to exactly one top-level driver. Services are thin.
5. **Cross-domain orchestration lives at the protocol layer**, never in drivers.

Three roles:

- ISR vector binding: hardware demux only.
- Driver: hierarchical FSM with strict contract.
- Service: protocol-trait adapter.

The result is firmware where reading any one file gives you a complete picture of one concern, where the wire diagram between concerns is concentrated in composite-driver `process()` bodies, and where the protocol layer remains unaware of any specific peripheral.

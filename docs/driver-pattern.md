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
| **Provider** | `provider/<role>.rs` (or `<role>/v<variant>.rs`) | Implement driver-defined interfaces over HAL primitives (production) or record calls (tests); own per-peripheral init including clock-gate enable | Role-to-peripheral allocation; chip variant (cfg-gated per project) |
| **HAL** | `hal/<peripheral>.rs` (or `<peripheral>/v<ip>.rs`) | Register access; peripheral primitives; chip-specific resource identifiers (pin enums, peripheral handles, configuration structs) | Peripheral IP version (multiple chips may share; cfg-gated when they diverge) |

The dependency direction is strictly downward: services depend on drivers, drivers depend on interfaces they themselves define, providers implement those interfaces and consume HAL, HAL talks to the silicon. **Nothing skips a layer.** A driver never imports HAL types or calls HAL functions directly; a service never reaches past its bound driver.

Three small facades sit *outside* the stack:
- The **driver registry** — owns static storage for installed driver *instances* and exposes typed accessors. Driver types themselves carry no statics. §9.1 covers it.
- The **provider facade** — orchestrates hardware setup and deferred IRQ enable. Bringup goes through this single entry point. §9.2 covers it.
- The **IRQ vector binding** — a thin dispatcher that reads an interrupt-source flag, classifies into a logical event, and calls a driver method through the registry. §8 covers it.

None are layers of their own — they're orchestration surfaces over the four-layer stack.

### 2.1 Why the layers exist

Each layer corresponds to a thing that varies independently:

- **Chip variant** changes ripple into the provider's variant-specific files (peripheral choice differs per chip) and may ripple into HAL when peripheral IP version differs. Driver interfaces and driver logic stay pinned across chip variants — that's the point of the provider seam (§5.6).
- **Protocol changes** ripple into services. Everything below stays pinned.
- **Feature and FSM work** ripples into drivers. Providers and HAL stay pinned.
- **Test substitution** ripples into providers only. Drivers are generic over their interfaces, so a test substitutes a fake provider without touching production code.

If a single change requires editing two layers at once, the layering is leaking. The typical culprit is a chip-specific type appearing in a driver method signature — the fix is to define a domain type at the driver layer (or hoist a shared primitive to a sibling location) so the interface carries no HAL types.

### 2.2 Reading order

To understand a system built this way, read top-down for orchestration or bottom-up for hardware:

- **Top-down**: service → top-level driver → composite routing → sub-drivers → interfaces → providers. This tells the story of "what the protocol asked for and how it became register writes."
- **Bottom-up**: HAL → providers → interfaces → driver methods → composite routing → service. This tells the story of "this register exists; what consumes it?"

Each file is independently legible. The IRQ vector binding file is read after the driver — it tells you which hardware sources route to which driver methods.

### 2.3 Chip-agnostic drivers by construction

The four-layer split has a stronger consequence than "driver code stays pinned across chip variants" — **drivers are by construction unaware of any specific chip.** Their imports name only their own trait surface (e.g., `drivers/traits.rs`) and Rust core; chip-specific types never appear in a driver method signature, field, or body. A second chip is a second provider set; the entire `drivers/` tree compiles unchanged.

The unit-test seam (§6) is the in-tree proof: `Driver<Fake>` and `Driver<Real>` are the same driver code with different providers — same as `Driver<ChipA>` and `Driver<ChipB>` would be. The property generalizes from "swap a fake for a real provider" to "swap chip A's provider set for chip B's."

In hexagonal-architecture terms (§11), drivers are the **domain core** and providers are the bottom adapter ring. The convention makes drivers portable across chips for the same reason hexagonal makes the domain core portable across infrastructure — it's the same property applied to the chip-portability axis. In practice this means a mature multi-chip codebase can lift the entire `drivers/` tree into a chip-agnostic core crate alongside the protocol stack, leaving only `provider/`, `hal/`, and bringup in each chip-specific firmware crate. §2.4 describes that crate-level packaging.

### 2.4 Packaging across crates

The four-layer split is a module convention inside a single firmware crate, and that's enough for small projects. For projects with multi-chip support, long-term-support guarantees, or quality goals that benefit from machine-enforced layering, the same four layers map cleanly across crates — and the chip-family crate takes on a sharpened role: it becomes an **orchestration crate**.

```
<project>-core            ← domain types, protocol stack, control loops — chip-agnostic
<project>-drivers         ← driver state machines + driver-trait surface — chip-agnostic
                            depends on <project>-core for domain types
<chip-family>             ← ORCHESTRATION CRATE
                            composes hal + providers + services + drivers (instantiated)
                            + ISRs + bringup into a running system
                            depends on <project>-core, <project>-drivers, <chip-family>-metapac
                            chip-variant feature flags forward to <chip-family>-metapac
firmware/<board>          ← binary entry point
                            holds the BoardWiring const + main()
                            depends on <chip-family>, pins one chip-variant feature flag
```

Two crate-level properties fall out:

- **The driver crate is a pure library.** `<project>-drivers` defines driver types and their trait surfaces — nothing else. No registry, no concrete instances, no bringup. Drivers stay generic over their trait bounds; the crate has no HAL dependency to import. Chip-agnosticity becomes a `cargo build` invariant.
- **The chip-family crate is the orchestration crate.** It holds *everything* that composes the system: HAL primitives, trait impls (providers), trait impls for core (services), the driver registry (`Drivers` struct holding concrete instances like `DxlRx<DmaRing07>`), the bringup sequence, and the ISR vector binding. §2.5 covers its internal layout.

The consumer-owned-interface rule (§5.1) carries across crate boundaries: **driver traits live in `<project>-drivers`, not in `<chip-family>`.** Providers in `<chip-family>` import the trait from `<project>-drivers` and implement it over local HAL primitives. This is cross-crate dependency-inversion — the lower-level crate depends on the higher-level crate's interface.

The driver registry stays chip-side rather than in `<project>-drivers`. Its fields are concrete chip-side instantiations (`dxl_rx: DxlRx<DmaRing07>`, `dxl_clock: DxlClock<Tim2Mono>`); putting the registry in the driver crate would either force a circular dependency (driver crate names chip-side types) or force generic-parameter explosion (every accessor signature carries the full provider set). Letting the orchestration crate own its registry is honest: each chip family has its own bag of concrete driver instances, and that bag belongs with the chip.

A second chip family is a new sibling orchestration crate with the same shape — its own HAL, its own providers, its own registry, its own bringup. `<project>-drivers` and `<project>-core` are unchanged. Board binaries select chip family by depending on the corresponding orchestration crate.

The trade-off:

- **Single-crate firmware** — all four layers as modules in one crate. Less ceremony. Chip-agnosticity is a code-review convention.
- **Multi-crate firmware** — chip-agnosticity is a build-time invariant; the driver crate physically cannot import HAL. Multi-chip support, layered review automation, and long-term-support boundaries all become structural. Recommended when project longevity or quality discipline matters.

The split is mechanical once the role-shaped provider convention is in place. The cost is paid in extra crate scaffolding (Cargo.toml, lib.rs entries); the value is the build-time enforcement of the architectural property the convention is designed to provide.

### 2.5 Internal layout of the orchestration crate

Within the chip-family orchestration crate, files organize into five top-level modules. The split mirrors the conceptual layer model but adds explicit homes for things the layer model leaves implicit (declarative config, orchestration glue):

| Module | Shape | Contents | Reads from |
| --- | --- | --- | --- |
| `hal/` | Foundation | Peripheral IP primitives (register dance); chip-family type enums (`Pin`, `UsartRemap`, `TimerChannel`, `DmaChannel`) — values may be chip-variant-routed | — |
| `cfg/` | Declarative | Board-wiring schema struct (`BoardWiring { dxl_uart_rx_pin: Pin, ... }`) — types only, no concrete values | `hal/` types |
| `providers/` | Impl (driver-facing) | Role-shaped impls of `<project>-drivers` traits | `hal/`, `cfg/` |
| `services/` | Impl (core-facing) | Impls of `<project>-core` traits | `runtime::registry` |
| `runtime/` | Orchestration | `registry.rs` (Drivers struct + facade), `init.rs` (bringup sequence), `isr.rs` (handler bodies + `install_isrs!` macro) | All of the above |

Dependency flow (within the crate):

```
hal ──┐
cfg ──┴── providers ──┐
                       ├── runtime::registry ── runtime::init
                       │                         runtime::isr
                       │
                       └── services
```

The split rests on three observations:

- **Two shapes, cleanly separated.** Providers and services are *impls* — consumer-owned-interface fulfillment, just for different consumer crates (drivers and core respectively). Registry / init / isr are *orchestrators* — they hold, sequence, or dispatch concrete instances. The shapes don't mix in one file: a provider file impls a trait for any consumer; an ISR file names a specific driver to dispatch to.
- **Wiring schema is chip-family-shaped; wiring values are board-shaped.** The `BoardWiring` struct definition lives in `cfg/` because its fields are typed with chip-family enums (`Pin`, `UsartRemap`, …) — the schema varies by chip variant. The concrete values (which physical pin? which UART? which timer channel?) live in the board binary as a `pub const WIRING: BoardWiring`. The runtime entry point takes `&'static BoardWiring` as a parameter; nothing in the chip-family crate hardcodes pin choices.
- **ISRs are peripheral-driver bindings, not role-shaped.** A handler names a specific peripheral *and* a specific driver — that pairing is a one-off binding, not a reusable role. So ISRs live in `runtime/isr.rs` (alongside `install_isrs!`) rather than in provider files. A provider file impls `DmaRing` for any consumer; the ISR file names "DMA1_CH7 dispatches to `DxlRx` specifically."

**Chip-variant routing** uses the same cfg-routing pattern at three layers:

```
hal/types.rs              -- cfg-routed enum bodies per package (V006P8U6, V006E8U6, V307VCT6 …)
cfg/board_wiring.rs       -- cfg-routed schema (different chips support different fields)
providers/<role>.rs       -- cfg-routed impl (different chips need different register writes)
```

A single-variant project uses flat files (`hal/types.rs`, `cfg/board_wiring.rs`, `providers/monotonic.rs`). When a second variant lands, each splits into a sub-directory (`hal/types/{mod,v006p8u6,v307vct6}.rs`, etc.) with cfg-routed re-exports. The pattern is the same at all three layers — one mental model handles every chip-variant axis.

---

## 3. The driver convention

A driver is a Rust type that owns hardware state. Its public surface is pure methods — no `handle(Command)` indirection, no per-driver `Event`/`Effect` enums. Operations are named for what they do; signatures carry only the inputs and outputs they actually need.

The convention is in *naming and role separation*, not in shape:

- **Hardware events**: methods prefixed `on_*` (e.g. `on_idle`, `on_deadline`). They mutate state and may return an outcome value when something downstream needs to act on the transition. They never return `Result` — a hardware event already happened and can't be refused.
- **Software commands**: methods named for the operation (`arm`, `stage_baud`, `set`). They mutate state. They return `Result<T, E>` *when the operation has a runtime failure mode* a caller might reasonably encounter and handle. Programmer-error invariants (call-order violations, double-install) become `debug_assert!`s, not `Error` variants.
- **Accessors**: methods named for what they expose (`window`, `last_tick`, `is_armed`). `&self` (or `&mut self` when truly necessary), returning primitives or small typed values. No transition coupling.

The driver is always fully valid after `new(...)`. The driver type itself carries no static storage and exposes no `install` / `get` methods — singleton lifecycle is delegated to a separate registry (§9.1). Tests construct drivers directly with fake providers; the registry exists only to give production code a place to park installed instances.

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

### 4.3 Sub-composites for disjoint borrows

§7.4's data-centric trait surface relies on the driver caching the request that drives the reply. When the cached request borrows one part of the driver's state and the reply path mutates another, the naive single-struct form runs into Rust's borrow checker: returning `Item<'a>` from a method holds `&mut self` for as long as the item is held, and the reply call can't reborrow.

A **sub-composite** resolves this. Its structural job is to expose disjoint mutable halves:

```rust
pub struct Inner {
    a: HalfA,
    b: HalfB,
}

impl Inner {
    /// Disjoint borrow — both halves usable at once.
    pub fn split_mut(&mut self) -> (&mut HalfA, &mut HalfB) {
        (&mut self.a, &mut self.b)
    }

    // Forwarders for sequential single-half call sites.
    pub fn observe(&mut self) -> Option<Token> { self.a.observe() }
    pub fn act(&mut self, payload: Payload) -> Result<(), Error> { self.b.act(payload) }
}
```

The parent's hot path is closure-based — `split_mut` at the seam hands both halves to the dispatcher at once:

```rust
impl Top {
    pub fn poll<F>(&mut self, f: F)
    where
        F: for<'a> FnOnce(Item<'a>, &mut Handle<'_, /* ... */>),
    {
        let (a, b) = self.inner.split_mut();
        // ...derive context from a-side state...
        let item = a.materialize();
        f(item, &mut Handle::new(b, /* ... */));
    }
}
```

This is a different pattern from the coordinating composite the rest of §4 describes. A coordinating composite owns routing between sub-drivers in its method bodies (the §4 opening's `Top::on_external_trigger → sub_a.on_triggered → sub_b.on_data`). A borrow-splitting sub-composite owns no routing — the dispatcher closure does. `split_mut` is the reason the type exists; forwarder methods are there so non-hot call sites stay compact.

Reach for it when §7.4's cached context and reply path naturally split into two halves of state. Don't reach for it when the borrow conflict can be reshaped away — returning owned data instead of a borrow, or restructuring the trait method, often eliminates the need. Over-applying gives sub-composites that don't earn their weight.

---

## 5. Interfaces and providers

A driver doesn't talk to hardware directly. It talks to one or more **interfaces** — Rust traits that describe what the driver needs done — and is generic over the concrete type that implements each interface. The interfaces are defined alongside the drivers; the implementations (called **providers**) live one layer below.

### 5.1 Drivers own their interfaces

The interface is a contract *written by the consumer*. It expresses what the driver wants done — set this output low, give me the current tick count, apply this trim step to the clock — not what the peripheral exposes. This direction matters:

- **Driver-shaped methods, not peripheral-shaped.** An interface to "set a digital output" reads `fn set(&mut self, level: Level)`. The implementor is already bound to one specific output; the driver issues a command. Compare to a peripheral-shaped equivalent `fn set_level(pin: Pin, level: Level)` where the driver has to thread a pin identifier on every call and import a chip-specific `Pin` type just to compile. The driver-shaped form lets the driver stay free of chip types entirely.
- **Narrow per-driver granularity.** Each interface declares only what one driver actually calls. A clock driver's "apply trim" interface doesn't expose the rest of the clock-controller registers. Reading the trait bounds on a driver tells you exactly what hardware operations it depends on.
- **Domain types at the interface boundary.** A `Level` enum (high/low) is a domain concept — it has meaning at the driver layer and above. It lives with the interface that uses it, not with the HAL. The HAL itself can take primitives (a `bool`) and the provider does the one-line translation.

The reverse direction — HAL exposes traits, drivers consume them — is the conventional layered-HAL design. It works, but the trait surface ends up shaped by what the HAL chose to offer rather than by what the driver needs, which tends to produce wider traits than the consumer actually uses.

### 5.2 Providers implement interfaces over HAL or fakes

The provider is the layer where the choice of *which peripheral plays this role* lives, plus the bridge from the driver's trait surface to HAL primitives. Each interface has at least two implementations:

- **Production provider.** A small struct that holds whatever per-instance state is needed (often zero-sized for singleton peripherals) and dispatches each interface method into HAL functions. Lives in `provider/`. Its thinness is the point, not redundancy — the production driver pays no size or speed cost (monomorphization plus `#[inline]` collapses the dispatch to a direct HAL call), and the file is the unambiguous home for the "we picked this peripheral for this role" decision.
- **Fake provider.** A test-only implementation that records calls into a log instead of touching hardware. Used by host-side unit tests. Lives alongside the production provider, gated `cfg(test)`.

Both satisfy the same interface, so they're interchangeable from the driver's perspective. Production code constructs the driver with the real provider; tests construct it with the fake.

### 5.3 Naming

The provider layer is **role-shaped**: file names and type names both reflect the trait role, not the peripheral. This is the convention's single most useful tell — a directory listing of `provider/` reads as the catalog of driver-trait roles the firmware fulfills.

- **Interface (trait)** — a capability-flavored name (`Monotonic`, `DigitalOut`, `DmaRing`, `UsartBaud`, `ClockTrim`). Reads as something the driver wants done. Defined in the drivers tree (e.g., `drivers/traits.rs`).
- **Provider module file** — `snake_case` of the trait name: `provider/monotonic.rs`, `provider/dma_ring.rs`. One file per role.
- **Provider type** — `PascalCase` of the trait name, same word: `pub struct Monotonic;` inside `provider/monotonic.rs`. The fully-qualified path reads as a sentence: `provider::monotonic::Monotonic` says *"the Monotonic provider."* The peripheral choice (SysTick, TIM6, …) is the file's *content*, not its name.
- **Fake** — `pub struct Fake;` in the same file under `#[cfg(test)]`. Resolved by path: `provider::monotonic::Fake` parses as *"the test fake for the Monotonic role."*

The trait and the provider type share a name, so the provider file imports the trait by fully-qualified path in the `impl` line rather than via `use`:

```rust
// provider/monotonic.rs
pub struct Monotonic;

impl Monotonic {
    pub fn init() { /* claim and configure the peripheral */ }
}

impl crate::drivers::traits::Monotonic for Monotonic {
    const TICKS_PER_US: u32 = /* ... */;
    fn ticks(&self) -> u32 { /* HAL call */ }
}

#[cfg(test)]
pub struct Fake { /* recorded calls */ }
#[cfg(test)]
impl crate::drivers::traits::Monotonic for Fake { /* ... */ }
```

The verbosity stays in this one line per provider file; every other call site reads `provider::monotonic::Monotonic` or imports the trait normally.

HAL is the orthogonal axis: HAL files are peripheral-IP-shaped (`hal/systick.rs`, `hal/timer/v3.rs`, `hal/usart/v1.rs`). A provider file imports HAL primitives by peripheral and calls them — the role-to-peripheral mapping lives in the provider, the peripheral primitives live in HAL.

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

### 5.5 Provider categories and init responsibility

Providers come in two flavors:

- **Driver providers** implement a driver-trait and are consumed by generic drivers. Examples: `provider::monotonic::Monotonic`, `provider::dma_ring::DmaRing`. The driver holds one as a type parameter and calls trait methods.
- **System providers** are init-only bundles called once from the orchestrator. They expose no trait and are not consumed by drivers — they configure chip-wide preconditions on which everything else depends. Examples: `provider::clocks::Clocks` (clock tree setup), `provider::pins::Pins` (pin modes per board wiring), `provider::interrupts::Interrupts` (interrupt-controller base config).

Both follow the same role-shaped naming convention (file = role, type = role).

**Init responsibility.** A driver provider owns *everything* required to make its claimed peripheral usable: the **peripheral clock-gate enable** (RCC ENR bit, or the chip's equivalent), peripheral mode/baud/IRQ-config writes, and any one-shot register setup. System providers own chip-wide preconditions (clock tree, pin modes, interrupt-controller base).

The cohesion rule:

> The provider that decides "I'm using peripheral X for this role" is the one that enables X's clock gate and configures X.

This keeps the peripheral allocation decision and its setup code in the same file. Multi-chip swap (§5.6) is then one file: the replacement provider enables whichever peripheral *it* picks. Conversely, no init scattered across bringup or other files reaches into a peripheral that "belongs" to a provider.

IRQ enable is the one exception — it's deferred to a separate facade phase so handlers are installed before vectors can fire. See §9.2 for the contract.

### 5.6 Multi-chip variants

When a codebase supports more than one chip variant, the provider layer is the chip-variant boundary. Driver code stays variant-agnostic — its trait bounds don't change — and the provider type stays role-named at the import surface (`provider::monotonic::Monotonic`). Concrete chip-specific implementations hide inside.

The convention extends to a sub-module per role:

```
provider/
  monotonic.rs            ← variant dispatcher (or monotonic/mod.rs)
  monotonic/
    v00x.rs               ← variant A impl
    v30x.rs               ← variant B impl
```

The dispatcher cfg-routes the re-export:

```rust
// provider/monotonic.rs
#[cfg(feature = "chip-v00x")] mod v00x;
#[cfg(feature = "chip-v00x")] pub use v00x::Monotonic;

#[cfg(feature = "chip-v30x")] mod v30x;
#[cfg(feature = "chip-v30x")] pub use v30x::Monotonic;
```

Gate on whatever chip feature flags the project already uses (typically the same flags exposed by the chip-support crate). A single-chip project uses the flat form (`provider/monotonic.rs`); a multi-chip project sub-modules. **Drivers, the registry, and the `Provider` facade are unchanged either way** — the variant axis lives entirely inside each role module.

Two chips with the same IP for a peripheral can share a HAL file (`hal/timer/v3.rs`) but still differ at the provider — e.g., one chip picks SysTick for `Monotonic`, the other picks a timer. The provider encodes the choice; HAL provides the IP-shaped primitives.

System providers follow the same sub-module pattern when their setup is chip-specific (`provider/clocks/v00x.rs`, `provider/clocks/v30x.rs`). Clock tree, pin alternates, and interrupt-controller base config typically vary more across chips than driver-trait providers do, so multi-chip projects often see system providers sub-moduled first.

The variant-routing pattern extends beyond providers: `hal/types.rs` (chip-family enum bodies) and `cfg/board_wiring.rs` (board-wiring schema fields) follow the same cfg-routing convention. §2.5 covers the pattern across all three layers.

When variant counts grow large enough that the per-chip-family cfg gates become unwieldy — or when chips diverge enough that providers don't usefully share a file — the next step is to split chip families into separate crates. §2.4 describes that packaging.

---

## 6. Unit testing via provider swap

The interface/provider split exists primarily for one reason: **driver logic is unit-testable on the host without an MCU, without a simulator, and without writing integration-style harnesses.** Tests construct a driver with fake providers, drive it through a scenario, and assert on the recorded calls and the driver's resulting state.

### 6.1 What this looks like

A leaf-driver test:

```rust
#[test]
fn arming_drives_output_low_then_high() {
    let fake = FakeDigitalOut::default();
    let mut driver = SomeDriver::new(fake);
    driver.arm();
    assert_eq!(driver.provider().ops(), &[Op::Set(Level::Low), Op::Set(Level::High)]);
}
```

The fake records every method call in order. The test asserts on the call sequence, the call arguments, or the resulting state of the driver — whichever the driver under test is responsible for.

For composite drivers, a single fake-host struct hands out per-resource fake providers that share an underlying log. The composite test constructs one host, asks it for each sub-driver's fake, and inspects the combined log:

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
- **Provider correctness.** The production provider's translation from interface call to HAL function. Tested at the provider layer (small surface, mostly mechanical) or by running the firmware on hardware.
- **ISR-priority races.** Any concurrency between ISRs sharing a driver. Out of scope for host tests; needs hardware or a model checker.

The split is intentional. Driver-layer host tests catch the bugs that show up in FSM and policy logic — by far the largest class of bugs in firmware of this shape. Hardware-shaped bugs are caught by hardware-shaped tests; the host-test layer isn't trying to replace them.

### 6.4 Fakes are providers too

Fake providers live in the same `provider/` layer as production providers, gated `cfg(test)`. They satisfy the same interfaces. From the driver's perspective there's no distinction; from the layering's perspective, **the provider layer is the testability seam by design**. The fakes aren't a separate "mock layer" bolted on — they're a second flavor of provider, parallel to the production one. That's why the interface/provider split is in the architecture rather than added as an afterthought when tests get hard to write.

---

## 7. Services

Services are **adapters** between protocol-side traits (defined in a chip-agnostic core) and driver methods. They are thin:

```rust
// Protocol-side trait, defined in the chip-agnostic core:
pub trait SomeBus {
    fn send(&mut self, payload: Payload);
    fn poll(&mut self) -> Option<Request<'_>>;
}

// Chip-specific service:
pub struct SomeBusAdapter;

impl SomeBus for SomeBusAdapter {
    fn send(&mut self, payload: Payload) {
        let _ = unsafe { Drivers::top() }.send(|buf| serialize(payload, buf));
    }

    fn poll(&mut self) -> Option<Request<'_>> {
        unsafe { Drivers::top() }.poll()
    }
}
```

(`send` carries data, not scheduling — see §7.4. The driver derives wire timing from the request it polled. `poll` returns decoded protocol-domain values, not raw bytes — anything wire-shaped stays in the driver.)

That's the whole service. Each method:

1. Acquires access to the driver.
2. Translates protocol-level arguments to driver method calls.
3. Translates the driver result back to the protocol-level return type.

(Services and providers both adapt between two type universes — that's the family resemblance with hexagonal-architecture adapters. The two roles don't conflict: §5 **providers** bridge driver interfaces to HAL; **services** here bridge protocol traits to driver methods. The struct name `SomeBusAdapter` is convention for service implementations; it's a name choice, not a layer identifier.)

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

### 7.4 Data-centric trait surfaces

Service traits should be **data-centric** — methods carry *what* to do, not *how to position it on the wire*. Scheduling, slot positioning, retry timing, framing details, and any other "where/when on the wire" concern belongs inside the driver whenever the driver has the cached request state to derive it.

The driver already holds the request that drives the response (the polled token, the parsed command, the wire-end tick). Extending that cache to derive positioning is natural and keeps the service trait protocol-shape pure. The dispatcher then reads as straight protocol logic — every method call is "reply with this Status" or "reply with this slot," and nothing about timing or framing leaks into the protocol layer.

A trait method *should* take a scheduling parameter when the protocol genuinely lets the consumer decide — a retry policy chosen by the caller, a deadline that originates above the driver, a deferred-send queue the protocol layer manages. When the protocol determines the schedule from the *received request* — slot positioning, response delay, chain-CRC anchor — the driver derives it from cached state and the trait stays clean.

Symptom of the wrong split: a `Schedule` struct (or analogous record of "where on the wire") threaded through every send method. If the driver can compute that struct from state it already keeps for other reasons, the parameter is leaking driver knowledge into the service interface.

When the cached request and reply path touch disjoint parts of driver state, §4.3 covers the implementation seam.

---

## 8. Hardware demultiplexing

The IRQ vector binding file is structurally minimal: each ISR is a small dispatcher that reads the hardware status register, classifies into logical events, and calls into one or more drivers.

```rust
// runtime/isr.rs — vector binding
pub fn on_some_peripheral() {
    let status = peripheral_status();
    let driver = unsafe { Drivers::top() };

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
    unsafe { Drivers::top() }.on_deadline();
}
```

In hot-path cases where the dispatcher's overhead matters, the ISR can call the relevant sub-driver directly instead of going through the composite. This is a deliberate exception to the composition rule and should be commented as such — it's an optimization, not the default shape.

---

## 9. Registries and facades

Driver types are pure (no statics) and provider types are decision-free at the import site (peripheral choice is hidden inside the file). Production firmware still needs two pieces of orchestration: a place to hold installed driver *instances* and reach them from ISRs (§9.1, the **driver registry**), and a single entry point for hardware setup + deferred IRQ enable (§9.4, the **provider facade**). §9.5 states the HAL access rule that keeps both layers honest.

### 9.1 The driver registry

Driver types are pure: they own state, expose methods, and know nothing about being singletons. Production firmware still needs a place to hold installed driver *instances* and a way to reach them from ISRs and main-loop code. That place is the **registry** — a single facade type that owns one static cell per instance and exposes typed accessors.

```rust
use core::cell::SyncUnsafeCell;

struct Cells {
    top: SyncUnsafeCell<Option<Top>>,
    aux: SyncUnsafeCell<Option<Aux>>,
}

static CELLS: Cells = Cells {
    top: SyncUnsafeCell::new(None),
    aux: SyncUnsafeCell::new(None),
};

pub struct Drivers;

impl Drivers {
    /// SAFETY: bringup-only, pre-IRQ; sole writer. Must be called exactly once.
    pub unsafe fn install(/* construction args */) {
        // SAFETY: see fn doc.
        let top = unsafe { &mut *CELLS.top.get() };
        debug_assert!(top.is_none(), "Drivers: top already installed");
        *top = Some(Top::new(/* args */));

        // SAFETY: see fn doc.
        let aux = unsafe { &mut *CELLS.aux.get() };
        debug_assert!(aux.is_none(), "Drivers: aux already installed");
        *aux = Some(Aux::new(/* args */));
    }

    /// SAFETY: bringup installs `top` before any ISR runs; access follows
    /// the driver's IRQ-priority contract documented at the call site.
    #[inline(always)]
    pub unsafe fn top() -> &'static mut Top {
        // SAFETY: see fn doc.
        let cell = unsafe { &mut *CELLS.top.get() };
        debug_assert!(cell.is_some(), "Drivers::top() before install");
        // SAFETY: bringup ensures Some before any ISR fires.
        unsafe { cell.as_mut().unwrap_unchecked() }
    }

    // ...one accessor per instance.
}
```

The shape has three properties worth naming:

- **Driver types are decoupled from singleton lifecycle.** A driver named for its *mechanism* — a generic LED controller, a generic counter — can be instantiated as many times as the firmware needs. Adding a second instance is one cell + one accessor; the driver type doesn't change. (Before this split, drivers that hosted their own static cell tended to absorb their usage into the type name — "status LED" instead of "LED" — because the type *was* the singleton.)
- **Each instance has its own cell, not a shared `&mut Drivers`.** This preserves borrow independence: an ISR mutating one driver and the main loop mutating another don't pass through a shared mutable reference to the registry, so there's no aliasing UB even at the conceptual level. The registry struct itself is a ZST; the cells are the storage.
- **Tests bypass the registry entirely.** Unit tests construct driver types directly with fake providers — no `install`, no `get`, no `unsafe`. The registry is a production-only artifact for singleton management; the testability seam from §5–§6 is unaffected by it.

The two-layer `Option<Driver>` inside each cell keeps the driver type itself honest: `Driver::new(...)` never returns a half-constructed object. The "is it installed?" question is asked once per access, at the registry accessor, `debug_assert`-checked in dev / `unwrap_unchecked` in release (sound because bringup must run before any IRQ).

### 9.2 Access discipline

The discipline beyond install depends on the runtime model. For a no-RTOS firmware where ISRs share a single priority level and don't preempt each other (or use cooperative scheduling), the rules are:

- **From an ISR at the shared priority**: `Drivers::foo()` is safe because no other code at the same priority can preempt.
- **From the main loop**: `Drivers::foo()` is safe only after disabling the relevant ISRs around the access, OR if the access only touches state the ISRs never mutate (read-only accessors are usually safe without IRQ masking; mutating commands typically need a masked critical section).

Every `unsafe { Drivers::foo() }` site is making a contract claim. The discipline is:

1. State the contract once, in the accessor's `SAFETY:` doc comment.
2. Site-level comments only when the call site deviates from the default discipline.
3. Restrict all mutation to driver methods (no raw field access from outside).

If the project uses RTIC, an async executor, or another framework that provides safer ownership patterns, those should be preferred. The registry + `SyncUnsafeCell<Option<Driver>>` cells is the floor — it works without runtime support but requires discipline.

### 9.3 Cross-cell coordination

Most coordination happens through the composite-driver routing in §4. The remaining cases are where two genuinely independent top-level drivers need to exchange a value — and per §4.2, that should be rare. When it does happen, the channel is one of:

- An atomic flag/counter, when the value is a scalar.
- A small lock-free queue or `SyncUnsafeCell<Option<T>>` with seqlock semantics, when the value is a struct.

These cross-cell channels live alongside the registry's instance cells but aren't *of* it — the registry holds driver instances, not communication channels. If you find yourself adding more than one or two channels, the architecture is telling you to reconsider whether the two drivers should be composed.

### 9.4 The provider facade

The provider facade mirrors the driver registry, but for *hardware setup* rather than *runtime access*. Bringup goes through a single entry point that orchestrates every provider's init in a fixed order, then runs `Drivers::install` to populate cells, then enables IRQs last.

```rust
pub struct Provider;

impl Provider {
    /// SAFETY: bringup-only, called exactly once before Drivers::install.
    /// Sets up hardware; does NOT enable IRQs.
    pub unsafe fn init(/* wiring, defaults */) {
        // System providers — chip-wide preconditions
        clocks::Clocks::init();
        pins::Pins::init(/* wiring */);
        interrupts::Interrupts::init();

        // Driver providers — peripheral clock-gate enable + mode/config writes
        monotonic::Monotonic::init();
        dma_ring::DmaRing::init();
        usart_baud::UsartBaud::init(/* baud */);
        clock_trim::ClockTrim::init();
        // Multi-instance providers (e.g., digital_out per pin) are constructed
        // inside Drivers::install, not here.
    }

    /// SAFETY: call AFTER Drivers::install. Cells must be populated before
    /// any vector unmasks.
    pub unsafe fn enable_irqs() {
        dma_ring::DmaRing::enable_irq();
        usart_baud::UsartBaud::enable_irq();
        // …one call per provider that fires an IRQ
    }
}
```

Bringup collapses to three lines with a structural contract:

```rust
pub unsafe fn bringup(/* wiring, defaults */) {
    Provider::init(/* … */);          // hardware powered + configured
    Drivers::install(/* … */);        // handlers ready
    Provider::enable_irqs();          // IRQs hot
}
```

The contract: *hardware powered + configured → handlers ready → IRQs hot.* Hard to get the order wrong.

Three intentional asymmetries vs the driver registry:

- **No cells, no storage.** Provider types are zero-sized; nothing to store. `Provider` is a pure namespace + init entry point.
- **No accessors.** `Drivers::install` reaches directly into provider types (e.g., `provider::monotonic::Monotonic`) to construct driver instances. Routing those through `Provider::monotonic()` would just rename the same unit struct.
- **`init` vs `install` verb.** `install` implies storage (cells); `init` implies setup. Layer names stay symmetric (`Provider` ↔ `Drivers`); method verbs differ because the work differs.

**Init-order discipline.** System providers run first (clock tree must exist before any peripheral is configured; pin modes must be set before any peripheral that drives those pins comes up). Driver providers follow. Each driver provider's `::init()` performs the clock-gate enable + peripheral configuration writes; IRQs stay masked. After `Drivers::install` populates cells, `Provider::enable_irqs` unmasks vectors.

### 9.5 Chip-agnosticity boundary

Drivers stay chip-agnostic because they have no path to chip-specific types. The enforcement model differs by packaging:

**Single-crate firmware.** Chip-agnosticity is a code-review convention: `hal::*` imports are confined to the `provider/` layer; a `use crate::hal::` outside `provider/*` is a layering violation. The rule is mechanically grep-able and amenable to lint enforcement, but its weight rests on review discipline.

**Multi-crate firmware (recommended for long-lived projects).** Chip-agnosticity is a `cargo build` invariant. The driver crate (`<project>-drivers`) has no dependency on the chip-family crate, so it physically cannot import HAL. Within the chip-family orchestration crate, HAL is just an internal module — providers, runtime, and ISR bodies all consume it as needed. There is no within-crate HAL access rule; the boundary that matters is the crate edge.

The multi-crate form is preferred where chip-agnosticity actually matters because the invariant is structural rather than social. Review still catches local quality issues (provider ergonomics, leaky type signatures), but the foundational property — drivers cannot see HAL — holds without anyone having to check.

Three downstream properties hold under either form:

- **Drivers stay generic** over their trait bounds, so unit tests substitute `Fake` providers without touching driver code.
- **Chip variants land in providers** (§5.6), in `cfg/` (board-wiring schema), and in `hal/types.rs` (chip-family enums) — not in driver bodies. §2.5 covers the variant-routing pattern across all three layers.
- **Bringup carries no chip knowledge of its own.** Its body names which providers exist and the order they init in — not which registers they touch.

**ISR vector bodies** (§8) dispatch through `Drivers::*` to driver methods. In the multi-crate form, ISR bodies live in `runtime/isr.rs` inside the orchestration crate and import HAL directly to ack hardware flags (read NDTR, clear HT/TC, sample EXTI pin) — unrestricted within the orchestration crate. In the single-crate form, ISR HAL access is typically routed through a provider method to keep the within-crate rule local. The ISR's job — dispatch to a driver — is the same either way; only the ack code's home differs.

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

    pub fn send_reply<F>(&mut self, writer: F) -> Result<(), BusError>
    where
        F: FnOnce(&mut [u8]) -> usize,
    {
        // The Bus already cached the request that produced this reply
        // (`self.rx.last_request()`), so it derives wire-end tick + slot
        // offset + chain-CRC anchor internally. The caller hands data; the
        // bus places it. See §7.4 for the underlying principle.
        self.tx.arm(writer).map_err(BusError::Tx)
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

### 10.2 Mapping to providers

Each driver-trait the `Bus` sub-drivers consume has a provider file under `provider/`:

```
provider/
  monotonic.rs      Monotonic    — tick source for fire scheduling
  usart_baud.rs     UsartBaud    — claims the UART and sets baud
  dma_ring.rs       DmaRing      — claims a DMA channel for the RX byte ring
  digital_out.rs    DigitalOut   — drives the TX-direction GPIO (multi-instance)
  clock_trim.rs     ClockTrim    — applies clock-rate trim steps
  clocks.rs         Clocks       — (system) clock tree
  pins.rs           Pins         — (system) pin modes per board wiring
  interrupts.rs     Interrupts   — (system) IRQ controller base
```

The orchestrator collapses bringup to three lines:

```rust
pub unsafe fn bringup(wiring: &Wiring, defaults: &Defaults) {
    Provider::init(wiring, defaults);   // hardware powered + configured
    Drivers::install(wiring, defaults); // handlers ready
    Provider::enable_irqs();            // IRQs hot
}
```

If the example were ported to a second chip variant where the UART is on a different peripheral block, only `provider/usart_baud.rs` (and possibly its `vNNx.rs` sub-files per §5.6) changes. `drivers/bus/*` is untouched.

### 10.3 Mapping to ISRs

```rust
// runtime/isr.rs
pub fn on_uart() {
    let status = uart_status();
    // SAFETY: see Drivers::bus.
    let bus = unsafe { Drivers::bus() };
    if status.has_rx_error() { bus.on_uart_rx_error(status.rx_flags()); }
    if status.idle()         { bus.on_uart_idle(); }
    if status.tc()           { bus.on_uart_tc_completed(); }
}

pub fn on_timer_compare() {
    unsafe { Drivers::bus() }.on_timer_match();
}
```

Two ISRs. Combined: about a dozen lines. Each does one thing and reads as one thing.

### 10.4 Mapping to a service

```rust
// services/bus.rs
pub struct BusAdapter;

impl ProtocolBus for BusAdapter {
    fn rx_window(&mut self) -> Option<&[u8]> {
        unsafe { Drivers::bus() }.rx_window()
    }

    fn send(&mut self, packet: Packet<'_>, schedule: Schedule) {
        let _ = unsafe { Drivers::bus() }.arm_reply(
            |buf| serialize(packet, buf),
            schedule,
        );
    }
}
```

The protocol layer's dispatcher consumes `ProtocolBus`, knowing nothing about UARTs, timers, or GPIOs. The service is purely type translation. If the same protocol runs on a different chip with different peripherals, only the providers (and possibly the driver, if the hardware concerns diverge enough) change — the protocol stack is identical.

---

## 11. Related patterns and origins

This convention is a synthesis of established patterns rather than a novel design. The components map roughly as:

- **Command Query Responsibility Segregation (CQRS).** The mutating-command / read-only-query split. Commands here are pure methods rather than reified message objects, but the read/write separation is the same. Long established in distributed systems and DDD.
- **Domain events.** `on_*` methods are the *intake* of domain events; their return values are the *output*. Mealy-machine outputs from event-sourced systems map directly.
- **Hexagonal architecture / Ports and Adapters.** Drivers are the domain core; the interface/provider split in §5 is a direct application of Cockburn's ports-and-adapters at the module level. The "interface" in this doc corresponds to a "port" in hexagonal vocabulary; what hexagonal calls an "adapter" this doc calls a **provider** — the rename emphasizes the role-shaped naming convention (the file lives at the role's name, the type lives at the role's name) and avoids the term collision with services. Services play the analogous role one layer up, adapting protocol traits to driver methods.
- **DDD aggregates.** A composite driver is an aggregate root; sub-drivers are entities reachable only through the aggregate. The "no sibling access" rule is the aggregate-boundary rule.
- **Active Object** (Samek, *Practical Statecharts in C/C++*). The most embedded-specific analog: each active object is a state machine with an event queue, composed hierarchically, with no direct peer access. Used heavily in safety-critical embedded.
- **Four-layer chip stack.** The services / drivers / providers / HAL split is more structured than the typical drivers-on-top-of-HAL embedded layering. The provider layer is the explicit dependency-inversion seam: drivers depend on interfaces they themselves define, not on HAL, which is what makes driver logic testable without hardware *and* what makes drivers chip-agnostic by construction (chip choice lives in providers, not drivers).

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
- **Generic-parameter propagation.** Drivers carry one type parameter per interface they consume; composites carry the union. A `pub type` alias hides the verbosity at production call sites, but compile errors and IDE hovers show the long form. The cost is paid in compile-time verbosity, not runtime: production providers are typically zero-sized, so monomorphization plus `#[inline]` makes the provider dispatch identical to a direct HAL call.
- **One more layer of files.** The provider layer adds one file per driver-trait role (plus the fake alongside, cfg(test) in the same file). The cost is fixed — providers don't grow per driver — and it buys the testability seam, but it's a real addition to the file tree.

---

## 13. Summary

Six rules:

1. **Drivers own hardware state.** Expose pure methods: `on_*` for hardware events (named for the logical event, not the peripheral that delivered it), descriptive names for commands, accessors for stable observations. Programmer-error invariants are `debug_assert!`s; runtime failures return `Result`.
2. **Composite drivers route between sub-drivers.** Sub-drivers are reachable only through the parent.
3. **Top-level drivers do not communicate with each other.** If two would need to, compose them into a higher driver.
4. **Services adapt protocol traits to drivers.** Each service binds to exactly one top-level driver. Services are thin, and their trait surfaces stay **data-centric** (§7.4) — driver-derivable scheduling and wire-positioning info stays inside the driver.
5. **Cross-domain orchestration lives at the protocol layer**, never in drivers.
6. **Drivers are generic over driver-defined interfaces; providers implement them.** Production providers wrap HAL calls; fake providers record calls for tests. Driver logic is unit-testable on the host without hardware, and drivers are chip-agnostic by construction (chip choice lives in providers).

Four layers, top to bottom:

- **Services** — adapt protocol traits to driver method calls.
- **Drivers** — hierarchical pure types with pure-method APIs, generic over their interfaces. No statics, no `install` / `get` on the type. Chip-agnostic.
- **Providers** — implement driver interfaces over HAL primitives (production) or recording stubs (test); role-shaped file and type names; own per-peripheral init including clock-gate enable.
- **HAL** — peripheral-IP-shaped register access and resource identifiers, cfg-gated when IP version differs across chips.

Plus three orthogonal facades:

- **Driver registry** (§9.1) — owns one static cell per driver *instance* and exposes typed accessors. The only place that knows which type plays which singleton role.
- **Provider facade** (§9.4) — single entry point for hardware setup (`Provider::init`) and deferred IRQ enable (`Provider::enable_irqs`). Bringup is three lines: `Provider::init → Drivers::install → Provider::enable_irqs`.
- **IRQ vector binding** — hardware demux that routes interrupt sources to driver methods via the registry.

The result is firmware where reading any one file gives you a complete picture of one concern, the wire diagram between concerns lives in composite-driver method bodies, the protocol layer stays unaware of any specific peripheral, drivers are unaware of any specific chip, and driver logic is exercisable by host-side unit tests.

For projects with multi-chip support or quality goals that benefit from build-time enforcement of layering, the chip-family crate becomes an **orchestration crate** (§2.4) that composes HAL, providers, services, the driver registry, ISRs, and bringup into a running system. Driver state machines and their trait surface live in a pure library crate; chip-agnosticity becomes a `cargo build` invariant rather than a code-review convention. §2.5 covers the orchestration crate's internal module layout.

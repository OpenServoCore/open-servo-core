# ARCHITECTURE

This repo is building a **Smart Servo firmware stack** with a clean separation between:

- **Kernel**: control loop + safety policy + live state (single-writer, ISR/tick-owned)
- **Host plane**: a byte-addressable **Shadow Table** used by any future wire protocol (Dynamixel 2.0 first)
- **Device runtime**: ties kernel ↔ hardware together, schedules ticks, owns the shadow storage, and (later) hosts comms

The design goal is to make **Dynamixel (and other protocols)** “drop in” later without forcing kernel refactors.

---

## 1. Big rules (read this first)

### 1.1 Single-plane rule (register/control plane)

There is exactly **one** register/control plane:

> **Shadow Table bytes** are the *only* way to read/write “register-like” state.

- Future comms (DXL 2.0) will read/write byte ranges directly.
- The kernel does **not** expose register read/write ops.
- The HostOp queue is **not** a second register plane.

### 1.2 Single-writer rule (kernel invariant)

The kernel’s **live state** is mutated only by the **tick/ISR context**.

- Host / main loop may write the Shadow Table at any time (under `critical_section`)
- The kernel reads host-written bytes only at **safe boundaries** (`commit_shadow`)
- The kernel writes telemetry bytes only at **safe boundaries** (`publish_telemetry`)

This keeps control behavior deterministic and prevents “mid-step” config changes.

### 1.3 Atomics and portability

MCU atomics support varies:
- **Cortex-M3+**: Native hardware atomics
- **CH32V006**: No hardware atomics (uses critical-section emulation)

This project optimizes for the lowest common denominator:
- Shared memory between main loop and ISR uses **`critical_section`** discipline
- `heapless` uses `portable-atomic` which auto-detects and falls back to CS
- Explicit CS wrappers ensure consistent behavior across all targets

---

## 2. Crates and responsibilities

### 2.1 `open-servo-kernel-api`
**Pure API + contracts**. No hardware.

Contains:
- `shadow.rs`: Shadow Table types, views, dirty tracking, staging, and the kernel seam trait(s)
- `host_op.rs`: **sideband** commands only (no register IO)

This crate defines what *must* remain stable for later comms integration.

### 2.2 `open-servo-kernel`
**Concrete kernel implementation**.

Owns:
- Live state (mode, engaged, limits, gains, fault latch, integrators)
- Tick pipeline (fast/medium/slow control)
- `ShadowKernel` impl:
  - `commit_shadow(view)`: apply host-written bytes to live state
  - `publish_telemetry(view)`: serialize live telemetry into shadow bytes

Defines:
- `shadow_fields.rs`: the **schema** (offsets + sizes + encoding) for bytes

### 2.3 `open-servo-hw` (v2)
Hardware boundary traits and types: ADC samples, GPIO, timers, UART bus, motor driver IO, etc.

This crate is where we keep “what the kernel/runtime needs from the MCU” abstracted.

### 2.4 `open-servo-device`
The **runtime glue**.

Owns:
- `ShadowStorage`: the actual `[u8; N]` + dirty bits + staging buffer, plus concurrency discipline
- `Executor`: tick scheduling / boundaries and calling kernel methods at the right times
- (Later) comms RX/TX services, packet parsing, scheduling, etc.

### 2.5 Board crates (e.g. `open-servo-board-stm32f301`)
Concrete MCU + board wiring.

Owns:
- Pin setup, clocks, DMA, ISR plumbing
- Implementations of `open-servo-hw` traits
- Concrete UART direction-gating behavior (see §6)

---

## 3. Concurrency model (the real reason this doc exists)

There are two relevant contexts:

| Context | What it does | Allowed to mutate |
|---|---|---|
| **Main loop** | host/comms work, slow housekeeping | Shadow Table **via CS only** |
| **ISR / tick** | control loop, safety, motor command | **Kernel live state**; Shadow Table **only via KernelView** at safe boundaries |

### 3.1 Shadow Table access discipline

- **Host path** uses `ShadowStorage::host_with_view(...)` which wraps a `critical_section`
- **Kernel path** uses `ShadowStorage::kernel_with_view(...)` and is ISR-only by convention

This is the intended reason for the view types:

- `HostView`: can write CTRL/CONFIG, cannot write TELEM
- `KernelView`: can write TELEM, can read CTRL/CONFIG, and can clear dirty bits

### 3.2 Dirty bits semantics

Dirty bits mean exactly one thing:

> “Host wrote bytes in CTRL/CONFIG that have not been applied to live kernel state yet.”

- Host writes mark dirty (block-based)
- Kernel telemetry writes **do not** mark dirty
- Kernel clears dirty only for fields it successfully committed
- Validation errors must preserve dirty for the invalid field (host can fix and retry)

---

## 4. Shadow Table memory map

The Shadow Table is a fixed-size byte array (`N`, typically 512).

### 4.1 Regions (stable ABI)

Region boundaries are part of the ABI and live in `kernel-api::shadow::layout`:

- **TELEM**: host-read-only; kernel writes
- **CTRL**: host read/write; kernel reads/applies
- **CONFIG**: host read/write; kernel reads/applies

Field offsets inside these regions are **kernel-defined** in `open-servo-kernel/src/shadow_fields.rs`.

### 4.2 Staging and ACTION (future Dynamixel compatibility)

DXL 2.0 has `REG_WRITE` + `ACTION`. We support the concept at the shadow level:

- Host can stage one or more writes into a bounded staging buffer
- `action()` applies staged writes atomically to the Shadow Table (then marks dirty)
- Kernel will see dirty bytes and apply at next commit boundary

This keeps the kernel ignorant of protocol details while supporting transactional writes.

---

## 5. Tick model and data flow

The system is structured around three conceptual tick rates (exact scheduling is board/runtime-defined):

- **Fast**: hard real-time control math + immediate safety gating → `MotorCommand`
- **Medium**: safe boundary for “bulk” work:
  - `publish_telemetry(view)`
  - `commit_shadow(view)` if dirty
- **Slow**: future persistence, calibration, self-test, background estimation

### 5.1 What happens on a medium boundary (important)

At the medium boundary, the runtime does:

1. kernel serializes live telemetry into TELEM bytes (`publish_telemetry`)
2. if CTRL/CONFIG is dirty, kernel applies it (`commit_shadow`)
3. kernel continues control with updated live state starting next cycle

This is the “transaction boundary” that makes the concurrency sane.

---

## 6. Hardware contract: half-duplex UART with RX gating

We standardize a **self-echo-free** half-duplex UART bus across boards.

Conceptually, this is “DE semantics”:

- “DE asserted” means “we are transmitting; do not receive our own bytes”
- Boards may implement this as **RX enable gating** internally; the kernel/runtime uses the DE mental model

Board-level requirements:

- TX line has an external **47K pull-up** (bus idles high across resets)
- RX pin uses **internal pull-up**
- DE/RX-enable pin uses **internal pull-down** (default listen)

Transmit sequence (board implementation detail, but conceptually required):
1. assert DE (stop RX / enable TX path)
2. transmit bytes
3. wait for **TC** (transmission complete), not just TXE
4. deassert DE (resume RX)

---

## 7. Sideband commands (HostOp)

HostOp exists for operations that are **procedural**, not “register access”.

Examples:
- soft reset (deferred)
- persist commit / backup / restore
- fault ack / clear-latch actions
- mode requests that are not modeled purely as bytes (optional policy choice)
- ping/diagnostics

**Rule of thumb:** if the operation needs ordering, deferral, or a “do this now” effect, it’s HostOp.
If it’s config/telemetry, it belongs in the Shadow Table.

---

## 8. Where “register meanings” live

`open-servo-device::shadow_storage` does **not** define meanings.
It only stores bytes and enforces access/dirty/staging rules.

Meanings are defined by:

- `open-servo-kernel/src/shadow_fields.rs` (offsets, lengths, encoding)
- `ServoKernel::commit_shadow()` (validation + apply-to-live behavior)
- `ServoKernel::publish_telemetry()` (serialization of live state)

This is intentional: the kernel is the authority on what’s configurable and what telemetry exists.

---

## 9. How to add a new field (the happy path)

### 9.1 Add a new CONTROL field (host-writable)

1. Add offset + size in `shadow_fields::ctrl`
2. Update `commit_shadow()`:
   - check dirty overlap
   - read bytes via `view.read_ctrl(...)`
   - validate
   - apply to live state
   - `view.clear_range_dirty(...)` only on success
3. Add a unit test that:
   - host writes via `HostView`
   - commit applies and clears dirty
   - invalid input preserves dirty

### 9.2 Add a new TELEMETRY field (kernel-written)

1. Add offset + size in `shadow_fields::telem`
2. Update `publish_telemetry()` to write with `view.write_telem(...)`
3. Add a unit test verifying LE encoding and offsets

---

## 10. Testing and “don’t break the board”

Minimum checks we keep running as we evolve:

- `cargo check -p open-servo-kernel-api`
- `cargo check -p open-servo-kernel`
- `cargo test  -p open-servo-kernel`
- `cargo check -p open-servo-device`
- `cargo check -p open-servo-board-stm32f301 --no-default-features`

Key invariants worth unit tests:
- invalid values do **not** clear dirty bits
- telemetry writes do **not** set dirty bits
- staged writes apply atomically (when staging is enabled)

---

## 11. Roadmap notes (future comms / Dynamixel 2.0)

This document is intentionally **kernel-first**, but the architecture is designed so that adding
Dynamixel Protocol 2.0 later does **not** force a kernel refactor.

The key idea is still: **DXL is a client of the Shadow Table**.

### 11.1 What maps to the Shadow Table vs HostOp

**Register / control-table plane (Shadow Table):**
- DXL `READ` / `WRITE` → `shadow.host_read(offset,len)` / `shadow.host_write(offset,data)`
- DXL `SYNC_READ` / `BULK_READ` → repeated `host_read()` into one response frame
- DXL `SYNC_WRITE` / `BULK_WRITE` → repeated `host_write()` (possibly staged)
- DXL `REG_WRITE` + `ACTION` → `stage_write_range()` + `action()` (atomic apply of staged writes)

**Sideband / procedural plane (HostOp):**
- DXL `REBOOT`, `FACTORY_RESET`, `CLEAR`, “backup/restore”-style procedures (if implemented)
- Project-specific procedures that are not “just bytes”, e.g. *persist-to-flash*,
  *reset subsystems*, *fault ack policy*, etc.

Rule of thumb:
- If the host can express it as “write these bytes, then the kernel commits at a boundary” → Shadow Table.
- If it is “do a procedure with ordering/timing/safety semantics” → HostOp.

### 11.2 Sync Read is the timing footgun (Scheduled TX)

DXL `SYNC_READ` is not just “multiple reads” — it is a **slot-timed response schedule** on a shared
half-duplex bus.

Typical flow on the bus:
1. Master transmits a `SYNC_READ` request that names *a range* and *a list of IDs*.
2. Devices respond **one at a time** in ID order. Each device must start transmitting inside its
   allocated slot, with an additional programmable *return delay*.

To make this robust we will implement **Scheduled TX** in the device runtime:

- The main loop parses the request and determines **whether this device must respond**.
- It builds the response bytes by reading from the Shadow Table (telemetry/control bytes).
- It computes `tx_start_time_us = rx_end_time_us + slot_offset_us + return_delay_us`.
- It arms a hardware timer compare for `tx_start_time_us`.
- **Timer ISR MUST ONLY flip a flag** (e.g. `TX_DUE = true`) and disable its own interrupt.
  It must not touch kernel state or allocate buffers.
- The main loop/poller observes `TX_DUE` and starts TX:
  - set bus direction (see UART contract below)
  - start UART TX DMA (or interrupt-driven TX)
  - wait for UART **TC** (transmission complete), then return to listen mode

This preserves the single-writer kernel invariant and gives repeatable timing.

> Design note: for initial versions, it is OK if the poller introduces a small jitter (µs-scale)
> as long as we stay within DXL timing tolerances and do not collide with other IDs. The timer compare
> ISR exists so “when to start” is accurate even if the main loop is busy.

### 11.3 Return Delay Time and Status Return Level

DXL has control-table fields that directly affect comms behavior. Our plan is:

- **Status Return Level**: handled entirely in comms. It decides *whether to emit a Status Packet* for
  an instruction. (The Shadow Table can store the configured value; comms reads it.)
- **Return Delay Time**: used when computing `tx_start_time_us` for any response, especially `SYNC_READ`.

These are “comms policy knobs” but live nicely as bytes in Shadow Table (config region), because they
are still host-configurable values.

### 11.4 Broadcast semantics

DXL broadcast ID (typically 0xFE) means:
- Apply writes (including staged writes + action) **but do not respond**.

This is purely a comms-layer decision. The kernel doesn’t need special cases; it just commits shadow
changes at boundaries.

### 11.5 CRC-16 and acceleration

DXL 2.0 uses CRC-16 for packet integrity. CRC calculation belongs in the comms layer.
If the MCU offers a compatible hardware CRC mode, comms can use it; otherwise it uses a software CRC.
This choice should not leak into kernel APIs.

### 11.6 Why the kernel seam is already “DXL-ready”

Because comms will only need:
- byte-range read/write into Shadow Table
- staged write + action
- a small set of HostOp sideband procedures

Everything else (slot timing, packet stuff/unstuff, CRC, response policy) lives in comms + device.

## 12. Decisions log and future work (recorded from design chats)

This section exists so we don’t later ask “what did we decide last time?”. It mixes **decisions**,
**contracts**, and **planned-but-not-implemented** work. Update it whenever we make a new architectural
decision.

### 12.1 Hardware / bus decisions

**Echo-free half duplex (required)**
- All boards must be **self-echo free** by hardware design: RX is gated/disabled during TX so the MCU
  does not see its own bytes.
- We model this as **DE semantics** in firmware even if some boards implement it as “RX enable”.
  (Firmware uses a single “direction” control concept; board hides the electrical details.)

**Dual tri-state buffer wiring (board-level contract)**
- Channel 1: `n1OE <- TX`, `1A <- GND`, `1Y -> DATA (+33–68Ω series on data line)`
- Channel 2: `n2OE <- DE/RX_EN`, `2A <- DATA`, `2Y -> RX`
- Power: `VCC=3.3V` with local decoupling to GND

**Pull configuration**
- DATA/TX line has external **47k pull-up** so the bus is high during MCU reset/boot.
- MCU RX pin uses **internal pull-up**.
- MCU DE/RX_EN pin uses **internal pull-down** so default is “listen”.

**Connector order**
- Standard connector pin order: **DATA / GND / POWER**.

> Status: hardware contract is a requirement; firmware APIs should reflect it (direction control + “wait for TC”).

### 12.2 Concurrency model decisions

**Single-writer kernel invariant (still true)**
- “Kernel live state” is mutated only by the kernel in tick/ISR context.
- Host/comms never calls into kernel internals to “set config”; it only writes shadow bytes.
- Any ISR used for timing (e.g., scheduled TX) must only flip a flag and must not mutate kernel state.

**Atomics support**
- Cortex-M3+ has native atomics; CH32V006 does not
- `heapless` and `portable-atomic` auto-detect and fall back to critical sections
- Explicit CS wrappers in `open-servo-device` ensure portable behavior
- Kernel developers should optimize for the lowest common denominator

> Status: implemented at the seam level (ShadowStorage host methods use CS; kernel methods are ISR-only).

### 12.3 Control-plane decisions

**Single-plane rule**
- The Shadow Table is the **only** register/control plane.
- HostOp is **sideband only** and must not become a “second register plane”.

> Status: implemented (RegRead/RegWrite removed from HostOp; typed regs vocabulary removed).

**Staging is supported (REG_WRITE/ACTION equivalent)**
- Staging buffer exists to support DXL `REG_WRITE` + `ACTION` semantics.
- Staging applies to shadow bytes, then kernel commits at the next safe boundary.

> Status: staging exists in the Shadow API; full DXL integration will use it.

**Shadow table sizing**
- Default Shadow Table size is **512 bytes** (fits DXL-ish control table + vendor extensions).
- Compile-time guard prevents exceeding dirty tracking capacity.

> Status: implemented (default + guard).

### 12.4 Planned protocol/comms work (not implemented yet)

**Dynamixel 2.0 support (planned)**
- Implement DXL instructions as a client of Shadow Table reads/writes:
  - `READ`/`WRITE`, `SYNC_READ`/`BULK_READ`, `SYNC_WRITE`/`BULK_WRITE`, `REG_WRITE`/`ACTION`,
    plus standard ID/broadcast semantics.
- Maintain DXL-specific policy knobs (Status Return Level, Return Delay Time) as shadow config bytes;
  comms reads them to decide response behavior and timing.

> Status: planned; architecture seam is intended to make this drop-in.

**Scheduled TX for Sync Read timing (planned)**
- Use hardware timer compare to set a precise “TX due” moment.
- Timer ISR flips a `TX_DUE` (or similar) flag only; main loop starts TX when it sees the flag.
- Compute schedule time as: `rx_end + slot_offset + return_delay`.

> Status: planned; documented in §11.2.

**Zero-copy / low-copy packet handling (planned)**
- Avoid unnecessary copying for `WRITE` ranges by parsing into a bounded buffer and passing slices.
- A buffer pool (fixed N buffers) is an acceptable approach if needed for DMA/parsing ergonomics.

> Status: planned; revisit when comms is implemented.

**CRC-16 policy (planned)**
- CRC belongs in the comms layer.
- If hardware CRC exists and matches the DXL CRC-16 polynomial/config, comms may use it; otherwise
  use a software CRC (table or nibble).
- Kernel APIs should not expose CRC details.

> Status: planned; verify MCU CRC peripherals during comms implementation.

### 12.5 Planned kernel evolution (not implemented yet)

**Fast → Medium aggregation (planned)**
- Stage 1: medium tick may use “last fast sample” for decisions (simple).
- Later: add an accumulator so medium tick consumes aggregated/windowed fast data
  (e.g., avg/peak current, derived velocity) and migrate more safety/mode logic there.

> Status: planned (intentional phased rollout).

**Persistence and reset semantics (planned)**
- `PersistCommit` and `SoftReset` exist as HostOps.
- Actual flash persistence policy, factory reset scope, and reboot behavior will be implemented later
  and may be board-specific.

> Status: HostOp variants exist; behavior likely “Unsupported/Busy” until implemented.

### 12.6 Open questions to revisit (explicit)

- Exact DXL control-table compatibility level vs “DXL-inspired but vendor-extended”.
- Final list of required sideband procedures (beyond Persist/Reset/FaultAck).
- Whether comms should live inside `open-servo-device` initially or be split into `open-servo-comms`
  once multiple protocols/runtimes exist.

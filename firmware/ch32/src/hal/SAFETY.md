# HAL concurrency safety boundary

This crate wraps `ch32-metapac` registers. The metapac exposes safe `.modify()` /
`.write()` / `.read()` for every register, but **MMIO register access is outside
Rust's data-race model** — `.modify()` is a lw/op/sw on a peripheral address and
the compiler will not catch a race between two priority contexts touching the
same register. The lost-update bug class is real: an RMW racing against a
different-bit write from a higher-priority context silently drops the latter.

This module is the safety boundary. Code outside `hal/` should only call HAL
helpers and never touch the metapac directly. Each helper that needs more than
"single-context, safe" treatment carries an inline contract above its
definition — read those at the source, not from a parallel registry here.

## Concurrency model on V006 (QingKe V2A)

The chip has two PFIC priority classes:

- **HIGH** — ISRs at this class serialize against each other (no mutual
  preemption) and preempt both LOW ISRs and MAIN.
- **LOW** — ISRs preempt MAIN; they cannot preempt each other or HIGH.
- **MAIN** — the dispatcher / service loop; preempted by any ISR; cannot
  preempt them.

The chip has no atomic XOR / fetch-or on MMIO. The available atomic
primitives are hardware-provided per-register:

- GPIO `BSHR` / `BCR` — per-pin set/clear.
- DMA channel `IFCR` — per-flag write-1-to-clear.
- USART status bits — clear by the SR-then-DR read sequence.

Implication: any register written by ≥ 2 priority contexts is unsafe to RMW
unless **every bit one context might write is also written by the other** (so
the RMW is at worst self-healing), OR the access is wrapped in a critical
section, OR an atomic primitive replaces the RMW.

## Audit tiers

Every `.modify()` in this module sits in one of three tiers. New callers must
keep this invariant true.

### Tier 1 — single-context: safe

The register is only ever written by one priority context. RMW is fine; no
annotation required.

### Tier 2 — multi-context registers, RMW guarded by internal CS

The register is written from ≥ 2 priority contexts but only same-bit. The
helper wraps its `.modify()` in `critical_section::with` so the RMW is atomic
against any preempting writer to the same register. Callers don't need their
own CS.

CS cost on V006/V2A is ~5 cycles via `mstatus.MIE` (csrrci + csrw). Nested CS
(calling these from inside an ISR or another CS) is a no-op — the inner
`with` re-disables already-disabled interrupts.

### Tier 3 — must use atomic primitives, not `.modify()`

Registers where multiple priorities touch *different bits*. RMW here corrupts
state. Use the per-register atomic primitive instead:

- GPIO port-bit toggle → `BSHR` / `BCR`, never `OUTDR.modify`.
- DMA channel flag clear → `IFCR.write` (write-1-to-clear, hardware-atomic
  per bit), never `IFCR.modify`.

## Adding a new HAL helper — checklist

1. **Identify every priority context that calls it.** Trace from this crate
   outward; if the helper is `pub`, also check callers in other crates.
2. **Identify every other helper that writes the same register.** Their
   callers count too.
3. **If single-context** → Tier 1, no annotation needed.
4. **If multi-context same-bit** → Tier 2; wrap the RMW in
   `critical_section::with` and document the contract inline.
5. **If multi-context different-bit** → Tier 3 is required. Options:
   - Replace RMW with an atomic primitive (BSHR/BCR/IFCR/single-bit write).
   - Wrap the modify in `critical_section::with(|_| ...)`.
   - Hoist the write to a single context and route requests via an atomic
     "pending" flag (the apply-after-ISR pattern).

## Why we don't mark `.modify()` `unsafe` upstream

The race class is *any* non-atomic peripheral access reachable from multiple
priorities, not `.modify()` specifically — a bare `read()` then `write()` has
the same race. Marking `.modify()` `unsafe` at the metapac layer would force
`unsafe` on every legitimate single-writer use across the entire chiptool
ecosystem (every chip family, every embedded Rust HAL). The proper gate is
the HAL surface: this module is the chokepoint, and these tier rules are how
we enforce it.

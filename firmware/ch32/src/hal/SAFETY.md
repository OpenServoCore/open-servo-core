# HAL concurrency safety boundary

This crate wraps `ch32-metapac` registers. The metapac exposes safe `.modify()` /
`.write()` / `.read()` for every register, but **MMIO register access is outside
Rust's data-race model** — `.modify()` is a lw/op/sw on a peripheral address and
the compiler will not catch a race between two priority contexts touching the
same register. The lost-update bug class is real; see
`project_v006_tx_en_wedge_diagnosis` in the project memory log for a worked
example (a `GPIOC.OUTDR ^= mask` toggle from a LOW-priority ISR silently undid
a HIGH-priority ISR's `GPIOC.BCR` write, wedging TX_EN HIGH).

This module is the safety boundary. Code outside `hal/` should only call HAL
helpers and never touch the metapac directly. Each helper below is annotated
with its concurrency contract.

## Concurrency model on V006 (QingKe V2A)

- Only two PFIC priority classes: HIGH and LOW.
- HIGH ISRs (USART1, SysTick, DMA1_CH5) cannot preempt each other — they
  serialize. They can preempt MAIN and LOW ISRs.
- LOW ISRs (DMA1_CH1 = ADC) can preempt MAIN but not each other or HIGH.
- MAIN context (dispatcher poll loop, services) can be preempted by any ISR.
- MAIN cannot preempt ISRs.
- The chip has no atomic XOR / fetch-or on MMIO; only individual port-bit
  set/clear via GPIO `BSHR` / `BCR`, DMA channel `IFCR.set_tcif(n)`, and
  USART status bits cleared by SR-then-DR.

Implication: any register written by ≥ 2 priority contexts is unsafe to RMW
unless **every bit one context might write is also written by the other** (so
the RMW is at worst self-healing), OR the access is wrapped in a critical
section, OR an atomic primitive replaces the RMW.

## Audit tiers

Every `.modify()` in this module sits in one of three tiers. New callers must
keep this invariant true.

### Tier 1 — single-context: safe

The register is only ever written by one priority context (typically bringup
in MAIN before any ISR is unmasked, or one specific ISR thereafter). RMW is
fine.

- `adc/v00x.rs` — all sites are `configure()` / `enable()` in bringup.
- `afio/v00x.rs` — bringup remap setup.
- `flash/v00x.rs` — MAIN-only (flash writer + `Device::reboot`).
- `rcc/v00x.rs` — bringup; `apply_clock_trim_delta` (set_hsitrim) is invoked
  only from the USART1 TC ISR after an apply-after-TC swap.
- `timer/v3.rs` — bringup-only TIM1 PWM configuration.
- `gpio/v0.rs::configure` (cfglr + outdr.set_odr) — bringup.
- `usart/usart_common.rs::init`, `set_dma_rx`, `set_idle_irq` — bringup.
- `usart/usart_common.rs::set_baud` (set_ue toggle + BRR write) — called only
  from USART1 TC ISR (apply-after-TC).

### Tier 2 — multi-context registers, RMW guarded by internal CS

These registers are written from MAIN + at least one ISR. The helpers wrap
their `.modify()` in `critical_section::with` so the RMW is atomic against any
preempting writer to the same register. Callers don't need their own CS.

| Helper | Register | Bits touched today | MAIN writer | ISR writer |
|---|---|---|---|---|
| `usart::set_tc_irq` | CTLR1 | TCIE | `dxl_fast::arm_tx` | `irq::on_usart1_tc` |
| `usart::set_dma_tx` | CTLR3 | DMAT | `dxl_fast::arm_tx` | `irq::on_usart1_tc` |
| `usart::clear_tc` | STATR | TC (w0c) | `dxl_fast::arm_tx` | `irq::on_usart1_tc` |
| `dma::enable` / `dma::disable` (CH4) | DMA1.CH(4).CR | EN | `dxl_fast::fire_now` | `irq::on_usart1_tc` |
| `dma::set_tcie` (CH5) | DMA1.CH(5).CR | TCIE | `dxl_fast::start_fast_after` | `dxl_fast::on_systick`, `dxl_fast::cancel`(via TC ISR) |
| `dma::set_htie` (CH5) | DMA1.CH(5).CR | HTIE | `dxl_fast::start_fast_after` | `dxl_fast::on_systick`, `dxl_fast::cancel`(via TC/HT ISR) |
| `systick::set_irq` | SYSTICK.CTLR | STIE | `dxl_fast::start_plain_after`, `start_fast_after`, `cancel` | `dxl_fast::on_systick`, `cancel`(via TC ISR) |

Other helpers that touch these same registers without a CS — fine because they
are single-context:

- `usart::init`, `usart::set_dma_rx`, `usart::set_idle_irq` — bringup-only.
- `usart::set_baud` (UE toggle on CTLR1) — called only from USART1 TC ISR; ISR
  cannot be preempted by MAIN, and HIGH ISRs serialize. If `set_baud` ever
  becomes MAIN-callable, it needs the same CS treatment.

CS cost on V006/V2A is ~5 cycles via `mstatus.MIE` (csrrci + csrw); see
`portable-atomic-v006` memory note. Nested CS (calling these from inside an
ISR or another CS) is a no-op — the inner `with` re-disables already-disabled
interrupts.

### Tier 3 — must use atomic primitives, not `.modify()`

Registers where multiple priorities touch *different bits*. RMW here corrupts
state. Use atomic single-bit writes or a critical section.

- `gpio::set_level` — uses `BSHR` / `BCR` (atomic). Never `OUTDR.modify`.
- `gpio::toggle` — reads `INDR`, writes `BSHR` / `BCR`. Never `OUTDR.modify`.
  (Earlier `OUTDR ^= mask` form caused the V006 TX_EN wedge.)
- `dma::clear_tc_flag` — uses `IFCR.write` (write-1-to-clear, atomic per-bit
  by hardware), not `IFCR.modify`.

## Adding a new HAL helper — checklist

1. **Identify every priority context that calls it.** Trace from this crate
   outward; if the helper is `pub`, also check `firmware/boards/`.
2. **Identify every other helper that writes the same register.** Their
   callers count too.
3. **If single-context** → Tier 1, no annotation needed.
4. **If multi-context same-bit** → Tier 2; add an entry to the table above
   and a `// SAFETY: same-bit toggle, see hal/SAFETY.md` comment on the call
   site.
5. **If multi-context different-bit** → Tier 3 is required. Options:
   - Replace RMW with an atomic primitive (BSHR/BCR/IFCR/single-bit write).
   - Wrap the modify in `critical_section::with(|_| ...)`. CS cost on V006/V2A
     is ~5 cycles via `mstatus.MIE` (see `portable-atomic-v006` memory note).
   - Hoist the write to a single context and route requests via an atomic
     "pending" flag (the apply-after-TC pattern used for `set_baud` /
     `set_hsitrim` / `recompute_fire_advance_fine_ticks`).

## Audit grep hooks

```sh
# Survey all .modify sites in the firmware crates:
rg -n '\.modify\(' firmware/ch32/src/ firmware/lib/core/src/ firmware/boards/

# Watch especially for OUTDR / CTLR1 / CTLR3 / STATR / CR / IFCR:
rg -n 'outdr\(\)\.modify|ctlr1\(\)\.modify|ctlr3\(\)\.modify|statr\(\)\.modify|cr\(\)\.modify|ifcr\(\)\.modify' firmware/
```

## Why we don't mark `.modify()` `unsafe` upstream

The race class is *any* non-atomic peripheral access reachable from multiple
priorities, not `.modify()` specifically — a bare `read()` then `write()` has
the same race. Marking `.modify()` `unsafe` at the metapac layer would force
`unsafe` on every legitimate single-writer use across the entire chiptool
ecosystem (every chip family, every embedded Rust HAL). The proper gate is
the HAL surface: this module is the chokepoint, and these tier rules are how
we enforce it.

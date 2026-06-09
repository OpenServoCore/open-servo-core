# DXL Fast Chain CRC: Walk-Loop + Split-Fire

Companion to [dxl-fast-chain-crc.md](dxl-fast-chain-crc.md), [dxl-fast-chain-crc-compose.md](dxl-fast-chain-crc-compose.md), and [dxl-fast-chain-crc-time-driven.md](dxl-fast-chain-crc-time-driven.md). Read those first — this doc replaces the periodic catchup body's "fold what's in the ring and re-arm" with a single anchored busy-wait loop, removes RXNE entirely from the chain path, and splits the fire body into its own SysTick CMP so the catchup ISR doesn't block low-priority work.

---

## 0. What changes and why

### 0.1 Why the time-driven design (as shipped) fails on the bench

The time-driven scheme from chain-crc-time-driven.md ran the catchup as a periodic "fold-and-re-arm" with stage 3 (RXNE) covering the last `TAIL_GAP = 20 µs` of the wire. Bench results at the 3M Last 32B corner showed two regressions:

**(a) RXNE fold-cap race.** A single RXNE entry can find multiple bytes in the ring if ISR latency stacks two byte arrivals. The mask threshold `bytes_walked ≥ N_pred − GUARD` then trips *after* the body has already folded the last GUARD bytes — stage 4 has nothing to wait on and the GUARD invariant degrades to "honored by accident". The 3M Last 32B σ ballooned to ±45 µs (bimodal: clean vs chain-broken at INJ_LAST).

**(b) Intermediate-tick drift.** Each `PeriodicCatchup` body computed `next_cmp = now + interval`, so PFIC trap entry + body work (~1–2 µs) accumulated as drift across 5–8 intermediate ticks. By the last catchup, the body was firing 6–10 µs late; `fire_now` then landed past `INJ_LAST + 1 char_time` and tripped pirate's IDLE detector deterministically.

The fold-cap fix (cap RXNE's per-entry fold at `N_pred − GUARD − bytes_walked`) closed (a) at 3M but regressed 1M/2M Last to 100% CRC failure — at low baud the cap interacts badly with the natural RXNE rate. Rather than chase the RXNE race across bauds, drop RXNE.

### 0.2 The redesign in one paragraph

**Anchor a single busy-wait fold loop at the start of the last catchup body.** The loop folds `[snoop_head .. write_pos]` into `bulk_crc` continuously until either `bytes_walked = N_pred` (target met) or `walk_deadline = t_prior_end − t_guard` (timeout). Earlier catchups still fire periodically and fold whatever the ring has, but the last one *anchors itself in absolute time* and holds the CPU through the predecessor's transmission. **After the walk-loop, the catchup body returns** — it sets a SysTick CMP at `fire_tick` and lets the **separate `TxArmed` body** fire and run the post-fire walk + patch.

This trades "one long ISR holding the CPU through fire and patch" for "a long-ish walk ISR + a short fire ISR, with `t_guard − tx_latency` of CPU-yield between them". With `GUARD = 1` (see §4) the gap is +3.875 µs at 1M (yield mode) and goes slightly negative at 2M / 3M (-1.1 / -2.8 µs, recursive-fire mode — §7.2). At low baud the catchup ISR yields cleanly between walk and fire; at high baud the body's past-tick check re-enters inline and fires without a yield. Both paths share the same code and the same wall-clock anchors.

### 0.3 What goes away

- **RXNE-tail tier.** `usart::set_rxne_irq(USART1, true)` is never called from `dxl_fast` again. The `on_rxne` entry stays as a no-op stub so the USART1 ISR dispatch in `irq.rs` can keep its dead branch; future framing modes (Phase B low-baud) can plug their own consumer in there.
- **`TAIL_GAP_US`.** The hand-off-to-stage-3 boundary is gone; the catchup body's deadline is `t_prior_end − t_guard` instead.
- **Fold cap in `on_rxne`.** Together with the RXNE handler itself.
- **Inline fire + post-fire + patch** in the catchup body. Now in `TxArmed`.

What stays: `accumulate_snoop`, `predict_n_pred` (for the walk-loop's target), `BYTES_PER_INTERVAL = 15` and the per-baud interval table from chain-crc-time-driven.md §2, the four-phase FSM (now `PeriodicCatchup → TxArmed → TxStreaming → CrcPatched → Done`), and stage 4 (renamed *post-fire walk*) as the final residue absorber.

---

## 1. The formula

Seven quantities, all in ticks (HCLK = 48 MHz, 1 µs = 48 ticks):

```
t_fire                = request_end + fire_us × ticks_per_us − t_fast_entry
t_prior_start         = request_end + rdt + byte_time             # T_0 lands here
t_prior_end           = request_end + rdt + n_pred × byte_time    # T_{n_pred-1} lands here
t_prior_duration      = t_prior_end − t_prior_start = (n_pred − 1) × byte_time
t_schedule_duration   = BYTES_PER_INTERVAL × byte_time            # = catchup_interval
t_guard               = GUARD_BYTES × byte_time                   # GUARD = 1
t_catchup_entry       = CATCHUP_ENTRY_TICKS                       # 110 ticks ≈ 2.29 µs, baud-independent
t_fast_entry          = FAST_ENTRY_TICKS                          # 165 ticks ≈ 3.44 µs, baud-independent (placeholder)
```

`t_fast_entry` is the Fast chain `TxArmed` ISR's wire-fire latency: PFIC trap entry + `fire_now` (TX_EN flip + DMA CH4 enable) + USART start-bit latch. The Plain reply path has its own analogue `t_plain_entry` (used by `start_plain_after`, not by the chain math here). Both live in [`firmware/ch32/src/measurements.rs`](../firmware/ch32/src/measurements.rs) as compile-time constants alongside `t_catchup_entry`.

The **last catchup tick** (start of the busy-wait fold loop) is:

```
t_catchup_last  =  t_fire − min(t_schedule_duration, t_prior_duration) − t_guard − t_catchup_entry
```

The `t_catchup_entry` term back-dates the SysTick CMP so the body's CRC-fold work begins at the intended wall-clock anchor instead of `anchor + ISR_entry_latency`. It covers PFIC trap entry, `clear_match`, state-load, match arm, and the post-fold `set_cmp`/`set_irq`/past-tick check. See §1.1 for the bench derivation.

The walk-loop's **deadline**:

```
walk_deadline   =  t_prior_end − t_guard
```

The **first catchup tick** is derived by stepping back from `t_catchup_last` in `interval`-sized steps while one more step would still land at or after `t_prior_start + interval`:

```
t_catchup_start = t_catchup_last
while (t_catchup_start − t_prior_start) ≥ interval:
    t_catchup_start −= interval
```

(equivalent to the user's "loop subtracting `t_tick` until less than `t_prior_start + t_tick`").

The formula's shape captures both regimes:

| Regime | Condition | min(...) | t_catchup_last vs t_fire |
|---|---|---|---|
| **Small n_pred** | `t_prior_duration ≤ interval` | `t_prior_duration` | walk runs full wire window |
| **Large n_pred** | `interval < t_prior_duration` | `interval` | walk runs one chunk; intermediates handle the rest |

In both regimes the walk-loop exits at `walk_deadline` with `bytes_walked = n_pred − GUARD`. The remaining GUARD bytes belong to the post-fire walk.

### 1.1 Deriving entry-tick constants

Each fire-path ISR has fixed overhead between its hardware trigger and its wire-affecting work — same code path every shot, baud-independent in wall-clock. The three values in `firmware/ch32/src/measurements.rs` are taken from scope captures using the same methodology:

- `stat_high()` at the very top of the ISR (before `clear_match`), `stat_low()` at every exit.
- `dbg_high()`/`dbg_low()` bracket the useful work (CRC fold for catchup, or `fire_now` for the fire paths).
- Stat-pulse width = full ISR duration; dbg-pulse width = useful work. The difference is fixed overhead.

**`t_catchup_entry`.** Bench 2026-06-02 (`--features bench`, INJ=1 single-body). Five shots, stat-minus-dbg: 38.70−36.39, 38.51−36.25, 39.64−37.34, 39.72−37.42, 38.50−36.22 (µs). Mean = 2.29 µs, σ = 0.03 µs → 110 HCLK ticks at 48 MHz, exported as the `CATCHUP_ENTRY_TICKS` const. Subtracting it from `t_catchup_last` aligns the fold-loop's wall-clock start with the formula's intent. At 3M, 110 ticks ≈ 0.7 byte_time — significant relative to the wire byte rate. Without this term the body's effective anchor drifts later by 2.29 µs every shot, eating into the walk-loop's deadline budget.

**`t_fast_entry` and `t_plain_entry`.** Pending bench measurement with the same dbg/stat methodology applied to `on_systick`'s `TxArmed` arm (Fast) and `Plain` arm. Current values carry forward the prior CT-tuned defaults from before the runtime tunables were removed: `FAST_ENTRY_TICKS = 165` (≈ 3.44 µs) and `PLAIN_ENTRY_TICKS = 156` (≈ 3.25 µs). Re-measure when bench access permits; the constants' purpose is to back-date `t_fire` so wire-fire lands on the formula's intended deadline regardless of ISR-entry latency.

---

## 2. The pipeline

```
chain arm  (start_fast_after):
    snoop_head        = parsed_end & RX_MASK
    bulk_crc          = 0
    n_pred            = predict_n_pred(fire_us)
    t_guard           = GUARD_BYTES × byte_time
    t_prior_duration  = (n_pred − 1) × byte_time
    interval          = BYTES_PER_INTERVAL × byte_time
    t_fire            = request_end + fire_us × TICKS_PER_US − fast_latency
    walk_deadline     = request_end + rdt + n_pred × byte_time − t_guard

    if n_pred > GUARD:
        t_catchup_last = t_fire − min(interval, t_prior_duration) − t_guard − t_catchup_entry
    else:
        t_catchup_last = t_fire                                # walk-loop no-ops; post-fire does it all

    # Step back from anchor to first tick that still leaves ≥ 1 interval to t_prior_start.
    t_prior_start = request_end + rdt + byte_time
    delta = t_catchup_last − t_prior_start
    while delta ≥ interval:
        delta −= interval
    t_catchup_start = max(t_prior_start + delta, now)
    set_phase(PeriodicCatchup)
    SysTick CMP at t_catchup_start

on_systick  PeriodicCatchup:
    accumulate_snoop()                                          # fold ring[snoop_head..write_pos]
    if not at_anchor(now, t_catchup_last):                      # ±½ byte_time tolerance
        # Snap to anchor if `now + interval` would overshoot — prevents
        # ISR-entry drift from pushing the last body past t_catchup_last.
        next = min(now + interval, t_catchup_last)
        SysTick CMP at next
        return

    # Last catchup. Busy-wait + fold until target OR deadline.
    wait_and_fold_until(target = n_pred, deadline = walk_deadline)
    set_phase(TxArmed)
    SysTick CMP at t_fire                                       # hand off, yield CPU

on_systick  TxArmed:
    fire_now()                                                  # TX_EN high + DMA CH4 EN
    wait_and_fold_until(target = n_pred,
                        deadline = walk_deadline + (GUARD+1) × byte_time)
    patch_crc()
    set_phase(TxStreaming → CrcPatched)
    # USART1 TC ISR drives → Done.

wait_and_fold_until(target, deadline):
    loop:
        accumulate_snoop()
        if bytes_walked ≥ target: return
        if now ≥ deadline:        return
```

Two things to note about the pipeline:

- **Catchup is allowed to fire before the wire starts.** For small `n_pred`, `t_catchup_last` lands ≤ `t_prior_start` — the walk-loop spins on an empty ring until the first byte lands. That's fine; the loop is just `dma::remaining` reads in a tight loop, ~20 ns per iteration, no ISR overhead.
- **The catchup ISR returns at `walk_deadline`, not at `t_fire`.** That's the whole point — the CPU is free to service other ISRs in the `t_guard − tx_latency` gap. The fire ISR then fires `~ISR_re-entry_latency` after `t_fire`, which is < 1 char_time at every baud we ship, so pirate's IDLE detector never trips.

---

## 3. Numerical examples

Numbers below assume `t_fast_entry = 165 ticks (3.44 µs)` — the current `FAST_ENTRY_TICKS` placeholder. Bench measurement (§1.1) may shift this; refresh the worked examples when it lands.

### 3.1 3 Mbaud, n_pred = 14 (inj_len=4 dut_len=anything Last)

```
byte_time            = 160 ticks   (3.333 µs)
t_fast_entry         = 165 ticks   (3.4375 µs)
rdt                  = 12000 ticks (250 µs)

t_fire               = req_end + 14 × 160 − 165             = req_end + 14075 ticks  (293.23 µs)
t_prior_start        = req_end + 12000 + 160                = req_end + 12160 ticks  (253.33 µs)
t_prior_end          = req_end + 12000 + 14 × 160           = req_end + 14240 ticks  (296.67 µs)
t_prior_duration     = 13 × 160                             = 2080 ticks             (43.33 µs)
t_schedule_duration  = 15 × 160                             = 2400 ticks             (50 µs)
t_guard              = 1 × 160                              = 160 ticks              (3.33 µs)
t_catchup_entry      = 110 ticks                                                       (2.29 µs)
walk_deadline        = t_prior_end − t_guard                = req_end + 14080 ticks  (293.33 µs)

t_catchup_last       = 293.23 − min(50, 43.33) − 3.33 − 2.29 = 244.28 µs
t_catchup_start      = (no step-back: 244.28 − 253.33 < 0)   = 244.28 µs              # single tick
```

`walk_deadline (293.33 µs)` and `t_fire (293.23 µs)` differ by `t_fast_entry − t_guard = +0.10 µs` — at 3M with GUARD=1 the design lands marginally inside **yield mode** (§4), but the gap is small enough (≈ 5 ticks) that any ISR-entry jitter on the `TxArmed` body can flip it into the **recursive-fire mode** (§7.2). Treat 3M as the boundary case at the current placeholder.

| t (µs from req_end) | Event |
|---|---|
| 244.28 | SysTick CMP match. ISR entry overhead (~2.29 µs of `clear_match`/state-load/match arm) consumed before fold work begins. |
| ~246.57 | Walk-loop body starts; `accumulate_snoop` folds 0 (ring empty). |
| 253.33 | Byte 0 lands; walk-loop folds, bytes_walked = 1. |
| ... | (bytes 1..11 fold incrementally, one per 3.33 µs) |
| 290.00 | Byte 11 lands; bytes_walked = 12. |
| 293.23 | (`t_fire` — yield-mode hand-off would fire here if no ISR-entry jitter…) |
| 293.33 | Byte 12 lands; bytes_walked = 13. `now ≥ walk_deadline` → walk-loop exits. Body sets `phase = TxArmed`, `set_cmp(t_fire)`, `set_irq(true)`; past-tick check `now − t_fire ≈ +0.1 µs ≥ 0` → recursive `on_systick()` (yield window collapsed). |
| ~293.5 | TxArmed body runs inline: `fire_now()` (TX_EN high + DMA CH4 enable). |
| ~293.7 | post-fire walk entered; `post_deadline = walk_deadline + (GUARD+1) × byte_time = 293.33 + 6.67 = 300.00 µs`. |
| 296.67 | Byte 13 lands; walk folds, bytes_walked = 14 → `patch_crc()` → `CrcPatched`. |
| ~297    | (parallel: first chip wire bit hits the bus ≈ INJ_LAST + bus turnaround) |

The yield window at 3M is too narrow to count on as a real CPU-yield opportunity; treat it as recursive-fire for ADC-latency budgeting. The ISR-entry comp + GUARD=1 keeps fire landing within ~1 char_time of `t_prior_end` regardless.

### 3.2 3 Mbaud, n_pred = 50 (large case)

```
t_fire            = req_end + 12000 + 50 × 160 − 165 = req_end + 19835 ticks  (413.23 µs)
t_prior_duration  = 49 × 160                         = 7840 ticks             (163.33 µs)
walk_deadline     = req_end + 12000 + 50 × 160 − 160 = req_end + 19840 ticks  (413.33 µs)

t_catchup_last    = 413.23 − min(50, 163.33) − 3.33 − 2.29 = 357.61 µs
step back:
  357.61 − 253.33 = 104.28 ≥ 50 → 307.61
  307.61 − 253.33 =  54.28 ≥ 50 → 257.61
  257.61 − 253.33 =   4.28 < 50 → stop
t_catchup_start   = 257.61 µs
```

Three catchups at 257.61 / 307.61 / 357.61 µs. The first two are intermediates: each `accumulate_snoop` folds whatever the ring has, then `next = min(now + interval, t_catchup_last)` schedules the next anchor-snapped CMP. The third is the last one and runs the walk-loop from 357.61 → 413.33 µs (~56 µs of busy-wait + fold, picking up bytes 31..48 as they arrive). Same marginal-yield boundary as §3.1 since `walk_deadline − t_fire = +0.10 µs`.

---

## 4. Why GUARD = 1 (with `t_catchup_entry`)

The fold-loop deadline is `t_prior_end − t_guard`. Once the loop returns, the catchup body sets the fire CMP. The gap to fire is:

```
fire_tick − walk_deadline = (t_prior_end − t_fast_entry) − (t_prior_end − t_guard)
                          = t_guard − t_fast_entry
```

Two regimes depending on sign:

- **`t_guard > t_fast_entry` → yield mode.** Body returns, CPU is free for `t_guard − t_fast_entry` until the fire CMP matches and the TxArmed ISR re-enters to fire. This is the "split-fire" win — low-priority ISRs (ADC TC) can land in the gap.
- **`t_guard ≤ t_fast_entry` → recursive-fire mode.** Walk-loop times out *after* `t_fire`. The body's past-tick check (`if (now − fire_tick) ≥ 0: on_systick()`) re-enters `TxArmed` inline and fires immediately. No CPU yield, but timing is still anchored to wall-clock and fire lands within ~1 char_time of `t_prior_end`.

With `t_fast_entry = 3.44 µs` (current `FAST_ENTRY_TICKS = 165` placeholder; bench measurement pending — §1.1):

| Baud | byte_time | GUARD=1 `t_guard` | gap = `t_guard − t_fast_entry` | Mode |
|---|---|---|---|---|
| 3M | 3.33 µs | 3.33 µs | −0.10 µs | recursive (marginal) |
| 2M | 5.00 µs | 5.00 µs | +1.56 µs | yield |
| 1M | 10.00 µs | 10.00 µs | +6.56 µs | yield |

The 3M row is at the boundary — ISR-entry jitter on the `TxArmed` body can flip the sign either way. Until bench measurement pins `t_fast_entry` more precisely, treat 3M as recursive-fire for budgeting.

The `t_catchup_entry` term (§1.1) is what makes `GUARD = 1` viable. Without it, the body's actual fold-start drifts 2.29 µs later than the formula's `t_catchup_last`, eating 2.29 µs of the `walk_deadline − t_catchup_last` budget. At 3M `byte_time = 3.33 µs`, that's nearly a full byte of lost fold time — pre-fire `bytes_walked` would fall short of `n_pred − GUARD` and post-fire would need to absorb the deficit *and* race patch_crc's DMA-prefetch deadline. Compensating in the formula keeps the budget aligned with reality.

Why drop from `GUARD = 3` to `GUARD = 1`:

- **Patch-CRC slack.** Post-fire fold count = `GUARD` bytes (chain-crc.md §3 has 11-byte DMA-prefetch budget at the tightest 3M dut_len=1 slack). Each GUARD byte we don't pre-fold costs ~0.5 µs of CRC-walk time inside `patch_crc`'s budget. `GUARD = 1` gives the most patch slack — relevant for the tightest dut_len corner.
- **Empirical validation.** Bench 2026-06-02 at GUARD = 1 + `t_catchup_entry` comp ran a 1000-shot stress sweep across inj × dut with 0 CRC failures (chip running at the bench-tuned `TX_FAST_LATENCY ≈ 6.125 µs`; the runtime tunable has since been removed in favor of `FAST_ENTRY_TICKS`). 1M Last failures observed in the same session were traced to the tuner's Phase A "all SAFE" picker over-shooting `TX_FAST` to the top of the sweep range — independent of the GUARD choice and the motivating reason the tunable was deleted.

Cost of dropping GUARD: at high baud we lose (or nearly lose) the CPU-yield window between walk-loop exit and fire. At 3M the catchup ISR holds PFIC HIGH continuously from `t_catchup_last` through `patch_crc` (~57 µs at `n_pred = 14`). ADC TC (PFIC LOW) can be delayed by up to that window — one ADC sample frame at 20 kHz. Acceptable for now; revisit if tighter ADC determinism becomes a requirement, or if bench measurement raises `t_fast_entry` enough to pull 2M into recursive mode.

---

## 5. State & invariants

### 5.1 Phase enum

```rust
enum FastChainPhase {
    PeriodicCatchup,    // SysTick CMP-driven; last body runs the walk-loop
    TxArmed,            // SysTick CMP armed at fire_tick; body fires + patches
    TxStreaming,        // DMA shifting chip TX
    CrcPatched,         // patch_crc done; USART TC drives → Done
    Done,
    Fault(FastChainFault),
}
```

Legal transitions: `PeriodicCatchup → TxArmed → TxStreaming → CrcPatched → Done` (plus any `→ Fault(_)`).

### 5.2 New state fields

`ReplyState::Chain` carries `last_catchup_tick` (the anchor) and `walk_deadline` (the absolute exit time). Both are computed once in `start_fast_after` and read by `on_systick`. The "snap to anchor" code in `PeriodicCatchup` reads `last_catchup_tick` to decide the next CMP; the walk-loop reads `walk_deadline`; the `TxArmed` post-fire walk reads `walk_deadline + (GUARD+1) × byte_time`.

### 5.3 Field ownership

Same single-writer-per-instant argument as chain-crc-compose.md §2.2 / chain-crc-time-driven.md §4.2: USART1 and SysTick share PFIC HIGH, so STATE access from either ISR is uncontested. No atomics on `snoop_head` / `bulk_crc` / `bytes_walked`.

---

## 6. CPU budget

### 6.1 ISR durations

3M, n_pred = 14 (single catchup, marginal-recursive mode at GUARD=1 + current `t_fast_entry`):

| ISR | Duration | Notes |
|---|---|---|
| `PeriodicCatchup` + recursive `TxArmed` | ~57 µs | walk-loop 244 → 293 µs, recursive fire + post-walk + patch 293 → 300 µs |

Total chain-CRC CPU per shot: ~57 µs of contiguous PFIC HIGH at 3M. The +0.10 µs yield window is too narrow to count on.

1M, n_pred = 11 (single catchup, yield mode at GUARD=1):

| ISR | Duration | Notes |
|---|---|---|
| `PeriodicCatchup` (last) | ~107 µs | walk-loop 244 → 350 µs |
| (CPU yield) | ~6.6 µs | `t_guard − t_fast_entry` |
| `TxArmed` (fire + post) | ~10 µs | fire + 1-byte post-walk + patch |

3M, n_pred = 50 (3 catchups, marginal-recursive mode):

| ISR | Duration | Notes |
|---|---|---|
| Intermediate #1 | ~10 µs | empty fold (wire just started) |
| Intermediate #2 | ~10 µs | 15-byte CRC walk |
| `PeriodicCatchup` (last) + recursive `TxArmed` | ~63 µs | walk-loop 358 → 413, then inline fire + patch |

Total: ~83 µs spread across 3 ISRs (vs 4 at GUARD=3) — the recursive path collapses the final two into one body.

### 6.2 Control-loop interaction

GUARD=3 (prior shipping config) put 3.9 µs of yield between walk and fire at 3M / 23.9 µs at 1M. GUARD=1 with the current `FAST_ENTRY_TICKS = 165` placeholder keeps yield at 1M (+6.56 µs) and 2M (+1.56 µs), and lands 3M marginally yield (+0.10 µs) — too narrow to budget as a real opportunity. At 3M the catchup body effectively holds PFIC HIGH from `t_catchup_last` through `patch_crc` (~57 µs at n_pred=14, ~83 µs at n_pred=50). ADC TC (PFIC LOW) can be delayed by that window — one to two 20 kHz sample frames at worst.

If ADC determinism becomes a hard requirement at 3M, options are:
- Bump GUARD back to 2 or 3 at the cost of post-fire byte count → tighter `patch_crc` deadline.
- Re-measure `t_fast_entry` and lower it (shorten the catchup-to-fire-now path) to put the 3M gap firmly positive.

---

## 7. Edge cases

### 7.1 `n_pred ≤ GUARD`

The arm code collapses to `t_catchup_last = t_fire`. PeriodicCatchup body fires *at* `t_fire`, walk-loop exits immediately (deadline already past), CMP at `t_fire` re-fires (recursion via `on_systick()` since now ≥ CMP), TxArmed body fires inline. Post-fire absorbs all `n_pred` bytes. Same outcome as the small-n_pred Plain path.

### 7.2 `t_fast_entry > t_guard` (recursive-fire mode)

When `t_fast_entry > t_guard`, `walk_deadline > t_fire` — the walk-loop is still running when fire was supposed to happen. The catchup body's final `if (now − fire_tick) ≥ 0: on_systick()` catches this: it recursively enters `TxArmed` inline and fires immediately. No CPU-yield window, but timing stays anchored to wall-clock and the wire fire lands within ~1 char_time of `t_prior_end`.

At GUARD=1 with the current `FAST_ENTRY_TICKS = 165` placeholder (3.44 µs), 3M lands at the boundary (gap = −0.10 µs) and is effectively recursive once ISR-entry jitter is accounted for. 2M and 1M sit cleanly in yield mode (+1.56 / +6.56 µs). Bench measurement of `t_fast_entry` (§1.1) may shift these.

If a tighter ADC-yield budget is needed at high baud, bump GUARD or re-measure `t_fast_entry` and shorten the catchup-to-fire-now path to push the gap positive — see §6.2.

### 7.3 Predecessor stops mid-window

USART1 IDLE backdating + the walk-loop's deadline together make this graceful. If the predecessor drops out, no more bytes land; `accumulate_snoop` returns early; the walk-loop spins until `walk_deadline`, then hands off to `TxArmed` which fires on time. `bytes_walked < n_pred` raises `PreviousSlotTimeout` in the diagnostics path post-patch. patch_crc still runs (computing CRC over an incomplete predecessor stream) — wire CRC will be wrong, but the chip's TX completes and the bus recovers cleanly.

### 7.4 Catchup body delayed past `t_catchup_last`

Same "wash" property as chain-crc-time-driven.md §3.2: a late entry's `accumulate_snoop` picks up more bytes than usual. The walk-loop then has less wall-clock until `walk_deadline`. As long as the loop's CRC-compute throughput (24 ticks/byte = 0.5 µs/byte) exceeds the wire byte rate, it catches up.

3M / 0.5 µs/byte / 3.33 µs/byte wire = 6.7× headroom — even a 30 µs delay on entry is recoverable inside the walk-loop's deadline.

---

## 8. Migration & what tunable

Migration from the time-driven shipping code:

1. Strip RXNE arm/disarm calls from `dxl_fast` (the `on_rxne` body becomes a no-op stub).
2. Strip `TAIL_GAP_US` constant.
3. Add `last_catchup_tick` and `walk_deadline` fields to `ReplyState::Chain`.
4. Replace `wait_and_accumulate_tail(expected)` (relative deadline) with `wait_and_fold_until(target, absolute_deadline)`.
5. Split `on_systick`'s chain branch into `PeriodicCatchup` (walk-loop ends, CMP at `t_fire`) and `TxArmed` (fire + post-walk + patch).
6. Set `GUARD_BYTES = 1` and add `CATCHUP_ENTRY_TICKS = 110` (the `t_catchup_entry` term — see §1.1). Subtract it from `last_catchup_tick` in `start_fast_after`.
7. Update `is_legal_transition` for the new phase chain.
8. Delete the `dxl_tx_plain_latency_us` / `dxl_tx_fast_latency_us` CT fields, their atomics, store fns, dispatcher write handlers, and bench tuner, and replace with the compile-time constants `PLAIN_ENTRY_TICKS` / `FAST_ENTRY_TICKS` in `firmware/ch32/src/measurements.rs`.

Tuning knobs that remain (all compile-time):

- `BYTES_PER_INTERVAL = 15` (`firmware/ch32/src/dxl_fast.rs`) — same as chain-crc-time-driven.md §2.1. Controls walk-cost-per-tick. Bump up only if we hit the kernel's 50 µs heartbeat on a wider window.
- `GUARD_BYTES = 1` (`firmware/ch32/src/dxl_fast.rs`) — see §4. Drop further at your peril (sub-byte pre/post split). Bump up to 2 or 3 if ADC-yield determinism at high baud becomes a hard constraint.
- `CATCHUP_ENTRY_TICKS = 110` (= `t_catchup_entry`, `firmware/ch32/src/measurements.rs`) — see §1.1. Re-measure with the scope-feature stat/dbg markers if PFIC priorities, the catchup-body code path, or the HCLK frequency change.
- `FAST_ENTRY_TICKS = 165` (= `t_fast_entry`) and `PLAIN_ENTRY_TICKS = 156` (= `t_plain_entry`) (`firmware/ch32/src/measurements.rs`) — same role as the prior CT-tuned `TX_{FAST,PLAIN}_LATENCY_TICKS`, but now silicon-fixed. Re-measure with the same scope methodology as `t_catchup_entry` when bench access permits; the catchup anchor formula is otherwise a pure function of n_pred / baud / GUARD / `t_catchup_entry`.

# DXL Fast Chain CRC: Time-Driven Catchup

Companion to [dxl-fast-chain-crc.md](dxl-fast-chain-crc.md) and [dxl-fast-chain-crc-compose.md](dxl-fast-chain-crc-compose.md). Read those first — this doc assumes you know what stages 1–4 are, what `bulk_crc` / `snoop_head` / `bytes_walked` track, and why stage 4 has to fit TX DMA prefetch slack.

The composed pipeline in chain-crc-compose.md is **byte-driven**: stage 1 fires at ring half/full boundaries, and a single stage-2 CMP lands at `fire_tick − M(N_pred)`. It works for the easy corners but has two failure modes that surfaced in bench (see §0.1). This doc replaces stages 1 + 2 with a **time-driven** catchup that fires on a fixed period and walks whatever is in the ring — no HT/TC, no per-shot M sizing, no spin-until-threshold body.

---

## 0. What changes and why

### 0.1 Why the byte-driven design fails

Two distinct failure modes, observed at the 3 Mbaud bench corner and at low-baud after tuning the high-baud one:

**(a) RXNE-vs-fire CMP collision at 3 Mbaud INJ=1 DUT=1.** With M sized so catchup lands ~20 µs before fire, the catchup body bails when it sees `fire_tick − now < J`, handing the tail to stage 3 (RXNE). The last RXNE entry takes ~3–4 µs and queues the fire CMP behind it, producing `extra_idle` and intermittent CRC corruption. GUARD=2 doesn't prevent this because the RXNE body width is comparable to GUARD's safety margin (~6.66 µs = 2 byte_times at 3M).

**(b) HT/TC + catchup is a wash, not a win.** When DMA1_CH5 HT or TC fires first, it advances `snoop_head` so catchup walks fewer bytes when it gets the CPU. When catchup fires first, HT/TC queues behind it and finds nothing to walk. Either way the CPU spends roughly the same total time on the CRC walk — stage 1 doesn't shrink stage 2's work, it just shuffles it. Meanwhile the IRQ contention between them (chain-crc-compose.md §2.3 queue cases A and B) adds variance without subtracting work.

A spin-loop body in stage 2 (walk until `bytes_walked ≥ N_pred − GUARD` or `to_fire ≤ J`) closes the RXNE-collision hole at 3M but regresses 2M Last across all payload sizes (CRC σ ≈ 1 µs). The bail margin J = 120 ticks = 2.5 µs is < 1 byte_time (5 µs) at 2M, so the loop bails inside the wire-write window of the byte that just arrived, racing the fire body.

### 0.2 The redesign in one paragraph

**Drop DMA HT/TC entirely.** **Make stage 2 periodic**: a SysTick CMP fires every I µs (where I is sized so each catchup walks a bounded number of bytes — target ≈ 15). Each catchup body walks `[snoop_head .. write_pos]` into `bulk_crc`, advances `snoop_head`, and re-arms the next CMP at `now + I` **unless** the next CMP would land within `TAIL_GAP` of `fire_tick` — then it hands off to stage 3 (RXNE) and arms the fire CMP at `fire_tick`. Stage 4 unchanged: walks the GUARD residue and patches.

This trades "one perfectly-sized CMP" for "a handful of cheaply-sized CMPs". Each CMP is bounded by design (~7.5 µs walk + ~1 µs housekeeping), and there is no decision tree per call — same code path for every (N_pred, baud, INJ) combination.

### 0.3 What goes away

- Stage 1 (DMA1_CH5 HT/TC for CRC). The handler still exists for ring-overflow detection, but its `bulk_crc` walk and `snoop_head` write disappear.
- `switch_margin_ticks(N_pred, byte_time)` and the H/J/W tuning vocabulary. Replaced by a single per-baud interval table.
- The catchup spin-loop and its `bail_margin` heuristic.
- The "M = walk + H + J" closed-form derivation in chain-crc-compose.md §4. The interval is sized once per baud; no per-shot calculation.

What stays: stage 3 (RXNE) as the last-mile pre-folder, stage 4 (fire body) as the patch-and-finalize, GUARD = 2 as stage 4's residue budget, predict_chain_pure for N_pred (still used by stage 3's mask threshold), `accumulate_snoop` for the ring walk.

---

## 1. The pipeline

One flow:

    chain arm:
        snoop_head = parsed_end & RX_MASK
        bulk_crc   = 0
        I          = catchup_interval_us(baud)              # §2
        fire_tick  = wire_end_tick(predecessor)
        next_cmp   = now + I                                 # first periodic catchup
        if next_cmp > fire_tick − TAIL_GAP:
            next_cmp = fire_tick                             # short wire: pure RXNE + stage 4
            phase    = TxArmed
            enable RXNEIE                                    # stage 3 on
        else:
            phase    = PeriodicCatchup
        SysTick CMP at next_cmp

    on_systick  PeriodicCatchup body:
        accumulate_snoop()                                   # walk ring[snoop_head..write_pos]
        next_cmp = now + I
        if next_cmp + TAIL_GAP > fire_tick:
            # last interval: hand off to stage 3 + stage 4
            set_phase(TxArmed)
            SysTick CMP at fire_tick
            enable RXNEIE
        else:
            SysTick CMP at next_cmp                          # another periodic catchup

    on_rxne  (stage 3, while phase ∈ {TxArmed}):
        walk ring[snoop_head .. write_pos] into bulk_crc
        if bytes_walked ≥ N_pred − GUARD:
            mask RXNEIE

    on_systick  TxArmed body  (stage 4):
        fire_now()
        wait_for_predecessor(N_pred, bound = 1 byte_time)
        accumulate_snoop()                                   # GUARD residue
        patch_crc()
        mask RXNEIE (if still armed)

The differences from chain-crc-compose.md §1's pipeline:

- **No HT/TC arm.** DMA CH5 HT/TC is left masked; the on_dma1_ch5 handler retains only its ring-overflow guard (chain-crc-compose.md §6.5's "stale-pending" hazard goes away with the IE itself).
- **Periodic CMP loop.** The catchup body re-arms itself on every entry until `now + I` would overshoot `fire_tick − TAIL_GAP`. Each entry is a single `accumulate_snoop()` call; no threshold check, no spin loop.
- **Stage 3 arms at handoff, not at arm.** RXNEIE goes on only when the last periodic catchup decides "next interval would land in the tail" — keeping stage 3 idle through the bulk of the wire window so its per-byte cost doesn't pile up on top of catchup's walk.

---

## 2. Sizing the interval

Two design parameters: `BYTES_PER_INTERVAL` (the bytes-per-catchup target) and `TAIL_GAP` (how much of the wire window stage 3 owns).

### 2.1 BYTES_PER_INTERVAL → I per baud

The walk cost is W = 24 ticks/byte (= 0.5 µs at 48 MHz HCLK; bench-measured, `measured_constants.rs`). Targeting ~7.5 µs of walk per catchup body keeps the body well under one kernel tick (50 µs) and leaves plenty of return-to-foreground time before the next CMP:

    BYTES_PER_INTERVAL  =  15
    walk_per_body       =  15 × 0.5 µs = 7.5 µs
    body_total          ≈  walk + 1.5 µs trap entry + 1 µs housekeeping ≈ 10 µs

Per-baud interval:

    I(baud)  =  BYTES_PER_INTERVAL × 10 / baud_mhz   (µs)

| Baud | bytes/µs | I (µs) | Rounded to 50 µs grid |
|---|---|---|---|
| 3 Mbaud   | 0.300   | 50.0   | **50**  |
| 2 Mbaud   | 0.200   | 75.0   | **75**  |
| 1 Mbaud   | 0.100   | 150.0  | **150** |
| 500 kbaud | 0.050   | 300.0  | **300** |
| 230400    | 0.0230  | 651.0  | **650** |
| 115200    | 0.0115  | 1302   | **1300**|

The "rounded to 50 µs grid" column matters because the kernel runs at 20 kHz ticks. Aligning catchup CMPs to multiples of 50 µs means the catchup heartbeat doesn't beat against the kernel heartbeat at an unrelated phase. At 3M, 2M, 1M, and 500k the natural I is already a clean multiple of 50 µs; at 230400 and below the rounding error is <1% and irrelevant given the per-byte cost at those bauds.

### 2.2 TAIL_GAP

Stage 3 has to cover the bytes that arrive between the last periodic catchup walk and the fire body. Pick `TAIL_GAP = 20 µs`:

    bytes_in_tail  =  TAIL_GAP × bytes/µs
                   =  20 × 0.3 = 6 bytes at 3M
                   =  20 × 0.2 = 4 bytes at 2M
                   =  20 × 0.1 = 2 bytes at 1M

These are small enough that stage 3's per-byte cost (~3 µs/entry, trap entry dominated) doesn't accumulate into a contention window large enough to push the fire CMP. At 3M / 6 bytes tail, total RXNE wall-clock is ~18 µs spread over a 20 µs wire window — utilization ~90%, but **bounded**: there are exactly 6 RXNE entries, and the last one ends ≥ 1 byte_time before fire_tick by construction (the fire CMP is at fire_tick; the last byte arrives at fire_tick − byte_time; RXNE for it ends at fire_tick − byte_time + 3 µs ≈ fire_tick − 0.3 µs).

20 µs is comfortably > 1 byte_time at every baud we ship (3.33 µs at 3M, 10 µs at 1M), so the tail is always > 1 byte regardless of baud. At lower bauds the tail spans fewer bytes, but the wire windows are also longer, so the periodic catchups dominate and the tail is a small fraction of total work.

### 2.3 Why not a single interval like 50 µs everywhere

Two reasons:

- At 1M, 50 µs is only 5 bytes — the catchup body trap-entry overhead (~1.5 µs) dominates the walk (2.5 µs). CPU% spent on chain-CRC inflates by 3× with no benefit.
- At 230400 and below, 50 µs is < 1 byte. The catchup body would find an empty ring on most entries.

Constant bytes-per-interval is the right invariant: it bounds the per-ISR walk cost at every baud, and at low baud where wire windows are long it naturally fires less often (because there are fewer bytes to fold per unit time).

---

## 3. Edge cases

### 3.1 Wire window shorter than I + TAIL_GAP

INJ=1 Last at 3M has bytes_before = 10 + 1 = 11 → wire_window ≈ 37 µs. With I = 50 µs and TAIL_GAP = 20 µs, the first scheduled catchup at `now + 50` lands past `fire_tick − 20`. The arm code (§1) detects this and skips the periodic phase entirely, going straight to TxArmed + RXNEIE. Stage 3 handles all 11 bytes; stage 4 patches the GUARD=2 residue. This is the only "decision" in the pipeline, and it's a single comparison.

Below 3M, INJ=1 wire windows are even longer (55 µs at 2M, 110 µs at 1M), so this short-wire branch only triggers at 3M with INJ ≤ 1. At 3M INJ=2 the wire is 47 µs vs `I + TAIL_GAP = 70` — still triggers. At 3M INJ=3 wire is 57 µs — still triggers. At 3M INJ=4 wire is 67 µs — still triggers (just barely). At 3M INJ=5 wire is 77 µs — periodic catchup runs once.

So the short-wire branch covers everything up to INJ=4 at 3M. That's fine: those are the cases where the existing pure-stage-3 path was already working before the catchup-loop regression.

### 3.2 Catchup body delayed by an unrelated HIGH-priority handler

Same-priority queueing (chain-crc-compose.md §2.3) still applies: if USART1 IDLE/TC fires while the catchup body is mid-walk, the next catchup CMP queues. When the body returns, the queued CMP fires immediately. The walk on the late entry finds more bytes than usual (the in-flight bytes plus whatever arrived during the delay), but `accumulate_snoop()` is unbounded — it walks them all. The cost is one larger walk, not a missed walk.

The "wash" property mentioned in §0.1 reappears here but in our favor: a delayed catchup picks up the bytes the delay accumulated. Total CPU on CRC walk is roughly the same; what changes is the distribution across CMPs.

### 3.3 Catchup body delayed past the tail-handoff point

If the catchup body is so late that `now + I + TAIL_GAP > fire_tick` when it's halfway through deciding the next CMP, it falls through to the "last interval" branch (arms RXNEIE, sets CMP at fire_tick). Stage 3 then has less than TAIL_GAP to walk the residue. If the residue is large, stage 4 absorbs it (up to GUARD=2 bytes for free, more if slack allows — §3 of chain-crc.md gives ≤ 11 bytes of headroom at the tightest slack).

The hard failure mode would be "RXNE pushes fire CMP late" again — but here the catchup body has just completed an `accumulate_snoop()`, so `snoop_head` is at `write_pos` of the moment the body started. The RXNE queue is shorter than under the original spin-loop design because the tail starts with `bytes_walked` close to its threshold. RXNE masks itself early, queue empties, fire CMP fires on time.

### 3.4 fire_tick already passed when catchup arms

Possible if the dispatcher computes a fire_tick in the past (clock skew, IRQ latency, etc.). The arm code's `next_cmp = now + I` followed by the `next_cmp > fire_tick − TAIL_GAP` check correctly skips the periodic phase. The "set CMP at fire_tick" then arms a CMP in the past; SysTick CMP raises immediately on the next instruction; the recheck-and-run at the end of the arm path (`if (now − fire_tick) as i32 ≥ 0: on_systick()`) catches it. Same behavior as the existing code's "short-wire CMP lands in the past" handling.

---

## 4. State & invariants

### 4.1 Phase enum

    enum FastChainPhase {
        Idle,
        PeriodicCatchup,      // periodic SysTick CMP, no RXNEIE
        TxArmed,              // RXNEIE on, fire CMP armed at fire_tick
        TxStreaming,
        CrcPatched,
    }

`CatchupArmed`, `Snoop`, and `Catchup` from the byte-driven design go away. There's only one "before fire" phase (PeriodicCatchup) and one "tail" phase (TxArmed).

### 4.2 Field ownership

`snoop_head`, `bulk_crc`, and `bytes_walked` are still plain fields on `STATE` (no atomics) by the same same-priority argument from chain-crc-compose.md §2.2. The set of writers shrinks (no on_dma1_ch5 walk) and the field-access pattern is unchanged. The "only one writer running at any instant" property still holds.

### 4.3 What bytes_walked is for

Stage 3's mask threshold (`bytes_walked ≥ N_pred − GUARD`) still uses it. `accumulate_snoop` increments it on every walk regardless of which phase called it, so PeriodicCatchup entries contribute too. By the time RXNE arms, `bytes_walked` is already at `N_pred − bytes_in_tail`, so RXNE folds `bytes_in_tail − GUARD` bytes before masking.

At 3M / 6 bytes tail / GUARD=2, RXNE folds 4 bytes and masks. At 1M / 2 bytes tail, RXNE folds 0 bytes — `bytes_walked` is already at or past the threshold when RXNE first fires, so it masks itself on entry. That's fine; stage 4 handles the residual GUARD.

---

## 5. CPU budget

Per-shot CPU time on chain-CRC handlers, at 3M with N_pred = 30 bytes (INJ=20 Last, wire window ≈ 100 µs):

    periodic catchups       =  (wire − TAIL_GAP) / I  =  80 / 50 = 2 catchups
    walk per catchup        ≈  15 bytes × 0.5 µs     =  7.5 µs
    body overhead           ≈  1.5 µs (trap) + 1 µs (housekeeping) = 2.5 µs
    catchup CPU             ≈  2 × (7.5 + 2.5) = 20 µs

    RXNE entries            =  bytes_in_tail − GUARD = 6 − 2 = 4
    RXNE per entry          ≈  3 µs (trap + walk-1)
    RXNE CPU                ≈  4 × 3 = 12 µs

    fire body               ≈  fire_now (1.2) + walk-GUARD (1) + patch (1) ≈ 3 µs

    total                   ≈  35 µs over 100 µs wire = 35% CPU during snoop

Comparison: the current byte-driven design at the same corner spends roughly 10 µs on HT/TC + 10 µs on catchup + 12 µs on RXNE-tail + 3 µs on fire = ~35 µs. The total is the same; the distribution is different. The time-driven scheme trades "one big catchup spike" for "two smaller catchup spikes plus shorter RXNE tail", and crucially removes the HT/TC contention that made the spike's timing unpredictable.

At 3M INJ=100 (wire ≈ 380 µs, N_pred ≈ 114):

    catchups   =  360 / 50 = 7
    catchup    =  7 × 10 = 70 µs
    RXNE       =  4 × 3 = 12 µs
    fire       =  3 µs
    total      =  85 µs over 380 µs = 22% CPU

Comparable to existing INJ=100 numbers. The bound is "≤ 10 µs per 50 µs window" regardless of INJ — so peak CPU stays at 20% of any one window.

---

## 6. Open questions

- **Single global interval table vs. per-baud constants?** The §2.1 table can live as a `const fn catchup_interval_ticks(baud: BaudRate) -> u32` next to `measured_constants.rs`, or as a match on BaudRate in dxl_fast.rs. Either works; the table values are platform-invariant (purely a function of byte_time × target_bytes).
- **TAIL_GAP per-baud or fixed at 20 µs?** Fixed seems fine — see §2.2. If bench at 1M ever shows RXNE-tail wall-clock issues, scale TAIL_GAP down at low baud (fewer bytes/µs → smaller residue allowed in stage 3).
- **What happens to on_dma1_ch5?** Strip the `bulk_crc` walk and `snoop_head` update; keep the ring-overflow guard (which is a different concern — buffer wraparound during a long pause, not CRC accumulation).
- **Migration cost.** The shipping code has ~300 lines of byte-driven catchup body + bail logic + HT/TC interaction. The replacement is ~50 lines. Worth doing in one commit, not a phased migration.

---

## 7. Why this is the right move

The byte-driven design was elegant on paper (chain-crc-compose.md §4 derives M from first principles) but every refinement to make it robust at one corner regressed another. The pattern was: one tuning knob (M, GUARD, J, bail_margin) per failure mode, each with a different optimum at different bauds.

Time-driven flips the design from "size one CMP perfectly per shot" to "fire many cheap CMPs uniformly". The cost model is linear in wire window, the per-ISR cost is bounded by construction, and there's only one tuning knob (BYTES_PER_INTERVAL) that's a property of the platform (W = 0.5 µs/byte) rather than the shot.

It also removes the architectural dependency on DMA HT/TC for CRC walking, which simplifies the same-priority queueing analysis (chain-crc-compose.md §2.3 collapses from three queue cases to one: catchup-vs-RXNE, and that one is well-behaved because catchup re-arms itself rather than depending on byte boundaries).

The 50 µs anchor matches the kernel's 20 kHz beat: the system already has a periodic 50 µs heartbeat, and chain-CRC now rides on multiples of it instead of beating against it at an unrelated phase.

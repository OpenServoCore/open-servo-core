# DXL Fast Chain CRC: Stage Composition

Companion to [dxl-fast-chain-crc.md](dxl-fast-chain-crc.md). That doc lays out the four-stage thesis and the central design problem (shrink stage 4's wall-clock so it fits TX DMA prefetch slack). This doc covers how the four stages actually **compose** in the shipped implementation: how the catchup margin M is sized per-N_pred, why the RX DMA ring is 256 B and not 512, how the catchup body adapts when stage 1 delays it, and why RXNE keeps firing even when it pushes the fire CMP late.

Read chain-crc.md first. This one assumes you already know what stages 1/2/3/4 are, what `bulk_crc` and `snoop_head` track, and why stage 4 is the one that has to fit slack.

---

## 0. What changed and why

The chain-crc.md §0.2 thesis says stages compose: `{1, 2, 3, 4}` and any subset that pre-folds enough bytes is valid. The shipped code in `start_fast_after` originally treated stage 2 and stage 3 as **mutually exclusive** — it picked one strategy per reply based on `(N_pred, wire_window, slack)`. That worked for most cases but failed at the 3 Mbaud / large-INJ corner, where:

- Stage 3 alone (the "RXNE-tail" path) saturates CPU at 90%+ and the very last RXNE entry can queue the fire CMP, missing patch_crc by enough to corrupt the wire CRC.
- Stage 2 alone (the "catchup" path) overruns the fixed 50 µs `SWITCH_MARGIN_US` for predecessor windows > 100 bytes, pushing fire late.

The fix is to do what the thesis said: **always run all four stages, with each shrinking the work the next inherits**. The design problem reduces to one sizing question (M, the catchup margin), one runtime adaptation (catchup walks what fits if stage 1 ate its budget), and one policy call (RXNE walks to completion even if that pushes fire CMP a few µs late).

This doc is the design for that composition.

---

## 1. The shipping pipeline, end to end

One flow, no decision tree (other than "is there a predecessor or not"):

    chain arm:
        snoop_head = parsed_end & RX_MASK
        bulk_crc   = 0
        M          = switch_margin_ticks(N_pred, byte_time)   # §4
        enable HT/TC_IE                                       # stage 1 on
        SysTick CMP at fire_tick − M

    on_dma1_ch5  (stage 1, ring HT or TC):
        delta = (boundary − snoop_head) & RX_MASK
        bulk_crc = ring_crc(bulk_crc, ring, snoop_head, boundary)
        bytes_walked += delta             # see §9 — counter spans all four stages
        snoop_head = boundary

    on_systick  CatchupArmed body  (stage 2):
        budget_bytes = (fire_tick − now − J) / W_walk
        accumulate_snoop_bounded(budget_bytes)                # §6
        set_phase(TxArmed)
        SysTick CMP at fire_tick
        disable HT/TC_IE                                      # ← stop stage 1
        clear HT/TC_IF                                        # §6.5 — drop stale pending
        enable RXNEIE                                         # stage 3 on

    on_rxne  (stage 3, while phase ∈ {Catchup, TxArmed}):
        walk ring[snoop_head .. write_pos] into bulk_crc
        if bytes_walked ≥ N_pred − GUARD:
            mask RXNEIE                                       # §7

    on_systick  TxArmed body  (stage 4):
        fire_now()
        wait_for_predecessor(N_pred, bound = 1 byte_time)        # §3.2
        accumulate_snoop()    # walks GUARD + any stage-3 leftover from ring
        patch_crc()
        mask RXNEIE (if still armed)

Stages compose: stage 1 folds half-ring chunks during snoop, stage 2 folds whatever ring-residue is left when its CMP fires, stage 3 folds the per-byte tail as predecessor finishes, stage 4 folds the GUARD bytes that were in flight at mask time. Each stage exists to make the next one's work smaller.

The differences from chain-crc.md §8's transition table:
- **Always enter `CatchupArmed`** when `snoop_from = Some`, regardless of `wire_window`. (Short wire just means catchup CMP lands in the past and re-fires immediately via the existing recheck.)
- **Stage 1 is always armed** when there's a snoop. Cleared at catchup body entry, before stage 3 starts.
- **Stage 3 is always armed** at the end of catchup body. Not strategy-conditional.
- **M is dynamic**, sized per-call from N_pred.

---

## 2. PFIC HIGH and ISR serialization (the load-bearing primitive)

Everything in §3–§7 depends on one architectural choice: **all four chain handlers run at the same PFIC priority level**. This is what makes the four stages safe to share `snoop_head` and `bulk_crc` without locks, atomics, or critical sections — and it's what creates the queue floor that §4 has to size around.

### 2.1 The setup (`firmware/ch32/src/statics.rs:178-181`)

    pfic::set_priority(USART1,        Priority::High);    // stage 3 RXNE + IDLE/TC
    pfic::set_priority(DMA1_CHANNEL5, Priority::High);    // stage 1 HT/TC
    pfic::set_systick_priority(       Priority::High);    // stages 2 + 4 (CMPs)
    pfic::set_priority(DMA1_CHANNEL1, Priority::Low);     // ADC pump (separate)

The V006's QingKe core has exactly two priority classes (HIGH and LOW; chain-crc.md §3.3, and `firmware/ch32/src/hal/SAFETY.md`). All four chain-CRC handlers — stage 1 (DMA1_CH5), stage 2 (SysTick), stage 3 (USART1 RXNE), stage 4 (SysTick again) — sit at HIGH. ADC pump and other LOW-priority work runs in the gaps.

### 2.2 What we get from same-priority

**No preemption between stages.** When any chain handler is mid-body, no other chain handler can interrupt it. Concretely:

- Stage 1 walking `ring[snoop_head..boundary]` and writing `snoop_head = boundary` is atomic w.r.t. stage 2/3/4 reads of `snoop_head`.
- Stage 2 walking `accumulate_snoop_bounded` and writing `bytes_walked += delta` is atomic w.r.t. stage 3's read.
- Stage 3's `ring_crc` + `snoop_head = write_pos` is atomic w.r.t. stage 1/4's read.
- Stage 4's `fire_now()` + `accumulate_snoop()` + `patch_crc()` runs without any other chain handler sneaking in.

So `snoop_head`, `bulk_crc`, and `bytes_walked` are plain fields inside `STATE: SyncUnsafeCell<ReplyState>`. No `AtomicU16`, no `compare_exchange`, no critical sections. The "SAFETY: USART1 / SysTick / DMA1_CH5 share PFIC HIGH and never preempt each other" comment at `dxl_fast.rs:446` is doing real work — if any one handler dropped to LOW, that note would become a lie and the design would need to atomize the field accesses.

### 2.3 What it costs: same-priority queueing

The same property that gives us lock-freedom also gives us **queueing**: when an IRQ pends while another HIGH-priority handler is mid-body, the new IRQ waits until the running body returns. This is the load-bearing constraint on M.

The three queue interactions that matter:

    ┌────────────────────────────────────────────────────────────────────┐
    │ Queue case A: HT body in flight when catchup CMP matches.          │
    │   • HT body walks up to ring/2 bytes = up to 64 µs at ring=256.    │
    │   • Catchup CMP queues. Body runs after HT returns.                │
    │   • Floor on M: §4.3 queue_delay_max.                              │
    │                                                                    │
    │ Queue case B: Catchup body in flight when HT/TC IRQ pends.         │
    │   • HT/TC queues. Body runs after catchup returns.                 │
    │   • Catchup body has already updated snoop_head past the boundary  │
    │     in many cases — on_dma1_ch5 then sees snoop_head ≥ target and  │
    │     skips its walk. Safe by construction (dxl_fast.rs:602-612).    │
    │                                                                    │
    │ Queue case C: RXNE body in flight when fire CMP matches.           │
    │   • Fire CMP queues. Fire is late by ~3 µs (one RXNE body).        │
    │   • This is what §7 ("late is better") is about. CRC stays         │
    │     correct because slack is measured from fire_now, not from CMP. │
    └────────────────────────────────────────────────────────────────────┘

### 2.4 The HT/TC disable at catchup body entry

`dxl_fast.rs:491-492` masks HT_IE/TC_IE **first thing** inside the TxArmed body:

    dma::set_tcie(dma::Channel::CH5, false);
    dma::set_htie(dma::Channel::CH5, false);

This isn't bookkeeping — it's load-bearing for ISR contention. From the code comment:

> Mask CH5 TCIE/HTIE FIRST so the walk below is the sole writer — otherwise a CH5 IRQ queued around fire_tick can run AFTER the body returns but with stale ring-walk semantics, corrupting bulk_crc for later shots. Empirically caught: deferring these regressed 3M Last 32B from 123/128 → 91/128.

Same idea applies at the **catchup body** entry in the composed design: once stage 2 has decided to walk, stage 1 can't be allowed to fire concurrently. The composed flow in §1 calls out `disable HT/TC_IE` inside the catchup body for this reason.

### 2.5 Why stages 3 + 4 don't race over `snoop_head`

Stage 3 (RXNE) and stage 4 (TxArmed body) both walk `ring[snoop_head..write_pos]` and advance `snoop_head`. Without same-priority, they'd race. With same-priority:

- If stage 3 fires first (RXNE pending before fire CMP matches), it runs to completion, masks RXNEIE, updates `snoop_head`. Then fire CMP body runs.
- If fire CMP matches first, stage 4 body runs to completion (fire_now → walk → patch). Stage 3 is masked by stage 4's "mask RXNEIE if still armed" at the end.
- If both pend during the body of one, the other runs after.

Each writer leaves `snoop_head` consistent (pointing past everything they've walked), so the next reader sees a coherent view. The lock-freedom is "by construction" — not because the fields are atomic, but because only one writer is running at any instant.

---

## 3. Stage 4's hard deadline

Reproduced from chain-crc.md §2 for context, with corrected numbers and the "late is fine" framing made explicit.

The TX DMA reads `TX_BUF[k]` at roughly `k × byte_time` after `fire_now()` returns. The deadline by which patch_crc must finish writing the trailing two CRC bytes is:

    slack_raw  =  (n − 2) × byte_time

The smallest Fast Last reply we can emit is **n = 5 bytes**, from `write_fast_slot_inner` in `fast.rs`:

    error(1) + id(1) + data(L ≥ 1) + CRC_PLACEHOLDER(2) = 3 + L + 2

so `n_min = 5` when `L = 1`. At 3 Mbaud, `slack_raw = 9.99 µs`. Stage 4's post-fire-now budget:

    fire_now      ~1.2 µs   TX_EN HIGH + DMA CH4 enable
    glue          ~0.5 µs   phase write, dispatch into accumulate_snoop
    walk          0.5 µs × walk_bytes_total
    finalize      ~0.5 µs   crc16 finalize after CRC walk
    write         ~0.5 µs   2-byte store into TX_BUF[n-2..n]

    post_fire_total  =  2.7 µs + 0.5 × walk_bytes_total

Constraint:

    post_fire_total  <  slack_raw
    walk_bytes_total <  (slack_raw − 2.7) / 0.5

The walk inside stage 4 covers **both** the predecessor residue still in the ring AND our own (n−2) bytes for computing the final CRC. So:

    walk_bytes_total  =  residue + (n − 2)

At worst case (n=5, 3 Mbaud):

    walk_bytes_total ≤ (9.99 − 2.7) / 0.5 = 14
    residue          ≤ 14 − 3 = 11 bytes

**At the tightest slack on the platform, stage 4 can absorb up to ~11 predecessor bytes** before patch_crc misses the deadline. GUARD=2 fits comfortably; that leaves **9 bytes of headroom** for "stage 2 didn't get to" or "stage 3 didn't finish in time" residue.

That 11-byte number assumes the wait term in §3.2 is zero — see §3.2 for the worst-case tightening when the last predecessor byte hasn't landed yet at fire CMP.

### 3.1 Subtlety the parent doc glossed over

The deadline is **patch_crc completion**, not ISR exit. Anything stage 4 does after the second CRC byte is written doesn't count against slack:

    fire_now()                     ─┐
    accumulate_snoop()              │  ← these race the deadline
    patch_crc() ─ finalize + write ─┘
    ────────────── ↑ patch_crc completes here. Deadline cleared ───────
    set_phase(TxStreaming)          ─┐
    set_phase(CrcPatched)            │
    mask RXNEIE                      │  ← these are off-critical
    CrcPatchDeadlineMiss check       │
    PreviousSlotTimeout check        │
    SlotTimingMiss check            ─┘

The `dxl_fast.rs:498-527` ordering puts all the "race the deadline" work first, then everything else after. The post-patch work is observation and bookkeeping — its timing doesn't change what lands on the wire.

### 3.2 The post-fire busy-wait

Stages 1+2+3 walk every byte the predecessor has DMA'd into the ring **by the time stage 4 fires**. That leaves a corner: the very last predecessor byte (or two — up to GUARD) may still be mid-stop-bit on the wire when fire CMP matches on time. Concretely, when the predecessor is running at its nominal rate (no slack ahead of us), `fire_tick` lands roughly at the boundary between its last stop bit and our first start bit — and that last byte is **not yet at NDTR**:

- RXNE didn't fire for it (it's still clocking in at RDR — RXNE asserts on the *next* sample).
- DMA hasn't written it to the ring (NDTR advances only after RDR transfers).
- `accumulate_snoop` walking `ring[snoop_head..write_pos]` would skip it.

patch_crc would then run over an incomplete predecessor count → wrong wire CRC, host drops the frame. The parent doc covered this with `wait_and_accumulate_tail`; this design keeps that idea but folds it into the same one-flow pipeline.

The fix: after `fire_now()` and before `accumulate_snoop()`, stage 4 busy-waits for `write_pos` to reach the target byte count, bounded by 1 byte_time:

    # Target is N_pred − 1, not N_pred. predict_chain_pure floors twice and
    # decide_rxne_tail_pure biases the result up by 1, so N_pred is a hair
    # high by construction. Using N_pred verbatim would time out on every
    # reply (the +1th byte never arrives). The "− 1" cancels the prediction
    # bias; the actual count never falls below this without a true host-side
    # drop, which PreviousSlotTimeout correctly catches.
    wait_target = N_pred − 1
    deadline = systick::ticks() + byte_time_ticks
    loop:
        ring_unwalked = (write_pos − snoop_head) & RX_MASK
        if bytes_walked + ring_unwalked >= wait_target: break
        if systick::ticks() − deadline >= 0:
            report_fault(PreviousSlotTimeout)
            break
    accumulate_snoop()
    patch_crc()

`bytes_walked` here is the running counter incremented by every stage (1+2+3) as it folds bytes — see §9. Stage 1 advances it in addition to `snoop_head`, so the check sees the full predecessor count, not just what RXNE walked.

Predecessor bytes ship back-to-back at byte_time cadence, so once the wait starts, each pending byte lands within 1 byte_time of the previous one. The **absolute** wait is bounded by 1 byte_time, not GUARD × byte_time — if the byte that triggered the wait lands at `+ ε`, the next would-be byte was already past mid-shift and lands at `+ ε + byte_time`. So worst case is the wait covers exactly one byte_time.

If the predecessor is genuinely missing bytes (host bug, dropped slot), the wait times out and `PreviousSlotTimeout` increments. Stage 4 still patches what it has and fires the CRC over the partial count — the wire stays consistent (host drops the frame, fast recovery on next request) rather than going silent.

### 3.3 Slack budget split by wait outcome

The wait happens **concurrently with TX byte 0 shifting out** — `fire_now()` already enabled DMA CH4. So the wait eats into the same `(n−2) × byte_time` slack budget §3 derived; it doesn't open a new budget. Two cases:

- **Typical (wait = 0)**: predecessor finished before fire CMP. RXNE walked the last byte already, or it landed at NDTR before stage 4 got to the wait check. The wait loop's first iteration exits — adds essentially zero cost. §3's main derivation holds: `walk_bytes_total ≤ 14`, residue ≤ 11 bytes.
- **Worst-case (wait ≈ 1 byte_time)**: predecessor's last byte arrives mid-wait. At 3 Mbaud the wait costs 3.33 µs of slack. New budget:

      walk_bytes_total ≤ (slack_raw − 2.7 − 3.33) / 0.5  =  7.92 ≈ 7 bytes
      residue          ≤ 7 − 3  =  4 bytes

At n=5 the worst-case residue budget tightens to 4 bytes — still covers GUARD=2 plus 2 bytes of slop from "stages 1+2+3 didn't quite finish." At n=6 the budget reopens to ~7 residue bytes; at n=7+ the budget is comfortable. This is consistent with the wire-format constraint in §3.4 — n=5 + worst-case-wait is the tightest corner the design supports, and it's tight by 0 bytes (no headroom). At n=6 there's real room.

Note: the wait is not chosen — it's a runtime measurement. The "typical" budget is what the design targets; the "worst-case" budget is what the design **survives**.

### 3.4 Reply size as a wire-format constraint

For n < 5, slack_raw underflows and the design can't function (the `tx_len < 4` guard in `decide_rxne_tail_pure` catches this defensively). For n = 5 at 3 Mbaud the design works in the typical case (residue ≤ 11) but **has zero headroom in the §3.3 worst case** (wait + residue = 4-byte budget, GUARD alone is 2 bytes). For n ≥ 6 at 3 Mbaud, the worst-case budget reopens enough that worst-case wait + worst-case HT-queue stacks still fit.

This is a **wire-format design constraint**, not a chain-CRC knob: the dispatcher must not emit a Fast Last reply with `data.len() ≤ 1` at 3 Mbaud if reliability matters. Bench tests should focus on n ≥ 6 corners as the supported envelope.

---

## 4. The catchup margin M, derived

M is the time between catchup CMP and fire CMP. Pick it too small and the catchup body's walk doesn't finish before fire. Pick it too big and stage 3 has more bytes to walk per RXNE, burning CPU on per-byte ISR entries when stage 2 could have folded them in bulk. The right M lets catchup walk exactly its share — the bytes accumulated since chain arm — and no more.

### 4.1 Setup

Variables (all in µs unless noted):

    N_pred       predecessor byte count (predicted from wire_window × bytes_per_us)
    byte_time    1 byte's wire time = 10 / baud_hz × 1e6
    W_walk       per-byte CRC walk cost, bench-measured = 0.5 µs/B on V006
    W            wire_window = N_pred × byte_time

When catchup CMP fires at `fire_tick − M`, the predecessor has been on the wire for `(W − M)` µs, so:

    bytes_in_ring_at_catchup  =  (W − M) / byte_time
    catchup_walk_time         =  (W − M) × W_walk / byte_time

### 4.2 The safety constraint

Catchup body must finish before fire CMP matches. Set equality for the **tightest safe M** — the value that walks every µs available before fire and leaves no idle time:

    (W − M) × W_walk / byte_time  =  M
    W × W_walk  =  M × (byte_time + W_walk)

    ┌──────────────────────────────────────────────┐
    │   M_opt  =  W × W_walk / (byte_time + W_walk) │
    │          =  N_pred × W_walk × byte_time       │
    │             / (byte_time + W_walk)            │
    └──────────────────────────────────────────────┘

Equality is both tightest-safe **and** minimum-stage-3-load: any smaller M means catchup overruns and pushes fire late; any larger M means more bytes arrive in the M window and stage 3 walks them per-byte (3 µs of ISR per byte) instead of stage 2 walking them in a block (0.5 µs/byte). Linear in N_pred at any given baud.

### 4.3 The queue cap (from §2.3 case A)

The formula above assumes catchup body runs the instant its CMP matches. But stage 1 (HT/TC) and the catchup CMP share PFIC HIGH and serialize — if HT fires just before catchup CMP, catchup queues behind HT's body. The longest possible HT body walks `ring/2` bytes and takes:

    T_HT_body_max  =  (ring/2) × W_walk

During HT body, DMA keeps writing — by HT body exit, `write_pos` has advanced. Catchup then walks those newly-arrived bytes:

    T_catchup_post_HT  =  T_HT_body_max × W_walk / byte_time

Total queue floor:

    queue_delay_max  =  T_HT_body_max × (1 + W_walk / byte_time)
                     =  (ring/2 × W_walk) × (byte_time + W_walk) / byte_time

This is the absolute worst-case M for the **defensive** sizing approach. The actual implementation uses adaptive catchup (§6) to make this a soft ceiling rather than a hard one.

### 4.4 The full formula

    M  =  min( M_opt(N_pred), M_CAP ) + J

where:

- `M_opt(N_pred)` is §4.2's safe-walk formula, sized per-call from the predicted predecessor count.
- `M_CAP` is `queue_delay_max` from §4.3.
- `J` is a jitter cushion (~5 µs) covering PFIC trap entry variance, the +1 bias on N_pred from floor-twice rounding (`predict_chain_pure` floors twice; `decide_rxne_tail_pure` biases up by 1), and walk-cost noise.

### 4.5 Sanity table at 3 Mbaud, ring=256 (M_CAP ≈ 73 µs)

    | N_pred | W (µs) | M_opt (µs) | M (µs) | catchup walks | stage 3 entries |
    |--------|--------|------------|--------|---------------|-----------------|
    |     1  |   3.3  |       0.4  |    5   | ~0 B          | 0               |
    |    11  |  36.6  |       4.8  |   10   | ~8 B          | 1               |
    |    42  | 140    |      18.3  |   23   | ~35 B         | 5               |
    |   138  | 460    |      60    |   65   | ~115 B        | 17              |
    |   256  | 853    |     111    |   73 (cap) | ~221 B    | 22              |
    |   500+ | 1666+  |     200+   |   73 (cap) | ≤ 128 B   | 22              |

For N_pred up to ~150 at 3 M, M scales linearly with the predecessor wire window. Past that, stage 1 absorbs the extra and M plateaus at M_CAP.

---

## 5. Ring size: 256, not 512

The original ring was 512 B for two reasons that no longer apply:

- An older "no stage 1" design needed the ring to bound the entire snoop window directly. Stage 1's incremental folding made that constraint disappear (chain-crc.md §7 Constraint C clears trivially at any plausible M).
- The doc's Constraint A (stage 2 walks ≤ M) used ring/2 = 256 B as the worst case, but that's an upper bound, not a tight one.

The new lever for picking ring size is **the stage-1 queue floor from §4.3**.

### 5.1 Queue floor at three ring sizes (3 Mbaud)

    | ring | T_HT_body_max | queue_delay_max | stage 3 entries during M |
    |------|---------------|-----------------|--------------------------|
    | 512  |   128 µs      |   147 µs        | 42                       |
    | 256  |    64 µs      |    74 µs        | 22                       |
    | 128  |    32 µs      |    37 µs        | 11                       |

Cutting ring in half cuts T_HT_body_max in half, and the queue floor follows. Fewer stage-3 entries per chain reply means more headroom for the ADC pump (LOW priority — starved while any HIGH handler is running).

### 5.2 Why not just go to 128?

Two reasons:

- **Main-loop polling cadence**: the ring has to hold bytes accumulated between `services.poll()` invocations. Polling is wake-driven (PFIC IRQs wake `wfi`), and HT/TC fires every `ring/2` bytes — so the natural floor is "ring/2 bytes per poll." But the dispatcher's parse + dispatch can take ~100 µs in worst-case packet handling; at ring=128, that's 32 byte_times at 3 M (= 107 µs available — tight). At ring=256 it's 64 byte_times (213 µs — comfortable).
- **Diminishing returns**: dropping queue_delay_max from 74 µs to 37 µs only matters if the formula M_opt is regularly above the cap. At 3 Mbaud with N_pred ≤ 150, M_opt is below 74 µs and the cap doesn't kick in. Most workloads see ring=256's cap and ring=128's cap equally rarely.

Ring=256 is the sweet spot: cuts queue delay in half vs 512, keeps polling cadence margin comfortable.

### 5.3 The dxl-protocol crate doesn't care

Verified: nothing in `firmware/lib/dxl-protocol/` depends on `DXL_RX_BUF_LEN`. The protocol's frame-size cap (249 B) is driven by **scratch** (`DXL_SCRATCH_LEN = 256` in `core/services/dxl/api.rs`), not the DMA ring. Stage 1/2/3/4 walk the ring directly — `accumulate_snoop` / `ring_crc` never go through scratch. So shrinking the ring is an isolated change in `firmware/ch32/src/statics.rs`.

---

## 6. Adaptive catchup: walk what fits

The queue cap above says M ≥ 73 µs covers worst-case HT queueing. But that's defensive — most of the time HT doesn't queue catchup, and reserving 73 µs of pre-fire window every reply wastes stage 4's potential to start earlier. We want the **typical** case to use the smaller `M_opt(N_pred)`, with a runtime fallback when queueing actually happens.

The fallback is: at catchup body entry, **measure how late we are and walk only what fits**. This converts the queue floor from a hard ceiling on M into a graceful-degradation behavior.

### 6.1 The bounded walk

`accumulate_snoop` (unchanged) walks all bytes between `snoop_head` and `write_pos`. The new variant takes a byte cap:

    fn accumulate_snoop_bounded(max_bytes: u32) {
        let write_pos = current_rx_write_pos();
        let delta_full = (write_pos − snoop_head) & RX_MASK;
        if delta_full > DXL_RX_BUF_LEN / 2 {
            // Ring lapped between walks — snoop_head is now past data DMA already
            // overwrote. bulk_crc is unrecoverable. Abort the chain (§6.5).
            report_fault(UnexpectedByteCount);
            cancel();
            return;
        }
        let delta = min(delta_full, max_bytes);
        if delta == 0 { return; }
        let stop = (snoop_head + delta) & RX_MASK;
        bulk_crc = ring_crc(bulk_crc, ring, snoop_head, stop);
        bytes_walked += delta;
        snoop_head = stop;
    }

This is the only new primitive. Stage 4's existing `accumulate_snoop()` (unbounded) stays — it picks up whatever stage 2 left in the ring — and gets the same lap-detection guard at its top.

Lap threshold of `ring/2` matches stage 1's HT cadence: HT fires every `ring/2` bytes, so under normal operation no stage should ever see an unwalked delta larger than that. A delta exceeding `ring/2` means stage 1 missed a wrap, which means stage-1 IRQ latency stacked beyond `ring/2 × byte_time` (213 µs at 3 Mbaud, ring=256). That's structurally impossible at PFIC HIGH unless something is very wrong — but the check is one compare and pays nothing in the happy path.

### 6.2 The catchup body

    fn on_systick_catchup_body() {
        let now = systick::ticks();
        let to_fire_ticks = (fire_tick − now) as i32;
        let walk_budget_ticks = (to_fire_ticks − WALK_JITTER_TICKS).max(0) as u32;
        let walk_budget_bytes = walk_budget_ticks / CRC_WALK_TICKS_PER_BYTE;

        accumulate_snoop_bounded(walk_budget_bytes);   // §6.1

        set_phase(TxArmed);
        systick::set_cmp(fire_tick);
        systick::set_irq(true);
        dma::set_htie(CH5, false);                     // stage 1 off (§2.4)
        dma::set_tcie(CH5, false);
        dma::clear_htif(CH5);                          // §6.5 — drop pending HT/TC IF
        dma::clear_tcif(CH5);                          //         so next chain doesn't see stale flags
        usart::set_rxne_irq(USART1, true);             // stage 3 on

        // set-and-recheck for the in-past case (dxl-rx-timing.md §8.3)
        let now = systick::ticks();
        if (now − fire_tick) as i32 >= 0 { on_systick(); }
    }

The body reads SysTick **once at entry** to learn how queued it is. If HT just ate 64 µs of the M budget, `walk_budget_ticks` is correspondingly smaller, and the bounded walk leaves bytes in the ring for stage 4 to pick up.

### 6.3 Where the leftover bytes go

If catchup_bounded walked `K < delta_full` bytes, then `delta_full − K` bytes are still in the ring between `snoop_head` and `write_pos`. Two paths cover them:

- **Stage 3 (RXNE) does NOT cover them.** RXNE only fires for bytes arriving at RDR after IE is enabled. Bytes that already DMA'd into the ring are past RDR — they don't generate RXNE. Common misunderstanding worth pinning.
- **Stage 4 covers them via its own `accumulate_snoop()`** before `patch_crc()`. Stage 4 walks `ring[snoop_head .. write_pos]` unconditionally, including anything stage 2 left behind.

So the budget check is: `stage 4 absorbs leftover + GUARD ≤ 11 bytes` (§3 derivation). Under worst-case HT queueing at ring=256, M=73 µs:

- HT body ate 64 µs.
- Catchup body has ~9 µs of walk budget. Walks ~17 bytes.
- Bytes accumulated during HT body = 64 × 0.3 = 19. Catchup walks all of them (17 ≤ 19 ish — actually slightly under; ~2 bytes leftover).
- Stage 4 walks GUARD (2) + leftover (2) = 4 bytes. Fits the 11-byte budget with 7 bytes of headroom.

### 6.4 What if walk_budget_ticks is zero or negative?

If `to_fire_ticks ≤ WALK_JITTER_TICKS` when catchup body runs (HT was running long enough that catchup is at or past `fire_tick`), the bounded walk does nothing. The body still arms fire CMP (which immediately recurses via the recheck), and stage 4 walks everything stage 2 missed.

At 3 Mbaud worst case (HT body just barely ended, ring residue ≈ 19 bytes), stage 4 walks 19 + own = 22 bytes total. Cost: `22 × 0.5 + 2.7 = 13.7 µs`. Already misses the 9.99 µs deadline by ~4 µs at n=5. If the §3.2 busy-wait *also* fires (the predecessor's last byte was still mid-stop-bit at fire CMP because the wire window stretched to fire_tick), the total grows to `13.7 + 3.33 = 17 µs` — deadline missed by ~7 µs, which is roughly the trailing 2 bytes of the wire CRC slot getting garbled.

This is a real failure case for n=5 specifically, but only under stacked worst-case (HT queue **and** large N_pred **and** n=5 **and** worst-case wait). For n ≥ 6 the budget reopens enough to absorb; for n ≥ 7 it's comfortable. We accept this as the platform limit at 3 Mbaud min-payload-min-INJ corner — see §3.4.

### 6.5 Flag hygiene and lap safety

Two design rules that don't fit neatly elsewhere but are load-bearing for getting the implementation correct:

**Clear HT/TC IF when you disable HT/TC IE.** At the end of catchup body (and on `cancel()`), the sequence is:

    set_htie(false); set_tcie(false);   # mask new IRQs
    clear_htif(); clear_tcif();          # drop any pending flag

Without the clear, an HT_IF or TC_IF that latched between the last IRQ and the disable stays set indefinitely. On QingKe V2A, IF without IE doesn't vector — but the next chain arm that re-enables IE will instantly fire on the stale flag, walking `ring[old_snoop_head .. boundary]` with the *new* chain's `snoop_head`. That walk either folds garbage or skips (case-B logic) — non-deterministic depending on alignment.

Same rule applies to RXNEIE — when stage 4 masks RXNEIE at end of body, any pending RDR data is harmless (next chain re-arms with a fresh `snoop_head` past it). No clear needed there; just don't accidentally re-enable RXNEIE without resetting snoop_head first.

**Lap-detection in `accumulate_snoop[_bounded]`.** Both the bounded (§6.1) and unbounded variants in stage 4 check `delta > ring/2` at top and fault out via `UnexpectedByteCount + cancel`. Stage 1's HT cadence makes lapping impossible under normal operation — but ISR latency stacks (HT body taking 64 µs, then catchup queueing for 73 µs, then the catchup body itself walking for tens of µs at high N_pred) can creep toward the threshold. The check is one compare and turns a silent CRC corruption into an observable fault.

### 6.6 Why this is better than a fixed M

Fixed M = queue_delay_max sized for worst case (73 µs at ring=256, 3 M) burns the full M window for **every** chain reply, even though the queue almost never happens. Stage 3 entries scale with M (each byte_time of M is one potential RXNE entry), so a fixed-large M means a fixed-high CPU floor for stage 3 — burning the ADC pump's budget on chains that didn't need stage 3 at all.

Adaptive catchup lets typical replies use the smaller `M_opt(N_pred)` (e.g., 10 µs for N_pred=11), with the worst-case overhead only paid when HT actually queues catchup.

---

## 7. RXNE: keep firing, even when late

The single most important policy in this design, made load-bearing by §2's same-priority queue model.

### 7.1 The race we used to fear

Chain-crc.md §5.1 introduced GUARD=2 as the "stop short of N_pred so the last RXNE doesn't collide with fire CMP" rule. The mask threshold is `bytes_walked ≥ N_pred − GUARD`; bytes beyond that fall through to stage 4.

The fear was queue case C from §2.3: if RXNE fires for the byte at position `N_pred − 1` (the second-to-last predecessor byte), and the fire CMP for `fire_tick` matches mid-RXNE-body, fire CMP queues behind on_rxne. Fire is late by one RXNE body (~3 µs).

### 7.2 The policy: late fire is better than corrupted CRC

A 3 µs late fire **doesn't corrupt CRC**:

- The TX prefetch slack budget is measured from `fire_now()` onward (§3). If `fire_now()` happens 3 µs late, DMA shifts start 3 µs later — and stage 4 has the **same** `slack_raw` budget relative to fire_now. CRC stays correct.
- All predecessor bytes get folded into `bulk_crc` via three mutually exhaustive paths: stage 3 (RXNE) walks them as they arrive at RDR; the §3.2 busy-wait catches anything mid-stop-bit at fire CMP; `accumulate_snoop` walks anything already in the ring that stages 1+2 didn't reach. Together these cover every byte of the predecessor's wire window, so the CRC written into TX_BUF[n−2..n] is mathematically correct.

What a late fire **does** do:
- **Widen the inter-slot coalesce gap** on the wire. The host sees a tiny gap (≤ 1 byte_time) between predecessor end and our reply start. DXL 2.0 doesn't mandate zero-gap coalesce; the host parser handles small gaps.
- **Increment `SlotTimingMiss`** if the late amount exceeds 1 byte_time. That's a diagnostic, not a wire-correctness failure.

The alternative — having stage 3 abort or refuse to walk close to fire_tick — would leave the byte unwalked, and stage 4's accumulate_snoop would still pick it up. The only difference is whether stage 3 fires once more or not. Either way the byte is folded. So aborting RXNE saves nothing in terms of CRC correctness and only hides a diagnostic.

### 7.3 What this means in code

`on_rxne` (`dxl_fast.rs:625-677`) walks `ring[snoop_head .. write_pos]` to completion, then checks the mask threshold, then writes RXNEIE off. **No early abort, no time-budget check, no skip-if-near-fire.**

The same policy applies to the body inside `on_systick TxArmed` — it runs `fire_now() → walk → patch_crc()` to completion, even if other IRQs are pending. PFIC same-priority queues them; they all get serviced after stage 4 returns.

### 7.4 The role of GUARD in this design

With "late is fine" as policy, what does GUARD still do? Two things:

1. **It bounds stage 4's deadline pressure.** Stage 4's residue is at most GUARD bytes from stage 3's mask (plus any bytes catchup didn't get to from §6). GUARD = 2 keeps stage 4's walk small enough to fit in slack.
2. **It reduces the frequency of queue case C.** Without GUARD, stage 3 might walk N_pred bytes one-at-a-time, with the last walk's timing essentially adjacent to fire CMP. With GUARD = 2, stage 3 stops 2 bytes early — `~6.66 µs` at 3 Mbaud — well before fire CMP normally matches. Late fires from stage 3 collisions become rare instead of routine.

GUARD = 2 is the same value as chain-crc.md §5.1 (in-flight race + jitter cushion). The justification holds: one byte for "stop bit clocking when threshold check runs," one byte of jitter cushion for "RXNE entry landing slightly past the boundary."

---

## 8. The full composed flow

Putting §1–§7 together, the chain timeline at 3 Mbaud with a representative `N_pred = 138` (3M Last 1B × 128B INJ — the bench-failing corner):

    t=0       chain arm. snoop_head = parsed_end & 255.
              M = switch_margin_ticks(138, byte_time_3M) ≈ 65 µs.
              HT/TC_IE on. SysTick CMP at fire_tick − 65.

    t=0..427  predecessor sends 128 bytes (at HT trigger).
              ── Stage 1 ── HT fires somewhere in this window if snoop_head
                            crosses ring/2 (snoop_head + 128 > 128, depends on
                            alignment). If HT fires, walks up to 128 bytes,
                            updates snoop_head to boundary.

    t=395     SysTick CMP at fire_tick − 65 matches.
              (If HT body is in flight here, catchup queues — §4.3, §6.)
              ── Stage 2 ── on_systick CatchupArmed body:
                            walk_budget = (fire_tick − now − J) / W_walk
                            accumulate_snoop_bounded(walk_budget)
                            → walks ring[snoop_head .. write_pos], up to budget
                            Bytes pending after stage 1 might be ≤ 64 (just
                            since last HT). Walk time ≤ 32 µs, fits 65 µs.
              ── Stage 1 off (HT/TC_IE cleared — §2.4)
              ── Stage 3 on (RXNEIE enabled)

    t=395..460  predecessor sends remaining ~20 bytes.
              ── Stage 3 ── on_rxne fires per byte (or every few bytes if
                            RXNE doesn't keep up — self-recovering),
                            walks ring[snoop_head .. write_pos] each entry.
                            At bytes_walked ≥ 136 (N_pred − GUARD = 138 − 2),
                            mask RXNEIE.
                            Runs to completion regardless of timing — §7.

    t=460     SysTick CMP at fire_tick matches.
              (If RXNE body is in flight here, fire queues briefly — §7.2.)
              ── Stage 4 ── on_systick TxArmed body:
                            fire_now()
                            accumulate_snoop()  ← walks GUARD bytes (≤2)
                            patch_crc()         ← (n−2) own + finalize + write
                            ─────── deadline cleared here (§3.1) ───────
                            mask RXNEIE if still on
                            (off-critical: diagnostics, phase writes)

    t=460..463  TX shifts our reply. DMA reads TX_BUF[n−2] at t ≈ 463 (= 460 + 3 × byte_time at n=5). patch_crc completed earlier, within slack budget.

At any N_pred where M_opt > M_CAP, the catchup body runs with reduced walk budget (queued behind HT), `accumulate_snoop_bounded` walks fewer bytes, stage 4 picks up the residue. The pipeline stays valid; it just shifts where the work happens.

---

## 9. Variables and constants

The runtime values that appear in code and bench tooling. Keep these synchronized between code, doc, and bench scripts.

    DXL_RX_BUF_LEN              = 256   bytes   (was 512)
    CRC_WALK_TICKS_PER_BYTE     = 24    ticks   (0.5 µs at 48 MHz)
    SWITCH_JITTER_TICKS         = 240   ticks   (5 µs, J in §4.4)
    GUARD                       = 2     bytes   (chain-crc.md §5.1, §7.4)
    PREDECESSOR_WAIT_BOUND      = 1 byte_time (DXL_BYTE_TIME_TICKS atomic; ~3.33 µs at 3 M)
                                          (§3.2 busy-wait cap; PreviousSlotTimeout on overrun)
    PREDECESSOR_WAIT_TARGET     = N_pred − 1   (§3.2; cancels predict_chain_pure's +1 bias)
    LAP_THRESHOLD               = DXL_RX_BUF_LEN / 2 = 128 bytes
                                          (§6.1, §6.5 — unwalked delta exceeding this
                                           ⇒ UnexpectedByteCount + cancel)
    M_CAP_TICKS                 = (ring/2) × W_walk × (byte_time + W_walk) / byte_time
                                ≈ 73 µs at ring=256, 3 Mbaud
    POST_FIRE_FLOOR             ~2.7 µs (fire_now + glue + finalize + write,
                                         excluding the per-byte walk and §3.2 wait)

State fields on `ReplyState::Chain`:

    bytes_walked: u32   total predecessor bytes folded into bulk_crc, across stages
                        1+2+3+4. Every stage that walks ring bytes increments it. u16
                        would suffice up to ~65 kB chains (DXL caps far below) but u32
                        costs nothing and saves a future-proofing worry.
    snoop_head:   u16   ring offset of the next unwalked byte. Wraps with RX_MASK.
    bulk_crc:     u16   running CRC over [start, snoop_head). Finalized in stage 4
                        over our own (n−2) bytes before patch_crc writes the trailing
                        two CRC bytes.
    expected_predecessor_bytes: u16   = N_pred. Set at chain arm; used as the wait
                                       target's source (target = N_pred − 1).

Pure functions, unit-tested in `dxl_fast.rs::tests`:

    switch_margin_ticks(n_pred, byte_time_ticks) -> ticks
        # §4.4 formula with cap and jitter

    accumulate_snoop_bounded(max_bytes)
        # §6.1, walks up to max_bytes from snoop_head

Stage entries with their where-to-find code anchors:

    | Stage | Code site                            | Walks                       | Priority |
    |-------|--------------------------------------|-----------------------------|----------|
    | 1     | on_dma1_ch5 (HT/TC body)             | ring[snoop_head..boundary]  | HIGH     |
    | 2     | on_systick CatchupArmed/Snoop body   | bounded(N) via §6.1         | HIGH     |
    | 3     | on_rxne                              | ring[snoop_head..write_pos] | HIGH     |
    | 4     | on_systick TxArmed body              | §3.2 wait → ring[snoop_head..write_pos] + own (n−2) | HIGH |

All four at PFIC HIGH — see §2.1 and `statics.rs:178-181`. ADC pump (DMA1_CH1) at LOW, gets the gaps.

---

## 10. What this design closes, what it doesn't

**Closes:**
- The 3 Mbaud large-INJ failure mode where stage 3 alone saturates CPU and the last RXNE collides with fire CMP. Stage 2 now does the bulk of the work; stage 3 is a short tail.
- The stage 2 overrun for predecessor windows > 100 bytes at fixed M=50. The new M scales with N_pred.
- The opaque "stage 1 always armed" claim from chain-crc.md §3. It's now genuinely always armed during snoop (no strategy gating), and explicitly disabled in §6.2 at catchup body entry to prevent stage 1 racing stages 3/4.
- The HT body queue floor on M (§4.3) is no longer a hard ceiling; adaptive catchup (§6) handles it as graceful degradation.

**Does not close:**
- Worst-case n=5 + worst-case HT queue + worst-case N_pred + worst-case §3.2 wait. §6.4 spells this out. At 3 Mbaud min-payload-min-INJ, patch_crc can miss the deadline by a few µs. The fix is "use n ≥ 6 for replies in chains" — a wire-format constraint, not a chain-CRC design knob (§3.4).
- The structural fire floor from chain-crc.md §12. Coalesce gap jitter is independent of CRC correctness.
- Per-baud `J` tuning. The 5 µs jitter cushion is conservative; bench can tighten it (task #74).
- The RXNEIE composer for the framing-mode FSM (chain-crc.md §10). Phase B work, doesn't block this design.

---

## 11. Bench validation plan

What we expect to measure once this is shipped, in order of decision importance:

1. **CRC pass rate at 3 Mbaud × {Last 1B, Last 4B, Last 32B} × INJ_LEN ∈ {0, 4, 32, 128, 256}**. The matrix that's currently failing. With the new pipeline, expect 100% at all corners except the n=5 + HT-queue-worst alignment (rare; should be visibly rare in scope captures).
2. **SlotTimingMiss counter** under sustained traffic. Some queue-case-C events are *expected* (§7.2 explicitly allows them). Expected non-zero but bounded; spikes should correlate with high INJ_LEN and large N_pred.
3. **CrcPatchDeadlineMiss counter**. Expected zero except the n=5 stacked-worst corner.
4. **ADC pump CPU budget**. Stage 3 entries dropped from `~N_pred − GUARD` per chain (old RxneFull path) to `~M / byte_time − GUARD` (new path, capped at ~22 entries at 3 M). ADC pump should see noticeably less starvation.
5. **Scope trace**: catchup body entry time vs `fire_tick − M`. If it consistently lands within J=5 µs, the cushion is right-sized. If it lands earlier, J can shrink. If HT queueing is visible (catchup entry pushed past `fire_tick − M`), adaptive catchup should be visibly compensating.

Items 4–5 are the bench-side tasks tracked under #74. Items 1–3 are the "did this actually fix the failing corner" checks.

---

## 12. Cheat sheet

When in doubt, the rules to fall back on:

- **All four chain handlers run at PFIC HIGH.** That's what makes `snoop_head`/`bulk_crc`/`bytes_walked` lock-free across stages. If you find yourself adding atomics, check whether something dropped to LOW.
- **Same-priority means queueing, not racing.** A handler in flight blocks others until it returns. Three cases matter: HT blocks catchup (§2.3 A), catchup blocks HT (§2.3 B, safe by construction), RXNE blocks fire (§2.3 C, deliberately allowed).
- **M scales with N_pred**, capped by stage 1's queue floor. Not a fixed constant.
- **Stage 1 always armed during snoop**, disabled at catchup body entry (§2.4, §6.2) so its walk is uncontested.
- **Stage 2 walks what fits**, leaves the rest for stage 4 (§6).
- **Stage 3 fires until it self-masks at N_pred − GUARD**, and runs to completion even if it lands near fire_tick. Late fire is fine; corrupted CRC is not (§7).
- **Stage 4's deadline is patch_crc completion**, not ISR exit (§3.1). Diagnostics, phase writes, RXNEIE mask all happen post-deadline.
- **Stage 4 busy-waits for the predecessor's last byte** between fire_now and accumulate_snoop, bounded by 1 byte_time, PreviousSlotTimeout on overrun (§3.2). Wait target is `N_pred − 1`, not `N_pred` — predict_chain_pure biases up by 1.
- **`bytes_walked` is the cross-stage counter.** Every stage that folds bytes into bulk_crc increments it. Stage 1 must increment, not just advance snoop_head — the §3.2 wait target depends on it (§9).
- **Disable HT/TC_IE and clear HT/TC_IF together** (§6.5). Stale IF without IE re-fires on next chain re-arm.
- **`delta > ring/2` ⇒ UnexpectedByteCount + cancel** (§6.1, §6.5). Cheap guard against ring lap; impossible in happy path but observable when ISR latency stacks.
- **slack budget runs from fire_now**, not from fire_tick CMP-match. A late fire shifts the entire downstream pipeline by the same amount — CRC stays correct.
- **Ring=256**, not 512. The protocol doesn't care (§5.3). Stage 1 queue delay halves.
- **n ≥ 6 for safe operation at 3 Mbaud**. n=5 has zero §3.3-worst-case headroom; n=6 reopens it.

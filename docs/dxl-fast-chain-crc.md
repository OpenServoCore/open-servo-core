# DXL Fast Last-Slave Chain CRC on the CH32V006

Companion to [dxl-rx-timing.md](dxl-rx-timing.md). Read that first — this doc assumes you already know what "the wire-end timestamp," "SysTick CMP fire," and "the V006's two PFIC priorities" mean.

When we're the last slave in a DXL Fast Sync/Bulk Read, we have to compute a CRC over bytes we didn't generate — every byte the predecessor slaves' replies put on the wire — and stitch our own bytes onto the end. The CRC is accumulated across up to four cooperative stages that pre-fold predecessor bytes before the fire deadline; the design problem is sizing those stages so the final stage fits inside the V006's tight TX DMA prefetch slack without any earlier stage delaying the fire itself.

---

## 0. Central thesis: minimize stage 4

The Fast Last-slave chain CRC accumulates predecessor wire bytes across up to **four CRC accumulation stages**. They exist for a single goal: **shrink stage 4's wall-clock so when it fires it only has a handful of bytes left to walk**. Stage 4 is the one whose wall-clock fights the TX DMA prefetch deadline (`slack = (tx_len − 2) × byte_time − post_fire_floor`). Stages 1, 2, and 3 are cooperative pre-folders — every stage's existence is justified by this objective, not by any other.

### 0.1 The four stages

| # | Where in code | When it folds |
|---|---|---|
| 1 | `on_dma1_ch5` (HT + TC) | DMA half-transfer (256 B) and transfer-complete (512 B) — fires only if the predecessor reply crosses those ring boundaries |
| 2 | `on_systick` `CatchupArmed`/`Snoop` arm | One SysTick CMP at `fire_tick − SWITCH_MARGIN_US`; one walk of `[snoop_head .. write_pos]` |
| 3 | `on_rxne` | USART1 RXNE per byte during the predecessor wire window; folds one byte per entry; self-masks at `bytes_walked >= N_pred − GUARD` |
| 4 | `on_systick` `TxArmed` arm, after `fire_now()` | Once at `fire_tick`; walks whatever residue remains, computes the final CRC, patches the trailing two bytes of `DXL_TX_BUF` before DMA CH4 reads them |

### 0.2 Stages compose

Different `(N_pred, wire_window_us, slack)` triples call for different subsets of {1, 2, 3}. Stages are **not** mutually exclusive alternatives; they are cooperative pre-folders that each shrink the work stage 4 inherits. All of these combinations are valid:

    1 → 2 → 3 → 4
    1 → 2 → 4
    2 → 3 → 4
    2 → 4
    3 → 4
    4

Which combination wins for a given reply is whatever leaves stage 4 with the smallest residue **without** any pre-folder's own wall-clock pushing the fire deadline (§0.3). Stage 1 is "always armed, sometimes fires" — no decision. Stage 2 and stage 3 have decisions, and they are **independent** decisions: arming stage 3 does not preclude arming stage 2 and vice versa.

### 0.3 The ISR contention constraint

A pre-folder that delays the fire CMP defeats its own purpose. Stages 2 and 3 must each complete (or self-mask) early enough that they don't push stage 4's firing time via same-priority ISR queueing on PFIC HIGH (see [dxl-rx-timing.md §8.4](dxl-rx-timing.md)). Concretely:

- Stage 2's walk must finish before `fire_tick`, or stage 4 chains synchronously after it and takes the wall-clock hit.
- Stage 3's last RXNE entry must end before `fire_tick − jitter_margin`, or the SysTick CMP for stage 4 queues behind an in-flight RXNE body.

The bounds on `SWITCH_MARGIN_US`, the `GUARD` reserve, and the stage 3 mask threshold are all derived from this constraint.

---

## 1. What the last slave has to do

A normal DXL Read reply is self-contained — we know every byte we're about to send, so we compute the CRC at build time and let DMA shovel the buffer onto the wire.

Fast Sync/Bulk Read breaks this. The host addresses N slaves at once and all N replies stitch into one coalesced Status frame on the wire, zero idle gap between slots. The CRC at the very end covers the **whole** frame — header, every preceding slave's contribution, our contribution.

So if we're slot 4 of 5:

    Wire (we are the last slot):
        [header][slot 0][slot 1][slot 2][slot 3] OUR_BYTES [CRC]
                                                  ^         ^
                                                  |         we have to write these
                                                  this is our fire deadline

The bytes labeled `slot 0`..`slot 3` came from other slaves. They're already on the wire by the time we fire. We can see them — our USART is receiving them into the RX DMA ring, same as any other traffic. We just have to fold them into the CRC accumulator so the trailing 2 bytes we emit reflect the correct CRC over the whole frame.

That's the snoop CRC. It's only needed for the **Last** slot (`FastSlotPosition::Last` in `firmware/lib/dxl-protocol/src/fast.rs`). Slot 0 (First), middle slots, and the Only-slot path emit no CRC at all — they only contribute payload.

---

## 2. The hard part: TX DMA prefetch slack

The fire ISR runs `fire_now()` (flip TX_EN high, enable DMA CH4) and then has a small window to patch the CRC bytes into the trailing slot of the TX buffer before DMA reads from those positions.

The V006's USART TX DMA reads one byte at a time from RAM with exactly one byte of look-ahead — no FIFO, no burst (see [dxl-rx-timing.md §8.2](dxl-rx-timing.md)). For a reply of `n` bytes, DMA reads position `n−2` at roughly `(n−2) × byte_time` after `fire_now()`. So the budget for stage 4 is:

    slack = (n − 2) × byte_time − post_fire_floor

At 3 Mbaud `byte_time = 3.33 µs`. For a typical 4-byte-payload reply (`n = 8`) slack is ~20 µs; for the smallest possible reply (`n = 4`) slack is ~6.66 µs. Miss it and the placeholder bytes (`[0xAA, 0xBB]` from `fast.rs::CRC_PLACEHOLDER`) land on the wire — the host sees a CRC mismatch and drops the frame.

`post_fire_floor` is the irreducible non-walk work in stage 4:

    fire_now()        ~1.2 µs   TX_EN HIGH + DMA CH4 enable
    glue + dispatch   ~0.6 µs   phase transitions, busy-wait for the in-flight tail byte
    patch_crc()       ~1.9 µs   CRC compute + 2-byte memory store

About 4 µs of fixed cost, leaving `slack − 4 µs` for the actual byte walk. The walk costs ~0.5 µs/byte at 48 MHz (24 cycles per loop iter — ring-index masking, function-call overhead, plus the flash-resident CRC LUT's wait-state fetch). At 6.66 µs slack that's about 5 bytes the walk can afford; at 20 µs slack, about 32. For predecessor byte counts above that threshold, stage 4 alone can't fit slack and the pre-folder stages have to do their job. That's the rest of this doc.

---

## 3. Stage 1 — DMA HT/TC pre-walk

USART1 RX → DMA1_CH5 → ring buffer is already running, always. The ring is 512 bytes, circular, never disabled. When stage 1 is armed (the chain enters `CatchupArmed` and HT_IE/TC_IE are set), each HT (midpoint) and TC (wrap) entry walks `ring[snoop_head .. cursor]` into the running `bulk_crc` and advances `snoop_head`. The ring no longer has to hold the whole snoop window — by the time it wraps, stage 1 has already folded the earlier half into the CRC.

Two design points worth pinning:

- **HT_IE/TC_IE are stage-gated**, enabled at chain start and cleared on cancel/fault. Always-on TC would wake the CPU for every RX byte stream including plain Sync Reads — exactly the control-loop jitter we're trying to avoid. The gating costs a handful of lines of code per arm/cancel site.
- **Stage 1 is the only stage that supports oversize Fast Bulk Reads.** For predecessor wire windows larger than the ring, stages 2/4 alone would overwrite earlier bytes before walking them; stage 1 catches each wrap before it gets clobbered. Without stage 1 the ring size would have to grow with the maximum snoop window — and the DXL 2.0 spec doesn't cap Fast Bulk Read frame size anywhere near our buffer.

HT in addition to TC means each fire walks half a ring rather than a full one — fewer worst-case bytes per ISR entry (~25 µs vs ~51 µs of CRC work), at the cost of roughly doubling ISR count for large snoops. The trade-off favors lower per-ISR cost since ISR entries serialize with the fire SysTick at the same PFIC level.

---

## 4. Stage 2 — switch CMP catchup

At chain arm, a SysTick CMP is set at `fire_tick − SWITCH_MARGIN_US`. When it fires, the handler walks `ring[snoop_head .. write_pos]` once into `bulk_crc` — every byte that has arrived since the last stage-1 wrap (if any). Then it re-arms SysTick CMP for `fire_tick` and transitions to `TxArmed`.

Sizing constraint: stage 2's walk has to fit inside `SWITCH_MARGIN_US` so it ends before `fire_tick`:

    walk_switch  =  N_residue × 0.5 µs/B  <  SWITCH_MARGIN_US

`N_residue` is bounded by `min(N_pred, N_ring − 1)` — even on a giant Bulk Read, stage 1 has already walked everything older than the current ring contents.

For predecessor wire windows shorter than `SWITCH_MARGIN_US`, stage 2 fires before the predecessor has put any bytes on the wire and walks zero bytes; all N_pred bytes hit later stages instead. That's the zero-gap coalesce / short-predecessor corner stage 3 was added to cover.

---

## 5. Stage 3 — RXNE-tail pre-walk

Stage 3 enables USART1 RXNE_IE for the duration of the predecessor's wire window and folds one byte per RXNE entry into `bulk_crc`. It self-masks once enough bytes have been pre-walked.

### 5.1 The `tail_bytes = N_pred − GUARD` rule

If stage 3 walked all N_pred bytes, the last RXNE entry would land inside the inter-slot idle gap right when the fire CMP wants to run. Same-priority PFIC means SysTick would queue behind the in-flight RXNE body, delaying stage 4 by one RXNE body's worth of ticks. So stage 3 has to stop short and let the tail bytes fall through to stage 4.

The rule is:

    tail_bytes  =  N_pred − GUARD     (GUARD = 2)

`bytes_walked` is incremented per RXNE entry; once `bytes_walked >= tail_bytes`, the handler masks RXNEIE through the composer ([dxl-rx-timing.md §7.3](dxl-rx-timing.md)). The trailing GUARD bytes hit stage 4's post-fire walk.

Why GUARD = 2:

- **One byte for the in-flight race.** The byte whose stop bit is currently clocking in is not yet visible at NDTR when stage 3's threshold check runs; stage 4's busy-wait picks it up.
- **One byte of jitter cushion.** Covers small drift in where the threshold check lands relative to the actual byte boundary.

### 5.2 Stage 3's design problem: minimize N_pred when stage 3 is needed

Stage 3 costs CPU. Every RXNE entry is roughly a few µs of PFIC trap entry + ISR body, walking exactly one byte each. For a long predecessor, stage 3 can burn a meaningful fraction of CPU spread across the predecessor's wire window — sustainable at the per-baud byte rate, but expensive, and the same-priority ADC pump is starved.

More importantly: every additional RXNE entry is another chance for one to land right at `fire_tick − epsilon` and the fire CMP to still queue behind it. The `GUARD` bytes only protect against the *known last* byte of the predecessor — they don't help if some mid-stream entry overruns into the fire window. So when stage 3 is needed at all, the design problem is:

> **Minimize N_pred (the count of bytes stage 3 has to walk) when stage 3 is needed.**

Stages 1 and 2 are how we do that. They pre-fold earlier bytes so stage 3 only needs to cover the predecessor *tail*. Two regimes:

- **Predecessor wire_window > SWITCH_MARGIN_US**: stage 2 walks the body, stage 3 (if needed at all) only handles the residue.
- **wire_window < SWITCH_MARGIN_US** (zero-gap coalesce with a short predecessor): stage 2 walks zero bytes; stage 3 has to cover the full N_pred minus GUARD.

The second case is the worst-case stage-3 load and is the corner that motivated stage 3's existence — small-N_pred / small-slack Fast Reads at high baud (e.g. 3 Mbaud, L=1, N_pred ≈ 11) where stage 4 alone can't fit slack and stage 2 fires uselessly.

### 5.3 Composition with stage 2

§0.2 asserts stages 2 and 3 compose: they are independent cooperative pre-folders. The shipping implementation in `firmware/ch32/src/dxl_fast.rs::start_fast_after` treats them as mutually exclusive (stage 2 OR stage 3) as a known simplification; the thesis-aligned composed-arming rule remains to be derived and bench-validated. The "minimize N_pred" framing in §5.2 is the lever any future composition rule has to pull.

---

## 6. Stage 4 — post-fire walk + patch

When the fire SysTick CMP lands, the `TxArmed` handler runs:

    fire_now()                              # TX_EN HIGH + DMA CH4 enable
    if rxne_tail_at != u16::MAX:
        wait_and_accumulate_tail(N_pred)    # busy-wait for the GUARD bytes
    else:
        accumulate_snoop()                  # walk whatever stage 1/2 didn't cover
    patch_crc()                             # finalize + write trailing 2 bytes

`fire_now` goes **first** — it's the jitter-critical write. Everything else races DMA prefetch slack.

The `wait_and_accumulate_tail` branch covers the stage-3-active case: stage 3 pre-walked all but `GUARD` bytes, so the walk is at most `GUARD` bytes long but must busy-wait for the very last predecessor byte to land at NDTR before walking it. The busy-wait is bounded by ~1 byte_time and times out to `PreviousSlotTimeout` if the predecessor never finished.

The `accumulate_snoop` branch covers the no-stage-3 case: stages 1+2 left some residue, stage 4 walks it directly.

Then `patch_crc` finalizes `bulk_crc` over our own reply bytes and writes the 2-byte CRC into the trailing slot of `DXL_TX_BUF` before DMA reads from it.

---

## 7. Sizing SWITCH_MARGIN_US, N_ring, and GUARD

(All constraints derived at 3 Mbaud — the fastest baud is also the tightest slack — at 0.5 µs/byte walk cost.)

**Constraint A — stage 2 walk fits inside SWITCH_MARGIN_US.** With stage 1 keeping CRC current up to the last ring wrap, stage 2 only walks bytes since that wrap, at most `min(N_pred, N_ring − 1)`:

    walk_switch ≤ N_residue × 0.5 µs/B ≤ M
    →  N_residue ≤ 2 × M

At M = 50 µs that's 100 B, well inside the 512 B ring; at M = 150 (the original sizing) it was 300 B, also fine.

**Constraint B — stage 4 fits slack when stage 2 covers the bulk.** The post-fire walk covers the M µs of residue accumulated since stage 2:

    straggle_bytes = M × bytes/µs
    walk_straggle  = straggle_bytes × 0.5 µs/B
    walk_straggle + post_fire_floor  <  slack
    →  M  <  (slack − post_fire_floor) / 0.15      (at 3 Mbaud)

At slack = 6.66 µs (n=4) the bound says M < ~18 µs without stage 3; at slack = 20 µs (typical) M can comfortably be ~100 µs.

**Constraint C — DMA can't lap during the straggle.** A single stage-1 wrap between stage 2 and stage 4 is fine (snoop_head reset + stage 4's walk semantics handle it consistently), but two wraps in M µs means DMA would overwrite the tail stage 1 just walked:

    N_ring > M × bytes/µs

Trivially clears at N_ring = 512 for any plausible M.

**Constraint D — stage 1 IRQ latency.** Stage 1 must finish a walk before DMA writes another N_ring bytes:

    latency_max < N_ring × byte_time

At N_ring = 512 / 3 Mbaud that's 1.7 ms — vast budget for any plausible queue depth.

**Values:** `SWITCH_MARGIN_US = 50`, `N_ring = 512`, `GUARD = 2`. M is sized to keep stage 4 inside slack at small-reply slack budgets (Constraint B); the ring at 512 B saves SRAM relative to the original 1024 (which the early "no stage 1" design needed to bound the snoop window directly); GUARD = 2 from the in-flight + jitter analysis in §5.1.

---

## 8. The state machine

`ReplyState::Chain` carries `FastChainPhase` substate plus chain bookkeeping. Phase transitions are enforced; out-of-table calls increment `FastChainFault::IllegalTransition` and cancel.

    ReplyState:
      Idle
      Plain                     — non-snoop reply (Ping, Read, Sync slot,
                                  Bulk slot, Fast Only/First/Middle)
      Chain { phase,
              fire_tick,
              snoop_head,
              bulk_crc,
              expected_predecessor_bytes,
              bytes_walked,
              rxne_tail_at }

    FastChainPhase:
      CatchupArmed   — switch CMP pending, no DMA wrap seen
      Snoop          — at least one HT/TC wrap has folded into bulk_crc
      Catchup        — switch CMP body running its walk
      TxArmed        — switch done; fire CMP set
      TxStreaming    — fire CMP fired; TX shifting + tail walk
      CrcPatched     — trailing CRC bytes patched into TX
      Done           — USART1 TC drained; bus released
      Fault(_)       — chain aborted; counter incremented

    Legal transitions (set_phase enforces):
      CatchupArmed → Snoop | Catchup
      Snoop        → Catchup
      Catchup      → TxArmed
      TxArmed      → TxStreaming
      TxStreaming  → CrcPatched
      CrcPatched   → Done
      *            → Fault(_)

    Transitions into Chain (dispatcher entry):
      Idle → Chain{CatchupArmed}   Fast last-slave with wire_window > M
                                   - pre-configure DMA CH4
                                   - snoop_head = parsed_end & RX_MASK
                                   - expected_predecessor_bytes = predict(wire_window)
                                   - rxne_tail_at = (N_pred − GUARD) if stage 3 armed
                                                    else u16::MAX
                                   - clear DMA1_CH5 HT_IF/TC_IF, enable HT_IE/TC_IE
                                   - enable RXNEIE via composer if stage 3 armed
                                   - SysTick CMP at request_end + RDT + wire_window − M

      Idle → Chain{TxArmed}        Only-slot path, OR wire_window ≤ M
                                   - pre-configure DMA CH4
                                   - HT_IE/TC_IE off
                                   - SysTick CMP at request_end + RDT + wire_window

    on_systick dispatch:
      Chain{CatchupArmed | Snoop}:
                                   set_phase(Catchup)
                                   accumulate_snoop()       # walk residue
                                   set_phase(TxArmed)
                                   SysTick CMP at fire_tick

      Chain{TxArmed}:
                                   SlotTimingMiss check
                                   disable DMA1_CH5 HT_IE/TC_IE
                                   set_phase(TxStreaming)
                                   fire_now()               # jitter-critical, first
                                   if rxne_tail_at != u16::MAX:
                                       wait_and_accumulate_tail(N_pred)
                                   else:
                                       accumulate_snoop()
                                   patch_crc()
                                   CrcPatchDeadlineMiss check
                                   PreviousSlotTimeout check
                                   set_phase(CrcPatched)

    on_dma1_ch5 HT/TC dispatch:
      always: clear HT_IF/TC_IF
      while phase ∈ {CatchupArmed, Snoop, TxArmed}:
        walk ring[snoop_head..cursor] into bulk_crc
        snoop_head = cursor
        CatchupArmed → Snoop on first wrap

    on_usart1 RXNE dispatch (when stage 3 armed):
      if phase ∈ {CatchupArmed, Snoop, Catchup}:
        write_pos = current_rx_write_pos()
        if write_pos != snoop_head:
          bulk_crc = ring_crc(bulk_crc, ring, snoop_head, write_pos)
          bytes_walked += (write_pos − snoop_head) & RX_MASK
          snoop_head = write_pos
        if bytes_walked >= rxne_tail_at:
          set_rxne_irq(false)    # mask before SysTick collision

`cancel` from any phase goes back to Idle and disables HT_IE/TC_IE and the stage-3 RXNE owner. Idempotent — running from Idle is a no-op, and any `set_phase` that races with cancel sees `STATE = Idle` and silently skips.

`PreviousSlotTimeout` is checked in the `TxArmed` body after the tail walk so it covers both the long-reply path (`CatchupArmed → … → TxArmed`) and the short-reply skip-switch path (`Idle → TxArmed`). The check is observational — the reply fires whether or not the predecessor delivered on time; the counter records the miss.

The `CatchupArmed/Snoop → TxArmed` transition uses the set-and-recheck pattern from [dxl-rx-timing.md §8.3](dxl-rx-timing.md) — without it, an overshoot in the switch handler silently sleeps until the next 89-second wrap.

All four hot-path handlers (USART1, SysTick CMP, DMA1_CH5 HT/TC, USART1 TC) sit at PFIC HIGH priority and serialize cleanly — each updates `snoop_head` atomically with respect to the others, and the order they happen to fire in doesn't affect correctness.

---

## 9. RX buffers and frame sizes

There are two staged buffers between the wire and the parser:

    wire ─→ USART1 ─→ DMA CH5 ─→ [DXL_RX_BUF, 512 B, circular]
                                            │
                                            │ RingReader::ingest() copies fresh
                                            │ bytes each poll, with backpressure
                                            │ that drops the oldest byte when full
                                            ▼
                                  [scratch, DXL_SCRATCH_LEN = 256, contiguous]
                                            │
                                            │ parse_one() reads
                                            ▼
                                        parser

The DMA ring (`DXL_RX_BUF`) is the hardware-facing buffer. Stages 1/2/3/4 all walk this directly via NDTR. Constraints A–D in §7 all live here.

The scratch buffer is the parser-facing buffer. It exists because the parser needs the whole frame contiguous (for CRC and length-counting), and the ring wraps over frame boundaries with no respect for them.

The cap on receivable frame size is therefore `DXL_SCRATCH_LEN − 7 = 249 bytes` — not the DMA ring size, not `MAX_LENGTH`. Anything bigger can't fit contiguously in scratch and silently fails through backpressure draining the in-progress header off the front.

`MAX_LENGTH` (currently 1024) is **not** a frame-size cap — it's an optimization for phantom-header recovery latency. `Length > MAX_LENGTH` rejects immediately as `BadLength { skip: 4 }`; without a cap the parser returns `Incomplete` and the bad header eventually falls off the front of scratch via backpressure. Recovery time without the cap is ~853 µs at 3 Mbaud but ~267 ms at 9600. Tightening `MAX_LENGTH` to 249 would convert the silent failure on a > 249-byte legit request into a fast `BadLength` skip; that's a parser-path change separate from this design.

The snoop path doesn't go through scratch — `accumulate_snoop` and `ring_crc` walk the DMA ring directly. That's why the ring size decouples from the maximum snoop window once stage 1 is in play.

---

## 10. The framing layer is orthogonal

The chain CRC machinery is independent of the framing-mode FSM ([dxl-rx-timing.md §5, §7.1, §7.3](dxl-rx-timing.md)) that picks between IDLE-backdating and RXNE-per-byte timestamp publishing based on `(baud, RDT)`. Framing and chain-CRC stage 3 are two independent owners of USART1's RXNEIE bit, composed through:

    RXNEIE = framing_wants  OR  snoop_wants

    | framing | stage 3 | RXNEIE | handler body runs                       |
    | ------- | ------- | ------ | --------------------------------------- |
    | Idle    | off     |  off   | (IRQ disabled)                          |
    | Idle    | on      |   on   | snoop tail-walk only                    |
    | Rxne    | off     |   on   | publish per-byte timestamp only         |
    | Rxne    | on      |   on   | publish + snoop tail-walk               |

At high baud (framing in IDLE mode), framing wants RXNEIE off, so it's off unless stage 3 turned it on for the chain CRC. At low baud (framing in RXNE mode), RXNEIE is always on for framing, and stage 3's threshold-mask only affects the snoop owner; the framing publisher keeps running through the entire wire window.

The handler runs both branches in sequence with their own state guards — when only one matches, the other is a single state-check skip; when both match, both run in one IRQ entry. No extra interrupts, no double-handling.

Don't ever write the RXNEIE hardware bit from one site without going through the composer. The composer's OR-and-write must be atomic with respect to the USART1 RXNE handler that reads the FSM states; on V006 the handler only reads, never writes, the owner flags, so the composition is race-free.

---

## 11. Types and vocabulary

These names appear across the code, control table, and bench tooling. Listing here so design + impl + docs share one vocabulary.

### 11.1 Timing strategy

```rust
pub enum DxlTimingStrategy {
    /// Low baud or short RDT: IDLE fires too late, so track packet end by length.
    LengthCounted,

    /// Normal/high baud: use UART IDLE and subtract one byte-time.
    IdleBackdated,

    /// Fast Sync/Bulk: chain status response after previous slave segment.
    ChainedFastStatus,
}
```

Maps to the framing-mode FSM in [dxl-rx-timing.md §7.1](dxl-rx-timing.md):

- `LengthCounted` is parent's `Rxne` framing — per-byte RXNE publishes a `(rx_cursor, systick)` cell.
- `IdleBackdated` is parent's `Idle` framing — one IDLE IRQ per packet, backdated by `9 × BRR` ticks.
- `ChainedFastStatus` is *this* doc's contribution — the Fast Last-slave path. Orthogonal to `LengthCounted` / `IdleBackdated`: the chain is its own strategy, but it relies on whichever framing mode published the host request's wire-end.

The framing FSM picks `LengthCounted` vs `IdleBackdated` automatically from `(baud, RDT)`. `ChainedFastStatus` is selected per-reply by the dispatcher — only Fast Last-slave replies use it.

### 11.2 Fast chain phases

`FastChainPhase` enumerates the chain timeline; see §8 for transitions. `ReplyState::Chain { phase, ... }` carries the current phase as runtime state.

### 11.3 Fault taxonomy

```rust
pub enum FastChainFault {
    IllegalTransition,
    UnexpectedByteCount,
    PreviousSlotTimeout,
    SlotTimingMiss,
    CrcPatchDeadlineMiss,
    DmaOverrun,
    UartError,
}
```

| Variant | Detected when | Maps to |
|---|---|---|
| `IllegalTransition` | `set_phase` called with (from, to) pair not in §8 legal table | FSM hygiene |
| `UnexpectedByteCount` | `accumulate_snoop` sees NDTR inconsistent — ring lapped twice between walks | Constraint D |
| `PreviousSlotTimeout` | post-tail-walk `bytes_walked` falls short of `expected_predecessor_bytes` | host/protocol issue |
| `SlotTimingMiss` | SysTick CMP fires later than `fire_tick + 1 byte_time` (jitter cap) | dxl-rx-timing §13 |
| `CrcPatchDeadlineMiss` | `patch_crc()` completes after TX DMA has read past `n−2` | Constraint B |
| `DmaOverrun` | USART1 STATR.ORE — RX outpaced DMA drain | chip limit |
| `UartError` | USART1 STATR.{PE,FE,NE} — parity/framing/noise | PHY/wire issue |

Each variant maps 1:1 to a `u32` counter in `TelemetryDxlLink`. `report_fault` does a volatile RMW on the matching field; the host reads via standard DXL `Read` against the telemetry CT region.

---

## 12. The structural fire floor

Chain CRC correctness lives on stage 4 fitting slack. Coalesce hand-off jitter lives on a separate budget: the fire SysTick path has a floor of ~5–6 µs from CMP-match to first wire bit, made up of:

- PFIC trap entry: ~1 µs (irreducible without core changes).
- Fire ISR body before `fire_now` (post-highcode): ~1.5 µs.
- `fire_now` itself (DMA EN write → TX_EN GPIO toggle → first start bit): ~2 µs.

Plus jitter on top from "what was running when CMP matched." The HSI cal flow ([dxl-hsi-calibration.md](dxl-hsi-calibration.md)) subtracts the mean from the deadline via `TX_FAST_LATENCY_TICKS` so on average we land on time, but the jitter remains and at 3 Mbaud it's on the order of a third of a byte time.

This is structural — the chain CRC design doesn't close it. The chain CRC ensures stage 4 fits slack so the CRC is correct; the fire floor governs whether the start bit lands when it should (so coalesce is clean). At lower bauds the floor sits well under the inter-slot jitter cap. At 3 Mbaud zero-gap coalesce, the floor and the cap sit at the same order of magnitude, and a visible idle gap can appear on the wire even though the CRC is correct and the host parses fine.

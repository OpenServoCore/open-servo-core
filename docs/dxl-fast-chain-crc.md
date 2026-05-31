# DXL Fast Last-Slave Chain CRC on the CH32V006

Companion to [dxl-rx-timing.md](dxl-rx-timing.md). Read that first — this doc assumes you already know what "the wire-end timestamp," "SysTick CMP fire," and "the V006's two PFIC priorities" mean.

**TL;DR.** When we're the last slave in a DXL Fast Sync/Bulk Read, we have to compute a CRC over bytes we didn't generate — every byte the predecessor slaves' replies put on the wire — and stitch our own bytes onto the end. We've tried two strategies and bounced off both: walking everything at fire blows past the TX DMA prefetch slack for any large snoop; firing RXNE per byte pollutes the system with hundreds of interrupts and shoves SysTick's fire deadline back by ~4 µs at 3 Mbaud. The fix is a three-handed split: DMA's Transfer-Complete IRQ on the RX channel pre-walks the CRC every time the ring wraps (so the ring no longer has to hold the whole snoop window); a SysTick CMP fires a fixed margin before the real deadline and closes whatever gap remains; the fire ISR grabs the tail and patches CRC into the TX buffer. The RX ring drops from 1024 to 512 bytes, the snoop window stops being bounded by ring size, and oversize Fast Bulk Reads stop silently corrupting the bus. The 150 µs margin is still derived from baud and slack, not magical.

> **Update 2026-05-30 — read §14 first.** Bench validation found two
> things this original write-up got wrong: per-byte CRC walk is **~0.5
> µs/B**, not 0.1 (5× off — table-fetch reads are slower than a naive
> cycle count), and the residue model breaks when wire_window < M in
> zero-gap coalesce (switch fires before the predecessor's first byte).
> The §1–§13 design below ships and works for L ≥ 2 once M shrinks to
> 50 µs, but L=1 Last at 3 Mbaud falls through the gap. §14 adds the
> **RXNE-tail fourth tier** — per-byte CRC pre-walk for just the
> slack-budget tail bytes — and re-derives the sizing constants with
> the corrected per-byte cost. Read §1–§13 as the design's intent;
> §14 is the implementation truth.
>
> **Update 2026-05-31 — and §15 if you want the next chapter.** The
> RXNE-tail tier from §14 makes the CRC right at 3 M Last 1B, but
> bench shows ~25-30% of shots still ship with a visible idle gap on
> the wire — fire-arrival jitter from the SysTick PFIC trap entry,
> not snoop saturation. §15 lays out the plan to close it: hardware-
> driven fire via TIM2 OPM + DMA1_CH7 (CC4 → GPIOC.BSHR for TX_EN) +
> DMA1_CH2 (UP → USART1.CTLR3 for DMAT). PFIC leaves the critical
> path entirely; CMP-match → wire-bit drops from ~6 µs ± jitter to
> sub-microsecond + the ½ baud-tick USART sync floor. Gated behind a
> `dxl-hw-fire` Cargo feature so TIM2 stays free for bench/encoder
> use under `dxl-systick-fire`. Not yet implemented — H-series tasks
> #57-#65 in the tracker.

---

## 1. Four things we're trying to get right

Before getting into the design it helps to spell out what we're actually after. Four things, all hard-won — we'd like to hang onto every one of them.

**R1 — the fire SysTick lands on schedule.** Our reply has to start at exactly `wire-end + RDT + slot_offset` so it coalesces with the predecessor slot at zero idle gap. DXL Fast's jitter cap is one byte time — 3.33 µs at 3 Mbaud — and anything that shoves the fire SysTick back by more than a couple of microseconds breaks coalesce. The current code's RXNE-per-byte snoop hits exactly that: the last RXNE for slot N-2 tends to land inside the inter-slot idle gap, and since USART1 and SysTick share PFIC priority (§8.4 parent), SysTick CMP queues behind the in-flight RXNE. On the bench we see ~4 µs of slip at 3 Mbaud. That's the thing we most want to fix.

**R2 — snoop CRC for the early slots happens "at DMA," not per byte.** DMA CH5 is already collecting every snoop byte silently; there's no protocol reason the CPU needs to wake up for each one. Per-byte RXNE stacks ~200 USART1 trap entries per Fast Last reply at 3 Mbaud, each ~3 µs of pure trap-entry overhead. That's ~600 µs of CPU spent doing nothing useful every Fast Read, and the ADC kernel pump (20 kHz, Low priority) gets preempted by every one of them — control loop jitter goes up whenever the bus is busy. We'd rather let DMA do the byte-shoveling it's built for and walk the buffer in batches at moments we pick.

**R3 — slot N-2 is the awkward one, and whatever we do here can't compromise R1.** N-2's bytes arrive at the very end of the snoop window, right when fire is imminent. RXNE-during-N-2 is exactly what causes R1's failure, so if we drop RXNE we have to put N-2's CRC somewhere else. Post-fire is the natural place — TX is already shifting and the CPU has the `(n-2) × byte_time` prefetch window to work in (§3) — but that window is only 6–20 µs at 3 Mbaud, much less than the walk time for an asymmetric Bulk Read where one predecessor has a big payload and we have a tiny one. Two ways out: (a) drop USART1 to Low so SysTick at High preempts RXNE during snoop, or (b) get rid of RXNE entirely and find a CRC path for N-2 that respects the slack ceiling. V006's two-level PFIC makes (a) hard to layer cleanly without surprising the framing layer, so this design goes with (b).

**R4 — oversize Fast Bulk Reads can't silently corrupt the bus.** This one snuck up on us mid-design. The DXL 2.0 spec doesn't cap Fast Bulk Read frame size anywhere near our RX ring, and a host querying ~30 servos for 32 bytes each (or ~10 for whole control-table dumps) overflows the ring before we finish snooping. The failure mode is grim: we walk bytes that DMA has already wrapped over and overwritten, we compute a wrong CRC, and the host drops the *entire coalesced frame* — meaning every other slave on the bus gets punished for our mistake. We'd rather support anything spec-legal than make our neighbors' replies disappear because we sized our buffer for "typical."

---

## 2. What the last slave has to do

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

## 3. The hard part: TX DMA prefetch slack

The fire ISR runs `fire_now()` (flip TX_EN high, enable DMA CH4) and then has a small window to patch the CRC bytes into the trailing slot of the TX buffer before DMA reads from those positions.

The V006's USART TX DMA reads one byte at a time from RAM with exactly one byte of look-ahead — no FIFO, no burst (§8.2 of the parent doc). For a reply of `n` bytes, DMA reads position `n−2` at roughly `(n−2) × byte_time` after `fire_now()`. So:

    slack = (n − 2) × byte_time

At 3 Mbaud `byte_time = 3.33 µs`. For a typical 4-byte-payload reply (`n = 8`, namely ERR + ID + 4 data + 2 CRC), slack is **20 µs**. For the smallest possible reply (`n = 4`, no data), slack is **6.66 µs**.

That's the entire budget for "catch the running CRC up to the wire + fold our payload in + write 2 bytes." Miss it and the placeholder bytes (`[0xAA, 0xBB]` from `fast.rs::CRC_PLACEHOLDER`) land on the wire — the host sees a CRC mismatch and drops the frame.

## 4. What we've tried

### 4.1 Walk everything at fire

Simplest possible. Fire ISR, post-`fire_now()`, walks the whole RX ring from `parsed_end` (where the host's request ended) up to wherever NDTR currently points. CRC every byte. Fold in our payload. Write the 2 CRC bytes.

CRC walks at ~5 cycles/byte at 48 MHz = **~0.1 µs/byte**. Compare to slack:

| snoop bytes | walk time | fits 20 µs slack? | fits 6.66 µs slack? |
|---|---|---|---|
| 26 (5-slave Fast Sync, 4-byte payload) | 2.6 µs | ✓ | ✓ |
| 138 (1 predecessor with 128-byte payload) | 13.8 µs | ✓ | ✗ |
| 800 (~6 predecessors with 128-byte payloads) | 80 µs | ✗ | ✗ |

Asymmetric Bulk Read is the killer case: one slave returning 128 bytes, us returning 1 byte. 138 wire bytes to walk, 6.66 µs of slack. We lose.

### 4.2 RXNE per byte (the current code)

Enable USART1 RXNE_IE for the duration of the snoop window. Each received byte fires a USART1 IRQ; the handler reads NDTR, folds new bytes into the running CRC, advances `snoop_head`. By fire time the CRC is essentially current — the fire ISR sweeps up at most a one-byte straggle.

The CRC math is fine. The cost is volume. At 3 Mbaud a 200-byte snoop fires 200 USART1 IRQs in 666 µs, each costing ~3 µs of PFIC trap entry plus the snoop body.

Worse: the **last** RXNE in the window lands inside the inter-slot idle gap right before our fire deadline, and SysTick CMP — which is supposed to fire at that deadline — sits at the same PFIC priority as USART1 (§8.4 parent). Same-priority IRQs don't preempt each other. SysTick gets queued behind the in-flight USART1 entry, pushing fire back by **~4 µs at 3 Mbaud** (bench-measured). On a chip whose structural fire floor is already ~5 µs (§13 parent), this 4 µs penalty is the difference between "barely meets spec" and "definitely doesn't."

And the 200 trap entries per Fast Read interleave badly with the 20 kHz ADC kernel pump. Control loop jitter goes up under traffic.

### 4.3 The hybrid: DMA TC keeps CRC current, SysTick closes the gap, fire grabs the tail

Three coordinated handlers, none of them firing per byte.

**DMA Transfer-Complete IRQ on RX (CH5)**, enabled only during the snoop window. Every time the RX ring wraps (the DMA NDTR counts down to zero and reloads), the TC handler walks `ring[snoop_head..N]` into the running CRC and resets `snoop_head = 0`. At 3 Mbaud with a 512-byte ring this fires every ~1.7 ms, so even a huge multi-slave Bulk Read response triggers only a handful of TC entries spread across the snoop. The ring no longer has to hold the whole snoop window at once — it just has to absorb one full wrap of latency before TC catches up.

**SysTick CMP at `fire − M`** (M = 150 µs) — the WaitingSwitch deadline. By the time switch fires, TC has already walked everything up to the last wrap; the switch ISR walks whatever's arrived since (at most N − 1 bytes, so ~25 µs at N = 256 or ~51 µs at N = 512) and advances `snoop_head` to the current NDTR position.

**SysTick CMP at `fire`** — fires DMA TX first (jitter-critical), then walks the straggle (≤ 45 bytes at 3 Mbaud since M µs of bytes have arrived since switch), folds in our own reply bytes, and writes the CRC into the trailing two bytes of the TX buffer.

    arm                  fire − M                fire
    |                       |                     |
    | snoop bytes streaming into RX DMA ring (no CPU)
    |←─ TC fires per ring wrap ─→| (handful per snoop, walks N bytes each)
    |                       |                     |
    |                  SysTick CMP:           SysTick CMP:
    |                  walk ring[snoop_head..NDTR]  fire_now()  ← first!
    |                  ≤ N bytes worth         walk straggle (≤45 bytes)
    |                                          patch CRC into TX

Crucially, all three handlers sit at the same PFIC priority (HIGH). They serialize, never preempt each other, and the order they happen to fire in doesn't matter — each one walks `ring[snoop_head..endpoint]` and updates `snoop_head`, atomically with respect to the others.

The snoop path doesn't enable `RXNEIE` at any point — no per-byte interrupts on its behalf. (At low baud the framing layer may still drive per-byte RXNE for its own scheduling work; that's an independent owner of the same hardware bit and §9 walks through how the two coexist.)

#### Why TC and not HT

DMA also offers a half-transfer IRQ that fires at the ring midpoint. We rejected it earlier when the alternative was "always on" — HT during normal RX traffic is exactly the kind of CPU wake we're trying to avoid. With stage-gating (TC_IE off outside the snoop window) HT becomes viable too, but TC alone gives us bigger walk chunks per ISR entry. At N = 512, TC walks 512 bytes per fire (51 µs of CRC work); HT would walk 256 (25 µs) and double the ISR count for the same total walk work. TC alone — fewer interrupts, same correctness.

#### Why we didn't ship just SysTick

The earlier draft of this plan had only the WaitingSwitch + fire pair, no TC, and sized the ring at 1024 to fit any plausible snoop window in one shot. Two things drove the rework:

1. **R4.** A 1024-byte ring caps the snoop window at 1024 wire bytes, and the spec lets Fast Bulk Read frames go larger. The silent-corruption failure mode (wrong CRC ships, host drops every slave's reply) is much worse than the bench cost of a few extra TC ISRs.
2. **SRAM.** 1024 bytes is 12% of the V006's 8 KB SRAM. A 512-byte ring saves real budget for the control loop and future features.

The hybrid keeps the WaitingSwitch + fire structure intact and adds TC as the mechanism that lets us shrink the ring and remove the cap.

## 5. Sizing N and M

Four constraints now (one new, one fewer ring-coupling), all worst-case at 3 Mbaud — the fastest baud is also the tightest slack.

**Constraint A — switch walk has to fit inside the margin.** With TC keeping the CRC current up to the last ring wrap, the switch only walks bytes that arrived between that wrap and now — at most `N − 1` bytes:

    walk_switch ≤ N × 0.1 µs ≤ M
    →  N ≤ 10.3 × M

For M = 150 µs: N ≤ 1545 bytes. Both 256 and 512 clear this comfortably — and notably, **N no longer has to grow with the snoop window**.

**Constraint B — straggle walk has to fit inside the slack.** Unchanged from the earlier draft. The straggle is the M µs of wire bytes that arrived between switch and fire, walked post-fire:

    (M × 0.3) × 0.1  +  patch  <  slack
    →  M < (slack − patch) / 0.03

Smallest reply has slack 6.66 µs (n = 4 at 3 Mbaud), patch ≈ 1 µs, so M < 188 µs.

**Constraint C — DMA can't lap during the straggle.** This is really a tighter version of D applied to the M-µs window: a single TC firing between switch and fire is fine (snoop_head reset + fire's walk semantics handle it consistently), but two wraps in M µs means DMA overwrites the tail TC just walked. So:

    N > M × bytes/µs = 45 bytes at 3 Mbaud, M = 150
    →  N ≥ 64

Even at N = 64 this is hard to trip; at N = 512 it's not a thing.

**Constraint D — TC IRQ latency before DMA laps `snoop_head`.** After TC fires, `snoop_head` jumps to 0; DMA keeps writing. If the TC handler is queued behind other HIGH-priority work (USART1 IDLE, SysTick), it has to finish before DMA writes N more bytes:

    latency_max < N × byte_time
    N = 256, 3 Mbaud: 853 µs  ← vast budget, any plausible queue clears this
    N = 512:           1707 µs

**Picking the values: M = 150 µs (still in [100, 188] from A and B), N = 512** for the comfortable middle. 512 saves half the SRAM and pushes Constraint A's headroom past 3× before it'd matter. N = 256 also works and saves more SRAM.

The DMA ring size N is a purely snoop-side knob. The host-request parser cap (`MAX_LENGTH`) is a separate dial tied to `DXL_SCRATCH_LEN` (the parser's contiguous-frame buffer in `core::services::dxl`), not to N — the parser feeds off scratch, which `RingReader::ingest` copies into from the DMA ring with backpressure. Shrinking N doesn't constrain request size.

### Worked examples

The pathological asymmetric Bulk Read case — 128-byte predecessor payload, 1-byte our payload, 138 wire bytes:

    fire_us       = 138 × 3.33  = 460 µs
    TC entries    = 0   (138 < N = 512, no wrap)
    switch walk   = up to 138 × 0.1 = 14 µs       ← fits 150 µs margin ✓
    straggle      = 45 bytes
    fire-time     = 5 µs                           ← fits 10 µs slack ✓ (5-byte reply)

Realistic worst case — ~6 predecessors with 128-byte payloads, us with 1 byte, 800 wire bytes:

    fire_us       ≈ 2664 µs
    TC entries    = 1   (one full 512-byte wrap)
    switch walk   ≤ 288 × 0.1 = 29 µs              ← fits 150 µs ✓
    fire-time     = 5 µs                           ← fits 6.66 µs slack ✓ (4-byte reply)

Oversize Fast Bulk Read — 20 servos × 80 bytes each, ~1620 wire bytes (over the old 1024 cap):

    fire_us       ≈ 5400 µs
    TC entries    = 3   (three full 512-byte wraps)
    switch walk   ≤ 84 × 0.1 = 8 µs                ← fits 150 µs ✓
    fire-time     = 5 µs                           ← fits 6.66 µs slack ✓
    → works cleanly, no silent corruption

Typical Fast Sync — 5 slaves with 4-byte payloads, 26 wire bytes:

    fire_us       = 86 µs
    fire_us < M   → skip WaitingSwitch
    TC entries    = 0
    walk-at-fire  = 26 × 0.1 = 2.6 µs              ← fits 20 µs slack ✓ (8-byte reply)

When `fire_us ≤ M` the snoop is small enough that one walk at fire fits trivially. No switch, no TC; the FSM transitions straight from arm into `WaitingFire`.

## 6. State machine

The reply scheduler carries a richer state than the parent doc's three-state Stage. `ReplyState::Chain` encodes the Fast last-slave timeline as substages so each handler dispatches on phase directly, and illegal transitions are detectable as a fault (`FastChainFault::IllegalTransition`). The trade-off — ~20 bytes of static state vs a shadow-only design — buys type-state encoding of the FSM and observable hygiene faults.

    ReplyState:
      Idle
      Plain                     — non-snoop reply (Ping, Read, Sync slot,
                                  Bulk slot, Fast Only/First/Middle)
      Chain { phase,            — Fast last-slave reply with snoop CRC
              fire_tick,
              snoop_head,
              bulk_crc,
              expected_predecessor_bytes,
              bytes_walked }

    FastChainPhase (substates of Chain):
      CatchupArmed   — WaitingSwitch CMP pending, no DMA TC wrap seen yet
      Snoop          — at least one TC wrap has folded a full ring into bulk_crc
      Catchup        — WaitingSwitch CMP body running its bulk walk
      TxArmed        — switch done; fire CMP set
      TxStreaming    — fire CMP fired; TX shifting + straggle walk in progress
      CrcPatched     — trailing CRC bytes patched into TX
      Done           — USART1 TC drained the reply; bus released
      Fault(_)       — chain aborted; matching counter incremented

    legal phase transitions (set_phase enforces; out-of-table calls bump
    IllegalTransition and cancel; calls outside Chain are silent no-ops):
      CatchupArmed → Snoop | Catchup
      Snoop        → Catchup
      Catchup      → TxArmed
      TxArmed      → TxStreaming
      TxStreaming  → CrcPatched
      CrcPatched   → Done
      *            → Fault(_)

    transitions out of Idle (entered by the dispatcher):
      Idle → Plain                     non-snoop reply gets scheduled
                                       - pre-configure DMA CH4 (count + source, EN=0)
                                       - SysTick CMP at: request_end + reply_delay

      Idle → Chain{CatchupArmed}       Fast last-slave with fire_us > M
                                       - pre-configure DMA CH4
                                       - snoop_head = parsed_end & RX_MASK
                                       - expected_predecessor_bytes = predict(fire_us)
                                       - clear DMA1_CH5 TC_IF, enable TC_IE
                                       - SysTick CMP at: request_end + fire_us − M

      Idle → Chain{TxArmed}            Only-slot path, OR Fast last-slave with
                                       fire_us ≤ M (single walk at fire is enough)
                                       - pre-configure DMA CH4
                                       - TC_IE stays off
                                       - SysTick CMP at: request_end + fire_us

    SysTick CMP body (on_systick) dispatches on phase:
      Chain{CatchupArmed | Snoop} CMP fires:
                                       - set_phase(Catchup)
                                       - accumulate_snoop() walks ring[snoop_head..NDTR]
                                       - set_phase(TxArmed)
                                       - SysTick CMP at: request_end + fire_us

      Chain{TxArmed} CMP fires:
                                       - SlotTimingMiss check (now vs fire_tick)
                                       - disable DMA1_CH5 TC_IE
                                       - set_phase(TxStreaming)
                                       - fire_now()             ← jitter-critical, first
                                       - accumulate_snoop()     ← straggle catch-up
                                       - patch CRC into TX trailing 2 bytes
                                       - CrcPatchDeadlineMiss check
                                       - PreviousSlotTimeout check
                                       - set_phase(CrcPatched)

    DMA1_CH5 TC IRQ body (on_dma1_ch5_tc), runs whenever TC fires:
      always: clear TC_IF (stale flag would re-trigger on next unmask)
      while phase ∈ {CatchupArmed, Snoop, TxArmed}:
        - walk ring[snoop_head..N] into bulk_crc
        - snoop_head = 0
        - CatchupArmed → Snoop on first wrap

`cancel` from any state goes back to Idle and disables TC_IE. Idempotent contract — running from Idle is a no-op, and any `set_phase` that races with cancel sees STATE = Idle and silently skips (no double-fault).

`PreviousSlotTimeout` is checked in the TxArmed body after `accumulate_snoop` so it covers both the long-reply path (CatchupArmed → … → TxArmed) and the short-reply skip-switch path (Idle → TxArmed). The check is observational — the reply fires whether or not the predecessor delivered on time; the counter simply records the miss.

All four hot-path handlers (USART1, SysTick CMP, DMA1_CH5 TC, USART1 TC) sit at PFIC HIGH priority and serialize cleanly — each updates `snoop_head` atomically with respect to the others, and the order they happen to fire in doesn't affect correctness. The TC ISR is also the simplest of the four: walk a fixed-size block, reset `snoop_head` to a constant, clear a flag.

The `CatchupArmed/Snoop → TxArmed` transition is still subject to the "CNTIF latches only on `CNT == CMP` up-count" gotcha from §8.3 of the parent doc — the set-and-recheck pattern after writing the new CMP is mandatory, otherwise an overshoot in the switch ISR silently sleeps until the next 89-second wrap.

## 7. Two RX buffers, two caps, two different jobs

Worth pausing to spell out the RX path's layout before "what disappears" makes claims about it. There are two staged buffers between the wire and the parser, not one:

    wire ─→ USART1 ─→ DMA CH5 ─→ [DXL_RX_BUF, 1024 → 512, circular]
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

The **DMA ring** (`DXL_RX_BUF`) is the hardware-facing buffer. DMA writes circularly forever, doesn't know about frame boundaries, exists because the CPU can't keep up with per-byte interrupts and needs somewhere to park bytes between IRQ entries. **This is what the hybrid plan shrinks from 1024 to 512.** Constraints A–D in §5 all live on this buffer.

The **scratch buffer** (`DXL_SCRATCH_LEN`, in `core::services::dxl::api`) is the parser-facing buffer. `RingReader::ingest()` copies fresh ring bytes into scratch on each `Dxl::poll()`, and `parse_one` reads scratch as a contiguous slice. It exists because the parser needs the whole frame contiguous (for CRC and length-counting) and the ring wraps over frame boundaries with no respect for them.

So the actual cap on receivable frame size is **`DXL_SCRATCH_LEN − 7 = 249 bytes`** — anything bigger can't fit contiguously in scratch and silently fails through backpressure draining the in-progress header off the front. Not the DMA ring size; not `MAX_LENGTH`.

The snoop path is different. `accumulate_snoop()` walks the DMA ring **directly** (it doesn't go through scratch), so the snoop window's only limit is how often TC catches up — exactly the §5 constraints. That's why shrinking the ring doesn't touch our ability to handle big Fast Bulk Read responses.

### What `MAX_LENGTH` actually does

`MAX_LENGTH` in `dxl-protocol::packet` looks like a hard cap on what we can receive, but it isn't — it's really just an **optimization for phantom-header recovery latency**. When the parser sees a `HEADER` followed by a Length field, it has two options:

- **With cap (today, 1024):** `Length > MAX_LENGTH` rejects immediately as `BadLength { skip: 4 }`. Recovery in microseconds.
- **Without cap (or with very large cap like 65535):** the parser returns `Incomplete` and waits. Scratch fills with new wire bytes; backpressure eventually drops the bad header off the front and the parser unsticks itself. Recovery time ≈ `scratch_size × byte_time` — roughly 853 µs at 3 Mbaud, but **~267 ms at 9600 baud**.

Either way the parser eventually recovers — it's effectively a stream parser through RingReader's backpressure. Neither version changes *what we can receive*; only how quickly we shake off bad headers from bus noise.

One latent footgun worth flagging: today `MAX_LENGTH = 1024` is **looser** than the actual scratch cap of 249. So a 500-byte legitimate request slips past the `MAX_LENGTH` check, then silently fails because scratch can never hold it contiguously. Tightening `MAX_LENGTH` to `DXL_SCRATCH_LEN − 7 = 249` would convert that silent failure into a fast `BadLength` skip. Worth doing, but it's separate from this plan — the hybrid only touches the DMA ring.

## 8. What disappears, what's new

The snoop owner of RXNE_IE no longer exists. The OR-composer from §7.3 of the parent doc collapses to a single owner (framing).

Gone:

- `dxl_fast::on_rxne` — nothing enables RXNE for snoop anymore.
- The dispatch from `irq.rs::on_usart1` to `dxl_fast::on_rxne`.
- `DXL_RX_BUF_LEN = 1024` → drops to 512. (`MAX_LENGTH` is independent — it's tied to `DXL_SCRATCH_LEN` in the parser path, not the DMA ring.)
- The implicit "snoop window ≤ 1024" precondition. Fast Bulk Reads of any spec-legal size now compute the right CRC.

Kept:

- `usart::set_rxne_irq` in the HAL — the framing layer still uses it in RXNE mode (§5 parent).

New:

- `DMA1_CHANNEL5` IRQ vector wired up (previously unused). Priority HIGH (same as USART1 + SysTick), gated by phase so it only fires during snoop windows.
- `ReplyState::Chain` variant carrying phase + chain bookkeeping; eight-state `FastChainPhase` substage enum; `SWITCH_MARGIN_US = 150` constant.
- Per-fault counters in `TelemetryDxlLink` (`illegal_transition`, `unexpected_byte_count`, `previous_slot_timeout`, `slot_timing_miss`, `crc_patch_deadline_miss`, `dma_overrun`, `uart_error`) — observable from the host control table.

ISR count for the snoop window goes from `N_wire_bytes` (today) to **`2 + ⌈total / N⌉`** — so 2 for small snoops (no TC), 3-4 for realistic Bulk Reads, growing linearly for huge ones rather than overflowing the buffer.

## 9. Orthogonal piece: RXNE for framing at low baud

The hybrid kills "RXNE for snoop." It does **not** kill "RXNE for framing" — those are two independent owners of the same hardware bit, and the framing layer's design from [parent doc §5](dxl-rx-timing.md) and [§7.3](dxl-rx-timing.md) is untouched here.

Quick recap of why framing wants RXNE at all. The wire-end timestamp the slot scheduler arms SysTick against has to arrive *before* the fire deadline. With IDLE framing, that timestamp shows up roughly one char-time after wire-end:

    publish_latency_idle  ≈  9 / baud × 10⁶  µs

If that exceeds RDT, IDLE is too late and the scheduler can't honor the protocol. The decision rule is:

    use_rxne_framing = char_time_us + pipeline_margin_us > rdt_us

picks RXNE whenever IDLE can't make it. At 9600 baud, `char_time ≈ 937 µs` blows past a typical `RDT = 250 µs` — framing flips to RXNE so each byte's stop-bit interrupt can publish the timestamp directly. At 57600 baud and above (with typical RDT), framing uses IDLE.

### How the two owners coexist under the hybrid

The OR-composer from parent §7.3 routes both owners onto the single `RXNEIE` bit:

    RXNEIE = framing_wants OR snoop_wants

The hybrid sets `snoop_wants` to **always false**, so the composer collapses:

    RXNEIE = framing_wants

- **High baud (framing = IDLE):** `framing_wants = false`. `RXNEIE` stays off. Zero per-byte interrupts of any kind — R2 fully realized.
- **Low baud (framing = RXNE):** `framing_wants = true`. `RXNEIE` is on, per-byte interrupts fire through request reception *and continue* through the snoop window. But the handler body's old snoop branch is gone — only framing's publish branch runs each entry.

The parent doc's RXNEIE state table simplifies under the hybrid (the WaitingFire row no longer adds a snoop dependency):

    | framing | scheduler              | RXNEIE | handler body runs |
    | ------- | ---------------------- | ------ | ----------------- |
    | Idle    | Idle                   |  off   | (IRQ disabled)    |
    | Idle    | WaitingPlain           |  off   | (IRQ disabled)    |
    | Idle    | WaitingSwitch/Fire     |  off   | (IRQ disabled)    |
    | Rxne    | Idle                   |   on   | publish only      |
    | Rxne    | WaitingPlain           |   on   | publish only      |
    | Rxne    | WaitingSwitch/Fire     |   on   | publish only      |

### Is the low-baud per-byte cost a problem?

Not in practice. IRQ volume scales with byte rate, which is *low* at low baud by definition:

| baud  | bytes/sec | framing IRQ entries during an 800-byte snoop window |
|---|---|---|
|  9600 |   ~960 | 800 entries over ~830 ms (~0.3% CPU at ~3 µs/entry)  |
| 57600 |  ~5760 | 0 (framing already in IDLE)                          |
|  1 M  | ~100 k | 0 (framing already in IDLE)                          |

The hybrid's R2 win lands **fully at high baud** (where it matters for CPU jitter against the 20 kHz ADC pump and where typical control-loop traffic actually lives) and is **inexpensive at low baud** (where framing's own per-byte work is the dominant cost and the byte rate keeps total load light).

### USART1 ISR body under the hybrid

The snoop branch is deleted; framing's branch is unchanged:

    on USART1 entry (when RXNE is the cause):
        if framing FSM == Rxne:
            publish (rx_cursor_now, systick::ticks()) to single-cell snapshot
        # snoop branch from the old design is removed

That's it. The snoop CRC machinery (TC + WaitingSwitch + fire) is entirely separate from this path.

## 10. Suggested types and vocabulary

A few types worth sketching so implementation and documentation share a vocabulary. These don't change the design — they name what §1–§9 already describe.

### 10.1 Timing strategy

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

Maps onto the framing layer's mode FSM in [parent doc §7.1](dxl-rx-timing.md):

- `LengthCounted` is parent doc's `Rxne` framing — per-byte RXNE entries publish a `(rx_cursor, systick)` cell. Named "length-counted" because the framing layer effectively tracks byte count to identify the last one.
- `IdleBackdated` is parent doc's `Idle` framing — one IDLE IRQ per packet, backdated by `9 × BRR` ticks (parent §8.6) to recover wire-end.
- `ChainedFastStatus` is *this* doc's contribution — the Fast Last-slave path that chains a snoop-CRC'd Status reply onto predecessor wire bytes. Orthogonal to (`LengthCounted`, `IdleBackdated`): the chain is its own strategy, but it still relies on whichever of those two published the host request's wire-end timestamp.

The framing FSM picks `LengthCounted` vs `IdleBackdated` automatically from `(baud, RDT)`. `ChainedFastStatus` is selected per-reply by the dispatcher — only Fast Last-slave replies use it.

### 10.2 Fast chain phases

```rust
pub enum FastChainPhase {
    CatchupArmed,
    Snoop,
    Catchup,
    TxArmed,
    TxStreaming,
    CrcPatched,
    Done,
    Fault(FastChainFault),
}
```

The chain timeline. `ReplyState::Chain { phase, ... }` carries the current phase as runtime state — `set_phase` enforces the legal transition table from §6 and any out-of-table call bumps `FastChainFault::IllegalTransition` and cancels the chain. Each handler dispatches on phase: `on_systick` routes CatchupArmed/Snoop into the bulk walk, TxArmed into the fire-and-patch body; `on_dma1_ch5_tc` gates its walk on `phase ∈ {CatchupArmed, Snoop, TxArmed}`.

| Phase | When | Where in code |
|---|---|---|
| `CatchupArmed` | `start_fast_after` armed the switch CMP; no DMA TC wrap seen yet | tail of `start_fast_after` |
| `Snoop` | First DMA TC wrap has folded a full ring into `bulk_crc` | tail of `on_dma1_ch5_tc` |
| `Catchup` | `on_systick` running the switch-CMP bulk walk | `on_systick` CatchupArmed/Snoop arm |
| `TxArmed` | Switch walk done; fire CMP set; or short-reply skip-switch path | end of CatchupArmed/Snoop arm; from `start_fast_after` direct |
| `TxStreaming` | Fire ISR after `fire_now()`, straggle walk + CRC patch in progress | start of TxArmed arm body |
| `CrcPatched` | `patch_crc()` done, CRC bytes in TX trailing slot | end of TxArmed arm body |
| `Done` | USART1 TC has fired, bus released | (not yet wired — see A7) |
| `Fault(_)` | Any error path that aborts the chain | `report_fault` + `cancel` |

### 10.3 Fault taxonomy

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

What each one catches and which constraint it maps to:

| Variant | Detected when | Constraint |
|---|---|---|
| `IllegalTransition` | `set_phase` called with a (from, to) pair not in the §6 legal table | FSM hygiene |
| `UnexpectedByteCount` | `accumulate_snoop` sees NDTR inconsistent — e.g. ring lapped twice between walks | Constraint D (TC latency) |
| `PreviousSlotTimeout` | post-straggle-walk `bytes_walked` falls short of `expected_predecessor_bytes` | host/protocol issue |
| `SlotTimingMiss` | SysTick CMP fires later than `fire_tick + 1 byte_time` (the R1 jitter cap) | R1, §13 parent |
| `CrcPatchDeadlineMiss` | `patch_crc()` completes after TX DMA has already read past `n−2` | Constraint B |
| `DmaOverrun` | USART1 STATR.ORE — RX outpaced DMA drain | chip limit |
| `UartError` | USART1 STATR.{PE,FE,NE} — parity / framing / noise | PHY/wire issue |

Each variant maps 1:1 to a u32 counter in `TelemetryDxlLink`. `report_fault` does a volatile RMW on the matching field. `UnexpectedByteCount` and `Done` are reserved for the bench-validation pass (A7) — the rest are live in the hot path today.

## 11. Honest accounting

What this buys:

- **R1: the 4 µs SysTick fire delay we measured at 3 Mbaud is gone.** RXNE is off; nothing same-priority can queue ahead of the fire SysTick during the jitter-sensitive window.
- **R2: control loop jitter from Fast Read snoop windows is gone at high baud.** Hundreds of USART1 trap entries (snoop's per-byte) vanish at every baud, and the ADC kernel pump (Low priority) no longer has to fight them during Fast Reads. At high baud (framing in IDLE mode, §9) `RXNEIE` stays off entirely — zero per-byte interrupts on the snoop path *or* the framing path. At low baud framing's per-byte work persists, but byte rate is naturally low (~960 IRQ/s at 9600) so total CPU load stays under a percent. CRC accumulation itself happens in 2–4 HIGH-priority ISRs total instead of ~200 USART1 ISRs.
- **R3: asymmetric Bulk Read CRC stays correct without any priority dance.** The 128-byte-predecessor + 1-byte-our-reply case works cleanly because TC + WaitingSwitch keep `snoop_head` close to NDTR all the way through, leaving only ~45 bytes of straggle for the fire ISR — well within even the smallest reply's prefetch slack.
- **R4: oversize Fast Bulk Read frames just work.** The snoop window is no longer coupled to the ring size. Whether it's 200 bytes or 4000, TC walks each full wrap into the CRC and the fire ISR catches the same small straggle at the end. No silent CRC corruption, no neighbor servo's reply lost because we sized our buffer for typical.
- **Bonus: 512 bytes of SRAM back.** The ring shrinks from 1024 to 512 (≈6% of the V006's 8 KB SRAM freed for the control loop and future features).

What this doesn't buy:

- **The chip's ~5 µs structural floor on the SysTick-fire path itself is unchanged.** That floor is the irreducible PFIC trap entry + ISR body + DMA EN write + TX_EN GPIO toggle on V006 V2 — the time from CMP-match to bytes leaving the wire even with nothing else happening. It's what "on time" *means* on this chip. The hybrid just removes the extra ~4 µs we were paying on top of it from RXNE-queueing contention. Today fire happens at floor + 4 µs; under the hybrid it happens at floor + 0. Closing the floor itself needs the TIM1 OPM hardware-driven fire lever (§13 parent, item 3).
- **Total CRC walk work is unchanged.** We're rearranging when bytes get walked across more ISRs, not skipping any.

What this trades off:

- A richer `ReplyState::Chain` variant carrying the `FastChainPhase` substage + chain bookkeeping (~20 bytes of static state), one extra constant (`SWITCH_MARGIN_US`), and a previously-unused DMA1_CH5 IRQ vector wired into the dispatch macro.
- TC_IE has to be phase-gated (enable in `start_fast_after`, disable in fire ISR and `cancel`), and TC_IF cleared before re-enable to avoid stale-pending spurious fires. ~6 lines of code.
- The constants are derived numbers with documented math, not magic values. Re-derive if `DXL_RX_BUF_LEN` shrinks below 64 (Constraint C), grows past ~1545 (Constraint A), or the smallest-reply assumption changes (Constraint B). `MAX_LENGTH` is independent of all of this — it lives on the request-parser path, not the snoop path.

## 12. Why TC, why gated, and why not just HT

The DMA controller offers both Half-Transfer (HT) and Transfer-Complete (TC) interrupts. The hybrid uses TC, gated to fire only during snoop. Two choices worth justifying:

**Why not HT.** HT fires at the ring midpoint, TC at the wrap. Together they'd double the ISR count for the same total walk work — each fire would walk half the ring (256 bytes at N = 512, 25 µs of CRC work) instead of one TC firing on the wrap (512 bytes, 51 µs). At 3 Mbaud worst-case snoop the difference is ~6 vs ~3 ISRs per Fast Last reply for identical bytes walked. TC alone — bigger chunks, fewer interrupts, same correctness.

**Why not always-on.** The temptation is to leave TC_IE on permanently and maintain a running CRC during all RX traffic — saves the stage-gating logic. But TC would then fire during every normal Sync Read / Write and phantom-noise episode too, waking the CPU for CRC work we're going to throw away. That's exactly what we're trying to avoid for the control loop. Stage-gating TC_IE costs ~6 lines of code; "always on" costs ongoing CPU jitter on every RX byte stream forever. Easy call.

**Why not just SysTick.** The other candidate we considered was the SysTick-only plan (WaitingSwitch + fire, no TC, 1024-byte ring sized to fit any plausible snoop in one shot). That works for typical traffic but breaks on R4 — oversize Fast Bulk Reads silently corrupt because the ring can't hold the whole snoop window. The hybrid keeps SysTick doing what it's good at (deterministic, tied to *our* deadlines) and adds TC to handle the wire-rate accumulation that SysTick alone can't bound.

## 13. One-paragraph summary

> The Fast Sync/Bulk Read last slave has to compute a CRC over predecessor slots' wire bytes and stitch it onto its own reply. Walking everything at fire blows past the V006's tight TX DMA prefetch slack as soon as the snoop crosses ~70 bytes for a small reply; firing RXNE per byte costs hundreds of USART1 IRQs at 3 Mbaud and shoves SysTick's fire deadline back by ~4 µs because USART1 and SysTick share PFIC priority; the SysTick-only plan that came before this one caps the snoop window at the ring size and silently CRC-corrupts oversize Fast Bulk Reads. The fix is a three-handed split: DMA Transfer-Complete on RX (CH5, stage-gated) pre-walks the CRC every time the 512-byte ring wraps; a SysTick CMP at `fire − 150 µs` walks whatever's accumulated since the last wrap; the fire ISR grabs a ~45-byte straggle and patches CRC into the TX buffer. All three handlers sit at PFIC HIGH and serialize cleanly. The ring drops from 1024 to 512 bytes, the snoop window stops being bounded by ring size, and oversize Fast Bulk Reads just work. The 150 µs margin still derives from two constraints — switch walk fits the margin (`N ≤ 10.3 × M`, comfortable at N = 512) and straggle walk fits the smallest reply's TX prefetch slack (`M < (slack − patch) / 0.03`, so M < 188 µs). The reply-scheduler FSM gains one state (`WaitingSwitch`), the RXNE composer collapses to a single owner (framing), the previously-unused DMA1_CH5 IRQ gets a stage-gated TC handler, and the 4 µs queueing penalty disappears.

## 14. Bench validation and the RXNE-tail addendum (2026-05-30)

§1–§13 are the design as written. This section is the design as it
actually shipped, after bench-instrumenting the post-fire path with a
PC3 scope marker and measuring real per-segment timings against a
pirate-fired Fast Sync chain. Two assumptions turned out to be off
enough to matter, and one new failure mode showed up that the original
design didn't anticipate.

### 14.1 Per-byte CRC walk is ~5× slower than estimated

§4.1's "~5 cycles/byte at 48 MHz → 0.1 µs/byte" is what you'd get if
the table fetch hit a single-cycle SRAM port. In reality:

- The default `dxl_protocol::crc16_continue` LUT lives in `.rodata` →
  flash, which costs 1 wait state per fetch at 48 MHz on V006.
- The walk loop has bookkeeping the cycle count missed: ring-index
  masking, function-call overhead from `ring_crc` → `crc16_continue`,
  per-iter branch.
- Each iteration is ~24 cycles, not 5.

Bench measured at 3 Mbaud (11-byte snoop, flash-resident LUT):

    walk_cost  =  5.9 µs / 11 B  =  **0.54 µs/B**

Putting the LUT in `.highcode` (RAM) shaves it to 0.50 µs/B — a 0.04
µs/B win that's measurable but inside scope-eyeball jitter, and not
worth the 256 B of SRAM. Use the portable flash-resident `crc16` and
move on. (The A/B sits in `git stash` if we ever revisit for huge
Bulk Read snoops where the 0.04 µs/B compounds.)

This 5× revision rewrites everything downstream in §4 and §5:

| §4.1 walk-at-fire example | doc estimate | bench reality |
|---|---|---|
| 26 B (5-slave Sync, 4 B payload) | 2.6 µs | 13 µs |
| 138 B (1 pred × 128 B payload) | 13.8 µs | 69 µs |
| 800 B (~6 pred × 128 B) | 80 µs | 400 µs |

The "walk at fire" strategy was already known to lose for big snoops;
this just makes it lose harder. More important is what it does to the
hybrid's straggle budget — see §14.2.

### 14.2 M had to shrink from 150 µs to 50 µs

§5 Constraint B's straggle walk has to fit slack:

    straggle_bytes = M × bytes/µs                 (M µs of new bytes)
    walk_straggle  = straggle_bytes × 0.5 µs/B    (was 0.1, now 0.5)

At 3 Mbaud with M = 150: `straggle = 45 B`, walk = **22.5 µs** —
exceeds even the typical 20 µs slack at L = 4. The smallest reply
slack (n = 4, no payload, 6.66 µs) was never gonna fit; under the
corrected per-byte estimate, neither did n = 8 with L = 4.

The shipped code uses **M = 50 µs**. That gives:

    straggle_bytes = 15 B (at 3 Mbaud)
    walk_straggle  = 7.5 µs
    + fire_now (1.2) + patch_crc (1.9) + glue (0.6)
    = post_fire_floor + walk_straggle ≈ 12.2 µs

Fits slack for L ≥ 2 at 3 M (slack = (n−2) × byte_time = 13.3 µs at
L=2 → 1.1 µs margin) and comfortable for everything bigger.

Re-deriving §5's bounds with the corrected per-byte:

| Constraint | original (0.1 µs/B) | corrected (0.5 µs/B) |
|---|---|---|
| A: switch walk ≤ M | N ≤ 10.3 × M (1545 @ M=150) | N ≤ 2 × M (100 @ M=50) ← still clears N=512 |
| B: straggle walk + patch < slack | M < (slack − 1) / 0.03 | M < (slack − 1) / 0.15 |
| C: N > M × bytes/µs at 3M | N > 45 @ M=150 | N > 15 @ M=50 |
| D: TC latency_max < N × byte_time | unchanged | unchanged |

At M = 50, smallest-slack (6.66 µs, no payload) just barely fits the
B bound; at L = 1 (slack 10 µs) it sits 2.2 µs above the floor — fine
in the mean, fragile under jitter. This is what motivates §14.3.

### 14.3 The structural gap: switch fires before predecessor starts

Zero-gap coalesce means our fire is essentially coincident with the
predecessor's last byte landing on the wire. Concretely, the predecessor
wire stream spans:

    wire_window  =  N_pred × byte_time
    fire         =  request_end + RDT + wire_window   (for Last slot, k = N-1)

The switch CMP is scheduled at `fire − M`. So switch lands at:

    switch  =  request_end + RDT + wire_window − M

If `wire_window < M`, switch lands **before the predecessor has
transmitted a single byte**. It walks zero bytes, transitions to
TxArmed, all `N_pred` bytes hit the post-fire walk.

For inj_len=1 + dut_len=1 Last at 3 M:
- N_pred = `FAST_SLOT0_PREFIX(10) + 1 = 11` bytes (see
  [reference: dxl-fast-sync-semantics] for the wire-byte formula)
- wire_window = 11 × 3.33 = 37 µs < M = 50 µs ✓ switch is useless
- post_fire = 1.2 + 5.5 + 1.9 + 0.6 = **9.2 µs**
- slack at L = 1 = 10 µs → **0.8 µs margin, ~3-4% CRC error rate**

The "small snoop, tight slack" corner. §13's three-handed design
covers everything *except* this corner — the worst slot-budget : snoop
ratio in the whole design space (any L ≤ 2 with N_pred above the
switch-fires-usefully threshold; especially L = 1 at any baud ≥ 2 M).

### 14.4 The fourth hand: RXNE for the slack-budget tail

The chain-CRC doc's R3 rejected per-byte RXNE because of the trailing
RXNE-vs-SysTick collision (slips fire by ~4 µs). The clean answer
that recovers most of RXNE's benefit *without* re-introducing the
collision is to use RXNE for **only the tail bytes that walk-at-fire
can't afford to walk**, and mask 1–2 bytes early so the post-fire
walk catches the final stragglers.

This is the fourth tier on top of §4.3's three-handed split:

| Tier | When it runs | Walks |
|---|---|---|
| TC (existing) | Every 512 B ring wrap | 512 B per fire |
| HT (existing) | Every 256 B (gated) | 256 B per fire |
| Switch (existing) | `fire − M` SysTick CMP, once | residue since last TC/HT |
| **RXNE tail (new)** | per byte, last `tail_bytes` only | 1 byte per entry |
| Walk-at-fire (existing) | TxArmed body | final 0-2 stragglers |

**Sizing `tail_bytes`** — the count of bytes that need RXNE-tier
coverage. Pre-walk eliminates the walk-at-fire residue exactly when
the residue would exceed the slack budget:

    slack_budget    =  slack − post_fire_floor       (e.g. 10 − 4.7 = 5.3 µs at L=1, 3M)
    slack_budget_B  =  slack_budget / 0.5 µs/B       (10.6 B)
    fire_residue_B  =  M × bytes/µs                  (15 B @ M=50, 3M)
    tail_bytes      =  max(0, fire_residue_B − slack_budget_B + GUARD)

For L = 1 at 3 M: `tail_bytes = max(0, 15 − 10.6 + 2) = 6`. Six RXNE
entries during the last ~20 µs of the predecessor's wire window.
Pre-walks 6 bytes, leaves ~9 bytes for the post-fire walk — wait,
that's worse. The arithmetic above only works when switch *does* fire
usefully. For the failing corner (`wire_window < M`), switch walks
zero, so the relevant budget is the full `N_pred` minus what
walk-at-fire can afford:

    tail_bytes  =  max(0, N_pred − slack_budget_B + GUARD)

For L = 1 at 3 M with N_pred = 11: `tail_bytes = max(0, 11 − 10.6 + 2)
= 2`. Two RXNE entries pre-walk bytes 1-2, walk-at-fire handles
bytes 3-11 (9 × 0.5 = 4.5 µs + floor 4.7 = 9.2 µs — wait, no
improvement?).

Let me re-derive cleanly. The goal is "make the post-fire walk small
enough to fit slack." If we use RXNE to pre-walk `k` bytes, the
post-fire walk is `(N_pred − k)`. Solving for k:

    (N_pred − k) × 0.5 + post_fire_floor + jitter_margin  <  slack
    k > N_pred − (slack − post_fire_floor − jitter_margin) / 0.5
    k > N_pred − 2 × (slack − post_fire_floor − jitter_margin)

For L = 1 at 3 M, N_pred = 11, slack = 10, floor = 4.7, jitter = 0.5:
    k > 11 − 2 × (10 − 4.7 − 0.5)
    k > 11 − 9.6 = 1.4
    → **k = 2** suffices (round up + GUARD).

Post-fire walk drops from 11 B (5.5 µs) to 9 B (4.5 µs). Total
post-fire = 4.7 + 4.5 = 9.2 µs ... still 9.2 µs! That doesn't help.

The issue: with tiny `tail_bytes`, the floor (4.7 µs) dominates and
shrinking the walk by 1 µs isn't enough. To actually close the gap
we need bigger k — pre-walk almost everything, leaving ≤ 2 bytes
post-fire:

    k = N_pred − GUARD     (GUARD = 2 → walk-at-fire handles last 2)
    post_fire_walk = GUARD × 0.5 = 1 µs
    total          = 4.7 + 1 = **5.7 µs**

That's a 3.5 µs improvement, well under the 10 µs slack. The cost is
9 RXNE entries during the predecessor's 37 µs wire window. At 3 µs
ISR-entry each, that's 27 µs of CPU spread across the wire window —
sustainable.

**Revised sizing rule** — use RXNE-tail whenever post-fire walk-only
violates slack:

    use_rxne_tail  =  N_pred × 0.5 + post_fire_floor + jitter > slack
    tail_bytes     =  N_pred − GUARD    when use_rxne_tail
                      0                  otherwise
    GUARD          =  2                 (catch last byte + 1 for jitter)

For the matrix at 3 M, this resolves cleanly:

| Scenario | N_pred | post-fire only | use RXNE-tail? | with RXNE-tail |
|---|---|---|---|---|
| 2 sl × L=1 | 11 | 10.2 µs > 10 ✗ | yes | 5.7 µs |
| 5 sl × L=1 | 20 | 14.7 > 10 ✗ | yes | 5.7 µs |
| 2 sl × L=4 | 14 | 11.7 < 20 ✓ | no | (unchanged) |
| 2 sl × L=16 | 26 | 17.7 < 60 ✓ | no | (unchanged) |
| 10 sl × L=4 | 62 | covered by switch (wire_window > M) | no | (unchanged) |
| Bulk 1500 B, L=4 | 1500 | covered by HT/TC + switch | no | (unchanged) |

RXNE-tail kicks in **only when post-fire walk alone can't meet slack
and switch can't cover the gap** — exactly the structural corner
§13's design left open. Comfortable cases pay zero RXNE cost.

### 14.5 Timeline with RXNE-tail (3 M, 2 slaves, L = 1)

    t = 0 µs       request_end (USART1 IDLE, backdated)
    t ≈ 10 µs      dispatcher parses, calls start_fast_after.
                     STATE = Chain{ CatchupArmed, N_pred=11,
                                    rxne_tail_at=0,
                                    fire_tick=request_end + 287 - latency }
                     RXNEIE: ON (tail_bytes=9, arm immediately
                                 since wire_window < M)
    t = 250 µs     INJ first start-bit on wire
    t = 253.33     INJ byte 1 stop complete → RXNE fires, walks 1 B, bytes_walked=1
    t = 256.66     byte 2 → walk → bytes_walked=2
       ...
    t = 280 µs    byte 10 → walk → bytes_walked=10
                  bytes_walked >= N_pred − GUARD(2)? 10 >= 9 ✓
                  mask RXNEIE
    t = 283.33     byte 11 arrives → no RXNE (masked); DMA writes ring
    t ≈ 284 µs    SysTick fire CMP (fire_tick) → TxArmed body:
                     fire_now (TX_EN HIGH, DMA CH4 enable)  — 1.2 µs
                     accumulate_snoop — busy-waits NDTR for byte 11,
                                        walks 2 B (10 + 11)        — 1.5 µs
                     patch_crc                                      — 1.9 µs
                     total post-fire ≈ 5 µs (was 9.2 µs)
    t = 286.66    INJ byte 11 stop complete (during our fire ISR;
                  NDTR drop arrives during busy-wait)
    t ≈ 291 µs    DUT first start-bit on wire (≤ 1 byte_time gap
                  from INJ end — within spec coalesce tolerance)

Post-fire path: **9.2 µs → 5 µs**, well under 10 µs slack with
~5 µs margin. Jitter source shifts from "variable byte count at
fire" to "busy-wait NDTR for the late byte" — the latter is bounded
by 1 byte_time (3.33 µs at 3 M) deterministically.

### 14.6 State machine deltas

`ReplyState::Chain` gains one field:

    Chain { ...,
            rxne_tail_at: u16,    — bytes_walked threshold to mask RXNEIE
                                    (= N_pred − GUARD when use_rxne_tail,
                                    = u16::MAX otherwise)
          }

`start_fast_after` computes `use_rxne_tail`, sets `rxne_tail_at`, and
enables RXNEIE if `use_rxne_tail`. No new `FastChainPhase` variant —
RXNE-tail behavior is gated by the existing `CatchupArmed` / `Snoop` /
`Catchup` phase checks plus the `rxne_tail_at` counter.

`on_usart1` gains an RXNE branch (currently runs only the IDLE / TC
bodies):

    on USART1 entry, RXNE pending:
        if STATE = Chain { phase ∈ {CatchupArmed, Snoop, Catchup},
                           bytes_walked, snoop_head, bulk_crc, rxne_tail_at }:
            write_pos = current_rx_write_pos()
            if write_pos != snoop_head:
                bulk_crc = ring_crc(bulk_crc, ring, snoop_head, write_pos)
                bytes_walked += (write_pos − snoop_head) & RX_MASK
                snoop_head = write_pos
            if bytes_walked >= rxne_tail_at:
                set_rxne_irq(false)              # mask before SysTick collision

`accumulate_snoop` (TxArmed body) needs a brief busy-wait for the
final byte to land in the ring before walking:

    fn wait_for_predecessor_end(expected: u16):
        let deadline = systick::ticks() + BYTE_TIME_TICKS × 2
        loop:
            if current_rx_write_pos() − snoop_start >= expected: return Ok
            if systick::ticks() >= deadline: return Err(PredecessorTimeout)

Wait bounded by ~1 byte_time (3.33 µs at 3 M), aborts to
`PreviousSlotTimeout` fault if no byte arrives.

### 14.7 What §13's design got right, what §14 changes

Kept:
- TC + HT pre-walking for large snoops. R4 (oversize Bulk Read
  correctness) is unchanged.
- Switch + walk-at-fire for medium snoops where slack comfortably
  covers the M=50 µs residue.
- PFIC priority pinning, FSM phase model, fault taxonomy.

Revised:
- M shrunk 150 → 50 to make the residue fit slack with the corrected
  0.5 µs/B walk cost.
- §5's per-byte constant (0.1 → 0.5) and worked examples re-derive
  with the new constant.

Added:
- RXNE-tail tier, gated by per-request slack budget calculation in
  `start_fast_after`.
- `Chain.rxne_tail_at` field + RXNE handler body.
- `accumulate_snoop` busy-wait for predecessor tail byte.

Net effect: 3 M Last 1B (the structural corner that motivated this
addendum) drops from 3-4% CRC error rate to design-bounded zero. No
regression on L ≥ 2 or on big Bulk Reads — those paths are unchanged
because `use_rxne_tail` evaluates false.

## 15. The next lever: TIM2 OPM hardware fire (2026-05-31)

§14's RXNE-tail tier closed the CRC-correctness gap — 3 M Last 1B
drops from 3-4% CRC error to design-bounded zero. But the bench verify
matrix kept showing the same residue from a different angle: ~25-30%
of shots ship with a visible idle gap on the wire between predecessor
stop bit and our reply's start bit. The CRC is right, the slot fires,
the host parses fine — but coalesce is broken because the wire wasn't
actually zero-gap.

Where's the gap coming from? Scope captures of the SysTick fire ISR
body break the path into four phases:

    dbg high  → fire_now                  ≈ 1.2 µs
    dbg low   → (gap)                     ≈ 0.3 µs
    stat high → accumulate_snoop + patch_crc  ≈ 3.7 µs
    stat low  → (gap)                     ≈ 0.3 µs
    dbg high  → wait_and_accumulate_tail  ≈ 2.0 µs

Total post-fire path ≈ 7.5 µs, comfortably under the 10 µs slack at
L=1 / 3 M (so CRC patching is fine — confirming the 0 CRC errors).
But `fire_now` is only the *last* 1.2 µs of the pre-wire path. Ahead
of it: PFIC trap entry (~5 µs irreducible on V006 V2A) plus a few
hundred ns of `on_systick` body before `fire_now` starts. So
CMP-match → first wire bit ≈ 6.4 µs ± PFIC jitter.

The `TX_FAST_LATENCY_TICKS` tune subtracts that mean from the deadline,
so on average we land on time. But the jitter is ±a microsecond or so
(varies with what was running when CMP matched), and at 3 M that's
±a-third-of-a-bit-time. Half the time the start bit lands a smidge
late; half early; in the worst case we either eat into the
predecessor's stop bit or leave an idle slot the host's IDLE flag
latches onto and the coalesced frame splits.

So extra_idle isn't a snoop saturation problem. It's a fire-arrival
jitter problem, and PFIC entry latency is the dominant variable.

### 15.1 What TIM2 OPM buys

TIM2 is the only timer on the V006 with all four pieces we need:

- **CNT we own.** TIM1's counter is busy with motor PWM. TIM3 has no
  prescaler and no OPM. TIM2's free (modulo the bench/encoder
  coexistence question in §15.3).
- **OPM bit.** Once CC4 matches and ARR wraps, the counter stops
  itself. No re-arm IRQ needed.
- **Free DMA paths.** TIM2_CH4 routes to DMA1_CH7 (the alternate
  consumer is USART2_RX, which we don't use). TIM2_UP routes to
  DMA1_CH2 (alternates SPI1_RX / TIM1_CH1, also unused).
- **Output compare on a free DMA channel.** CC4 generates a DMA
  request to Ch7, and the OPM stops cleanly on UP.

Two DMA channels because the fire deadline triggers two distinct
register writes: `GPIOC.BSHR` to assert TX_EN HIGH, and
`USART1.CTLR3` to set DMAT (kicks off the USART pulling bytes from
the already-armed CH4 TX DMA). Each DMA channel writes one target per
trigger, so we need two events. CC4 first, UP one or two ticks later
— OPM stops the counter cleanly after UP fires.

Layout:

    arm:
        DMA1.Ch7 pre-configured (peri=GPIOC.BSHR, src=TX_EN_HIGH_pattern, NDTR=1)
        DMA1.Ch2 pre-configured (peri=USART1.CTLR3, src=DMAT_pattern, NDTR=1)
        DMA1.Ch4 pre-configured for TX as today (EN=1, DMAT held off via CTLR3=0)
        CCR4 = deadline_tick - small_N     (N = ~half a bit-time; see §15.2)
        ARR  = deadline_tick
        CEN=1  (counter starts, OPM=1)

    hardware-clocked:
        CC4 match → DMA1.Ch7 writes BSHR  → TX_EN goes HIGH
        ARR wrap  → UP event              → DMA1.Ch2 writes CTLR3 → DMAT enabled
                                         → USART pulls from Ch4
                                         → start bit on wire ½ baud-tick later
        OPM stops CNT, CEN clears, counter idle until next arm

PFIC entry leaves the critical path. The ISR-body contribution
leaves. Fire latency drops from ~6.4 µs ± jitter to a sub-microsecond
fixed pipeline plus the ½ baud-tick sync mean. The latter is the
silicon floor — even dxl-pirate's TIM1-OPM fire path on the V20x
sees it, per `[[pirate-fire-jitter-floor]]`.

### 15.2 TX_EN goes first, but only by a tick

TX_EN HIGH puts our transceiver in TX mode. If it goes HIGH while
the predecessor is still driving the wire LOW (mid-byte), we get
**bus contention** — two push-pull CMOS drivers fighting on the
wire, through-current spikes, garbled bits for any other listener
on the bus. Not what we want.

The safe window for asserting TX_EN HIGH early is the predecessor's
**stop-bit window** (~1 bit-time wide, 333 ns at 3 M). Both ends
drive HIGH during the stop bit, so no contention. Outside that
window: contention on the early side, garbled start-bit on the late
side.

Setting `CCR4 = ARR - N` for `N ≈ half a bit-time in ticks` lands
TX_EN's BSHR write a few hundred ns before DMAT's CTLR3 write —
squarely inside the stop-bit safe window and just slightly before
the start-bit edge. The OPM stop-on-UP guarantees nothing re-arms
between fires.

(There's no AF pin route from TIM2_CH4 to the current TX_EN pin PC2,
so we can't drive TX_EN HIGH via TIM2's OC4 pin output directly. The
DMA-write-to-BSHR path achieves the same effect without a board
change.)

### 15.3 Coexistence with future bench/encoder use

A reasonable future use of TIM2 is reading a rotary encoder for
motor-side calibration — TIM2 in encoder mode counts edges on TI1 +
TI2 inputs, with the counter direction tracking the encoder's phase.
The two modes (OPM fire vs encoder counter) are mutually exclusive:
encoder mode owns CNT for edge counting and can't share with OPM's
deadline countdown.

Resolved as a build-time choice via two mutually-exclusive Cargo
features in `firmware/ch32`:

    dxl-systick-fire   (default)  TIM2 free for encoder mode; SysTick
                                  CMP drives fire (today's path)
    dxl-hw-fire                   TIM2 in OPM for fire; encoder mode
                                  unavailable

The arm/cancel API surface stays the same; only the body of a small
dispatch differs by cfg. Both paths share the same FSM, state, and
bench-validated correctness work from §14 — `dxl-hw-fire` is purely
a fire-path optimization, not a rewrite.

### 15.4 What disappears, what changes

CT layout stays stable. Under `dxl-hw-fire`, the two
`dxl_tx_{plain,fast}_latency_us` fields turn into reserved bytes
(read returns 0, write returns DXL Status `0x07 Access Error`) at
the same byte offsets — no longer tunable, and host tools that try
to tune get a hard error instead of a silent no-op. A new
`comms.fire_mode` RO byte lets the pirate tune script branch its
flow at session start. The reserved-byte mechanic is a small new
`#[ct_field(reserved)]` proc-macro attribute on the control-table
derive — handy for any future "this slot only exists in dev builds"
case too.

`FIRE_ADVANCE_FINE_TICKS` and the `clock_trim` /
`clock_fine_trim_us` CT fields stay common across both modes. They
compensate for HSI drift in the deadline math, which is
mode-independent — both timers (SysTick and TIM2) run on HCLK and
drift identically with HSI.

A new compile-time const `HW_FIRE_PIPELINE_TICKS` captures the small
fixed gap from CC4 match to start-bit (DMA arbitration + memory
write + USART pull + start-bit shift). Sub-microsecond, no per-chip
variance worth tuning. If bench validation ever finds per-chip
nudging matters, we promote it to an atomic without changing the CT
schema.

### 15.5 Honest accounting

What this should buy (pending H-series implementation + bench, tasks
#57-#65):

- **Extra_idle floor close to zero** at 3 M Last 1B. Wire jitter
  reduces to the ½ baud-tick start-bit sync mean (~170 ns at 3 M).
- **PFIC budget back for the ADC pump and main loop.** The fire path
  no longer contends for the trap-entry window — only the post-fire
  CRC patch runs in software, and that stays well under slack
  budget per §14.
- **RXNE-tail tier likely no longer needed under HW.** With ~5 µs
  of slack reclaimed, the post-fire walk can cover N_pred = 11
  directly in most cases. Plan is to keep both tiers under SysTick
  mode (where the tail still matters) and drop RXNE-tail under HW
  mode (one less ISR class) — task H6 confirms with the bench.

What this doesn't buy:

- **The ½ baud-tick sync floor is silicon.** Can't be removed
  without changing the USART. At 3 M that's ~170 ns mean, well
  under the 3.33 µs jitter cap. At higher hypothetical bauds it'd
  start to matter.
- **TIM2 unavailable for anything else under HW mode.** Encoder
  mode is the obvious tradeoff; the feature gate keeps the SysTick
  path alive for dev builds that need TIM2 for something else.

What this trades off:

- One more DMA channel (Ch2) spoken for, plus the pre-armed Ch7.
- TIM2 setup at bringup, then OPM per-arm.
- A small cfg surface around the latency atomics + CT field types
  (the new `#[ct_field(reserved)]` attribute carries most of this).

The chain-CRC mechanics from §1-§14 don't change — TIM2 OPM lives
alongside the snoop path, not on top of it. Worth re-reading §13
("honest accounting" for the SysTick design) and noticing that this
plan closes the one bullet that section left open: "the chip's ~5 µs
structural floor on the SysTick-fire path is unchanged. Closing the
floor itself needs the TIM1 OPM hardware-driven fire lever (§13
parent, item 3)." The lever turned out to be TIM2 OPM on this chip
(TIM1's CNT is too tied to motor PWM), but the shape is the same.

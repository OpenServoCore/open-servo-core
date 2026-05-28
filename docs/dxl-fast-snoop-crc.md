# DXL Fast Last-Slave Snoop CRC on the CH32V006

Companion to [dxl-rx-timing.md](dxl-rx-timing.md). Read that first — this doc assumes you already know what "the wire-end timestamp," "SysTick CMP fire," and "the V006's two PFIC priorities" mean.

**TL;DR.** When we're the last slave in a DXL Fast Sync/Bulk Read, we have to compute a CRC over bytes we didn't generate — every byte the predecessor slaves' replies put on the wire — and stitch our own bytes onto the end. We've tried two strategies and bounced off both: walking everything at fire blows past the TX DMA prefetch slack for any large snoop; firing RXNE per byte pollutes the system with hundreds of interrupts and shoves SysTick's fire deadline back by ~4 µs at 3 Mbaud. The fix is a three-handed split: DMA's Transfer-Complete IRQ on the RX channel pre-walks the CRC every time the ring wraps (so the ring no longer has to hold the whole snoop window); a SysTick CMP fires a fixed margin before the real deadline and closes whatever gap remains; the fire ISR grabs the tail and patches CRC into the TX buffer. The RX ring drops from 1024 to 512 bytes, the snoop window stops being bounded by ring size, and oversize Fast Bulk Reads stop silently corrupting the bus. The 150 µs margin is still derived from baud and slack, not magical.

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

**Constraint C — no TC inside the straggle window.** If TC fires between switch and fire, it'd reset `snoop_head` mid-straggle and the fire ISR would walk the wrong range. Avoid by sizing N so DMA can't fill another full ring during M:

    N > M × bytes/µs = 45 bytes at 3 Mbaud, M = 150
    →  N ≥ 64

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

## 6. State machine update

One state added to the reply-scheduler FSM from §7.2 of the parent doc, plus a DMA TC handler that runs alongside it.

    states (was 3, now 4):
      Idle
      WaitingPlain
      WaitingSwitch    ← new: snoop accumulation in progress
      WaitingFire      ← unchanged shape; entered from arm OR from WaitingSwitch

    new transitions:
      Idle → WaitingSwitch     Fast last-slave reply with fire_us > M
                               - pre-configure DMA CH4 (count + source, EN=0)
                               - record snoop_head = parsed_end & RX_MASK
                               - clear DMA1_CH5 TC_IF, enable TC_IE
                               - SysTick CMP at: wire-end + fire_us − M

      WaitingSwitch → WaitingFire    first CMP fires
                               - accumulate_snoop()  ← walks ring[snoop_head..NDTR]
                               - SysTick CMP at: wire-end + fire_us

    new handler (runs alongside the FSM, no stage transition):
      DMA1_CH5 TC fires        while stage ∈ {WaitingSwitch, WaitingFire}
                               - walk ring[snoop_head..N] into bulk_crc
                               - snoop_head = 0
                               - clear TC_IF

    unchanged transitions (shape):
      Idle → WaitingFire       Only-slot path, OR Fast last-slave with fire_us ≤ M
                               - (TC_IE stays off — no snoop, walk at fire only)

      WaitingFire → Idle       CMP fires:
                               - fire_now()         ← jitter-critical, first
                               - accumulate_snoop() ← straggle catch-up
                               - patch CRC into TX buffer trailing 2 bytes
                               - disable DMA1_CH5 TC_IE

`cancel` still wipes back to Idle from any state and additionally disables TC_IE. Same idempotent contract.

All three handlers (TC, WaitingSwitch CMP, WaitingFire CMP) sit at PFIC HIGH priority and serialize cleanly — each updates `snoop_head` atomically with respect to the others, and the order they happen to fire in doesn't affect correctness. The TC ISR is also the simplest of the three: walk a fixed-size block, reset `snoop_head` to a constant, clear a flag.

The `WaitingSwitch → WaitingFire` transition is still subject to the "CNTIF latches only on `CNT == CMP` up-count" gotcha from §8.3 of the parent doc — the set-and-recheck pattern after writing the new CMP is mandatory, otherwise an overshoot in the switch ISR silently sleeps until the next 89-second wrap.

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

- `DMA1_CHANNEL5` IRQ vector wired up (currently unused). Priority HIGH (same as USART1 + SysTick), gated by stage so it only fires during snoop windows.
- One `Stage` variant (`WaitingSwitch`) and one constant (`SWITCH_MARGIN_US = 150`).

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

## 10. Honest accounting

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

- One extra `Stage`, one extra constant (`SWITCH_MARGIN_US`), and a previously-unused DMA1_CH5 IRQ vector wired into the dispatch macro.
- TC_IE has to be stage-gated (enable in `start_fast_after`, disable in fire ISR and `cancel`), and TC_IF cleared before re-enable to avoid stale-pending spurious fires. ~6 lines of code.
- The constants are derived numbers with documented math, not magic values. Re-derive if `DXL_RX_BUF_LEN` shrinks below 64 (Constraint C), grows past ~1545 (Constraint A), or the smallest-reply assumption changes (Constraint B). `MAX_LENGTH` is independent of all of this — it lives on the request-parser path, not the snoop path.

## 11. Why TC, why gated, and why not just HT

The DMA controller offers both Half-Transfer (HT) and Transfer-Complete (TC) interrupts. The hybrid uses TC, gated to fire only during snoop. Two choices worth justifying:

**Why not HT.** HT fires at the ring midpoint, TC at the wrap. Together they'd double the ISR count for the same total walk work — each fire would walk half the ring (256 bytes at N = 512, 25 µs of CRC work) instead of one TC firing on the wrap (512 bytes, 51 µs). At 3 Mbaud worst-case snoop the difference is ~6 vs ~3 ISRs per Fast Last reply for identical bytes walked. TC alone — bigger chunks, fewer interrupts, same correctness.

**Why not always-on.** The temptation is to leave TC_IE on permanently and maintain a running CRC during all RX traffic — saves the stage-gating logic. But TC would then fire during every normal Sync Read / Write and phantom-noise episode too, waking the CPU for CRC work we're going to throw away. That's exactly what we're trying to avoid for the control loop. Stage-gating TC_IE costs ~6 lines of code; "always on" costs ongoing CPU jitter on every RX byte stream forever. Easy call.

**Why not just SysTick.** The other candidate we considered was the SysTick-only plan (WaitingSwitch + fire, no TC, 1024-byte ring sized to fit any plausible snoop in one shot). That works for typical traffic but breaks on R4 — oversize Fast Bulk Reads silently corrupt because the ring can't hold the whole snoop window. The hybrid keeps SysTick doing what it's good at (deterministic, tied to *our* deadlines) and adds TC to handle the wire-rate accumulation that SysTick alone can't bound.

## 12. One-paragraph summary

> The Fast Sync/Bulk Read last slave has to compute a CRC over predecessor slots' wire bytes and stitch it onto its own reply. Walking everything at fire blows past the V006's tight TX DMA prefetch slack as soon as the snoop crosses ~70 bytes for a small reply; firing RXNE per byte costs hundreds of USART1 IRQs at 3 Mbaud and shoves SysTick's fire deadline back by ~4 µs because USART1 and SysTick share PFIC priority; the SysTick-only plan that came before this one caps the snoop window at the ring size and silently CRC-corrupts oversize Fast Bulk Reads. The fix is a three-handed split: DMA Transfer-Complete on RX (CH5, stage-gated) pre-walks the CRC every time the 512-byte ring wraps; a SysTick CMP at `fire − 150 µs` walks whatever's accumulated since the last wrap; the fire ISR grabs a ~45-byte straggle and patches CRC into the TX buffer. All three handlers sit at PFIC HIGH and serialize cleanly. The ring drops from 1024 to 512 bytes, the snoop window stops being bounded by ring size, and oversize Fast Bulk Reads just work. The 150 µs margin still derives from two constraints — switch walk fits the margin (`N ≤ 10.3 × M`, comfortable at N = 512) and straggle walk fits the smallest reply's TX prefetch slack (`M < (slack − patch) / 0.03`, so M < 188 µs). The reply-scheduler FSM gains one state (`WaitingSwitch`), the RXNE composer collapses to a single owner (framing), the previously-unused DMA1_CH5 IRQ gets a stage-gated TC handler, and the 4 µs queueing penalty disappears.

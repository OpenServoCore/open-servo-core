# DXL Fast Last-Slave Snoop CRC on the CH32V006

Companion to [dxl-rx-timing.md](dxl-rx-timing.md). Read that first — this doc assumes you already know what "the wire-end timestamp," "SysTick CMP fire," and "the V006's two PFIC priorities" mean.

**TL;DR.** When we're the last slave in a DXL Fast Sync/Bulk Read, we have to compute a CRC over bytes we didn't generate — every byte the predecessor slaves' replies put on the wire — and stitch our own bytes onto the end. We've tried two strategies and bounced off both: walking everything at fire blows past the TX DMA prefetch slack for any large snoop; firing RXNE per byte pollutes the system with hundreds of interrupts and shoves SysTick's fire deadline back by ~4 µs at 3 Mbaud. The fix is a second SysTick CMP deadline a fixed margin before fire: peek the RX DMA cursor (NDTR), bulk-walk the ring into a running CRC, let a small tail wait until fire. Two SysTick entries per Fast last-slave reply, zero RXNE entries. The margin (150 µs) is derived, not magical.

---

## 1. Three things we're trying to get right

Before getting into the design it helps to spell out what we're actually after. Three things, all hard-won — we'd like to hang onto every one of them.

**R1 — the fire SysTick lands on schedule.** Our reply has to start at exactly `wire-end + RDT + slot_offset` so it coalesces with the predecessor slot at zero idle gap. DXL Fast's jitter cap is one byte time — 3.33 µs at 3 Mbaud — and anything that shoves the fire SysTick back by more than a couple of microseconds breaks coalesce. The current code's RXNE-per-byte snoop hits exactly that: the last RXNE for slot N-2 tends to land inside the inter-slot idle gap, and since USART1 and SysTick share PFIC priority (§8.4 parent), SysTick CMP queues behind the in-flight RXNE. On the bench we see ~4 µs of slip at 3 Mbaud. That's the thing we most want to fix.

**R2 — snoop CRC for the early slots happens "at DMA," not per byte.** DMA CH5 is already collecting every snoop byte silently; there's no protocol reason the CPU needs to wake up for each one. Per-byte RXNE stacks ~200 USART1 trap entries per Fast Last reply at 3 Mbaud, each ~3 µs of pure trap-entry overhead. That's ~600 µs of CPU spent doing nothing useful every Fast Read, and the ADC kernel pump (20 kHz, Low priority) gets preempted by every one of them — control loop jitter goes up whenever the bus is busy. We'd rather let DMA do the byte-shoveling it's built for and walk the buffer in batches at moments we pick.

**R3 — slot N-2 is the awkward one, and whatever we do here can't compromise R1.** N-2's bytes arrive at the very end of the snoop window, right when fire is imminent. RXNE-during-N-2 is exactly what causes R1's failure, so if we drop RXNE we have to put N-2's CRC somewhere else. Post-fire is the natural place — TX is already shifting and the CPU has the `(n-2) × byte_time` prefetch window to work in (§3) — but that window is only 6–20 µs at 3 Mbaud, much less than the walk time for an asymmetric Bulk Read where one predecessor has a big payload and we have a tiny one. Two ways out: (a) drop USART1 to Low so SysTick at High preempts RXNE during snoop, or (b) get rid of RXNE entirely and find a CRC path for N-2 that respects the slack ceiling. V006's two-level PFIC makes (a) hard to layer cleanly without surprising the framing layer, so this design goes with (b).

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

Asymmetric Bulk Read is the killer case: one slave returning `MAX_READ = 128` bytes, us returning 1 byte. 138 wire bytes to walk, 6.66 µs of slack. We lose.

### 4.2 RXNE per byte (the current code)

Enable USART1 RXNE_IE for the duration of the snoop window. Each received byte fires a USART1 IRQ; the handler reads NDTR, folds new bytes into the running CRC, advances `snoop_head`. By fire time the CRC is essentially current — the fire ISR sweeps up at most a one-byte straggle.

The CRC math is fine. The cost is volume. At 3 Mbaud a 200-byte snoop fires 200 USART1 IRQs in 666 µs, each costing ~3 µs of PFIC trap entry plus the snoop body.

Worse: the **last** RXNE in the window lands inside the inter-slot idle gap right before our fire deadline, and SysTick CMP — which is supposed to fire at that deadline — sits at the same PFIC priority as USART1 (§8.4 parent). Same-priority IRQs don't preempt each other. SysTick gets queued behind the in-flight USART1 entry, pushing fire back by **~4 µs at 3 Mbaud** (bench-measured). On a chip whose structural fire floor is already ~5 µs (§13 parent), this 4 µs penalty is the difference between "barely meets spec" and "definitely doesn't."

And the 200 trap entries per Fast Read interleave badly with the 20 kHz ADC kernel pump. Control loop jitter goes up under traffic.

### 4.3 The plan: SysTick peeks the DMA cursor

Two SysTick CMP deadlines instead of one. The first fires a fixed margin before the real fire deadline, peeks the DMA cursor, bulk-walks whatever's there. The second is the same fire deadline we had before; it sweeps up a small straggle, fires TX, patches CRC.

    arm                            fire − M                                fire
    |                                  |                                     |
    | snoop bytes streaming into RX DMA ring (no CPU)                        |
    |                                  |                                     |
    |                              SysTick fires:                       SysTick fires:
    |                              walk ring[snoop_head..NDTR]          fire_now()  ← first!
    |                              advance snoop_head                   walk ring[snoop_head..NDTR]
    |                              ← bulk-walk catches up               patch CRC into TX buf
    |                                                                   ↑
    |                                                                   last 2 bytes of TX

DMA's RX channel (CH5, always on, §12 parent) is doing all the actual reception work. The CPU isn't touching individual bytes at all — it's just *sampling DMA's position* at two well-chosen moments and walking the ring between samples. Wrap-aware (§8.2 parent) since the ring is circular.

Crucially: **RXNE_IE stays off the entire time.** No per-byte interrupts. SysTick fires unimpeded at the deadline.

## 5. Sizing the margin

Two constraints, both worst-case at 3 Mbaud (the fastest baud is also the tightest slack):

**Constraint A — switch walk has to fit inside the margin.** During the margin window `M`, DMA collects `M × 0.3` bytes (3 Mbaud = ~0.3 bytes/µs). Everything before that arrived earlier and gets bulk-walked at switch:

    walk_switch = (total − M × 0.3) × 0.1   µs    ≤   M
    →  M ≥ total / 10.3

Worst-case `total` is bounded by the RX ring size: `DXL_RX_BUF_LEN = 1024` bytes. So `M ≥ 99 µs`.

**Constraint B — straggle walk has to fit inside the slack.** Straggle walk plus patch overhead must clear before DMA reads the CRC positions:

    (M × 0.3) × 0.1  +  patch  <  slack
    →  M < (slack − patch) / 0.03

Worst-case slack is the smallest possible reply: `n = 4` (ERR + ID + 0 data + CRC), slack = 6.66 µs. Patch overhead is ~1 µs (folds in 4-byte buffer + 2 writes). So `M < 188 µs`.

**Any margin in [100, 188] µs works for every (slot count × payload size) combination bounded by the 1024-byte RX ring.** Pick `M = 150 µs` for headroom on both sides.

### Worked examples

The pathological asymmetric Bulk Read case — 128-byte predecessor payload, 1-byte our payload:

    fire_us       = 138 × 3.33  = 460 µs     (138 wire bytes before us)
    switch_us     = 460 − 150   = 310 µs
    switch walk   = (138 − 45) × 0.1 = 9.3 µs  ← fits 150 µs margin ✓
    straggle      = 45 bytes
    fire-time     = 45 × 0.1 + 0.5 = 5 µs       ← fits 10 µs slack ✓ (5-byte reply)

Realistic worst case — ~6 predecessors with 128-byte payloads, us with 1 byte:

    fire_us       = 800 × 3.33  ≈ 2664 µs
    switch_us     = 2664 − 150  = 2514 µs
    switch walk   = (800 − 45) × 0.1 = 75 µs   ← fits 150 µs margin ✓
    fire-time     = 5 µs                        ← fits 6.66 µs slack ✓ (4-byte reply)

Typical Fast Sync — 5 slaves with 4-byte payloads:

    fire_us       = 26 × 3.33  = 86 µs          (only 26 wire bytes before us)
    fire_us < M   → skip WaitingSwitch entirely
    walk-at-fire  = 26 × 0.1   = 2.6 µs         ← fits 20 µs slack ✓ (8-byte reply)

When `fire_us ≤ M` the snoop is small enough that one walk at fire fits trivially. No second SysTick entry needed; the FSM transitions straight from arm into `WaitingFire`.

## 6. State machine update

One state added to the reply-scheduler FSM from §7.2 of the parent doc:

    states (was 3, now 4):
      Idle
      WaitingPlain
      WaitingSwitch    ← new: snoop bulk-walk pending
      WaitingFire      ← unchanged shape; entered from arm OR from WaitingSwitch

    new transitions:
      Idle → WaitingSwitch     Fast last-slave reply with fire_us > M
                               - pre-configure DMA CH4 (count + source, EN=0)
                               - record snoop_head = parsed_end & RX_MASK
                               - SysTick CMP at: wire-end + fire_us − M
                               - (no RXNE — that's the point)

      WaitingSwitch → WaitingFire    first CMP fires
                               - accumulate_snoop()  ← walks ring[snoop_head..NDTR]
                               - SysTick CMP at: wire-end + fire_us
                               - (still no RXNE)

    unchanged transitions (shape):
      Idle → WaitingFire       Only-slot path, OR Fast last-slave with fire_us ≤ M

      WaitingFire → Idle       CMP fires:
                               - fire_now()         ← jitter-critical, first
                               - accumulate_snoop() ← straggle catch-up
                               - patch CRC into TX buffer trailing 2 bytes

`cancel` still wipes back to Idle from any state. Same idempotent contract.

The `WaitingSwitch → WaitingFire` transition is subject to the "CNTIF latches only on `CNT == CMP` up-count" gotcha from §8.3 of the parent doc — the set-and-recheck pattern after writing the new CMP is mandatory, otherwise an overshoot in the switch ISR (a 75 µs walk for a giant snoop is plausible) silently sleeps until the next 89-second wrap.

## 7. What disappears

The snoop owner of RXNE_IE no longer exists. The OR-composer from §7.3 of the parent doc collapses to a single owner (framing).

In code terms:

- `dxl_fast::on_rxne` is deleted — nothing ever enables RXNE for snoop.
- The dispatch from `irq.rs::on_usart1` to `dxl_fast::on_rxne` is deleted.
- `usart::set_rxne_irq` stays in the HAL — the framing layer still uses it in RXNE mode (§5 parent).

USART1 IRQ entries during a Fast last-slave snoop window go from `N_wire_bytes` (today) to **zero**. SysTick CMP entries go from `1` to `1 or 2` depending on whether `fire_us > M`.

## 8. Honest accounting

What this buys:

- **R1: the 4 µs SysTick fire delay we measured at 3 Mbaud is gone.** RXNE is off; nothing same-priority can queue ahead of the fire SysTick.
- **R2: control loop jitter from Fast Read snoop windows is gone.** Hundreds of USART1 trap entries vanish, and the ADC kernel pump (Low priority) no longer has to fight them. CRC accumulation happens in 1–2 SysTick ISRs total instead of ~200 USART1 ISRs.
- **R3: asymmetric Bulk Read CRC stays correct without any priority dance.** The 128-byte-predecessor + 1-byte-our-reply case that breaks walk-everything-at-fire works cleanly because the bulk walk happens at `fire − 150 µs`, leaving only ~45 bytes of straggle for the fire ISR — well within even the smallest reply's prefetch slack.

What this doesn't buy:

- **The chip's ~5 µs structural floor on the SysTick-fire path itself is unchanged.** That floor is the irreducible PFIC trap entry + ISR body + DMA EN write + TX_EN GPIO toggle on V006 V2 — the time from CMP-match to bytes leaving the wire even with nothing else happening. It's what "on time" *means* on this chip. v6 just removes the extra ~4 µs we were paying on top of it from RXNE-queueing contention. Today fire happens at floor + 4 µs; under v6 it happens at floor + 0. Closing the floor itself needs the TIM1 OPM hardware-driven fire lever (§13 parent, item 3).
- **Total CRC walk work is unchanged.** We're rearranging when bytes get walked, not skipping any.

What this trades off:

- One extra `Stage` and one extra constant (`SWITCH_MARGIN_US`).
- The constant is a derived number with documented math, not a magic value. Re-derive if `DXL_RX_BUF_LEN` changes or the smallest-reply assumption changes.

## 9. Why this isn't "use DMA half-transfer interrupt"

The DMA controller has Half-Transfer (HT) and Transfer-Complete (TC) interrupts that fire at fixed ring positions (half and full). Tempting to use HT_IE on CH5 as a "free" CRC accumulation point.

Two problems:

1. **HT fires at a fixed ring position, not at a fixed offset from our snoop window.** Where `snoop_head` happens to land determines whether HT fires during our window at all, and if so, how far through. For a 200-byte snoop that doesn't cross the ring midpoint, HT never fires.
2. **HT would fire constantly during normal RX traffic too**, not just during our snoop window. We'd need stage-gating in the HT handler to avoid doing snoop work outside Fast last-slave windows.

SysTick CMP is deterministic, tied to *our* snoop window, and the FSM already has the infrastructure. `WaitingSwitch` reuses what's there; DMA HT would be a new IRQ source for strictly worse alignment.

## 10. One-paragraph summary

> The Fast Sync/Bulk Read last slave has to compute a CRC over predecessor slots' wire bytes and stitch it onto its own reply. Walking everything at fire blows past the V006's tight TX DMA prefetch slack as soon as the snoop crosses ~70 bytes for a small reply; firing RXNE per byte costs hundreds of USART1 IRQs at 3 Mbaud and shoves SysTick's fire deadline back by ~4 µs because USART1 and SysTick share PFIC priority. The fix is a second SysTick CMP deadline at `fire − 150 µs`, where the CPU peeks the RX DMA cursor (NDTR), bulk-walks the ring into a running CRC, and lets a ~45-byte straggle wait until the fire ISR. Two SysTick entries per Fast last-slave reply, zero RXNE entries. The 150 µs margin is derived from two constraints: the switch walk has to fit inside the margin (`M ≥ total / 10.3`, so 99 µs for our 1024-byte RX ring) and the straggle walk has to fit inside the TX prefetch slack (`M < (slack − patch) / 0.03`, so 188 µs for the smallest possible reply). The reply-scheduler FSM gains one state (`WaitingSwitch`), the RXNE composer collapses to a single owner (framing), and the 4 µs queueing penalty the user observed at 3 Mbaud disappears.

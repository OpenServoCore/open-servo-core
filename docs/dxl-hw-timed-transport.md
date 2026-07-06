# DXL 2.0 Hardware-Timed TX + Software-Timed RX on the CH32V006

A design note for the DXL transport on the V006. Read [dxl-rx-timing.md](dxl-rx-timing.md) and [dxl-streaming-rx.md](dxl-streaming-rx.md) first — this doc covers the timing subsystem: how a reply's wire start is placed on a hardware tick (TX side) and how the transport recovers the timing it needs to place it (RX side). The pieces that don't live here (USART1 framing, half-duplex muxing, the flat frame classifier, Fast Last CRC composition, baud/RDT decision rule) are covered in the docs above.

The transport is **asymmetric**: TX timing is hardware, RX timing is software.

- **TX side.** Every fire moves off the CPU. A TIM2 compare match drives a DMA write that enables the USART TX stream, with **zero CPU in the deadline path** (§5). TX_EN rides a second TIM2 compare. The fire floor collapses from the ISR era's ~1 µs + PFIC-jitter tail to the pure silicon compare→DMA→DR path.
- **RX side.** The transport needs one number per reply — the request's packet-end tick — plus a per-baud HSI drift estimate. Both come from **software readings of the free-running WireClock taken at drain-ISR entry**, corrected by drain flavor. There is no per-byte edge-timestamp hardware.

## The RX side was hardware once, and isn't anymore

An earlier design (the bulk of the git history behind this file) timestamped **every falling edge on the RX pin** through TIM2 input capture: PC1 dual-fed USART1_RX and TIM2_CH4, a DMA channel drained capture stamps into an "ET ring", and a `.highcode` walker turned edge times into per-byte "BT" start ticks. It worked. It was also the single largest CPU consumer in the transport — a per-edge classifier walk running at up to ~1.5 M edges/sec at 3M — and it existed to deliver a precision the protocol never spends.

Measurement killed it. The transport's timing consumers are: (a) the packet-end tick that schedules a reply at `packet_end + RDT`, (b) the FAST k > 0 status-start anchor, and (c) the HSI drift estimate. None of the three needs a per-byte tick:

- The reply deadline is `packet_end + RDT`. RDT is tens of µs; the host's read window around it is wider still. A packet-end estimate good to a few µs is invisible at that scale. The shipped software estimate lands plain-reply wire excess at **min +0.17 µs / median +5.4 µs at 3M** (§8.1) — the median tail is same-priority poll blocking, tracked separately, not a timing-source limit.
- FAST k > 0 slots anchor on the chain Status packet's **start**, which a byte-count estimate off the RX ring cursor resolves without any edge stamp (§8.2).
- HSI drift is a rate estimate over a burst — `Δticks / Δbytes` — and NDTR/byte-count spans measure that directly (§8.3). Per-byte resolution buys nothing a multi-byte span doesn't.

So the edge subsystem was deleted: no PC1 dual-remap, no ET/BT rings, no classifier, no LUT walker. TIM2_CH4 stays — but only as a **pin-less output compare** that fires the TX kickoff (§5). DMA1_CH7 stays — but only as the kickoff channel (§6). The result: the TX-side hardware timing (the part that actually needed to be jitter-free) survives intact, and the RX side reduces to a handful of WireClock reads. What follows describes the shipped design; where a section is a lightly-edited survivor of the hardware-RX draft it says so.

---

## 1. Why hardware-time TX and software-time RX

The two sides have opposite jitter economics, and the design follows the asymmetry.

**TX jitter propagates to the wire; software can't hide it.** A software fire is SysTick/TIM2 CMP → PFIC entry → ISR body → DMA EN + TX_EN toggle → first wire bit. At 3M one byte is 3.33 µs and the PFIC-entry floor alone is ~1 µs with a jitter tail over half a byte-time (measured +1.22 µs p99.9, +2.85 µs max in the software-CC3 era). That tail lands *directly* on the wire as a shifted start bit and, for FAST slots, as inter-slot contention. The fix is to take the CPU out of the deadline path entirely: a TIM2 compare match fires a one-transfer DMA write that enables the TX DMA (§5). The remaining spread is pure silicon (compare → DMA request → AHB → USART DR), an order below the ISR floor.

**RX jitter does *not* propagate to the wire.** The RX-side timing feeds `packet_end + RDT`. A few µs of error in `packet_end` shifts the reply by a few µs against a deadline that already has tens of µs of RDT and a much wider host read window around it. Nothing downstream is sensitive at the µs scale. So the RX side spends **no** hardware on jitter reduction: it reads the free-running WireClock at drain-ISR entry, corrects for the drain flavor, and is done. The CPU that the deleted edge classifier spent — up to ~55% during sustained 3M RX — goes back to the motor loop.

**Allocation principle: TIM2 is reserved for the TX fire path (CC4 OC kickoff, CC2 TX_EN OC), where jitter reaches the wire. SysTick covers long-horizon FAST scheduling, where ~5 µs PFIC entry is absorbed by an early-biased wake lead and never reaches the wire. RX timing is software, because its error never reaches the wire at all.**

---

## 2. The CH32V006 peripheral inventory for this design

**TIM2** — 16-bit general-purpose timer, free-running counter, four CC channels, DMA on UP and each of CC1..CC4. This design uses **CC4** (pin-less OC → DMA1_CH7 kickoff) and **CC2** (OC → PC2 TX_EN). CC1/CC3 unused.

**DMA1** — single-transfer per request, no FIFO, fixed source-to-channel mux. Channels in play: CH4 (USART1_TX), CH5 (USART1_RX circular), CH7 (TX kickoff). §6 enumerates the allocation.

**USART1** — HDSEL half-duplex single-wire on PC0 (TX) + PC1 (RX). Unchanged. RXNEIE is toggled on for the per-byte wake window (§8.3); otherwise RX runs DMA-only.

**SysTick** — 32-bit at HCLK. Drives the FAST successor pickup wake (§10.6) and the wall clock / WireClock for all RX timing. No wire-edge events.

There is **no AFIO dual-remap** in the shipped design. PC1 feeds USART1_RX only; TIM2_CH4 is a pin-less compare (CC4E=0) and never reads or drives a pad.

---

## 3. Pin assignments

No schematic changes.

| Pin | Function | How |
| --- | --- | --- |
| PC0 | USART1_TX (HDSEL — quiet, data flows on PC1) | — |
| PC1 | USART1_RX | USART1 remap |
| PC2 | TX_EN driven by TIM2_CH2 OC | TIM2 remap routes CH2 to PC2 |

TIM2_CH4 has **no pin** — it is an internal compare event only (§5). This is the whole difference from the hardware-RX draft, which dual-tapped PC1 into TIM2_CH4 input capture.

---

## 4. The RX side: software packet-end estimate

The RX side produces exactly one timing datum per reply-bound request — the **packet-end tick**, in the WireClock (SysTick u32) domain — plus a running HSI drift estimate. Both derive from the WireClock value read at the entry of whichever ISR drains the request's last byte, corrected by which drain fired. The details are §8; the shape is:

- The codec stashes `(now, PollSrc)` at every drain-ISR entry, where `now = WireClock::now()` and `PollSrc ∈ {ByteBatch, LineIdle}` names the drain flavor.
- When the framer completes a frame (CRC-verified) the codec resolves the stash into a packet-end tick via a per-flavor formula (§8.1) and hands it to the reply scheduler as `packet_end_tick`.
- The reply schedules at `packet_end_tick + RDT` (single-target / slot 0) through the TX kickoff (§5).

No IDLE-derived measurement enters the number: IDLE only selects *which* formula applies; the tick itself is a WireClock reading taken at ISR entry (`[[no_idle_timing]]`). The drift estimate rides the same drain stamps as `(Δticks, Δbytes)` spans (§8.3).

---

## 5. The TX side: hardware kickoff

*(Survives from the hardware-RX draft essentially intact — the TX fire never depended on the edge subsystem. Only the CC4DE latch discipline, §5.1, is newer, and the surrounding text is trimmed to the shipped shape.)*

### 5.1 Timer-triggered TX, hardware-kicked via DMA1_CH7

    TIM2 CNT counts up free-running.
    TIM2_CH4 is a permanent pin-less OC (OC4M=Frozen, CC4E=0): the compare
    EVENT fires the CC4DE DMA request without driving any pad.
    Arm time (tx_kickoff::arm): CCR4 = start_tick − TX_KICKOFF_FLOOR_TICKS;
              wipe CC4IF; (re)load DMA1_CH7 as a one-transfer MEM→PER kickoff
              (PAR = &DMA1_CH4.CR, MAR = &KICKOFF_WORD = CH4 TX config | EN,
              NDTR = 1); enable CH7; THEN enable CC4DE.
    When CNT == CCR4, CC4DE fires → CH7 writes EN=1 into DMA1_CH4.CR
    → CH4 fetches byte 0 → USART DR (TE is set permanently at init — §5.3).
    CH7's TC (~5 HCLK later) raises the DMA1_CH7 IRQ, whose body parks CH7
    disabled and drops CC4DE (post-deadline, non-critical).

Zero CPU in the deadline path. Two silicon behaviors the RM doesn't state verbatim were spike-verified (`osc-dev-v006-bringup` `tim2_ch4_oc_dma_kickoff`): an OC compare match with CC4E=0 / OC4M=Frozen still fires the CC4DE request, and DMA1_CH7 may legally target the DMA controller's own register block as a peripheral address.

**CC4DE latch discipline (#134).** A CC4 DMA request **latches in the V006 DMA controller and survives a channel disable** — a request pulsed into a *disabled* CH7 is delivered the instant CH7 re-enables (probe-verified: a boot-time stale match fired the kickoff word ~720 ticks early, streaming 2 bytes before TX_EN). Clearing CC4IF does not purge the latch. The gate is therefore **CC4DE itself**:

- Parked windows (`on_kickoff_complete` after TC, and the `cancel` path) hold **CC4DE off** — no request is ever generated to latch.
- `arm` enables CC4DE **last**, only after CCR4 points strictly ahead, CC4IF is wiped, and CH7 is live. A match that lands *inside* the arm window sets CC4IF but pulses no request; the set-and-recheck (§5.4) sees CNT past CCR4 and re-aims.
- The DMA1_CH7 TC restore body is **TC-flag-gated** so a stale PFIC pend can't disable a freshly-armed channel mid-window.

There is no RX edge channel to time-share against anymore. CH7 is kickoff-only; between sends it is simply parked disabled.

**Floor compensation.** `TX_KICKOFF_FLOOR_TICKS` (`firmware/ch32/src/measurements.rs`) back-dates CCR4 so first-bit lands at or just after `start_tick`. Bench-calibrated to **0** (2026-07-04): at K=0, CCR4 aims at the deadline itself and the silicon request→DR path (~20–30 ticks) supplies the positive TX_EN lead — median wire excess ≈ +0.6 µs, p0.1 ≈ +0.02 µs. K=8 pushed the p0.1 excess *negative* (−0.15 µs: first bits emerge before CCR2 raises TX_EN), which is harmless for `0xFF`-leading single-target replies but clips `0x00`/err-leading FAST slot emissions in the direction-switch window. `tool-tune-tx-start` measures both the wire-excess distribution and any missing-leading-FF `DroppedLeadingFf` events (direct evidence CCR2 raced first-bit).

### 5.2 Timer-fired TX_EN

PC2 is TIM2_CH2's alternate-function output. The OC mode sequence is an explicit state machine, not toggle mode.

| Phase | OC2M setting | PC2 state |
| --- | --- | --- |
| Idle (no reply armed) | Force inactive (`OC2M = 100`) | Idle level (low for active-high TX_EN) |
| Arm time (reply scheduled) | Write CCR2, then Active-on-match (`OC2M = 001`) | Still idle until match |
| CCR2 match (== fire_tick) | unchanged | Active level (TX_EN asserted, ~2 ticks later by OC pad lag) |
| USART1 TC (reply done) | Force inactive (`OC2M = 100`) | Back to idle level |

No GPIO write from software at fire time. The Force-inactive write at TC is not jitter-critical (the reply is already done shifting), so the TC ISR handles it.

### 5.3 Composing CCR2 and CCR4

- **CCR2 drives PC2 directly via hardware OC** — TX_EN asserts on match, pad lagging ~2 ticks.
- **CCR4's match fires the DMA kickoff** — CH7 writes the enable word `TX_KICKOFF_FLOOR_TICKS` before `start_tick`.

Composition:

    CCR4 = start_tick − TX_KICKOFF_FLOOR_TICKS   # K = 0 today
    CCR2 = start_tick                             # no offset

No T_setup offset on CCR2: the floor calibration is the single knob positioning first-bit relative to TX_EN. Keeping CCR2 = start_tick verbatim also keeps the boundary with the previous slave's TX_EN release clean.

**TE is set permanently at USART init.** HDSEL tristates the USART's own TX driver between shifts and the external buffer gates the bus via TX_EN, so leaving TE on doesn't drive the bus when idle. Per-send the deadline path only writes EN into DMA1_CH4.CR — and that write comes from DMA, not the CPU.

### 5.4 Wrap handling and the set-and-recheck pattern

TIM2's CNT is 16-bit; at PSC=0 it wraps every 1.365 ms (§7), while RDT can be ~24 k ticks. `start_tick = (packet_end + rdt) mod 65536` can sit either side of the next wrap. The compare fires on the first CNT == CCR4 after arm, even across a wrap — automatic.

The hazard is **armed in the past**: if CNT just passed CCR4 (parse overran into the send window, or packet-end fell close to the deadline), the *next* CNT == CCR4 is a full wrap away — 1.365 ms, far too late. Set-and-recheck at arm time guards it:

    ccr4_tick = (packet_end + rdt − TX_KICKOFF_FLOOR_TICKS) & 0xFFFF
    CCR2      = (packet_end + rdt) & 0xFFFF
    arm kickoff (CH7 reload + CCR4 = ccr4_tick + CC4DE enable, §5.1)

    remaining = (ccr4_tick − TIM2.CNT) & 0xFFFF
    if remaining > SCHEDULE_WRAP_GUARD_TICKS:
        # missed — a real compare event is the only trigger left:
        force CCMR2 OC2M = Force-active                     # drive PC2 high (CC2 is equally past)
        CCR4 = TIM2.CNT + KICKOFF_RETRY_LEAD_TICKS          # re-aim just ahead; fires ~1.3 µs later

`SCHEDULE_WRAP_GUARD_TICKS` is the largest legitimate start-in-the-future (RDT_max + slot-offset slack — long horizons are absorbed by the SysTick handoff, §10.6, so CCR4 only ever arms within RDT-bounded distance of CNT). The retry re-aims CCR4 rather than software-firing because **TIM2's `SWEVGR.CC4G` is dead silicon on V006** (spike-verified: the write never latches CC4IF) — a real compare event is the only thing that can trigger the armed kickoff.

---

## 6. DMA channel assignments

CH32V006 DMA1 has a fixed source-to-channel mux. Allocation:

| DMA1 channel | Role | Notes |
| --- | --- | --- |
| CH1 | ADC pump | unchanged |
| CH4 | USART1_TX (single-shot) | Enabled by CH7's kickoff write at the CC4 match; per-send NDTR staged at arm |
| CH5 | USART1_RX (circular) | RX byte ring; HT/TC enabled for byte-ring publish + framer drain (§9). `PL = LOW`. |
| CH7 | **TX kickoff only** | One-transfer MEM→PER (`&KICKOFF_WORD` → `&DMA1_CH4.CR`); TC IRQ parks it. `PL = VERYHIGH`. Parked disabled between sends. |

CH7 is no longer bimodal. In the hardware-RX draft it time-shared between an "ET ring" edge-capture role and the kickoff; the edge role is gone, so CH7 sits idle between sends and `arm`/`cancel`/`on_kickoff_complete` own its whole lifecycle (§5.1). ADC's pump is untouched — no channel moves. The RX DMA provider still exposes an intra-loop NDTR refresh (`rx_dma.rs`) so the codec sees the freshest published byte cursor on every drain and pickup spin.

---

## 7. TIM2 configuration

*(Survives the hardware-RX draft minus the input-capture filter — there is no capture anymore.)*

- **Free-running, no OPM, no slave-mode.** SMS = 0, OPM = 0. CNT counts forever.
- **CC4 = pin-less output compare** (OC4M = Frozen, CC4E = 0) feeding the DMA1_CH7 kickoff (§5). **CC2 = output compare** (TX_EN → PC2). CC1/CC3 unused. No CC4 IC↔OC swap, no cached IC filter to restore — CC4 is OC for its whole life.
- **Prescaler.** PSC = 0 — CNT at HCLK = 48 MHz, one tick = 20.83 ns, ARR = 0xFFFF, period ≈ 1.365 ms. PSC=0 gives the finest OC placement on the fire path; wrap is handled by set-and-recheck (§5.4).
- **SysTick/TIM2 low-16 alignment.** SysTick and TIM2.CNT both run at HCLK. They are co-zeroed at boot (`init_tim2_ch4_oc_kickoff` → `reset_cnt`, immediately before `CEN=1`) so the low 16 bits of the WireClock u32 equal TIM2.CNT. That lets the scheduler truncate a u32 WireClock deadline directly into a 16-bit CCR4/CCR2 compare. **Post-boot no code may reset CNT on either timer** or the mapping breaks silently.

---

## 8. RX timing sources

Everything the RX side produces — packet-end tick, FAST status-start, HSI drift — comes from WireClock reads at drain-ISR entry. No per-byte hardware.

### 8.1 Packet-end estimate (`codec/packet_end.rs`)

The codec stashes the most recent drain ISR's `(now, PollSrc)` and resolves it when the framer completes a frame (CRC-verified):

    packet_end_tick = drain_ref(now, src, ticks_per_bit) − PACKET_END_ENTRY_COMP_TICKS

with the drain-flavor reference:

- **`PollSrc::ByteBatch`** (DMA1_CH5 HT/TC landed on the CRC byte): `drain_ref = now`. The CRC byte hit DMA immediately before the poll, so `now` *is* packet-end with negligible ISR-entry offset.
- **`PollSrc::LineIdle`** (USART1 IDLE): `drain_ref = now − BITS_PER_FRAME · ticks_per_bit`. IDLE asserts one idle character after the last stop bit; the packet's wire end sits one frame before the idle latch, so a **single-frame** back-date centers the estimate in the window with symmetric slack against ns→tick truncation. (The full IDLE-to-stamp interval is two frames — the byte's own 10 bits plus the 10-bit idle threshold — but the reference we want is the wire end, one frame back.)

`PACKET_END_ENTRY_COMP_TICKS` is the chip's entry-latency compensation, **calibrated to 0** and pinned there by the flavor split: ByteBatch drains carry near-zero latency (min wire excess **+0.17 µs** at 3M), so any nonzero comp would fire ByteBatch-drained replies *before* the deadline. IDLE drains carry ~5 µs; the residual **median +5.4 µs** excess is that IDLE-flavor offset plus same-priority poll blocking — protocol-irrelevant at RDT scale. Per-flavor comps are the lever if that ever needs to change.

**Measured jitter (3M, hardware).** The two drain flavors' entry-jitter spreads (~1.05 µs and ~1.19 µs) are within measurement equivalence — the estimate's precision is drain-ISR-entry-bound, not source-bound. The median-vs-min gap is the **same-priority-blocking tail**: a drain ISR that lands behind another High-priority body (or a long critical section) enters late, shifting `now` — this is the poll-cost tail tracked separately (§12), not a floor of the estimator. Turnaround at rdt=0 improved **151 → 101 µs** against the ISR-fire baseline.

### 8.2 FAST k > 0 status-start (byte-count estimate)

A deferred FAST successor slot anchors on the observed start of the chain's single Status packet, then fires `bytes_before(k) · byte_ticks` after it (no RDT, no IDLE floor — see [dxl-streaming-rx.md §5.6](dxl-streaming-rx.md)). The start tick is resolved from the **RX ring byte cursor** at the per-byte RXNE wake, not from an edge stamp: the wake that first sees a byte past the Status packet's wire cursor reads `WireClock::now()` and back-projects to the packet's first byte by the published byte count. One physical reading anchors the whole chain-wide grid, so contiguity holds at any baud including RDT = 0. Bench: FAST 50/50 clean at 3M, chain CRC 0/50.

### 8.3 HSI drift: NDTR spans + the RXNE window

The slave's HCLK is HSI (internal RC); the host's baud is HSE (crystal). The transport measures the ratio and trims HSITRIM + a sub-step fire-deadline residual (the integrator and feedback are unchanged — see §10.7 in this doc's history / [dxl-streaming-rx.md §5.4](dxl-streaming-rx.md)). What changed is the **signal source**: instead of BT-ring byte-pair intervals, the estimate is `(Δticks, Δbytes)` spans measured off drain stamps.

**Natural spans (`codec/span.rs`, `SpanTracker`).** Each drain-ISR stamp records `(now, published_cursor, flavor)`. A span emits only when the new stamp pairs with the previous over one contiguous **same-flavor** burst of **Instruction** bytes:

- *Same-flavor only* — a mixed HT/TC↔IDLE pair carries the differential ISR-prologue latency as a systematic offset, so it's rejected.
- *Instruction-only* — foreign Status bytes are another servo's HSI-clocked TX and must never contribute (`[[drift_sampling_instruction_only]]`).
- *Same-burst gate* — a span whose tick length deviates from `Δbytes · byte_ticks` by more than `expected / 16` is rejected; host inter-packet gaps blow this by orders of magnitude, while the largest real trim offset (~2%) passes wide.
- The stamp is already back-dated to the last-byte reference for its flavor (IDLE subtracts one frame upstream) so both flavors share a reference.

**RXNE window (`codec/span.rs` `DriftWindow` + `drift_window.rs` `RxWakeGate`).** An isolated short packet (a Ping, a short Read) trips only **one** drain ISR and so forms no `SpanTracker` pair — it would never sample. The RXNE window restores it: while open, USART1 RXNEIE wakes per byte; the driver records the burst's **first** wake `(now, cursor)` and overwrites the **last**, and at the packet boundary `settle` emits a **single long span** from that pair. One span per burst by design — pairing consecutive per-byte wakes would hand the integrator 1-byte spans whose ±quantization (±6250 ppm at 3M) dwarfs the ~2500 ppm trim step and biases the batch minimum. The accumulated span is gated on `DRIFT_WINDOW_MIN_BYTES` (8: brings quantization to ≤781 ppm) and the same-burst rule before it reaches the integrator.

**Window lifecycle (`RxWakeGate`).** RXNEIE is one hardware bit with two independent consumers — the FAST k > 0 wait and the drift window — so the gate holds a two-bool reason set and toggles the provider only on the OR-edge (a FAST resolution's `unwatch` never closes a window the drift sampler still needs, and vice-versa). Rules:

- **Open** at cold boot, an applied baud change, or a staleness reopen.
- **Batch-close predicate.** The window closes when the integrator **closes a batch, regardless of outcome** — a below-deadband close is a success (drift ≈ 0 learned), and the per-byte IRQs must never outlive the sampling they exist for (a perfectly-trimmed chip would otherwise hold ~300k IRQs/sec at 3M forever). Batch closes are caught both at the poll boundary (`service`) and inside a drain-ISR handler (`on_batch_closed`).
- **Give-up.** An open window that sees `DRIFT_WINDOW_MAX_PACKETS` (4) instruction packets without one qualifying span closes — all-short or all-foreign-Status bursts can't feed the sampler and shouldn't hold RXNEIE on.
- **Staleness reopen.** A closed window reopens after `DRIFT_STALENESS_INSTRUCTIONS` (64) instructions with no accepted span (natural or window) — a bus of only pings/short reads would otherwise let temperature drift go untracked once the boot batch ended the window.

Cold convergence is one packet: `tool-hsi-conv` converges from cold within a single ping's boot batch (a 6-sample batch with a full register-range emit cap lands the ±2% factory drift; steady 20-sample batches with a half-step deadband squeeze the residual).

---

## 9. ISRs and priorities

Two priority levels (V006 PFIC has nothing more). All DXL-side IRQs at High; only ADC at Low.

| Priority | IRQ | Body | Where |
| --- | --- | --- | --- |
| High | USART1 | IDLE (framer kick) + TC (release bus, apply pending) + RX errors + per-byte RXNE wake (FAST status-start + drift window, §8.2/§8.3) | flash |
| High | DMA1_CH5 HT/TC | RX byte-ring publish + framer drain ([dxl-streaming-rx.md](dxl-streaming-rx.md) §3) | flash |
| High | DMA1_CH7 TC | Kickoff restore only (once per send): park CH7 disabled, drop CC4DE. Post-deadline. | flash |
| High | SysTick CMP | FAST successor pickup: ring drains for long windows, then the wake body that reads the checkpoint and lands `patch_crc` (§10.6). | flash entry; pickup body in `.highcode` |
| Low | DMA1_CH1 | ADC kernel pump | flash |

No vector sits on the wire deadline — the TX start is the CC4→CH7 hardware kickoff (§5). Every ISR lives in flash (`lowcode`); per-drain and per-packet-end slack absorbs flash-fetch jitter (the FAST pickup body is the one `.highcode` exception, §10.6).

**DMA1_CH5 HT/TC publishes the byte ring and drives the framer drain.** Pinning publish to byte-ring HT/TC bounds `write_seq` lag to `RX_BUF_LEN/2` regardless of traffic content — load-bearing under long byte-skip windows where the universal byte-skip consumes payload faster than any other event would fire. CH5 stays `PL = LOW`.

**Why all DXL-side IRQs share High.**

1. **Driver-state exclusivity.** Every vector that reaches the DXL driver's `&mut` state shares High — no preemption between them means no concurrent borrow, zero locking.
2. **TC release before the next start bit.** TX_EN drop in TC must land before any next request's start edge; TC at High keeps that bounded.
3. **Pickup wake latency.** The FAST wake is early-biased (§10.6); SysTick CMP at High keeps Low work from stretching the worst-case entry the lead is sized for.
4. **Kickoff restore vs arm/cancel.** The DMA1_CH7 TC body reconfigures the same channel the scheduler's arm/cancel paths touch; sharing High serializes them.

**PFIC preemption facts.** The V006 PFIC is configured for **2-level nesting** (verified). The kernel/control loop runs at **Low** and *cannot add DXL jitter*: a Low body is preempted by any High vector, so it never delays a wire-side ISR. The transport's jitter budget is therefore **the worst-case High-priority ISR body plus the longest critical section** any High body takes — nothing at Low enters it. This is why the packet-end median-vs-min tail (§8.1) is a same-*priority* blocking effect (one High body behind another), not a priority-inversion one.

**TC tail budget.** New per-baud values (BRR, fine-trim) are *computed at parse time*; the TC tail does only the atomic register writes — bounded under 1 µs.

**SysTick CMP drives the FAST successor pickup wake** (§10.6); the wall-clock/WireClock side stays free-running for RX timing and telemetry. **TX_EN is hardware-driven** on CCR2 → PC2 (§5.2); **the TX start is hardware-driven** on CCR4 → CH7 (§5.1) — no ISR runs at the deadline for any reply kind.

---

## 10. Fast successor chain-CRC checkpoint pickup

*(§10.1–§10.5 described the deleted window classifier and §10.7 its BT-ring drift loop; both are gone — the software replacements are §8. The checkpoint pickup keeps its **§10.6** numbering because the TX-side code cross-references it.)*

### 10.6 The pickup

Under the official Fast Sync/Bulk Read layout every slot's block ends with the **cumulative packet CRC** — the running chain state, checkpointed on the wire after each device (e-manual: "CRC values are used for internal calculation in DYNAMIXEL to confirm packet integrity between DYNAMIXELs"). A successor slot (k > 0) therefore never folds the predecessor window: the window's last two wire bytes ARE the chain state. The pipeline reads that checkpoint, extends the state over the checkpoint's own two bytes and the chip's reply bytes (`err, id, data`), and patches the reply's trailing CRC slot before DMA1_CH4's read cursor reaches it. **O(own reply) CPU per exchange, independent of chain length** — the wire hands each device the baton; nobody re-runs the race.

State + trigger:

- **State** lives composite-side: the window's start cursor and byte budget (`FoldEngine`). Finalize seeds `u16::from_le_bytes(checkpoint)`, extends via the monomorphic table step (`dxl_protocol::crc16_umts_continue`), and patches; idempotent and a no-op when not armed.
- **Trigger** is a single SysTick CMP **wake** just before the checkpoint lands, plus O(1) ring-drain CMPs for long windows. Same path at every baud.

RAM placement of the wake body is load-bearing and **verified in the ELF, not assumed**: one flash-resident hop in the per-iteration chain costs more than the whole patch margin. The hot path is `#[link_section = ".highcode"]` end to end, and the pickup body is a *named* function rather than closure logic — a closure is its own unsectioned symbol, and when sectioned callees inline into it the whole body silently lands back in flash (bench-measured as exactly the difference between 50/50 CRC failures and 0/50 at 3M).

#### 10.6.1 Why SysTick, not TIM2

An earlier draft put the trigger on a TIM2 CC channel to keep the transport on one timer. It fails below ≈1 Mbaud: a predecessor window at 9600 baud spans many TIM2 wraps (16-bit at PSC=0 wraps every 1.365 ms), while SysTick is 32-bit (89.5 s horizon) and the math just works. SysTick's ~5 µs PFIC entry doesn't reach the wire — the wire start is hardware-armed, and the wake is early-biased by `FAST_CRC_WAKE_LEAD_TICKS` so a late vector entry costs spin time, never patch margin. TIM2's budget stays on the fire path (§5), where jitter actually reaches the wire.

#### 10.6.2 Wake path (SysTick)

**Defer time** (`send_slot` for a successor slot): the reply is encoded (placeholder CRC) but nothing is armed. The reply gate parks a status-start wait and the RxDma provider opens the per-byte RXNE wake; `poll()` gates on the parked wait so the framer leaves the Status packet's bytes to the pipeline.

**Arm time** (status-start wake — the Status packet's first byte observed):

    status_start_tick = byte-count estimate of the Status packet's first byte (§8.2)
    window_end        = status_start_tick + bytes_before × byte_time   # fire deadline, no RDT
    schedule(window_end, SendKind::FastLast); unwatch RXNE
        # within the direct-arm horizon: hardware kickoff arms NOW (§5.1) —
        # the wire start is locked in regardless of CPU state from here.
        # beyond the horizon (low baud, long predecessor): stash, committed
        # by the first wake/drain body via commit_pending().
    arm pickup state (start_cursor, bytes_before)
    set_busy_wait_deadline(window_end + byte_time)                     # checkpoint-late bound
    if bytes_before > drain_stride: SysTick CMP = status_start + stride  # ring drains
    else:                           SysTick CMP = window_end − WAKE_LEAD

**Ring drains (long windows only).** The RX ring publishes its producer head from an NDTR readback *modulo the ring depth*, so `on_publish` must run at least once per ring period or the head aliases. Windows longer than half the ring get intermediate CMPs whose body is one O(1) pickup attempt — publish + consume-to-cap by pure cursor arithmetic, no CRC work, no per-byte walk.

**The wake body** (final CMP, or entered inline from the observation when the window is within `FAST_LAST_INLINE_FOLD_HORIZON_TICKS` — a short window can't afford the CMP dispatch):

    commit_pending()                     # far-horizon stash → no-op if armed at observation
    loop:
        if poll_checkpoint():            # publish + cursor consume; Some(cp) once the window is in
            finalize(cp); patch_crc; done
        if patch_window_expired(): done  # CH4 read cursor reached the CRC slot → CrcPatchDeadlineMiss
        if deadline_passed(): break      # checkpoint late (silent predecessor)
    SysTick CMP += byte_time             # starve retry until the TX drain closes the window

The wake is deliberately **early-biased**: waking early costs a short spin (a pickup iteration is a fraction of a byte-time); waking late eats the patch-vs-fetch margin. A silent predecessor exits via the deadline arm to byte-stride retries and converges at `patch_window_expired` — the hardware kickoff fired regardless, so the TX drain always closes the loop, recording one `crc_patch_deadline_miss` and shipping the placeholder.

**Late observation** (wake resolves after the slot deadline — short predecessor at high baud): every window byte is already in the ring, so the inline body's first pickup succeeds immediately; `on_status_start` runs it BEFORE `schedule`, so a valid chain CRC ships on a slightly-late fire (~2 µs slip) rather than a placeholder on time.

**Patch-vs-fetch budget.** `patch_crc` must land in `tx_buf[len−2..len]` before DMA1_CH4's read cursor reaches them. The checkpoint publishes one DMA-write after the window's last stop bit, the succeeding pickup iteration reads + finalizes in well under a byte-time (RAM-resident, table-step CRC), and the patch lands with ~a quarter byte-time to spare at 3M (bench: 50/50 valid chain CRCs at the 1-byte-DUT corner, the previous design's 50/50 failure case).

#### 10.6.3 Pipeline ownership of the RX tail

The parked successor wait and then the armed pickup own the RX ring tail from `send_slot` until finalize or TC ([dxl-streaming-rx.md](dxl-streaming-rx.md) §6): `poll()` gates on both, so only pickup/drain bodies consume Status-packet bytes. DMA1_CH5 HT/TC stays live throughout for byte-ring publish; its framer drain is a no-op behind the poll gate. **There is no edge channel to mask** — the fold's RX-tail ownership is enforced by the poll gates alone. (In the hardware-RX draft this section masked the edge channel's HT/TC; that machinery is deleted.)

#### 10.6.4 CPU cost

Per exchange, any chain length: one wake body (a short spin + ~a dozen table steps + two byte writes) plus, for windows deeper than half the RX ring, one O(1) drain body per half-ring of predecessor bytes. The old per-byte fold — ~54% CPU for the whole reception window, scaling with packet length across every servo in the chain — is deleted.

#### 10.6.5 Plain replies

Plain replies skip the pickup arm and the SysTick wake entirely. Every pickup entry point is a no-op when not armed, so the HT/TC + IDLE callbacks stay safe to invoke unconditionally.

---

## 11. Retired designs

Two subsystems that this doc once specified are deleted; they are named here so a reader tracing the git history knows they were considered and dropped, not lost:

- **Per-byte edge timestamping** (TIM2 CH4 input capture → ET ring → window classifier → BT ring). Delivered per-byte wire times at up to ~55% CPU during sustained 3M RX. Deleted because no timing consumer needs per-byte resolution (see the intro): the packet-end estimate (§8.1), byte-count status-start (§8.2), and NDTR-span drift (§8.3) each meet their consumer's tolerance from cheap WireClock reads.
- **LUT walker with resync detector.** A faster (~5×) alternative to the window classifier that predicted per-byte edge structure from byte data; rejected then for silent-desync risk, moot now that the whole edge subsystem is gone.

---

## 12. Open questions

- **Poll-cost median tail (§8.1).** Plain-reply wire excess is min +0.17 µs but median +5.4 µs at 3M; the gap is same-priority poll blocking — a drain ISR entering behind another High body / long critical section. Tracked as a poll-cost optimization separate from the timing source. Bench the worst-case blocker and shorten it; the estimator itself is not the limit.
- **Min-statistic bias caveat (§8.1).** The batch **minimum** wire excess is the calibration anchor for `PACKET_END_ENTRY_COMP_TICKS = 0`. Under two-sided ISR-entry jitter a batch minimum biases *slightly negative*, yet bench shows a stable **~half-step positive residual** — watched, not acted on. If it ever needs correcting, the lever is swapping the calibration statistic from **min to median**, not a per-flavor comp.
- **FAST only/first slot lateness.** RESOLVED for k > 0: after the flat-frame RX landing, middle and last slots measure −0.29 µs median (σ 0.95 / 0.125 µs) at 3M — the former ~+7 µs middle-slot shift is gone. What remains late is the *only/first* positions (+3.6 µs, σ ~3 µs): the slot-0 shape schedules like a single-target reply (`packet_end + RDT`), so it shares the kickoff-floor retune tracked with the K item above, not the chain grid.
- **`TX_KICKOFF_FLOOR_TICKS` across baud (§5.1).** K = 0 is calibrated at 3M. Re-check at 1M/4M that the silicon request→DR path still supplies positive TX_EN lead without clipping FAST `0x00`-leading emissions.
- **`SCHEDULE_WRAP_GUARD_TICKS` measurement (§5.4).** Conservative 0x8000 today; measure the true `RDT_max + slot_offset_max` bound.
- **`FAST_CRC_WAKE_LEAD_TICKS` down-tune (§10.6).** 500 ticks covers worst-case SysTick entry with margin; bench-tune downward once pickup timing is characterized (waking early costs spin, waking late costs patch margin — size to the WORST entry, not the typical).
- **Drift staleness/give-up bounds (§8.3).** `DRIFT_WINDOW_MAX_PACKETS = 4` and `DRIFT_STALENESS_INSTRUCTIONS = 64` are reasoned, not swept. Confirm on a real traffic mix that temperature drift stays tracked without leaving RXNEIE on wastefully.
- **Validated-write cost — RESOLVED via rules-as-code.** Final Write poll-body ledger at 3M: 4 B goal_position (two cross-register compares) **17.3 µs** (was 45 at band close), 16 B unruled 10.6 µs (was 16.8), 1 B locked+hooked 7.8 µs (was 9.4). The journey settled two things worth keeping. (1) *Interpretation, not placement, was the cost*: the old `Rule::eval` compiled to a 468-instruction zero-call interpreter at ~10 µs per evaluated rule, and every placement lever measured flat — RAM-placing the read leaf (no change), the whole eval (~8 µs/eval, −1.2 K stack, rejected), case-split reads (flat), rule tables in `.data` (−0.9 µs for 384 B, reverted). The fix that worked: the derives emit each validation as straight-line monomorphic compares (`Block::ct_check(view, lo, hi, base)`, chained by the section, called as one fn pointer per overlapping section), deleting `Rule`/`RuleKind`/`Rhs`/`CmpOp` and the interpreter outright — +0.9 K flash, zero RAM, ~3.5 µs per compiled rule. Section-level block-range gating replaces the interpreter's sorted-scan early exit so miss-heavy small writes stay fast. (2) *RV32E codegen traps*: runtime u64 shifts are `__ashldi3` libcalls (edge masks build in u32 with an explicit `== 32` branch), and hot leaf fns inlined into cold scan loops bloat flash-resident loop bodies — measure before inlining.

---

## 13. One-paragraph summary

> The DXL transport is timing-asymmetric: **TX is hardware-timed, RX is software-timed**, because TX jitter reaches the wire and RX jitter does not. The TX start is pure hardware — TIM2_CH4 is a permanent pin-less output compare whose match fires the CC4DE DMA request; DMA1_CH7 (kickoff-only, parked disabled between sends) writes the CH4 enable word into `DMA1_CH4.CR`, streaming the reply with zero CPU in the deadline path. CC4DE is the latch gate: a V006 CC4 DMA request survives a channel disable, so parked windows hold CC4DE off and `arm` enables it last, after CCR4 points ahead and CH7 is live (#134, probe-verified). TX_EN rides TIM2_CH2 OC on PC2; CCR2 = start_tick, CCR4 = start_tick − `TX_KICKOFF_FLOOR_TICKS` (calibrated to 0). Set-and-recheck on CCR4 catches "armed in the past" against TIM2's 1.365 ms wrap (retry re-aims CCR4 — `SWEVGR.CC4G` is dead silicon). The RX side produces one number per reply, the packet-end tick, as a drain-flavor-corrected WireClock read at drain-ISR entry: `now` for a byte-batch drain, `now − one frame` for an IDLE drain, `PACKET_END_ENTRY_COMP_TICKS` (0) subtracted — no IDLE-derived measurement, IDLE only selects the formula. Plain-reply wire excess lands min +0.17 / median +5.4 µs at 3M (the tail is same-priority poll blocking, tracked separately); turnaround at RDT = 0 improved 151 → 101 µs. FAST k > 0 slots anchor on a byte-count status-start estimate off the RX ring cursor at a per-byte RXNE wake (50/50 clean, chain CRC 0/50). HSI drift samples `(Δticks, Δbytes)` NDTR/byte-count spans — same-flavor, instruction-only, one-span-per-window, gated on an 8-byte floor and a `±expected/16` same-burst rule — with an RXNE window restoring sampling for isolated short packets (batch-close predicate ends it, staleness reopens it); cold convergence is one ping's boot batch. All DXL-side ISRs share PFIC High; the kernel at Low cannot add DXL jitter, so the jitter budget is the worst High body plus the longest critical section. In FAST mode a SysTick CMP wake reads the predecessor's on-wire cumulative-CRC checkpoint and patches the reply's trailing CRC ahead of the TX DMA's read — O(own reply), no per-byte fold, the RX-tail ownership enforced by the poll gates alone now that no edge channel exists. **Allocation principle: TIM2 for the TX fire path where jitter reaches the wire; SysTick for long-horizon FAST scheduling where PFIC entry is dwarfed by body cost; software for RX timing, whose error never reaches the wire.** The per-byte edge-timestamp subsystem (input capture, ET/BT rings, window classifier, LUT walker) that earlier drafts specified is deleted — measurement showed the protocol never needed its precision and its servicing dominated the transport's CPU.

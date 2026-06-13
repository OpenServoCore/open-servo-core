# DXL 2.0 Hardware-Timed Transport on the CH32V006

A design note for the DXL transport on the V006. Read [dxl-rx-timing.md](dxl-rx-timing.md) first if you haven't — this doc replaces the ISR-driven fire path it describes with a hardware-timed one, and adds per-byte RX timestamping built on TIM2 input capture. The pieces that didn't change (USART1 framing, half-duplex muxing, Fast Last CRC, baud/RDT decision rule) still apply.

The headline change: every fire and every received-byte timestamp move into TIM2. SysTick stops driving wire-edge events. The ~5 µs PFIC-entry floor on TX fire goes away. RX timestamps become per-byte instead of per-packet, which closes the door on a whole class of low-baud / Fast-Last edge cases that today rely on backdating estimates.

The cost is that TIM2 is now the DXL transport's timer, and is no longer available for motor encoder counting. On the production board that's fine — there's no encoder. On bench builds it means the encoder bringup will need to find a different home (TIM3 stripped variant won't cut it; see §3 in the existing rx-timing doc).

---

## 1. Why move off SysTick

Three structural problems with the SysTick-driven design.

**TX fire has a ~5 µs PFIC floor.** SysTick CMP → PFIC entry → dispatched ISR body → DMA EN + TX_EN GPIO toggle → first wire bit. At 3 Mbaud one byte is 3.33 µs; the floor exceeds the inter-slot cap, so Fast-Last replies show a visible idle gap before their slot.

**Every timing decision goes through IDLE.** IDLE fires one char-time after wire-end, so wire-end is always backdated. At low baud the publish latency exceeds RDT, forcing the framing FSM to switch to per-byte RXNE timestamps and burning ~100k IRQs/sec at 1 Mbaud. The framing FSM, decision rule, pipeline_margin knob, and char-time backdate constant all exist because IDLE is the only timing source.

**RX timestamps are per-packet, not per-byte.** Snoop tail walks, chain-CRC stages, slot timing all want "when did *this specific byte* arrive?" and only get "when did the packet end." At 3 Mbaud the resolution loss matters.

The hardware-timed design fixes all three at once: TX fire moves to a TIM2 compare with a lean ISR; PC1 dual-feeds USART1_RX and TIM2_CH4 IC so every falling edge captures into an ET ring via DMA; a classifier turns the edge ring into a per-byte BT ring at HT/TC of the IC DMA. All timing consumers read `BT[i] + 10·bit_time` directly — no backdate, no char-time math, no IDLE-vs-RXNE branch. IDLE becomes a signal (drain parser, packet done), never a timing source. The expensive part — turning edge timestamps into byte timestamps — runs off the wire-edge critical path.

---

## 2. The CH32V006 peripheral inventory for this design

**TIM2** — 16-bit general-purpose timer, free-running counter, four CC channels, DMA on UP and each of CC1..CC4. Its DMA routing (TIM2_CH4 → DMA1_CH7, TIM2_UP → DMA1_CH2, TIM2_CH3 → DMA1_CH1) is what makes the rest of this plan possible.

**AFIO peripheral-side remap.** V006 supports the F1-style trick: a single GPIO pin can simultaneously feed multiple on-chip peripherals. PC1 with `USART1_REMAP = 0b0011` connects to USART1_RX; the same PC1 with `TIM2_REMAP = 0b010` connects to TIM2_CH4. Both routes are alternate-input — they read the pad without driving it — so they coexist.

**DMA1** — single-transfer per request, no FIFO, fixed source-to-channel mux. Channel sharing is the tightest constraint in this plan; §6 enumerates the allocation.

**USART1** — HDSEL half-duplex single-wire on PC0 (TX) + PC1 (RX). Unchanged.

**SysTick** — still 32-bit at HCLK, but no longer drives the wire. Reduced to a free-running wall clock for telemetry.

---

## 3. Pin assignments

No schematic changes — we're re-routing internal MUXes.

| Pin | Function | How |
| --- | --- | --- |
| PC0 | USART1_TX (HDSEL — quiet, data flows on PC1) | — |
| PC1 | USART1_RX **and** TIM2_CH4 input capture | F1-style dual remap (§2) |
| PC2 | TX_EN driven by TIM2_CH2 OC | TIM2 remap routes CH2 to PC2 |

PC1 dual-tap is the load-bearing trick. The rest is conventional timer wiring.

---

## 4. Peripheral plan: the RX side

### 4.1 Hardware timestamping

TIM2 runs free at HCLK. PC1's falling edges feed TIM2_CH4 in input-capture mode; each capture writes CCR4 to the ET ring via DMA1_CH7 (circular, HT/TC armed).

    PC1 ─┬─→ USART1_RX (DMA1_CH5 → RX ring)
         └─→ TIM2_CH4 IC ─→ CCR4 ─→ DMA1_CH7 ─→ ET ring

- **Captures are independent of slave-mode.** CCxIF latches on every selected edge regardless of gating. TIM2 runs free, no slave-mode, no OPM.
- **No per-byte ISR.** DMA writes the timestamp; the CPU wakes only at HT (ring half) and TC (ring full).
- **ET overrun is detectable** via NDTR-vs-consumer comparison in the HT/TC handler — a counted fault, not silent corruption.

### 4.2 Why timestamp falling edges, not bytes

The USART exposes no "byte arrived" event a timer can capture (RXNE is a flag, not a trigger). What we *can* capture is every falling edge on PC1.

UART frame: start (0) + 8 data LSB-first + stop (1):

    bit position:  S  0  1  2  3  4  5  6  7  Stop
    value:         0  d0 d1 d2 d3 d4 d5 d6 d7  1

Edges-per-byte ranges from 1 (data all 1s) to 5 (start bit + four 1→0 data transitions, e.g. 0x55). Edge timing is bit-level but not byte-aligned. Turning edge timestamps into byte timestamps is what §10 and §11 are about.

### 4.3 What the consumer wants

Given RX ring index `i`, what was TIM2 CNT at the *start bit* of byte `i`? With that:

- Wire-end of any byte: `BT[i] + 10 × bit_time`.
- Snoop CRC can timestamp each byte for jitter analysis.
- Fast-Last slot fires at `BT[i_last_master] + 10·bit + RDT + slot_offset` with no backdate.
- The framing FSM's IDLE/RXNE split disappears — per-byte timestamps at every baud.

This is the BT ring: byte-time, sized to match RX, indexed parallel.

---

## 5. Peripheral plan: the TX side

### 5.1 Timer-triggered TX, software-fired with bias

    TIM2 CNT counts up free-running.
    fire_tick is written to CCR3, biased by the MIN PFIC + ISR + USART pipeline latency.
    When CNT == CCR3, CC3IF latches → TIM2 IRQ fires.
    .highcode ISR: enable DMA CH4 (TE is set permanently at init — see §5.3).
    For Plain replies the body ends there. For Fast Last replies it tails
    with the post-fire residue fold + trailing-CRC `patch_crc` — see §10.6.

The fire trigger moves off SysTick onto a TIM2 compare event, but the actual register write stays in software. The fire-side critical action — `dma::enable(CH4)` — runs first, so wire-bit landing is unaffected by anything after it. For Plain replies the ISR has nothing else to do; body lands at ~0.3 µs MIN. Total fire floor: ~0.7 µs PFIC + ~0.3 µs body ≈ 1 µs MIN, comfortably under the 3.33 µs Fast-Last inter-slot cap at 3 Mbaud. Fast Last replies extend the body's tail (busy-wait for residue + fold + patch) but the wire bit has already left; tail timing is bounded by DMA1_CH4's prefetch slack on `tx_buf[len-2..len]`, not the wire fire. Not as good as a hardware-DMA fire (~100 ns) but enough to close the gap, and ADC stays on its existing DMA pump.

**Bias compensation.** Measure CCR3-match → first-wire-bit latency on hardware and store its **1st-percentile lower bound** as `FIRE_BIAS_TICKS`. CCR3 is then armed at `fire_tick − FIRE_BIAS_TICKS`. MIN calibration (not mean) guarantees the wire bit always lands at or after `fire_tick`: PFIC variance and USART bit-clock alignment only push it later. The tradeoff is ~15 ticks (~312 ns at 3M) of mean lateness on the wire in exchange for a one-sided bound that lets §5.3 drop T_setup compensation entirely.

Jitter comes from same-priority IRQs running when CC3IF latches. In practice neither USART1 IDLE nor TC are active at fire-time (IDLE finished by `wire_end + RDT`; TC hasn't fired yet for the reply we're about to start). The fire IRQ's worst-case contention is the classifier.

### 5.2 Timer-fired TX_EN

PC2 is TIM2_CH2's alternate-function output. The OC mode sequence is an explicit state machine, not toggle mode — toggle flips on every match without a defined start state, which would invert TX_EN if we miscount edges across a fire-cancel.

| Phase | OC2M setting | PC2 state |
| --- | --- | --- |
| Idle (no reply armed) | Force inactive (`OC2M = 100`) | Idle level (low for active-high TX_EN) |
| Arm time (parser scheduled reply) | Write CCR2, then set Active-on-match (`OC2M = 001`) | Still idle until match |
| CCR2 match (== fire_tick) | unchanged | Active level (TX_EN asserted, ~2 ticks later by OC pad lag) |
| USART1 TC (reply done) | Force inactive (`OC2M = 100`) | Back to idle level |

    TIM2 CNT == CCR2 → CH2 transitions to active level → PC2 rises → bus driver turns on.

No GPIO write from software at fire time. No ISR in the wire-edge path. The transition lands on a hardware tick. The Force-inactive write at TC is not jitter-critical (the reply is already done shifting), so the TC ISR handles it. CCER's CC2P bit sets polarity at init.

### 5.3 Composing CCR2 and CCR3

The two channels have different roles:

- **CCR2 drives PC2 directly via hardware OC** — TX_EN asserts on hardware match. The pad lags CCR2 by ~2 ticks (CCxIF latch + OC mux + pad synchronizer); the external bus buffer (`SN74LVC2G241` tPZH ≤4.7 ns at 3.3 V) adds <1 tick on top.
- **CCR3 raises CC3IF to trigger the fire ISR** — the lean ISR body enables DMA CH4 `FIRE_BIAS_TICKS` after CCR3 match (§5.1). Bias is sized to the *minimum* observed path, not the mean.

Composition:

    CCR3 = fire_tick - FIRE_BIAS_TICKS       # ~46 ticks ≈ 960 ns at 3M
    CCR2 = fire_tick                          # no offset

No T_setup offset on CCR2. The ~2-tick OC pad latency means PC2 rises ~40 ns after `fire_tick`. Against the 333 ns bit-time at 3M, the buffer enables during the leading ~12% of the start bit; the bus pull-up (R17 = 10 kΩ on the schematic) holds DATA high during the brief hi-Z window; the receiver samples at mid-bit and reads the driven LOW. Invisible on the wire.

**Why no T_setup compensation?** MIN-calibrated `FIRE_BIAS_TICKS` guarantees the wire bit never appears before `fire_tick` — jitter only pushes it later, so there's no "wire bit early, TX_EN late" race. Shifting CCR2 earlier instead risks bus contention with the previous slave still releasing TX_EN; the asymmetric cost makes "late not early" the safe direction.

**TE is set permanently at USART init.** The half-duplex (HDSEL) USART tristates its own TX driver between shifts, and the external buffer gates the bus via TX_EN regardless of TE state — leaving TE on doesn't drive the bus when idle. Per-fire the ISR only enables DMA CH4.

### 5.4 Wrap handling and the set-and-recheck pattern

TIM2's CNT is 16-bit. At PSC=0 it wraps every 1.365 ms (§7); RDT can be up to 502 µs ≈ 24 k ticks. `fire_tick = (wire_end_cnt + rdt_ticks) mod 65536` can sit on either side of the next wrap from `wire_end_cnt`. The compare logic raises CC3IF on the first CNT == CCR3 after arm, even across a wrap — that part is automatic.

The hazard is "armed in the past": if CNT *just passed* CCR3 (because parse overran into the fire window, or wire-end fell close to the deadline), the **next** CNT == CCR3 is one full wrap away — 1.365 ms. Far too late.

Set-and-recheck at arm time guards against it:

    ccr3_tick = (wire_end + rdt_ticks - FIRE_BIAS_TICKS) & 0xFFFF
    TIM2.CCR3 = ccr3_tick
    TIM2.CCR2 = (wire_end + rdt_ticks) & 0xFFFF
    enable CC3IE

    cnt_now   = TIM2.CNT
    remaining = (ccr3_tick - cnt_now) & 0xFFFF
    if remaining > MAX_REASONABLE_REMAINING:
        # we missed — fire by software now (same body as the ISR)
        force CCMR2 OC2M = Force-active     # drive PC2 high
        enable DMA CH4                       # TE already on; DMA pumps first byte

`MAX_REASONABLE_REMAINING` is the largest legitimate fire-in-the-future we expect (RDT_max + slot_offset_max + a few µs slack). Anything past it means the modular subtraction wrapped backwards — i.e., we're "in the past."

The recheck only needs to look at CCR3. CCR3 is `FIRE_BIAS_TICKS` earlier than CCR2 in tick value (CCR2 = CCR3 + FIRE_BIAS); if CCR3 isn't in the past, CCR2 — which sits later by exactly that delta — isn't either. The manual-fire branch runs the same body as the CC3 ISR (drive PC2 high, enable DMA CH4).

---

## 6. DMA channel assignments

CH32V006 DMA1 has fixed source-to-channel mux. Allocation:

| DMA1 channel | Old | New | Notes |
| --- | --- | --- | --- |
| CH1 | ADC pump | unchanged | ADC stays on its existing DMA pump |
| CH4 | USART1_TX (single-shot) | unchanged | Still per-fire reconfigured |
| CH5 | USART1_RX (circular) | unchanged | RX byte ring |
| CH7 | free | **ET ring (TIM2_CH4 IC capture)** | |

Only one new DMA channel comes into play: CH7 for the ET ring. Keeping fire on the software ISR path means TX doesn't need DMA1_CH1, so ADC's existing pump is untouched. A hardware-DMA fire alternative (TIM2_CH3 → DMA1_CH1 with ADC moved off CH1) would shave another ~1 µs but the software-fire floor is already under the 3.33 µs Fast-Last cap, so the rework isn't load-bearing.

**ET channel priority.** Set `DMA_CFGR7.PL = 0b11` (Highest); ADC stays at default. DMA arbitration is per-transfer and atomic — priority decides the *queued-request* race, not the in-flight transfer. ET arriving mid-ADC waits ≤80 ns; ADC arriving mid-ET waits the same. ADC's sample-and-hold instant is TIM1-TRGO-anchored and not jittered by DMA contention — only the sample-complete → memory-write latency moves by ≤80 ns, invisible to a 40 kHz control loop. At 3 Mbaud worst case (1.5 M edges/sec) ET sees at most one 80-ns ADC contention per edge, well under the 667-ns intra-byte minimum.

---

## 7. TIM2 configuration

- **Free-running, no OPM, no slave-mode trigger.** SMS = 0, OPM = 0. CNT counts forever.
- **CC4 = input capture; CC2 / CC3 = output compare.**
- **Prescaler.** PSC = 0 — CNT counts at HCLK = 48 MHz. One tick = 20.83 ns. ARR = 0xFFFF, period ≈ 1.365 ms. The IC side wants fine resolution: at 3M, bit_time = 16 ticks; the classifier's `[9·bit, 11·bit]` window = `[144, 176]` ticks. Higher prescalers eat the tolerance budget. Wrap is handled by a cumulative byte counter (ET consumers) and by the set-and-recheck pattern (§5.4, TX fire arming).
- **Capture filter on CH4** (ICF bits in CCMR2). Filter width must be shorter than one bit-time at the operating baud. The picker rule: largest min-pulse strictly under `bit_time` at `fDTS = fck_int = HCLK` (CKD = 0). Computed alongside BRR; precomputed at parse time, register write at TC tail (§9).
- **Polarity on CH4** (CC4P): falling edge only — the start-bit *begin*.

---

## 8. Memory rings

Three rings, three lifetimes. RX and BT are always 64 entries; ET sizing is a CPU-vs-memory tradeoff (see §8.4).

### 8.1 RX ring

64 bytes, circular, DMA1_CH5 from USART1.DR. The parser drains continuously (at classifier-end in Plain mode; at each fold interval in Fast Last mode) instead of waiting for IDLE — ring size decouples from max packet size.

### 8.2 ET ring (new)

Circular, DMA1_CH7 from TIM2.CCR4. Captures the TIM2 CNT value at every falling edge on PC1. Default sizing: **128 entries × 16-bit = 256 B** (Fast Last-safe).

ET overflow is detected by comparing NDTR against the consumer's last drained index. Counted as a fault; the classifier's anchor resets at the next IDLE. Wire-CRC still validates the packet — a missed BT entry is a degraded timing signal, not a wire-level error.

### 8.3 BT ring (new)

64 entries × 16-bit = 128 B (with a separate 32-bit cumulative-byte counter for wrap disambiguation). Sized to match the RX ring so byte index `i` in RX maps to byte time `BT[i mod 64]`.

In Plain mode the classifier writes BT at HT/TC of DMA1_CH7. In Fast Last mode at ≥1M the Fast Last scheduler ISR writes BT (classifier ISR is disabled — §10.6). Consumers read by index regardless of which ISR wrote — same High priority, no locking.

### 8.4 ET sizing: 128 (default) vs 64 (memory-tight)

The Fast Last scheduler runs at 17-byte intervals; classifier is disabled between intervals, so ET must absorb one interval's edges without overwriting. Worst-case 5 edges/byte (0x55-heavy stream): 17 × 5 = 85 edges → **128 is the safe choice.** Total ring memory: 64 + 256 + 128 = **448 B**.

Dropping ET to 64 (saves 128 B) requires a 12-byte interval (12 × 5 = 60 edges). The CPU delta is ~1% (per-byte work dominates per-ISR overhead); 128 B on an 8 KB chip is modest. Default to **A**; revisit if SRAM tightens. Plain mode is unaffected either way — its classifier-end drain handles peak rates at ET = 64 already.

---

## 9. ISRs and priorities

Same two priority levels (V006 PFIC has nothing more). DXL-related IRQs stay at High alongside today's; only ADC sits at Low.

| Priority | IRQ | Body | Where |
| --- | --- | --- | --- |
| High | USART1 | IDLE (parser kick) + TC (release bus) + RX errors | `.highcode` |
| High | DMA1_CH7 HT/TC | Classifier walk (ET → BT) + parser drain. Folds Fast Last CRC inline when armed. *Paused during Fast Last at ≥ 1M only* (§10.6.4) | `.highcode` |
| High | TIM2 (CC1IE) | Fast Last scheduler (at ≥ 1M only): classifier + parser drain + CRC fold over 15-byte interval. Last fold body cancels CC1 after the busy-wait; CC3 owns post-fire residue. | `.highcode` |
| High | TIM2 (CC3IE) | Fire TX: enable DMA CH4 (FIRST action — wire bit lands here for all reply kinds). For Plain replies, body ends there. For Fast Last, tail extends with residue fold + `patch_crc`. | `.highcode` |
| Low | DMA1_CH1 | ADC kernel pump (unchanged) | flash |

All wire-side IRQs at High serialize cleanly via same-priority no-preemption. The structural protection for the Fast-Last fire deadline isn't priority — it's **scheduling**:

- During Plain replies, classifier may fire whenever HT/TC trips. No inter-slot jitter cap, so a ~5 µs classifier walk delay before fire is harmless (RDT ≥ 250 µs typical).
- During Fast Last replies at ≥1M, the classifier ISR is masked at first fold entry. From then until USART1 TC, classifier doesn't compete for the High slot — Fast Last mode is entirely owned by the Fast Last scheduler ISR cadence (§10.6) and the fire ISR. The last fold is scheduled to exit before `fire_tick − FIRE_BIAS`, so fire never waits behind the fold scheduler either.

**DMA1_CH5 HT/TC interrupts are not enabled.** USART1's RX byte-DMA pump runs, but the parser-drain consumer is the classifier walk (or the Fast Last scheduler at high baud), not a per-half RX IRQ. Framing has no RXNE owner anymore either.

**Why all DXL-side IRQs share High priority.** Three invariants depend on same-priority serialization:

1. **Fold-and-fire serialize through the same vector.** CC1 and CC3 share TIM2's IRQ vector. The dispatcher checks CC1IF first, then CC3IF — both in one entry. If CC1's body is running when CC3IF latches (the normal case at 3M GUARD=1, site 1 spins past `fire_deadline_tick`; see §10.6.2 and [dxl-fast-chain-crc-walkloop.md](dxl-fast-chain-crc-walkloop.md) §4), the dispatcher's second `if` catches CC3IF on CC1's exit. Wire-bit jitter at 3M is ~3 µs late vs `fire_deadline_tick`, under one char_time. Dispatcher order is load-bearing: CC1 before CC3.
2. **TC release before master's next start bit.** TX_EN drop in TC must land before any next request's start edge to avoid bus contention. TC at High keeps that bounded.
3. **Fold interval cadence.** §10.6 budgets ~54% CPU per interval; CC1 at High keeps Low work from eating the headroom.

The intuitive alternatives ("TX at High, RX at Low"; "CC3 alone at High") break all three. The remaining concern — TC tail of reply N−1 delaying CC3 of reply N — is addressed at root by the TC tail budget below.

**TC tail budget.** `apply_pending_after_tc` must run after wire activity ends, but *computation* of new values (BRR, capture filter, fine-trim) moves to **parse time**: the parser stores the computed result, TC tail does only the atomic register writes — bounded under 1 µs. Under one byte-time at the highest baud, the "next CC3 latches mid-TC-tail" delay vanishes for any realistic RDT. Out-of-envelope contention surfaces as fire-late telemetry.

**USART1's role shrinks.** IDLE as a parser-kick signal (no timing value derived from it), TC for "reply done, release the bus" + apply pending, RX errors for telemetry. The framing FSM, `decide(brr, rdt)` rule, `pipeline_margin_us` knob, char-time backdate constant, IDLE-stamp queue, and RXNE snapshot all **go away** — every byte gets a per-byte timestamp from BT, no high-vs-low-baud strategy split.

**SysTick CMP is unused for DXL.** Free for telemetry. **TX_EN is hardware-driven** on CCR2 → PC2 via OC mode (§5.2). **Fire** is software on CCR3 → CC3IE → ISR (§5.1). The fire ISR is M3-lean for Plain replies (`dma::enable(CH4)` then return). For Fast Last replies the body extends post-fire to fold the GUARD-byte residue and `patch_crc` the trailing CRC slot inline, matching legacy `b03dd355::on_systick::TxArmed` — see §10.6.2. The wire-bit timing is unchanged (fire is always the first action); only the body's tail differs by reply kind.

---

## 10. The window classifier (primary RX byte-timing design)

The classifier turns ET entries into BT entries. It runs at every HT/TC of DMA1_CH7.

### 10.1 Core idea

Each UART byte starts with a 1→0 transition (the start bit). After the start bit, more 1→0 transitions can happen inside the byte (data bits). After the stop bit, the line is high until the next byte's start bit.

If we know `BT[i]` (start tick of byte `i`), then byte `i+1`'s start bit is **exactly** 10 bit-times later (start bit + 8 data bits + 1 stop bit = 10 bit positions). Any ET entry between `BT[i]` and `BT[i] + 10·bit_time` is a data-bit transition (discard it). The first ET entry at or after `BT[i] + 10·bit_time` is `BT[i+1]`.

That's the whole algorithm:

    for each new ET entry t:
        if t ≤ anchor + 11 × bit_time and t ≥ anchor + 9 × bit_time:
            # t is the next start bit
            anchor = t
            BT[byte_idx] = t
            byte_idx += 1
        elif t < anchor + 9 × bit_time:
            # data-bit transition — skip
        else:
            # gap > 11 bit-times: inter-byte stall or end of packet
            # re-anchor at the next entry treated as a start bit
            anchor = t
            BT[byte_idx] = t
            byte_idx += 1

The window is `[9 × bit_time_spec, 11 × bit_time_spec]` where `bit_time_spec = HCLK / baud` — the slave's computed bit-time. At 3 Mbaud, `bit_time_spec = 16` ticks; the window is `[144, 176]` ticks from the previous anchor. The algorithm re-anchors `anchor = t` on every successful match, so drift between slave HSI and master HSE doesn't compound across the packet — only the static window tolerance matters. §10.7 covers the drift budget and the BT-ring-based feedback loop that closes it.

**First-edge seeding and header-grid alignment.** At classifier reset (boot, IDLE, post-cancel), `anchor` is invalid; the next ET entry seeds it. This is structurally indistinguishable from the case where a pre-packet glitch (driver settle, EMI) produces an edge before the real start bit — `BT[0]` would commit to the glitch tick and every subsequent BT index would shift by one in RX-space.

The classifier doesn't try to disambiguate. It commits BT entries naively. **Alignment is the parser's job**: a DXL 2.0 packet always begins with the fixed header `FF FF FD 00`, so `BT[j..j+3]` (where `j` is the parser's hypothesis for the BT index of `RX[0]`) must form a strict 10·bit_time grid. The parser probes `j = bt_frontier − packet_len`, validates that `BT[j+1] − BT[j]`, `BT[j+2] − BT[j+1]`, `BT[j+3] − BT[j+2]` all equal `10·bit_time_spec` within ±1 tick. If the grid fails, slip `j` by ±1 and retry. After 2–3 slip retries with no match, declare BT degraded for this packet and fall back to packet-boundary timing for fire scheduling.

This keeps the classifier free of protocol awareness and the parser free of timing arithmetic until alignment is established. The two paths reconcile at one well-defined point per packet.

### 10.2 Why this is robust

- **Glitch in the middle of a byte:** a spurious falling edge between `anchor + 0` and `anchor + 9 × bit_time` is rejected by the window test. No state corruption.
- **Glitch in the inter-byte gap that happens to fit the window:** the next *real* start bit will land outside the window (because the real byte is offset by the glitch's arrival time). The classifier re-anchors on the next entry that fits a window from the *new* anchor — self-heals within 1 byte.
- **Missed edge (ET overrun):** the number of dropped edges is unknown, so `anchor` cannot be a valid predictor of the next start bit. Detect at HT/TC via NDTR-vs-consumer comparison; invalidate `anchor` and mark BT entries from the overrun point as sentinel-invalid. The next ET entry seeds a fresh anchor via the same first-edge path as packet start (§10.1), and the parser detects degradation when `bt_count_in_packet < rx_count_in_packet` — falling back to packet-boundary stamps for the remainder of this packet. Next IDLE refreshes everything. Detectable and bounded.
- **Spec-baud mismatch:** the window is computed from `bit_time = HCLK / baud`. If the host writes a baud the slave isn't running at, every entry falls outside every window — the whole packet's BT marks as invalid. Telemetry visible.

The classifier is **constructive**: it builds byte boundaries from edge times, not from byte content. The RX ring's data and the BT ring's times are derived from the same wire events but via independent paths. A byte data error doesn't poison BT; a BT error doesn't poison byte data.

### 10.3 CPU cost

Per HT/TC the classifier walks up to 32 ET entries (half the 128-entry ring). Per entry: 1 load (ET), 2 comparisons (window low, window high), 1 store (BT), 1 anchor update. ~8 cycles per entry × 32 = ~256 cycles per walk = ~5.3 µs at 48 MHz.

At 3 Mbaud worst-case edge rate (1.5 M edges/s), HT/TC fire at ~47 kHz. Classifier consumes 47 kHz × 5.3 µs = ~25% CPU during peak burst.

Per-walk CPU summary at sustained 3M traffic (worst case, real traffic is bursty):

| Component | Per walk | Cadence (5 edges/byte) | Cadence (1 edge/byte) | CPU % |
| --- | --- | --- | --- | --- |
| Classifier walk | 5.3 µs | 21.3 µs | 107 µs | ~25% (cadence-bound) |
| Parser drain (~50 cyc/byte) | 3 µs (6 B) — 33 µs (32 B) | — | — | ~30% (byte-bound) |
| **Total during sustained RX** | | | | **~55%** |
| **During TX** (no edges) | 0 | n/a | n/a | **~0%** |

The TX-phase CPU is what makes the design pay off — the chip can spend its budget on motor control during TX while the bus is busy with our reply.

### 10.4 Where it lives

Body in `.highcode`. The HT and TC interrupts on DMA1_CH7 give two entry points: HT walks the first half of the ET ring, TC walks the second half. State is a single `(anchor_tick, byte_idx)` pair updated in-place. Reset on USART1 IDLE (packet boundary): anchor invalid until the next ET entry, which is the next packet's first start bit.

### 10.5 Coordinating with the parser

The classifier produces BT entries at ET HT/TC; USART1's RX DMA produces RX bytes at each stop bit. `RX[i]` lands ~10 bit-times after byte i's start edge; `BT[i]` lands when the classifier walks the ET entry, at the next HT/TC. Worst case, BT lags RX by up to half a ring of edges.

**Parser invariant:** consume bytes only up to `min(rx_write_pos, bt_write_pos)`. Past that point, RX data may exist without a BT tick. The parser is stateful — a mid-packet trigger drains to the BT frontier, yields with in-progress state preserved, resumes on the next event:

    parsed_idx: u16         # bytes consumed so far (cumulative, with wrap counter)
    decode_state: ...       # header / length / payload / CRC stage
    expected_end: Option<u16>  # set once header.length is decoded

**Trigger sites:**

- **Classifier HT/TC (DMA1_CH7).** Natural drain point — the classifier just walked ET → BT, so drain RX up to the new BT frontier in the tail of the same ISR.
- **USART1 IDLE.** Backstop for small packets that don't fill ET enough to trip HT. A 14-byte all-0xFF reply generates only 14 edges, below half the 128-entry ring. **IDLE is a *signal*, not a timing source** — wire-end still comes from `BT[last_byte] + 10·bit_time`.

No RX HT/TC parser trigger, no SysTick trigger, no decimated counter.

**Packet boundaries are a parser concept, not a classifier concept.** DXL 2.0 doesn't require inter-packet silence; back-to-back requests don't reset the classifier's `anchor` and `byte_idx` keeps counting across boundaries. The parser detects `FF FF FD 00` and stamps `packet_start_rx_idx` + `packet_start_bt_idx`; BT consumers walk forward from `packet_start_bt_idx + k`. The seam pair (last byte of N → first byte of N+1, separated by exactly 10·bit_time) folds into drift estimation as a bonus tight sample.

**Wire-end derivation** at request_complete:

    wire_end_tick = BT[parsed_end_idx - 1] + 10 × bit_time
    fire_tick     = (wire_end_tick + RDT_ticks - FIRE_BIAS) & 0xFFFF
    TIM2.CCR3     = fire_tick
    # ... + set-and-recheck guard from §5.4

No backdate constant, no char-time math.

**Plain mode:** classifier HT/TC + IDLE drain the parser; no CRC fold, no Fast Last state.

**Fast Last mode at ≥1M:** the Fast Last scheduler ISR (§10.6) replaces both. At first fold entry, classifier HT/TC is masked; from then until USART1 TC, the scheduler owns the full classifier + parser + CRC fold cadence. IDLE doesn't run during a Fast Last reply window.

### 10.6 Fast Last CRC fold

Fast Last CRC accumulates the predecessor's reply bytes into a running CRC, then patches the trailing CRC slot of our own reply so the wire bytes form one continuous CRC stream covering header + predecessors + our own bytes. ("Fast Last" is internal shorthand for the Fast Sync/Bulk Read reply at the last slot — not a Robotis spec term.) The mechanism factors into **state + trigger**:

- **State** lives composite-side: a running bulk CRC, a snoop-head index (the parsed index at arm time), a byte counter, and the predecessor-byte budget. The per-byte fold operation, called from the parser drain, is a no-op when not armed. When the byte counter hits the budget, fold finalizes over our own reply bytes, patches the TX buffer's trailing CRC slot, and disarms — idempotent across redundant trigger sites.
- **Trigger** picks which ISR(s) drive the parser drain (and therefore the fold) during the predecessor window. **The trigger varies with baud.**

#### 10.6.1 Baud-dispatched trigger paths

| Baud | Trigger sites during Fast Last window | Finalize lands at |
| --- | --- | --- |
| ≥ 1M | TIM2 CC1 grid walks + last-anchor busy-wait. DMA1_CH7 HT/TC paused. CC3 fire body tail folds GUARD residue + patches. | CC3 fire body, ≈ fire_deadline_tick + ~5 µs (walker + finalize + patch). |
| < 1M | DMA1_CH7 HT/TC (when it fires) + USART1 IDLE. Nothing paused. | USART1 IDLE body, ≈ t_prior_end + 1 byte_time. |

The threshold is 1M because three constraints converge:
- **Ring-fill vs predecessor length.** At ≥ 1M the predecessor often doesn't fill the ET ring enough to trigger HT (3M n_pred=14 lands 14 ring entries; HT fires at ~32). CC1 grid walks become necessary to drain on a tighter cadence.
- **u16 fit on TIM2.** `interval_ticks = 15 × byte_time` stays comfortably below half u16 only at ≥ 1M. Below that the CC1 grid math runs out of register width before low-baud is even useful.
- **IDLE-as-finalize budget.** At < 1M USART1 IDLE fires comfortably before fire_deadline (IDLE detection ≈ 1 byte_time after the predecessor's last byte; fire_deadline is at least rdt + slot_offset_bytes ≫ idle_window away). At ≥ 1M, with close coalesce, fire_deadline can land before IDLE fires — finalize MUST happen in the CC3 fire body's tail (matching legacy `b03dd355::TxArmed`) instead of IDLE.

#### 10.6.2 High-baud path (≥ 1M, TIM2_CH1)

The Fast Last scheduler runs at fixed-byte intervals on TIM2_CH1, owning **all** RX-side work during the predecessor reception window. Same scheduling pattern as the canonical walk-loop + split-fire design in [dxl-fast-chain-crc-walkloop.md](dxl-fast-chain-crc-walkloop.md), moved to TIM2 CC1.

**Arm time** (Fast Last reply scheduled, baud ≥ 1M):

    arm Fast Last CRC state with (snoop_head = parsed_idx_now, expected_n_pred)
    pause DMA1_CH7 HT/TC                                # §10.6.4 classifier merge
    walk_deadline         = t_prior_end - GUARD × byte_time
    last_anchor_tick      = fire_deadline_tick - min(interval, t_prior_duration) - GUARD × byte_time
    current_fold_anchor   = step_back(last_anchor_tick, interval, t_prior_start)
    patch_deadline_tick   = fire_deadline_tick + (tx_len - 2) × byte_time
    CCR1 = current_fold_anchor - FAST_LAST_ENTRY_TICKS  # every CMP back-dated, not just last
    # CCR2 + CCR3 already armed by the TX scheduler (§5.3); CC1 added here.

`interval_ticks = 15 × byte_time`. Every grid CMP is back-dated by `FAST_LAST_ENTRY_TICKS` so the body's fold-start lands on the formula's wall-clock anchor instead of `anchor + ISR_entry_latency`. Tracking `current_fold_anchor` in state and advancing by fixed `+ interval` keeps the grid rigid regardless of per-body overhead.

**At every CC1 IRQ (intermediate):**

    walker():
        walk classifier                      # ET → BT, advance anchor
        drain parser, folding Fast Last CRC per byte
    current_fold_anchor += interval_ticks    # grid advance, drift-free
    CCR1 = current_fold_anchor - FAST_LAST_ENTRY_TICKS

**At the last CC1 IRQ (anchor = last_anchor_tick):**

    walker()
    loop:                                    # site 1 busy-wait
        walker()
        if bytes_walked >= expected_n_pred - GUARD: break
        if now >= walk_deadline: break
    cancel CC1                               # no post-fire CC1 — CC3 owns residue

The busy-wait is uncontested — DMA1_CH7 HT/TC is masked (§10.6.4), so CC1 is the sole High consumer in the Fast Last window. At 3M GUARD=1 the loop exits via `walk_deadline` (recursive-fire mode; see [dxl-fast-chain-crc-walkloop.md](dxl-fast-chain-crc-walkloop.md) §4). CC3IF latches during the spin; the TIM2 dispatcher catches it on CC1's exit and runs the fire body. Wire-bit jitter at 3M is ~3 µs late vs `fire_deadline_tick`, within < 1 char_time.

**At CC3 (fire body):**

    dma::enable(CH4)                         # FIRST action — wire bit lands here
    if fast_last_crc.armed:
        loop:                                # post-fire residue fold
            walker()                         # GUARD byte landed in NDTR via CH5
            if !fast_last_crc.armed: break   # fold's finalize ran → patch_crc landed
            if now >= patch_deadline_tick: break   # bail; trailing CRC stays at placeholder
        fast_last.cancel                     # idempotent — already done at last CC1

The post-fire residue fold lives in CC3's body — matches legacy `b03dd355::on_systick::TxArmed` (which busy-waited GUARD bytes via `wait_and_fold_until`, then `patch_crc`, all inline after `fire_now`). `patch_crc` must land in `tx_buf[len-2..len]` before DMA1_CH4's read cursor reaches them; CH4 reads byte[N-2] at ≈ `fire + (tx_len-2) × byte_time` (~40 µs at 3M for a 14-byte reply), and the post-fire tail body takes ~5 µs — slack of ~10 byte_times. Legacy reported any patch landing with `dma::remaining(CH4) ≤ 2` as `CrcPatchDeadlineMiss`; same shape here, telemetry deferred to Plan 2 follow-ups.

**Why CC3 and not a separate post-fire CC1 anchor.** An earlier draft of this design split the post-fire fold onto a dedicated CC1 anchor at `t_prior_end + GUARD_LATCH_TICKS` to keep CC3 minimal. That split is wrong — the inter-vector transition (CC3 body return + TIM2 vector re-entry + CC1 ENTRY_TICKS) burns enough of the ~5 µs typical post-fire budget that any wrap-guard slip drives `patch_crc` past CH4's read of `tx_buf[len-2]`. Co-locating fire and residue in CC3 keeps the patch deadline bounded by `tx_len × byte_time`, not by inter-vector overhead.

**At USART1 TC (reply complete):**

    disarm Fast Last CRC state               # idempotent — fold already disarmed
    cancel Fast Last scheduler               # idempotent — already cancelled at last CC1
    un-pause DMA1_CH7 HT/TC

#### 10.6.3 Low-baud path (< 1M, HT/TC + IDLE inline)

Fast Last CRC state arms as usual; **the high-baud scheduler is NOT armed** and DMA1_CH7 HT/TC stays **un-paused**. The existing classifier path now folds CRC as a side-pass:

    DMA1_CH7 HT/TC → classifier walk → drain parser, folding Fast Last CRC per byte
    USART1 IDLE    → drain parser, folding Fast Last CRC per byte

At < 1M the predecessor wire window is long enough (500k n_pred=14: 280 µs of wire time) that `n_pred` × ~0.5 µs CRC update = ~7 µs of inline fold work in the IDLE body — trivial. HT/TC may or may not fire depending on whether the predecessor fills the ring half; either way, the fold completes inside IDLE.

No new ISR, no timer, no busy-wait, no grid math, no §10.6.4 classifier merge. The fold operation is identical at high and low baud — only the trigger differs.

#### 10.6.4 Classifier+parser merge (high-baud only)

At ≥ 1M the CC1 path masks DMA1_CH7 HT/TC at first-fold entry. From then until USART1 TC, classifier doesn't compete for the High slot — CC1 is the sole HIGH consumer. This is what makes the site 1 busy-wait deterministic.

**Pending-IRQ race.** If classifier HT/TC latches right before CC1 fires, the fold body's pause write happens *after* the latch — classifier IRQ is still pending and would run after CC1 exits. After masking HT/TC, also clear pending on DMA1_CH7. CC1's first body has already done the classifier's job inline, so dropping the pending bit is correct.

#### 10.6.5 CCR1 wrap guard

`CCR1 += interval_ticks` re-arm has the same 16-bit wrap hazard as CCR3 (§5.4). If CC1 runs late, `CCR1 + interval_ticks` can fall behind `CNT` — the next match wraps 1.365 ms before firing. Set-and-recheck applies: after writing CCR1, compare `(CCR1 − CNT) & 0xFFFF` against the wrap-guard threshold; if exceeded, manually invoke the fold body now and re-arm forward from the new CNT.

#### 10.6.6 Per-interval cost at 3M

Default 15-byte interval, mid-range edge density:

| Work | Cycles | µs |
| --- | --- | --- |
| Classifier walk (~45 edges) | ~360 | 7.5 |
| Parser drain (15 bytes × 50 cyc) | 750 | 15.6 |
| CRC fold (15 bytes × ~10 cyc) | 150 | 3.1 |
| ISR entry/exit + bookkeeping | ~50 | 1.0 |
| **Total per interval** | **~1310** | **~27.3 µs** |

Against a 50 µs cadence (15 × 3.33 µs) that's ~54% CPU during the Fast Last reception window — same order as the Plain-mode (classifier + parser drain) cost during sustained RX. The work hasn't grown; it's just relocated into the CC1 cadence at high baud, or into HT/TC + IDLE at low baud.

#### 10.6.7 Plain replies

Plain replies skip both the Fast Last CRC arm and the CC1 fold scheduler. Classifier HT/TC stays armed; parser drains at classifier-end (§10.5). The per-byte fold is a no-op when not armed, so the HT/TC + IDLE callbacks remain safe to invoke unconditionally.

### 10.7 HSI drift and the bit_time assumption

The slave's HCLK comes from HSI (internal RC), the master's baud comes from HSE (crystal). The classifier's window math uses `bit_time_spec = HCLK_slave / baud` — what the slave *thinks* a bit-time is — but the actual on-wire bit length is set by the master. If slave HSI runs at fraction `(1 + D)` of master's reference, inter-byte spacing measured into ET is `10 × bit_time_spec × (1 + D)`.

**Static tolerance: the window survives ±10% drift.** Upper-window failure (gap re-anchor) triggers at `D > 10%`; lower-window failure (bit-7 bleed into start window) at `D > 12.5%`. V006 HSI typical excursion is ≤ 0.5% at 25 °C post-factory-cal, ~3% over -40..125 °C uncal, well under 1% after cold-start trim. The window has ~10× margin — **the spec-bit_time math is correct as-is, no within-packet correction needed.**

**Per-byte error accumulation: zero.** §10.1's `anchor = t` (not `anchor += 10·bit_time`) means each window re-centers on the actually-measured previous start. Drift errors don't compound across the packet.

#### 10.7.1 BT ring as the drift signal

The same BT entries give us a direct drift measurement. For any tight-window byte pair:

    observed_bit_time_ticks = (BT[i+1] - BT[i]) / 10
    drift_ppm = (observed - bit_time_spec) / bit_time_spec × 1e6

Inter-byte interval at 3M ≈ 160 ticks; capture noise σ ≈ 1 LSB per BT entry. Over 50 pairs in a single packet, drift estimate σ ≈ 1250 ppm — borderline per-packet. Averaging across ~10 packets tightens to ~400 ppm; over a minute, sub-100 ppm.

Filtering: only count tight matches (no re-anchor between), discard pairs spanning ET overrun, pause measurement during baud-change windows. Replaces IDLE-based drift inference (see [[no-idle-timing]]) — BT measurement is per-byte and timing-source-clean.

#### 10.7.2 Feedback into HSITRIM

V006's HSI tune is `RCC_CTLR.HSITRIM[4:0]`, ~2500 ppm per step. One step shifts the 16-tick bit_time by 0.04 ticks; window-edge shift at the 11·bit boundary is 0.4 ticks — negligible against the ±10% (16-tick) window. So HSITRIM can be freely retuned at packet boundaries without classifier desync.

Coarse trim aims drift inside ±1250 ppm (half a step). The HSITRIM write is committed at USART1 TC tail, alongside any pending BRR / capture-filter updates (§7). Input signal: BT-averaged drift estimate.

Sub-step residual goes to a software fine-trim: a per-ppm correction applied to the wire-end → fire arithmetic. Concretely, `BT[last_byte] + 10·bit_time + RDT_ticks` is adjusted by `residual_ppm × elapsed_ticks_since_last_BT` — a few-tick correction over the wire-end-to-fire window.

---

## 11. Alternative: LUT walker with resync detector

Considered, rejected, documented for completeness. May come back if §10's CPU cost bites under some workload.

**Idea.** Edge count per UART byte is a fixed function of byte value (1–5). A 256-entry LUT gives `edge_count[byte_data]`; the walker reads `RX[i]`, advances the ET cursor by `edge_count[RX[i]]`, takes the next ET entry as `BT[i+1]`. ~5 cycles per byte vs. ~24 cycles per byte for §10 — roughly 5x faster.

**Why it's flawed.** The LUT uses *byte data* to predict *edge structure*. Anything that desyncs cursor from bytes corrupts every subsequent BT entry until the next IDLE resync: a phantom edge (glitch) makes the walker under-advance, a missed edge (ET overrun) makes it over-advance, a corrupted byte feeds a wrong LUT value. A resync sanity-check (verify the next ET entry falls in `[BT[i] + 9·bit, BT[i] + 11·bit]`) catches most desyncs but misses ~2.5% of phantom-edge cases per glitch event, and can only resync at IDLE — mid-packet realignment isn't possible.

| | Window classifier (§10) | LUT walker + resync |
| --- | --- | --- |
| Self-healing | Within 1 byte | Detect-only, requires IDLE to resync |
| Silent corruption rate | Effectively zero | ~2.5% per glitch event escapes |
| CPU during peak burst | ~25% | ~5% |
| Memory | ET + BT | Same + 256 B LUT |

The CPU savings aren't load-bearing — §10's 25% peak leaves comfortable margin for everything else at High. Robustness is what matters for a transport whose entire point is precise per-byte timing.

---

## 12. Open questions for the spike

- **AFIO dual-remap stability.** F1 documentation is explicit about peripheral-side input dual-tap working; V006 RM is less so. Confirm by direct bench measurement.
- **Capture filter setting per baud (§7).** Filter width must be < 1 bit-time at the operating baud; default `(fCK_INT/2, N=2)` for 3M. Compute alongside BRR; relax (heavier filter) at lower baud if bench shows noise immunity needs it.
- **Fire-floor MIN measurement.** Measure CCR3-match → first-wire-bit at 3M on bench across many fires. Lock `FIRE_BIAS_TICKS` to the **1st-percentile lower bound** (not the mean) so the wire bit always lands at or after `fire_tick` — see §5.1 / §5.3. Placeholder estimate is ~46 ticks (~960 ns); confirm on hardware.
- **ET sizing + interval (§8.4 Option A vs B).** Default plan is 17-byte interval / ET depth = 128. Option B (12-byte interval / ET depth = 64) saves 128 B of SRAM at ~1% extra CPU. Decide based on memory pressure measured during integration.
- **Fast Last CRC fold cost.** §10.6 estimates ~10 cyc/byte for CRC16 update. If the implementation lands closer to 20 cyc/byte, peak CPU during Fast Last RX climbs from ~54% to ~60%. Still well under the 75% spike target.
- **Classifier-pending-at-fold-entry race.** §10.6 proposes "mask + clear pending" at first CC1 entry to defeat a classifier IRQ that latched right before the fold scheduler fired. Confirm the clear-pending write to `PFIC.IPRR.CH7` actually drops the latched IRQ on V006 (not just future ones).
- **`walk_deadline_margin` sizing.** The busy-wait exit in the last fold leaves GUARD bytes for post-fire. The prior SysTick-tuned value is the starting point; re-measure under TIM2 scheduling.
- **HSI drift convergence.** §10.7.1's BT-averaged drift estimator needs N packets for a usable estimate. Measure σ on bench at the target traffic mix to set the smoothing window.
- **First-byte BT seed under pre-packet glitches (§10.1).** Naive classifier + parser header-grid handles the common case. Verify on bench with EMI injection that 2–3 slip retries are enough in practice; if not, add tentative-anchor confirmation in the classifier.
- **TX_EN OC2M transitions (§5.2).** Bench-verify the Active-on-match + Force-inactive sequence produces clean rising/falling edges without spurious toggles at init, fire-cancel, and TC.
- **CC3IF latch on rearm at current CNT (§5.4).** Write CCR3 = current CNT on bench and observe whether CC3IF latches immediately, on next tick, or only after a wrap. Drives the size of `MAX_REASONABLE_REMAINING`.
- **TC tail budget (§9).** Profile worst-case TC tail housekeeping runtime after the parse-time precompute lands. Verify it stays under one byte-time at the highest expected baud.
- **Low-baud wrap span (§5.4).** At 9600 baud, `10·bit_time ≈ 50000` ticks — close to the 16-bit wrap. Size `MAX_REASONABLE_REMAINING` against the largest legal inter-byte gap, not just RDT.
- **TIM2 ↔ SysTick clock bridging.** Capture a fixed offset once at boot for telemetry events that span both clocks. Define the convention in the telemetry layer.
- **AFIO-then-DMA bringup order.** AFIO remap must complete before DMA1_CH7 arms, or the first ET entries are garbage on transient pad state.

---

## 13. One-paragraph summary

> Every DXL transport timing decision moves off USART IDLE and onto TIM2 input capture. PC1 dual-remaps to USART1_RX and TIM2_CH4; CH4 captures every falling edge into a ET ring via DMA. A `.highcode` window classifier walks ET into a per-RX-index BT ring at HT/TC using a `[9·bit, 11·bit]` start-bit test — constructive, self-healing within one byte, robust to glitches and overruns. Re-anchoring on every match keeps HSI-vs-master drift from compounding; ±10% static window margin covers ~10× realistic HSI excursion. Wire-end is `BT[last_byte] + 10·bit_time`, per-byte precise at every baud — no IDLE-derived backdate, no framing FSM, no RXNE branch. TX fires from a TIM2_CH3 compare → lean CC3IE ISR enabling DMA CH4, with CCR3 biased earlier by a MIN-calibrated `FIRE_BIAS_TICKS` so the wire bit lands on time; PC2 (TX_EN) toggles in hardware on TIM2_CH2. The fire floor drops from ~5 µs to ~1 µs, comfortably under the 3.33 µs Fast-Last cap at 3 Mbaud. In Fast Last mode at ≥1M, the classifier is masked at first-fold entry and a periodic TIM2_CH1 scheduler owns classifier + parser drain + CRC fold at 15-byte intervals, busy-waits the last fold past `walk_deadline`, then folds the GUARD residual post-fire; at <1M the fold runs inline in the classifier and IDLE bodies. The BT ring also doubles as the drift signal, feeding HSITRIM and a software fine-trim residual. Set-and-recheck on CCR3 (and CCR1) catches "armed in the past" against TIM2's 1.365 ms wrap. ADC stays on DMA1_CH1; only DMA1_CH7 is new. A LUT-walker alternative is documented in §11 and rejected for silent-desync risk.

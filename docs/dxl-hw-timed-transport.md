# DXL 2.0 Hardware-Timed Transport on the CH32V006

A design note for the next-generation DXL transport on the V006. Read [dxl-rx-timing.md](dxl-rx-timing.md) first if you haven't — this doc replaces the ISR-driven fire path it describes with a hardware-timed one, and adds per-byte RX timestamping built on TIM2 input capture. **Read [dxl-hw-timed-transport-addendum.md](dxl-hw-timed-transport-addendum.md) alongside this doc** — it documents the Sync/Bulk Read protocol findings that scope "fire-tick" and the §10 design simplifications they drive (deferred classifier walk, persistent BT ring removal). The pieces that didn't change (USART1 framing, half-duplex muxing, Fast Last CRC, baud/RDT decision rule) still apply.

The headline change: every fire and every received-byte timestamp move into TIM2. SysTick stops driving wire-edge events. The ~5 µs PFIC-entry floor on TX fire goes away. RX timestamps become per-byte instead of per-packet, which closes the door on a whole class of low-baud / Fast-Last edge cases that today rely on backdating estimates.

The cost is that TIM2 is now the DXL transport's timer, and is no longer available for motor encoder counting. On the production board that's fine — there's no encoder. On bench builds it means the encoder bringup will need to find a different home (TIM3 stripped variant won't cut it; see §3 in the existing rx-timing doc).

---

## 1. Why move off SysTick

Three structural problems with the current design.

**Problem 1 — TX fire has a 5 µs PFIC floor.** Every fire today goes: SysTick CMP match → PFIC trap entry (~1 µs) → highcode ISR body (~1.5 µs) → DMA EN + TX_EN GPIO toggle (~2 µs) → first wire bit. At 3 Mbaud, one byte is 3.33 µs. The 5 µs floor is the inter-slot jitter cap. Every Fast-Last reply at the top of the baud range sees a visible idle gap before its slot. The CRC stays correct (the Fast Last CRC machinery handles that), but the wire-side coalesce isn't.

**Problem 2 — every timing decision goes through IDLE.** Today's wire-end derivation is `systick::ticks() - 9 × BRR` at IDLE-handler entry. IDLE fires one char-time *after* wire-end, so we backdate. At high baud this works; at low baud the publish latency exceeds RDT and the framing FSM has to switch to per-byte RXNE timestamps (rx-timing doc §4–5), burning ~100 K IRQs/sec at 1 Mbaud just to keep timing source-of-truth. The whole IDLE-vs-RXNE FSM, the framing decision rule, the pipeline_margin tuning knob — all of it exists because IDLE is the only timing source the current design has. This is fragile and not extensible.

**Problem 3 — RX timestamps are per-packet, not per-byte.** Snoop tail walks, chain-CRC stage-3, slot timing — all want to know "when did *this specific byte* arrive?" Today they get "when did the packet end" and infer. At 3 Mbaud the sub-byte resolution loss matters.

The hardware-timed design fixes all three at once:

- TX fire becomes a TIM2 compare event with a leaner software ISR (§5). PFIC entry stays in the path but the ISR shrinks and the trigger source isn't shared with anything else.
- **Every byte gets its own wire-arrival timestamp** via TIM2 input capture on the RX pin's falling edges, written to a BT (byte-timestamp) ring at HT/TC of the IC DMA. IDLE becomes a *signal* (drain the parser, packet is done), never a *timing source*. The framing FSM disappears with it.
- All timing consumers — wire-end, fire-tick, slot deadline, snoop tick — look up `BT[i] + 10·bit_time` directly. No backdate constants, no char-time math, no IDLE-vs-RXNE branch.

The expensive part — turning edge timestamps into byte timestamps — runs in software at HT/TC of the IC DMA, off the wire-edge critical path.

---

## 2. The CH32V006 peripheral inventory for this design

Quick refresher on the parts we're claiming.

**TIM2** — 16-bit general-purpose timer. Free-running counter, four capture/compare channels, slave-mode controller, DMA on UP and each of CC1..CC4. Its DMA routing (TIM2_CH4 → DMA1_CH7, TIM2_UP → DMA1_CH2, TIM2_CH3 → DMA1_CH1) is what makes the rest of this plan possible. The existing rx-timing doc lists TIM2 as "reserved for future use" — that future is here.

**AFIO peripheral-side remap.** V006 supports the F1-style trick: a single GPIO pin can simultaneously feed multiple on-chip peripherals via the AFIO remap registers. PC1 with `USART1_REMAP = 0b0011` connects to USART1_RX; the same PC1 with `TIM2_REMAP = 0b010` connects to TIM2_CH4. Both routes are alternate-input — they read the pad without driving it — so they coexist. The pin's IO config (input + pull) is set once, both peripherals see the same waveform.

**DMA1** — same as the existing design: single-transfer per request, no FIFO, fixed source-to-channel mux. Channel sharing is the tightest constraint in this plan. §8 enumerates the conflicts.

**USART1** — unchanged. Still HDSEL half-duplex single-wire, still on PC0 (TX) + PC1 (RX) physically tied to the same data line by the half-duplex peripheral. The new piece is that CTLR1.TE gets written by DMA at the fire moment instead of by the ISR.

**SysTick** — still there, still 32-bit at HCLK, but it no longer drives the wire. After the migration its only role is `systick::ticks()` as a free-running wall clock for telemetry. The CMP-match scheduling logic in `dxl::state` goes away.

---

## 3. Pin assignments

Nothing new on the schematic. We're re-routing internal MUXes, not changing the board.

| Pin | Function (today) | Function (new) | How |
| --- | --- | --- | --- |
| PC0 | USART1_TX (HDSEL — unused pad) | unchanged | HDSEL keeps PC0 quiet; data flows on PC1 |
| PC1 | USART1_RX | USART1_RX **and** TIM2_CH4 input capture | F1-style dual remap (§2) |
| PC2 | TX_EN (GPIO output, driven by ISR) | TX_EN (TIM2_CH2 OC toggle output) | TIM2 remap routes CH2 to PC2 |

PC1 dual-tap is the load-bearing trick of the whole design. The rest is conventional timer wiring.

---

## 4. Peripheral plan: the RX side

### 4.1 Hardware timestamping

The plan is: TIM2 runs free at HCLK / prescaler. PC1's falling edges feed TIM2_CH4 in input-capture mode. Each capture writes CCR4 (the current CNT value) to a DMA destination — the ET ring. DMA1_CH7 is the channel; circular mode; HT and TC interrupts armed.

    PC1 ─┬─→ USART1_RX (DMA1_CH5 → RX ring, unchanged)
         └─→ TIM2_CH4 IC ─→ CCR4 ─→ DMA1_CH7 ─→ ET ring

Properties we lean on:

- **Captures are independent of the slave-mode controller.** CCxIF latches the timer's current count on every selected edge regardless of whether the counter is gated. We run TIM2 free, no slave-mode, no OPM.
- **No per-byte ISR.** DMA writes the timestamp. The CPU only wakes at HT (ring half) and TC (ring full).
- **ET overrun is detectable.** If the parser falls behind and DMA wraps past the consumer's read cursor, we lose timestamps. NDTR-vs-consumer comparison in the HT/TC handler turns this into a counted fault, not a silent corruption.

### 4.2 Why timestamp falling edges, not bytes

The USART doesn't expose a "byte arrived" event we can capture into a timer cheaply. RXNE has the wrong shape (it's a flag, not a timer trigger), and the chip has no peripheral that says "the data line's start bit just began." What we *can* capture is every falling edge on PC1.

UART frame: start (0) + 8 data LSB-first + stop (1). Bit times where a 1→0 transition can happen:

    bit position:  S  0  1  2  3  4  5  6  7  Stop
    value:         0  d0 d1 d2 d3 d4 d5 d6 d7  1

Edges-per-byte ranges from 1 (start bit only, when all data bits are 1) to 5 (start bit plus four 1→0 data transitions, worst case for byte 0x55 etc).

Counting falling edges directly gives us bit-level timing but not byte boundaries. Turning edge timestamps into byte timestamps is what §10 and §11 are about.

### 4.3 What the consumer wants

The consumer wants: given RX ring index `i`, what was SysTick (or TIM2 CNT, in the new world) at the *start bit* of byte `i`?

If we have that, then:

- Wire-end of any byte is `BT[i] + 10 × bit_time` (or 11, depending on stop-bit handling).
- Snoop CRC accumulator can timestamp each byte for jitter analysis.
- Fast-Last slot timing can fire at `BT[i_last_master_byte] + 10·bit + RDT + slot_offset` with no backdating estimate.
- The framing FSM's IDLE/RXNE split disappears — every packet has per-byte timestamps regardless of baud.

This is the BT ring: byte-time ring, sized to match RX, indexed parallel.

---

## 5. Peripheral plan: the TX side

### 5.1 Timer-triggered TX, software-fired with bias

Today the fire path is: SysTick CMP → IRQ → highcode ISR dispatches through STATE/DISPATCH → write `DMA1.CH4.CR |= EN` and `GPIOx.BSHR = TX_EN_bit`. In the new design:

    TIM2 CNT counts up free-running.
    fire_tick is written to CCR3, biased by the MIN PFIC + ISR + USART pipeline latency (§5.3).
    When CNT == CCR3, CC3IF latches → TIM2 IRQ fires.
    Tiny .highcode ISR: enable DMA CH4 (TE is set permanently at init — see §5.3).

The fire trigger moves off SysTick onto a TIM2 compare event, but the actual register write stays in software — like the SysTick design, but with a different IRQ source and a leaner ISR body. The single-purpose ISR (just `dma::enable(CH4)`) doesn't have to dispatch through the STATE/DISPATCH indirection that today's `on_systick` does, so the body shrinks from ~1.5 µs to ~0.3 µs MIN.

Floor accounting:

    Today (SysTick CMP, mean):   ~1 µs PFIC + ~1.5 µs dispatched body + ~2 µs reg writes        ≈ 5 µs
    New (TIM2_CH3 IRQ, MIN):     ~0.7 µs PFIC + qingke_rt + ~0.15 µs dma::enable + ~0.15 µs AHB/DR ≈ 1 µs

The ~1 µs MIN floor sits comfortably under the 3.33 µs Fast-Last inter-slot cap at 3 Mbaud. Not as good as a hardware-DMA fire (which would have been ~100 ns) but enough to close the gap that today's design can't, and it keeps ADC on its existing DMA pump.

Bias compensation: measure CCR3-match → first-wire-bit latency on hardware and store its **1st-percentile lower bound** as `FIRE_BIAS_TICKS` (`firmware/ch32/src/measurements.rs`). CCR3 is then armed at `fire_tick − FIRE_BIAS_TICKS`. MIN calibration (not mean) guarantees the wire bit always lands at or after `fire_tick`: PFIC variance and USART bit-clock alignment only push it later. The tradeoff is ~15 ticks (~312 ns at 3M) of mean lateness on the wire — 9% of the 3.33 µs Fast-Last inter-slot cap — in exchange for a one-sided bound that lets §5.3 drop T_setup compensation entirely. The shape mirrors today's `PLAIN_ENTRY_TICKS` / `FAST_ENTRY_TICKS` constants, just MIN-calibrated against TIM2 instead of mean-calibrated against SysTick.

Jitter (variance around the mean) comes from the same place it does today: same-priority IRQs already running when CC3IF latches (USART1 IDLE/TC, DMA1_CH5 chain CRC stage-1). Those handlers don't typically run at fire-time — USART1 IDLE has finished by `wire_end + RDT`, USART1 TC hasn't fired yet for the reply we're about to start. The fire IRQ's worst-case contention is the classifier (§9 question — High or Low priority).

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
- **CCR3 raises CC3IF to trigger the fire ISR** — the inlined `handle_start` enables DMA CH4 `FIRE_BIAS_TICKS` after CCR3 match (§5.1). Bias is sized to the *minimum* observed path, not the mean.

Composition:

    CCR3 = fire_tick - FIRE_BIAS_TICKS       # ~46 ticks ≈ 960 ns at 3M
    CCR2 = fire_tick                          # no offset

No T_setup offset on CCR2. The ~2-tick OC pad latency means PC2 rises ~40 ns after `fire_tick`. Against the 333 ns bit-time at 3M, the buffer enables during the leading ~12% of the start bit; the bus pull-up (R17 = 10 kΩ on the schematic) holds DATA high during the brief hi-Z window; the receiver samples at mid-bit and reads the driven LOW. Invisible on the wire.

**Why no T_setup compensation?** Two reasons:

1. **MIN-calibrated `FIRE_BIAS_TICKS` guarantees the wire bit never appears before `fire_tick`** — jitter only ever pushes it later (§5.1). There's no "wire bit early, TX_EN late" race to protect against.
2. **Shifting CCR2 earlier risks bus contention** with the previous slave still releasing TX_EN. Firing TX_EN late by ≤2 ticks (~40 ns) nibbles at the start bit's leading edge — invisible at any baud we support. Firing TX_EN early can collide with a foreign driver. The asymmetric cost makes "late not early" the safe direction.

**TE is set permanently at USART init.** The half-duplex (HDSEL) USART tristates its own TX driver between shifts, and the external `SN74LVC2G241` gates the bus via TX_EN regardless of TE state. So leaving TE on doesn't drive the bus when idle — only when DMA writes to TDR. Per-fire the ISR only calls `dma::enable(CH4)`; see `firmware/ch32/src/providers/dxl_tx_scheduler.rs::handle_start`.

Both compare events arm at parse time. CCR2's edge is deterministic (hardware, ~20 ns timer resolution + ~2-tick pad lag). CCR3's edge has the same precision but the ISR-derived DMA-enable inherits PFIC + USART pipeline jitter. With MIN calibration the jitter is one-sided late — net wire-edge timing is `fire_tick + [0, ~30 ticks]` at 3M.

### 5.4 Wrap handling and the set-and-recheck pattern

TIM2's CNT is 16-bit. At PSC=0 it wraps every 1.365 ms (§7). RDT can be up to 502 µs ≈ 24 k ticks. So `fire_tick = (wire_end_cnt + rdt_ticks) mod 65536` can sit on either side of a wrap from `wire_end_cnt`:

    wire_end_cnt = 50000           ← CNT at wire-end
    rdt_ticks    = 24096           ← 502 µs at 48 MHz
    raw_sum      = 74096
    fire_tick    = 74096 mod 65536 = 8560      ← wraps

We write `CCR3 = 8560`. TIM2's compare logic raises CC3IF the moment CNT == 8560, which here is after the next wrap. ~16 k ticks of counting up to 65535 (≈ 340 µs) plus 8560 ticks into the next wrap (≈ 178 µs) ≈ 502 µs total. Exactly the RDT.

The catch is the same one SysTick CMP has (rx-timing doc §8.3): if we arm CCR3 to a value the counter *just passed* (because parse overran into the fire window, or wire-end fell unusually close to the deadline), the **next** CNT == CCR3 is one full wrap away — 1.365 ms. Far too late.

Set-and-recheck at arm time guards against it:

    ccr3_tick = (wire_end + rdt_ticks - FIRE_BIAS_TICKS) & 0xFFFF
    TIM2.CCR3 = ccr3_tick
    TIM2.CCR2 = (wire_end + rdt_ticks) & 0xFFFF
    enable CC3IE

    cnt_now   = TIM2.CNT
    remaining = (ccr3_tick - cnt_now) & 0xFFFF
    if remaining > MAX_REASONABLE_REMAINING:
        # we missed — fire by software now (same body as the ISR)
        tim2_ch2_force_active()        # drive PC2 high via CCMR2
        dma::enable(CH4)               # TE already on; DMA pumps first byte

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

Only one new DMA channel comes into play: CH7 for the ET ring. The other channels keep their current roles. By keeping fire on the software ISR path (§5.1), we don't need DMA1_CH1 for TX, so ADC's existing circular DMA pump is untouched. SysTick stays unused for DXL — free for telemetry or future use.

This is the lightest-touch DMA allocation the redesign supports. Heavier alternatives (TIM2_CH3 → DMA1_CH1 hardware-DMA fire, with ADC moved to SysTick-polled or TIM3-chained sampling) would shave another ~1-2 µs off the fire floor but at the cost of migrating ADC. The 2.5 µs floor from the software-fire path is already under the 3.33 µs Fast-Last cap at 3 Mbaud, so the heavier rework isn't load-bearing for the redesign's goals.

**ET channel priority.** Set `DMA_CFGR7.PL = 0b11` (Highest); ADC (CH1) and the USART channels stay at default. DMA arbitration is per-transfer and atomic — each ~80-ns peripheral-read + memory-write completes before the arbitrator picks the next request, so priority decides the *queued-request* race, not the in-flight transfer. When both ET and ADC have pending requests, ET wins; when ET arrives mid-ADC-transfer it waits ≤80 ns. ADC's sample-and-hold instant is anchored to TIM1 TRGO and is *not* jittered by DMA contention — only the latency from sample-complete to memory-write moves by ≤80 ns, which is invisible to a 40 kHz × 8-channel control loop (~3% of the inter-sample period, on a value that's already buffered in ADC_RDATAR). At 3 Mbaud worst case (1.5 M edges/sec) ET sees at most one 80-ns ADC contention per edge, well under the 32-tick (667 ns) intra-byte minimum — no edges lost. The bringup spike confirmed zero overcaptures at 3 M without the ADC pump running; a follow-up regression with ADC active is in §13 step 2.

---

## 7. Putting the timer in slave/master mode (or not)

Decisions on TIM2 configuration:

- **No OPM** (one-pulse mode). OPM would auto-stop the counter on update — we need it counting forever.
- **No slave-mode trigger.** SMS = 0b000. Free-running.
- **CC1S / CC2S / CC3S / CC4S** chosen per channel: CH4 = input capture (CC4S = 01), CH2 / CH3 = output compare (CC2S/CC3S = 00).
- **Prescaler.** PSC = 0 — CNT counts at HCLK / 1 = 48 MHz. One tick = 20.83 ns. ARR = 0xFFFF, period ≈ 1.365 ms. The IC side wants the fine resolution: at 3 Mbaud, bit_time = 333 ns = 16 ticks; the classifier's `[9·bit, 11·bit]` window = `[144, 176]` ticks. Higher prescalers cut the bit_time to 8 or fewer ticks, eating the tolerance budget. ET-ring consumers handle wrap via a cumulative byte counter (same trick as today's IDLE-stamp ring). TX-fire arming handles wrap via the set-and-recheck pattern (§5.4).
- **Capture filter on CH4** (ICF bits in CCMR2). Reject narrow glitches before they reach the ET ring. **Filter width must be shorter than one bit-time at the operating baud** — at 3M (`bit_time = 333 ns`), the naive setting `fSAMPLING = fCK_INT/8, N=8` (minimum pulse width ≈ 1.33 µs) would reject the start bit of a 0xFF byte as a glitch. Use `fSAMPLING = fCK_INT/2, N=2` (≈ 83 ns) at 3M; at low baud the filter can be heavier for noise immunity. Filter setting is computed alongside BRR and applied via the §9 TC budget path (precomputed at parse time, register write at TC tail). The Fast Last CRC layer treats glitch-induced spurious edges as packet faults already; cutting them at the capture filter is cheap insurance.
- **Polarity on CH4** (CC4P): falling edge only. Rising edges are start-bit ends — we want the start-bit *begin*, which is the 1→0 transition.

---

## 8. Memory rings

Three rings, three lifetimes. RX and BT are always 64 entries; ET sizing is a CPU-vs-memory tradeoff (see §8.4).

### 8.1 RX ring

64 bytes, circular, DMA1_CH5 from USART1.DR. Byte data only. *Shrinks* from today's 512 B because the parser drains continuously (at classifier-end in Plain mode; at each fold interval in Fast Last mode) instead of waiting for IDLE — ring size decouples from max packet size.

### 8.2 ET ring (new)

Circular, DMA1_CH7 from TIM2.CCR4. Captures the TIM2 CNT value at every falling edge on PC1.

Two viable sizings — see §8.4. We default to **128 entries × 16-bit = 256 B** (the Fast Last-safe sizing); the spike may downgrade to 64 entries (128 B) if measurements support it.

If the consumer falls behind (longer-than-expected ISR somewhere), ET overflow is detected by comparing NDTR against the consumer's last drained index. We count it as a fault and reset the classifier's anchor at the next IDLE. Wire-CRC still validates the packet — a missed BT entry isn't a wire-level error, it's a degraded timing signal.

### 8.3 BT ring (new)

64 entries × 16-bit = 128 B (with a separate 32-bit cumulative-byte counter for wrap disambiguation, same pattern as today's IDLE-stamp ring). Sized to match the RX ring so byte index `i` in RX maps to byte time `BT[i mod 64]`.

Each entry: `BT[i] = TIM2 CNT at the start bit of byte i in the RX ring`. Consumers read it by RX index, no walk required.

In Plain mode the classifier writes BT at HT/TC of DMA1_CH7. In Fast Last mode the Fast Last scheduler ISR writes BT (classifier ISR is disabled — §10.6). Consumers (parser, fire scheduler, Fast Last CRC, snoop log) read by index regardless of which ISR was the writer. All happen under the same High priority — no locking needed.

### 8.4 ET sizing: 128 (default) vs 64 (memory-tight)

The Fast Last scheduler ISR runs at 17-byte intervals (matching today's well-tuned schedule). Classifier is disabled between intervals, so the ET ring has to absorb everything received in one interval window without overwriting.

Worst-case edge density is 5 edges/byte (0x55-heavy stream):

    edges per 17-byte interval = 17 × 5 = 85 edges

EDGE_TS_BUF_LEN ≥ 85 → **128 is the safe choice.** Total ring memory: 64 (RX) + 256 (ET) + 128 (BT) = **448 B**.

If we want to drop ET to 64 (saves 128 B), we shorten the fold interval so peak edges per interval fits:

    edges per N-byte interval ≤ 64
    N ≤ 12 bytes  (12 × 5 = 60 edges)

That shifts CPU cost — more frequent folds means more IRQ entry/exit overhead. Quantitatively:

| Option | EDGE_TS_BUF_LEN | Memory | Interval | Cadence @ 3M | ISR overhead extra |
| --- | --- | --- | --- | --- | --- |
| **A (default)** | 128 | 448 B | 17 B | 56.7 µs | baseline |
| **B (tight)** | 64 | 320 B | 12 B | 40 µs | +~1 µs/100 µs ≈ +1% CPU |

The CPU delta between A and B is small (per-byte work dominates over per-ISR overhead). 128 B of SRAM on an 8 KB chip is also modest. Default to **A**; revisit if SRAM gets tight elsewhere.

The Plain-mode constraints (parser keep-up at classifier cadence) are unaffected either way — Plain mode doesn't use the Fast Last scheduler ISR, and its classifier-end drain handles peak rates with margin at EDGE_TS_BUF_LEN=64 already.

---

## 9. ISRs and priorities

Same two priority levels (V006 PFIC has nothing more). DXL-related IRQs stay at High alongside today's; only ADC sits at Low.

| Priority | IRQ | Body | Where |
| --- | --- | --- | --- |
| High | USART1 | IDLE (parser kick) + TC (release bus) + RX errors | `.highcode` |
| High | DMA1_CH7 HT/TC | Classifier walk (ET → BT) + parser drain. Folds Fast Last CRC inline when armed. *Paused during Fast Last at ≥ 1M only* (§10.6.4) | `.highcode` |
| High | TIM2 (CC1IE) | Fast Last scheduler (at ≥ 1M only): classifier + parser drain + CRC fold over 15-byte interval, plus post-fire fold anchor | `.highcode` |
| High | TIM2 (CC3IE) | Fire TX: `dma::enable(CH4)`. Lean body — no Fast Last CRC work (M3 design, preserved through M6) | `.highcode` |
| Low | DMA1_CH1 | ADC kernel pump (unchanged) | flash |

All wire-side IRQs at High serialize cleanly via same-priority no-preemption (existing trick from `dxl-rx-timing.md` §8.4). The structural protection for the Fast-Last fire deadline isn't priority — it's **scheduling**:

- During Plain replies, classifier may fire whenever HT/TC trips. Plain replies have no inter-slot jitter cap, so a ~5 µs classifier walk delay before fire is harmless (RDT ≥ 250 µs typical).
- During Fast Last replies, the classifier ISR is disabled at the first fold entry. From then until USART1 TC, classifier doesn't compete for the High slot — Fast Last mode is entirely owned by the Fast Last scheduler ISR cadence (§10.6) and the fire ISR. The last fold is scheduled to exit before `fire_tick - FIRE_BIAS`, so fire never waits behind the fold scheduler either.

**DMA1_CH5 HT/TC interrupts are no longer enabled.** The old chain-CRC stage-1 ISR (which walked RX bytes into the snoop CRC at DMA half/full events) is structurally replaced by the Fast Last scheduler ISR (§10.6). The CH5 channel itself still runs — it's USART1's circular byte-DMA pump — but no IRQ is wired to it. The RXNEIE composer from `dxl-rx-timing.md` §7.3 also disappears: framing has no RXNE owner anymore, and Fast Last CRC doesn't need one.

**Bringup note — INESTEN.** Per RM §6.5.2.21, each IRQ's `PFIC_IPRIORx` field uses bit 7 as the preempt-priority bit (bit 6 is sub-priority within a group; bits 5:0 are reserved), and `INTSYSCR.INESTEN = 0` at reset disables preemption entirely. Default IPRIOR = 0 → bit 7 = 0 → every IRQ defaults to High; that's fine for the DXL-side IRQs but means we explicitly write bit 7 = 1 for DMA1_CH1 (ADC) to put it at Low. Bringup must also flip INESTEN to 1 for that distinction to actually preempt.

**The fire moves from SysTick to TIM2 but stays in software.** TIM2_CH3 CC3IF latches on compare match; the TIM2 IRQ runs a single-purpose body that writes `USART1.CTLR1 \|= TE` and enables DMA CH4. PFIC entry stays in the path; CCR3 is biased earlier by `FIRE_BIAS_TICKS` so the wire bit lands on time (§5.1).

**TX_EN is hardware-driven.** CCR2 → PC2 via OC mode — no ISR involvement, edge lands at the exact compare tick.

**Why all DXL-side IRQs share High priority.** Three invariants depend on same-priority serialization:

1. **Fold-and-fire serialize through the same vector.** CC1 and CC3 share TIM2's IRQ vector. The dispatcher checks CC1IF first, then CC3IF — both in one vector entry. If CC1's body is running when CC3IF latches (which is the normal case at 3M GUARD=1 — site 1 spins past `fire_deadline_tick`, see §10.6.2 / `dxl-fast-chain-crc-walkloop.md` §4 recursive-fire mode), CC3 doesn't preempt; the dispatcher's second `if` catches CC3IF on CC1's exit. Wire-bit jitter at 3M is ~3 µs late vs `fire_deadline_tick`, under one char_time, within pirate's IDLE tolerance. Dispatcher order is load-bearing: CC1 before CC3.
2. **TC release before master's next start bit.** TX_EN drop in the TC ISR must land before any next request's start edge to avoid bus driver contention. TC at High keeps that latency bounded.
3. **Fold interval cadence.** §10.6 budgets ~54% CPU per interval; CC1 at High keeps ADC and other Low work from eating into the headroom.

The intuitive alternatives — "TX at High, RX at Low" or "CC3 alone at High" — break all three: CC3 would preempt CC1 mid-fold; TC at Low would queue behind ADC/classifier under fast back-to-back master traffic; CC1 at Low could miss its cadence under load. The remaining concern — TC tail of reply N−1 delaying CC3 of reply N when master's RDT is tight — is addressed at root by the TC tail budget below, not by priority shuffling.

**TC tail budget.** `apply_pending_after_tc` mutates clock/baud and must run after our reply's wire activity ends, so it can't move earlier. But the *computation* of new values (fine-trim recompute, BRR precompute, capture filter setting per §7) moves to **parse time**: the parser already sees the register write that requested the change, so it stores the *computed* result. TC tail then does only the cheap atomic register writes — bounded under 1 µs. With TC tail under one byte-time at the highest expected baud (<2 µs is comfortable), the worst-case "next CC3 latches mid-TC-tail" delay vanishes for any realistic RDT. Out of normal operating envelope (master RDT_min approaching TC tail duration), residual contention surfaces as fire-late telemetry — documented risk, not engineered solution.

**Pre-fire fold scheduling on TIM2_CH1** (Fast Last replies only) belongs to the fire family — see §10.6.

**SysTick CMP is unused for DXL.** Free for telemetry or other tasks.

**USART1's role shrinks dramatically.** It still handles:

- IDLE as a parser-kick signal (drain RX up to current frontier, packet is done) — *no timing value is derived from it*; wire-end comes from BT,
- TC for "reply finished, release the bus" (drops TX_EN via timer CCMR2 force-inactive + clears DMA CH4 + applies pending baud/RDT),
- RX errors for telemetry.

The framing FSM (IDLE vs RXNE), the `decide(brr, rdt)` rule, the `pipeline_margin_us` tuning knob, the `DXL_CHAR_TIME_TICKS` backdate constant, the IDLE-stamp queue, and the RXNE single-cell snapshot all **go away**. Every byte gets a per-byte timestamp from BT, regardless of baud — there's no high-vs-low-baud strategy split because there's no estimation involved at any baud.

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

Per HT/TC the classifier walks up to 32 ET entries (EDGE_TS_BUF_LEN/2). Per entry: 1 load (ET), 2 comparisons (window low, window high), 1 store (BT), 1 anchor update. ~8 cycles per entry × 32 = ~256 cycles per walk = ~5.3 µs at 48 MHz.

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

`firmware/ch32/src/rx_classifier.rs` (new). Body in `.highcode`. Two entry points: `on_et_ht()` walks indices `0..EDGE_TS_BUF_LEN/2`; `on_et_tc()` walks `EDGE_TS_BUF_LEN/2..EDGE_TS_BUF_LEN`. State is a single `Anchor { tick: u32, byte_idx: u16 }` updated in-place. Reset on USART1 IDLE (packet boundary): anchor invalid until the next ET entry, which is the next packet's first start bit.

### 10.5 Coordinating with the parser

The classifier produces BT entries at ET HT/TC; USART1's RX DMA produces RX bytes at each stop bit. The two streams have different cadences:

- **`RX[i]`** lands in memory ~10 bit-times after byte i's start edge (the byte-shift time plus DMA latency) — fast and immediate.
- **`BT[i]`** lands in memory when the classifier walks the ET entry for byte i's start edge, which happens at the next ET HT/TC. In the worst case BT[i] lags RX[i] by up to half a ET ring's worth of edges.

**The parser's invariant:** consume bytes only up to `min(rx_write_pos, bt_write_pos)`. Past that point, RX data may exist without a corresponding BT tick — partial state from the parser's perspective.

This makes the parser stateful: an RX HT/TC (or classifier HT/TC) that lands mid-packet drains up to the BT frontier, parser yields with in-progress header/length/payload state preserved, and resumes on the next event. The state to keep is small:

    parsed_idx: u16         # bytes consumed so far (cumulative, with wrap counter)
    decode_state: ...       # header / length / payload / CRC stage
    expected_end: Option<u16>  # set once header.length is decoded

**Trigger sites:** two are sufficient.

- **Classifier HT/TC (DMA1_CH7).** The natural drain point — the classifier just walked ET → BT, so we drain RX up to the new BT frontier in the tail of the same ISR. Cadence is tied to EDGE_TS_BUF_LEN by design.
- **USART1 IDLE.** Backstop for small packets that don't fill enough of ET to trip classifier HT. A 14-byte all-0xFF reply generates only 14 edges (below EDGE_TS_BUF_LEN/2 = 32), so the classifier never fires on its own. IDLE catches it. **IDLE is a *signal* here, not a timing source** — it tells the parser "drain, packet done," and the wire-end tick still comes from `BT[last_byte] + 10·bit_time`, never from an IDLE-derived backdate.

No RX HT/TC parser trigger, no SysTick parser trigger, no decimated counter. RX HT/TC stays disabled for parser purposes; the chain-CRC stage-1 snoop walk is a separate consumer that may keep its own RX HT/TC enable.

**Packet boundaries are a parser concept, not a classifier concept.** DXL 2.0 doesn't require an inter-packet idle gap. If the master sends back-to-back requests without silence between them, USART1 IDLE never fires at the seam and the classifier's `anchor` is never reset — `byte_idx` keeps counting across the boundary. RX DMA likewise writes continuously, no boundary signal.

The parser is the only thing that knows packets exist. On detecting `FF FF FD 00` at the current header position, it stamps `packet_start_rx_idx` and `packet_start_bt_idx` into its own state. All BT consumers that want "byte k of *this* packet" walk forward from `packet_start_bt_idx + k`, never assuming "BT[0] is byte 0 of the current packet." Drift estimation (§10.7.1) naturally folds the seam pair (last byte of packet N → first byte of packet N+1, separated by exactly 10·bit_time) into its running average as a bonus tight-window sample.

**Wire-end derivation** at request_complete:

    wire_end_tick = BT[parsed_end_idx - 1] + 10 × bit_time
    fire_tick     = (wire_end_tick + RDT_ticks - FIRE_BIAS) & 0xFFFF
    TIM2.CCR3     = fire_tick
    # ... + set-and-recheck guard from §5.4

No backdate constant. No char-time math. The whole framing-mode FSM from `dxl-rx-timing.md` §7 evaporates.

**Plain-mode trigger sites:** classifier HT/TC + IDLE (above). The classifier walk drains the parser at its tail — no CRC fold, no Fast Last state.

**Fast Last mode trigger sites:** the Fast Last scheduler ISR (§10.6) replaces both. At first fold entry, classifier HT/TC interrupts are masked; from then until USART1 TC, the Fast Last scheduler ISR owns the full classifier + parser + CRC fold cadence at fixed 17-byte intervals (matching today's well-tuned schedule). IDLE doesn't run during a Fast Last reply window (no inter-slot wire idle in Fast Sync/Bulk by design).

### 10.6 Fast Last CRC fold

Fast Last CRC accumulates the predecessor's reply bytes into a running CRC, which we use to patch the trailing CRC slot of our own reply so the wire bytes form one continuous CRC stream covering header + every predecessor's bytes + our own bytes. ("Fast Last" is our internal shorthand for the Fast Sync Read / Fast Bulk Read reply at `SlotPosition::Last`; not a Robotis spec term, but earns its weight by composing the existing `SlotPosition::Last` enum + "Fast" protocol family.) The mechanism factors into **state + trigger**:

- **State** is composite-side (`DxlUart::fast_last_crc`): `armed`, `bulk_crc`, `snoop_head`, `bytes_walked`, `expected_n_pred`. One method, `fold(byte, parsed_idx, &mut tx)`, called once per RX byte from `DxlRx::drain_parser`. No-op when not armed. When `bytes_walked >= expected_n_pred`, fold finalizes the CRC over our own reply bytes, calls `tx.patch_crc`, and sets `armed = false` — idempotent across redundant trigger sites.
- **Trigger** picks which ISR(s) drive `drain_parser` (and therefore `fold`) during the predecessor window. **The trigger varies with baud.**

#### 10.6.1 Baud-dispatched trigger paths

| Baud | Trigger sites during Fast Last window | Finalize lands at |
| --- | --- | --- |
| ≥ 1M | TIM2 CC1 fold body (grid walks + busy-wait + post-fire anchor). DMA1_CH7 HT/TC paused. | Post-fire CC1 body, ≈ t_prior_end + GUARD_LATCH_TICKS (~1 µs). |
| < 1M | DMA1_CH7 HT/TC (when it fires) + USART1 IDLE. Nothing paused. | USART1 IDLE body, ≈ t_prior_end + 1 byte_time. |

The threshold is 1M because three constraints converge:
- **Ring-fill vs predecessor length.** At ≥ 1M the predecessor often doesn't fill the ET ring enough to trigger HT (3M n_pred=14 lands 14 ring entries; HT fires at ~32). CC1 grid walks become necessary to drain on a tighter cadence.
- **u16 fit on TIM2.** `interval_ticks = 15 × byte_time` stays comfortably below half u16 only at ≥ 1M. Below that the CC1 grid math runs out of register width before low-baud is even useful.
- **IDLE-as-finalize budget.** At < 1M USART1 IDLE fires comfortably before fire_deadline (IDLE detection ≈ 1 byte_time after the predecessor's last byte; fire_deadline is at least rdt + slot_offset_bytes ≫ idle_window away). At ≥ 1M, with close coalesce, fire_deadline can land before IDLE fires — finalize MUST happen at post-fire CC1 instead of IDLE.

#### 10.6.2 High-baud path (≥ 1M, TIM2_CH1)

The Fast Last scheduler runs at fixed-byte intervals on TIM2_CH1, owning **all** RX-side work during the predecessor reception window. Structurally the same scheduling pattern as the canonical SysTick-driven walk-loop + split-fire design (b03dd355 / `docs/dxl-fast-chain-crc-walkloop.md`), moved to TIM2 CC1.

**Arm time** (Fast Last reply scheduled, baud ≥ 1M):

    fast_last_crc.arm(snoop_head = parsed_idx_now, expected_n_pred)
    pause DMA1_CH7 HT/TC                                # §10.6.4 classifier merge
    walk_deadline       = t_prior_end - GUARD × byte_time
    last_catchup_tick   = fire_deadline_tick - min(interval, t_prior_duration) - GUARD × byte_time
    current_fold_anchor = step_back(last_catchup_tick, interval, t_prior_start)
    post_fire_fold_anchor = t_prior_end + GUARD_LATCH_TICKS_TICKS
    CCR1 = current_fold_anchor - FAST_LAST_ENTRY_TICKS    # every CMP back-dated, not just last
    # CCR2 + CCR3 already armed by DxlTxScheduler (§5.3); CC1 added here.

`interval_ticks = BYTES_PER_INTERVAL × byte_time`, with `BYTES_PER_INTERVAL = 15` (carries from b03dd355).

Every grid CMP is back-dated by `FAST_LAST_ENTRY_TICKS` so the body's fold-start lands on the formula's wall-clock anchor instead of `anchor + ISR_entry_latency`. The legacy implementation back-dated only the last anchor; intermediates re-armed via `now + interval` and drifted by `PFIC_ENTRY + body_work` per body. Tracking `current_fold_anchor` in state and advancing by fixed `+ interval` keeps the grid rigid regardless of any one body's overhead. See `dxl-fast-chain-crc-walkloop.md` §1.1 for the entry-latency derivation.

**At every CC1 IRQ (intermediate):**

    walker():
        rx.walk_classifier()                 # ET → BT, advance anchor
        rx.drain_parser(|byte, idx| fast_last_crc.fold(byte, idx, &mut tx))
    current_fold_anchor += interval_ticks    # grid advance, drift-free
    CCR1 = current_fold_anchor - FAST_LAST_ENTRY_TICKS

**At the last CC1 IRQ (anchor = last_catchup_tick):**

    walker()
    loop:                                    # site 1 busy-wait
        walker()
        if fast_last_crc.bytes_walked >= expected_n_pred - GUARD: break
        if now >= walk_deadline: break
    CCR1 = post_fire_fold_anchor - FAST_LAST_ENTRY_TICKS  # re-arm for post-fire fold

The busy-wait is uncontested — DMA1_CH7 HT/TC is masked (§10.6.4), so CC1 is the sole High consumer in the Fast Last window. At 3M GUARD=1 the loop exits via `walk_deadline` (recursive-fire mode — `walk_deadline > fire_deadline_tick` by ~2.8 µs at GUARD=1; see `dxl-fast-chain-crc-walkloop.md` §4). CC3IF latches during the spin; the TIM2 vector's dispatcher catches it on CC1's exit and runs the fire body. Wire-bit jitter at 3M is ~3 µs late vs `fire_deadline_tick`, within pirate's < 1 char_time tolerance.

**At CC3 (fire body — unchanged from M3, lean):**

    enable DMA CH4

No Fast Last CRC work on the fire path. The deterministic wire-fire property survives.

**At the post-fire CC1 IRQ (anchor = post_fire_fold_anchor):**

    walker()                                 # GUARD byte is in NDTR by now
    # fold's finalize branch fires inside drain_parser when
    # bytes_walked hits expected_n_pred → patch_crc → armed = false
    self.scheduler.cancel()                  # CC1IE off

Replaces the legacy CC3 in-body post-fire spin (site 2). CC3 stays minimal; CC1's third firing folds the residual. `GUARD_LATCH_TICKS_TICKS` is sized to cover (a) NDTR latch latency for the GUARD byte's stop bit, and (b) CC3 body's `dma::enable` return path so CC1 doesn't pend while CC3 is still running.

**At USART1 TC (reply complete):**

    fast_last_crc.disarm()                       # idempotent — fold already disarmed
    fast_last.cancel()                   # idempotent — post-fire already cancelled
    rx.resume_edge_advance_irq()             # un-pause DMA1_CH7 HT/TC

#### 10.6.3 Low-baud path (< 1M, HT/TC + IDLE inline)

`fast_last_crc.arm(snoop_head, expected_n_pred)` runs as usual; **fast_last is NOT armed** and DMA1_CH7 HT/TC stays **un-paused**. The existing classifier path now folds CRC as a side-pass:

    DMA1_CH7 HT/TC → on_rx_edge_advance → rx.drain_parser(|byte, idx| fast_last_crc.fold(...))
    USART1 IDLE    → on_rx_idle         → rx.drain_parser(|byte, idx| fast_last_crc.fold(...))

At < 1M the predecessor wire window is long enough (500k n_pred=14: 280 µs of wire time) that `n_pred` × ~0.5 µs CRC update = ~7 µs of inline fold work in the IDLE body — trivial. HT/TC may or may not fire depending on whether the predecessor fills the ring half; either way, the fold completes inside IDLE.

No new ISR, no timer, no busy-wait, no grid math, no §10.6.4 classifier merge. The trigger is the existing RX path; the fold callback is the same `fast_last_crc.fold` as the high-baud path.

#### 10.6.4 Classifier+parser merge (high-baud only)

At ≥ 1M the high-baud CC1 path masks DMA1_CH7 HT/TC at first-fold entry. From then until USART1 TC, classifier doesn't compete for the High slot — CC1 is the sole HIGH consumer. This is what makes the site 1 busy-wait deterministic: deadline budget at the last anchor is uncontested.

**Defense against the "tiny fold misses classifier disable" case.** If classifier HT/TC happens to latch right before CC1 fires (because classifier was about to walk on its own cadence), the fold body's pause write happens *after* the latch — classifier IRQ is still pending and would run after CC1 exits. Mitigation in the pause path: after masking HT/TC, also clear pending on DMA1_CH7 (`PFIC.IPRR.CH7 = 1`). CC1's first body has already done the classifier's job inline, so dropping the pending bit is correct — no work is lost.

At < 1M this isn't a concern because no pause happens — HT/TC fires whenever and the fold runs in either trigger.

#### 10.6.5 CCR1 wrap guard

`CCR1 += interval_ticks` re-arm is subject to the same 16-bit wrap hazard as CCR3 (§5.4). If CC1 ISR runs late (preempted by a slow same-priority handler — DMA1_CH7 in the pause window before the mask write, or USART1 IDLE landing concurrent), `CCR1 + interval_ticks` can fall behind `CNT` after modular subtraction — the next match wraps a full 1.365 ms before firing. Set-and-recheck applies: after writing CCR1, compare `(CCR1 − CNT) & 0xFFFF` against the wrap-guard threshold. If exceeded, manually invoke the fold body now — walk + drain over whatever's in ET — and re-arm CCR1 forward from the new CNT.

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

Plain replies skip both the fast_last_crc arm AND the CC1 fold scheduler. Classifier HT/TC stays armed; parser drains at classifier-end as in §10.5. CC1 arming is conditional on `SlotPosition::Last` at send_slot time; `fast_last_crc.fold` is a no-op when not armed, so the HT/TC + IDLE callbacks remain safe to invoke unconditionally.

### 10.7 HSI drift and the bit_time assumption

The slave's HCLK comes from HSI (internal RC), the master's baud comes from HSE (crystal). The classifier's window math uses `bit_time_spec = HCLK_slave / baud` — what the slave *thinks* a bit-time is — but the actual on-wire bit length is set by the master. If slave HSI runs at fraction `(1 + D)` of master's reference, slave ticks per master bit = `bit_time_spec × (1 + D)`, and the inter-byte spacing measured into the ET ring is `10 × bit_time_spec × (1 + D)`.

**Static tolerance: the window survives ±10% drift.** Two failure modes:

- **Upper-window failure** (next start lands past `11 × bit_time_spec`, treated as gap → spurious re-anchor): triggers when `10·(1+D) > 11`, i.e., `D > 10%`.
- **Lower-window failure** (last data-edge at master bit 7 bleeds up into the start window): triggers when `8·(1+D) > 9`, i.e., `D > 12.5%`.

V006 HSI typical excursion at 25 °C post-factory-cal is ≤ 0.5%; full -40..125 °C without trim ~3%. After cold-start trim and periodic re-tune, drift sits well under 1%. The window has ~10× margin over realistic worst-case drift — **the spec-bit_time math is correct as-is, no within-packet correction needed.**

**Per-byte error accumulation: zero.** §10.1's `anchor = t` (not `anchor += 10·bit_time`) means each window is re-centered on the actually-measured previous start. A packet's worth of drift errors don't compound: the last byte's window has the same tolerance as the first.

#### 10.7.1 BT ring as the drift signal

The same BT entries that drive the parser and fire scheduler give us a direct drift measurement. For any back-to-back byte pair (matched window, no gap re-anchor):

    observed_bit_time_ticks = (BT[i+1] - BT[i]) / 10
    drift_ppm = (observed - bit_time_spec) / bit_time_spec × 1e6

Inter-byte interval at 3M ≈ 160 ticks; capture noise σ ≈ 1 LSB per BT entry. Over 50 back-to-back pairs in a single packet, drift estimate σ ≈ √2 / 160 / √50 ≈ 1250 ppm — borderline useful per-packet. Averaging across ~10 packets (a few seconds of normal traffic) tightens to ~400 ppm; over a minute, sub-100 ppm.

Filtering rules to keep the signal clean:
- Only count pairs where the window match was "tight" (no re-anchor in between), so we know we measured a true 10-bit interval, not a gap.
- Discard pairs that span a ET overrun (anchor invalidated).
- Use the same baud as the master is currently using — pause measurement during baud-change windows.

Replaces today's IDLE-based drift inference (see [[no-idle-timing]]): IDLE-derived rate measurement folded a full char-time of latency into every sample. BT-ring measurement is per-byte and timing-source-clean.

#### 10.7.2 Feedback into HSITRIM

V006's HSI tuning is `RCC_CTLR.HSITRIM[4:0]` — 5 bits centered at 16, ~2500 ppm per step (60 kHz/step at 24 MHz nominal, per RM §3.3.2). One step shifts the classifier window by:

    bit_time shift   = 16 ticks × 0.0025 = 0.04 ticks/bit
    window edge shift = 0.4 ticks at the 11·bit edge

Negligible against the ±10% (16-tick) window. So we can freely retune HSITRIM at packet boundaries without risking classifier desync.

Coarse trim aims drift inside ±1250 ppm (half a step). The existing `DXL_CLOCK_TRIM_PENDING` write at `apply_pending_after_tc` (`irq.rs:203`) already plumbs this — the change is just the input signal: BT-averaged drift estimate replaces today's IDLE-derived one.

Sub-step residual goes to the existing software fine-trim: `DXL_CLOCK_FINE_TRIM_PENDING` → `recompute_fire_advance_fine_ticks` (`statics.rs`). In the new design, fine-trim adjusts the `BT[last_byte] + 10·bit_time + RDT_ticks` math by `residual_ppm × elapsed_ticks_since_last_BT` — a few-tick correction over the wire-end-to-fire window. Same machinery, BT-sourced input.

#### 10.7.3 Optional: adaptive bit_time within a packet

If glitch margin under drift turns out to be tight (it shouldn't, but for completeness), the classifier can run an in-packet low-pass on observed bit-times:

    on every successful window match:
        observed = t - anchor
        bit_time_est = α × (observed / 10) + (1 - α) × bit_time_est
        # use bit_time_est for the next window

With α = 1/8 the estimator converges in ~5-6 bytes. After convergence the window re-centers on the *actual* drifted bit-time, giving extra glitch margin in both directions.

Cost: a couple of cycles per match, plus a per-packet seed-from-spec at the first byte of each packet (re-armed at IDLE / `cancel`). Not load-bearing for v1 — skip and revisit if the spike shows borderline behavior at extreme drift or temperature.

---

## 11. Alternative: LUT walker with resync detector (secondary, more flawed)

Considered, rejected for now, documented for completeness. May come back if §10's CPU cost turns out to bite under some workload.

### 11.1 Idea

The number of falling edges in a UART byte is a fixed function of the byte's data value. A 256-entry lookup table gives `edge_count[byte_data] ∈ [1, 5]`. If we know `BT[i]` and `RX[i]`, then:

    expected_ET_cursor_advance = edge_count[RX[i]]
    BT[i+1] = ET[et_cursor + edge_count[RX[i]]]

The walker reads RX bytes in order, advances the ET cursor by the LUT value, takes the next ET entry as the next BT. ~5 cycles per byte instead of ~8 cycles per edge × ~3 edges/byte = ~24 cycles per byte. Roughly 5x faster.

### 11.2 Why it's flawed

The LUT lookup uses **byte data** to predict **edge structure**. Anything that corrupts the alignment between edges and bytes corrupts every subsequent BT entry until the next resync.

Failure modes:

- **Phantom edge** (glitch adds an unaccounted edge): LUT predicts 4 edges for current byte, reality is 5. Walker advances 4, lands on the glitch instead of the next byte's start bit. All BT entries past this point are shifted by 1 edge.
- **Missed edge** (ET overrun, capture-filter swallow): LUT predicts 5, only 4 captured. Walker advances 5, consuming one edge from the next byte's frame. Cascade.
- **Byte data corruption** (USART noise → wrong byte): LUT lookup uses the corrupted byte → wrong edge count → cascade.

### 11.3 The resync detector

To bound the damage we'd add a sanity check: after advancing the cursor by `edge_count[RX[i]]`, verify the next ET entry is within `[BT[i] + 9·bit_time, BT[i] + 11·bit_time]`. If yes, take it as `BT[i+1]`. If no, mark BT for the rest of the packet as invalid and fall back to packet-boundary timestamps.

Detection coverage is good but not perfect. The check catches:

- All ET-overrun desyncs (cursor under-advance lands on a real edge inside the next byte at offset 16–80 ticks past predicted; window catches it).
- Most phantom-edge desyncs (cursor over-advance lands data-dependently; usually outside window).

The check misses:

- **Phantom edge landing exactly in the next-start window.** Roughly 4/160 ≈ 2.5% probability per glitch event at 3 Mbaud. Walker silently desyncs and consumers act on corrupted BT.
- **Multi-byte cascade resync.** Once we detect, we can only resync at IDLE — the walker doesn't know how many edges to skip to realign mid-packet.

### 11.4 The actual tradeoff

| | Window classifier (§10) | LUT walker + resync |
| --- | --- | --- |
| Self-healing | Within 1 byte | Detect-only, requires IDLE to resync |
| Silent corruption rate | Effectively zero | ~2.5% per glitch event escapes |
| CPU during peak burst | ~25% | ~5% |
| Memory | ET 256 B + BT | Same + 256 B LUT |
| Algorithm complexity | One path | Two paths + resync state |

The CPU savings aren't load-bearing — peak burst at 25% leaves plenty of margin for Fast Last CRC and USART1 work running at the same High priority. The robustness story is what matters for a transport whose entire point is precise per-byte timing.

If the spike of §10 shows the V006 can't actually meet 25% peak (e.g., because chain CRC stage-1 is eating more cycles than expected), we can revisit. Hybrid would be possible: LUT walker in steady state with classifier fallback on anomaly. Adds complexity that isn't free either.

For now: build the window classifier, measure, decide.

---

## 12. ISR reshuffle summary

What changes in the existing firmware:

| File | Today | After |
| --- | --- | --- |
| `dxl/state.rs` | `ReplyState { Idle, Plain, Chain { … } }` w/ FSM + dispatch table | Same shape; fire dispatch moves to a TIM2 IRQ but the FSM stays |
| `dxl/isr.rs` | `body_plain_fire`, `body_fast_last`, `body_chain_fast_fire` | Same bodies, called from TIM2 IRQ instead of SysTick; leaner because no STATE/DISPATCH indirection on the TIM2 hot path |
| `irq.rs` | `on_systick_match` dispatches via `STATE`/`DISPATCH` | `on_systick_match` unused (free for telemetry); new `on_tim2` handler does the fire |
| `irq.rs` | `on_usart1_idle` backdates and stamps `IdleAnchor` | `on_usart1_idle` kicks the parser; no tick math, no backdate constant |
| `rx_classifier.rs` | n/a | New: HT/TC walker for ET → BT, calls parser at walk-tail |
| `dxl/parser.rs` (or wherever the DXL decoder lives) | parses on main-loop poll from a 512 B ring | stateful decoder draining 64 B ring at every classifier walk; resumable mid-packet |
| `framing.rs` (today's IDLE-stamp queue + RXNE snapshot + framing FSM) | ~hundreds of LoC | **deleted** — no consumers after BT arrives |
| `board/bringup.rs` | TIM2 unconfigured | TIM2 init: free-running, CH4 IC, CH2 OC (hardware TX_EN), CH3 OC (fire IRQ), DMA CH7 |
| `idle_anchor.rs` | EXTI snoop instrumentation (9 atomics) | All EXTI snoop fields removed; framing relies on BT, not snoop |

The EXTI-pin instrumentation built on `firmware/dxl-2.0-transport` (the current branch) becomes obsolete — its job is fully subsumed by TIM2 IC capture, which is per-edge instead of first-edge-only. We can land one more bench capture on the EXTI path for diff context, then strip it in the migration.

---

## 13. Spike plan

Recommended order — each step independently verifiable on the dev board.

0. **PFIC bringup.** Write `INTSYSCR.INESTEN = 1` and set bit 7 of `PFIC_IPRIORx` for DMA1_CH1 (ADC) to put it at Low. Verify with a read-back; the rest default to High which matches the desired layout.
1. **PC1 dual-tap.** Configure AFIO for USART1_RX + TIM2_CH4 simultaneously. Verify USART RX still framed correctly (no regression) and TIM2 CCR4 captures on every PC1 falling edge. Use a scope probe on PC1 + a bench-side print of captured CCR4 values to compare.
2. **ET ring + classifier + stateful parser.** Wire DMA1_CH7 to CCR4 → 64-entry ET ring (circular). Implement `rx_classifier.rs` (HT/TC walker → BT). Shrink RX ring to 64 B. Convert the DXL parser to stateful drain-up-to-`min(rx,bt)`. Drive the parser from classifier-end + IDLE only (no RX HT/TC, no SysTick, no decimated counter). Verify at 115200, 1M, 3M that all packets are decoded correctly and BT entries land within ±1 tick of where the wire said they should.
3. **Software TX fire via TIM2_CH3 IRQ.** Wire CC3IE at High; implement the leaner ISR body (TE write + DMA CH4 enable). Measure mean CCR3-match → first-wire-bit latency, store as `FIRE_BIAS_TICKS`. Target: < 3 µs floor at 3M. Verify the §5.4 set-and-recheck path by forcing a small-RDT case (RDT=0 at low baud) that exercises the manual-fire branch.
4. **TX_EN via TIM2_CH2.** Wire CH2 → PC2 in OC mode (set-active-on-match). With `CCR2 = fire_tick` (no offset), verify TX_EN rises ~2 ticks after `fire_tick` on a scope (hardware OC pad lag) and falls correctly at TC.
5. **Fast Last scheduler on TIM2_CH1.** Move today's SysTick-driven `body_fast_last` to TIM2 CC1 at 17-byte intervals (default; or 12-byte if going for §8.4 Option B). At first CC1 entry: mask DMA1_CH7 HT/TC + clear its pending bit. Each CC1 walks classifier → parser → CRC fold inline. Last interval busy-waits walk_deadline. Re-enable DMA1_CH7 HT/TC at USART1 TC. Drop the old SysTick scheduling and the DMA1_CH5 stage-1 ISR. Validate at 3M that snoop CRC matches the wire for all-foreign Fast Sync Read traffic.
6. **Fast Last scheduler ISR cost measurement.** Profile real-world worst-case classifier-walk + parser-drain + CRC-fold runtime per interval. Confirm ≤ 75% of interval cadence at 3M (leaves headroom for USART1 errors, TIM2 fire, ADC preempt). If margin is tight, switch from 17-byte to 12-byte interval and shrink EDGE_TS_BUF_LEN to 64 (§8.4 Option B).
7. **Integration: Fast Last-slave at 3 Mbaud.** Validate inter-slot coalesce gap — target: under one byte time. This is the test the SysTick design fails.
8. **Strip the old timing paths.** Once §7 passes, remove SysTick-CMP scheduling, IDLE-stamp queue, RXNE single-cell snapshot, framing-mode FSM, RXNEIE composer, `DXL_CHAR_TIME_TICKS` backdate constant, DMA1_CH5 chain-CRC stage-1 ISR, and EXTI snoop instrumentation. Anything that computes a tick from IDLE goes.

Each step is a self-contained commit per the project's "one reviewable unit" rule.

---

## 14. Open questions for the spike

- **AFIO dual-remap stability.** F1 documentation is explicit about peripheral-side input dual-tap working; V006 RM is less so. Step 1 confirms by direct measurement.
- **Capture filter setting per baud (§7).** Filter width must be < 1 bit-time at the operating baud; default `(fCK_INT/2, N=2)` for 3M. Compute alongside BRR; relax (heavier filter) at lower baud if bench shows noise immunity needs it.
- **Fire-floor MIN measurement.** Measure CCR3-match → first-wire-bit at 3M on bench across many fires. Lock `FIRE_BIAS_TICKS` to the **1st-percentile lower bound** (not the mean) so the wire bit always lands at or after `fire_tick` — see §5.1 / §5.3. Placeholder estimate is ~46 ticks (~960 ns); confirm on hardware.
- **ET sizing + interval (§8.4 Option A vs B).** Default plan is 17-byte interval / EDGE_TS_BUF_LEN=128. Option B (12-byte interval / EDGE_TS_BUF_LEN=64) saves 128 B of SRAM at ~1% extra CPU. Decide based on memory pressure measured during integration.
- **Fast Last CRC fold cost.** §10.6 estimates ~10 cyc/byte for CRC16 update. If the implementation lands closer to 20 cyc/byte, peak CPU during Fast Last RX climbs from ~54% to ~60%. Still well under the 75% spike target.
- **Classifier-pending-at-fold-entry race.** §10.6 proposes "mask + clear pending" at first CC1 entry to defeat a classifier IRQ that latched right before the fold scheduler fired. Confirm the clear-pending write to `PFIC.IPRR.CH7` actually drops the latched IRQ on V006 (not just future ones).
- **`walk_deadline_margin` sizing.** The busy-wait exit in the last fold leaves GUARD bytes for post-fire. Today's value (in `dxl_fast`) was tuned for SysTick scheduling; re-measure under TIM2 scheduling.
- **HSI drift convergence.** §10.7.1's BT-averaged drift estimator needs N packets for a usable estimate. Measure σ on bench at the target traffic mix to set the smoothing window. Decide whether §10.7.3's in-packet adaptive bit_time is worth the cycles (probably not, given 10% static window margin).
- **First-byte BT seed under pre-packet glitches (§10.1).** Naive classifier + parser header-grid handles the common case. Verify on bench with EMI injection that 2–3 slip retries are enough in practice; if not, add tentative-anchor confirmation in the classifier.
- **TX_EN OC2M transitions (§5.2).** Bench-verify the Active-on-match + Force-inactive sequence produces clean rising/falling edges without spurious toggles at init, fire-cancel, and TC.
- **CC3IF latch on rearm at current CNT (§5.4).** Write CCR3 = current CNT on bench and observe whether CC3IF latches immediately, on next tick, or only after a wrap. Drives the size of `MAX_REASONABLE_REMAINING`.
- **TC tail budget (§9).** Profile worst-case `apply_pending_after_tc` runtime after the parse-time precompute lands. Verify it stays under one byte-time at the highest expected baud.
- **Low-baud wrap span (§5.4).** At 9600 baud, `10·bit_time ≈ 50000` ticks — close to the 16-bit wrap. Size `MAX_REASONABLE_REMAINING` against the largest legal inter-byte gap, not just RDT.
- **TIM2 ↔ SysTick clock bridging.** Capture a fixed offset once at boot for telemetry events that span both clocks. Define the convention in `idle_anchor`-style telemetry surfaces.
- **AFIO-then-DMA bringup order (§13 step 1).** AFIO remap must complete before DMA1_CH7 arms, or the first ET entries are garbage on transient pad state.

---

## 15. One-paragraph summary

> We move every DXL transport timing decision off USART IDLE and onto TIM2 input capture. PC1's dual remap feeds USART1_RX and TIM2_CH4 IC simultaneously; CH4 captures every falling edge into a 128-entry ET ring via DMA1_CH7. In Plain mode a `.highcode` window-classifier ISR runs at ET HT/TC, walking ET into a per-RX-index BT ring with a `[9·bit, 11·bit]` start-bit window test — constructive, self-healing within one byte, robust to glitches and overruns. The window uses `bit_time_spec = HCLK / baud` and re-anchors on every match, so HSI-vs-master drift doesn't compound across the packet; static window margin tolerates ±10% drift, ~10× the realistic HSI excursion (§10.7). The BT ring also doubles as the drift signal — inter-byte intervals averaged across packets give ppm-precise HSI drift, feeding the existing coarse `HSITRIM` step and software fine-trim residual. RX (64 B) and BT (128 B) stay small because a stateful parser drains up to `min(rx_write, bt_write)` at the tail of every classifier walk; IDLE backstops small packets but only as a signal, never as a timing source. **No tick value comes from IDLE anymore** — wire-end is `BT[last_byte] + 10·bit_time`, per-byte precise at every baud. The framing-mode FSM, IDLE-stamp queue, RXNE snapshot, and `9 × BRR` backdate constant all delete. In Fast Last mode, the classifier ISR is disabled at first-fold entry and the entire RX-side workload — classifier walk + parser drain + Fast Last CRC fold — runs in a periodic TIM2_CH1 Fast Last scheduler ISR at 17-byte intervals, the same scheduling pattern as today's SysTick-driven `body_fast_last`. The last interval busy-waits walk_deadline before exit; post-fire walks the GUARD residual and patches CRC into the TX buffer via DMA prefetch slack. TX fires from TIM2_CH3 compare → CC3IE IRQ → tiny ISR writing `USART1.CTLR1 \|= TE`, with CCR3 biased earlier by a measured `FIRE_BIAS_TICKS` so the wire bit lands on time. PC2 toggles in hardware on TIM2_CH2 compare to gate the bus driver before the TE write. Fire floor drops from ~5 µs to ~2.5 µs, comfortably under the 3.33 µs Fast-Last cap at 3 Mbaud. ADC stays on its existing DMA1_CH1 pump — the only new DMA channel is CH7 for ET. Priority layout matches today's: DXL-side IRQs at High, ADC at Low; the Fast Last scheduler's protection against same-priority delay is structural (classifier disabled mid-Fast-Last, last fold scheduled before fire). CCR2/CCR3 wrap with TIM2's 16-bit CNT (period 1.365 ms at PSC=0); the §5.4 set-and-recheck pattern catches "fire armed in the past" cases. A LUT-walker alternative for byte timing (read RX[i], advance ET cursor by `edge_count[RX[i]]`) is documented in §11 and rejected for the silent-desync risk. Total sustained-RX CPU at 3M peak is ~55% (Plain) or ~54% (Fast Last, all-in-one fold); during TX it's ~0% because the bus is being driven and no edges arrive. ET sizing is a CPU/memory knob (§8.4): default 128 entries / 17-byte interval, or 64 entries / 12-byte interval if SRAM tightens.

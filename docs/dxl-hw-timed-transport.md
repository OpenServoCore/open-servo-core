# DXL 2.0 Hardware-Timed Transport on the CH32V006

A design note for the next-generation DXL transport on the V006. Read [dxl-rx-timing.md](dxl-rx-timing.md) first if you haven't — this doc replaces the ISR-driven fire path it describes with a hardware-timed one, and adds per-byte RX timestamping built on TIM2 input capture. The pieces that didn't change (USART1 framing, half-duplex muxing, chain CRC, baud/RDT decision rule) still apply.

The headline change: every fire and every received-byte timestamp move into TIM2. SysTick stops driving wire-edge events. The ~5 µs PFIC-entry floor on TX fire goes away. RX timestamps become per-byte instead of per-packet, which closes the door on a whole class of low-baud / Fast-Last edge cases that today rely on backdating estimates.

The cost is that TIM2 is now the DXL transport's timer, and is no longer available for motor encoder counting. On the production board that's fine — there's no encoder. On bench builds it means the encoder bringup will need to find a different home (TIM3 stripped variant won't cut it; see §3 in the existing rx-timing doc).

---

## 1. Why move off SysTick

Two structural problems with the current design.

**Problem 1 — TX fire has a 5 µs PFIC floor.** Every fire today goes: SysTick CMP match → PFIC trap entry (~1 µs) → highcode ISR body (~1.5 µs) → DMA EN + TX_EN GPIO toggle (~2 µs) → first wire bit. At 3 Mbaud, one byte is 3.33 µs. The 5 µs floor is the inter-slot jitter cap. Every Fast-Last reply at the top of the baud range sees a visible idle gap before its slot. The CRC stays correct (the chain-CRC machinery handles that), but the wire-side coalesce isn't.

**Problem 2 — RX wire-end is estimated, not measured.** IDLE fires one char-time after wire-end and we backdate by `9 × BRR`. RXNE fires at the stop bit and we read the cursor. Both work, but neither gives us per-byte timestamps. At 3 Mbaud the snoop tail walk loses sub-byte resolution, and at low baud the IDLE delay forces the framing FSM to fall back to per-byte RXNE just to meet RDT — burning IRQs at exactly the speeds where we have the most CPU per byte to spare.

The hardware-timed design fixes both at once:

- TX fire becomes a timer compare event with a DMA-driven register write. PFIC isn't involved in the fire moment at all.
- RX timestamps come from TIM2 input capture on the RX pin's falling edges, DMA'd into a ring without ISR cost.

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

The plan is: TIM2 runs free at HCLK / prescaler. PC1's falling edges feed TIM2_CH4 in input-capture mode. Each capture writes CCR4 (the current CNT value) to a DMA destination — the TS ring. DMA1_CH7 is the channel; circular mode; HT and TC interrupts armed.

    PC1 ─┬─→ USART1_RX (DMA1_CH5 → RX ring, unchanged)
         └─→ TIM2_CH4 IC ─→ CCR4 ─→ DMA1_CH7 ─→ TS ring

Properties we lean on:

- **Captures are independent of the slave-mode controller.** CCxIF latches the timer's current count on every selected edge regardless of whether the counter is gated. We run TIM2 free, no slave-mode, no OPM.
- **No per-byte ISR.** DMA writes the timestamp. The CPU only wakes at HT (ring half) and TC (ring full).
- **TS overrun is detectable.** If the parser falls behind and DMA wraps past the consumer's read cursor, we lose timestamps. NDTR-vs-consumer comparison in the HT/TC handler turns this into a counted fault, not a silent corruption.

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

- Wire-end of any byte is `BC[i] + 10 × bit_time` (or 11, depending on stop-bit handling).
- Snoop CRC accumulator can timestamp each byte for jitter analysis.
- Fast-Last slot timing can fire at `BC[i_last_master_byte] + 10·bit + RDT + slot_offset` with no backdating estimate.
- The framing FSM's IDLE/RXNE split disappears — every packet has per-byte timestamps regardless of baud.

This is the BC ring: byte-time ring, sized to match RX, indexed parallel.

---

## 5. Peripheral plan: the TX side

### 5.1 Timer-triggered TX, software-fired with bias

Today the fire path is: SysTick CMP → IRQ → highcode ISR dispatches through STATE/DISPATCH → write `DMA1.CH4.CR |= EN` and `GPIOx.BSHR = TX_EN_bit`. In the new design:

    TIM2 CNT counts up free-running.
    fire_tick is written to CCR3, biased by mean PFIC + ISR latency.
    When CNT == CCR3, CC3IF latches → TIM2 IRQ fires.
    Tiny .highcode ISR: USART1.CTLR1 |= TE; enable DMA CH4.

The fire trigger moves off SysTick onto a TIM2 compare event, but the actual register write stays in software — like the SysTick design, but with a different IRQ source and a leaner ISR body. The single-purpose ISR (just "start TX now") doesn't have to dispatch through the STATE/DISPATCH indirection that today's `on_systick` does, so the body shrinks from ~1.5 µs to ~0.5 µs.

Floor accounting:

    Today (SysTick CMP):    ~1 µs PFIC entry + ~1.5 µs dispatched body + ~2 µs reg writes  ≈ 5 µs
    New (TIM2_CH3 IRQ):     ~1 µs PFIC entry + ~0.5 µs single-purpose body + ~1 µs reg writes ≈ 2.5 µs

The 2.5 µs floor sits comfortably under the 3.33 µs Fast-Last inter-slot cap at 3 Mbaud. Not as good as a hardware-DMA fire (which would have been ~100 ns) but enough to close the gap that today's design can't, and it keeps ADC on its existing DMA pump.

Bias compensation matches the existing approach: measure mean PFIC + ISR latency once on hardware, store it as a `FIRE_BIAS_TICKS` constant, and arm `CCR3 = fire_tick - FIRE_BIAS_TICKS` so the wire-bit lands at the intended `fire_tick`. Same shape as today's `PLAIN_ENTRY_TICKS` / `FAST_ENTRY_TICKS` in `measurements.rs`, just measured against TIM2 instead of SysTick.

Jitter (variance around the mean) comes from the same place it does today: same-priority IRQs already running when CC3IF latches (USART1 IDLE/TC, DMA1_CH5 chain CRC stage-1). Those handlers don't typically run at fire-time — USART1 IDLE has finished by `wire_end + RDT`, USART1 TC hasn't fired yet for the reply we're about to start. The fire IRQ's worst-case contention is the classifier (§9 question — High or Low priority).

### 5.2 Timer-fired TX_EN

PC2 is TIM2_CH2's alternate-function output. We use channel 2 in **output-compare toggle** mode, configure CCR2 to fire at the same tick as CCR3 (or a few ticks earlier if we need TX_EN setup time before the start bit), and let the timer drive the pin directly.

    TIM2 CNT == CCR2 → TIM2_CH2 output toggles → PC2 flips → bus driver turns on.

No GPIO write from software. No ISR. The pin transitions on a hardware tick.

We can also pre-load the TX_EN polarity into CCER's CC2P bit at init time so the toggle direction is correct, and use a "force inactive" register write (CCMR2's OC2M) to reset PC2 after TX completes (the USART TC ISR can still drive that — it's not jitter-critical, only fire is).

### 5.3 Composing CCR2 and CCR3

The two channels have different roles:

- **CCR2 drives PC2 directly via hardware OC** — TX_EN rises at exactly the timer tick, no software in the path, jitter = timer resolution (~20 ns).
- **CCR3 raises CC3IF to trigger the fire ISR** — TE write lands ~2.5 µs after CCR3, biased by the floor constant so the *wire bit* lines up with the intended `fire_tick`.

Composition:

    CCR3 = fire_tick - FIRE_BIAS_TICKS
    CCR2 = fire_tick - T_setup_ticks         # T_setup > FIRE_BIAS_TICKS

T_setup needs to be larger than FIRE_BIAS_TICKS so PC2 rises before the TE write (otherwise the start bit goes out into a high-impedance bus). Concretely: FIRE_BIAS ≈ 2.5 µs = ~120 ticks; T_setup ≈ 3-4 µs = ~150-200 ticks. The TX_EN edge ends up ~1-2 µs ahead of the first wire bit — plenty for bus driver settle.

Both compare events arm at parse time. CCR2's edge happens precisely (hardware); CCR3's edge has the same precision but the ISR-derived register write inherits PFIC entry jitter. Net wire-edge jitter is determined by the CCR3 path — significantly smaller than today because the ISR body is leaner and the fire trigger is no longer scheduled by the same SysTick used for other things.

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
    TIM2.CCR2 = (wire_end + rdt_ticks - T_setup_ticks) & 0xFFFF
    enable CC3IE

    cnt_now   = TIM2.CNT
    remaining = (ccr3_tick - cnt_now) & 0xFFFF
    if remaining > MAX_REASONABLE_REMAINING:
        # we missed — fire by software now (same body as the ISR)
        pc2_high()
        USART1.CTLR1 |= TE

`MAX_REASONABLE_REMAINING` is the largest legitimate fire-in-the-future we expect (RDT_max + slot_offset_max + a few µs slack). Anything past it means the modular subtraction wrapped backwards — i.e., we're "in the past."

The recheck only needs to look at CCR3. CCR2 = CCR3 - (T_setup - FIRE_BIAS) is implicitly in the future whenever CCR3 is in the future by at least that delta. The manual-fire branch runs the same body as the CC3 ISR — drive PC2 high, then write TE.

---

## 6. DMA channel assignments

CH32V006 DMA1 has fixed source-to-channel mux. Allocation:

| DMA1 channel | Old | New | Notes |
| --- | --- | --- | --- |
| CH1 | ADC pump | unchanged | ADC stays on its existing DMA pump |
| CH4 | USART1_TX (single-shot) | unchanged | Still per-fire reconfigured |
| CH5 | USART1_RX (circular) | unchanged | RX byte ring |
| CH7 | free | **TS ring (TIM2_CH4 IC capture)** | |

Only one new DMA channel comes into play: CH7 for the TS ring. The other channels keep their current roles. By keeping fire on the software ISR path (§5.1), we don't need DMA1_CH1 for TX, so ADC's existing circular DMA pump is untouched. SysTick stays unused for DXL — free for telemetry or future use.

This is the lightest-touch DMA allocation the redesign supports. Heavier alternatives (TIM2_CH3 → DMA1_CH1 hardware-DMA fire, with ADC moved to SysTick-polled or TIM3-chained sampling) would shave another ~1-2 µs off the fire floor but at the cost of migrating ADC. The 2.5 µs floor from the software-fire path is already under the 3.33 µs Fast-Last cap at 3 Mbaud, so the heavier rework isn't load-bearing for the redesign's goals.

---

## 7. Putting the timer in slave/master mode (or not)

Decisions on TIM2 configuration:

- **No OPM** (one-pulse mode). OPM would auto-stop the counter on update — we need it counting forever.
- **No slave-mode trigger.** SMS = 0b000. Free-running.
- **CC1S / CC2S / CC3S / CC4S** chosen per channel: CH4 = input capture (CC4S = 01), CH2 / CH3 = output compare (CC2S/CC3S = 00).
- **Prescaler.** PSC = 0 — CNT counts at HCLK / 1 = 48 MHz. One tick = 20.83 ns. ARR = 0xFFFF, period ≈ 1.365 ms. The IC side wants the fine resolution: at 3 Mbaud, bit_time = 333 ns = 16 ticks; the classifier's `[9·bit, 11·bit]` window = `[144, 176]` ticks. Higher prescalers cut the bit_time to 8 or fewer ticks, eating the tolerance budget. TS-ring consumers handle wrap via a cumulative byte counter (same trick as today's IDLE-stamp ring). TX-fire arming handles wrap via the set-and-recheck pattern (§5.4).
- **Capture filter on CH4** (ICF bits in CCMR2). Reject narrow glitches before they reach the TS ring. Conservative setting: `fSAMPLING = fCK_INT/8, N=8` for stable sampling. The chain CRC layer treats glitch-induced spurious edges as packet faults already; cutting them at the capture filter is cheap insurance.
- **Polarity on CH4** (CC4P): falling edge only. Rising edges are start-bit ends — we want the start-bit *begin*, which is the 1→0 transition.

---

## 8. Memory rings

Three rings, three lifetimes.

### 8.1 RX ring (unchanged)

512 bytes, circular, DMA1_CH5 from USART1.DR. Byte data only. No change from today.

### 8.2 TS ring (new)

128 entries × 16-bit = 256 B. Circular, DMA1_CH7 from TIM2.CCR4. Captures the TIM2 CNT value at every falling edge on PC1.

Why 128 entries:

- At 3 Mbaud, worst-case edge rate is 5 edges per byte × 300 KB/s = 1.5 M edges/s.
- HT fires at 64 entries written, TC at 128. At 1.5 M edges/s, that's HT every ~43 µs.
- The classifier (§10) runs in ~60 cycles per edge → 64 edges = ~3840 cycles = 80 µs at 48 MHz.
- Net: classifier keeps up with the worst case with margin.

If the classifier falls behind (e.g., a higher-priority IRQ steals the slot), TS overflow is detected by comparing NDTR's progression against the consumer's last drained index. We count it as a fault and reset the classifier's anchor at the next IDLE. Wire-CRC still validates the packet — a missed BC entry isn't a wire-level error, it's a degraded telemetry signal.

### 8.3 BC ring (new)

`BC_LEN` entries × 32-bit = 4 × `BC_LEN` bytes. `BC_LEN` is sized to match the RX ring (so byte index `i` in RX maps to byte time `BC[i]`). At RX_LEN = 512, BC = 2 KB. That's a third of SRAM; we may shrink BC entries to 16 bits + a wrap counter if SRAM gets tight.

Each entry: `BC[i] = TIM2 CNT at the start bit of byte i in the RX ring`. Consumers read it by RX index, no walk required.

The classifier writes BC entries. The consumer (chain CRC, fire scheduler, snoop log) reads them by index. Both happen under the same High priority — no locking needed.

---

## 9. ISRs and priorities

Same two priority levels (V006 PFIC has nothing more). Reshuffled responsibilities:

| Priority | IRQ | Body | Where |
| --- | --- | --- | --- |
| High | DMA1_CH7 HT/TC | TS ring classifier walk → BC ring updates | `.highcode` |
| High | USART1 | IDLE + RXNE + TC, framing/telemetry only (no fire) | `.highcode` |
| High | DMA1_CH5 HT/TC | Chain CRC stage-1 (unchanged) | `.highcode` |
| High | TIM2 (CC3IE) | Fire TX: `USART1.CTLR1 \|= TE` + DMA CH4 enable | `.highcode` |
| Low | DMA1_CH1 | ADC kernel pump (unchanged) | flash |

**The fire moves from SysTick to TIM2 but stays in software.** TIM2_CH3 CC3IF latches on compare match; the TIM2 IRQ runs a single-purpose body that writes `USART1.CTLR1 \|= TE` and enables DMA CH4. PFIC entry stays in the path; CCR3 is biased earlier by `FIRE_BIAS_TICKS` so the wire bit lands on time (§5.1).

**TX_EN is hardware-driven.** CCR2 → PC2 via OC mode — no ISR involvement, edge lands at the exact compare tick.

**The classifier sits at top priority** by default. Its job is to keep BC entries fresh enough that any consumer's `BC[i]` lookup gets a stable answer before the consumer needs it. At HT/TC rates of ~23 kHz (3 Mbaud worst case), the classifier interrupts serialize cleanly with other High IRQs via same-priority no-preemption (existing trick from §8.4 of the rx-timing doc).

The trade: classifier-at-High means a classifier-mid-walk can delay the fire IRQ by up to its runtime (~10 µs worst case at 3M peak burst), pushing the 2.5 µs fire floor up correspondingly when contention happens. Worst-case contention is rare — classifier HT/TC fires at ~23 kHz at peak, fire happens once per reply, and the windows have to coincide. We accept this and instrument it; if it becomes a problem we revisit classifier-at-Low (see §14 open questions).

**SysTick CMP is unused for DXL.** Free for telemetry or other tasks.

**USART1's role shrinks.** It still handles:

- IDLE for packet-boundary detection (parsing trigger),
- TC for "reply finished, release the bus" (drops TX_EN via timer CCMR2 force-inactive + clears DMA CH4 + applies pending baud/RDT),
- RX errors for telemetry.

It no longer drives fire timing or backdates wire-end. The framing FSM and IDLE/RXNE split disappear; per-byte timestamps come from BC for every byte, regardless of baud.

---

## 10. The window classifier (primary RX byte-timing design)

The classifier turns TS entries into BC entries. It runs at every HT/TC of DMA1_CH7.

### 10.1 Core idea

Each UART byte starts with a 1→0 transition (the start bit). After the start bit, more 1→0 transitions can happen inside the byte (data bits). After the stop bit, the line is high until the next byte's start bit.

If we know `BC[i]` (start tick of byte `i`), then byte `i+1`'s start bit is **exactly** 10 bit-times later (start bit + 8 data bits + 1 stop bit = 10 bit positions). Any TS entry between `BC[i]` and `BC[i] + 10·bit_time` is a data-bit transition (discard it). The first TS entry at or after `BC[i] + 10·bit_time` is `BC[i+1]`.

That's the whole algorithm:

    for each new TS entry t:
        if t ≤ anchor + 11 × bit_time and t ≥ anchor + 9 × bit_time:
            # t is the next start bit
            anchor = t
            BC[byte_idx] = t
            byte_idx += 1
        elif t < anchor + 9 × bit_time:
            # data-bit transition — skip
        else:
            # gap > 11 bit-times: inter-byte stall or end of packet
            # re-anchor at the next entry treated as a start bit
            anchor = t
            BC[byte_idx] = t
            byte_idx += 1

The window is `[9 × bit_time, 11 × bit_time]`. Tolerance comes from the bit-time uncertainty itself — at 3 Mbaud, bit_time = 16 ticks; the window is `[144, 176]` ticks from the previous anchor. HSI drift within a packet (~853 µs at 3 Mbaud, 256-byte packet) is sub-tick; no drift correction needed within a packet.

### 10.2 Why this is robust

- **Glitch in the middle of a byte:** a spurious falling edge between `anchor + 0` and `anchor + 9 × bit_time` is rejected by the window test. No state corruption.
- **Glitch in the inter-byte gap that happens to fit the window:** the next *real* start bit will land outside the window (because the real byte is offset by the glitch's arrival time). The classifier re-anchors on the next entry that fits a window from the *new* anchor — self-heals within 1 byte.
- **Missed edge (TS overrun):** anchor stays stuck at the last valid entry; the next entry will be too late for the window and trigger re-anchor as "gap." We mark BC entries from the overrun point as invalid (sentinel value); consumers fall back to packet-boundary stamps for those bytes. Detectable and bounded.
- **Spec-baud mismatch:** the window is computed from `bit_time = HCLK / baud`. If the host writes a baud the slave isn't running at, every entry falls outside every window — the whole packet's BC marks as invalid. Telemetry visible.

The classifier is **constructive**: it builds byte boundaries from edge times, not from byte content. The RX ring's data and the BC ring's times are derived from the same wire events but via independent paths. A byte data error doesn't poison BC; a BC error doesn't poison byte data.

### 10.3 CPU cost

Per HT/TC the classifier walks up to 64 TS entries. Per entry: 1 load (TS), 2 comparisons (window low, window high), 1 store (BC), 1 anchor update. ~8 cycles per entry × 64 = ~512 cycles per HT/TC = ~10.7 µs at 48 MHz.

At 3 Mbaud worst-case edge rate (1.5 M edges/s), HT/TC fire at ~23 kHz. Classifier consumes 23 kHz × 10.7 µs = ~25% CPU during peak burst. Bursts are bounded by packet length (256 bytes max DXL = ~853 µs @ 3M = one HT/TC pair). Steady-state CPU is much lower; bus duty cycle in real workloads is well under 50%.

### 10.4 Where it lives

`firmware/ch32/src/rx_classifier.rs` (new). Body in `.highcode`. Two entry points: `on_ts_ht()` walks indices `0..TS_LEN/2`; `on_ts_tc()` walks `TS_LEN/2..TS_LEN`. State is a single `Anchor { tick: u32, byte_idx: u16 }` updated in-place. Reset on USART1 IDLE (packet boundary): anchor invalid until the next TS entry, which is the next packet's first start bit.

---

## 11. Alternative: LUT walker with resync detector (secondary, more flawed)

Considered, rejected for now, documented for completeness. May come back if §10's CPU cost turns out to bite under some workload.

### 11.1 Idea

The number of falling edges in a UART byte is a fixed function of the byte's data value. A 256-entry lookup table gives `edge_count[byte_data] ∈ [1, 5]`. If we know `BC[i]` and `RX[i]`, then:

    expected_TS_cursor_advance = edge_count[RX[i]]
    BC[i+1] = TS[ts_cursor + edge_count[RX[i]]]

The walker reads RX bytes in order, advances the TS cursor by the LUT value, takes the next TS entry as the next BC. ~5 cycles per byte instead of ~8 cycles per edge × ~3 edges/byte = ~24 cycles per byte. Roughly 5x faster.

### 11.2 Why it's flawed

The LUT lookup uses **byte data** to predict **edge structure**. Anything that corrupts the alignment between edges and bytes corrupts every subsequent BC entry until the next resync.

Failure modes:

- **Phantom edge** (glitch adds an unaccounted edge): LUT predicts 4 edges for current byte, reality is 5. Walker advances 4, lands on the glitch instead of the next byte's start bit. All BC entries past this point are shifted by 1 edge.
- **Missed edge** (TS overrun, capture-filter swallow): LUT predicts 5, only 4 captured. Walker advances 5, consuming one edge from the next byte's frame. Cascade.
- **Byte data corruption** (USART noise → wrong byte): LUT lookup uses the corrupted byte → wrong edge count → cascade.

### 11.3 The resync detector

To bound the damage we'd add a sanity check: after advancing the cursor by `edge_count[RX[i]]`, verify the next TS entry is within `[BC[i] + 9·bit_time, BC[i] + 11·bit_time]`. If yes, take it as `BC[i+1]`. If no, mark BC for the rest of the packet as invalid and fall back to packet-boundary timestamps.

Detection coverage is good but not perfect. The check catches:

- All TS-overrun desyncs (cursor under-advance lands on a real edge inside the next byte at offset 16–80 ticks past predicted; window catches it).
- Most phantom-edge desyncs (cursor over-advance lands data-dependently; usually outside window).

The check misses:

- **Phantom edge landing exactly in the next-start window.** Roughly 4/160 ≈ 2.5% probability per glitch event at 3 Mbaud. Walker silently desyncs and consumers act on corrupted BC.
- **Multi-byte cascade resync.** Once we detect, we can only resync at IDLE — the walker doesn't know how many edges to skip to realign mid-packet.

### 11.4 The actual tradeoff

| | Window classifier (§10) | LUT walker + resync |
| --- | --- | --- |
| Self-healing | Within 1 byte | Detect-only, requires IDLE to resync |
| Silent corruption rate | Effectively zero | ~2.5% per glitch event escapes |
| CPU during peak burst | ~25% | ~5% |
| Memory | TS 256 B + BC | Same + 256 B LUT |
| Algorithm complexity | One path | Two paths + resync state |

The CPU savings aren't load-bearing — peak burst at 25% leaves plenty of margin for chain CRC and USART1 work running at the same High priority. The robustness story is what matters for a transport whose entire point is precise per-byte timing.

If the spike of §10 shows the V006 can't actually meet 25% peak (e.g., because chain CRC stage-1 is eating more cycles than expected), we can revisit. Hybrid would be possible: LUT walker in steady state with classifier fallback on anomaly. Adds complexity that isn't free either.

For now: build the window classifier, measure, decide.

---

## 12. ISR reshuffle summary

What changes in the existing firmware:

| File | Today | After |
| --- | --- | --- |
| `dxl/state.rs` | `ReplyState { Idle, Plain, Chain { … } }` w/ FSM + dispatch table | Same shape; fire dispatch moves to a TIM2 IRQ but the FSM stays |
| `dxl/isr.rs` | `body_plain_fire`, `body_chain_catchup`, `body_chain_fast_fire` | Same bodies, called from TIM2 IRQ instead of SysTick; leaner because no STATE/DISPATCH indirection on the TIM2 hot path |
| `irq.rs` | `on_systick_match` dispatches via `STATE`/`DISPATCH` | `on_systick_match` unused (free for telemetry); new `on_tim2` handler does the fire |
| `irq.rs` | `on_usart1_idle` backdates and stamps `IdleAnchor` | Still publishes packet boundary; BC ring is now the per-byte source |
| `rx_classifier.rs` | n/a | New: HT/TC walker for TS → BC |
| `board/bringup.rs` | TIM2 unconfigured | TIM2 init: free-running, CH4 IC, CH2 OC (hardware TX_EN), CH3 OC (fire IRQ), DMA CH7 |
| `idle_anchor.rs` | EXTI snoop instrumentation (9 atomics) | All EXTI snoop fields removed; framing relies on BC, not snoop |

The EXTI-pin instrumentation built on `firmware/dxl-2.0-transport` (the current branch) becomes obsolete — its job is fully subsumed by TIM2 IC capture, which is per-edge instead of first-edge-only. We can land one more bench capture on the EXTI path for diff context, then strip it in the migration.

---

## 13. Spike plan

Recommended order — each step independently verifiable on the dev board.

1. **PC1 dual-tap.** Configure AFIO for USART1_RX + TIM2_CH4 simultaneously. Verify USART RX still framed correctly (no regression) and TIM2 CCR4 captures on every PC1 falling edge. Use a scope probe on PC1 + a bench-side print of captured CCR4 values to compare.
2. **TS ring + classifier.** Wire DMA1_CH7 to CCR4 → TS ring (circular). Implement `rx_classifier.rs`. At low baud (e.g. 115200) compare classifier-produced BC against IDLE-backdated tick for the same packet. Should agree to within 1 tick per byte. Then ramp baud to 1M, 3M.
3. **Software TX fire via TIM2_CH3 IRQ.** Wire CC3IE; implement the leaner ISR body (TE write + DMA CH4 enable). Measure mean CCR3-match → first-wire-bit latency, store as `FIRE_BIAS_TICKS`. Target: < 3 µs floor at 3M. Verify the §5.4 set-and-recheck path by forcing a small-RDT case (RDT=0 at low baud) that exercises the manual-fire branch.
4. **TX_EN via TIM2_CH2.** Wire CH2 → PC2 in OC mode (set-active-on-match). Verify TX_EN rises before CCR3's fire by `T_setup - FIRE_BIAS` ticks and falls correctly at TC.
5. **Integration: Fast Last-slave at 3 Mbaud.** Validate inter-slot coalesce gap — target: under one byte time. This is the test the SysTick design fails.
6. **Strip the old fire path.** Once §5 passes, remove SysTick-CMP scheduling, IDLE-stamp queue's wire-end role (keep IDLE for packet boundary only), framing-mode FSM, RXNEIE composer, EXTI snoop instrumentation.

Each step is a self-contained commit per the project's "one reviewable unit" rule.

---

## 14. Open questions for the spike

- **AFIO dual-remap stability.** F1 documentation is explicit about peripheral-side input dual-tap working; V006 RM is less so. Step 1 confirms by direct measurement.
- **Capture filter setting.** Cap filter `(fCK_INT/8, N=8)` is a starting guess. May need to relax it if real bus glitches are wider than expected, or tighten it if too many spurious edges reach TS.
- **Fire-floor measurement.** Measure CCR3-match → first-wire-bit at 3M on bench. Estimate is ~2.5 µs; confirm and lock `FIRE_BIAS_TICKS`.
- **Classifier priority placement.** Default plan keeps classifier at High alongside USART1 / chain-CRC / TIM2 fire IRQs — same-priority serialization keeps state coherent but a classifier-mid-walk can push the fire floor up by up to ~10 µs. Alternative: move classifier to Low so the fire IRQ preempts it; classifier may then fall behind during USART1/chain-CRC bursts. Decide once we have measurements of how often the contention windows actually overlap.
- **BC entry width.** 32-bit BC entries cost 2 KB at RX_LEN=512. 16-bit entries with a wrap-counter cost 1 KB but add bookkeeping. Decide after BC consumers are written and we know the access pattern.

---

## 15. One-paragraph summary

> We move the DXL transport's wire-edge events into TIM2. PC1's dual remap feeds USART1_RX and TIM2_CH4 IC simultaneously; CH4 captures every falling edge into a 256-byte TS ring via DMA1_CH7. A `.highcode` window-classifier ISR runs at TS HT/TC, walking TS into a per-RX-index BC ring with a `[9·bit, 11·bit]` start-bit window test — constructive, self-healing within one byte, robust to glitches and overruns. TX fires from TIM2_CH3 compare → CC3IE IRQ → tiny ISR writing `USART1.CTLR1 \|= TE`, with CCR3 biased earlier by a measured `FIRE_BIAS_TICKS` so the wire bit lands on time. PC2 toggles in hardware on TIM2_CH2 compare to gate the bus driver before the TE write. The fire floor drops from ~5 µs (SysTick CMP + dispatched ISR + reg writes) to ~2.5 µs (TIM2 CC + single-purpose ISR + leaner reg writes), comfortably under the 3.33 µs Fast-Last cap at 3 Mbaud. ADC stays on its existing DMA1_CH1 pump — the only new DMA channel is CH7 for TS. CCR2/CCR3 wrap with TIM2's 16-bit CNT (period 1.365 ms at PSC=0); the §5.4 set-and-recheck pattern catches the case where an arm lands "in the past" and falls back to running the same fire body inline to avoid sleeping a full wrap. A LUT-walker alternative for byte timing (read RX[i], advance TS cursor by `edge_count[RX[i]]`) is documented in §11 and rejected for the silent-desync risk: the window classifier's ~25% peak-burst CPU is the price we pay for not silently corrupting BC entries when the bus glitches.

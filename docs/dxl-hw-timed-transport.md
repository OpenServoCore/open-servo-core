# DXL 2.0 Hardware-Timed Transport on the CH32V006

A design note for the next-generation DXL transport on the V006. Read [dxl-rx-timing.md](dxl-rx-timing.md) first if you haven't — this doc replaces the ISR-driven fire path it describes with a hardware-timed one, and adds per-byte RX timestamping built on TIM2 input capture. The pieces that didn't change (USART1 framing, half-duplex muxing, chain CRC, baud/RDT decision rule) still apply.

The headline change: every fire and every received-byte timestamp move into TIM2. SysTick stops driving wire-edge events. The ~5 µs PFIC-entry floor on TX fire goes away. RX timestamps become per-byte instead of per-packet, which closes the door on a whole class of low-baud / Fast-Last edge cases that today rely on backdating estimates.

The cost is that TIM2 is now the DXL transport's timer, and is no longer available for motor encoder counting. On the production board that's fine — there's no encoder. On bench builds it means the encoder bringup will need to find a different home (TIM3 stripped variant won't cut it; see §3 in the existing rx-timing doc).

---

## 1. Why move off SysTick

Three structural problems with the current design.

**Problem 1 — TX fire has a 5 µs PFIC floor.** Every fire today goes: SysTick CMP match → PFIC trap entry (~1 µs) → highcode ISR body (~1.5 µs) → DMA EN + TX_EN GPIO toggle (~2 µs) → first wire bit. At 3 Mbaud, one byte is 3.33 µs. The 5 µs floor is the inter-slot jitter cap. Every Fast-Last reply at the top of the baud range sees a visible idle gap before its slot. The CRC stays correct (the chain-CRC machinery handles that), but the wire-side coalesce isn't.

**Problem 2 — every timing decision goes through IDLE.** Today's wire-end derivation is `systick::ticks() - 9 × BRR` at IDLE-handler entry. IDLE fires one char-time *after* wire-end, so we backdate. At high baud this works; at low baud the publish latency exceeds RDT and the framing FSM has to switch to per-byte RXNE timestamps (rx-timing doc §4–5), burning ~100 K IRQs/sec at 1 Mbaud just to keep timing source-of-truth. The whole IDLE-vs-RXNE FSM, the framing decision rule, the pipeline_margin tuning knob — all of it exists because IDLE is the only timing source the current design has. This is fragile and not extensible.

**Problem 3 — RX timestamps are per-packet, not per-byte.** Snoop tail walks, chain-CRC stage-3, slot timing — all want to know "when did *this specific byte* arrive?" Today they get "when did the packet end" and infer. At 3 Mbaud the sub-byte resolution loss matters.

The hardware-timed design fixes all three at once:

- TX fire becomes a TIM2 compare event with a leaner software ISR (§5). PFIC entry stays in the path but the ISR shrinks and the trigger source isn't shared with anything else.
- **Every byte gets its own wire-arrival timestamp** via TIM2 input capture on the RX pin's falling edges, written to a BC (byte-time) ring at HT/TC of the IC DMA. IDLE becomes a *signal* (drain the parser, packet is done), never a *timing source*. The framing FSM disappears with it.
- All timing consumers — wire-end, fire-tick, slot deadline, snoop tick — look up `BC[i] + 10·bit_time` directly. No backdate constants, no char-time math, no IDLE-vs-RXNE branch.

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

Three rings, three lifetimes. RX and BC are always 64 entries; TS sizing is a CPU-vs-memory tradeoff (see §8.4).

### 8.1 RX ring

64 bytes, circular, DMA1_CH5 from USART1.DR. Byte data only. *Shrinks* from today's 512 B because the parser drains continuously (at classifier-end in Plain mode; at each catchup interval in Chain mode) instead of waiting for IDLE — ring size decouples from max packet size.

### 8.2 TS ring (new)

Circular, DMA1_CH7 from TIM2.CCR4. Captures the TIM2 CNT value at every falling edge on PC1.

Two viable sizings — see §8.4. We default to **128 entries × 16-bit = 256 B** (the Chain-safe sizing); the spike may downgrade to 64 entries (128 B) if measurements support it.

If the consumer falls behind (longer-than-expected ISR somewhere), TS overflow is detected by comparing NDTR against the consumer's last drained index. We count it as a fault and reset the classifier's anchor at the next IDLE. Wire-CRC still validates the packet — a missed BC entry isn't a wire-level error, it's a degraded timing signal.

### 8.3 BC ring (new)

64 entries × 16-bit = 128 B (with a separate 32-bit cumulative-byte counter for wrap disambiguation, same pattern as today's IDLE-stamp ring). Sized to match the RX ring so byte index `i` in RX maps to byte time `BC[i mod 64]`.

Each entry: `BC[i] = TIM2 CNT at the start bit of byte i in the RX ring`. Consumers read it by RX index, no walk required.

In Plain mode the classifier writes BC at HT/TC of DMA1_CH7. In Chain mode the catchup ISR writes BC (classifier ISR is disabled — §10.6). Consumers (parser, fire scheduler, chain CRC, snoop log) read by index regardless of which ISR was the writer. All happen under the same High priority — no locking needed.

### 8.4 TS sizing: 128 (default) vs 64 (memory-tight)

The Chain catchup ISR runs at 17-byte intervals (matching today's well-tuned schedule). Classifier is disabled between intervals, so the TS ring has to absorb everything received in one interval window without overwriting.

Worst-case edge density is 5 edges/byte (0x55-heavy stream):

    edges per 17-byte interval = 17 × 5 = 85 edges

TS_LEN ≥ 85 → **128 is the safe choice.** Total ring memory: 64 (RX) + 256 (TS) + 128 (BC) = **448 B**.

If we want to drop TS to 64 (saves 128 B), we shorten the catchup interval so peak edges per interval fits:

    edges per N-byte interval ≤ 64
    N ≤ 12 bytes  (12 × 5 = 60 edges)

That shifts CPU cost — more frequent catchups means more IRQ entry/exit overhead. Quantitatively:

| Option | TS_LEN | Memory | Interval | Cadence @ 3M | ISR overhead extra |
| --- | --- | --- | --- | --- | --- |
| **A (default)** | 128 | 448 B | 17 B | 56.7 µs | baseline |
| **B (tight)** | 64 | 320 B | 12 B | 40 µs | +~1 µs/100 µs ≈ +1% CPU |

The CPU delta between A and B is small (per-byte work dominates over per-ISR overhead). 128 B of SRAM on an 8 KB chip is also modest. Default to **A**; revisit if SRAM gets tight elsewhere.

The Plain-mode constraints (parser keep-up at classifier cadence) are unaffected either way — Plain mode doesn't use the catchup ISR, and its classifier-end drain handles peak rates with margin at TS_LEN=64 already.

---

## 9. ISRs and priorities

Same two priority levels (V006 PFIC has nothing more). DXL-related IRQs stay at High alongside today's; only ADC sits at Low.

| Priority | IRQ | Body | Where |
| --- | --- | --- | --- |
| High | USART1 | IDLE (parser kick) + TC (release bus) + RX errors | `.highcode` |
| High | DMA1_CH7 HT/TC | Classifier walk (TS → BC) + parser drain — *Plain mode only; disabled during Chain* | `.highcode` |
| High | TIM2 (CC1IE) | Chain catchup (Chain only): classifier + parser drain + CRC fold over 17-byte interval | `.highcode` |
| High | TIM2 (CC3IE) | Fire TX: `USART1.CTLR1 \|= TE` + DMA CH4 enable + post-fire CRC residual + patch | `.highcode` |
| Low | DMA1_CH1 | ADC kernel pump (unchanged) | flash |

All wire-side IRQs at High serialize cleanly via same-priority no-preemption (existing trick from `dxl-rx-timing.md` §8.4). The structural protection for the Fast-Last fire deadline isn't priority — it's **scheduling**:

- During Plain replies, classifier may fire whenever HT/TC trips. Plain replies have no inter-slot jitter cap, so a ~5 µs classifier walk delay before fire is harmless (RDT ≥ 250 µs typical).
- During Chain replies (Fast Last-slave), the classifier ISR is disabled at the first catchup entry. From then until USART1 TC, classifier doesn't compete for the High slot — Chain mode is entirely owned by the catchup ISR cadence (§10.6) and the fire ISR. The last catchup is scheduled to exit before `fire_tick - FIRE_BIAS`, so fire never waits behind catchup either.

**DMA1_CH5 HT/TC interrupts are no longer enabled.** The old chain-CRC stage-1 ISR (which walked RX bytes into the snoop CRC at DMA half/full events) is structurally replaced by the catchup ISR (§10.6). The CH5 channel itself still runs — it's USART1's circular byte-DMA pump — but no IRQ is wired to it. The RXNEIE composer from `dxl-rx-timing.md` §7.3 also disappears: framing has no RXNE owner anymore, and chain CRC doesn't need one.

**Bringup note — INESTEN.** Per RM §6.5.2.21, each IRQ's `PFIC_IPRIORx` field uses bit 7 as the preempt-priority bit (bit 6 is sub-priority within a group; bits 5:0 are reserved), and `INTSYSCR.INESTEN = 0` at reset disables preemption entirely. Default IPRIOR = 0 → bit 7 = 0 → every IRQ defaults to High; that's fine for the DXL-side IRQs but means we explicitly write bit 7 = 1 for DMA1_CH1 (ADC) to put it at Low. Bringup must also flip INESTEN to 1 for that distinction to actually preempt.

**The fire moves from SysTick to TIM2 but stays in software.** TIM2_CH3 CC3IF latches on compare match; the TIM2 IRQ runs a single-purpose body that writes `USART1.CTLR1 \|= TE` and enables DMA CH4. PFIC entry stays in the path; CCR3 is biased earlier by `FIRE_BIAS_TICKS` so the wire bit lands on time (§5.1).

**TX_EN is hardware-driven.** CCR2 → PC2 via OC mode — no ISR involvement, edge lands at the exact compare tick.

**Everything-at-Low-except-fire** is the structural fix for the fire-jitter problem. Any wire-side parser/classifier/CRC work at High would force TIM2 CC3 to wait behind it, pushing the 2.5 µs fire floor up to whatever that work cost. With the rule "only TIM2 CC1/CC3 at High," fire is guaranteed to preempt whatever else was running. The floor stays at 2.5 µs at peak load.

The Low band serializes among itself (USART1, classifier, ADC) via same-priority no-preempt. That's fine — none of them have wire-edge deadlines.

**Pre-fire catchup on TIM2_CH1** (Chain replies only) belongs to the fire family — see §10.6.

**SysTick CMP is unused for DXL.** Free for telemetry or other tasks.

**USART1's role shrinks dramatically.** It still handles:

- IDLE as a parser-kick signal (drain RX up to current frontier, packet is done) — *no timing value is derived from it*; wire-end comes from BC,
- TC for "reply finished, release the bus" (drops TX_EN via timer CCMR2 force-inactive + clears DMA CH4 + applies pending baud/RDT),
- RX errors for telemetry.

The framing FSM (IDLE vs RXNE), the `decide(brr, rdt)` rule, the `pipeline_margin_us` tuning knob, the `DXL_CHAR_TIME_TICKS` backdate constant, the IDLE-stamp queue, and the RXNE single-cell snapshot all **go away**. Every byte gets a per-byte timestamp from BC, regardless of baud — there's no high-vs-low-baud strategy split because there's no estimation involved at any baud.

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

Per HT/TC the classifier walks up to 32 TS entries (TS_LEN/2). Per entry: 1 load (TS), 2 comparisons (window low, window high), 1 store (BC), 1 anchor update. ~8 cycles per entry × 32 = ~256 cycles per walk = ~5.3 µs at 48 MHz.

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

`firmware/ch32/src/rx_classifier.rs` (new). Body in `.highcode`. Two entry points: `on_ts_ht()` walks indices `0..TS_LEN/2`; `on_ts_tc()` walks `TS_LEN/2..TS_LEN`. State is a single `Anchor { tick: u32, byte_idx: u16 }` updated in-place. Reset on USART1 IDLE (packet boundary): anchor invalid until the next TS entry, which is the next packet's first start bit.

### 10.5 Coordinating with the parser

The classifier produces BC entries at TS HT/TC; USART1's RX DMA produces RX bytes at each stop bit. The two streams have different cadences:

- **`RX[i]`** lands in memory ~10 bit-times after byte i's start edge (the byte-shift time plus DMA latency) — fast and immediate.
- **`BC[i]`** lands in memory when the classifier walks the TS entry for byte i's start edge, which happens at the next TS HT/TC. In the worst case BC[i] lags RX[i] by up to half a TS ring's worth of edges.

**The parser's invariant:** consume bytes only up to `min(rx_write_pos, bc_write_pos)`. Past that point, RX data may exist without a corresponding BC tick — partial state from the parser's perspective.

This makes the parser stateful: an RX HT/TC (or classifier HT/TC) that lands mid-packet drains up to the BC frontier, parser yields with in-progress header/length/payload state preserved, and resumes on the next event. The state to keep is small:

    parsed_idx: u16         # bytes consumed so far (cumulative, with wrap counter)
    decode_state: ...       # header / length / payload / CRC stage
    expected_end: Option<u16>  # set once header.length is decoded

**Trigger sites:** two are sufficient.

- **Classifier HT/TC (DMA1_CH7).** The natural drain point — the classifier just walked TS → BC, so we drain RX up to the new BC frontier in the tail of the same ISR. Cadence is tied to TS_LEN by design.
- **USART1 IDLE.** Backstop for small packets that don't fill enough of TS to trip classifier HT. A 14-byte all-0xFF reply generates only 14 edges (below TS_LEN/2 = 32), so the classifier never fires on its own. IDLE catches it. **IDLE is a *signal* here, not a timing source** — it tells the parser "drain, packet done," and the wire-end tick still comes from `BC[last_byte] + 10·bit_time`, never from an IDLE-derived backdate.

No RX HT/TC parser trigger, no SysTick parser trigger, no decimated counter. RX HT/TC stays disabled for parser purposes; the chain-CRC stage-1 snoop walk is a separate consumer that may keep its own RX HT/TC enable.

**Wire-end derivation** at request_complete:

    wire_end_tick = BC[parsed_end_idx - 1] + 10 × bit_time
    fire_tick     = (wire_end_tick + RDT_ticks - FIRE_BIAS) & 0xFFFF
    TIM2.CCR3     = fire_tick
    # ... + set-and-recheck guard from §5.4

No backdate constant. No char-time math. The whole framing-mode FSM from `dxl-rx-timing.md` §7 evaporates.

**Plain-mode trigger sites:** classifier HT/TC + IDLE (above). The classifier walk drains the parser at its tail — no CRC fold, no chain state.

**Chain-mode trigger sites:** the Chain catchup ISR (§10.6) replaces both. At first catchup entry, classifier HT/TC interrupts are masked; from then until USART1 TC, the catchup ISR owns the full classifier + parser + CRC fold cadence at fixed 17-byte intervals (matching today's well-tuned schedule). IDLE doesn't run during a Chain reply window (no inter-slot wire idle in Fast Sync/Bulk by design).

### 10.6 Chain catchup ISR (Fast Last-slave replies)

The Chain catchup ISR runs at fixed-byte intervals on TIM2_CH1, owning **all** RX-side work during the predecessor reception window. This is structurally the same scheduling pattern as today's SysTick-driven `body_chain_catchup`, just moved to TIM2 CC1 and extended to do classifier + parser work alongside the chain-CRC fold.

**Arm time** (Chain reply scheduled):

    snoop_head     = rx_write_pos at parse-complete  # bytes before this are master's request
    chain_crc      = seed
    last_catchup_tick = fire_tick - FIRE_BIAS - walk_deadline_margin
    intervals_before_last = ceil((last_catchup_tick - now) / interval_ticks)
    CCR1 = last_catchup_tick - intervals_before_last × interval_ticks
    CCR2 = fire_tick - T_setup
    CCR3 = fire_tick - FIRE_BIAS
    enable CC1IE + CC3IE

`interval_ticks` = 17 × bit_time × 10 ≈ 56.7 µs at 3M (default) or 12 × ... ≈ 40 µs (tight option, §8.4).

**At every CC1 IRQ (Chain catchup):**

    if first_catchup:
        mask DMA1_CH7 HT/TC                # classifier ISR off for the rest of Chain
    walk classifier inline:
        for each new TS entry up to NDTR:
            apply window check, update anchor
            if start-bit edge: write BC[byte_idx]; byte_idx += 1
    parser drain:
        while parsed_idx < min(rx_write, bc_write):
            byte = RX[parsed_idx]
            advance parser state machine
            if parsed_idx >= snoop_head:
                chain_crc = crc16_update(chain_crc, byte)
            parsed_idx += 1
    if this was last interval:
        busy-wait until last_catchup_tick (== walk_deadline) reached
        # leaves trailing GUARD bytes for the post-fire walk
        return
    else:
        CCR1 += interval_ticks             # re-arm for next interval

The pattern mirrors today's `PeriodicCatchup → busy-wait → TxArmed` handoff (`dxl/state.rs`). The new piece is just steps 2-3 inside each ISR — the classifier walk and parser drain that were happening in DMA1_CH7 HT/TC are now folded in.

**Per-interval cost at 3M** (default 17-byte interval, mid-range edge density):

| Work | Cycles | µs |
| --- | --- | --- |
| Classifier walk (~51 edges) | ~408 | 8.5 |
| Parser drain (17 bytes × 50 cyc) | 850 | 17.7 |
| CRC fold (17 bytes × ~10 cyc) | 170 | 3.5 |
| ISR entry/exit + bookkeeping | ~50 | 1.0 |
| **Total per interval** | **~1480** | **~30.7 µs** |

Against a 56.7 µs cadence that's ~54% CPU during the Chain reception window — about the same as the Plain-mode (classifier + parser drain) cost during sustained RX. The work hasn't grown; it's just localized into one ISR cadence.

**At CC3 (fire ISR):**

    1. write USART1.CTLR1 |= TE        ← jitter-critical
    2. enable DMA CH4 (TX ring)
    3. drain residual: classify + parse + fold CRC for the GUARD bytes
       that arrived after last_catchup_tick (~2-5 bytes at 3M)
    4. patch CRC into TX buffer trailing slot (rides DMA prefetch — §8.2 of rx-timing doc)

Same "fire first, walk after" pattern as today. The residual is tiny because last_catchup_tick was the busy-wait exit — `walk_deadline_margin` is sized so only GUARD bytes can arrive in the [last_catchup_tick, fire_tick] window.

**At USART1 TC (reply complete):**

    1. unmask DMA1_CH7 HT/TC               # re-enable classifier ISR
    2. ...usual TC handling (release bus, apply pending baud/RDT, etc.)

**Plain replies** skip CC1 entirely — no chain CRC, no Chain catchup work. Classifier HT/TC stays armed; parser drains at classifier-end as in §10.5. CC1 arming is conditional on `ReplyState::Chain` at parse time.

**Defense against the "tiny catchup misses classifier disable" case.** The user-observed edge: if classifier HT/TC happens to latch right before CC1 fires (because classifier was about to walk on its own cadence), the catchup ISR's `mask` write happens *after* the latch — classifier IRQ is still pending and will run after catchup exits. Without further protection, classifier walks again immediately, eating cycles in the [CC1, CC3] window.

Mitigation in the CC1 ISR: after masking HT/TC, *clear pending* on DMA1_CH7 (`PFIC.IPRR.CH7 = 1`). This drops any latched-but-not-yet-serviced classifier IRQ. The catchup ISR has already done the classifier's job inline, so dropping the pending bit is correct — no work is lost.

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
| `irq.rs` | `on_usart1_idle` backdates and stamps `IdleAnchor` | `on_usart1_idle` kicks the parser; no tick math, no backdate constant |
| `rx_classifier.rs` | n/a | New: HT/TC walker for TS → BC, calls parser at walk-tail |
| `dxl/parser.rs` (or wherever the DXL decoder lives) | parses on main-loop poll from a 512 B ring | stateful decoder draining 64 B ring at every classifier walk; resumable mid-packet |
| `framing.rs` (today's IDLE-stamp queue + RXNE snapshot + framing FSM) | ~hundreds of LoC | **deleted** — no consumers after BC arrives |
| `board/bringup.rs` | TIM2 unconfigured | TIM2 init: free-running, CH4 IC, CH2 OC (hardware TX_EN), CH3 OC (fire IRQ), DMA CH7 |
| `idle_anchor.rs` | EXTI snoop instrumentation (9 atomics) | All EXTI snoop fields removed; framing relies on BC, not snoop |

The EXTI-pin instrumentation built on `firmware/dxl-2.0-transport` (the current branch) becomes obsolete — its job is fully subsumed by TIM2 IC capture, which is per-edge instead of first-edge-only. We can land one more bench capture on the EXTI path for diff context, then strip it in the migration.

---

## 13. Spike plan

Recommended order — each step independently verifiable on the dev board.

0. **PFIC bringup.** Write `INTSYSCR.INESTEN = 1` and set bit 7 of `PFIC_IPRIORx` for DMA1_CH1 (ADC) to put it at Low. Verify with a read-back; the rest default to High which matches the desired layout.
1. **PC1 dual-tap.** Configure AFIO for USART1_RX + TIM2_CH4 simultaneously. Verify USART RX still framed correctly (no regression) and TIM2 CCR4 captures on every PC1 falling edge. Use a scope probe on PC1 + a bench-side print of captured CCR4 values to compare.
2. **TS ring + classifier + stateful parser.** Wire DMA1_CH7 to CCR4 → 64-entry TS ring (circular). Implement `rx_classifier.rs` (HT/TC walker → BC). Shrink RX ring to 64 B. Convert the DXL parser to stateful drain-up-to-`min(rx,bc)`. Drive the parser from classifier-end + IDLE only (no RX HT/TC, no SysTick, no decimated counter). Verify at 115200, 1M, 3M that all packets are decoded correctly and BC entries land within ±1 tick of where the wire said they should.
3. **Software TX fire via TIM2_CH3 IRQ.** Wire CC3IE at High; implement the leaner ISR body (TE write + DMA CH4 enable). Measure mean CCR3-match → first-wire-bit latency, store as `FIRE_BIAS_TICKS`. Target: < 3 µs floor at 3M. Verify the §5.4 set-and-recheck path by forcing a small-RDT case (RDT=0 at low baud) that exercises the manual-fire branch.
4. **TX_EN via TIM2_CH2.** Wire CH2 → PC2 in OC mode (set-active-on-match). Verify TX_EN rises before CCR3's fire by `T_setup - FIRE_BIAS` ticks and falls correctly at TC.
5. **Chain catchup on TIM2_CH1.** Move today's SysTick-driven `body_chain_catchup` to TIM2 CC1 at 17-byte intervals (default; or 12-byte if going for §8.4 Option B). At first CC1 entry: mask DMA1_CH7 HT/TC + clear its pending bit. Each CC1 walks classifier → parser → CRC fold inline. Last interval busy-waits walk_deadline. Re-enable DMA1_CH7 HT/TC at USART1 TC. Drop the old SysTick scheduling and the DMA1_CH5 stage-1 ISR. Validate at 3M that snoop CRC matches the wire for all-foreign Fast Sync Read traffic.
6. **Catchup ISR cost measurement.** Profile real-world worst-case classifier-walk + parser-drain + CRC-fold runtime per interval. Confirm ≤ 75% of interval cadence at 3M (leaves headroom for USART1 errors, TIM2 fire, ADC preempt). If margin is tight, switch from 17-byte to 12-byte interval and shrink TS_LEN to 64 (§8.4 Option B).
7. **Integration: Fast Last-slave at 3 Mbaud.** Validate inter-slot coalesce gap — target: under one byte time. This is the test the SysTick design fails.
8. **Strip the old timing paths.** Once §7 passes, remove SysTick-CMP scheduling, IDLE-stamp queue, RXNE single-cell snapshot, framing-mode FSM, RXNEIE composer, `DXL_CHAR_TIME_TICKS` backdate constant, DMA1_CH5 chain-CRC stage-1 ISR, and EXTI snoop instrumentation. Anything that computes a tick from IDLE goes.

Each step is a self-contained commit per the project's "one reviewable unit" rule.

---

## 14. Open questions for the spike

- **AFIO dual-remap stability.** F1 documentation is explicit about peripheral-side input dual-tap working; V006 RM is less so. Step 1 confirms by direct measurement.
- **Capture filter setting.** Cap filter `(fCK_INT/8, N=8)` is a starting guess. May need to relax it if real bus glitches are wider than expected, or tighten it if too many spurious edges reach TS.
- **Fire-floor measurement.** Measure CCR3-match → first-wire-bit at 3M on bench. Estimate is ~2.5 µs; confirm and lock `FIRE_BIAS_TICKS`.
- **TS sizing + interval (§8.4 Option A vs B).** Default plan is 17-byte interval / TS_LEN=128. Option B (12-byte interval / TS_LEN=64) saves 128 B of SRAM at ~1% extra CPU. Decide based on memory pressure measured during integration.
- **Chain-CRC fold cost.** §10.6 estimates ~10 cyc/byte for CRC16 update. If the implementation lands closer to 20 cyc/byte, peak CPU during Chain RX climbs from ~54% to ~60%. Still well under the 75% spike target.
- **Classifier-pending-at-catchup-entry race.** §10.6 proposes "mask + clear pending" at first CC1 entry to defeat a classifier IRQ that latched right before catchup fired. Confirm the clear-pending write to `PFIC.IPRR.CH7` actually drops the latched IRQ on V006 (not just future ones).
- **`walk_deadline_margin` sizing.** The busy-wait exit in the last catchup leaves GUARD bytes for post-fire. Today's value (in `dxl_fast`) was tuned for SysTick scheduling; re-measure under TIM2 scheduling.

---

## 15. One-paragraph summary

> We move every DXL transport timing decision off USART IDLE and onto TIM2 input capture. PC1's dual remap feeds USART1_RX and TIM2_CH4 IC simultaneously; CH4 captures every falling edge into a 128-entry TS ring via DMA1_CH7. In Plain mode a `.highcode` window-classifier ISR runs at TS HT/TC, walking TS into a per-RX-index BC ring with a `[9·bit, 11·bit]` start-bit window test — constructive, self-healing within one byte, robust to glitches and overruns. RX (64 B) and BC (128 B) stay small because a stateful parser drains up to `min(rx_write, bc_write)` at the tail of every classifier walk; IDLE backstops small packets but only as a signal, never as a timing source. **No tick value comes from IDLE anymore** — wire-end is `BC[last_byte] + 10·bit_time`, per-byte precise at every baud. The framing-mode FSM, IDLE-stamp queue, RXNE snapshot, and `9 × BRR` backdate constant all delete. In Fast Last-slave Chain mode, the classifier ISR is disabled at first-catchup entry and the entire RX-side workload — classifier walk + parser drain + chain-CRC fold — runs in a periodic TIM2_CH1 catchup ISR at 17-byte intervals, the same scheduling pattern as today's SysTick-driven `body_chain_catchup`. The last interval busy-waits walk_deadline before exit; post-fire walks the GUARD residual and patches CRC into the TX buffer via DMA prefetch slack. TX fires from TIM2_CH3 compare → CC3IE IRQ → tiny ISR writing `USART1.CTLR1 \|= TE`, with CCR3 biased earlier by a measured `FIRE_BIAS_TICKS` so the wire bit lands on time. PC2 toggles in hardware on TIM2_CH2 compare to gate the bus driver before the TE write. Fire floor drops from ~5 µs to ~2.5 µs, comfortably under the 3.33 µs Fast-Last cap at 3 Mbaud. ADC stays on its existing DMA1_CH1 pump — the only new DMA channel is CH7 for TS. Priority layout matches today's: DXL-side IRQs at High, ADC at Low; the Chain catchup's protection against same-priority delay is structural (classifier disabled mid-Chain, last catchup scheduled before fire). CCR2/CCR3 wrap with TIM2's 16-bit CNT (period 1.365 ms at PSC=0); the §5.4 set-and-recheck pattern catches "fire armed in the past" cases. A LUT-walker alternative for byte timing (read RX[i], advance TS cursor by `edge_count[RX[i]]`) is documented in §11 and rejected for the silent-desync risk. Total sustained-RX CPU at 3M peak is ~55% (Plain) or ~54% (Chain, all-in-one catchup); during TX it's ~0% because the bus is being driven and no edges arrive. TS sizing is a CPU/memory knob (§8.4): default 128 entries / 17-byte interval, or 64 entries / 12-byte interval if SRAM tightens.

# DXL 2.0 RX Timing on the CH32V006

A design note for anyone bringing up a Dynamixel 2.0 slave on the CH32V006. You don't need prior DXL or RISC-V expertise — read it top to bottom.

**TL;DR.** A DXL slave has to start its reply exactly N microseconds after the host's request ends on the wire. N is configurable and can be as small as zero. The CH32V006's USART gives us two ways to detect "the request ended": a per-byte interrupt (RXNE) or a per-packet interrupt (IDLE). Neither one works across the full baud range — IDLE is too slow at low baud, RXNE costs too much at high baud. The fix is to switch between them based on baud and reply-delay, and to glue on a bag of tricks (highcode, DMA prefetch riding, SysTick CMP scheduling, priority pinning) to hit DXL 2.0's timing budgets all the way up to 3 Mbaud — which is the V006's USART ceiling. This doc walks through why, the formulas, the state machines, the ISRs, what each trick buys us, and an honest accounting of which baud/mode combinations actually pass the spec today.

---

## 1. What's on the wire

DXL is a half-duplex serial bus: one host, many slaves, one pair of wires. Slaves take turns. Every exchange looks like:

    HOST  --->  "request"  --->  bus
    bus   <---                <---  SLAVE  "status reply"

The question the slave has to answer: **after the host's request finishes, how long should I wait before replying?**

The answer is a per-slave setting in the control table called **Return Delay Time (RDT)**. Range: 0 to 508 microseconds, in 2 µs steps. The host writes it once during setup; the slave honors it on every reply.

Why precision matters: in Sync Read and Bulk Read, the host addresses many slaves at once and they reply back-to-back in pre-assigned time slots. A slave that's 200 µs late steps on the next slave's reply and corrupts the bus. RDT is a hard deadline.

In the **Fast** Sync/Bulk variants the constraint gets even tighter: every addressed slave's reply stitches into a single coalesced Status frame with **zero idle gap** between slots. The jitter cap for inter-slave hand-off is exactly one byte time. At 3 Mbaud that's 3.33 µs. The last slave additionally writes a CRC over the entire coalesced frame — including bytes it didn't generate.

So the slave needs to do two things, accurately:

1. **Detect the wire-end** — the moment the request's last byte finishes (specifically, when its stop bit clocks out).
2. **Schedule its reply** to start exactly RDT µs (plus any slot offset) after that moment.

This doc is mostly about (1) and the timing engineering needed to make (2) hit its budget at speeds up to 3 Mbaud.

---

## 2. The CH32V006 — what we have to work with

If you've used a Cortex-M0 or M4 before, the V006 will look familiar in shape but the corners are sharper than you might expect. Quick tour of the parts we lean on.

**The core.** QingKe V2A @ 48 MHz HCLK. RISC-V instruction set, two-stage pipeline, no FPU, no atomics worth depending on (the LR.W/SC.W instructions on V4 silently execute as plain LW/SW; V2 doesn't even have the A extension). Single-cycle SRAM access. Flash has 2 wait-states above 24 MHz — a function fetched cold from flash is meaningfully slower than the same function copied into SRAM. For ISRs in the hot path, this matters; §8.1 covers the workaround.

**Interrupt controller (PFIC).** Only **2 priority levels**, "High" and "Low" — you can't slot a third level between them. Nesting (one ISR preempting another) is opt-in via the INESTEN bit, and even then only "High preempts Low," never within a level. Same-priority IRQs **never preempt each other** — that's a guarantee we lean on heavily for mutual exclusion without locks.

**DMA1.** Single transfer per request, **no FIFO, no burst, no flow-controller mode**. We use three channels:

- **CH4** = USART1_TX, single-shot per reply, configured per fire.
- **CH5** = USART1_RX, circular, never disabled.
- **CH1** = ADC kernel pump (the only thing competing with USART1 for PFIC time).

The TX channel's lack of a prefetch buffer is what makes the CRC-patching trick (§8.2) work. Once TX is enabled, DMA reads byte N into the data register one byte-time before byte N+1 needs to land in the shift register. You can keep writing into the buffer ahead of DMA's cursor.

**USART1.** Where our DXL bus lives. Three flags we care about:

- **IDLE** — fires when the line has been idle for **9 bit-times** of mark. (Datasheets often say 10; on the V006 it's empirically 9.)
- **RXNE** — fires when a byte's stop bit clocks in.
- **TC** — fires when the last byte of a TX has fully shifted out (not just left the data register).

All three multiplex onto **one** USART1 vector. Our handler checks each flag in turn.

**SysTick.** 32-bit free counter at HCLK, separate from the USART. Compare-match (CMP) mode raises an IRQ when `CNT == CMP` on the **up-count** — see §8.3 for the gotcha if you write a CMP value that's already in the past. We use SysTick CMP as the slot-scheduling primitive: arm CMP at `wire_end + RDT + slot_offset`, fire TX when the IRQ lands.

**What we _don't_ have:**

- **TIM3** is the "SLTM" stripped variant — no IRQ, no prescaler, no one-pulse mode. It can drive DMA channels but USART1's `DMAT=1` makes USART issue its own DMA request the moment TXE asserts, racing past TIM3's gating. Unusable for slot timing.
- **TIM2** can't route to DMA1_CH4 at all.
- **TIM1** is in motor PWM service — its CNT wraps every ~50 µs with the PWM period, leaving no usable forward range for hundreds-of-microseconds slot deadlines (without invasive surgery — §13 talks about an OPM lever for closing the 3 Mbaud gap).

The net is: our slot timer is **SysTick CMP**, fired from software, not a hardware-only timer path. That's the structural choice driving everything else in this doc, and the source of the ~5 µs jitter floor we measure on Fast last-slave (§13).

---

## 3. The two USART flags

The USART hardware hands us two different "the request ended" signals. Each has its own personality.

### 3.1 RXNE — "a byte just arrived"

Fires once for each byte received. The interrupt happens **as the byte's stop bit clocks in**, so the timestamp is exactly the wire-end of that byte. Very precise.

    Wire:    [byte1][byte2][byte3]...
    RXNE:        ^      ^      ^         (per byte)

The catch is volume: at high baud you get a lot of these. At 1 Mbaud a stream of back-to-back bytes is 100 000 IRQs per second.

**V006 quirk: STATR.RXNE reads 0 inside the ISR.** When the RX DMA channel is enabled, DMA reads the data register _before_ the CPU enters the IRQ handler. The hardware flag clears as a side effect. Reading STATR.RXNE in the ISR always sees 0 — DMA wins the clear race. PFIC's pending bit latches per byte independently, so the IRQ **does** fire 1:1 per byte with zero overruns (bench-validated to 3 Mbaud). The takeaway: don't write `if statr.rxne() { ... }` — that branch never executes. Treat IRQ entry as "a byte arrived" and read the RX DMA cursor (NDTR) to know which position.

### 3.2 IDLE — "the wire went quiet"

Fires once after the line has been idle for 9 bit-times. One interrupt per packet, regardless of packet length. Cheap.

    Wire:    [byte1][byte2][byte3]________________
    IDLE:                                  ^         (~1 char-time after last byte)

The catch is latency: the interrupt fires roughly one character time **after** the wire actually went quiet. So if you use the IRQ-entry timestamp as the wire-end, you're systematically a character-time late.

We fix the _value_ by **backdating** — subtract `9 × BRR` HCLK ticks from the IRQ-entry timestamp. That's straightforward. But there's a deeper problem: even with a correct wire-end value, the slave doesn't _find out_ about wire-end until ~1 char-time after the wire went quiet. The next section is about why that matters.

---

## 4. Why one char-time of delay breaks low baud

Here's how the reply scheduler is structured:

    1. Wait until the wire-end timestamp shows up from the framing layer.
    2. Arm SysTick CMP at: wire-end + RDT.

The scheduler needs the wire-end timestamp to arrive **before** the deadline. With IDLE framing, the timestamp shows up roughly one character time after the wire-end happened. That's the **publish latency**, and it's baked into the IDLE flag — backdating fixes the value but not when you learn it.

What happens if publish latency exceeds RDT? The deadline is already in the past by the time the scheduler tries to use it. SysTick CMP can't be scheduled in the past (§8.3), so the only thing it can do is fire immediately, which technically violates the protocol.

**Concrete example: 9600 baud, RDT = 250 µs**

    char_time  = 9 bits / 9600 bps  =  937.5 µs
    publish latency (IDLE)          =  937.5 µs   (one char time)
    RDT (the deadline)              =  250.0 µs
    we are late by                  =  687.5 µs   ← every single reply

The slave can never make this deadline using IDLE framing. **At high baud the same math is fine.** At 1 Mbaud the character time is 9 µs; with RDT = 250 µs the slave has ~241 µs of slack. IDLE is plenty fast.

So the answer is baud-dependent: low baud needs the per-byte (RXNE) approach, high baud is happy with the per-packet (IDLE) approach.

---

## 5. The decision rule

Use IDLE whenever it can meet the deadline; use RXNE when it can't:

    char_time_us         = 9_000_000 / baud_hz       publish latency of IDLE
    rdt_us               = return_delay_2us × 2      the deadline budget
    pipeline_margin_us                               headroom for ISR/dispatch work

    use_rxne_framing = char_time_us + pipeline_margin_us > rdt_us

The `pipeline_margin_us` term accounts for the work between "IDLE ISR publishes timestamp" and "scheduler reads it." Measure on hardware and add a few µs of headroom. ~20 µs is a reasonable starting point on V006.

### 5.1 What does the rule pick?

With `pipeline_margin_us = 0` to keep the numbers clean (a real margin nudges the boundary slightly toward more RXNE):

    | baud      | char_time | RDT=0 µs | RDT=250 µs | RDT=508 µs |
    | --------- | --------- | -------- | ---------- | ---------- |
    |     9600  |  937.5 µs |  RXNE    |  RXNE      |  RXNE      |
    |    57600  |  156.3 µs |  RXNE    |  IDLE      |  IDLE      |
    |   115200  |   78.1 µs |  RXNE    |  IDLE      |  IDLE      |
    |  1000000  |    9.0 µs |  RXNE    |  IDLE      |  IDLE      |
    |  3000000  |    3.0 µs |  RXNE    |  IDLE      |  IDLE      |

Quick read:

- RDT=0 always forces RXNE — no budget for publish latency.
- At a typical RDT of 250 µs, IDLE works from 57600 baud upward.
- Slow buses (below ~17.7 kbaud) need RXNE even at the max RDT.

The two strategies sit at opposite ends of the trade-off naturally: at low baud, bytes are slow and per-byte interrupts are cheap in total; at high baud, packets are short and per-packet interrupts are all you need.

---

## 6. End-to-end timelines

### 6.1 IDLE mode (typical high-baud operation)

    Wire:        [b1][b2][b3] _ _ _ _ _ _ _ _ _ _ _ TX[r1][r2]
    RXNE:           x  x  x                                       (framing ignores these)
    IDLE:                            ^
                                     |
                          IRQ fires (one char-time after b3)
                                     |
                                     v
                          wire_end = now - 9*BRR ticks    ← backdate
                          push (bytes_added, wire_end) onto IDLE-stamp queue
                          ...
                          main loop parses, dispatcher calls request_complete(parsed_end)
                          scheduler arms SysTick CMP at wire_end + RDT
                          ...
                          SysTick CMP fires → fire_now() flips TX_EN, enables DMA CH4
                                                                       ^
                                                            wire-end + RDT

**One interrupt per request.** Cheap and clean.

### 6.2 RXNE mode (low-baud or zero-RDT operation)

    Wire:        [b1] _ _ [b2] _ _ [b3] _ _ _ _ TX[r1][r2]
    RXNE:           ^       ^       ^                          (per byte)
                    |       |       |
                    v       v       v
              overwrite single-cell snapshot with (rx_cursor, now)
                                    |
                                    `--- main loop parses; dispatcher reads
                                         the snapshot for parsed_end's position
                                         and gets the wire-end tick
                                         ...
                                         SysTick CMP → fire
                                                            ^
                                                wire-end + RDT

**One interrupt per byte.** At 9600 baud a 14-byte request is 14 interrupts spread across 14.5 ms — barely a blip. At 1 Mbaud it would be 14 interrupts in 140 µs — far too much, which is why the rule never picks RXNE at high baud.

### 6.3 Fast last-slave (with snoop CRC) — runs at every baud

DXL Fast Sync/Bulk Reads coalesce all addressed slaves' replies into one Status frame on the wire, zero idle gap between slots. The last slave appends a CRC covering the **entire** coalesced frame — header, every preceding slave's bytes, and its own. The bytes we didn't generate still have to be CRC'd, which means watching the wire during the predecessor slots and folding each arriving byte into a running CRC.

> **The per-byte RXNE sketch below was the original 2025 design and is
> kept here for context.** It has since been replaced by the layered
> snoop architecture in [dxl-fast-chain-crc.md](dxl-fast-chain-crc.md) —
> DMA TC + HT pre-walk for the bulk, a SysTick CMP at `fire − M` for
> the medium-residue, RXNE only for the slack-budget tail bytes when
> they exist (§14 of chain-crc.md), walk-at-fire for the final
> straggle. RXNE-for-snoop is OFF by default; framing remains the sole
> RXNE owner unless slack-budget math demands the tail-tier (rare for
> L ≥ 2 payloads). The state machine in §7.2 below has been superseded
> too — see chain-crc.md §6 + §14.6 for the current FSM and §14.5 for
> the timeline.

(Historical sketch — for the current implementation see chain-crc.md):

This snoop is independent of the framing mode from §5. It runs at every baud, including 3 Mbaud, alongside whatever framing source is publishing wire-end. They share the USART1 vector (§7.3) but their logic doesn't overlap.

    Wire (we're the last of three slaves):
        [request bytes] _ _ [slot 0 bytes][slot 1 bytes] TX[our bytes][CRC]
                         ^                              ^
                         |                              |
              parse done — we know we're slot 2     SysTick CMP fires
                         |                              |
                         v                              v
              build reply + 2-byte CRC slot     disable RXNE-for-snoop
              pre-arm DMA CH4 (count set,       set TX_EN high, enable DMA CH4
                EN=0)                           ← bytes start shifting out
              turn RXNE-for-snoop ON            CRC any bytes that arrived
              schedule CMP at:                    since the last RXNE entry
                wire-end + RDT + slot_offset    write the CRC bytes into the
                                                  TX buffer's trailing slot

Between parse and fire, every RXNE entry runs the snoop body: read the RX DMA cursor, CRC any new bytes since the last entry, advance the cursor. By the fire moment the running CRC is up-to-date except for a tail of bytes still in flight on the wire — those get caught up inside the fire ISR before the CRC bytes are written.

The order "enable TX, _then_ write CRC" looks wrong at first — you're turning on the bytes-going-out-the-wire machine before you've finished writing the bytes. The reason it works is the V006 DMA's one-byte prefetch, covered in §8.2.

---

## 7. State machines

Two small FSMs cover everything. They live in different parts of the layer and don't transition each other — the framing mode doesn't change when the scheduler is busy, and the scheduler doesn't care which mode the framing layer is in.

### 7.1 The framing-mode FSM

Two states. Transitions only happen when the host changes a control table field, so this FSM is at rest the vast majority of the time.

    states:    Idle, Rxne
    initial:   decide(default_baud, default_rdt) at boot
    transitions:
      Any → decide(new_baud, current_rdt)    on baud change   (deferred to TC, §11.1)
      Any → decide(current_baud, new_rdt)    on RDT change    (immediate, §11.2)

`decide` is pure — same inputs always produce the same output:

    decide(brr, rdt_2us):
        idle_latency_us = 9 × brr / 48          # 48 MHz HCLK; brr is ticks-per-bit
        rdt_us          = rdt_2us × 2
        if idle_latency_us + pipeline_margin_us > rdt_us:
            return Rxne
        else:
            return Idle

When this FSM enters Rxne, the framing layer needs USART1 RXNEIE on. When it enters Idle, the framing layer no longer wants RXNE — but the hardware bit can't just be cleared, because the Fast snoop path may also want it on. See §7.3.

### 7.2 The reply-scheduler FSM

The scheduler arms SysTick CMP to fire TX at the right moment. It's mode-agnostic — once it has a wire-end timestamp it doesn't care which framing source produced it.

    states:
      Idle          — nothing scheduled; SysTick IRQ off; DMA CH4 disabled
      WaitingPlain  — SysTick CMP armed for a non-snoop reply
                      (Ping, Read, Sync slot, Bulk slot, Fast Only/First/Middle)
      WaitingFire   — SysTick CMP armed for a Fast LAST-slave reply
                      (DMA CH4 pre-armed; snoop CRC accumulating; RXNE-for-snoop ON)

The split is functional: every reply that doesn't involve snooping predecessor slots' bytes lands in `WaitingPlain`, no matter how complex the slot math is. Only the Fast last-slave path needs `WaitingFire`, because only that path has a CRC to compute over bytes it didn't generate.

> **Superseded.** `WaitingFire` now expands to the
> `Chain { phase, ... }` substate machine in chain-crc.md §6 +
> §14.6. The "snoop CRC accumulating; RXNE-for-snoop ON" gloss is
> only accurate when the slack-budget math in §14.4 demands the
> tail-tier; for L ≥ 2 cases the chain accumulates via DMA TC/HT +
> SysTick switch with no RXNE involvement.

Transitions:

    Idle → WaitingPlain   any non-snoop reply gets scheduled
                          - pre-configure DMA CH4 (count + source, EN=0)
                          - set SysTick CMP at: wire-end + reply_delay
                          - no RXNE state change

    Idle → WaitingFire    Fast last-slave reply gets scheduled
                          - pre-configure DMA CH4 same as above
                          - turn RXNE-for-snoop ON (§7.3)
                          - initialize CRC accumulator at "RX cursor right now"
                          - set SysTick CMP at: wire-end + reply_delay

    WaitingPlain → Idle   CMP fires:
                          - enable TX (DMA CH4 + TX_EN high) → bytes start shifting
                          - no other work

    WaitingFire → Idle    CMP fires:
                          - turn RXNE-for-snoop OFF
                          - enable TX → first byte starts shifting
                          - CRC any bytes that arrived since the last RXNE entry
                          - write the CRC bytes into the trailing slot of TX buffer
                          (order is load-bearing — see §8.2)

    Any → Idle (cancel)   triggered by:
                          - TX completes (USART1 TC interrupt — every reply ends here)
                          - a new send arrives with old state still lingering
                          - the parser sees a packet we can't honor mid-flight

Cancel is idempotent — running it from Idle is a no-op. Every send routine pre-cancels just in case, to avoid a stale CMP re-firing into a fresh reply's buffer. The cost is one CMP-disable write per send, which is negligible.

When cancel runs in `WaitingFire`, "turn RXNE-for-snoop OFF" has to go through the composer (§7.3), not blanket-clear the hardware bit — otherwise it would also kill framing's per-byte publish when framing is in Rxne mode.

### 7.3 Sharing the RXNE interrupt

RXNEIE has two independent owners:

    framing-mode publisher    active iff framing FSM == Rxne
    snoop tail-pre-walk       active iff Chain { rxne_tail_at } says so
                              (see chain-crc.md §14.4 for when this is set)

These are orthogonal — any combination is possible:

    | framing | snoop tail | RXNEIE | handler body runs                       |
    | ------- | ---------- | ------ | --------------------------------------- |
    | Idle    | off        |  off   | (IRQ disabled)                          |
    | Idle    | on (tail)  |   on   | snoop tail-walk only                    |
    | Rxne    | off        |   on   | publish per-byte timestamp only         |
    | Rxne    | on (tail)  |   on   | publish + snoop tail-walk               |

The snoop owner's role here is narrower than the original design — it
only arms during the predecessor's last `tail_bytes` of a Fast Last
reply when `start_fast_after` determined the post-fire walk alone
wouldn't fit slack (chain-crc.md §14.4). For Fast Sync with L ≥ 2 the
snoop owner stays off entirely; framing remains the sole RXNE owner.

The hardware has one enable bit. Simple way to compose two owners onto one bit: each owner tracks its own boolean, and on every change the layer recomputes `final = framing_wants OR snoop_wants` and writes that to RXNEIE. Conceptually:

    set_rxne_owner(owner, on):
        owners[owner] = on
        USART1.CTLR1.RXNEIE = owners.framing OR owners.snoop

Avoids the foot-gun where one owner disables the IRQ while the other still needs it. Don't ever write the hardware bit from one site without going through the composer.

The handler runs both branches in sequence, each guarded by its own FSM check:

    on USART1 entry (when RXNE is the cause):
        if framing FSM == Rxne:
            publish (rx_cursor_now, systick_now) to the single-cell snapshot
        if scheduler FSM == WaitingFire:
            CRC new bytes from RX ring (head = NDTR-derived) into the accumulator

When only one branch matches, the other is a single state-check skip. When both match, both run in one IRQ entry — no extra interrupts, no double-handling.

The composer is touched from two contexts: the main loop (when the host writes RDT and the framing FSM may flip; §11.2) and the USART1 TC handler (when a deferred baud change applies; §11.1). Both can race with the USART1 RXNE handler that reads the FSM states. As long as the composer's OR-and-write is atomic with respect to RXNE-handler entry, the composition is safe — the handler only reads the FSM states, never writes them.

---

## 8. The bag of tricks

This is the section that earns the rest of the design its timing budget. Each trick is small on its own; together they get the V006 from "barely talks DXL at 1 Mbaud" to "passes the DXL 2.0 spec at all standard baud rates plus 2 Mbaud Fast, and brushes 3 Mbaud Fast within ~2 µs." §13 has the honest accounting.

### 8.1 Highcode: living in SRAM, not flash

Default Rust on V006 puts all `.text` in flash. Flash has 2 wait-states above 24 MHz, so each fetch on a cache miss costs extra cycles. For a tight ISR doing ~50 instructions, the overhead adds up to roughly a microsecond just in fetch latency.

`qingke-rt`'s `highcode` Cargo feature adds a `.highcode` section to the linker script and copies it from flash to SRAM at boot, much like `.data`. Tag any function you want SRAM-resident with `#[link_section = ".highcode"]` and it runs at single-cycle fetch speed.

What we put in `.highcode`:

- Every Rust-defined ISR body in the DXL hot path: `on_systick`, `fire_now`, `accumulate_snoop`, `ring_crc`, `patch_crc`.
- The qingke-rt trap stubs (the assembly that decodes the IRQ vector and the wrappers around each Rust ISR). Without these in SRAM, you eat a flash hit on every IRQ entry regardless of where the body lives. Enabling the feature pulls these in automatically.
- The PFIC vector tables (also moved automatically). `mtvec` is rewritten by `_setup_interrupts` to point into SRAM.

What it cost on a real build (rev_b app): `.highcode = ~1.5 KB`, total RAM usage ~4 KB of 8 KB. Plenty of headroom.

What it bought: SysTick CMP → TX_EN-high shrank from ~7 µs to ~5 µs at 3 Mbaud (~2 µs saved). Not enough on its own to hit the 3.33 µs spec, but the largest single lever we've pulled.

### 8.2 Riding DMA's one-byte prefetch

This is the trick that lets the Fast last-slave path patch its CRC bytes into the TX buffer _after_ TX is already shifting bytes out.

The V006 RM §14.6 says USART TX DMA reads one byte at a time from RAM into the data register (TDR) on each TXE event. TDR holds it until the shift register accepts it. There's exactly one byte of look-ahead — no FIFO, no burst, no flow controller, no prefetch buffer beyond TDR.

So for TX buffer position M, DMA reads it from RAM at `(M − 1) × byte_time` after TX is enabled. If you can write byte M into the buffer before that moment, it lands correctly on the wire.

How we use this: at the SysTick CMP fire moment, enable TX **first**, then compute the snoop CRC over our payload, then write the two CRC bytes into the trailing slot of the TX buffer. The fire-moment jitter cap is just the time from CMP IRQ entry to DMA EN write + TX_EN GPIO toggle — a handful of instructions. The CRC compute can take tens of microseconds and still finish before DMA needs the result.

Bench validation (`tx_dma_race`, 2026-05-24) swept payload lengths 1 to 128 at 3 Mbaud, captured the wire bytes via a separate serial bridge, and confirmed every length passes — including the pathological 2-byte case where the deadline is exactly one byte time (3.33 µs).

Wrap-aware CRC: the snoop window can span the RX ring boundary. The accumulator handles it in two passes when `head < snoop_pos`:

    if head >= snoop_pos:
        crc16_continue(seed, ring[snoop_pos..head])
    else:
        mid = crc16_continue(seed, ring[snoop_pos..])
        crc16_continue(mid, ring[..head])

Unit-tested in `dxl_fast.rs::tests::ring_crc_*`.

### 8.3 SysTick CMP — the up-count match gotcha

The V006 RM §6.5.4 spells out: `CNTIF` sets only on `CNT == CMP` during an **up-count**. If you write `CMP < current CNT`, you're not "instantly past the match" — you're scheduled for the next wrap, which on a 32-bit counter at 48 MHz is **89 seconds away**.

Our scheduling pattern handles it:

    set_cmp(deadline)
    enable_systick_irq()
    if now() − idle_tick >= needed_ticks:
        # we crossed the deadline between arming and the post-check — fire now
        on_systick()    # manual call into the same body the IRQ would have run

This "set + recheck + manual-fire" loop is in every Fast path arm site (`start_plain_after`, `start_fast_after`). Without it, a slot deadline that lands microseconds past the arming moment would silently sleep for 89 seconds.

Related V006 quirk: qingke-rt v2 init enables STIE and `mstatus.MIE`, but does **not** set the PFIC IENR1 bit for SysTick. Until that's set, `CNTIF` latches but the IRQ never fires. Bringup hit this. Production firmware now sets it explicitly during init.

### 8.4 Priority pinning: USART1 = SysTick

Two PFIC priority levels and three IRQs to place: USART1 (the DXL bus), SysTick (the slot fire), and DMA1_CH1 (the ADC kernel pump). We need both DXL IRQs above ADC, but we don't have a level between them.

The trick: put **both USART1 and SysTick at High** and use PFIC's same-priority guarantee:

> Same-priority IRQs never preempt each other.

This gives mutual exclusion across the two ISRs that share state — scheduler FSM, snoop CRC accumulator, RXNEIE composer — without writing any locks. The cost is that the worse of the two ISR runtimes shows up as latency for the other; current measurements:

- `on_systick` (Fast fire ISR): ~2 µs entry-to-fire post-highcode.
- `on_usart1_idle` (IDLE backdate + queue push): ~1.5 µs.
- Combined back-to-back worst case: ~3.5 µs — still under one byte time at 2 Mbaud (5 µs).

The ADC pump runs at Low and **can** be preempted by either DXL IRQ once INTSYSCR.INESTEN is set. ADC jitter is comfortable with that.

### 8.5 The IDLE-stamp queue with cumulative counter

The IDLE ISR pushes `(bytes_total_so_far, idle_tick)` into a drop-oldest ring (depth 4). The byte counter lives **outside** the ring so it survives evictions when traffic outpaces the dispatcher's polling.

Why this matters: Sync Read traffic can stack three IDLE events before the main loop polls (each sibling slave's Status frame ends with an IDLE). The ring eats them in order. If a fourth arrives and the dispatcher hasn't drained, drop-oldest pops the oldest entry. The cumulative counter inside that popped entry is **not** lost — it stays alive in the standalone atomic, so subsequent stamps still record the correct running total.

Without this split, recovery from queue overflow would lose track of "how many bytes have we seen since boot," and stamp lookups would silently mismatch.

The match logic in `pop_matching(parsed_end)` walks the ring:

- entry's `bytes` < `parsed_end` → stale, drop and continue.
- entry's `bytes` == `parsed_end` → take, return tick.
- entry's `bytes` > `parsed_end` → future stamp, leave for later, return None.

### 8.6 Mode-specific backdating

USART IDLE fires 9 bit-times _after_ wire-end. We subtract `9 × BRR` HCLK ticks from `systick::ticks()` at IRQ entry to recover the wire-end timestamp:

    idle_tick = systick::ticks().wrapping_sub(DXL_CHAR_TIME_TICKS)

This is correct only for **IDLE** framing. RXNE framing must **not** backdate — RXNE fires at the stop-bit moment, which is wire-end itself. Mixing them up shifts every Rxne-mode timestamp one char-time too early and the slave fires its reply that much too soon. Easy bug to write.

### 8.7 Pre-arm at parse, fire at deadline

DMA CH4's `EN` bit gates whether bytes go out. Splitting "configure" from "enable" lets us do the slow work (set count, set source, clear stale TC) at parse time, and have the fire path be two writes: `DMA1.CH4.CR |= EN; GPIOx.BSHR = TX_EN_bit`. Pre-caching those two register values at arm time is a further lever we haven't pulled yet (§13).

### 8.8 RXNE for snoop without polling the flag

Because STATR.RXNE always reads 0 in DMA mode (§3.1), the snoop accumulator doesn't gate on the flag. It uses NDTR (DMA's remaining-count register) as the head pointer:

    write_pos = (RX_BUF_LEN as u16).wrapping_sub(NDTR) & RX_MASK

and CRC's `ring[snoop_head..write_pos]` (with the wrap split from §8.2). If two bytes arrive close enough that one IRQ entry serves both (PFIC pending bit latches per byte, but ISR entry can coalesce when the handler was already running), NDTR's value at entry-time still reflects both — no byte is lost.

---

## 9. Interrupt responsibilities

| Priority | IRQ | What it handles | Where it lives |
| --- | --- | --- | --- |
| High | USART1 | IDLE + RXNE + TC (all multiplexed) | `.highcode` (SRAM) |
| High | SysTick CMP | Reply deadline → fire TX | `.highcode` (SRAM) |
| Low | DMA1_CH1 | ADC kernel pump (20 kHz) | flash (cold path) |

### 9.1 The USART1 vector

Three flags can wake it; handle each in turn.

**IDLE flag** (line just went quiet):

    if framing FSM == Idle:
        idle_tick = systick::ticks() − DXL_CHAR_TIME_TICKS    # backdate
        compute bytes delta since last DXL_RX_WRITE_POS
        DXL_RX_WRITE_POS = current NDTR-derived position
        push (delta_bytes, idle_tick) onto idle_ring
    else (framing FSM == Rxne):
        nothing to push — RXNE entries already published wire-end
        clear the IDLE flag so it doesn't re-fire forever

**RXNE flag** (a byte arrived — we entered because at least one owner wanted RXNEIE on; §7.3):

    if framing FSM == Rxne:
        publish (rx_cursor_now, systick::ticks()) to single-cell snapshot
    if scheduler FSM == WaitingFire:
        catch the snoop CRC up to the current RX DMA cursor

The RXNE branches don't gate on STATR.RXNE (§3.1). Both branches' guards run on every USART1 entry; either or both can do work.

**TC flag** (the reply just finished shifting out):

    set TX_EN low — release the bus to the host
    clear TC, disable TC IRQ
    disable DMA CH4 (TX channel)
    clear TX buffer
    scheduler cancel → Idle (clears the snoop owner of RXNEIE via §7.3)
    if a baud change is pending:
        apply the new BRR
        recompute DXL_CHAR_TIME_TICKS = 9 × BRR
        recompute framing mode (may flip framing's RXNE owner via §7.3)
    if a reboot is pending:
        software reset

The TC handler is where deferred state changes apply. We can't safely retune the UART or reset the chip while a reply is still shifting out, so anything timing-sensitive that the dispatcher kicked off mid-reply queues here and applies once TC says we're idle.

### 9.2 The SysTick CMP vector

Single purpose: fire the reply at the slot deadline. Body is in `dxl_fast::on_systick`:

    clear CMP match flag, disable SysTick IRQ
    read scheduler FSM
    if WaitingPlain:
        set FSM = Idle
        fire_now()    # enable TX_EN + DMA CH4
    if WaitingFire:
        turn RXNE-for-snoop off (composer)
        set FSM = Idle
        fire_now()                  # fire first — jitter-critical
        accumulate any residual snoop bytes
        patch CRC into TX buffer's trailing slot

The fire-first order in `WaitingFire` is the §8.2 trick. Bookkeeping after fire is fine because DMA hasn't reached the CRC slot yet.

---

## 10. The consumer side

When the dispatcher finishes parsing a request, it asks the framing layer "when did this request's last byte hit the wire?" via `request_complete(parsed_end)`. The lookup branches on the current framing mode.

**In IDLE mode** the framing layer scans the queue for the entry whose cumulative byte count matches `parsed_end`, returns that tick, and pops it. If no entry matches, return None — either the IDLE flag hasn't fired yet, or the queue overflowed and the matching entry was dropped.

**In RXNE mode** the framing layer reads the single-slot snapshot of the most recent byte's `(position, tick)`. The consumer checks whether the position matches `parsed_end`:

- If yes, return the tick.
- If no (more bytes arrived between parse and lookup), return None.

Returning None is graceful degradation. Slot-timed callers (Sync, Bulk, Fast Read) **must skip** when the answer is unknown — they can't safely fire without a precise timestamp. Direct unicasts **may** proceed with an immediate fire, accepting the timing slip.

### 10.1 Why a queue for IDLE but a single cell for RXNE

Different timing regimes, different simplest solution.

- **IDLE mode** runs at high baud. Packets can arrive faster than the main loop polls — three Sync Reads in quick succession will pile three IDLE timestamps before the dispatcher gets to any of them. A small FIFO absorbs the burst. Depth 4 is plenty.
- **RXNE mode** runs at low baud. Inter-byte time is at least ~17 µs (at 57600) and grows to ~100 µs at 9600. The dispatcher's parse-and-lookup is sub-microsecond. By the time the next byte's IRQ fires, the previous packet's lookup has long since completed. No concurrent stacking, so a single cell suffices.

### 10.2 Reading the single cell without tearing

A wrinkle: the consumer needs to read both `position` and `tick` together, and the read must be atomic — if the IRQ fires mid-read, half-old half-new is corrupt.

Trick: pack both into one `AtomicU32`:

    high 16 bits: position    (RX ring index, ring ≤ 64 KB)
    low  16 bits: low half of SysTick at IRQ entry

The publisher writes the whole word in one `sw`; the consumer reads it in one `lw`. On the V006 (RV32EC), `AtomicU32::store(Relaxed)` and `load(Relaxed)` lower to plain `sw`/`lw` — naturally atomic for aligned access. No critical section needed.

The consumer reconstructs the full SysTick value by stitching the saved low half onto the current SysTick's high bits, watching for wrap (the 16-bit window is ~1.4 ms at 48 MHz — wider than any sane parse-to-lookup gap).

---

## 11. When to recompute the mode

The framing mode is a pure function of (baud, RDT). It changes only when one of those changes. Two trigger points.

### 11.1 Host writes BAUD_RATE

Can't retune the USART immediately — the Status reply telling the host "OK, value accepted" hasn't been sent yet. Retuning mid-reply garbles it.

    1. Dispatcher parses the Write packet.
    2. Control table is updated.
    3. Dispatcher tells the bus "baud change pending — here's the new BRR."
    4. Bus stashes DXL_BAUD_PENDING_BRR. No hardware change yet.
    5. Status reply queues and TX shifts at the OLD baud.
    6. Reply finishes draining → TC IRQ.
    7. TC handler applies new BRR, updates DXL_CHAR_TIME_TICKS = 9 × BRR,
       recomputes the framing mode, and (if the mode changed) flips
       framing's RXNE owner via the composer (§7.3).

Without the deferral, the host's last reply arrives garbled.

### 11.2 Host writes RETURN_DELAY_TIME

No hardware to reconfigure — RDT is just a number:

    1. Dispatcher parses the Write packet.
    2. Control table is updated.
    3. Dispatcher tells the framing layer "RDT is now X."
    4. Framing layer publishes the new RDT, recomputes the mode, and (if it
       changed) flips framing's RXNE owner via the composer (§7.3) — all in
       main-loop context, before the next IRQ.
    5. Status reply is sent honoring the new RDT and (possibly) the new mode.

### 11.3 Both fields in one packet

The order works out naturally:

- RDT applies immediately and triggers a recompute.
- BAUD is staged and triggers a second recompute at TC.

The TC recompute sees the already-updated RDT (RDT step ran first) and the freshly-applied baud, so the final mode is correct. Write order doesn't matter.

---

## 12. The RX DMA channel

DMA1_CH5 sits behind everything and captures every byte on the wire into the ring. The framing layer never touches it:

    Mode:      circular, memory-increment, never disabled
    Size:      power-of-two ring (1024 bytes on rev_b; minimum 256)
    Source:    USART1 data register
    Trigger:   per-byte from USART1 RXNE

Two reasons it's always on:

1. **Foreign traffic doesn't crash anything.** Bytes from other protocols sharing the bus (bootloader handshakes, USB bridges, debug streams) land in the ring but never get parsed — harmless.
2. **It's the source of position values.** Both timestamp publishers (IDLE-queue push, RXNE single-cell publish) and the snoop accumulator read NDTR to derive the ring cursor. No per-byte software bookkeeping needed.

TX uses CH4, single-shot per reply, configured per-fire and torn down in the TC handler.

---

## 13. Where the V006 sits on the DXL 2.0 spec

Honest accounting of what we hit and what we miss as of 2026-05-27.

**Hardware constants:**

- 48 MHz HCLK, 1-cycle SRAM, 2-wait flash above 24 MHz.
- ~5 µs from SysTick CMP → TX_EN GPIO high, post-highcode (V006 V2 structural floor).
- USART1 + SysTick at same High priority (no preemption between them).

**Per-path fire latency** (`firmware/ch32/src/statics.rs`, ticks at 48 MHz HCLK):

    | path                            | const                     | initial bootstrap |
    | ------------------------------- | ------------------------- | ----------------- |
    | plain reply (`start_plain_after`) | `TX_PLAIN_LATENCY_TICKS`  | 144 (~3.0 µs)     |
    | Fast chain (`start_fast_after`)   | `TX_FAST_LATENCY_TICKS`   | 144 (~3.0 µs)     |

The Fast path does extra work before `fire_now` (DMA TCIE off, phase transition, dbg pin, snoop-CRC scaffolding) so its effective latency is larger. The split lets each path's wire-edge land at the scheduled tick — a single knob forces one path to overshoot. Per-chip values come from #52; the per-chip `clock_fine_trim_us` residual lives in a separate atomic and is summed at the fire site.

**Per-baud accounting:**

    | baud  | byte time | plain DXL (RDT 250 µs) | Fast last-slave (jitter cap = 1 byte) |
    | ----- | --------- | ---------------------- | ------------------------------------- |
    | 9600  |  1042 µs  | RXNE framing (auto)    | (rare config; not bench-tested)       |
    | 57600 |   174 µs  | passes                 | passes                                |
    | 115200|    87 µs  | passes                 | passes                                |
    | 1 M   |    10 µs  | passes                 | passes (5 µs floor < 10 µs cap)       |
    | 2 M   |     5 µs  | passes                 | **6 µs gap, cap 5 µs — over spec**    |
    | 3 M   |  3.33 µs  | passes                 | **5 µs gap, cap 3.33 µs — over spec** |

The 2 Mbaud and 3 Mbaud Fast last-slave numbers are scope captures on rev_b with an INJ V203 as the predecessor slave. The bus _does_ go idle between slots in those cases — visible as an extra IDLE on the injector's listener — which breaks coalesce. The host parses each slot as a separate Status frame, which is wrong but recoverable; some hosts tolerate it, some don't.

**Where the 5 µs goes:**

- PFIC trap entry: ~1 µs (irreducible without core changes).
- Fire ISR body (post-highcode): ~1.5 µs.
- DMA EN write → TX_EN GPIO toggle → first start bit on the wire: ~2 µs.

Total ~5 µs is the structural floor for an ISR-driven fire on V006 V2 @ 48 MHz. Highcode bought ~2 µs of that. The remaining levers, in increasing complexity:

1. **Pre-cache TX_EN BSHR + DMA CR|EN values at arm time.** Makes `fire_now` two stores with no computation. ~0.3 µs.
2. **Defer bookkeeping after `fire_now`.** Cleanup (clear FSM stage, clear RXNE snoop owner) happens before `mret` but after TX is already shifting. ~0.5 µs.
3. **Hardware-driven fire via TIM1 OPM.** Configure a TIM1 OPM channel at arm-time with CCR = fire_tick, driving TX_EN through TIM1's output toggle and the DMA channel through TIM1's update event. Bypasses the ISR entirely — fire moment is hardware-clocked. The only path that has a realistic shot at <3.33 µs on V006 V2. Complex because TIM1 is shared with motor PWM.

Until lever 3 lands, the V006 cap for Fast Sync/Bulk Read **as last slave** is effectively 1 Mbaud. Slot 0, middle, and Only positions all use plain `WaitingPlain` scheduling timed against the master's request IDLE — those work at all baud rates because there's no inter-slot jitter cap.

---

## 14. Failure modes and edge cases

### 14.1 Foreign packet arrives mid-mode-change

A baud change defers to TC. If a foreign packet arrives between the host's BAUD_RATE write and the TC IRQ, it gets framed under the **old** mode. Its timestamp may not survive the lookup after the flip. Acceptable: the host has to re-sync after a baud change anyway, and any in-flight foreign traffic during a baud transition is best-effort.

RDT changes happen in main-loop context — they can't interleave with the ISRs. The flip is observed by the **next** USART1 IRQ onward.

### 14.2 Lookup returns None

Two ways:

- **IDLE mode:** no queue entry matches `parsed_end`. Either the IDLE flag hasn't fired yet (we got ahead of it), or the queue overflowed and the matching entry was dropped.
- **RXNE mode:** extra bytes arrived between parse and lookup, so the single-cell position no longer matches.

Caller decides: slot-timed callers skip; direct unicasts may proceed with an immediate fire. Either way, no protocol-violating "wrong timestamp" gets used.

### 14.3 Two bytes arrive during one IRQ entry

Can happen if the handler was already running for a previous byte. Fine: PFIC's pending bit latches per byte, so a second entry follows. The first entry publishes for the earlier byte; the second for the later. Position values come from NDTR each time, so coalescing reduces interrupt count but never loses byte tracking.

### 14.4 Mode-specific backdating — don't mix them up

- **IDLE mode** backdates by `9 × BRR` ticks to recover wire-end from a delayed IRQ.
- **RXNE mode** does **not** backdate. RXNE fires at the stop-bit moment — IRQ entry is wire-end.

Applying the IDLE backdate to RXNE timestamps silently shifts wire-end by one char-time and the slave fires its reply one char-time too early. Easy bug to write and easy to miss.

### 14.5 Hostile (baud, RDT) combos

The decision rule prevents a bad config from breaking the deadline, but it can't prevent bad configs entirely. RDT=0 at 1 Mbaud forces RXNE framing at high baud → 100 000 RXNE IRQs per second under heavy traffic. The implementation will keep working but it's a load regime it isn't designed for. On the host to avoid.

### 14.6 TC racing a pending CMP

The TC handler runs `scheduler::cancel`, which disables SysTick CMP IRQ. If a CMP match landed before TC ran and the SysTick IRQ is already pending, it will still execute (same-priority can't preempt, but pending state survives). The `on_systick` body checks the FSM and finds Idle (cancel already ran), so it returns without firing — no harm done.

---

## 15. One-paragraph summary

> At high baud the USART's IDLE flag publishes wire-end fast enough to meet RDT; at low baud it shows up too late and we fall back to per-byte RXNE timestamps. A small decision rule, `char_time_us + pipeline_margin_us > rdt_us`, picks between them based on the host-configured baud and RDT. The framing FSM only changes when the host changes baud (recompute at TC, after the reply drains) or RDT (recompute immediately). The reply scheduler is a separate FSM (Idle / WaitingPlain / WaitingFire) that arms SysTick CMP for the fire moment; WaitingFire is reserved for Fast last-slave replies with their snoop CRC over predecessor slots' bytes. Both FSMs share the RXNE interrupt via an OR-composer because each can independently want it on. The V006 specifics — flash wait-states, two PFIC priority levels, DMA single-byte prefetch, SysTick up-count match semantics — drive the rest of the design: highcode for ISR bodies, fire-before-CRC-patch for Fast last-slave, set-and-recheck for SysTick CMP, equal-priority pinning of USART1 and SysTick for lockless mutual exclusion. The whole stack passes DXL 2.0 spec through 2 Mbaud plain DXL and 1 Mbaud Fast; 2 Mbaud and 3 Mbaud Fast last-slave miss the inter-slave jitter cap by 1–2 µs, and the lever to close it (hardware-driven fire via TIM1 OPM) is documented but not yet implemented.

# Pirate Hardware Timing

The uart-pirate's job is to be a more accurate, more precise, and more reliable
ruler than anything it measures. This doc is the architecture contract for how
that's wired — peripheral assignments, the time domain, the per-byte stamp
pipeline, and the validation principles that keep measurements
hardware-grounded.

The chip is the MuseLab nanoCH32V203 (V203C8T6, QingKe V4B @ HCLK = 144 MHz).
Wire is single-pair half-duplex UART up to 3 Mbaud.

## 1. Time domain

One clock, one base, no software wrap counter on the hot path.

- **TIM2** — PSC=0, ARR=0xFFFF, free-running at HCLK. Low 16 bits.
  MMS=COMPARE_PULSE: TRGO pulses on every CC1IF set (§3 IC capture).
- **TIM3** — free-running, PSC=0xFFFF, ARR=0xFFFF, no slave mode.
  Increments once per HCLK/65536 cycles = same rate as TIM2 wraps. High
  16 bits.

Combined stamp:

    tick32 = (TIM3.CNT << 16) | TIM2.CNT

Phase-locked at init: TIM3.CEN is asserted before TIM2.CEN inside a
critical section, so TIM3's prescaler leads TIM2's counter by the AHB
write gap (~3–6 cycles). TIM3.CNT therefore increments a few cycles
*before* each TIM2 wrap. The 1-cycle race window where TIM3 has just
ticked but TIM2 hasn't yet wrapped is detected and corrected as
described in §3.3. The `read_tick32()` read protocol (§3.3) absorbs
this offset via its retry-on-wrap loop — peripheral reads span ~10
cycles, comfortably wider than the 1-cycle race.

| Quantity     | Value           |
| ------------ | --------------- |
| Resolution   | 6.94 ns/tick    |
| Wrap period  | ~29.8 s         |
| Host wire    | `tick32`, always |

`HZ?` replies `144`. Host tools query it at runtime — no compile-time constant
on either side.

## 2. Measurement contract

Production firmware on the V006 answers a single question per packet: "when
did the request end so I can schedule my reply?" — it emits one stamp
(`packet_end_tick`) and discards the rest.

The pirate answers a different question: **"when did every byte on the wire
start?"** Its product is a per-byte stamp stream — `(byte_value, start_tick,
flags)` tuples for every received byte. The RX pipeline below is shaped
around that contract.

## 3. Wire-side capture (RX)

Two GPIO pins bridged externally to one wire, four peripheral block
taps, zero CPU per edge.

    PB10 ──┬── USART3_TX (AF PP) ◄─ DMA1_CH2 ◄─ TX burst (per-send, §5)
           │
           └── TIM2_CH3 pin ─┐
                             ├── TI1 = TI1 XOR TI2 XOR TI3 (CTLR2.TI1S=1)
    PA0 ─── TIM2_CH1 pin ──┘     │
    PA1 ─── TIM2_CH2 pin ──┘     │
    (both pulled-down)           │
                                 ▼
                       TIM2_CC1 IC (CCMR1.CC1S=TI4, CCER.CC1P=1 falling,
                                    IC1F per-baud)
                                 │
                                 ├──► DMA1_CH5 ─► falling_lo (low 16)
                                 │
                                 └──► TIM2 TRGO pulse (MMS=COMPARE_PULSE)
                                            │
                                            ▼
                                  TIM3_CC1 IC (CCMR1.CC1S=TRC, CCER.CC1P=0
                                               rising-edge of TRGO pulse)
                                            │
                                            └──► DMA1_CH6 ─► falling_hi (high 16)
           │
           │   external jumper:  PB10 ───┬──── wire
           │                             │
           │                             └──── PB11
           │
    PB11 ──── USART3_RX (input pullup) ─► DMA1_CH3 ─► rx_ring (DMA-populated)

USART3 runs full-duplex: PB10 drives the wire via AF push-pull TX, PB11
listens via input-with-pullup RX, and the bench jumper ties both pins to
the same wire so on-chip RX sees both our own TX echo and any remote
replies. HDSEL would internally tri-state RX during TX and is not used
here. (Push-pull on PB10 vs the spec-correct open-drain is bench-only:
the loopback wiring has no transceiver and only PB11's ~30 kΩ internal
pull-up, giving τ ≈ 500–900 ns rise time and RX mis-sample at ≥ 2 Mbaud
under OD — PP drives both edges actively.)

PA2/PA3 were the original choice (USART2 default mapping) but boot-wedge
on the MuseLab nanoCH32V203: PA3 has R10 = 10 kΩ to 3V3 on the board
(SD-card CS pull-up). With the wire connected idle-high before USB plugs
in, current trickles via wire → series-R → PA3 → R10 → chip 3V3 rail and
parasitically charges VDD above POR threshold; the chip then enters a
half-booted state that doesn't recover when USB later supplies real
power. PB10/PB11 are clean GPIO breakouts with no onboard pulls (verified
against schematic v1.1).

**TI1S XOR routing.** TIM2 CC1 doesn't have a direct path to PB10 under
any AFIO remap. The fix is `CTLR2.TI1S=1`, which routes TI1 internal =
XOR(CH1_pin, CH2_pin, CH3_pin). With `TIM2_RM=0b10` putting CH3 on
PB10, CH1 on PA0, and CH2 on PA1 — and PA0/PA1 configured as input
pull-down — TI1 collapses to PB10's signal alone. CC1 then captures
the falling edge through the standard IC pipeline. PB11 (CH4 under the
same remap) is unused.

**Atomic 32-bit IC.** Every PB10 falling edge sets TIM2 CC1IF, which
gates two events on the same silicon cycle: (a) TIM2.CCR1 latches the
low half via DMA1_CH5, and (b) TRGO pulses (MMS=COMPARE_PULSE) into
TIM3's TRC, latching TIM3.CCR1 high half via DMA1_CH6. The pair forms a
32-bit tick that the walker reads without any software lift step. A
1-cycle TRC synchronization window per TIM2 wrap can produce an
off-by-+65536 pair; §3.3 documents detection and correction.

**No per-edge ISR.** At 3 Mbaud worst case that's the difference
between a usable instrument and a wedged one. The walker (§3.4)
drains the rings on DMA HT/TC and USART IDLE events, forward-walks
both rings in lockstep, and produces per-byte stamps.

### 3.1 Ring layout

Three host-facing rings produced by the walker plus three internal DMA
staging buffers. The IC pair is split across two DMA channels writing
parallel u16 rings; the walker reads them in lockstep through a single
`falling_at(idx, ceiling)` accessor that combines and wrap-race-corrects.

| Ring             | Width | Producer  | Role                                       |
|------------------|-------|-----------|--------------------------------------------|
| `falling_lo[i]`  | u16   | DMA1_CH5  | internal — TIM2.CCR1, low 16 of IC tick    |
| `falling_hi[i]`  | u16   | DMA1_CH6  | internal — TIM3.CCR1, high 16 of IC tick   |
| `rx_ring[i]`     | u8    | DMA1_CH3  | internal — USART RX bytes, walker-only     |
| `bytes_ring[i]`  | u8    | walker    | byte value at stamp time                   |
| `ts_ring[i]`     | u32   | walker    | byte start tick (`tick32`)                 |
| `flags_ring[i]`  | u8    | walker    | anomaly flags per byte                     |

`bytes_ring`, `ts_ring`, and `flags_ring` are **parallel arrays** indexed
in lockstep. One byte's complete record is `(bytes[i], ts[i], flags[i])`.

`falling_lo` / `falling_hi` are parallel u16 rings, both `FALL_LEN`
entries deep, written by separate DMA channels on the same TIM2 TRGO
pulse. DMA1_CH5 (TIM2.CCR1) services first per same-priority arbitration,
DMA1_CH6 (TIM3.CCR1 via TRC) trails by one DMA-transfer cycle. The
walker tracks **CH6's NDTR** as `falling_total` — when CH6 has
decremented past entry `i`, both halves of slot `i` are written and the
combined 32-bit tick is safe to read. `falling_at(idx, ceiling)` is the
single read accessor: it reads both halves, combines `(hi << 16) | lo`,
and applies the §3.3 wrap-race correction. The host-facing `BICSNAP`
diagnostic uses the same accessor, so it returns sub-µs-accurate ticks
regardless of how long ago each entry was captured — no per-wrap window
limit.

The walker mirrors the byte value from `rx_ring` into `bytes_ring` at
stamp time. `rx_ring` is a circular DMA buffer the walker reads from at
processing time; the host never reads through to it. Mirroring decouples
drain semantics from `rx_ring`'s DMA wrap: a host that drains long after
stamping (or with the ring almost full) still reads the byte value that
the walker actually saw on the wire, not whatever DMA has since written
to that physical slot.

Producer invariant: `byte_head ≤ rx_total`. The walker can't stamp a byte
that hasn't arrived; the host reads up to `byte_head`.

### 3.2 Walker triggers

The walker runs on three event sources. All three share one PFIC
priority so `walk()` runs single-threaded across them — no concurrent
walker hazard.

| ISR vector       | Event                              | Phase tag              |
|------------------|------------------------------------|------------------------|
| `DMA1_CHANNEL6`  | IC ring half-transfer / complete   | `TRACE_PHASE_IC_HT/TC` |
| `DMA1_CHANNEL3`  | RX ring half-transfer / complete   | `TRACE_PHASE_RX_HT/TC` |
| `USART3`         | USART3 line-idle (IDLE flag)       | `TRACE_PHASE_IDLE`     |

CH6 is the IC ring's trailing DMA writer (§3.1); its HT/TC therefore
guarantees both halves of every entry up to the cursor are written. CH5
captures off the same TIM2 TRGO pulse but ~1 DMA-transfer cycle earlier
and is left without IRQ. DMA HT runs at half-ring fill, TC at full-fill
— the walker drains promptly without waiting for a separate cadence.
Time-to-HT scales with edge density at the configured baud.

USART3 IDLE asserts one character-time after the wire goes idle. It
does two things — both signal-only, neither derives a tick from elapsed
idle time:

1. **Drains tail bytes.** Runs `walk()` so the last byte of every
   reply burst gets stamped without waiting for the next DMA HT/TC.
2. **Marks the chain boundary.** Once `walk()` finishes draining and
   `byte_head == rx_total`, the walker flushes any trailing interior
   edges of the last byte (`walked = falling_total`) and drops the PLL
   anchor (`has_anchor = false`). The next byte the walker stamps cold-
   starts on its own IC edge instead of free-running off a stale
   `last_anchor + 10·bit_ticks` prediction that would sit one inter-
   packet quiet window (RDT) before reality.

TIM2 CC1 (TI1S XOR routing per §3) and TIM3 CC1 (TRC) drive their DMA
requests via CCxDE; neither CCxIE is enabled. Capture stays zero-CPU
per edge. No TIM2 update or compare interrupts feed the walker.

### 3.3 Constructing `ceiling`

`ceiling` is the wrap-race detection reference for `falling_at(idx,
ceiling)`. Since the IC pair is captured atomically in hardware (§3),
there's no per-wrap lift step; the only correction needed is the
1-cycle TRC-sync race that can mis-pair a low half from before TIM2
wraps with a high half from after TIM3 increments.

`walk()` constructs `ceiling` by reading `read_tick32()` **after**
refreshing the IC NDTR snapshot:

    falling_total = refresh_falling_total()    // CH6 NDTR → u32 cursor
    rx_total      = refresh_rx_total()
    ceiling       = read_tick32()              // (TIM3.CNT << 16) | TIM2.CNT

The ordering matters. Every IC pair reflected in `falling_total` was
latched by hardware before the subsequent `read_tick32()` call, so
`combined ≤ ceiling` for every valid entry. An entry latched between
the NDTR refresh and the `read_tick32()` call isn't counted in this
walker's `falling_total` and falls to the next trigger.

**Wrap-race correction.** TIM3 phase-leads TIM2 by the AHB write gap at
init (§1). TIM3.CNT therefore increments a few cycles BEFORE each TIM2
wrap. If a capture latches inside that 1-cycle window:

- TIM2.CCR1 = 0xFFFF (still pre-wrap)
- TIM3.CCR1 = NEW_hi (TIM3 just ticked, latched via TRC one sync cycle later)
- combined = (NEW_hi << 16) | 0xFFFF = actual + 65536

Detection is mathematically disjoint from valid entries: bad pairs have
`combined > ceiling` (NEW_hi rolled forward while actual entry's tick is
still in OLD_hi); valid pairs have `combined ≤ ceiling`. Correction:

    if combined != ceiling
       && combined.wrapping_sub(ceiling) <= u32::MAX / 2 {
        combined.wrapping_sub(1 << 16)        // raced — subtract one wrap
    } else {
        combined                               // valid
    }

One arithmetic compare plus one conditional subtract per entry. Zero
false positives (the cyclic distance check is symmetric and the
`!= ceiling` guard excludes the exact-tie case where a legitimate edge
happened to capture at the same tick we read in `read_tick32`).

`read_tick32()` is a single read pair (TIM3.CNT, TIM2.CNT) with a
coherence loop only on the rare wrap-mid-read; cost is well under one
bit-time even at 3 Mbaud. The phase offset between TIM3 (free-running)
and TIM2 doesn't break the read protocol — peripheral reads span ~10
cycles, much wider than the 1-cycle race window, so `lo_2 < lo_1`
triggers retry whenever a wrap could have raced the read.

### 3.4 Predict-and-snap PLL walker

Walker state across calls (entire surface):

    walked       : u32   — count of IC entries consumed, monotonic
    byte_head    : u32   — count of bytes emitted, monotonic
    last_anchor  : u32   — most recently emitted byte's start tick (lifted)
    has_anchor   : bool  — false at boot / post-RESET / post-set_baud

Those four fields plus the `DESYNCED` sticky flag (§3.5) are the
walker's entire mutable state.

#### Reading IC entries

Each entry is read via `falling_at(idx, ceiling)`, which combines the
parallel `falling_lo[idx]` / `falling_hi[idx]` u16 halves into a u32
tick and applies the §3.3 wrap-race correction. No backward-lift step,
no per-wrap window limit — the hardware delivers a 32-bit pair
atomically and `ceiling` is only used to detect the 1-cycle TRC-sync
race. Realistic UART bursts at any supported baud (down to 57.6 kbaud
verified by tool-pirate-tune) work identically.

#### Classification

The algorithm is **predict-and-snap with closest-edge tiebreak**, with
`SNAP_BITS = 1` half-width on the snap window.

For each byte `B` newly present in `rx_ring`:

1. **Cold-start path (`has_anchor == false`).** No prediction available
   yet, so anchor on the next unconsumed IC entry. Yield mid-byte if
   `ceiling < first_edge + 10·bit_ticks` — more interior edges of
   this byte may still be in flight, and anchoring now would risk a
   silent slip if one arrives between iterations. Otherwise
   `chosen_anchor = first_edge`; advance `walked` past it. This path
   runs at every cold-start trip — boot, RESET, set_baud, post-
   `IcOverrun` recovery, **and post-USART-IDLE chain boundary (§3.2)**;
   every subsequent byte inside a contiguous burst goes through the
   steady-state path.

2. **Steady-state path (`has_anchor == true`).** Predict
   `predicted = last_anchor + 10·bit_ticks`. Yield mid-byte if
   `ceiling < predicted + SNAP_BITS·bit_ticks` — a real start edge
   inside the snap window may still be inbound, and stamping a free-run
   miss now would overwrite a recoverable hit at the next trigger.

   a. Skip-walk `walked` past any IC entries before
      `predicted − SNAP_BITS·bit_ticks` (leftover interior edges of
      byte `B−1`, plus any wire glitches that survived the CC filter
      and landed in the inter-byte gap).
   b. Scan `[predicted − SNAP_BITS·bit_ticks, predicted + SNAP_BITS·
      bit_ticks]` for the IC entry closest to `predicted`. Tiebreak
      prefers the **later** entry: real start edges always sit AT or
      AFTER prediction (upstream chip's inter-byte hardware idle shifts
      the start LATER, never earlier), so on a distance tie the later
      candidate is far more likely the real start than the earlier one
      (which would be ringing-tail noise from byte `B−1`).
   c. **Hit**: `chosen_anchor = matched entry`; advance `walked` past it.
   d. **Miss**: `chosen_anchor = predicted` (free-run). Flag
      `COUNT_UNDER`. Do NOT advance `walked` past anything in the scan
      window — those entries stay in the ring for the next byte's
      skip-walk pass.

3. **Emit.** `start_tick = chosen_anchor − cc_filter_delay`; write
   `(B, start_tick, flags)` into the three parallel rings; update
   `last_anchor = chosen_anchor`; advance `byte_head`.

#### Why `SNAP_BITS = 1`

The snap has to absorb whatever shifts the real wire-side start edge
away from `predicted = last_anchor + 10·bit_ticks`. The bit-time unit
normalizes HCLK out: `SNAP_BITS · bit_ticks` is `SNAP_BITS` bit-times
of wall-clock regardless of who's measuring, so 1 here means "1 bit-
time of inter-byte gap variation tolerated." Wider hurts glitch
resistance — a wire glitch surviving the CC filter is more likely to
land closer to `predicted` than the real start edge as the window
grows. Narrower drops bytes whose upstream-chip TX hardware idle
exceeded the window, free-running on prediction and flagging
`COUNT_UNDER` until the chain recovers.

1 is ~50× the observed loopback jitter floor: tool-pirate-tune stage 1
reports max inter-byte deviation `dev ≤ 24 ticks` at the slowest
supported baud (57.6 kbaud, `brr = 2500`) — 0.01 bit-times. At every
faster baud `dev = 0`. Real upstream chips may drive up to ~3 bit-times
of hardware idle between bytes in a chain, so widen to 2 or 3 if mid-
chain bytes go missing on a real target. The loopback path cannot
exercise this directly — both halves of the PB10/PB11 bridge are the
same chip — so widening is a wire-telemetry decision, not a
tool-pirate-tune one.

#### Why predict-and-snap

The prior **lockstep window** walker anchored each byte on its own
first unconsumed IC entry and counted edges in `[anchor, anchor +
10·bit_ticks)`. That window unintentionally absorbed the **next**
byte's start edge whenever the upstream chip's tpb ran ahead of the
pirate's — pushing the next byte's anchor onto whatever non-edge sat
after the absorbed one, then cascading the misalignment forward through
the whole burst. Low-edge bytes (`0x00`, `0xFF`) made this worst: no
intermediate edges separated the byte from its successor, so the count
loop ate the next start bit with nothing to break the cascade.

PLL avoids the cascade two ways:

- Anchor on `last + 10·bit_ticks ± snap`, not on "first unconsumed
  edge." A next-byte start that would have been absorbed by the old
  window now sits one byte-period ahead and is the exact target of the
  next iteration's snap scan. Bounded error, no propagation.
- Closest-to-prediction snap rejects glitches by distance. A glitch in
  the snap window beats the real start only when the glitch sits
  closer to `predicted` than the real start does — and the later-on-tie
  rule covers the equidistant case in favour of the chip-physics-real
  edge.

#### Trade-off

Cold-start re-anchors on the next unconsumed IC entry. A stale glitch
sitting in the IC ring from before the cold-start moment becomes
byte 0's start tick — the same vulnerability the lockstep walker had
at **every** byte. PLL hardens steady-state behaviour without rescuing
cold start. The trip points span boot/RESET/set_baud (the host re-
issues a packet either way), `IcOverrun` recovery (designed-impossible,
host hard-fails), and post-USART-IDLE chain boundaries (every
request/reply gap). The last one is the common-case trip: a glitch
landing in the inter-burst quiet window between echo and reply would
mis-anchor reply byte 0. In practice the CC filter eats sub-bit-time
glitches at every baud (§4) and inter-burst quiet windows on a healthy
wire are clean — the IDLE-driven flush also drops trailing interior
edges of the last byte, so cold-start scans only edges that landed
after the wire actually quiesced.

In steady state, a glitch surviving the CC filter and landing in a
byte's snap window pulls the anchor onto itself only when it sits
strictly closer to `predicted` than the real start does. The real
start edge sits at `predicted + chip_idle_gap` where `chip_idle_gap ∈
[0, ~3·bit_ticks]`; a glitch closer than that has to land within
`[predicted, predicted + chip_idle_gap]` — a narrow band relative to
the full snap window. On a clean wire the CC filter eats glitches
before they reach this stage.

Mitigations explicitly rejected (per design constraints):

- IDLE-driven flush of stale edges before cold-start — violates the
  "IDLE doesn't touch walker state" contract (IDLE is signal-only;
  walker state never derives from elapsed idle time).
- Best-effort invented stamps on free-run miss — PLL's
  `chosen_anchor = predicted` IS the invented stamp, but it's bounded
  to `±snap` of the real value and flagged `COUNT_UNDER` so the host
  can recognize it as estimated rather than measured.

**Surfacing the anomaly via `COUNT_UNDER` is the contract; silent
recovery is not.**

### 3.5 Desync contract

Two conditions trip a single sticky `DESYNCED` flag — one
designed-impossible internal failure plus one host-paced bench-script
bug. Once tripped, the walker stops emitting and the host protocol
surface fails loud; recovery is the explicit `RESET` command (or `BAUD`,
which implicitly resets).

| Cause            | Check                                              | Class                              |
|------------------|----------------------------------------------------|------------------------------------|
| `ic_overrun`     | `falling_total − walked > FALL_LEN`                | Designed-impossible — IC ring (256 entries) fills slower than the walker drains it under the event-driven HT/TC contract (§3.2). |
| `stamp_overflow` | At emit, `byte_head − byte_tail ≥ STAMP_LEN`       | Host-paced — bench script didn't drain via BBATCH; ring depth is 1024 (~10 ms at 1 Mbaud). |

When either trips:

    DESYNC_CAUSE.compare_exchange(0, cause, …)   // first trip wins
    DESYNCED.store(true, Release)
    // walker stops emitting; rx_ring + falling_lo/hi keep filling but
    // are forfeit — next RESET drops them.

Subsequent host commands return `ERR desync <cause>` — except `STATUS`
(which reports the cause), `RESET` (which clears it), and `BAUD` (which
implicitly resets via `reset_walker`). Recovery is one of those three
commands, not a power cycle.

`ic_overrun` is designed-impossible: if it ever trips, that's a hard
bug worth investigating — a clean RESET clears the symptom but doesn't
fix the underlying timing budget violation. `stamp_overflow` is the
expected "your test script needs to drain more often" signal; it's a
contract surface, not a bug indicator.

The walker hot path may be placed in SRAM via `qingke-rt`'s
`#[interrupt(highcode)]` attribute to shave flash-wait-state cycles and
keep ISR latency well under the IC ring's HT-to-fill window.

## 4. Validation contract

### Timing slack

The PLL snap absorbs whatever shifts the real wire-side start edge
away from `predicted = last_anchor + 10·bit_ticks`. Known contributors:
pirate-vs-upstream clock drift (sub-bit_ticks for crystal-vs-HSI
pairs), DMA arbiter latency (sub-bit_ticks), and upstream-chip
inter-byte hardware idle inside a chain (up to ~3 bit_ticks for typical
half-duplex protocols).

`SNAP_BITS = 1` sized against tool-pirate-tune loopback measurements:
`dev ≤ 24 ticks` at 57.6 kbaud (`brr = 2500` → 0.01 bit-times), and
`dev = 0` at every faster baud. 1 bit-time is ~50× the observed
jitter floor under loopback. Real-target chains may expose the
upstream-idle contributor — widen toward 2 or 3 if the host sees
mid-chain `COUNT_UNDER` flags on chained replies; the snap width is
the knob that hides this unknown from the host.

### Anomaly flags

Each byte carries one `u8` of flags:

| Bit | Name              | Meaning                                                 |
|-----|-------------------|---------------------------------------------------------|
| 0   | `COUNT_UNDER`     | snap miss — `tick` is the predicted free-run value      |
| 1-7 | reserved          |                                                         |

`flags == 0` means the walker found a real IC edge inside the snap
window and the emitted `tick` is hardware-precise (sub-tick of the
wire edge after subtracting `cc_filter_delay`). `COUNT_UNDER` means no
edge landed in `[predicted ± SNAP_BITS·bit_ticks]` — the emitted
`tick = predicted − cc_filter_delay` is the walker's best estimate,
accurate to `± SNAP_BITS·bit_ticks`. Host code that needs sub-tick
precision should filter on `COUNT_UNDER` and treat those bytes as
ranged, not point, measurements.

The PLL has no separate recovery state to engage (every byte is either
hit or free-run-miss), and `COUNT_UNDER` already names the best-effort
case. If the walker ever encounters a designed-impossible condition
that could leave the rings desynced (IC ring overrun) it does not emit
a flag — it flips `DESYNCED` and the host gets an explicit error on
the next command (§3.5).

### CC filter delay

TIM2 CC1 IC's digital filter (CCMR1 IC1F + CKD) suppresses glitches
narrower than `N·t_sample`. It adds a **systematic per-edge delay** of
`N·t_sample`, identical across all edges — calibratable shift, not
jitter widening.

The filter is **retuned per baud** on every `set_baud`. CKD is pinned at
`DIV_1` (CKD=00, fDTS = HCLK = 144 MHz) so the whole ICxF table is
reachable without bouncing CKD; only the IC1F nibble in CCMR1 moves.

The picker rule is **largest filter delay ≤ brr/3 (≈ 0.333·bit time)**.
The bench loopback wiring has no transceiver and only PB11's ~30 kΩ
internal pull-up. RC rise time τ ≈ 500–900 ns puts the Schmitt-
threshold crossing ~400 ns after OD release (PB10 is push-pull on this
bench but the rule was sized under OD as the worst case). Inside a
back-to-back 1-bit-time inter-byte HIGH gap (every byte whose
predecessor has `b7=0`), the line only stays above threshold for
~400 ns before the next fall — so a filter that needs > 400 ns of
stable HIGH to confirm state will silently drop the next byte's anchor.

A transceiver-driven target (τ ≈ 50 ns) would tolerate a looser
`2·brr/3`-equivalent pick. The pirate can't assume a transceiver, so
its rule is stricter. Looser rules wire-empirically lose anchors at
1 Mbaud: `< brr` ate ~60%; `≤ 2·brr/3` was marginal — stochastically
clean trials interleaved with ~50% anchor-loss trials depending on
instantaneous VDD/Schmitt noise.

Per-baud picks (HCLK = 144 MHz, fDTS = 144 MHz):

| Baud | bit (ns) | IC1F | fSAMPLING | N | delay (ns) | delay (ticks) |
|------|----------|------|-----------|---|------------|---------------|
| 3 M    | 333    | 0101 | fDTS/2 = 72 MHz  | 8 | 111    | 16   |
| 2 M    | 500    | 0110 | fDTS/4 = 36 MHz  | 6 | 167    | 24   |
| 1 M    | 1000   | 1000 | fDTS/8 = 18 MHz  | 6 | 333    | 48   |
| 500 k  | 2000   | 1011 | fDTS/16 = 9 MHz  | 6 | 667    | 96   |
| 250 k  | 4000   | 1110 | fDTS/32 = 4.5 MHz| 6 | 1333   | 192  |
| ≤ 115k | ≥ 8680 | 1111 | fDTS/32 = 4.5 MHz| 8 | 1778   | 256  |

The exhaustive ICxF table (15 entries; one per legal `ICxF[3:0]` value
per RM `CH32FV2x_V3xRM` §14.4.7) lives as a const LUT in `capture.rs`;
`filter_for_brr(brr)` walks it at compile time. `CC_FILTER_DELAY_TICKS`
is an atomic refreshed from the LUT entry each `set_baud`; the walker
subtracts it from every IC stamp before storing into `ts_ring` so
external time comparisons (e.g. against TIM4 OPM send deadlines) compose
cleanly.

A `set_baud` while bytes are in flight produces a brief window where
old-filter and new-filter IC entries could coexist in the IC ring;
the matching `reset_walker()` call drops them to `falling_total` so the
next byte anchors on a fresh-filter edge.

## 5. Wire-side drive (TX)

    host: SEND bytes=…  at=<tick32>
      └─► TIM4 OPM, ARR = (at − now_tick32) low 16
            └─► CC2 match → DMA1_CH4 stamps EN=1 over DMA1_CH2.CR
                  └─► DMA1_CH2 streams payload into USART3 DR
                        └─► AF PP drives PB10

TIM4 in one-pulse mode holds the send window; its CC2 match enables
USART3's TX DMA, which streams the prepared payload into the data
register. Hardware-coupled — no CPU between deadline and first wire
bit, no ISR-latency floor on the send side.

For `at − now_tick32 > 0xFFFF` (≈ 455 µs) the schedule falls through to
a software-immediate path. Sub-455 µs is the typical wire-timing regime
we measure, so the timer-driven path covers the common case.

## 6. Peripheral map

| Peripheral | Role                                                                |
| ---------- | ------------------------------------------------------------------- |
| TIM1       | unused (reserved for future debug instrumentation)                  |
| TIM2       | low 16 of `tick32`; PSC=0, ARR=0xFFFF, CKD=`DIV_1`; CTLR2.TI1S=1    |
|            | + CCMR1.CC1S=TI4 + IC1F per-baud + CCER.CC1P=1 → CC1 IC on PB10    |
|            | via XOR routing; CTLR2.MMS=COMPARE_PULSE → TRGO pulses TIM3 TRC;    |
|            | DMA1_CH5 (DMAINTENR.CC1DE); TIM2_RM=0b10 partial remap; no PFIC IRQ |
| TIM3       | high 16 of `tick32`; PSC=0xFFFF free-running (HCLK/65536 = TIM2     |
|            | wrap rate), phase-locked at init; SMCFGR.SMS=0 (slave disabled),    |
|            | SMCFGR.TS=1 routes TIM2 TRGO → TRC; CCMR1.CC1S=TRC + CCER.CC1P=0    |
|            | → TIM3.CCR1 latches on every TRGO pulse; DMA1_CH6 (DMAINTENR.CC1DE) |
| TIM4       | TX send OPM (CC2 → DMA1_CH4 stamps DMA1_CH2.CR)                     |
| SysTick    | embassy time-driver only (qingke V4 64-bit hw counter)              |
| USART3     | full-duplex: TX=PB10 AF PP → DMA1_CH2; RX=PB11 input-pullup →       |
|            | DMA1_CH3; PB10/PB11 bridged externally to one wire;                 |
|            | IDLE → signal-only                                                  |
| DMA1_CH2   | USART3 TX (per-send)                                                |
| DMA1_CH3   | USART3 RX → `rx_ring`; HT/TC → walker (PRIO_WALKER)                 |
| DMA1_CH4   | TIM4_CC2-triggered stamp of EN=1 over DMA1_CH2.CR                   |
| DMA1_CH5   | TIM2_CC1 capture → `falling_lo` (low 16 of IC pair); no IRQ         |
| DMA1_CH6   | TIM3_CC1 capture → `falling_hi` (high 16 of IC pair);               |
|            | HT/TC → walker (PRIO_WALKER); CH6 is the trailing writer            |

## 7. Host protocol surface

Single-pattern protocol: every host command gets exactly one reply
(ASCII line or binary frame), then the channel goes quiet until the next
command. No push streaming, no mode switch — the deep stamp ring + host
polling via `BBATCH` cover every workflow.

### Drain — primary path is `BBATCH`

    BBATCH <count>:u16  → 0xA5 0x5A <n>:u16 LE [<tick>:u32 LE <byte>:u8 <flags>:u8] × n

Binary frame: sync header `0xA5 0x5A`, record count `n:u16 LE`
(≤ `count`, ≤ `min(stamps_available, MAX_RECORDS_PER_BATCH)`), then `n`
× 6-byte records. No newline terminator; framing is length-prefixed. The
sync header lets a host that lost framing scan-and-relock on the next
call.

`MAX_RECORDS_PER_BATCH = 64` caps each frame at ~388 B (≤ 6 FS bulk
packets, ≤ 1 ms wire time). Throughput matters at 3 Mbaud's ~300 kB/s
byte rate — per-line ASCII framing won't keep up.

    BDRAIN              → BSTAMP <tick>:u32 <byte>:u8 <flags>:u8 | EMPTY

ASCII single-record drain for human debug only.

### State + recovery

    STATUS              → STATUS <baud>:u32 <avail>:u32 <desynced>:0|1
                            <cause>:none|ic_overrun|stamp_overflow
                            <last_tick>:u32

Always responds, even mid-desync — the host's one-shot health probe.
`desynced` is `1` iff `cause ≠ none`; both are reported for parse
simplicity.

    RESET               → OK

Clears `DESYNCED` + cause, drains stamp/IC rings, restarts walker.
Keeps baud. The routine recovery for any desync cause (§3.5).

    BAUD <bps>          → OK | ERR baud

Changes baud; implicit `RESET` inside. Caller must quiesce the wire
first.

### Diagnostics

    BTRACE              → BTRACE <phase> <tim2_cnt_entry> <tim2_cnt_exit>
                            <falling_pending_entry> <edges_consumed>
                            <bytes_emitted> <falling_total> <rx_total>
                          | EMPTY
    BTRACECLEAR         → OK

Each `BTRACE` record is one walker invocation tagged by trigger source
(phase ∈ {IDLE=0, RX_HT=1, RX_TC=2, IC_HT=3, IC_TC=4}). `tim2_cnt_*`
brackets the walker's ISR-to-exit duration in tick32 ticks;
`falling_pending_entry` is the IC backlog at ISR entry; `edges_consumed`
and `bytes_emitted` are this invocation's workload; `falling_total` and
`rx_total` are cumulative counters at entry. The trace ring is 64
entries deep; when the host falls behind, the oldest records get
overwritten and `BTRACE` snaps the tail forward.

## 8. Design principles

- **Hardware-coherent, not software-correlated.** The combined `tick32` is
  locked at the silicon level — no embassy thread, ISR, or critical-section
  can perturb the high/low relationship. Production firmware on the V006
  uses 16-bit TIM2 + software wrap tracking; that's acceptable for a DUT
  but disqualifying for a ruler.
- **One time domain on the wire.** Every stamp the pirate reports is a
  `tick32`. No SysTick ticks, no µs conversions, no mixed units. The host
  side does its own scaling using the runtime `HZ?` query.
- **DMA-only on the hot path.** A per-edge ISR at 3 Mbaud costs more than
  the measurement it would produce. RX edges land in a ring without CPU;
  only walker invocations spend cycles.
- **Event-driven walker, hardware-atomic IC pair.** Each PB10 falling
  edge captures a full 32-bit tick in hardware via TIM2.CCR1 +
  TIM3.CCR1 (TRC), eliminating any per-wrap software lift. The walker
  runs on DMA HT/TC and USART IDLE only — no cadence, no per-edge
  ISR. Works identically at 57.6 kbaud and 3 Mbaud.
- **Predict-and-snap PLL classification.** Each byte's stamp anchors
  on the IC edge closest to `predicted = last_anchor + 10·bit_ticks`
  within `±SNAP_BITS·bit_ticks`; on miss the walker free-runs on the
  prediction and flags `COUNT_UNDER`. The PLL bounds across-byte drift
  to one byte-period regardless of which edge inside a byte gets
  picked. Cold-start path (post-RESET, post-IDLE chain boundary)
  anchors on the next unconsumed IC entry — same code path as bootstrap.
- **Fail loud, never silently recover.** Two conditions — one
  designed-impossible (`ic_overrun`) and one host-paced bench-script
  bug (`stamp_overflow`) — flip a single sticky `DESYNCED` flag. Every
  host command then errors out until `RESET` (or `BAUD`, which
  implicitly resets). A walker that silently re-acquires would mask
  whatever bug let a designed-impossible failure trip; silently
  dropping stamps on overflow would lie about wire state.
- **Embassy stays off the wire.** SysTick hosts embassy's time queue for
  USB and utility timers. It does not appear in any wire-timing path, so
  embassy's choice of priority, jitter, and tick rate is orthogonal to
  bench accuracy.
- **TIM1 left free.** No advanced-timer feature is needed today. Keeping
  TIM1 unused leaves room for a future debug instrument (external trigger,
  encoder counter, second IC bank for rising-edge diagnostics) without
  rewiring.
- **The ruler exposes its own uncertainty.** Every byte stamp carries a
  flags byte; the host can distinguish hardware-precise from best-effort
  stamps, and wire-anomalous from clean conditions. A ruler that silently
  produces wrong measurements under noise is worse than one that flags its
  own limits.

## 9. ISR + DMA priority discipline

Two priority axes — PFIC (CPU interrupt preemption) and DMA arbiter (per-
channel CHCFG.PL) — both matter for keeping the measurement clock honest
under burst load. The doc previously glossed over them; here's the contract
the firmware enforces.

### 9.1 PFIC interrupt priorities

| Class         | IPRIOR | IRQs                                                  | Why                                  |
|---------------|--------|-------------------------------------------------------|--------------------------------------|
| `PRIO_WALKER` | `0x00` | `USART3`, `DMA1_CHANNEL6`, `DMA1_CHANNEL3`            | Single walker class; ISRs share it so `walk()` is single-threaded by construction. |
| `PRIO_USB`    | `0x80` | `USB_LP_CAN1_RX0`                                     | Lower preempt class — USB stack delays cannot delay a wire-side stamp. |

`USART3` carries only IDLE (signal-only per §3.2). `DMA1_CHANNEL6` is
the IC ring's trailing-writer HT/TC. `DMA1_CHANNEL3` is RX HT/TC. TIM2,
TIM3, DMA1_CH1/CH2/CH4/CH5 have **no IRQ enabled** — their PFIC slots
are intentionally unconfigured.

### 9.2 DMA arbiter priorities (CHCFG.PL)

The DMA arbiter serializes simultaneous transfers; without explicit PL the
default is LOW for every channel. At 3 Mbaud burst the IC ring fills at
~167 ns per edge, so an arbiter delay caused by a coincident USB DMA can
overrun the ring before the walker drains it.

| Channel              | PL          | Why                                                          |
|----------------------|-------------|--------------------------------------------------------------|
| CH5 (IC low half)    | `VERYHIGH`  | Measurement clock; arbiter delay → ring overrun → lost edges |
| CH6 (IC high half)   | `VERYHIGH`  | Paired with CH5; equal priority → numeric arbitration puts CH5 first, CH6 second — the source of CH6's trailing-writer property (§3.1) |
| CH3 (USART3_RX)      | `HIGH`      | Must not drop bytes at 3 Mbaud; one tier below the IC pair   |
| CH2 (USART3_TX)      | `VERYHIGH`  | Defensive; paced by USART (FIFO absorbs arbiter delay) but pinned to top tier against future inter-channel contention |
| CH4 (CC2 stamp)      | `VERYHIGH`  | Mirrors CH2 above; one-shot per send                         |
| (USB)                | `LOW` (default) | USB is throughput-bursty but tolerates µs-scale arbiter latency |

CH5/CH6 outrank CH3 so a back-to-back IC + RX simultaneous request
(typical at every byte's start bit) resolves to "stamp the edge before
the byte value lands" — i.e. the IC pair always leads `rx_ring`, never
lags. That matches the walker's invariant (§3.1: `byte_head ≤ rx_total`,
sourced from IC entries).

# Pirate Hardware Timing

The dxl-pirate's job is to be a more accurate, more precise, and more reliable
ruler than anything it measures. This doc is the architecture contract for how
that's wired — peripheral assignments, the time domain, the per-byte stamp
pipeline, and the validation principles that keep measurements
hardware-grounded.

The chip is the MuseLab nanoCH32V203 (V203C8T6, QingKe V4B @ HCLK = 144 MHz).
Bus is single-pair half-duplex DXL 2.0 up to 3 Mbaud.

## 1. Time domain

One clock, one base, no software wrap counter on the hot path.

- **TIM2** — PSC=0, ARR=0xFFFF, free-running at HCLK. Low 16 bits.
  MMS=update, so each wrap fires TRGO.
- **TIM3** — slave mode, TS=ITR1 (= TIM2 TRGO), SMS=ext-clock-mode-1.
  Increments exactly once per TIM2 wrap. High 16 bits.

Combined stamp:

    tick32 = (TIM3.CNT << 16) | TIM2.CNT

Hardware-locked: TIM3 only advances on the same silicon edge that wraps
TIM2.CNT to zero. No race in the silicon between high and low halves, no
software wrap bookkeeping. The read protocol that recovers `tick32` from
the two registers atomically lives in §3.3.

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

Two GPIO pins bridged externally to one DXL line, three peripheral block
taps, zero CPU per edge.

    PB10 ──┬── USART3_TX (AF OD) ◄─ DMA1_CH2 ◄─ TX burst (per-fire, §5)
           │
           └── TIM2_CH3 IC      ─► DMA1_CH1 ─► falling_ring (internal)
               (falling, CC-filtered; PB10 via TIM2_RM=0b10 partial remap)
           │
           │   external jumper:  PB10 ───┬──── DXL bus line
           │                             │
           │                             └──── PB11
           │
    PB11 ──── USART3_RX (input pullup) ─► DMA1_CH3 ─► rx_ring (DMA-populated)

USART3 runs full-duplex: PB10 drives the bus via AF open-drain TX, PB11
listens via input-with-pullup RX, and the bench jumper ties both pins to
the same DXL line so on-chip RX sees both our own TX echo and remote
servo replies. HDSEL would internally tri-state RX during TX and is not
used here.

PA2/PA3 were the original choice (USART2 default mapping) but boot-wedge
on the MuseLab nanoCH32V203: PA3 has R10 = 10 kΩ to 3V3 on the board
(SD-card CS pull-up). With the bus connected idle-high before USB plugs
in, current trickles via bus → series-R → PA3 → R10 → chip 3V3 rail and
parasitically charges VDD above POR threshold; the chip then enters a
half-booted state that doesn't recover when USB later supplies real
power. PB10/PB11 are clean GPIO breakouts with no onboard pulls (verified
against schematic v1.1).

TIM2_CH3 input capture taps PB10 via GPIO_INDR; no AFIO conflict because
IC reads the input register regardless of the USART block's output state.
TIM2_CH4 (PB11 by the same remap) is unused — PB10 and PB11 see
electrically identical edges (same wire node), so the IC stays on CH3 for
mechanical compatibility with the existing DMA channel and CC-filter
calibration.

DMA1_CH1 streams 16-bit IC captures into `falling_ring`. **No per-edge ISR**
— at 3 Mbaud worst case that's the difference between a usable instrument
and a wedged one. The walker (§3.4) drains the ring on periodic and
event-driven triggers, forward-walks both rings in lockstep, and produces
per-byte stamps.

### 3.1 Ring layout

Three host-facing rings produced by the walker plus two internal DMA
staging buffers:

| Ring           | Width | Producer  | Role                                  |
|----------------|-------|-----------|---------------------------------------|
| `falling_ring` | u16   | DMA1_CH1  | internal — raw IC stamps              |
| `rx_ring[i]`   | u8    | DMA1_CH3  | internal — USART RX bytes, walker-only|
| `bytes_ring[i]`| u8    | walker    | byte value at stamp time              |
| `ts_ring[i]`   | u32   | walker    | byte start tick (lifted to `tick32`)  |
| `flags_ring[i]`| u8    | walker    | anomaly flags per byte                |

`bytes_ring`, `ts_ring`, and `flags_ring` are **parallel arrays** indexed
in lockstep. One byte's complete record is `(bytes[i], ts[i], flags[i])`.

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

The walker fires on three event sources. All three share one PFIC
priority so `walk()` runs single-threaded across them — no concurrent
walker hazard.

| ISR vector       | Trigger                            | Phase tag              |
|------------------|------------------------------------|------------------------|
| `DMA1_CHANNEL1`  | IC ring half-transfer / complete   | `TRACE_PHASE_IC_HT/TC` |
| `DMA1_CHANNEL3`  | RX ring half-transfer / complete   | `TRACE_PHASE_RX_HT/TC` |
| `USART3`         | USART3 line-idle (IDLE flag)       | `TRACE_PHASE_IDLE`     |

DMA HT fires at half-ring fill, TC at full-fill — the walker drains
promptly without waiting for a separate cadence. Time-to-HT scales with
edge density at the configured baud and stays well under one u16 wrap of
TIM2 (≈ 455 µs at 144 MHz) at every baud, so §3.4's lift never crosses
a wrap boundary mid-walk.

USART3 IDLE fires one character-time after the wire goes idle — catches
the tail bytes of every reply burst. Per `[[no_idle_timing]]` it's
**signal-only**: triggers `walk()` to drain stamps that have already
arrived, never back-dates a tick from idle elapsed time.

TIM2 CC3 stays in input-capture mode driving DMA1_CH1; its CCxDE bit is
set, its CCxIE is not — capture stays zero-CPU per edge. No TIM2 update
or compare interrupts feed the walker.

### 3.3 Constructing `lift_ceiling`

`lift_ceiling` is the §3.4 lift formula's reference point — a **ceiling
on raw IC entry CNTs visible at this walker entry**. Each entry lifts
backward against it into the correct TIM3 wrap.

`walk()` constructs it by reading `read_tick32()` **after** refreshing
the IC NDTR snapshot:

    falling_total = refresh_falling_total()    // pulls NDTR, advances u32 cursor
    rx_total      = refresh_rx_total()
    lift_ceiling  = read_tick32()              // (TIM3.CNT << 16) | TIM2.CNT

The ordering matters. Every IC entry reflected in `falling_total` has
already been latched by hardware at `refresh_falling_total()` time, so
its raw u16 is ≤ `lift_ceiling as u16` after the subsequent
`read_tick32()`. An entry latched between the NDTR refresh and the
`read_tick32()` call isn't counted in this walker's `falling_total` and
won't be processed until the next trigger.

`lift_ceiling.lo`'s specific value within the wrap doesn't change the
lifted u32 — the formula returns the same value for any ceiling whose
hi half matches the entry's wrap, as long as `ceiling.lo ≥ entry.lo`.
The live-read formula therefore costs no precision over the previous
design's phase-derived constant; it just trades one MMIO read for
robustness against trigger-source phase variation.

`read_tick32()` is a single read pair (TIM3.CNT, TIM2.CNT) with a
coherence loop only on the rare wrap-mid-read; cost is well under one
bit-time even at 3 Mbaud.

### 3.4 Predict-and-snap PLL walker

Walker state across calls (entire surface):

    walked       : u32   — count of IC entries consumed, monotonic
    byte_head    : u32   — count of bytes emitted, monotonic
    last_anchor  : u32   — most recently emitted byte's start tick (lifted)
    has_anchor   : bool  — false at boot / post-RESET / post-set_baud

Those four fields plus the `DESYNCED` sticky flag (§3.5) are the
walker's entire mutable state.

#### Lift

For each IC entry to be classified, lift to u32 by walking **backward**
from `lift_ceiling`, which `walk()` constructs by reading `tick32`
immediately after refreshing the IC NDTR snapshot. The ordering
guarantees `lift_ceiling.lo ≥ entry.lo` for every entry in
`[walked, falling_total)` — entries deposited between the NDTR refresh
and the `tick32` read aren't in this walker's snapshot and fall to the
next trigger.

    delta  = (lift_ceiling as u16).wrapping_sub(entry)   // u16 modular
    lifted = lift_ceiling.wrapping_sub(delta as u32)

`lifted` is the most recent u32 whose low 16 bits equal `entry`, always
≤ `lift_ceiling`. Valid as long as actual age of the entry is < one u16
wrap of TIM2 (≈ 455 µs at 144 MHz). Realistic DXL bursts at every
supported baud keep this true: high-edge-density traffic fires DMA1_CH1
HT well inside one wrap, and sparse traffic ends in USART3 IDLE before
the IC ring half-fills. Sustained low-edge-density bursts at slow bauds
(e.g. all-`0xFF` at 9600) could in principle let an entry age past one
wrap; the §3.5 `ic_overrun` backstop catches the upper bound.
Mirrors `firmware/lib/drivers/.../edge_parser::lift` in production
firmware.

#### Classification

The algorithm is **predict-and-snap with closest-edge tiebreak**, with
`SNAP_BITS = 3` half-width on the snap window.

For each byte `B` newly present in `rx_ring`:

1. **Cold-start path (`has_anchor == false`).** No prediction available
   yet, so anchor on the next unconsumed IC entry. Yield mid-byte if
   `lift_ceiling < first_edge + 10·bit_ticks` — more interior edges of
   this byte may still be in flight, and anchoring now would risk a
   silent slip if one arrives between iterations. Otherwise
   `chosen_anchor = first_edge`; advance `walked` past it. This path
   runs once per cold start (boot, RESET, set_baud, post-`IcOverrun`
   recovery); every subsequent byte goes through the steady-state path.

2. **Steady-state path (`has_anchor == true`).** Predict
   `predicted = last_anchor + 10·bit_ticks`. Yield mid-byte if
   `lift_ceiling < predicted + SNAP_BITS·bit_ticks` — a real start edge
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

#### Why `SNAP_BITS = 3`

The snap has to absorb the worst-case inter-byte hardware idle that
upstream chips drive between bytes inside a Status burst. The bit-time
unit normalizes HCLK out: `SNAP_BITS · bit_ticks` is `SNAP_BITS` bit-
times of wall-clock regardless of who's measuring, so 3 here means
"3 bit-times of inter-byte gap variation tolerated." Wider hurts
glitch resistance — a wire glitch surviving the CC filter is more
likely to land closer to `predicted` than the real start edge as the
window grows. Narrower drops bytes whose upstream-chip TX hardware
idle exceeded the window, free-running on prediction and flagging
`COUNT_UNDER` until the chain recovers.

3 was the value that bottomed out the off-line classifier comparison
against the captured corpus. Tighten only after wire-side telemetry
shows inter-byte gaps are systematically smaller in real operation.

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
sitting in `falling_ring` from before the cold-start moment becomes
byte 0's start tick — the same vulnerability the lockstep walker had
at **every** byte. PLL hardens steady-state behaviour without rescuing
cold start; the trip points are bounded (boot, RESET, set_baud,
post-`IcOverrun` recovery) and the host re-issues a packet either way.

In steady state, a glitch surviving the CC filter and landing in a
byte's snap window pulls the anchor onto itself only when it sits
strictly closer to `predicted` than the real start does. The real
start edge sits at `predicted + chip_idle_gap` where `chip_idle_gap ∈
[0, ~3·bit_ticks]`; a glitch closer than that has to land within
`[predicted, predicted + chip_idle_gap]` — a narrow band relative to
the full snap window. On a clean wire the CC filter eats glitches
before they reach this stage.

Mitigations explicitly rejected (per design constraints):

- IDLE-driven flush of stale edges before cold-start — violates
  `[[no_idle_timing]]` and the "IDLE doesn't touch walker state"
  contract.
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
    // walker stops emitting; rx_ring + falling_ring keep filling but
    // are forfeit — next RESET drops them.

Subsequent host commands return `ERR desync <cause>` — except `STATUS`
(which reports the cause), `RESET` (which clears it), and `BAUD` (which
implicitly resets via `reset_walker`). Recovery is one of those three
commands, not a power cycle.

`ic_overrun` is designed-impossible: if it ever fires, that's a hard
bug worth investigating — a clean RESET clears the symptom but doesn't
fix the underlying timing budget violation. `stamp_overflow` is the
expected "your test script needs to drain more often" signal; it's a
contract surface, not a bug indicator.

The `DesyncCause::WalkerLate` enum variant is reserved (set in earlier
cadence-based revisions when more than one cadence flag was pending at
entry). The event-driven trigger model has no analogous condition, so
the cause is never raised — folded into `ic_overrun` if ISR latency
ever stretches that far. Variant retained in the wire-protocol enum to
keep the cause-code numbering stable; eligible for removal once the
host protocol can tolerate the renumbering.

The walker hot path may be placed in SRAM via `qingke-rt`'s
`#[interrupt(highcode)]` attribute to shave flash-wait-state cycles and
keep ISR latency well under the IC ring's HT-to-fill window.

## 4. Validation contract

### Timing slack

The PLL snap absorbs whatever shifts the real wire-side start edge
away from `predicted = last_anchor + 10·bit_ticks`. The known
contributors are pirate-vs-upstream clock drift (sub-bit_ticks for
crystal-vs-HSI pairs) and transceiver / DMA arbiter latency
(sub-bit_ticks), but the dominant contributor in the captured corpus
was the upstream chip's inter-byte spacing on TX, which empirically
reached the `±2..3·bit_ticks` range on slow-baud Status replies.

`SNAP_BITS = 3` sized empirically: `2` showed a ~14% per-byte miss
rate against the corpus, `3` showed ~1.2%, `4`+ wasn't probed but
trades glitch resistance for headroom we didn't need. The root cause
of the slow-baud inter-byte spread isn't pinned down in this
codebase — telemetry from wire-side captures across more chip families
would say whether it's PFIC + DMA latency, software inter-byte pacing,
ringing on the bench wiring, or some mix. The snap width is the knob
that hides this unknown from the host.

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

The prior `COUNT_OVER` flag is gone: the predict-and-snap walker
ignores edges outside the snap window, so an "extra edge in the byte's
wire window" no longer has a contract semantic. Glitches surviving the
CC filter only affect the byte if they sit closer to `predicted` than
the real start, in which case the misanchor surfaces in the *next*
byte's snap as either a hit at the corrected `predicted = misanchor +
10·bit_ticks` (PLL self-corrects within one byte) or a `COUNT_UNDER`
miss.

`TIMING_DRIFT`, `TIMING_RECOVER`, and `STAMP_BESTEFFORT` are absent by
design: the PLL has no separate recovery state to engage (every byte is
either hit or free-run-miss), and `COUNT_UNDER` already names the
best-effort case. If the walker ever encounters a designed-impossible
condition that could leave the rings desynced (multi-flag walker entry,
IC ring overrun), it does not emit a flag — it flips `DESYNCED` and
the host gets an explicit error on the next command (§3.5).

### CC filter delay

TIM2 CC3 IC's digital filter (CCMR2 IC3F + CKD) suppresses glitches narrower
than `N·t_sample`. It adds a **systematic per-edge delay** of `N·t_sample`,
identical across all edges — calibratable shift, not jitter widening.

The filter is **retuned per baud** on every `set_baud`. CKD is pinned at
`DIV_1` (CKD=00, fDTS = HCLK = 144 MHz) so the whole ICxF table is
reachable without bouncing CKD; only the IC3F nibble in CCMR2 moves.

The picker rule is **largest filter delay ≤ brr/3 (≈ 0.333·bit time)** —
*tighter* than production's natural pick at 48 MHz HCLK. The bench wiring
deviates from production's electrical model in a load-bearing way: PB10
is AF open-drain and the only pull-up on the wire (no servo, no external
resistor) is PB11's ~30 kΩ internal pull-up. RC rise time τ ≈ 500–900 ns
puts the Schmitt-threshold crossing ~400 ns after OD release. Inside a
back-to-back 1-bit-time inter-byte HIGH gap (every byte whose
predecessor has `b7=0`), the line only stays above threshold for ~400 ns
before the next fall — so a filter that needs > 400 ns of stable HIGH to
confirm state will silently drop the next byte's anchor.

Production runs through a DXL transceiver (τ ≈ 50 ns), so its
`2·brr/3`-equivalent pick has plenty of margin. The pirate cannot
assume a transceiver, so its rule is stricter. Looser rules
wire-empirically lose anchors at 1 Mbaud: `< brr` ate ~60%; `≤ 2·brr/3`
was marginal — stochastically clean trials interleaved with ~50%
anchor-loss trials depending on instantaneous VDD/Schmitt noise.

Per-baud picks (HCLK = 144 MHz, fDTS = 144 MHz):

| Baud | bit (ns) | IC3F | fSAMPLING | N | delay (ns) | delay (ticks) |
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
external time comparisons (e.g. against TIM4 OPM fire deadlines) compose
cleanly.

A `set_baud` while bytes are in flight produces a brief window where
old-filter and new-filter IC entries could coexist in `falling_ring`;
the matching `reset_walker()` call drops them to `falling_total` so the
next byte anchors on a fresh-filter edge.

## 5. Wire-side drive (TX)

    host: FIRE bytes=…  at=<tick32>
      └─► TIM4 OPM, ARR = (at − now_tick32) low 16
            └─► CC2 match → DMA1_CH4 stamps EN=1 over DMA1_CH2.CR
                  └─► DMA1_CH2 streams payload into USART3 DR
                        └─► AF OD drives PB10

TIM4 in one-pulse mode holds the inject window; its CC2 match enables
USART3's TX DMA, which streams the prepared payload into the data register.
Hardware-coupled — no CPU between deadline and first wire bit, no
ISR-latency floor on the inject side.

For `at − now_tick32 > 0xFFFF` (≈ 455 µs) the schedule falls through to a
software-fire path. Sub-455 µs is the entire DXL-timing regime we measure,
so the timer-fire path covers the common case.

## 6. Peripheral map

| Peripheral | Role                                                                |
| ---------- | ------------------------------------------------------------------- |
| TIM1       | unused (reserved for future debug instrumentation)                  |
| TIM2       | master, low 16 of `tick32`; CKD=`DIV_1` (fDTS = HCLK = 144 MHz);    |
|            | CC3 IC on PB10 + per-baud IC3F filter, DMA1_CH1;                    |
|            | UEV + CC1/CC2/CC4 quarter-wrap walker cadence (§3.2); TIM2_RM=0b10  |
|            | partial remap for CH3/CH4                                           |
| TIM3       | slave, high 16 of `tick32` (TIM2 TRGO → ITR1, ext-clock-mode-1)     |
| TIM4       | TX inject OPM fire (CC2 → DMA1_CH4 stamps DMA1_CH2.CR)              |
| SysTick    | embassy time-driver only (qingke V4 64-bit hw counter)              |
| USART3     | full-duplex: TX=PB10 AF OD → DMA1_CH2; RX=PB11 input-pullup →       |
|            | DMA1_CH3; PB10/PB11 bridged externally to one DXL line;             |
|            | IDLE → signal-only                                                  |
| DMA1_CH1   | TIM2_CH3 capture → `falling_ring`; no IRQ                           |
| DMA1_CH2   | USART3 TX (per-fire)                                                |
| DMA1_CH3   | USART3 RX → `rx_ring`; no IRQ                                       |
| DMA1_CH4   | TIM4_CC2-triggered stamp of EN=1 over DMA1_CH2.CR                   |

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
                            <cause>:none|walker_late|ic_overrun|stamp_overflow
                            <last_tick>:u32

Always responds, even mid-desync — the host's one-shot health probe.
`desynced` is `1` iff `cause ≠ none`; both are reported for parse
simplicity.

    RESET               → OK

Clears `DESYNCED` + cause, drains stamp/IC rings, re-arms walker. Keeps
baud. The routine recovery for any of the three desync causes (§3.5).

    BAUD <bps>          → OK | ERR baud

Changes baud; implicit `RESET` inside. Caller must quiesce the bus first.

### Diagnostics

    BTRACE              → BTRACE <phase> <intfr_post_clear> <cnt_entry>
                            <cnt_exit> <falling_pending_entry>
                            <edges_consumed> <bytes_emitted>
                          | EMPTY
    BTRACECLEAR         → OK

Each `BTRACE` record is one TIM2 ISR invocation (phase ∈ {UIF=0, CC1=1,
CC2=2, CC4=3, MULTI=4, SPURIOUS=5}). `intfr_post_clear` re-reads INTFR
after the cadence-flag clear write — a non-zero value means a flag
latched in the read→clear window and was silently dropped (the
`DESYNCED` contract relies on this never happening). The other fields
measure ISR latency, queue depth, and walker workload per ISR. The
trace ring is 64 entries deep; when the host falls behind, the oldest
records get overwritten and `BTRACE` snaps the tail forward.

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
- **Walker outpaces TIM2 wraps.** UEV + CC1/CC2/CC4 give 4 walker
  invocations per TIM2 wrap (~114 µs cadence), so any IC entry is at
  most one quarter wrap old when the walker sees it. The
  backward-from-now lift in §3.4 needs only one-wrap-of-bound to place
  each entry in the correct high-word window; we have 4× headroom on
  that, no software multi-wrap bookkeeping, no per-edge ISR cost.
  Works identically at 9600 baud and 3 Mbaud.
- **Lockstep window classification, no across-byte prediction.** Each
  byte's stamp anchors on its own first unconsumed IC edge; edges within
  `[anchor, anchor + 10·bit]` are that byte's. There is no
  `ts[k−1] + 10·bit` prediction, no across-byte state to go stale on a
  quiet bus; bootstrap, baud-change resume, and inter-packet "resync" are
  all the same code path. Edge count drives validation flags only, never
  walker advancement.
- **Fail loud, never silently recover.** Three conditions — two
  designed-impossible (`walker_late`, `ic_overrun`) and one host-paced
  bench-script bug (`stamp_overflow`) — flip a single sticky `DESYNCED`
  flag. Every host command then errors out until `RESET` (or `BAUD`,
  which implicitly resets). A walker that silently re-acquires would
  mask whatever bug let a designed-impossible failure fire; silently
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
  stamps, and bus-anomalous from clean conditions. A ruler that silently
  produces wrong measurements under noise is worse than one that flags its
  own limits.

## 9. ISR + DMA priority discipline

Two priority axes — PFIC (CPU interrupt preemption) and DMA arbiter (per-
channel CHCFG.PL) — both matter for keeping the measurement clock honest
under burst load. The doc previously glossed over them; here's the contract
the firmware enforces.

### 9.1 PFIC interrupt priorities

| Class         | IPRIOR | IRQs                                  | Why                                  |
|---------------|--------|---------------------------------------|--------------------------------------|
| `PRIO_WALKER` | `0x00` | `TIM2`, `USART3`                      | Single walker class; ISRs share it so `walk()` is single-threaded by construction. |
| `PRIO_USB`    | `0x80` | `USB_LP_CAN1_RX0`                     | Lower preempt class — USB stack delays cannot delay a wire-side stamp. |

`TIM2` covers all four walker-cadence events (UEV, CC1, CC2, CC4) plus the
CC3 IC capture flag — they mux into one vector. `USART3` carries only IDLE
(signal-only per §3.2). DMA1_CH1/CH2/CH3/CH4 have **no IRQ enabled** —
their PFIC slots are intentionally unconfigured.

### 9.2 DMA arbiter priorities (CHCFG.PL)

The DMA arbiter serializes simultaneous transfers; without explicit PL the
default is LOW for every channel. At 3 Mbaud burst the IC ring fills at
~167 ns per edge, so an arbiter delay caused by a coincident USB DMA can
overrun the ring before the walker drains it.

| Channel        | PL          | Why                                                          |
|----------------|-------------|--------------------------------------------------------------|
| CH1 (IC)       | `VERYHIGH`  | Measurement clock; arbiter delay → ring overrun → lost edges |
| CH3 (USART3_RX)| `HIGH`      | Must not drop bytes at 3 Mbaud; one tier below CH1           |
| CH2 (USART3_TX)| `MEDIUM`    | Paced by USART; arbiter delay is harmless (TX FIFO absorbs)  |
| CH4 (CC2 stamp)| `MEDIUM`    | One-shot per fire; not throughput-limited                    |
| (USB)          | `LOW` (default) | USB is throughput-bursty but tolerates µs-scale arbiter latency |

CH1 outranks CH3 so a back-to-back IC + RX simultaneous request (typical
at every byte's start bit) resolves to "stamp the edge before the byte
value lands" — i.e. `falling_ring` always leads `rx_ring`, never lags.
That matches the walker's invariant (§3.1: `ts_head ≤ rx_total`, sourced
from IC entries).

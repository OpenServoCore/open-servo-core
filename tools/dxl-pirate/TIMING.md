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

### 3.2 Walker cadence

One trigger source, four interrupts per TIM2 wrap, single IRQ vector:

| Event             | TIM2 register | Fires at      | Role                  |
|-------------------|---------------|---------------|-----------------------|
| Update (UEV)      | UIF           | CNT = 0x0000  | walker tick           |
| Compare CC1       | CC1IF         | CNT = 0x4000  | walker tick           |
| Compare CC2       | CC2IF         | CNT = 0x8000  | walker tick           |
| Compare CC4       | CC4IF         | CNT = 0xC000  | walker tick           |

CC1/CC2/CC4 in output-compare-frozen mode (OCxM=0b000, no pin, no DMA);
only their CCxIF flag matters. All four events mux into the TIM2 IRQ
vector — `walk()` runs single-threaded by construction, no concurrent
walker hazard. Cadence: 16384 ticks ≈ **114 µs** between walker entries
at 144 MHz.

CC3 stays in input-capture mode driving DMA1_CH1; its CCxDE bit is set,
its CCxIE is not — capture remains zero-CPU per edge.

This cadence bounds every IC entry's age at walker entry to one quarter
wrap (16384 ticks), which is what the §3.4 lift formula needs. No
high-baud HT/TC trigger, no IDLE-driven flush, no low-baud edge-density
argument: the bound holds at every baud, every bus state, by the timer
cadence alone.

Per `[[no_idle_timing]]`: USART3 IDLE is **signal-only** — it hands off
to the `inject::on_idle` path for `schedule_fire_after_idle` consumers
and does not touch walker state. The closing packet's tail gets stamped
at the next TIM2 walker tick (≤114 µs later).

### 3.3 Constructing `lift_ceiling`

`lift_ceiling` is the lift formula's reference point — **not** a
sampling of "now" — constructed from the trigger identity, not read
from a register. Naming intent: the value is a **ceiling on raw IC
entry CNTs visible at this walker entry**, used by §3.4's backward
lift to land each entry in the correct TIM3 wrap.

The four walker triggers fire at hardware-deterministic TIM2.CNT
phases; each phase has a corresponding `lift_ceiling.lo`:

| Trigger     | Fires at TIM2.CNT | `lift_ceiling.lo`    |
|-------------|-------------------|----------------------|
| UEV (UIF)   | 0x0000            | 0x3FFF               |
| CC1 (CC1IF) | 0x4000            | 0x7FFF               |
| CC2 (CC2IF) | 0x8000            | 0xBFFF               |
| CC4 (CC4IF) | 0xC000            | 0xFFFF               |

Each `lift_ceiling.lo` is `phase + cadence − 1` — the maximum CNT
possible within one cadence period before the next trigger fires. With
ISR latency bounded below one cadence by the `walker_late` contract
(§3.5), this is a safe upper bound on actual TIM2.CNT at every walker
entry. The bare `phase` value is **not** correct here: by the time the
ISR runs, TIM2.CNT has advanced to `phase + Δ`, and any IC entry
captured during Δ has raw u16 > phase. The §3.4 lift formula requires
`lift_ceiling.lo ≥ entry.lo` to place the entry in the correct wrap,
so the upper-bound value is load-bearing.

`lift_ceiling.lo`'s specific value within the wrap does **not** affect
the lifted u32 — `lift(lift_ceiling, entry)` returns the same u32 for
any `lift_ceiling` whose hi half matches the entry's wrap, as long as
`lift_ceiling.lo ≥ entry.lo`. The upper-bound constant therefore costs
no precision and saves the `TIM2.CNT` register read.

TIM3 only advances on TIM2 TRGO (= UEV); its CNT is stable between UEVs,
so a single load is race-free at every cadence point. Combine:

    lo = 0xFFFF if CC4IF
       else 0xBFFF if CC2IF
       else 0x7FFF if CC1IF
       else 0x3FFF                  // UIF
    lift_ceiling = (TIM3.CNT << 16) | lo

One register read (`TIM3.CNT`), no coherence loop, no `TIM2.CNT` access.
By design (§3.5) the walker can never see more than one of {UIF, CC1IF,
CC2IF, CC4IF} pending at entry, so the resolution is unambiguous.

### 3.4 Lockstep window walker

Walker state across calls (entire surface):

    walked    : u32  — count of IC entries consumed, monotonic
    byte_head : u32  — count of bytes emitted, monotonic

That's it. No `LAST_TS`, no `PREV_VALID`, no `PENDING_RECOVER`. Those two
producer counters and the `DESYNCED` sticky flag (§3.5) are the
walker's entire state.

#### Lift

For each IC entry to be classified, lift to u32 by walking **backward**
from the §3.3 `lift_ceiling`:

    delta  = (lift_ceiling as u16).wrapping_sub(entry)   // u16 modular
    lifted = lift_ceiling.wrapping_sub(delta as u32)

`lifted` is the most recent u32 whose low 16 bits equal `entry`, always
≤ `lift_ceiling`. Valid as long as actual age of the entry < 65536
ticks at walker entry — guaranteed with 4× headroom by §3.2's
quarter-wrap walker cadence (16384 ticks ≈ 114 µs at 144 MHz). Each
entry is lifted independently against `lift_ceiling`; no forward chain
across entries. Mirrors `firmware/lib/drivers/.../edge_parser::lift` in
production firmware.

#### Classification

The algorithm is **time-windowed, lockstep, no across-byte prediction**.
For each byte `B` newly present in `rx_ring`:

1. **Anchor.** The next unconsumed IC entry (lifted) is byte `B`'s
   window anchor. `start_tick = lifted − CC_FILTER_DELAY_TICKS`.
2. **Window.** Edges with `lifted − anchor < 10·bit_time` belong to
   byte `B`. Half-open on the right: an edge at exactly
   `anchor + 10·bit_time` is byte `B+1`'s start bit (which fires
   immediately after `B`'s stop bit) and belongs to the next iteration's
   anchor, not `B`'s window.
3. **Validate.** Compare count to `EDGES_PER_BYTE_FALLING[B]`. Set
   `COUNT_OVER` if count exceeds expected + `NOISE_TOLERANCE`;
   `COUNT_UNDER` if count falls short.
4. **Emit.** `(B, start_tick, flags)` into the three parallel rings;
   advance `byte_head`.

If `walked == falling_total` mid-byte, the walker yields and waits for
the next cadence — it never invents a stamp and never skips ahead.

**Key property: no across-byte state.** Each byte's window is anchored
on **its own** first unconsumed IC edge, not on `ts[B−1] + 10·bit`.
Bootstrap, post-baud-change resume, and inter-packet "resync" are all
the same code path — every byte's classification is self-contained.
There is no inter-byte prediction that could go stale during a quiet
bus.

**Count drives validation, never advancement.** A glitch surviving the
CC filter inside byte `B`'s window inflates `B`'s count (→
`COUNT_OVER`), but byte `B+1`'s anchor is the first edge *outside* `B`'s
window — no cascade. A missing edge inside `B`'s window flags
`COUNT_UNDER`, same story: `B+1`'s anchor is unaffected.

#### Trade-off

A glitch surviving the CC filter **during a long idle gap between
packets** corrupts the next packet's byte 0. Because the walker anchors
on the next available edge regardless of when it arrived, a stale glitch
sitting in `falling_ring` becomes byte 0's start tick. Downstream bytes
shift by the glitch-to-real-start-bit gap; `COUNT_UNDER` (or
`COUNT_OVER`) flags the affected byte once its edge pattern no longer
matches `EDGES_PER_BYTE_FALLING`.

Mitigations explicitly rejected (per design constraints):

- IDLE-driven flush of stale edges — violates `[[no_idle_timing]]` and
  the "IDLE doesn't touch walker state" contract.
- Predicted-window search around `ts[B−1] + 10·bit` — reintroduces the
  across-byte prediction state and resync complexity that this design
  exists to eliminate.
- Best-effort invented stamps when no edge lands in window — the walker
  never invents; the corresponding `STAMP_BESTEFFORT` flag is gone (§4).

On a clean DXL bus with proper grounding the CC filter eats glitches
before they reach `falling_ring`. If one slips through during idle, the
host sees `COUNT_OVER`/`COUNT_UNDER` and can drop the packet. **Surfacing
the anomaly is the contract; silent recovery is not.**

### 3.5 Desync contract

Three conditions trip a single sticky `DESYNCED` flag — two
designed-impossible internal failures plus one host-paced bench-script
bug. Once tripped, the walker stops emitting and the host protocol
surface fails loud; recovery is the explicit `RESET` command (or `BAUD`,
which implicitly resets).

| Cause            | Check                                                    | Class                              |
|------------------|----------------------------------------------------------|------------------------------------|
| `walker_late`    | More than one of {UIF, CC1IF, CC2IF, CC4IF} set at entry | Designed-impossible — `PRIO_WALKER=0x00` tops preempt class; walker drains faster than 3 Mbaud fills the IC ring. |
| `ic_overrun`     | `falling_total − walked > FALL_LEN`                      | Designed-impossible — IC ring (256 entries) fills slower than the walker drains it (~200 edges / 114 µs). |
| `stamp_overflow` | At emit, `byte_head − byte_tail ≥ STAMP_LEN`             | Host-paced — bench script didn't drain via BBATCH; ring depth is 1024 (~10 ms at 1 Mbaud). |

When any trips:

    DESYNC_CAUSE.compare_exchange(0, cause, …)   // first trip wins
    DESYNCED.store(true, Release)
    // walker stops emitting; rx_ring + falling_ring keep filling but
    // are forfeit — next RESET drops them.

Subsequent host commands return `ERR desync <cause>` — except `STATUS`
(which reports the cause), `RESET` (which clears it), and `BAUD` (which
implicitly resets via `reset_walker`). Recovery is one of those three
commands, not a power cycle.

The two designed-impossible causes still mean what they always did: if
`walker_late` or `ic_overrun` ever fires, that's a hard bug worth
investigating — a clean RESET clears the symptom but doesn't fix the
underlying timing budget violation. `stamp_overflow` is the expected
"your test script needs to drain more often" signal; it's a contract
surface, not a bug indicator.

The walker hot path may be placed in SRAM via `qingke-rt`'s
`#[interrupt(highcode)]` attribute to shave flash-wait-state cycles and
reinforce the latency budget that keeps `walker_late` impossible.

## 4. Validation contract

### Edge-count LUT

`EDGES_PER_BYTE_FALLING[B]` is the number of `1→0` transitions structurally
produced by the wire bit stream `[idle=1, start=0, b₀..b₇, stop=1, idle=1]`:

    falling[B] = 1 + (B & !(B >> 1) & 0x7F).count_ones()

Range: 1..=5. The wire **always** produces at least this count for byte
`B`; real conditions only add edges (EMI glitches that survived the CC
filter, ringing on direction reversals, etc.) — never remove them.

### Two slacks

| Slack       | Absorbs                                                      | Tolerance               |
|-------------|--------------------------------------------------------------|-------------------------|
| Edge count  | EMI surviving the filter, slow ringing                       | `+NOISE_TOLERANCE`; under never |
| Timing      | HSI drift, transceiver propagation, DMA arbiter latency      | `±1·bit` on `10·bit` cross-byte spacing |

`NOISE_TOLERANCE` starts at 1–2 per byte; bench-tuned.

### Anomaly flags

Each byte carries one `u8` of flags:

| Bit | Name              | Meaning                                                 |
|-----|-------------------|---------------------------------------------------------|
| 0   | `COUNT_OVER`      | actual falling > expected — EMI glitch survived filter  |
| 1   | `COUNT_UNDER`     | actual falling < expected — signal loss / filter ate edge|
| 2-7 | reserved          |                                                         |

Every emitted stamp uses a real hardware IC edge — the walker never
invents a `start_tick`. `flags == 0` means the wire byte's edge count
matched `EDGES_PER_BYTE_FALLING[B]` exactly. Non-zero flags signal wire
anomalies (EMI glitch survived the filter, edge loss inside the filter);
the stamp itself remains hardware-precise.

`TIMING_DRIFT`, `TIMING_RECOVER`, and `STAMP_BESTEFFORT` are absent by
design: the walker has no `ts[k−1] + 10·bit` prediction to drift against,
no recovery path it could re-engage from, and no condition under which it
would invent a stamp. If the walker ever encounters a designed-impossible
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

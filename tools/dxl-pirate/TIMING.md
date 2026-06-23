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

One physical pin, three peripheral block taps, zero CPU per edge.

    PA2 ──┬── USART2 HDSEL  ─► DMA1_CH6 ─► rx_ring     (DMA-populated)
          │                    DMA1_CH7 ◄─ TX burst (per-fire, §5)
          │
          └── TIM2_CH3 IC   ─► DMA1_CH1 ─► falling_ring (internal)
              (falling, CC-filtered)

USART2 in half-duplex (HDSEL) mode owns PA2 as a single open-drain
bidirectional pin. TIM2_CH3 input capture taps the same pin via GPIO_INDR;
no AFIO conflict because IC reads the input register regardless of the USART
block's output state.

DMA1_CH1 streams 16-bit IC captures into `falling_ring`. **No per-edge ISR**
— at 3 Mbaud worst case that's the difference between a usable instrument
and a wedged one. The walker (§3.4) drains the ring on periodic and
event-driven triggers, forward-walks both rings in lockstep, and produces
per-byte stamps.

### 3.1 Ring layout

Three rings exposed to the host, one internal:

| Ring           | Width | Producer  | Role                                  |
|----------------|-------|-----------|---------------------------------------|
| `falling_ring` | u16   | DMA1_CH1  | internal — raw IC stamps              |
| `rx_ring[i]`   | u8    | DMA1_CH6  | byte values from USART                |
| `ts_ring[i]`   | u32   | walker    | byte start tick (lifted to `tick32`)  |
| `flags_ring[i]`| u8    | walker    | anomaly flags per byte                |

`rx_ring`, `ts_ring`, and `flags_ring` are **parallel arrays** indexed in
lockstep. One byte's complete record is `(rx[i], ts[i], flags[i])`.

Producer invariant: `ts_head ≤ rx_head`. The walker can't stamp a byte that
hasn't arrived; the host reads up to `ts_head`.

### 3.2 Walker triggers

The walker is idempotent — it processes whatever's new since the last call.
Any of these invoke it:

| Trigger              | Source                              | Role                                       |
|----------------------|-------------------------------------|--------------------------------------------|
| TIM2 update IRQ      | TIM2 wrap (every 455 µs at 144 MHz) | guaranteed re-anchor floor                 |
| `falling_ring` HT/TC | DMA1_CH1                            | high-baud edge-throughput trigger          |
| `rx_ring` HT/TC      | DMA1_CH6                            | per-byte-batch granularity at any baud     |
| USART IDLE           | USART2                              | packet-boundary flush (signal-only)        |

The TIM2 update trigger is **load-bearing for low-baud correctness.** At
9600 baud one byte time = 1040 µs spans two TIM2 wraps; consecutive IC
entries can sit more than 65536 ticks apart. The single-wrap lift in §3.4
requires the walker to run between any two captures it processes — the
455 µs floor guarantees this regardless of edge density.

Per `[[no_idle_timing]]`: IDLE is a signal-only trigger. The walker may
use IDLE assertion as a hint to flush, but every stamp it emits is sourced
from `falling_ring` IC entries lifted via §3.3, never from IDLE
back-dating.

### 3.3 Coherent (TIM2, TIM3) snapshot

At each walker entry, refresh the walker's `running_tick` from a coherent
32-bit reading:

    loop:
        lo_1 = TIM2.CNT
        hi   = TIM3.CNT
        lo_2 = TIM2.CNT
        break if lo_2 >= lo_1      // no wrap between the two TIM2 reads
    running_tick = (hi << 16) | lo_2

The retry handles the brief window where TIM2 wraps between the first read
and the TIM3 read: `lo_2 < lo_1` means we caught the wrap, so re-read to
sample TIM3 after its update-driven increment. Three loads in the common
case, six in the (rare) wrap-collision case.

### 3.4 Forward walker

Walker state across calls:

    last_falling_idx : ring index, last IC entry consumed
    last_rx_idx      : ring index, last byte consumed
    running_tick     : u32, refreshed from §3.3 at each entry

For each new IC entry from `falling_ring` (oldest first), lift to u32:

    delta        = entry.wrapping_sub(running_tick as u16)   // u16 modular
    running_tick = running_tick.wrapping_add(delta as u32)
    // running_tick is now the lifted u32 stamp for this entry

Valid as long as actual elapsed since the previous entry < 65536 ticks —
guaranteed by the TIM2 update floor (§3.2).

For each new byte `B` from `rx_ring`:

- `expected_start = ts[k−1] + 10·bit_time`
- Search lifted IC entries for the first edge in `[expected_start ± 1·bit]`
  - **Hit** → `start_tick` = that edge's lifted u32 stamp
  - **Miss** → recovery: look ahead for next edge preceded by an idle gap,
    resume from there, emit `TIMING_RECOVER` on the first byte after
    re-engagement
- Walk lifted entries through `[start_tick, start_tick + 10·bit]`; count.
- Compare count against `EDGES_PER_BYTE_FALLING[B]`; set flags.
- Emit `(B, start_tick, flags)` into the three parallel rings.

Producer pointers (`last_falling_idx`, `last_rx_idx`) advance only on
emit; they never regress. `rx_ring` and `ts_ring` stay in lockstep by
construction. The walker position is **always** derived from the previous
byte's `ts[k−1] + 10·bit` — it can never get misaligned by edge-count
miscount.

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
| 2   | `TIMING_DRIFT`    | start edge near window edge — HSI drift accumulating    |
| 3   | `TIMING_RECOVER`  | first byte after recovery — preceding stamps may be lost|
| 4   | `STAMP_BESTEFFORT`| no edge in window; `start_tick` = `expected_start` (no hw stamp) |
| 5-7 | reserved          |                                                         |

`flags == 0` means hardware-precise stamp, fully validated bit pattern.
Non-zero flags tell the host "the stamp may be best-effort or the wire was
weird." Host filters or aggregates as the analysis demands.

### CC filter delay

TIM2 CC3 IC's digital filter (CCMR1 IC3F + CKD) suppresses glitches narrower
than `N·t_sample`. It adds a **systematic per-edge delay** of `N·t_sample`,
identical across all edges — calibratable shift, not jitter widening.

Baseline at 144 MHz: CKD=01 (HCLK/4 = 36 MHz sample), IC3F=1000 (N=6) →
~167 ns delay ≈ 0.5·bit at 3 Mbaud. The walker subtracts a
`CC_FILTER_DELAY_TICKS` constant from every IC stamp before storing into
`ts_ring` so external time comparisons (e.g. against TIM4 OPM fire
deadlines) compose cleanly.

## 5. Wire-side drive (TX)

    host: FIRE bytes=…  at=<tick32>
      └─► TIM4 OPM, ARR = (at − now_tick32) low 16
            └─► CC match enables DMA1_CH7 ─► USART2 DR
                  └─► HDSEL drives PA2

TIM4 in one-pulse mode holds the inject window; its CC match enables
USART2's TX DMA, which streams the prepared payload into the data register.
Hardware-coupled — no CPU between deadline and first wire bit, no
ISR-latency floor on the inject side.

For `at − now_tick32 > 0xFFFF` (≈ 455 µs) the schedule falls through to a
software-fire path. Sub-455 µs is the entire DXL-timing regime we measure,
so the timer-fire path covers the common case.

## 6. Peripheral map

| Peripheral | Role                                                                |
| ---------- | ------------------------------------------------------------------- |
| TIM1       | unused (reserved for future debug instrumentation)                  |
| TIM2       | master, low 16 of `tick32`; CC3 IC on PA2 + CC filter, DMA1_CH1;   |
|            | UP IRQ → walker re-anchor floor (§3.2)                              |
| TIM3       | slave, high 16 of `tick32` (TIM2 TRGO → ITR1, ext-clock-mode-1)     |
| TIM4       | TX inject OPM fire                                                  |
| SysTick    | embassy time-driver only (qingke V4 64-bit hw counter)              |
| USART2     | HDSEL on PA2; TX → DMA1_CH7, RX → DMA1_CH6; IDLE → walker flush     |
| DMA1_CH1   | TIM2_CH3 capture → `falling_ring`; HT/TC → walker trigger           |
| DMA1_CH6   | USART2 RX → `rx_ring`; HT/TC → walker trigger                       |
| DMA1_CH7   | USART2 TX (per-fire)                                                |

## 7. Host protocol surface

Per-byte stamps cross the USB-CDC boundary via:

    BDRAIN              → BSTAMP <tick>:u32 <byte>:u8 <flags>:u8 | EMPTY

    BBATCH <count>:u16  → STREAM <n>:u16 [<tick>:u32 <byte>:u8 <flags>:u8] × n
                          (binary frame; n ≤ count, n ≤ ring contents)

`BBATCH` matters for throughput: at 3 Mbaud full-RX the byte rate is
~300 kB/s. Per-byte text framing won't keep up; binary streaming amortizes
the CDC envelope.

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
- **Walker outpaces TIM2 wraps.** The TIM2 update IRQ guarantees a 455 µs
  floor between walker invocations, so any two IC entries the walker sees
  are separated by at most one TIM2 wrap. This keeps the high-word lift
  single-wrap modular — no software multi-wrap bookkeeping, no per-edge
  ISR cost. Works identically at 9600 baud and 3 Mbaud.
- **Timing-anchored walking, edge counting as validation.** The walker
  never advances on edge counts — only on bit-rate-paced timing windows.
  Misalignment is structurally impossible; edge-count anomalies surface as
  flags, never as walker drift.
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

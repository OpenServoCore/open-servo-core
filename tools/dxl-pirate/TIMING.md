# Pirate Hardware Timing

The dxl-pirate's job is to be a more accurate, more precise, and more reliable
ruler than anything it measures. This doc is the architecture contract for how
that's wired — peripheral assignments, the time domain, and the principles that
keep wire-timing measurements hardware-grounded.

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
TIM2.CNT to zero. No race between high and low halves, no software
bookkeeping, no jitter window where an ISR could perturb the relationship.

| Quantity     | Value           |
| ------------ | --------------- |
| Resolution   | 6.94 ns/tick    |
| Wrap period  | ~29.8 s         |
| Host wire    | `tick32`, always |

`HZ?` replies `144`. Host tools query it at runtime — no compile-time constant
on either side.

## 2. Wire-side capture (RX)

One physical pin, one peripheral block, zero CPU per edge.

    PA2 ──┬── USART2 (HDSEL)  ─► DMA1_CH6 ─► byte ring
          └── TIM2_CH3 input  ─► DMA1_CH1 ─► 16-bit edge-tick ring
                  capture

USART2 in half-duplex (HDSEL) mode owns PA2 as a single open-drain
bidirectional pin. TIM2_CH3 input capture taps the same pin via GPIO_INDR;
no AFIO conflict because IC reads the input register regardless of who's
driving the output. Two peripherals, one wire.

DMA1_CH1 streams 16-bit captures into a circular ring. The HT/TC interrupt
walks ring entries newest→oldest, detects TIM2 wraps by comparing
consecutive 16-bit values, and anchors each entry into 32-bit space using a
`(TIM3, TIM2)` snapshot taken at HT/TC entry. **No per-edge ISR** — at
3 Mbaud on a Fast Sync chain (~1.5 Medges/s worst case) that's the
difference between a usable instrument and a wedged one.

## 3. Wire-side drive (TX)

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

## 4. Peripheral map

| Peripheral | Role                                                            |
| ---------- | --------------------------------------------------------------- |
| TIM1       | unused (reserved for future debug instrumentation)              |
| TIM2       | master, low 16 of `tick32`; CC3 IC on PA2, DMA1_CH1             |
| TIM3       | slave, high 16 of `tick32` (TIM2 TRGO → ITR1, ext-clock-mode-1) |
| TIM4       | TX inject OPM fire                                              |
| SysTick    | embassy time-driver only (qingke V4 64-bit hw counter)          |
| USART2     | HDSEL on PA2; TX → DMA1_CH7, RX → DMA1_CH6                      |
| DMA1_CH1   | TIM2_CH3 capture ring                                           |
| DMA1_CH6   | USART2 RX byte ring                                             |
| DMA1_CH7   | USART2 TX (per-fire)                                            |

## 5. Design principles

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
  only the HT/TC boundary spends cycles.
- **Embassy stays off the wire.** SysTick hosts embassy's time queue for
  USB and utility timers. It does not appear in any wire-timing path, so
  embassy's choice of priority, jitter, and tick rate is orthogonal to
  bench accuracy.
- **TIM1 left free.** No advanced-timer feature is needed today. Keeping
  TIM1 unused leaves room for a future debug instrument (external trigger,
  encoder counter, second IC bank) without rewiring.

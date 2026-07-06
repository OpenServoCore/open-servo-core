# Runtime HSI Trim and Fire-Intercept Calibration for DXL Fast on the CH32V006

> **Status: frozen-track (2026-07-05).** Describes the feature-frozen DXL 2.0
> transport; pairs with the tune-fast-last tools. osc-native has no clock
> calibration at all ([osc-native-protocol.md](osc-native-protocol.md) §9.3).
> The dxl-fast-chain-crc*.md companions linked below are retired to git
> history.

Companion to [dxl-rx-timing.md](dxl-rx-timing.md) and [dxl-fast-chain-crc.md](dxl-fast-chain-crc.md). Read those first — they cover what "wire-end timestamp," "Fast last-slave coalesce," and "the V006's SysTick CMP fire path" mean. This doc covers what to do when the chip's internal clock isn't accurate enough for those mechanisms to work without help.

The V006 has no crystal — its 24 MHz HSI oscillator drifts ±1% from chip to chip and with temperature. For most embedded work that's fine. For DXL Fast mode at 3 Mbaud, where slaves stitch their replies onto the bus with less than ~3.3 µs of slop, ±1% drift turns into 15+ µs of wire-side error on a 128-byte predecessor — guaranteed bus collision or visible gap, either way coalesce breaks.

The chip exposes a 5-bit `HSITRIM` field that nudges the oscillator in ~0.25% steps, which gets us close. But "close" is still ±0.125% on a good day, and at 3 Mbaud over 128 bytes that residual is ~0.5 µs — near the cliff. We also have to compensate for the chip's own ISR + DMA latency between "SysTick deadline reached" and "first bit on wire," which is another few µs of *additive* slop. So on top of HSITRIM we expose two additional µs-granularity compensations: one for the structural ISR+DMA latency (fixed per chip family, per dispatcher path), and one for the per-chip leftover that the discrete HSITRIM steps couldn't quite resolve.

The slave can't reliably measure its own clock — measuring HSI with HSI is structurally biased (§5). So instead, the master sends `CAL(N)`, the slave responds with N zero bytes, and a crystal-accurate timer on the bus stamps the slave's response. From that single round trip the master derives both knobs: drift from the response duration, and the per-chip fire residual from the rounding remainder when discrete trim steps don't perfectly hit nominal. Master writes both back via standard `Write`. Slave applies on its next idle. Re-derived at every boot — HSI is temperature-sensitive, so cached cal values from a previous power cycle are stale before the first packet anyway.

---

## 1. Why HSI drift matters for DXL Fast

The whole point of Fast Sync/Bulk Read (see [dxl-fast-chain-crc.md](dxl-fast-chain-crc.md)) is that all addressed slaves' replies stitch into one coalesced Status frame on the wire with zero idle gap between slots. The host's USART recovers it as one big packet. The jitter cap for inter-slave hand-off is one byte time — 3.33 µs at 3 Mbaud. Land later than that and the host's USART asserts IDLE between slots; many masters tolerate this in practice but the frame is no longer spec-compliant, and the larger the gap, the higher the risk of bus contention when the next slot starts firing into a slow trailing edge.

The last slave's reply has to start at exactly:

    fire_us = RDT + bytes_before × byte_time

where `bytes_before` is the predecessors' total wire bytes already on the bus. That's `fire_us` measured in *real* microseconds of wall-clock time.

But the slave doesn't *have* wall-clock time — it has SysTick, which counts HCLK cycles. SysTick rate = HCLK = HSI × PLL_mult. If HSI is off by ε, SysTick is off by ε, and so is the slave's belief about how long `bytes_before × byte_time` is.

If HSI is **slow** by 1%: SysTick under-counts, the slave thinks the deadline is at `fire_us` but in real time it lands at `fire_us × 1.01`. That's a positive gap — the slave fires *after* the predecessor's wire-end. Visible idle, possible IDLE assertion on the host, coalesce broken.

If HSI is **fast** by 1%: the slave fires `fire_us × 0.99` in real time. Negative gap — overlap with the predecessor's last byte. Wire collision, CRC corruption, frame lost.

At the spec boundary (±1% HSI):

    1 Mbaud, 128-byte predecessor:
        predecessor wire time = 138 × 10 µs = 1380 µs
        drift bound at ±1%    = ±13.8 µs
    3 Mbaud, 128-byte predecessor:
        predecessor wire time = 138 × 3.33 µs = 460 µs
        drift bound at ±1%    = ±4.6 µs       ← already over the 3.33 µs jitter cap

Drift scales linearly with predecessor wire time (as it must — the longer the wire window, the more drift accumulates), so the bound only gets worse for longer chains. And the V006 spec is ±1% guaranteed; nothing prevents an individual chip from running wider.

For Fast to work cleanly across all bauds and predecessor sizes, the drift has to come out. The chip's own SysTick is the only on-chip time reference, so we have to fix HSI itself — or, as we'll see in §5, bring in a clock from outside the chip that *isn't* HSI to measure against.

---

## 2. The HSITRIM lever, and why it's only half the story

V006 RM §6 documents a 5-bit `HSITRIM` field in `RCC_CTLR[7:3]`, RW, default 16 (binary `10000b`). Each step shifts HSI by ~60 kHz at the nominal 24 MHz — about **0.25% per step** (2500 ppm/step). 5 bits give ±16 steps of range; the manual quotes ±1% as guaranteed, but the silicon takes more in practice. The factory `HSICAL[7:0]` byte (bits [15:8] of the same register, read-only) is the per-chip calibration baseline; `HSITRIM` is the *adjustment* superimposed on it.

The good news:

- A direct knob for correcting per-part HSI offset.
- Runtime-writable in a single register store — sub-microsecond, no flash controller, no CPU stall on the write itself.
- Affects HSI itself → HCLK, USART BRR, and SysTick all scale uniformly → **baud-independent**. Cal at 1 Mbaud, switch the bus to 3 Mbaud for operation, the trim applied at 1M is still optimal at 3M.

The catch:

- **0.25% steps leave ±0.125% residual** even with a perfect cal landing. At 3 Mbaud with a 128-byte predecessor that's ±0.53 µs of slop — uncomfortably close to the 3.33 µs jitter cap.
- The trim only fires whole steps, so if a chip's true optimum lies between two integer steps, you have to pick one. Pick the wrong side and you get bus collisions even though the cal "worked" by its own arithmetic. We need a finer-grained knob, expressible in µs rather than HSITRIM-step units, to take up the slack.

**Direction**: higher HSITRIM = faster HSI. So a slow chip (positive observed gap) needs HSITRIM *raised*; a fast chip needs it *lowered*.

The finer-grained correction comes from two additive µs compensations layered on top of HSITRIM. The next section decomposes them.

---

## 3. Decomposing the wire gap

Write the wire-side gap between a predecessor's last byte and the slave's reply as a function of how many bytes the predecessor had. It decomposes into a slope and an intercept:

    gap_us = drift × bytes_before × byte_time   +   structural_latency
             └─────────── slope ───────────┘       └──── intercept ───┘

The slope and the intercept are independent — changing one doesn't affect the other — so they're correctable independently. And the intercept itself splits into two contributors with different lifecycles. Three phenomena total, each named and corrected below.

### Phenomenon 1 — HSI drift (the slope)

The slave's HCLK is off from nominal by some ε. SysTick under- or over-counts proportionally, so "fire the deadline N µs from now" stretches or compresses by the same ε. The effect on the wire gap scales with how much wire time the predecessor occupies — a 1% slow chip fires 1% late, which on a 1 ms predecessor is 10 µs late and on a 100 µs predecessor is 1 µs late.

Corrected by the chip's hardware trim register (HSITRIM on V006), in ~0.25% steps. Coarse correction, leaves ±0.125% residual when the chip's true optimum lies between integer steps.

### Phenomenon 2 — structural latency (intercept, family-constant, per fire path)

The chip's own time from "SysTick deadline reached" to "first bit on wire": PFIC trap entry + ISR body + DMA prefetch + start-bit latch. Every component is a deterministic number of HCLK cycles, so the whole intercept is a fixed value per chip family — same whether the predecessor was 4 bytes or 4000, same across every instance of the family.

Corrected by **two** per-chip-family compile-time constants in HCLK ticks — one per dispatcher fire path: `PLAIN_ENTRY_TICKS` for the unicast reply path and `FAST_ENTRY_TICKS` for the Fast Sync/Bulk chain path. The Fast path does extra work before `fire_now` (FSM transition, snoop-CRC scaffolding) and so has a larger effective latency. A single shared knob forces one path to overshoot; the split nulls both.

Both constants live in `firmware/ch32/src/measurements.rs` alongside `CATCHUP_ENTRY_TICKS`. They are silicon-fixed per chip family; the host has no runtime override path. See `dxl-fast-chain-crc-walkloop.md` §1.1 for the scope methodology to re-measure when the affecting code path, PFIC priorities, or HCLK frequency change.

### Phenomenon 3 — sub-trim drift residual (intercept, per-chip)

HSITRIM only fires whole steps (~0.25% on V006), so even a perfect cal lands within ±half a step of the true optimum — about ±0.125% residual drift. Strictly speaking this is residual slope (phenomenon 1, partly uncorrected), but once we pick a worst-case predecessor length to design for, the residual collapses to a fixed µs value at that length and behaves like additional intercept. Per-chip: every chip lands in a slightly different spot relative to the integer trim steps.

Corrected by a per-chip runtime µs value, computed by the master from the trim-step rounding remainder. The slave receives it through `comms.clock_fine_trim_us` (i16 Q8.8 µs).

### Why all three are needed

- HSITRIM alone leaves ±0.125% residual *plus* the entire structural latency — maybe 4 µs at 3 Mbaud, over the jitter cap.
- Structural compensation alone has no way to handle the slope.
- Sub-trim residual compensation only makes sense layered on top of HSITRIM doing its main job.

The two intercept compensations are additive and unit-compatible (both are "advance the fire by this many µs"), so the firmware sums them into a single tick count at apply time. The split lives in the storage layout (§8) — different lifecycles call for different storage — not in the runtime fire path, which sees one number per dispatcher path.

---

## 4. Storage and why we re-cal at every boot

`comms.clock_trim` (i8, signed delta from factory HSITRIM) and `comms.clock_fine_trim_us` (i16 Q8.8 µs) live in the EEPROM section of the control table. Writes persist across power cycles (subject to DXL's standard "no EEPROM writes while torque is on" convention to avoid CPU-stalling the motor control loop during a flash commit).

Persistence doesn't replace boot-time recalibration. Re-cal at every boot anyway, for three reasons:

- **Temperature.** The V006's HSI carries roughly 0.05%/°C tempco. A cal value written at 25 °C bench conditions isn't right for a chip starting up at 60 °C in a heated robot enclosure. Re-cal at boot tracks actual startup conditions.
- **Supply.** HSI also varies slightly with VDD; re-cal absorbs whatever the supply rail looks like at startup.
- **Aging.** Over years, RC oscillator parts drift. Re-cal is automatic re-characterization.

Stored values are still useful as warm-start seeds — they shorten convergence when the operating environment is similar to the last save. But the boot-time CAL pass is cheap enough (~15 ms per slave at 1 Mbaud, ~350 ms for a 16-slave arm — §10) that re-cal from scratch is comfortably within boot budget too. The master can choose either path.

Cal lands before torque-on, which sidesteps two issues at once: EEPROM writes are only allowed under DXL's "no writes while torque is on" convention (since they ultimately commit to flash, with a CPU stall), and HSITRIM application shifts HCLK, which shifts SysTick rate, which would perturb a running control loop. Mid-operation re-cal is therefore future work, gated on either a torque-off window or a RAM-only "live trim" variant of the write.

---

## 5. Architecture: master measures, slave just responds

The chip can't measure itself reliably. SysTick is HSI-derived. USART IDLE assertion is HCLK-gated. ISR entry latency is HCLK-quantized. Every term in a chip-side timing measurement is itself a function of the unknown we're trying to characterize — there's no Archimedean lever to push against from inside the slave. Any "chip times itself" scheme picks up structural bias from the gap between when wire events physically happen and when the chip can sample its own clock against them, and that bias is on the same order of magnitude as the drift we're trying to detect.

The fix is to put the measurement on a clock domain that *isn't* the one under test. Three actors:

- **Slave**: generates a known-shape stimulus (N zero bytes at the bus baud) on the `CAL` command. No timing decisions, no math, no state machine.
- **HSE-accurate timer on the bus**: stamps the slave's first and last response bytes (and the master's last request byte) in HSE-derived ticks.
  - *On the bench*: the V20x-class pirate, sitting passive on the bus and reporting timestamps via its CDC channel.
  - *In production*: the MCU master itself (V20x/V30x), using USART RX input-capture clocked from its own HSE crystal. The DUT MCU never sees this timer.
- **Host**: queries the timer for stamps, does all the math, writes results back via standard DXL `Write`.

The wire op is identical bench and production — only the timing source swaps. On the bench, pirate stamps are queried via CDC; in production, the master reads its own timer's CCR after each byte. Both surface the same `(T_request_end, T_first_slave, T_last_slave)` tuple per slave response, in HSE ticks.

**Why this escapes the chip-side bias.** When the reference clock is external (HSE), the host measures the same physical wire events with the same hardware on every byte. Any systematic offset in the host's stamping is in the host's budget, not divided through HSI's drift, and it cancels in `T_last − T_first` rather than masquerading as drift.

---

## 6. The CAL instruction

One vendor DXL instruction. The slave receives the request, fills its TX buffer with `count` bytes of zero, sends a standard Status reply through its existing send path. No new chip-side timing state, no math, no apply-after-TC dance for this op itself.

    CAL = 0xE0    (vendor instruction byte, pick once)

**Request frame:**

    [ 0xFF 0xFF 0xFD 0x00 ] [ id ] [ length=5 LE ] [ 0xE0 ]
        [ count_lo ] [ count_hi ] [ CRC ]

`count` is the requested response payload byte count, `u16` little-endian, bounded `1 ≤ count ≤ 128`. Master errors out before sending if `count` is out of range.

**Status reply:**

    [ 0xFF 0xFF 0xFD 0x00 ] [ id ] [ length LE ] [ 0x55 ] [ err ]
        [ 0x00 ... 0x00 (count bytes) ] [ CRC ]

`data` is `count` zero bytes. Content is irrelevant — the measurement is purely timing-based. Slave clamps `count > 128` to 128 and returns `err = 0x07` (Data Range) with empty payload; this is impossible if the master honors the ceiling but the slave checks anyway.

**Why N ≤ 128.** This gives slack to other slave implementations that may have smaller TX buffers, keeps `DXL_TX_BUF_LEN = 256` on the V006 with plenty of margin (`128 + 11 framing = 139`), and avoids the DMA stream complexity (MINC=0 chaining) that would only be needed for N > 245. At 1 Mbaud the resulting 1.27 ms wire response carries plenty of signal. Higher bauds can multi-shot to recover SNR if needed.

**Why one instruction.** The slave doesn't surface any state — the master has all the timing it needs. The "apply" step is just a normal `Write(comms.clock_trim, ...)` and `Write(comms.clock_fine_trim_us, ...)` against the existing apply-after-TC machinery. CAL is a pure stimulus instruction.

---

## 7. The math: one CAL transaction → both knobs

From a single CAL round trip the master gets three HSE-stamped events:

    T_request_end:  master's last bit hit the wire
    T_first_slave:  slave's first reply bit hit the wire
    T_last_slave:   slave's last reply bit hit the wire

Plus it already knows: `N` (count it requested), `baud_hz`, `RDT` (slave's reply delay, typically 0), and `hse_hz` (its own timer's tick rate).

### 7.1 Deriving drift

The slave's TX clock is its own HSI (via the BRR divider). The time it actually takes to emit `(M − 1)` byte intervals at the bus baud is what the HSE timer measures directly, where `M` is the **total Status frame length** (header + framing + the N zero payload bytes + CRC = `N + 11` for the standard Status overhead). `T_first` lands at end-of-first-frame-byte, `T_last` at end-of-last-frame-byte, so the span covers the whole frame — not just the zero payload:

    M           = N + 11                                  // header(4)+id(1)+len(2)+instr(1)+err(1)+crc(2)
    observed_us = (T_last_slave - T_first_slave) / hse_hz * 1_000_000
    nominal_us  = (M - 1) * 10 / baud_hz * 1_000_000      // 10 bits per UART byte (8N1)
    drift_ppm   = (observed_us - nominal_us) / nominal_us * 1_000_000

Sign reading (higher HSITRIM = faster HSI, per V006 RM §6 and verified on hardware):

- `observed > nominal` → slave took *longer* than nominal → slave's HSI is **slow** → `drift_ppm > 0` → **raise** HSITRIM (positive step).
- `observed < nominal` → slave's HSI is **fast** → `drift_ppm < 0` → **lower** HSITRIM (negative step).

The natural sign of `drift_ppm` is exactly what we want to add to the trim register — no flip:

    step_exact = drift_ppm / ppm_per_step                 // real-valued, will be rounded

### 7.2 Biased rounding (so the intercept compensation can always advance fire)

Normal `round()` would land us on either side of optimal — half the chips end up fast, half slow. But the intercept compensation only *advances* fire (subtracts ticks from the deadline); it can't push fire *later*. A chip on the fast side has no way to be corrected by the intercept alone, and ends up firing into the predecessor's last byte — a bus collision that the cal arithmetic considered "good enough."

The fix is to use `floor()` (round toward −∞), which guarantees we always land on the slow side (residual ≥ 0):

    step         = floor(drift_ppm / ppm_per_step)
    residual_ppm = drift_ppm - step * ppm_per_step         // always in [0, ppm_per_step)

**Worked example 1** — a chip that's 1.25% slow at factory:

    drift_ppm = +12500
    floor(12500 / 2500) = 5  →  step = +5
    HSITRIM = 16 + 5 = 21
    residual_ppm = 12500 - 5 * 2500 = 0           ← lucky landing exactly on a step

**Worked example 2** — a chip that's a bit less slow, say 1.125%:

    drift_ppm = +11250
    floor(11250 / 2500) = 4  →  step = +4
    HSITRIM = 16 + 4 = 20
    residual_ppm = 11250 - 4 * 2500 = +1250        ← still 0.125% slow after trim

That residual is what the per-chip intercept compensation has to take up. It's a single atomic shared across both fire paths — the per-path latency defaults (§3 phenomenon 2) carry the path-specific structural bias, the fine-trim residual carries the per-chip bias.

**Cost of biased rounding**: worst case wastes one trim step of headroom (chip lands at -1 step from optimal instead of ±half-step). V006 has ±16 steps; factory variation typically needs ~4. Plenty of margin.

### 7.3 Deriving the sub-trim drift residual compensation

For a deployment target baud `b_op` and worst-case predecessor wire length `N_target` (defaults: 3 Mbaud, 128 bytes — see §9), the µs gap left by the residual drift is:

    residual_us = residual_ppm / 1_000_000 * N_target * byte_time(b_op)

`residual_ppm` is always ≥ 0 (chip slow after trim, per §7.2 biased rounding), so `residual_us` is always ≥ 0. That's how much *extra* fire-advance has to be applied to compensate. It rolls in on top of the structural latency:

    total_advance_us = structural_us + residual_us

Where `structural_us` is the per-path family default (§3). The master writes the *residual* part to the slave's control table; the slave's firmware sums it with the active path's latency atomic at apply time.

### 7.4 What the master writes back

Two writes:

    Write(slave_id, comms.clock_trim, current_trim + step)             // i8
    Write(slave_id, comms.clock_fine_trim_us, residual_us_q88)         // i16, Q8.8 µs

Both land in pending atomics; both apply in the next USART1 TC interrupt (§11).

---

## 8. Storage layout for the two intercept compensations

The two intercept compensations from §3 have different lifecycles:

1. **Structural latency**: the chip's PFIC + ISR + DMA + start-bit latency. Same for every instance of a given chip family because it's deterministic HCLK cycles. Knowable once, hardcoded as the per-path compile-time constant.
2. **Sub-trim drift residual**: the leftover slope from biased-rounded HSITRIM. Per-chip, recomputed at every boot, varies up to one trim step worth of µs.

They also don't share a unit naturally. Structural is best expressed in HCLK ticks — it *is* a cycle count, so storing it in ticks avoids any µs↔tick conversion in the fire hot path. Drift residual is computed by the master in physical µs (the master doesn't know the slave's HCLK rate) and needs unit conversion at apply time.

So the layout is:

```rust
// firmware/ch32/src/measurements.rs — per chip family, silicon-fixed
pub const PLAIN_ENTRY_TICKS: u32 = ...;                   // unicast path fire latency
pub const FAST_ENTRY_TICKS:  u32 = ...;                   // Fast chain path fire latency

// firmware/ch32/src/statics.rs
pub static FIRE_ADVANCE_FINE_TICKS: AtomicI16 = AtomicI16::new(0);

// firmware/lib/core control table
pub clock_trim: i8,                                       // RW, signed delta from HSITRIM default
pub clock_step_ppm: u16,                                  // RO, ppm/step metadata (= 2500 on V006)
pub clock_fine_trim_us: i16,                              // RW, signed Q8.8 µs
```

The fire site reads `{PLAIN,FAST}_ENTRY_TICKS + FIRE_ADVANCE_FINE_TICKS` (clamped to ≥ 0) and subtracts the sum from the deadline.

**Why signed for the fine trim.** With biased rounding the value is mathematically always ≥ 0, so unsigned would technically work. But signed (i16 Q8.8 = ±128 µs range) hedges against scenarios we haven't fully nailed down: thermal drift in operation, structural over-estimation on outlier chips, bench experimentation where we want to nudge negative, future re-cal heuristics.

**Why Q8.8.** 1/256 µs ≈ 4 ns granularity, finer than one SysTick at 48 MHz (~21 ns), so we never lose precision in the µs→ticks conversion. ±128 µs in 16 bits is more than enough.

**Why round down (floor) in the µs→ticks conversion.** Safe direction is always "smaller advance ticks" → "fires later" → "small gap rather than collision." For positive Q8.8 values, floor is the same as truncate (smaller magnitude → less advance → safer). For negative Q8.8 values, floor goes more-negative (more retardation → fires later → still safer). Conveniently, that's exactly what arithmetic shift right does on signed integers in Rust:

```rust
// At apply-after-TC time:
let q88 = SHARED.table.config.with(|c| c.comms.clock_fine_trim_us);
let fine_ticks = (q88 as i32 * HCLK_PER_US as i32) >> 8;
FIRE_ADVANCE_FINE_TICKS.store(fine_ticks.clamp(i16::MIN as i32, i16::MAX as i32) as i16,
                              Ordering::Release);
```

Single multiply + arithmetic shift + clamp + atomic store. The fire hot path reads `FIRE_ADVANCE_FINE_TICKS` and the active path's latency atomic directly — no per-fire µs↔tick conversion.

**Characterizing the per-path latency defaults.** `TX_{PLAIN,FAST}_LATENCY_DEFAULT_Q88_US` are one-time per-chip-family characterizations. Procedure:

1. Pick N ≥ 3 chips of the family.
2. For each, run closed-loop CAL with biased rounding → record `drift_ppm` and chosen `step`.
3. Compute the predicted `residual_us` for the operational target (§9).
4. Sweep the candidate latency value in firmware (binary classifier on Fast slot fit, separately for the plain and Fast paths) to find the total optimal advance for that chip.
5. `structural_us_per_chip = total_optimal_us - residual_us`.
6. Take the median across chips, store as the Q8.8 µs default.

The sweep is a firmware-development task when adding a new chip family — not a per-chip production step.

---

## 9. Deployment knob: N_target and b_op

The drift-residual contribution depends on `N_target` (the worst-case predecessor wire length you want clean) and `b_op` (the operational baud). The host picks these once for the deployment; they're not on the wire.

| profile                    | N_target   | b_op       | residual ceiling | over-correction at short predecessors |
| -------------------------- | ---------- | ---------- | ---------------- | ------------------------------------- |
| **conservative (default)** | 128        | 3 Mbaud    | 1.07 µs          | < 1 µs                                |
| tight                      | actual max | actual max | smaller          | smaller                               |

Default conservative: the over-correction at shorter predecessors is well under the jitter cap and harmless. If you know your bus tops out at, say, 64-byte predecessors at 2 Mbaud, you can tighten — but the savings are small and the tuning is more fragile if your "actual max" estimate is wrong.

---

## 10. Boot orchestration

Per-slave loop:

```python
for slave_id in enumerated_slaves:
    ppm_per_step = read_meta_cached(slave_id)               # comms.clock_step_ppm, RO
    current_trim = read(slave_id, comms.clock_trim)         # signed i8

    send CAL(slave_id, count=128) at 1 Mbaud
    T_request_end, T_first, T_last = hse_timer.stamps_for(slave_id)

    # § 7.1 — measurement spans the full Status frame (M = N + 11), not just N zeros.
    drift_ppm = compute_drift_ppm(T_first, T_last, M=128+11, baud=1_000_000, hse_hz)

    # § 7.2 — biased rounding always lands on the slow side (residual ≥ 0).
    step = floor(drift_ppm / ppm_per_step)
    new_trim = clamp(current_trim + step, TRIM_MIN, TRIM_MAX)

    # § 7.3
    residual_ppm = drift_ppm - step * ppm_per_step
    residual_us  = residual_ppm / 1e6 * N_target * byte_time(b_op)
    residual_q88 = round(residual_us * 256)                 # to i16 Q8.8

    if step != 0:
        write(slave_id, comms.clock_trim, new_trim)
    if abs(residual_q88) > Q88_NOOP_THRESHOLD:
        write(slave_id, comms.clock_fine_trim_us, residual_q88)
```

**Why unicast, not Sync.** Sync would cut one wire transaction per slave (master sends one request, slaves respond in slot order) and the firmware extension is small. But Sync slot timing assumes all slaves agree on what one microsecond means, and that assumption holds only *after* trim. Worst case: slave 1 at +1.2% drift, slave 2 at −1.2% drift, slot 2 fires 50 µs into slot 1's tail at 3 Mbaud (~15 byte-times of collision). At 1 Mbaud (the boot CAL baud) it's still ~26 µs of overlap. Either way, slot 2's measurement is corrupted by contention. Sync cal is a chicken-and-egg: needs trimmed chips to work, exists to trim them.

Sync re-cal *after* the chips are trimmed is safe and useful — that's the regime per-shot drift correction lives in, where worst-case drift is bounded by ±one trim step. But boot cal can't assume that. So boot cal is strictly unicast.

**Why pinned to 1 Mbaud.** Bootstrap-safe at any plausible HSI within ±2% (1 Mbaud classic UART tolerates ~3% drift on both sides), and `DXL_BYTE_TIME_TICKS` math is well-understood at this baud. We could cal at 3 Mbaud for shorter wall-clock time, but the marginal speedup isn't worth the precision regime change.

**Per-baud signal table.** Even though boot CAL is pinned to 1 M, the same `CAL` instruction is useful for per-baud bench validation (does my cal hold at the operational baud?). Signal scales with measurement duration:

| baud | byte_time | N=128 wire time | shift per trim step | HSE ticks/step (at 18 MHz) | shots for ≥45 ticks/step |
| ---- | --------- | --------------- | ------------------- | -------------------------- | ------------------------ |
| 1 M  | 10 µs     | 1280 µs         | 3.2 µs              | 58                         | **1**                    |
| 2 M  | 5 µs      | 640 µs          | 1.6 µs              | 29                         | 2                        |
| 3 M  | 3.33 µs   | 427 µs          | 1.06 µs             | 19                         | 3                        |

Boot CAL = single shot at 1 M. Per-baud diagnostic CAL = M shots at the target baud per the table.

**Timing budget.** Per slave at 1 M, rough order of magnitude:

    CAL request out:               ~120 µs
    CAL response in (128 B):       ~1280 µs
    HSE timer drain (CDC RTT):     ~5 ms
    Math + 2 × Write:              ~3 ms
    Optional verify CAL shot:      +~7 ms
    ─────────────────────────────────────
    Total per slave:               ~15–25 ms

For a 16-slave chain: ~350 ms total. Fits inside a 1-second boot budget.

---

## 11. The slave's apply path

Both writes use a pending-atomic apply-after-TC pattern:

1. Host writes the new value to the control table field.
2. Slave acks the Status frame at the *current* settings (so the host's current-baud USART can decode the ack).
3. USART1 TC fires after the Status drains onto the wire.
4. TC handler reads the pending values from the control table, applies to hardware.

For `clock_trim`: write the new value to `RCC_CTLR.HSITRIM`. Single register store, sub-microsecond.

For `clock_fine_trim_us`: convert Q8.8 µs → signed HCLK ticks and publish via atomic Release into `FIRE_ADVANCE_FINE_TICKS`. The fire site sums this with the active path's `TX_{PLAIN,FAST}_LATENCY_TICKS` and clamps to ≥ 0 before subtracting from the deadline — the residual is a single shared atomic, not duplicated per path.

**Why apply-after-TC matters.** Writing HSITRIM shifts HCLK, which shifts USART baud (since the BRR divider was computed against the previous HCLK). If we wrote HSITRIM while the Status reply was still in flight, bytes after the write would shift out at a slightly different baud than bytes before — the master's USART might or might not recover them depending on how big the jump was. By deferring to TC, the change happens on byte boundaries, when no bits are mid-flight. The slave's baud-change apply follows the same pattern for the same reason.

**Side effect on the control loop.** HSITRIM shifts HCLK, which also shifts the SysTick rate the motor control loop is clocked off. Cal lands before torque-on, so there's no in-flight loop to disrupt — but post-cal, the loop interval is now anchored to a more accurate HCLK and the loop's own time-domain math (PID gains in time units, integral accumulation, etc.) is consequently more accurate. The bigger the factory drift the chip arrived with, the bigger the accuracy bonus.

No baud-derived state needs recomputation after either change. `DXL_BYTE_TIME_TICKS = BRR × 10` is in HCLK cycles, and both HCLK and SysTick rates scale with HSI, so the *ratio* (ticks per byte) is invariant. Same for the per-path latency atomics and `FIRE_ADVANCE_FINE_TICKS` — both are tick counts, and tick rate moves with HSI.

---

## 12. Composes with per-shot drift correction

The plan calls for two cooperating mechanisms:

- **CAL (this doc)**: master-driven boot-time trim cal. Brings each chip's baseline drift from ±1% (factory) down to ±0.125% (one trim step), then absorbs that residual into the per-chip fine-trim per §7.3.
- **Per-shot drift correction**: per-shot `fire_tick` drift correction during normal Fast traffic. Observes actual byte cadence on the current predecessor and recomputes the fire deadline. Mops up residual drift on a per-shot basis.

Why both:

- **CAL alone, even with the residual rolled into the intercept compensation, is tuned for one (N_target, b_op) point.** Other operational points have a small over- or under-correction.
- **Per-shot correction alone has no observation window on short predecessors.** A test with 4 wire bytes has ~13 µs of wire time at 3 Mbaud — not enough to derive a robust byte-cadence estimate from. Without CAL's baseline, the chip eats the full ±1% factory drift on short predecessors.
- **Both together**: cal pins the baseline at the chosen operational point; per-shot cleans up everywhere else.

The two are independent — per-shot correction uses master-stamped predecessor cadence (per [dxl-fast-chain-crc.md](dxl-fast-chain-crc.md)); no chip-side RX byte-stamping is required by either consumer.

---

## 13. Protocol spec

### 13.1 Instruction code

One new vendor code in the DXL 2.0 instruction byte space:

    CAL = 0xE0    (pick once, document, never change)

### 13.2 CAL request frame

    [ 0xFF 0xFF 0xFD 0x00 ] [ id ] [ length=5 LE ] [ 0xE0 ]
        [ count_lo ] [ count_hi ] [ CRC ]

`count` is `u16` little-endian, bounded `1 ≤ count ≤ 128`. Master errors out before sending if out of range; slave clamps + errors if it ever sees a violation.

### 13.3 CAL Status reply

Standard Status frame with `count` payload bytes of zero:

    [ 0xFF 0xFF 0xFD 0x00 ] [ id ] [ length LE ] [ 0x55 ] [ err ]
        [ 0x00 ... 0x00 (count bytes) ] [ CRC ]

On `count > 128` or `count == 0`: `err = 0x07` (Data Range), empty payload.

### 13.4 Control table additions

In the `comms` region:

    Name:    clock_step_ppm
    Type:    u16
    Access:  RO
    Init:    2500 on V006   (one HSITRIM step ≈ 60 kHz on 24 MHz HSI)
    Purpose: master reads once per slave model; translates drift_ppm → trim steps

    Name:    clock_trim
    Type:    i8 (signed delta from device's factory-default HSITRIM)
    Access:  RW
    Range:   bounded by the slave's hardware (V006: −16 ≤ x ≤ +15)
    Apply:   pending-atomic; effective at next USART1 TC

    Name:    clock_fine_trim_us
    Type:    i16 (Q8.8 microseconds, signed)
    Access:  RW
    Init:    0
    Range:   ±128 µs (operational envelope: 0–1.1 µs with biased rounding)
    Apply:   pending-atomic; effective at next USART1 TC
    Purpose: per-chip sub-trim drift residual; contributes to fire advance

### 13.5 Master orchestration

    for each slave on the bus:
        send PING                               # enumeration
        ppm_per_step = read(slave_id, comms.clock_step_ppm)
                       # cache by model_number across slaves

    for each slave:
        current_trim = read(slave_id, comms.clock_trim)
        send CAL(slave_id, count=128) at 1 Mbaud
        (T_request_end, T_first, T_last) = hse_timer.stamps_for(slave_id)
        compute drift_ppm, step, new_trim, residual_q88   (per §7)
        if step != 0:
            send Write(comms.clock_trim, new_trim)
        if abs(residual_q88) > Q88_NOOP_THRESHOLD:
            send Write(comms.clock_fine_trim_us, residual_q88)
        await Status ACKs

    # all slaves trimmed; safe to enable Fast Sync/Bulk Read
    switch bus to operational baud if desired
    begin normal operation

---

## 14. Why not the alternatives

Other shapes for the design that don't quite work.

**Chip-side measurement (slave times itself).** Discussed in §5. Every term in a chip-side timing measurement is HSI-quantized, so the "ground truth" comparison `(observed − nominal)` can't cleanly separate drift from structural offset (ISR latency, IDLE assertion timing). Compensation requires hardcoded constants that don't generalize across chip revs or baud rates. Master-side measurement sidesteps the whole problem.

**Slot-fit binary classifier (brute-force sweep).** Master writes a candidate trim, runs a Fast BulkRead with a known co-slave in slot 0 and the DUT in slot 1, observes whether the slots coalesce cleanly. Sweeps trim values, picks the highest "clean" result. Workable as a bench tool but:

- Brute-force binary search, not closed-form one-shot.
- Needs a co-slave with HSE on the bus to provide the slot 0 timing reference. Production usually has just master + DUTs.
- Requires the Fast slot machinery to already be working on the DUT (chicken-and-egg with first-boot cal of new firmware).

Useful as a bench *health check* and as the characterization sweep for the per-path latency defaults (§8). Not the cal method itself.

**Sync CAL.** Reusing SyncRead slot infrastructure would make the firmware diff tiny. But slot timing relies on all slaves agreeing on µs, which only holds *after* trim. With untrimmed ±1.2% drift, worst-case adjacent slots collide by ~50 µs at 3 Mbaud (~26 µs at 1 Mbaud). Chicken-and-egg. Sync cal becomes safe and useful for periodic *re*-cal of already-trimmed chips, but not for first-boot.

**Broadcast CAL (ID 0xFE).** DXL broadcast forbids responses, which leaves the HSE timer with nothing to stamp. Useless for measurement.

**Bumping `DXL_TX_BUF_LEN` past 256 to support large N.** Would enable single-shot CAL at every baud (no multi-shot averaging needed at 3 M). Capping N at 128 instead keeps the buffer at 256, leaves margin for other slave implementations, and the 3-shot averaging at 3 Mbaud is cheap. Net simpler.

**MINC=0 DMA streaming for arbitrarily large N.** Hardware supports it (V006 RM §8.3.3 bit 7), HAL exposes it. Would let the slave emit N=600 with a 1-byte source. But N=128 covers needs at every CH32V006-reachable baud, and the multi-phase DMA reconfigure adds ~30% of a char-time of inter-phase wire gap at 3 Mbaud. Deferred until a chip past 3 Mbaud needs it.

**Single intercept value covering everything (no structural/residual split).** A single tunable intercept mixes the per-family structural latency with the per-chip drift residual; closed-form per-chip cal can't decompose them after the fact. Splitting into per-family path defaults plus a tunable `clock_fine_trim_us` makes the residual derivable from the CAL drift measurement in closed form. One characterization sweep on the family upfront, no per-chip brute force at boot.

**Two separate CAL instructions (one for drift, one for structural).** A "structural CAL" would have to measure the slave's reply latency from request-end → first-byte-on-wire, which goes through the *unicast reply path* — a different ISR/handler chain than the *Fast slot fire path* that the intercept compensation actually compensates. The measured structural delay would systematically mismatch. The design instead hardcodes the structural value per family/path (where it belongs — it's a chip-physics constant) and derives only the per-chip residual from the drift measurement. Single CAL stays sufficient.

**Auto-tuning the structural compensation at boot.** Possible by running a Fast-slot probe after CAL lands the drift, but requires Fast slot infrastructure working before CAL completes (chicken-and-egg), and the structural component is HCLK-cycle-deterministic with negligible chip-to-chip variation. One-time bench characterization is cheaper and more robust.

**Normal `round()` instead of biased `floor()` for the trim step.** Lands chips on either side of optimal. Half end up "fast" and need fire to be *retarded*, which the intercept compensation can't do — it only advances. Biased rounding always puts the chip on the slow side, where the intercept can take up the slack. Cost: wastes one trim step of headroom, well within the ±16 range V006 provides.

---

## 15. Honest accounting

What this design buys:

- **Per-chip drift correction *and* per-chip fire residual compensation from a single CAL round trip.** No second wire op, no second round of math.
- **No factory cal step required.** Slaves work out of the box; the runtime protocol handles characterization on first boot. Stored cal values are warm-start seeds, not a hard prerequisite.
- **Tiny slave firmware footprint.** A small CAL handler in the dispatcher plus the apply-after-TC plumbing already used by every other RW field. No chip-side cal state machine.
- **Environmentally current.** Re-cal at every boot tracks the chip's actual operating temperature, supply voltage, and aging.
- **Baud independent.** Cal once at 1 Mbaud, valid at all bauds. Master can change the bus baud after cal without re-cal.
- **Control loop accuracy bonus.** Post-cal HCLK is closer to nominal, so the motor control loop (clocked off SysTick) runs on more accurate time-domain math too — PID gain units, integral accumulation, etc. all benefit.
- **Composes with per-shot drift correction.** Cal pins the baseline at the chosen operational point; per-shot cleans up residual elsewhere.
- **Portable wire contract.** CAL, `clock_trim`, `clock_step_ppm`, and `clock_fine_trim_us` don't bake any V006-specific assumptions into the wire. Same protocol works for any future chip with a similar trim register and a published ppm/step ratio.

What this doesn't buy:

- **The per-path latency defaults still have to be characterized once per chip family.** Per §8 procedure. Cost lands on the firmware developer adding a new chip family, not on every boot.
- **Mid-operation re-cal is not supported in this revision.** Cal lands before torque-on; thermal re-cal during a torqued run would need either a torque-off window or a RAM-only "live trim" write variant.
- **Requires an HSE-accurate timer on the bus.** On the bench that's the pirate; in production it's the MCU master itself. A bare DUT plugged into a bus with a host-PC-only master (no MCU intermediary, no HSE-clocked stamping) can't be cal'd by this scheme. For OpenServoCore's deployment model this is fine — production deployments always have an MCU master — but a stock DXL host SDK won't transparently calibrate these chips.

What this trades off:

- **~15–25 ms per slave at boot, ~350 ms for a 16-slave arm.** Bounded, predictable, master-orchestrated.
- **One vendor instruction code** in DXL's instruction space.
- **Two new RW control-table fields** (`clock_trim` — i8, `clock_fine_trim_us` — i16) **plus one RO metadata field** (`clock_step_ppm` — u16). 5 bytes total.
- **Two compile-time per-path defaults** (`TX_{PLAIN,FAST}_LATENCY_DEFAULT_Q88_US`), characterized once on the bench.
- **A small amount of host-side machinery**: HSE-timer query, drift + residual math, per-slave loop.

---

## 16. One-paragraph summary

> The V006's HSI carries ±1% process tolerance, which translates to ~14 µs of wire-side timing drift on a 128-byte predecessor at 1 Mbaud — enough to break Fast Sync/Bulk Read coalesce by either opening a visible gap or running into the predecessor's last byte. We fix this with three orthogonal compensations for three orthogonal phenomena: HSITRIM (multiplicative drift correction, exposed via `comms.clock_trim`); two compile-time per-chip-family path defaults for the structural ISR+DMA latency (`TX_{PLAIN,FAST}_LATENCY_DEFAULT_Q88_US`, expressed in HCLK ticks at boot); and a per-chip RW sub-trim drift residual (`comms.clock_fine_trim_us`, signed Q8.8 µs). The slave can't reliably measure its own clock — measuring HSI with HSI is structurally biased — so the master sends `CAL(N=128)`, slave responds with N zero bytes through its standard reply path, and an HSE-accurate timer on the bus (pirate on the bench, MCU master in deployment) stamps the slave's first and last response bytes. From a single CAL the master derives drift via `(T_last − T_first)` vs nominal, picks a trim step using *biased rounding* (always toward "slow") so the intercept compensation can always advance fire to compensate, and computes the residual µs from the rounding remainder times the deployment's worst-case predecessor wire length. Both runtime values are written back via standard DXL `Write`; the slave applies them in its USART1 TC handler after the Status ACK drains. One round trip, no slave-side math, no chip-side cal state machine. The cal fields live in the EEPROM section of the control table and persist across power cycles, but the master re-runs CAL at every boot anyway because HSI is temperature-sensitive. ~15–25 ms per slave at 1 Mbaud, ~350 ms for a 16-slave arm. Composes with per-shot drift correction for residual cleanup elsewhere on the operational envelope. Stable 3 Mbaud Fast operation falls out comfortably within the 3.33 µs jitter cap.

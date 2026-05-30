# Runtime HSI Calibration and Fire-Floor Tuning for DXL Fast on the CH32V006

Companion to [dxl-rx-timing.md](dxl-rx-timing.md) and [dxl-fast-chain-crc.md](dxl-fast-chain-crc.md). Read those first — they cover what "wire-end timestamp," "Fast last-slave coalesce," and "the V006's SysTick CMP fire path" mean. This doc covers what to do when the chip's internal clock isn't accurate enough for those mechanisms to work without help.

## TL;DR

The V006 has no crystal — its 24 MHz HSI oscillator drifts ±1% from chip to chip and with temperature. For most embedded work that's fine. For DXL Fast mode at 3 Mbaud, where slaves stitch their replies onto the bus with less than ~3.3 µs of slop, ±1% drift turns into 15+ µs of wire-side error on a 128-byte predecessor — guaranteed bus collision or visible gap, either way the coalesce breaks.

The chip exposes a 5-bit `HSITRIM` field that nudges the oscillator in ~0.25% steps, which gets us close. But "close" is still ±0.125% on a good day, and at 3 Mbaud over 128 bytes that residual is ~0.5 µs — close to the cliff. We also have to compensate for the chip's own ISR + DMA latency between "SysTick deadline reached" and "first bit on wire," which is another few µs of *additive* slop. So we expose a second knob — `FIRE_FLOOR` — that handles both the structural latency and the leftover trim-step residual.

The slave can't reliably measure its own clock. Measuring HSI with HSI is structurally biased (an earlier prototype landed 9 trim steps off optimum and we couldn't derive a clean fix). So instead, the master sends `CAL(N)`, the slave responds with N zero bytes, and a crystal-accurate timer on the bus stamps the slave's response. From that single round trip the master derives *both* knobs: drift from the response duration, and the per-chip fire residual from the rounding remainder when discrete trim steps don't perfectly hit nominal. Master writes both back via standard `Write`. Slave applies on its next idle. No flash, no factory cal, re-derived every boot — which conveniently tracks temperature for free.

---

## 1. Why HSI drift matters for DXL Fast

The whole point of Fast Sync/Bulk Read (see [dxl-fast-chain-crc.md](dxl-fast-chain-crc.md)) is that all the addressed slaves' replies stitch into one coalesced Status frame on the wire with zero idle gap between slots. The host's USART recovers it as one big packet. The jitter cap for inter-slave hand-off is one byte time — that's 3.33 µs at 3 Mbaud. Land later than that and the host's USART asserts IDLE between slots, framing breaks, the whole coalesced frame is lost.

The last slave's reply has to start at exactly:

    fire_us = RDT + bytes_before × byte_time

where `bytes_before` is the predecessors' total wire bytes already on the bus. That's `fire_us` measured in *real* microseconds of wall-clock time.

But the slave doesn't *have* wall-clock time — it has SysTick, which counts HCLK cycles. SysTick rate = HCLK = HSI × PLL_mult. If HSI is off by ε, SysTick is off by ε, and so is the slave's belief about how long `bytes_before × byte_time` is.

If HSI is **slow** by 1%: SysTick under-counts, the slave thinks the deadline is at `fire_us` but in real time it lands at `fire_us × 1.01`. That's a positive gap — the slave fires *after* the predecessor's wire-end. Visible idle, possible IDLE assertion on the host, coalesce broken.

If HSI is **fast** by 1%: the slave fires `fire_us × 0.99` in real time. Negative gap — overlap with the predecessor's last byte. Wire collision, CRC corruption, frame lost.

**Concrete bench numbers (rev_b, HSITRIM at factory default 16):**

    1 Mbaud, 128-byte predecessor:
        predecessor wire time = 138 × 10 µs = 1380 µs
        observed gap          = 16 µs
        implied drift         = 16 / 1280 ≈ 1.25% slow
    3 Mbaud, basic 4-byte predecessor:
        predecessor wire time = 47 µs
        observed gap          = ~6 µs (mostly structural floor, ~0.6 µs drift)

So this particular chip's HSI runs about 1.25% slow at room temperature, and the drift scales linearly with predecessor wire time (as it must — the longer the wire window, the more drift accumulates).

For Fast to work cleanly across all bauds and predecessor sizes, the drift has to come out. The chip's own SysTick is the only on-chip time reference, so we have to fix HSI itself — or, as we'll see in §5, bring in a clock from outside the chip that *isn't* HSI to measure against.

---

## 2. The HSITRIM lever, and why it's only half the story

V006 RM §6 documents a 5-bit `HSITRIM` field in `RCC_CTLR[7:3]`, RW, default 16 (binary `10000b`). Each step shifts HSI by ~60 kHz at the nominal 24 MHz — about **0.25% per step** (2500 ppm/step). 5 bits give ±16 steps of range; the manual quotes ±1% as guaranteed, but the silicon takes more in practice. The factory `HSICAL[7:0]` byte (bits [15:8] of the same register, read-only) is the per-chip calibration baseline; `HSITRIM` is the *adjustment* superimposed on it.

The good news:

- A direct knob for correcting per-part HSI offset.
- Runtime-writable in a single register store — sub-microsecond, no flash controller, no CPU stall. Completely safe to do at any time, even mid-motor-loop.
- Affects HSI itself → HCLK, USART BRR, and SysTick all scale uniformly → **baud-independent**. Cal at 1 Mbaud, switch the bus to 3 Mbaud for operation, the trim applied at 1M is still optimal at 3M.

The catch:

- **0.25% steps leave ±0.125% residual** even with a perfect cal landing. At 3 Mbaud with a 128-byte predecessor that's ±0.53 µs of slop — uncomfortably close to the 3.33 µs jitter cap.
- The trim only fires whole steps, so if your chip's true optimal is "between steps 4 and 5" (HSITRIM=20.5, which doesn't exist), you have to pick one. Pick the wrong side and you get bus collisions even though the cal "worked." We hit this on the bench: HSITRIM=21 sometimes shows contention even with no extra fire-floor compensation, while HSITRIM=20 + 3 µs of fire floor lands clean. The chip wanted to be between trim steps, and we needed a finer-grained knob to express that.

**Direction**: higher HSITRIM = faster HSI. So a slow chip (positive observed gap) needs HSITRIM *raised*. The rev_b chip we calibrated landed at HSITRIM=20 (default 16 + 4 steps = +1.0%), within one step of the 1.25% drift estimate.

That second knob — the finer-grained one — is `FIRE_FLOOR`. The next section explains why it's already there for an unrelated reason, and how it conveniently also handles the trim-step gap.

---

## 3. The two knobs are orthogonal (this is the key insight)

If you write the wire-side gap between a predecessor's last byte and a slave's reply as a linear function of how many bytes the predecessor had, it decomposes cleanly into two terms:

    gap_us = drift × bytes_before × byte_time   +   structural_latency
             └─────────── slope ───────────┘       └──── intercept ───┘

The **slope** (drift) is *multiplicative* — its effect scales with how much wire time the predecessor occupies. A 1% slow chip fires 1% late, which on a 1 ms predecessor is 10 µs late, on a 100 µs predecessor is 1 µs late. HSITRIM controls this.

The **intercept** (structural latency) is *additive* and constant. It's the chip's own time from "SysTick deadline reached" to "first bit on wire": PFIC trap entry + ISR body + DMA prefetch + start-bit latch. Every one of those is a deterministic number of HCLK cycles, so the whole intercept is a fixed µs value per chip family — same whether the predecessor was 4 bytes or 4000. `FIRE_FLOOR` controls this.

Two contributions, two knobs. They don't interfere with each other, they don't need separate measurements, they don't even need to be tuned in any particular order. But you need *both*: trim alone leaves ±0.125% residual plus the whole structural latency (maybe 4 µs at 3 Mbaud — over the jitter cap). FLOOR alone has no way to compensate the slope. Together they land well under the cap.

### Bonus role: FLOOR as sub-trim-step nudge

Here's a happy accident of the additive math. When biased rounding (§7.2) puts the chip on the "slow" side of optimal — say the chip wants HSITRIM=20.5 but we picked 20 — there's leftover slope drift after trim. That residual is a small µs-scale gap that scales with predecessor length. Since FLOOR is additive and works in µs, we can absorb the residual into it: pick a worst-case predecessor length, compute the residual µs, roll it into FLOOR. Suddenly the trim-step granularity stops being a problem.

So FLOOR plays two roles: compensate the structural latency (chip family constant) and absorb the per-chip drift residual (per-chip variable). The math sums them; the firmware applies the sum. We'll split the two roles into two storage locations in §8 to keep the lifecycles clean — one is hardcoded, one is tunable — but as far as the fire path is concerned they're one number.

---

## 4. Why we don't store any of this in flash

The natural instinct is to cal once at production, store the result somewhere persistent, and load it at boot. Both main flash and the V006's option bytes (Data0/Data1 at `0x1FFFF804`) sit behind the same FLASH controller. The OB write sequence is slightly different from main flash (different unlock keys, different erase granularity), but the underlying hardware behavior is identical:

- Erase a flash page: ~ms-scale, CPU stalled
- Program a flash word: ~tens-of-µs, CPU stalled
- Total for one cal commit: ~5–10 ms of pure CPU stall

For a slave that's just sitting on the bench, that's fine. For a slave that's currently running a motor control loop at 20 kHz, it's catastrophic. The PWM duty register stays at whatever value it was holding when the stall started; the position-loop feedback stops updating; the motor either holds at the wrong torque or runs away depending on what the loop was about to do. Either way the robot just dropped a metaphorical wrench.

The DXL 2.0 protocol convention captures this constraint as "no EEPROM writes while torque is on" — it's the same physical concern lifted into the protocol layer. We could honor that convention and defer cal commits to torque-off windows. But this is OpenServoCore, intended for robots that operate continuously. A robot in operation is essentially always torqued on. The torque-off window may never come.

So the workaround paths don't work:

- "Just commit during torque-off" — never happens during operation.
- "Use OB instead" — same flash controller, same stall.
- "Special-case `clock_trim` and `fire_floor_drift_residual_us` in the torque-lock policy" — bends a clean protocol invariant to plaster over a hardware limit.

The sidestep is cleaner than any workaround: **don't store the cal values anywhere persistent. Recompute them on every boot.**

This sounds wasteful at first ("but we're throwing away the cal data on every reset!"), but actually solves several problems at once:

- **No flash wear.** Cal values are runtime state. Update them as often as you like.
- **No factory step.** Slaves don't need to be pre-flashed with cal data — the runtime protocol handles it.
- **Environmentally current.** HSI has a temperature coefficient (~0.05%/°C is typical for these RC oscillators). A factory cal done at room temperature isn't right for a chip running hot in a robot. Recalibrating at boot tracks whatever conditions the chip is actually starting up in. If thermal drift becomes significant mid-operation, the master can re-issue cal at an idle moment without restart.
- **Dev workflow.** During firmware development we re-flash constantly. Factory cal stored in flash would get wiped with every mass erase. With runtime cal, the firmware build doesn't care.

The cost is the boot-time latency of the cal handshake — bounded and small (§10).

---

## 5. Architecture: master measures, slave just responds

The chip can't measure itself reliably. SysTick is HSI-derived. USART IDLE assertion is HCLK-gated. ISR entry latency is HCLK-quantized. Every term in a chip-side timing measurement is itself a function of the unknown we're trying to characterize — there's no Archimedean lever to push against from inside the slave.

An earlier prototype tried anyway: timestamp the first and last received bytes inside the USART1 RXNE/IDLE handlers, compute `span = LAST − FIRST` in slave ticks, compare to `(N − 1) × nominal_byte_time`, derive drift_ppm, apply trim. It produced repeatable results (good precision) but landed ~9 trim steps off the scope-validated optimum (bad accuracy). The compensation we needed — a hardcoded 97-tick subtraction — broke into ~50 ticks of "USART IDLE actually waits 10 bit-times not 9" and ~50 ticks of "residual asymmetric ISR-entry latency the top-of-ISR tick capture didn't catch," but neither half was clean enough to call derived. We accepted the empirical constant for one rig and moved on, but it wouldn't generalize.

That's the wrong place to spend complexity. The fix is to put the measurement on a clock domain that *isn't* the one under test. Three actors:

- **Slave**: generates a known-shape stimulus (N zero bytes at the bus baud) on the `CAL` command. No timing decisions, no math, no state machine.
- **HSE-accurate timer on the bus**: stamps the slave's first and last response bytes (and the master's last request byte) in HSE-derived ticks.
  - *On the bench*: the V203 injector, already validated for chain-CRC timing work, sits passive on the bus and reports timestamps via its CDC channel.
  - *In production*: the MCU master itself (V20x/V30x), using USART RX input-capture clocked from its own HSE crystal. The DUT MCU never sees this timer.
- **Host**: queries the timer for stamps, does all the math, writes results back via standard DXL `Write`.

The wire op is identical bench and production — only the timing source swaps. On the bench, INJ stamps are queried via CDC; in production, the master reads its own timer's CCR after each byte. Both surface the same `(T_request_end, T_first_slave, T_last_slave)` tuple per slave response, in HSE ticks.

**Why this escapes the chip-side bias.** The whole 97-tick problem was that `observed` and `nominal` were anchored at different absolute points in the chip's ISR latency profile, and ISR latency couldn't be derived from chip-side data. When the reference clock is external (HSE), the host measures the same physical wire events with the same hardware on every byte. Any systematic offset in the host's stamping is in the host's budget, not divided through HSI's drift, and it cancels in `T_last − T_first` rather than masquerading as drift.

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

**Why one instruction.** The slave doesn't surface any state — the master has all the timing it needs. The "apply" step is just a normal `Write(comms.clock_trim, ...)` and `Write(comms.fire_floor_drift_residual_us, ...)` against the existing apply-after-TC machinery. CAL is a pure stimulus instruction.

---

## 7. The math: one CAL transaction → both knobs

From a single CAL round trip the master gets three HSE-stamped events:

    T_request_end:  master's last bit hit the wire
    T_first_slave:  slave's first reply bit hit the wire
    T_last_slave:   slave's last reply bit hit the wire

Plus it already knows: `N` (count it requested), `baud_hz`, `RDT` (slave's reply delay, typically 0), and `hse_hz` (its own timer's tick rate).

### 7.1 Deriving drift

The slave's TX clock is its own HSI (via the BRR divider). The time it actually takes to emit (N − 1) byte intervals at the bus baud is what the HSE timer measures directly:

    observed_us = (T_last_slave - T_first_slave) / hse_hz * 1_000_000
    nominal_us  = (N - 1) * 10 / baud_hz * 1_000_000      // 10 bits per UART byte (8N1)
    drift_ppm   = (observed_us - nominal_us) / nominal_us * 1_000_000

Sign reading:

- `observed > nominal` → slave took *longer* than nominal → slave's HSI is **slow** → raise HSITRIM.
- `observed < nominal` → slave's HSI is **fast** → lower HSITRIM.

So the natural sign of `drift_ppm` is the *negation* of what we want to add to the trim register. One flip:

    step_exact = -drift_ppm / ppm_per_step                 // real-valued, will be rounded

### 7.2 Biased rounding (so FLOOR can always do its job)

Normal `round()` would land us on either side of optimal — half the chips end up fast, half slow. But FLOOR only *advances* fire (subtracts ticks from the deadline); it can't push fire *later*. A chip on the fast side has no way to be corrected by FLOOR alone, and we end up in the HSITRIM=21 collision scenario from bench experience.

The fix is to use `floor()` (round toward −∞), which guarantees we always land on the slow side:

    step         = floor(-drift_ppm / ppm_per_step)
    residual_ppm = drift_ppm + step * ppm_per_step         // always in [-ppm_per_step, 0]

**Worked example 1** — a chip that's 1.25% slow at factory (close to our rev_b sample):

    drift_ppm = -12500
    floor(12500 / 2500) = 5  →  step = +5
    HSITRIM = 16 + 5 = 21
    residual_ppm = -12500 + 5 * 2500 = 0          ← lucky landing exactly on a step

**Worked example 2** — a chip that's a bit less slow, say 1.125%:

    drift_ppm = -11250
    floor(11250 / 2500) = 4  →  step = +4
    HSITRIM = 16 + 4 = 20
    residual_ppm = -11250 + 4 * 2500 = -1250      ← still 0.125% slow after trim

That residual is what FLOOR has to take up.

**Cost of biased rounding**: worst case wastes one trim step of headroom (chip lands at -1 step from optimal instead of ±half-step). V006 has ±16 steps; factory variation needs ~4. Plenty of margin.

### 7.3 Deriving the FIRE_FLOOR residual

For a deployment target baud `b_op` and worst-case predecessor wire length `N_target` (defaults: 3 Mbaud, 128 bytes — see §9), the µs gap left by the residual drift is:

    residual_us = -residual_ppm / 1_000_000 * N_target * byte_time(b_op)

`residual_ppm` is always ≤ 0 (chip slow after trim), so `residual_us` is always ≥ 0. That's how much *extra* the FLOOR has to advance fire to compensate. It rolls in on top of the structural floor:

    total_floor_us = structural_floor_us + residual_us

Where `structural_floor_us` is hardcoded per chip family (§8). The master writes the *residual* part to the slave's control table; the slave's firmware sums it with its compile-time structural constant at apply time.

### 7.4 What the master writes back

Two writes:

    Write(slave_id, comms.clock_trim, current_trim + step)               // i8
    Write(slave_id, comms.fire_floor_drift_residual_us, residual_us_q88) // i16, Q8.8 µs

Both land in pending atomics; both apply in the next USART1 TC interrupt (same mechanism the existing baud-change path uses).

---

## 8. Splitting FIRE_FLOOR into compile-time + tunable

The FIRE_FLOOR knob mixes two things with different lifecycles:

1. **Structural part**: the chip's PFIC + ISR + DMA + start-bit latency. Same for every instance of a given chip family because it's deterministic HCLK cycles. Knowable once, hardcoded forever (per family/board).
2. **Drift residual part**: the leftover slope from biased-rounded HSITRIM. Per-chip, recomputed at every boot, varies up to one trim step worth of µs.

These don't share a unit naturally. Structural is best expressed in HCLK ticks — it *is* a cycle count, so storing it in ticks avoids any µs↔tick conversion in the fire hot path. Drift residual is computed by the master in physical µs (the master doesn't know the slave's HCLK rate) and needs unit conversion at apply time.

So we split:

```rust
// firmware/ch32/src/board/<board>.rs — compile-time, per board/chip
pub const FIRE_STRUCTURAL_FLOOR_TICKS: u16 = 144;  // ~3 µs at 48 MHz (TBD, see below)

// firmware/lib/core control table — RW, signed Q8.8 microseconds
pub fire_floor_drift_residual_us: i16,             // Q8.8: 1/256 µs granularity
```

**Why signed.** With biased rounding the value is mathematically always ≥ 0, so unsigned would technically work. But signed (i16 Q8.8 = ±128 µs range) hedges against scenarios we haven't fully nailed down: thermal drift in operation, structural over-estimation on outlier chips, bench experimentation where we want to nudge negative, future re-cal heuristics. Costs us nothing in range (operational envelope is 0–1.1 µs).

**Why Q8.8.** 1/256 µs ≈ 4 ns granularity, finer than one SysTick at 48 MHz (~21 ns), so we never lose precision in the µs→ticks conversion. ±128 µs in 16 bits is more than enough.

**Why round down (floor) in the µs→ticks conversion.** Safe direction is always "smaller FLOOR ticks" → "fires later" → "small gap rather than collision." For positive Q8.8 values, floor is the same as truncate (smaller magnitude → less advance → safer). For negative Q8.8 values, floor goes more-negative (more retardation → fires later → still safer). Conveniently, that's exactly what arithmetic shift right does on signed integers in Rust:

```rust
// At apply-after-TC time:
let q88 = SHARED.table.config.with(|c| c.comms.fire_floor_drift_residual_us);
let residual_ticks = (q88 as i32 * HCLK_PER_US as i32) >> 8;
let total = FIRE_STRUCTURAL_FLOOR_TICKS as i32 + residual_ticks;
FIRE_FLOOR_TICKS.store(total.clamp(0, u16::MAX as i32) as u16, Ordering::Release);
```

Single multiply + arithmetic shift + clamp + atomic store. The fire hot path reads `FIRE_FLOOR_TICKS` directly — no per-fire µs↔tick conversion.

The `clamp(0, …)` guards against pathological combinations (e.g., a large negative residual eating all of structural). Underflow below 0 would mean firing *after* the nominal deadline, which is never useful — safe-fail to "no compensation at all."

**Characterizing the structural constant.** We don't actually know `FIRE_STRUCTURAL_FLOOR_TICKS` for V006 yet. The empirical 3 µs we got from bench tuning contains *both* the structural part and the residual part for whatever chip we happened to be testing. To separate them:

1. Pick N ≥ 3 V006 chips.
2. For each, run closed-loop CAL with biased rounding → record `drift_ppm` and chosen `step`.
3. Compute the predicted `residual_us` for your operational target.
4. Sweep `fire_floor_drift_residual_us` in firmware (binary classifier on Fast slot fit, à la `tune_hsitrim.py`) to find the total optimal FLOOR for that chip.
5. `structural_us_per_chip = total_optimal_us - residual_us`.
6. Take the median across chips, convert to ticks at HCLK=48 MHz, hardcode the result.

Expected landing: ~100–130 ticks (~2.1–2.7 µs) given the empirical 3 µs total. The bench sweep is a one-time firmware-development task — not a per-chip production step.

---

## 9. Deployment knob: N_target and b_op

The drift-residual contribution depends on `N_target` (the worst-case predecessor wire length you want clean) and `b_op` (the operational baud). The host picks these once for the deployment; they're not on the wire.

| profile | N_target | b_op | residual ceiling | over-correction at short predecessors |
|---|---|---|---|---|
| **conservative (default)** | 128 | 3 Mbaud | 1.07 µs | < 1 µs |
| tight | actual max | actual max | smaller | smaller |

Default conservative: the over-correction at shorter predecessors is well under the jitter cap and harmless. If you know your bus tops out at, say, 64-byte predecessors at 2 Mbaud, you can tighten — but the savings are small and the tuning is more fragile if your "actual max" estimate is wrong.

---

## 10. Boot orchestration

Per-slave loop, in pseudo-Python:

```python
for slave_id in enumerated_slaves:
    ppm_per_step = read_meta_cached(slave_id)               # comms.clock_trim_ppm_per_step, RO
    current_trim = read(slave_id, comms.clock_trim)         # signed i8

    send CAL(slave_id, count=128) at 1 Mbaud
    T_request_end, T_first, T_last = hse_timer.stamps_for(slave_id)

    # § 7.1
    drift_ppm = compute_drift_ppm(T_first, T_last, N=128, baud=1_000_000, hse_hz)

    # § 7.2
    step = floor(-drift_ppm / ppm_per_step)
    new_trim = clamp(current_trim + step, TRIM_MIN, TRIM_MAX)

    # § 7.3
    residual_ppm = drift_ppm + step * ppm_per_step
    residual_us  = -residual_ppm / 1e6 * N_target * byte_time(b_op)
    residual_q88 = round(residual_us * 256)                 # to i16 Q8.8

    if step != 0:
        write(slave_id, comms.clock_trim, new_trim)
    if abs(residual_q88) > Q88_NOOP_THRESHOLD:
        write(slave_id, comms.fire_floor_drift_residual_us, residual_q88)
```

**Why unicast, not Sync.** Sync would cut one wire transaction per slave (master sends one request, slaves respond in slot order) and the firmware extension is small. But Sync slot timing assumes all slaves agree on what one microsecond means, and that assumption holds only *after* trim. Worst case: slave 1 at +1.2% drift, slave 2 at −1.2% drift, slot 2 fires 50 µs into slot 1's tail at 3 Mbaud (~15 byte-times of collision). At 1 Mbaud (the boot CAL baud) it's still ~26 µs of overlap. Either way, slot 2's measurement is corrupted by contention. Sync cal is a chicken-and-egg: needs trimmed chips to work, exists to trim them.

Sync re-cal *after* the chips are trimmed is safe and useful — that's the regime A8.2 lives in, where worst-case drift is bounded by ±one trim step. But boot cal can't assume that. So boot cal is strictly unicast.

**Why pinned to 1 Mbaud.** Bootstrap-safe at any plausible HSI within ±2% (1 Mbaud classic UART tolerates ~3% drift on both sides), and `DXL_BYTE_TIME_TICKS` math is well-understood at this baud. We could cal at 3 Mbaud for shorter wall-clock time, but the marginal speedup isn't worth the precision regime change.

**Per-baud signal table.** Even though boot CAL is pinned to 1 M, the same `CAL` instruction is useful for per-baud bench validation (does my cal hold at the operational baud?). Signal scales with measurement duration:

| baud | byte_time | N=128 wire time | shift per trim step | INJ HSE ticks/step (at 18 MHz) | shots for ≥45 ticks/step |
|---|---|---|---|---|---|
| 1 M | 10 µs | 1280 µs | 3.2 µs | 58 | **1** |
| 2 M | 5 µs | 640 µs | 1.6 µs | 29 | 2 |
| 3 M | 3.33 µs | 427 µs | 1.06 µs | 19 | 3 |

Boot CAL = single shot at 1 M. Per-baud diagnostic CAL = M shots at the target baud per the table.

**Timing budget.** Per slave at 1 M:

    CAL request out:               ~120 µs
    CAL response in (128 B):       ~1280 µs
    HSE timer drain (CDC RTT):     ~5 ms
    Math + 2 × Write:              ~3 ms
    Optional verify CAL shot:      +~7 ms
    ─────────────────────────────────────
    Total per slave:               ~15–25 ms

For a 16-slave chain: ~350 ms total. Comfortable inside a 1-second boot budget.

---

## 11. The slave's apply path

Both writes use the same pending-atomic pattern as the existing `BAUD_RATE_IDX` apply path:

1. Host writes the new value to the control table field.
2. Slave acks the Status frame at the *current* settings (so the host's current-baud USART can decode the ack).
3. USART1 TC fires after the Status drains onto the wire.
4. TC handler reads the pending values from the control table, applies to hardware.

For `clock_trim`: write the new value to `RCC_CTLR.HSITRIM`. Single register store, sub-microsecond.

For `fire_floor_drift_residual_us`: recompute `FIRE_FLOOR_TICKS` per §8 and store via atomic Release. Same single-multiply-shift + atomic store.

**Why apply-after-TC matters.** Writing HSITRIM shifts HCLK, which shifts USART baud (since the BRR divider was computed against the previous HCLK). If we wrote HSITRIM while the Status reply was still in flight, bytes after the write would shift out at a slightly different baud than bytes before — the master's USART might or might not recover them depending on how big the jump was. By deferring to TC, we ensure HSITRIM only changes on byte boundaries, when no bits are mid-flight.

No baud-derived state needs recomputation after either change. `DXL_BYTE_TIME_TICKS = BRR × 10` is in HCLK cycles, and both HCLK and SysTick rates scale with HSI, so the *ratio* (ticks per byte) is invariant. Same for `FIRE_FLOOR_TICKS` — it's a tick count, and tick rate moves with HSI. The `store_baud_derived` function doesn't need to run.

---

## 12. Composes with live drift correction (A8.2)

The plan calls for two cooperating mechanisms:

- **CAL (this doc)**: master-driven boot-time trim cal. Brings each chip's baseline drift from ±1% (factory) down to ±0.125% (one trim step), then absorbs that residual into FLOOR per §7.3.
- **A8.2 live drift correction**: per-shot `fire_tick` drift correction during normal Fast traffic. Observes actual byte cadence on the current predecessor and recomputes the fire deadline. Mops up residual drift on a per-shot basis.

Why both:

- **CAL alone, even with FLOOR rolling in the residual, is tuned for one (N_target, b_op) point.** Other operational points have a small over- or under-correction.
- **A8.2 alone has no observation window on short predecessors.** A FastBasic test with 4 wire bytes has ~13 µs of wire time at 3 Mbaud — not enough to derive a robust byte-cadence estimate from. Without CAL's baseline, the chip eats the full ±1% factory drift on short predecessors.
- **CAL + A8.2 together**: cal pins the baseline at the chosen operational point; A8.2 cleans up everywhere else on a per-shot basis.

The two are independent in this revision — A8.2 uses master-stamped predecessor cadence (per [dxl-fast-chain-crc.md](dxl-fast-chain-crc.md)), not the chip-side RX byte-stamping primitive earlier iterations of this doc invoked. The RX byte-stamping primitive on the slave is no longer needed by either consumer.

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

Two new fields in the EEPROM section of `comms`:

    Name:    clock_trim_ppm_per_step
    Type:    u16
    Access:  RO
    Init:    2500 on V006   (one HSITRIM step ≈ 60 kHz on 24 MHz HSI)
    Purpose: master reads once per slave model; translates drift_ppm → trim steps

    Name:    fire_floor_drift_residual_us
    Type:    i16 (Q8.8 microseconds, signed)
    Access:  RW
    Init:    0
    Range:   ±128 µs (operational envelope: 0–1.1 µs with biased rounding)
    Apply:   pending-atomic; effective at next USART1 TC
    Purpose: per-chip drift-residual contribution to fire floor

Existing field used by CAL (A9.1, no schema change):

    Name:    clock_trim
    Type:    i8 (signed delta from device's factory-default HSITRIM)
    Access:  RW
    Range:   bounded by the slave's hardware (V006: −16 ≤ x ≤ +15)
    Apply:   pending-atomic; effective at next USART1 TC

### 13.5 Master orchestration

    for each slave on the bus:
        send PING                               # enumeration
        ppm_per_step = read(slave_id, comms.clock_trim_ppm_per_step)
                       # cache by model_number across slaves

    for each slave:
        current_trim = read(slave_id, comms.clock_trim)
        send CAL(slave_id, count=128) at 1 Mbaud
        (T_request_end, T_first, T_last) = hse_timer.stamps_for(slave_id)
        compute drift_ppm, step, new_trim, residual_q88   (per §7)
        if step != 0:
            send Write(comms.clock_trim, new_trim)
        if abs(residual_q88) > Q88_NOOP_THRESHOLD:
            send Write(comms.fire_floor_drift_residual_us, residual_q88)
        await Status ACKs

    # all slaves trimmed; safe to enable Fast Sync/Bulk Read
    switch bus to operational baud if desired
    begin normal operation

Total cal overhead per slave: ~15–25 ms at 1 Mbaud. Per 16-slave arm: ~350 ms. Fits in the gap between "robot powers up" and "control loops start sending position commands."

---

## 14. Alternatives we tried or considered

The architecture went through several false starts. Documenting them here so future readers don't re-derive the same dead ends.

**Chip-side measurement (slave times itself).** Discussed in §5. Structural bias: every term in the timing measurement is HSI-quantized, so the "ground truth" comparison `(observed − nominal)` can't separate drift from ISR-latency offset. Compensation requires a hardcoded magic constant that doesn't generalize across chip revs or baud rates. Killed.

**Slot-fit binary classifier (`tune_hsitrim.py` style).** Master writes a candidate trim, runs a Fast BulkRead with INJ as slot 0 and DUT as slot 1, observes whether the slots coalesce cleanly. Sweeps trim values, picks the highest "clean" result. *Works empirically* — this is in fact the script that scope-validated HSITRIM=20 on rev_b. But:

- Brute-force binary search, not closed-form one-shot.
- Needs a co-slave with HSE on the bus to provide the slot 0 timing reference. Production usually has just master + DUTs.
- Requires the Fast slot machinery to already be working on the DUT (chicken-and-egg with first-boot cal of new firmware).

Kept as a bench *health check* and as the characterization sweep for `FIRE_STRUCTURAL_FLOOR_TICKS` (§8) — not as the cal method itself.

**Sync CAL.** Reusing existing SyncRead slot infrastructure would make the firmware diff tiny. But slot timing relies on all slaves agreeing on µs, which only holds *after* trim. With untrimmed ±1.2% drift, worst-case adjacent slots collide by ~50 µs at 3 Mbaud (~26 µs at 1 Mbaud). Chicken-and-egg. Deferred — Sync cal becomes safe and useful for periodic *re*-cal of already-trimmed chips, but not first-boot.

**Broadcast CAL (ID 0xFE).** DXL broadcast forbids responses, which leaves the HSE timer with nothing to stamp. Useless for measurement. Killed.

**Bumping `DXL_TX_BUF_LEN` to 512 to support N up to ~500 at high baud.** Considered when we briefly wanted single-shot at every baud. Capping N at 128 instead keeps the buffer at 256, leaves margin for other slave implementations, and trades single-shot at 3 Mbaud for 3-shot averaging. Net simpler.

**MINC=0 DMA streaming for arbitrarily large N.** Hardware supports it (RM §8.3.3 bit 7), HAL exposes it. Would let us emit N=600 with a 1-byte source. But N=128 covers our needs at every CH32V006-reachable baud, and the multi-phase DMA reconfigure adds ~30% of a char-time of inter-phase wire gap at 3 Mbaud. Deferred until a chip past 3 Mbaud needs it.

**Single floor value covering everything (no structural/residual split).** Tried first — landed at empirical FLOOR=3 µs that mixed structural latency with drift residual. The mix varied per chip, so closed-form per-chip cal couldn't decompose it. Splitting into a compile-time per-family `FIRE_STRUCTURAL_FLOOR_TICKS` plus a tunable `fire_floor_drift_residual_us` makes the residual derivable from the CAL drift measurement in closed form. One characterization sweep on the family upfront, no per-chip brute force at boot.

**Two separate CAL instructions (one for drift, one for floor).** Considered when we worried that the CAL response path's structural latency might not match the Fast slot fire path's latency (different ISR entry points, different prologues). Resolved by *not measuring* the structural floor via CAL at all — hardcoding it per family from a one-time bench sweep — and computing only the residual from the drift measurement. Single CAL stays sufficient.

**Auto-tuning the structural floor at boot.** Possible (run a Fast-slot probe after CAL lands the drift), but requires Fast slot infrastructure working before CAL completes (chicken-and-egg), and the structural component is HCLK-cycle-deterministic with negligible chip-to-chip variation. One-time bench characterization is cheaper and more robust.

**Normal `round()` instead of biased `floor()` for the trim step.** Lands chips on either side of optimal. Half end up "fast" and need fire to be *retarded*, which FLOOR can't do — FLOOR only advances. Biased rounding always puts the chip on the slow side, where FLOOR can take up the slack. Cost: wastes one trim step of headroom, well within the ±16 range V006 provides.

---

## 15. Honest accounting

What this buys:

- **Per-chip drift correction *and* per-chip fire residual compensation from a single CAL round trip.** No second wire op, no second round of math.
- **No persistent storage.** No flash writes, no OB programming, no factory cal step, no chip-to-chip cal database.
- **No control-loop disruption.** Every slave-side write is to RAM or `RCC_CTLR.HSITRIM` — a single register store, sub-microsecond, no flash controller. Safe to do at any time.
- **Tiny slave firmware footprint.** ~15 LoC of dispatcher (CAL handler) + ~30 LoC of apply path. No chip-side cal state machine.
- **Environmentally current.** Re-cal at every boot tracks the chip's actual operating temperature, supply voltage, and aging.
- **Baud independent.** Cal once at 1 Mbaud, valid at all bauds. Master can change the bus baud after cal without re-cal.
- **Composes with A8.2.** Cal pins the baseline at the chosen operational point; A8.2 cleans up residual on a per-shot basis everywhere else.
- **Portable wire contract.** CAL, `clock_trim`, `clock_trim_ppm_per_step`, and `fire_floor_drift_residual_us` don't bake any V006-specific assumptions into the wire. Same protocol works for any future chip with a similar trim register and a published ppm/step ratio.

What this doesn't buy:

- **The structural floor still has to be characterized once per chip family.** One-time bench sweep across 3–5 chips, ~1 day on the bench. Cost lands on the firmware developer adding a new chip family, not on every boot.
- **Cal is volatile.** Slave reboots back to factory HSITRIM (±1%) and `fire_floor_drift_residual_us = 0`. The master must detect slave reboot (timeout + re-PING) and re-issue CAL before resuming Fast traffic on that slave.
- **Requires an HSE-accurate timer on the bus.** On the bench that's the V203 injector; in production it's the MCU master itself. A bare DUT plugged into a bus with a host-PC-only master (no MCU intermediary, no HSE-clocked stamping) can't be cal'd by this scheme. For OpenServoCore's deployment model this is fine — production deployments always have an MCU master — but a stock DXL host SDK won't transparently calibrate these chips.

What this trades off:

- **~15–25 ms per slave at boot, ~350 ms for a 16-slave arm.** Bounded, predictable, master-orchestrated.
- **One vendor instruction code** in DXL's instruction space.
- **Two new RW control-table fields** (`fire_floor_drift_residual_us` — i16) **plus one RO metadata field** (`clock_trim_ppm_per_step` — u16). 6 bytes total.
- **One compile-time constant per chip family** (`FIRE_STRUCTURAL_FLOOR_TICKS`), characterized once on the bench.
- **A small amount of host-side machinery**: HSE-timer query, drift + residual math, per-slave loop. For the bench, ~60 LoC of Python in conftest. For production, a one-time addition to the host SDK.

---

## 16. One-paragraph summary

> The V006's HSI carries ±1% process tolerance, which translates to ~16 µs of wire-side timing drift on a 128-byte predecessor at 1 Mbaud — enough to break Fast Sync/Bulk Read coalesce by either opening a visible gap or running into the predecessor's last byte. We fix this with two orthogonal knobs: HSITRIM (multiplicative drift correction, controlled via `comms.clock_trim`) and FIRE_FLOOR (additive fire-advance, split into a compile-time per-chip-family `FIRE_STRUCTURAL_FLOOR_TICKS` constant plus a per-chip RW `comms.fire_floor_drift_residual_us` in signed Q8.8 microseconds). The slave can't reliably measure its own clock (measuring HSI with HSI is structurally biased — an earlier prototype was 9 trim steps off optimum with a magic constant we couldn't derive), so the master sends `CAL(N=128)`, slave responds with N zero bytes through its standard reply path, and an HSE-accurate timer on the bus (V203 injector on the bench, MCU master in deployment) stamps the slave's first and last response bytes. From a single CAL the master derives drift via `(T_last − T_first)` vs nominal, picks a trim step using *biased rounding* (always toward "slow") so FLOOR can always advance fire to compensate, and computes the residual µs from the rounding remainder times the deployment's worst-case predecessor wire length. Both values are written back via standard DXL `Write`; the slave applies them in its USART1 TC handler after the Status ACK drains. One round trip, no slave-side math, no chip-side cal state machine, no flash writes. Cal is volatile and re-derived on every boot, which tracks temperature naturally. ~15–25 ms per slave at 1 Mbaud, ~350 ms for a 16-slave arm. Composes with live per-shot drift correction (A8.2) for residual cleanup. Stable 3 Mbaud Fast operation falls out comfortably within the 3.33 µs jitter cap.

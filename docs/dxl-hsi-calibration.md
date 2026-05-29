# Runtime HSI Calibration for DXL Fast on the CH32V006

Companion to [dxl-rx-timing.md](dxl-rx-timing.md) and [dxl-fast-chain-crc.md](dxl-fast-chain-crc.md). Read those first — this doc assumes you already know what "wire-end timestamp," "Fast last-slave coalesce," and "the V006's SysTick CMP fire path" mean.

**TL;DR.** The V006 has no crystal, so its 24 MHz HSI carries ±1% process tolerance straight into every wire-side timing decision the chip makes. On a 128-byte predecessor at 1 Mbaud that's ~16 µs of drift — more than enough to either open a visible idle gap or run the last slave straight into the predecessor's last byte. The chip *does* expose a 5-bit `HSITRIM` field that nudges HSI in ~0.25% steps, runtime-writable with a single register store. The question is how to find the right value and where to keep it. Storing in flash (main or OB) doesn't work — flash writes stall the CPU for milliseconds, which a torqued-on motor cannot tolerate; and a robot in operation is never not torqued on. So we don't store at all. The master sends a two-instruction calibration handshake at robot init: CAL_MEASURE → slave reports raw timing stamps → master computes the optimal HSITRIM step → CAL_APPLY → slave writes `RCC_CTLR.HSITRIM`, no flash touched. Single shot lands within ±0.125% (half a step). HSITRIM is baud-independent because it tunes HSI itself, so cal once at any baud and the correction holds at every baud. Cal is volatile and re-derived on every boot, which is a feature rather than a cost — it tracks temperature, supply, and aging naturally. Composes with the live per-shot drift correction from §A8 of the impl plan: cal pins the baseline, live correction mops up residual.

---

## 1. Why HSI drift matters for Fast Sync/Bulk Read

The whole point of the Fast Sync/Bulk Read chain CRC ([dxl-fast-chain-crc.md](dxl-fast-chain-crc.md)) is that all addressed slaves' replies stitch into one coalesced Status frame on the wire with zero idle gap between slots. The host's USART recovers it as one big packet. The jitter cap for inter-slave hand-off is one byte time — 3.33 µs at 3 Mbaud. Land later than that and the host's USART asserts IDLE between slots, framing breaks, the whole coalesced frame is lost.

The last slave's reply has to start at exactly:

    fire_us = RDT + bytes_before × byte_time

where `bytes_before` is the predecessors' total wire bytes already on the bus. That's `fire_us` measured in microseconds of real wall-clock time. The slave doesn't *have* wall-clock time — it has SysTick, which counts HCLK cycles. SysTick rate = HCLK = HSI × PLL_mult. If HSI is off by ε, SysTick is off by ε, and so is the slave's belief about how long `bytes_before × byte_time` is.

If HSI is **slow** by 1%: SysTick under-counts, the slave thinks the deadline is at `fire_us` but in real time it lands at `fire_us × 1.01`. That's a positive gap — the slave fires *after* the predecessor wire-end. Visible idle, possible IDLE assertion on the host.

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

For Fast Sync/Bulk to work cleanly across all bauds and predecessor sizes, the drift has to come out. There's no software-only workaround — the chip's own SysTick is the only available time reference, so we have to fix HSI itself.

---

## 2. The HSITRIM lever

The V006 reference manual §6 documents a 5-bit `HSITRIM` field in `RCC_CTLR[7:3]`, RW, default 16 (binary `10000b`). Each step shifts HSI by ~60 kHz at the nominal 24 MHz — about **0.25% per step**. Total range is ±16 steps = ±4% in principle, though the manual specifies ±1% as the trim range, suggesting the silicon is characterized in the inner 8 steps. The factory `HSICAL[7:0]` byte (bits [15:8] of the same register, read-only) is a per-chip calibration baseline; `HSITRIM` is the *adjustment* superimposed on it.

What this gives us:

- A direct knob for correcting the per-part HSI offset measured in §1
- 0.25% step granularity → after tuning, residual error bounded by ±0.125% (half a step)
- Runtime-writable: a single register store, sub-microsecond, no flash controller, no CPU stall, completely safe to do at any time including during the motor control loop
- Affects HSI itself → HCLK, BRR-derived bauds, and SysTick all scale uniformly → **baud-independent correction**

That last point is the whole reason this works as a runtime fix. We're not retiming individual peripherals — we're nudging the clock source they all hang off, so everything stays in proportion. Cal at 1 Mbaud, switch the bus to 3 Mbaud for operation, the HSITRIM applied at 1M is still optimal at 3M.

**Direction:** higher HSITRIM = higher HSI frequency. So a slow chip (positive observed gap) needs HSITRIM raised. The rev_b chip we calibrated landed at HSITRIM=20 (default 16 + 4 steps = +1.0%), within one step of the 1.25% drift estimate.

---

## 3. Why we don't store the trim value

The natural instinct is to cal once at production, store the result somewhere persistent, and load it at boot. Both main flash and the V006's option bytes (Data0/Data1 at `0x1FFFF804`) sit behind the same FLASH controller. The OB write sequence is slightly different from main flash (different unlock keys, different erase granularity), but the underlying hardware behavior is identical:

- Erase a flash page: ~ms-scale, CPU stalled
- Program a flash word: ~tens-of-µs, CPU stalled
- Total for one cal commit: ~5–10 ms of pure CPU stall

For a slave that's just sitting on the bench, that's fine. For a slave that's currently running a motor control loop at 20 kHz, it's catastrophic. The PWM duty register stays at whatever value it was holding when the stall started; the position-loop feedback stops updating; the motor either holds at the wrong torque or runs away depending on what the loop was about to do. Either way the robot dropped a metaphorical wrench.

The DXL 2.0 protocol convention captures this constraint as "no EEPROM writes while torque is on" — it's the same physical concern, lifted into the protocol layer. We could honor that convention and defer the cal commit to torque-off windows. But this is OpenServoCore, intended for robots that operate continuously. A robot in operation is essentially always torqued on. The torque-off window may never come.

So the workaround paths don't work:

- "Just commit during torque-off" — never happens during operation
- "Use OB instead" — same flash controller, same stall
- "Special-case HSITRIM in the torque-lock policy" — bends a clean protocol invariant to plaster over a hardware limit

The sidestep is cleaner than any workaround: **don't store the trim value anywhere persistent. Recompute it on every boot.**

This sounds wasteful at first ("but we're throwing away the cal data on every reset!"), but actually solves several problems at once:

- **No flash wear.** HSITRIM cal is one of those things you could imagine wanting to update over the lifetime of the chip; with no flash involvement, that's free.
- **No factory step.** Slaves don't need to be pre-flashed with cal data — the runtime protocol handles it.
- **Environmentally current.** HSI has a temperature coefficient (~0.05%/°C is typical for these RC oscillators). A factory cal done at room temperature isn't quite right for a chip running hot in a robot. Recalibrating at boot tracks whatever conditions the chip is actually starting up in. If thermal drift becomes significant mid-operation, the master can re-issue cal at an idle moment without restart.
- **Dev workflow.** During firmware development we re-flash constantly. A factory-cal stored in flash gets wiped with every mass erase, which means a re-cal step in the dev loop. With runtime cal, the firmware build doesn't care.

The cost is the boot-time latency of the cal handshake — bounded and small, covered in §4.

---

## 4. Master-driven calibration handshake

The master orchestrates cal as part of robot init, after PING enumeration but before enabling Fast Sync/Bulk Read on a slave. Two vendor DXL instructions, one round trip each:

    1.  Master  →  Slave:   CAL_MEASURE  (request: N bytes of any content)
        Slave   →  Master:  Status payload with raw timing stamps + current HSITRIM

    2.  Master computes optimal HSITRIM (host arithmetic, see §6)

    3.  Master  →  Slave:   CAL_APPLY  (request: 1 byte = new HSITRIM)
        Slave   →  Master:  Status ACK

    4.  Slave applies new HSITRIM after the Status TC drains (apply-after-TC pattern,
        mirrors the existing DXL_BAUD_PENDING_BRR mechanism so HSITRIM never changes
        mid-byte).

Time budget: ~3 ms per slave at 1 Mbaud (the cal burst is the slow part — 128 bytes × 10 µs = 1.28 ms each direction, plus protocol overhead). For a 30-DoF arm, total cal at boot is ~90 ms. Invisible against the rest of the robot's startup sequence.

**Why two instructions rather than one.** The natural alternative is a single CAL instruction that does measurement + apply in one shot. The reason for the split:

- The slave needs to *measure* during the request and *reply* with what it observed. So the request payload is the cal burst, and the Status payload carries the measurement.
- Computing the optimal HSITRIM requires division — `(observed - nominal) × 400 / nominal`. The V006 V2A core **has no hardware divide instruction**. A software divide call is fine in cold paths but it's another pile of code on the slave and an avoidable one if the master can do it (which it always can — it's on a real CPU).
- Keeping the slave dumb means the slave never makes policy decisions. The master can iterate, average multiple shots, apply convergence criteria, or just take the single shot and call it done. None of that logic lives on the slave.

So the slave is a meter (stamp + report); the master is the brain (compute + decide); the apply step is a separate write so the master's decision is explicit.

---

## 5. The measurement primitive

This is the same RX byte-stamping primitive that A8 (live per-shot drift correction) needs anyway — adding it once serves both consumers.

In the USART1 ISR, on each RXNE entry:

    on USART1 entry (when RXNE is the cause):
        count = BYTE_COUNT.load()
        tick  = systick::ticks()
        if count == 0:
            FIRST_TICK.store(tick)        # only on the very first byte after IDLE
        LAST_TICK.store(tick)             # every byte; only the last one ends up sticking
        BYTE_COUNT.store(count + 1)

On IDLE (packet end), the IDLE handler snapshots `(FIRST_TICK, LAST_TICK, BYTE_COUNT)` and resets `BYTE_COUNT = 0` for the next packet. The last tick gets backdated by `9 × BRR` ticks (same `DXL_CHAR_TIME_TICKS` mechanism from [parent doc §8.6](dxl-rx-timing.md)) to recover the moment the last byte's stop bit actually completed on the wire.

The CAL_MEASURE handler reads the snapshot:

    observed_ticks_total = LAST_TICK_backdated − FIRST_TICK   (in slave's SysTick ticks)
    byte_count           = the N actually received

That's the slave's complete contribution to the math. No division, no scaling, no per-byte averaging — just two timestamps and a count.

**Why differential ISR latency cancels.** Both stamps are taken at USART1 RXNE entry — first byte and last byte enter the ISR through the same code path. The latency from "stop bit completes on the wire" to "FIRST_TICK.store(tick)" is the same as the latency to "LAST_TICK.store(tick)" (modulo a few cycles of jitter from cache and branch effects). When we compute `LAST_TICK − FIRST_TICK`, those symmetric offsets cancel exactly — we get the genuine wall-clock duration of (N-1) byte times in our drifty SysTick ticks.

This is *much* cleaner than trying to time the wire-side start of the first byte directly. The chip's first-byte-after-IDLE latency is a few µs of PFIC trap entry + ISR body, and even if we could subtract that, it has jitter. RXNE-to-RXNE deltas are invariant to all of that.

**Why we can do this even with DMA RX.** The parent doc's [§8 V006 quirk note](dxl-rx-timing.md) covers this — DMA wins the flag-clear race on RXNE, so reading `STATR.RXNE` inside the ISR always sees 0, but the PFIC pending bit latches per byte and the IRQ fires 1:1 per byte with zero overruns up to 3 Mbaud. We don't need to read RXNE — we're not driving framing off it here. We just need the IRQ to fire, which it does.

---

## 6. The math (master-side)

The Status payload from CAL_MEASURE carries:

    observed_ticks_total: u32 LE   # (N-1) bytes wall-clock in slave's drifty ticks
    nominal_ticks_per_byte: u32 LE # slave's DXL_BYTE_TIME_TICKS at current baud
    byte_count: u16 LE             # actual N received
    current_HSITRIM: u8            # 0..=31, what's in RCC.HSITRIM right now

Master computes:

    nominal_total = (byte_count - 1) × nominal_ticks_per_byte
    drift_ppm     = (observed_ticks_total - nominal_total) × 1_000_000 / nominal_total
    step_delta    = round(drift_ppm / 2500)               # 0.25%/step = 2500 ppm/step
    new_HSITRIM   = clamp(current_HSITRIM + step_delta, 0, 31)

Notice the master doesn't need to know HCLK, baud, or any V006-specific constants — `nominal_ticks_per_byte` ships in the payload directly. The protocol is portable across baud, HCLK choice, and even chip family — anywhere we ship this calibration scheme on a part with a similar HSI trim, the same handshake works.

**Single-shot precision.** One CAL_MEASURE shot lands within ±0.125% (half a HSITRIM step) under typical bench conditions, well-bounded by the SNR analysis in §7. If sub-step precision is wanted, the master can average several shots before deciding — but the live drift correction from A8 picks up the residual better than master-side averaging anyway, so this is rarely worth doing.

**Sign convention.** Positive `step_delta` means HSI was slow, raise the trim. Negative means HSI was fast, lower the trim. The arithmetic gets this right naturally — `observed_ticks_total > nominal_total` ↔ slave thinks more time passed than really did ↔ HSI is fast — wait. Let me redo that.

    HSI fast  →  SysTick counts faster than real time  →  more ticks per real second
                 →  observed_ticks_total > nominal_total  (we counted more ticks for
                    the (N-1) real-time bytes than nominal would predict)
                 →  drift_ppm > 0, step_delta > 0
                 →  raise HSITRIM further?

Hmm, that's the opposite of what we want — if HSI is already fast we should *lower* HSITRIM. Let me re-derive.

When HSI runs at real `f_actual` vs nominal `f_nominal`, and the slave's SysTick is clocked from HSI directly (which it is on V006 V2):

    observed_ticks = real_duration_s × f_actual

For an INJ burst that takes a fixed real wall-clock time `t_real` (set by the master's crystal):

    observed_ticks = t_real × f_actual
    nominal_ticks  = t_real × f_nominal           (what we'd see if HSI were exact)

So `observed > nominal` when `f_actual > f_nominal` (HSI is fast). To correct, we lower HSITRIM.

That means `step_delta = round(drift_ppm / 2500)` with `drift_ppm = (observed - nominal) × 1e6 / nominal` gives the wrong sign. We need:

    step_delta = -round(drift_ppm / 2500)       # flip sign — high HSI wants lower HSITRIM
    new_HSITRIM = clamp(current_HSITRIM + step_delta, 0, 31)

OK so the actual formula has a negation in it. Worth noting in the spec so we don't get it backwards in the bench harness.

---

## 7. Baud independence and SNR

**HSITRIM tunes HSI itself**, not anything baud-specific. HCLK = HSI × PLL_mult, the USART BRR divider is a fixed integer (set when the application configures the baud), and SysTick is clocked off HCLK. All scale together with HSI. So a chip calibrated against 1 Mbaud traffic and then asked to talk at 3 Mbaud will have the same residual HSITRIM accuracy at 3 Mbaud — the trim never "knew" what baud it was applied at.

This means the master can pick whatever baud is convenient for the cal handshake. The OpenServoCore default startup baud is 1 Mbaud, so cal happens at 1M; the master can then switch the bus to 3M for operation if it wants the higher throughput, without needing to re-cal.

**SNR by baud (128-byte burst):**

    | baud   | byte_time | nominal_total ticks | jitter ratio (±2-tick ISR jitter) |
    | ------ | --------- | ------------------- | --------------------------------- |
    |  9600  |  ~1040 µs |          6,350,000  |   0.00003 %                       |
    |  115k  |    87 µs  |            530,000  |   0.0004 %                        |
    |   1 M  |    10 µs  |             60,960  |   0.003 %                         |
    |   3 M  |   3.33 µs |             20,320  |   0.01 %                          |

All well under the 0.25% per-step granularity of HSITRIM — even at 3 Mbaud we have **25× margin** over the one-step decision threshold. The bottleneck for cal precision is the HSITRIM step granularity (±0.125% residual after rounding), not measurement noise.

Lower baud gives more SNR cushion but slower cal. Higher baud is faster but slightly noisier. Anywhere in DXL's baud range works.

---

## 8. The slave's apply path

`CAL_APPLY` carries a 1-byte payload — the new HSITRIM value. The handler clamps to 0..=31 (a 5-bit field can't legally hold more), stores in a pending atomic, and emits a standard Status ACK. The actual write to `RCC_CTLR.HSITRIM` happens in the USART1 TC ISR, after the Status reply has fully shifted onto the wire.

Why apply-after-TC: writing HSITRIM shifts HCLK, which shifts USART baud (since the BRR divider is fixed at the previous HCLK/baud point). If we wrote HSITRIM while the Status reply was still in flight, the bytes after the write would shift out at a slightly different baud than the bytes before — the master's USART might or might not recover them depending on how big the jump was. By deferring to TC, we ensure HSITRIM only changes on byte boundaries, when no bits are mid-flight.

This pattern already exists in the codebase for the `BAUD_RATE_IDX` control-table write — when the host changes the slave's baud, the Status ACK ships at the *old* baud (so the host's current-baud USART can decode it), and the actual BRR change happens in TC. HSITRIM reuses the same mechanism with a different pending field.

No baud-derived state needs recomputation after a HSITRIM change. `DXL_BYTE_TIME_TICKS = BRR × 10` is in HCLK cycles — that's the SysTick tick count for one wire byte. Both HCLK and SysTick rates scale with HSI, so the *ratio* (ticks per byte) stays constant. The `store_baud_derived` function doesn't need to run.

---

## 9. Composes with live drift correction

The plan calls for two cooperating mechanisms:

- **CAL (this doc)**: master-driven boot-time HSITRIM cal. Brings each chip's baseline drift from ±1% (factory) down to ±0.125% (one HSITRIM step).
- **Live drift correction (A8)**: at WaitingSwitch CMP fire, observe actual byte cadence on the current predecessor and recompute the fire deadline. Mops up residual drift on a per-shot basis.

Why both:

- **CAL alone is good but the residual is ±0.125% of HSI itself.** Over an 800 µs predecessor wire window at 3 Mbaud that's ±1 µs of slop on the fire deadline — close to the 3.33 µs jitter cap but not safely under it.
- **Live correction alone has no observation window on short predecessors.** A FastBasic test with 4 wire bytes has ~13 µs of wire time at 3 Mbaud — not enough to derive a robust byte-cadence estimate from. Without a cal baseline, the chip eats the full ±1% factory drift on short predecessors.
- **CAL + live together**: cal gets the long-predecessor case clean to within ±0.125%; live drift correction gets the long-predecessor case clean to whatever the SNR-per-observation supports (usually much better than that); short-predecessor case rides cal's baseline.

The two share the RX byte-stamping primitive — first/last RXNE ticks, byte count, IDLE-backdating. CAL reads the stamps and reports raw values; A8 reads the stamps and recomputes a fire deadline. Same producer, two consumers.

---

## 10. Protocol spec

### 10.1 Instruction codes

Two vendor codes in the DXL 2.0 instruction byte space:

    CAL_MEASURE  = 0x40    (TBD — pick once, document, never change)
    CAL_APPLY    = 0x41

### 10.2 CAL_MEASURE request

Standard DXL 2.0 frame:

    [ 0xFF 0xFF 0xFD 0x00 ] [ id ] [ length LE ] [ 0x40 ] [ N bytes of any content ] [ CRC ]

The N bytes are the cal burst — slave just measures their timing on the wire. Content is irrelevant. Convention: `0xAA` for visual distinctiveness on a scope, but `0x55`, `0x00`, random data — anything works.

### 10.3 CAL_MEASURE Status reply

Standard Status frame with an extended data payload:

    [ 0xFF 0xFF 0xFD 0x00 ] [ id ] [ length LE ] [ 0x55 ] [ err ] [ data ] [ CRC ]

where `data` is 11 bytes, little-endian:

    Offset  Size    Field
    ------  ----    -----
       0     u32    observed_ticks_total      (LAST_TICK_backdated − FIRST_TICK)
       4     u32    nominal_ticks_per_byte    (DXL_BYTE_TIME_TICKS)
       8     u16    byte_count                (actual N received)
      10     u8     current_HSITRIM           (0..=31)

If `byte_count < 2` the slave can't form a meaningful (N-1)-byte duration; it should return `err = 0x07` (Data Range) and leave the rest of the payload zeroed. Master sees the error and either retries with a longer burst or declares cal failed.

### 10.4 CAL_APPLY request

    [ 0xFF 0xFF 0xFD 0x00 ] [ id ] [ length LE ] [ 0x41 ] [ HSITRIM byte ] [ CRC ]

HSITRIM byte is the new 5-bit value (high 3 bits zero). Slave clamps to 0..=31 silently if anything's set above bit 4.

### 10.5 CAL_APPLY Status reply

Standard ACK:

    [ 0xFF 0xFF 0xFD 0x00 ] [ id ] [ length LE ] [ 0x55 ] [ err ] [ CRC ]

with `err = 0` on success. The HSITRIM register write happens in the USART1 TC ISR after this frame fully ships — so by the time the master sees the ACK, the new HSITRIM is in effect on the slave's HSI.

### 10.6 Expected master orchestration

    for each slave on the bus:
        send PING                              # enumeration
        ...
    for each slave:
        send CAL_MEASURE with N=128 burst
        parse Status payload
        compute step_delta, new_HSITRIM        (see §6)
        send CAL_APPLY with new_HSITRIM
        await Status ACK
    # now safe to enable Fast Sync/Bulk Read on these slaves
    switch bus to operational baud if desired
    begin normal operation

Total cal overhead per slave: ~3 ms at 1 Mbaud. Per arm, ~100 ms. Fits in the gap between "robot powers up" and "control loops start sending position commands."

---

## 11. Honest accounting

What this buys:

- **Per-chip HSI drift correction without any persistent storage.** No flash writes, no OB programming, no factory cal step, no chip-to-chip cal database to manage.
- **No control-loop disruption.** The only write the slave performs is `RCC_CTLR.HSITRIM` — a single register store, sub-microsecond, no flash controller. Safe to do at any time.
- **Environmentally current.** Cal at every boot tracks the chip's actual operating temperature, supply voltage, and aging. A factory cal stored in flash is a snapshot of the chip's room-temperature behavior at the moment it was tested.
- **Baud independent.** Cal once at any baud, valid at all bauds. Master can change the bus baud after cal without re-cal.
- **Composes with A8.** HSITRIM nails the baseline within ±0.125%; A8 mops up residual on a per-shot basis. Together they get clean Fast coalesce across all bauds and predecessor lengths.
- **Sets us up for chip-family portability.** The protocol carries `nominal_ticks_per_byte` so the master needs no V006-specific constants. The same handshake works for any future chip with a similar HSI trim register.

What this doesn't buy:

- **The chip's structural fire floor (~4 µs on V006 V2A) is unchanged.** Cal removes the *drift* component of the wire-side gap, not the structural PFIC trap entry + ISR body + DMA prefetch + start-bit latch. The fixed pre-comp in `FIRE_STRUCTURAL_FLOOR_US` (set against the corrected HSI, currently 3 µs) handles the structural part.
- **Cal is volatile.** Slave reboots back to factory HSITRIM, which is ±1% drift. The master must detect slave reboot (e.g., timeout + re-PING) and re-issue cal before resuming Fast Read on that slave. For OpenServoCore this is part of the master's per-slave bring-up state machine.
- **No standalone-slave mode.** A slave plugged into a bus with no master that knows about CAL never gets calibrated. The chip operates at factory HSI tolerance. For OpenServoCore's deployment model this is fine — slaves always come paired with a master — but it does mean the protocol isn't transparently backward-compatible with a stock DXL master.

What this trades off:

- **~3 ms per slave at boot.** Bounded, predictable, master-orchestrated.
- **The RX byte-stamping primitive** in the USART1 ISR — small (~10 lines), shared with A8.
- **Two vendor instruction codes** in DXL's instruction space. We're already using the Fast Sync/Bulk Read vendor extensions, so adding two more is no protocol-aesthetic cost.
- **HSITRIM pending-apply infrastructure** in the USART1 TC ISR (mirrors the existing `DXL_BAUD_PENDING_BRR` mechanism — ~6 lines).
- **A small amount of master-side complexity**: per-boot cal handshake, reboot detection + re-cal. For OpenServoCore this lives in the host SDK once, then every consumer of the SDK gets it for free.

---

## 12. One-paragraph summary

> The V006's HSI carries ±1% process tolerance, which translates to ~16 µs of wire-side timing drift on a 128-byte predecessor at 1 Mbaud — enough to break Fast Sync/Bulk Read coalesce by either opening a visible gap or running into the predecessor's last byte. The chip exposes a 5-bit HSITRIM field at ~0.25%/step that fixes this in a single register store, but the natural instinct of "cal once and write to flash" doesn't work because flash writes (main or OB) stall the CPU for milliseconds and a torqued-on motor cannot tolerate that. Instead, the master sends a two-instruction calibration handshake at robot init — CAL_MEASURE (slave reports raw RXNE-to-RXNE timing stamps over a 128-byte burst) → master computes step_delta from the observed/nominal ratio → CAL_APPLY (slave writes the new value to RCC_CTLR.HSITRIM, after Status TC drains so no bytes are mid-flight when HCLK shifts). The slave does no division. Single-shot precision lands within ±0.125% (half a HSITRIM step), and SNR is fine at every DXL baud from 9600 to 3M. HSITRIM tunes HSI itself so cal is baud-independent — cal once at any convenient baud, the correction holds when the bus switches baud later. Cal is volatile and re-derived on every boot, which tracks temperature and supply naturally and eliminates the entire flash-storage tree. Composes with the live per-shot drift correction (A8) for residual: cal pins the baseline, live correction mops up the rest, and Fast Sync/Bulk Read coalesces cleanly across all bauds and predecessor lengths.

# Kernel and Scheduling Architecture

The scheduling architecture of the osc servo firmware on the CH32V006: the kernel tick and its phases, interrupt priorities, the budget table and the DES (discrete-event simulator), the telemetry lane, the configuration take, DMA use and the code-shape rules. The kernel lives in `firmware/lib/core/src/kernel/`; its chip binding lives in `firmware/servo-ch32/src/runtime/`. Companion pillars: [osc-native-protocol.md](osc-native-protocol.md) (the wire), [osc-servo-transport.md](osc-servo-transport.md) (the receiver), [control-theory.md](control-theory.md) (the loops) and [driver-pattern.md](driver-pattern.md) (the layering).

Code is authoritative; when this doc and the code disagree, fix the doc.

---

## 1. Overview

One core runs four execution contexts:

| Context | Trigger | Priority | Code | Job |
| --- | --- | --- | --- | --- |
| **Kernel tick** | DMA1 CH1 TC, once per PWM period | PFIC HIGH | `lib/core/src/kernel/`, `runtime/isr.rs` | sensing, control, motor write, TEL lane |
| **Bus vectors** | TIM2, USART1, SysTick | PFIC LOW | `lib/drivers/src/bus/`, `runtime/isr.rs` | frame service, dispatch, replies |
| **Main loop** | `wfi` wake | thread mode | `runtime/run.rs` | cold work |
| **DMA** | peripheral requests | DMA priority ladder | [transport sec 2](osc-servo-transport.md#2-hardware-ledger) | scans, copies, TX, CRC feed |

The platform facts the design assumes:

| Fact | Consequence |
| --- | --- |
| One core, HCLK 48 MHz (`CLOCK_HZ`) | every budget is in HCLK ticks |
| Flash fetched one 4-byte word per 3 HCLK, no cache | code bytes and taken branches are execution time ([sec 8](#8-code-shape-on-a-fetch-bound-chip)) |
| Hardware multiply, no divide | runtime divisors are compile-time constants |
| Small RAM; the stack is the binding limit ([v006-feature-budget.md](v006-feature-budget.md#budget-lines)) | no code in RAM, no per-task stacks |

---

## 2. Static phased scheduling

### 2.1 The tick

TIM1 runs center-aligned PWM at `TICK_HZ` (20 kHz). Each period it triggers two ADC scans, at the trough and at the crest; DMA1 CH1 lands both in the control table's tick block ([sec 7.2](#72-adc-scans-land-in-the-tick-block)). The TC (transfer complete) interrupt after the crest scan is the kernel tick. No software starts or re-arms it.

> **The tick index is the kernel's only clock.**

`sample_tick` counts executed ticks; each executed tick advances the kernel by one period; nothing in the kernel reads a wall clock. A tick that does not run is counted in `tick_lost_count` ([protocol sec 5.9](osc-native-protocol.md#59-health-osc-servo)) and is not compensated. Ticks are lost only to a body over two periods, a flash SAVE (torque off) or a debug halt. A shunt burst runs no tick for its window ([protocol sec 5.8](osc-native-protocol.md#58-limits-and-the-stall-permit-osc-servo)).

### 2.2 Rates and phases

| Rate | Period | Work |
| --- | --- | --- |
| **FAST** | every tick, `TICK_HZ` = 20 kHz | window measurement, overcurrent trip, back-EMF sums, current loop, motor write |
| **MEDIUM** | `DECIM_MED` = 10 ticks (2 kHz) | outer loops, estimators, limits, detectors, publishes; one phase per tick |
| **SLOW** | `DECIM_SLOW` = 32 medium periods (62.5 Hz) | stall permit lease, undervoltage, the two heats; then the winding estimate, its derate and overtemperature; one stage per period |

Medium phases, in data-flow order (`phase` in `kernel/scheduler/table.rs`):

| Phase | Name | Work |
| --- | --- | --- |
| 0 | CONTROL | CONTROL read, enable and mode edges, drive kind; the configuration on the first tick |
| 1 | OBSERVER | back-EMF close, observer step, speed source switch |
| 2 | TRAJECTORY | trajectory and position loop; the hold updates the drive kind |
| 3 | LIMITS | limits fold, OpenLoop base duty, stall fault |
| 4 | VELOCITY | velocity loop or current clamp; current reference and band land |
| 5 | RAIL | rail estimate and its fold, sensor screen, position error timer |
| 6 | SLOW | the slow block's due stage when due; its finish one period later |
| 7 | PUBLISH | estimates, mode, limits and sensors publish; tick-load close |
| 8 | SPARE_A | configuration take after a commit |
| 9 | SPARE_B | drive direction check while the wiring is unproven |

Module layout:

| Path | Contents |
| --- | --- |
| `kernel/scheduler/mod.rs` | `Kernel`, `on_tick`, spare-phase arbitration, `take_config` |
| `kernel/scheduler/table.rs` | rates, phase indices, health-close constants, `Medium::step` dispatch |
| `kernel/phases/` | one file per phase; `fast.rs` is the FAST half |
| `kernel/phases/loops/` | loops and detectors the phases run |
| `kernel/config.rs` | `KernelConfig`, the kernel's copy of CONFIG and CALIB |

Hand-off contracts: the medium half leaves the fast half a `Command` (drive kind, current reference and band, rail clamp); the FAST measurement reaches the same tick's medium phase as `Measured`. Only the CONTROL phase reads CONTROL, so a host write lands at the next period's CONTROL phase. The sample-to-command delay is fixed by the table ([control-theory.md, Why a Cascade](control-theory.md#why-a-cascade)).

### 2.3 Tick sequence

`runtime::isr::on_adc_dma_tc` and `Kernel::on_tick` run, in order:

1. Increment `sample_tick`; publish the scan's shunt and taps in the tick block.
2. Spare-phase arbitration: boot on the first tick, else the configuration take on SPARE_A when due ([sec 6](#6-the-configuration-take)), else on SPARE_B the drive direction check while the wiring is unproven: its plain check, or its stage (`DRIVE_STAGE`).
3. FAST measure: the window the previous command drove, published in the tick block.
4. TEL row, while a burst streams ([sec 5](#5-cut-through-tel)).
5. The medium phase.
6. FAST drive: current loop, open loop, brake or off; motor write.
7. At most one TEL lane step.
8. Shunt-burst handshake poll; tick-load accounting.

> **The tick finishes with its scan before the next scan lands.**

The next trough scan lands mid-period and the next crest scan near the period's end, in the same block. The tick moves the trough samples out by `TROUGH_DONE` and finishes crest reads and the row's DMA arm by `FRAME_DONE` (`the_tick_reads_its_scan_before_the_next_lands`).

### 2.4 Event slots

| Event | Slot | Rule |
| --- | --- | --- |
| Configuration take | first SPARE_A after a commit | owns the phase; holds no TX lane |
| SLOW block | SLOW phase, every `DECIM_SLOW` periods | the due stage, then the finish on the next period's SLOW phase (`the_slow_stages_take_one_period_each`); each holds the TX lane (`slow_stage_ticks_hold_the_tel_lane`) |
| Drive direction stage | SPARE_B, while the wiring is unproven | its own phase, apart from the take's; holds no TX lane |
| Tick-load close | PUBLISH (`HEALTH_CLOSE_PHASE`) | never on a stage or take phase (`health_close_lands_only_on_publish_while_streaming`) |
| TEL lane step | end of tick | at most one per tick; none on a head, bank or stage tick |

Arbitration is the first `match` in `on_tick`: boot, then the take, then the drive direction check.

> **Two rare events share a tick only where the budget table prices the combination.**

### 2.5 Scheduling rules

> **The schedule is static: no run-time queue, deadline or priority comparison.**

- Dispatch is one `match` on the phase counter.
- Kernel state has one owner, the tick, and no locks.
- A tick that does not fit is fixed by deleting work, making it cheaper or moving it to a phase with room ([sec 4.5](#45-fit-is-an-outcome)).
- The module tree mirrors the schedule; structure is admitted only at zero run-time cost.

### 2.6 The main loop

`runtime/run.rs` runs the work with no deadline inside a tick, then sleeps in `wfi`: status LED, link-counter and TEL-drop publish, clock trim, shunt-burst page copy, the data job (software CRC), deferred reboot, rescue-pulse sampler.

> **The main loop masks the bus level only (`pfic::mask_bus`), never the kernel.**

---

## 3. The kernel above the bus

### 3.1 Priority levels

PFIC (programmable fast interrupt controller) levels, set in `runtime::isr::install_irqs`:

| Vector | Level | Job |
| --- | --- | --- |
| DMA1 CH1 TC | HIGH | kernel tick; TEL lane while a burst streams |
| TIM2 | LOW | break wake ([transport sec 7](osc-servo-transport.md#7-the-dma-priority-ladder-and-receive-side-discipline)) |
| USART1 | LOW | TX arm drained |
| SysTick | LOW, taken last | deadline mux, dispatch inline |
| main loop | thread mode | cold work |

The bus vectors share LOW and never preempt each other. SysTick is taken last within LOW, so a pending wake or TX completion resolves before a pending deadline. Vector bodies are bindings in the sense of [driver-pattern sec 8](driver-pattern.md#8-hardware-demultiplexing).

> **The kernel never waits on the bus.**

### 3.2 The bus under preemption

A bus body can exceed a tick period (the `Deadline` body and every `frame` row exceed `TICK_PERIOD`), so the kernel preempts it. Contracts:

- Bus latency stretches by 1 / (1 - U), U the kernel's CPU share; a reply may pause about one kernel body between arms.
- RESPONSE_DEADLINE ([protocol sec 7](osc-native-protocol.md#7-timing-rules)) covers the stretched path (`reply_bound_covers_turnaround_under_kernel_preemption`).
- The kernel loses no tick to traffic (`kernel_on_top_loses_no_tick_under_polling`).
- The bus closes its preemption hazards ([transport sec 2](osc-servo-transport.md#2-hardware-ledger)): each TX arm clears TC before its DMA starts, frame projections read the ring cursor before the clock, and a configuration commit is bracketed by `config_seq` ([sec 6.1](#61-the-evenodd-count)).

### 3.3 Single-writer contracts

| Data | Writer | Kernel access |
| --- | --- | --- |
| CONTROL | bus | volatile reads via `region_ptr`, once per period at CONTROL; cross-field tearing accepted |
| CONFIG, CALIB | bus | only through the take ([sec 6](#6-the-configuration-take)) |
| position table | bus | volatile reads (`Shared::pos_lut_q4`) |
| tick block | ADC DMA and tick ISR (scan, `sample_tick`), then kernel | sole writer after the landing |
| estimates, references, mode, limits, fault byte | kernel | sole writer |
| health block | chip side (ISR wrapper, main loop) | none |
| TEL channel | split per field between `TelPost` and `TelLane` ([sec 5](#5-cut-through-tel)) | kernel half |

No reference is formed into a region another context writes. Cross-context access is volatile load and store with compiler fences where order matters; the core has no atomic read-modify-write.

---

## 4. Budgets and the DES

### 4.1 The budget table

`firmware/lib/core/src/budget/mod.rs`, in HCLK ticks; `TICK_PERIOD` = 2400.

| Rows | Scope |
| --- | --- |
| `KERNEL`, `KERNEL_MEAN`, `TICK_PUBLISH` | plain tick body by regime (Open, Closed) and phase, maximum and mean |
| `REFRESH`, `SLOW_DUE`, `SLOW_FINISH`, `DRIVE_STAGE`, `HEALTH_CLOSE`, `CATCH_UP` | events on top of a tick |
| `TEL_ROW`, `TEL_BANK`, `TEL_HEAD`, `TEL_POLL`, `TEL_FEED`, `TEL_ARM`, `TEL_PATCH`, `TEL_LANE_MEAN` | TEL row and lane steps |
| `LAYOUT` | layout allowance on every kernel row ([sec 4.2](#42-the-layout-allowance)) |
| `ENTRY_EXIT`, `ENTRY` | interrupt entry and exit |
| `TROUGH_DONE`, `FRAME_DONE`, `NEXT_TROUGH`, `NEXT_CREST` | scan deadlines ([sec 2.3](#23-tick-sequence)) |
| `body`, `body_mean`, `frame`, `frame_mean`, `CONFIG_COUNT` | bus vector bodies and frame types at `BUDGET_BAUD_HZ` (3 Mbaud) |

Users:

- the bench probe (`budget/probe.rs`) records the same components;
- the hardware test `budgets` fails a window whose maximum or mean exceeds its row;
- the DES charges the rows as its costs;
- static pins sum the rows a tick can carry against `TICK_PERIOD` (`the_take_tick_fits_at_budget_without_tel`).

Row sources:

| Kind | Value |
| --- | --- |
| **M** (measured) | probe maximum over runs plus 3%, at least 1 us; mean plus 5% |
| **E** (estimated) | named instrument (usually the release disassembly priced with the fetch model) plus stated headroom |

### 4.2 The layout allowance

`LAYOUT` (35 HCLK) sits on every kernel row, maximum and mean, and covers register-allocation and block-placement shifts between builds and between the bench and release images.

> **The allowance counts against every hard budget:** the tick period, the turnaround deadline, every DES pin at budget and the stack reserve.

### 4.3 Row changes

> **A row changes only with its evidence, in the same change.**

- A raised M row comes from the probe on the production image.
- A raised E row names its new instrument.
- The DES pins re-run against the new table.
- The PR states old and new value, the measurement and the DES result.

A red bench window or DES pin is a finding, resolved by a root cause or a redesign, never by a row raise alone. Rows also go down when a refill measures them lower.

### 4.4 DES pins

The DES (`firmware/lib/integration`, kernel lane in `sim/cpu.rs`) runs the real bus driver and dispatch against simulated providers, with the kernel lane charged from the budget table.

> **Gating pins run at budget:** every body at its maximum with `LAYOUT` and `ENTRY_EXIT`, every injected backlog or lag at the worst the table allows.

- A pin is never moved to typical mode, given a smaller backlog or a wider tolerance to pass.
- The typical lane (bodies drawn below their maxima around the mean rows) is report-only.
- The DES must predict no less than the bench: `step_soak_at_budget_drops_at_least_the_bench`, `window_load_at_budget_is_never_below_bench`.

[testing.md](testing.md) places the DES among the three test gears.

### 4.5 Fit is an outcome

> **A design that does not fit at budget changes; the pin stays.**

Remedies: ask for less (fields, rate, batch), make the work cheaper, or change the mechanism. Examples:

- TEL rows are at most 12 bytes because a 16-row frame of 13-byte rows does not clear the 3 Mbaud wire within its ticks (const assert in `firmware/lib/core/src/tel.rs`, `twelve_byte_rows_fit_the_three_megabaud_batch`).
- A burst waits for a pending take because the take tick has no room for a row ([sec 6.3](#63-tel-waits-for-a-pending-take)).

### 4.6 Health

The tick ISR is stamped at entry and exit (`runtime/tick_load.rs`); the health block reports mean load, overruns and lost ticks ([protocol sec 5.9](osc-native-protocol.md#59-health-osc-servo)). Every build carries it.

---

## 5. Cut-through TEL

`firmware/lib/drivers/src/tel/`; wire format in [protocol sec 5.6](osc-native-protocol.md#56-telemetry-stream-tel-bursts).

| Part | File | Side |
| --- | --- | --- |
| `TelChannel` | `tel/mod.rs` | shared storage: two frame images, arm mailbox, drop count |
| `TelPost` | `tel/post.rs` | bus: arm, lend the lane, take it back |
| `TelLane` | `tel/lane.rs`, `tel/rows.rs` | kernel: rows, frame heads, CRC, TX |

> **While a burst streams, the kernel tick owns the TX lane.**

The bus lends the lane once the arming ack is out (or at the arm when no ack follows) and takes it back in its first body after the LAST frame leaves the wire. No TX interrupt and no bus body runs for the burst.

Mechanism:

- One row per tick, 16 rows per frame, each copied once to its final wire offset in one of two frame images ([sec 7.3](#73-the-two-chunk-tel-row)).
- Cut-through: a frame breaks once its lead rows have landed, and its DMA reads later rows as the wire reaches them. The lead is the frame's wire time plus `LEAD_MARGIN_TICKS` (`tel_kernel_lane_never_underruns_at_budget`).
- The lane feeds the CRC engine as rows land and patches the CRC ahead of the DMA ([sec 7.4](#74-the-crc-beside-tx)).
- Ticks lost mid-frame fail that frame's CRC and leave a sequence hole; the burst runs on to LAST (`tel_lost_ticks_mid_frame_fail_the_crc_and_leave_a_seq_hole`).

Lane rules:

- At most one step per tick, at the end of the tick: frame start, CRC feed, DMA arm or CRC patch. Each has its own budget row.
- No step on a tick that lands a head, banks a frame or runs a slow-rate stage (`tel_lane_never_steps_on_a_stage_tick`).
- The producer never blocks: a row that finds both images waiting for the wire is dropped and counted in `tel_drop_count`.

---

## 6. The configuration take

CONTROL is read field by field. CONFIG and CALIB reach the kernel only as a whole committed set, through its copy `KernelConfig` (`kernel/config.rs`).

### 6.1 The even/odd count

`Shared::config_seq` (`lib/core/src/shared.rs`) brackets every commit that reaches the span `SPAN_START..SPAN_END`:

| Call | Count | Fence |
| --- | --- | --- |
| `config_commit_begin` | odd, before the commit's first store | Release after |
| `config_commit_end` | even, after the commit's last store | Release before |

An even count names one whole committed set. The bus is the only writer; the pair is the `CONFIG_COUNT` row.

### 6.2 The take

On SPARE_A, `config_due` compares the count with the one last taken:

| Count | Action |
| --- | --- |
| even, not yet taken | `Kernel::take_config`: one word copy of the span (offsets const-asserted against the table), then the derived values |
| odd | keep the current set; check again at the next SPARE_A |
| equal to the last taken | nothing |

> **The take is a CPU copy inside one tick.** The kernel preempts the bus, so no commit store lands during the copy.

A write takes effect within one medium period, or one more when the take's tick lands inside the next commit (`config_apply_lands_within_its_bound_at_budget`, `dispatched_config_write_lands_at_the_take`). The first tick takes the configuration unconditionally.

### 6.3 TEL waits for a pending take

> **A take and a TEL row never share a tick** (`the_take_tick_fits_at_budget_without_tel`).

- A burst's first row waits while the count differs from the last taken, and never starts on SPARE_A; the check runs only at a burst start.
- The host is silent through a burst, so no take arrives mid-burst.
- A burst armed in the same COMMIT as a configuration write starts up to a medium period and a tick late (`a_burst_never_streams_through_a_config_take`).
- A take does not hold the TX lane; a slow-rate stage does.

---

## 7. DMA as a second execution unit

### 7.1 Rules

DMA runs in parallel with the CPU. On hot paths (the tick, the bus frame, the TEL burst):

- Large copies go to DMA, one arm per contiguous run; the control table may be reordered to make runs contiguous.
- After an arm, the CPU does independent work before anything waits on the result.
- Completion is polled, not interrupted, where possible.
- A copy below the arm cost stays on the CPU; DMA cycles also stall CPU fetch on the AHB (the internal bus).
- Cold paths use a plain small copy.

> **Never queue two arms on one channel.**

Kernel-side channels (full map and priority ladder in [transport sec 2](osc-servo-transport.md#2-hardware-ledger) and [transport sec 7](osc-servo-transport.md#7-the-dma-priority-ladder-and-receive-side-discipline)):

| Channel | Moves | Kernel use |
| --- | --- | --- |
| DMA1 CH1 | ADC scans to the tick block | sensing; its TC is the tick |
| DMA1 CH6 | memory to memory | TEL row's longer chunk (reply snapshots outside bursts) |
| DMA1 CH3 | halfwords to SPI1 | CRC feed for TEL frames |
| DMA1 CH4 | frame image to USART1 | TEL frame TX |

### 7.2 ADC scans land in the tick block

DMA1 CH1 writes both scans of each PWM period into the tick block, laid out in scan order so the crest scan's slots from `pos` on are the published fields ([protocol sec 5.6](osc-native-protocol.md#56-telemetry-stream-tel-bursts)). The terminal taps land in sample order, so the scans' shunt and tap slots are unnamed: the tick moves the trough samples out of the next scan's way, copies the crest's shunt, writes each tap under its terminal's name and stamps `sample_tick`. A bus read never sees a tap under the other terminal's name. Bus reads and TEL rows copy from the same block.

### 7.3 The two-chunk TEL row

`RowPlan` (`tel/rows.rs`), built by the bus at the arm:

| Chunk | Mover | Bound |
| --- | --- | --- |
| longer | DMA1 CH6, one memory-to-memory arm | at most the whole row |
| shorter | CPU, straight-line halfword stores | at most half a row, three halfwords |

The CPU copies its chunk, then arms CH6; the DMA copy runs during the medium phase and the drive. One arm per row, landed before the next tick's entry, so CH6 is idle at every arm (`the_tick_reads_its_scan_before_the_next_lands`). CH6 is the kernel's for the burst; no reply snapshots are taken while the lane is lent.

### 7.4 The CRC beside TX

SPI1 is the CRC engine ([protocol sec 3.2](osc-native-protocol.md#32-osc-crc-16)), fed by DMA1 CH3; the CPU computes no CRC on a hot path.

| Path | Feed | Result |
| --- | --- | --- |
| reply | per TX arm, during transmission | patched into the last arm before the DMA reaches it ([transport sec 4](osc-servo-transport.md#4-why-this-is-fast--first-principles)) |
| TEL frame | as rows land | patched into the image ahead of the wire ([sec 5](#5-cut-through-tel)) |
| receive | from the covered checkpoint | polled at the verdict ([transport sec 4](osc-servo-transport.md#4-why-this-is-fast--first-principles)) |

---

## 8. Code shape on a fetch-bound chip

The facts behind these rules, the layout traps and how to measure a change are in [v006-performance.md](v006-performance.md).

### 8.1 The fetch model

Flash is fetched one aligned 4-byte word per 3 HCLK at 48 MHz, with no cache. Every taken branch, call and return restarts the fetch; a branch not taken costs its bytes. Budget E rows are priced with this model.

### 8.2 Lean code

> **Code bytes and taken branches are execution time.**

- Delete before adding: fewer conditionals and special cases, one general path.
- The hot path is the fall-through side of its branches; small hot bodies are branchless.
- Small hot helpers are inlined.
- Constants are folded at compile time (`KernelTiming`); the kernel never divides at run time.
- A provably redundant check on a hot path may be removed with its safety argument and a test pinning the invariant. Firmware never panics.
- No dynamic dispatch and no run-time-costly layers on hot paths.

### 8.3 Hot function alignment

> **Every out-of-line hot function carries `#[rustc_align(4)]` and is listed in `scripts/check-hot-align.sh`.**

Hot means the tick interrupt and its callees, the bus vector bodies, the bus frame path and the TEL lane. The pin applies whether or not the function is aligned today; new hot code adds the attribute and the script entry in the same change. The board app builds with `-align-all-nofallthru-blocks=2` (the `osc-dev-v006-app` profile in `firmware/boards/osc-dev-v006/Cargo.toml`), which does not cover function entries. The four interrupt bodies share one `.trap.rust` section and each carries its own pin.

### 8.4 Guards

| Rule | Guard |
| --- | --- |
| Hot entries at 0 mod 4 | `scripts/check-hot-align.sh`, CI, release and bench images |
| No soft div, mod or mul, no 64-bit arithmetic | `.github/check-soft-arith.sh`, CI |
| No `memcpy`, `memset`, `memcmp`, `memmove` on runtime paths; bringup statics only, through `firmware/servo-ch32/src/runtime/mem.rs` | `.github/check-no-mem.sh`, CI, release and bench images |
| No code in RAM | review |

Replacements for `mem*` builtins:

- small fixed-size copies: typed word loads and stores, or an inlined copy not lowered to a call;
- zero fills fully overwritten before use: `MaybeUninit`;
- the rest: a dedicated routine sized for its alignment and length, pinned as a hot function;
- implicit calls (struct moves, array zeroing, `copy_from_slice`, `fill`, `==` on arrays) count too.

### 8.5 Nothing runs from RAM

> **All code runs from flash.** RAM is reserved for data and stack; a slow body is made smaller.

---

## 9. Trade-offs to accept

- **Elastic replies.** Reply latency follows kernel load; a turnaround measured at rest does not hold while moving ([transport sec 10](osc-servo-transport.md#10-measured-turnarounds)).
- **Tickless windows.** A shunt burst runs no tick ([control-theory.md, What It Does Not Cover](control-theory.md#what-it-does-not-cover)); a flash SAVE or debug halt stalls the core. SAVE and halts count in `tick_lost_count`; the burst counts nowhere.

---

## 10. Summary

1. **One tick per PWM period, started by hardware.** The tick index is the only clock; lost ticks are counted, never compensated.
2. **The schedule is a static table.** Three rates, ten medium phases in data-flow order, a fixed slot for every rare event.
3. **The kernel is above the bus.** The kernel never waits on the bus; bus latency stretches instead.
4. **Every body has a budget row.** The bench holds the chip to it; the DES charges it with every gating pin at budget, layout allowance included.
5. **Rows move only on evidence; fit is an outcome.** A raise needs a production probe and a DES re-run in the same change.
6. **The kernel owns the TX lane during a TEL burst.** Rows are cut-through, one lane step per tick at most, and the producer never blocks.
7. **Configuration reaches the tick whole.** `config_seq` brackets each commit, the take runs at SPARE_A, and a burst never starts while a take is pending.
8. **DMA is a second execution unit.** One arm per channel at a time; the CPU works beside each arm.
9. **Code size is execution time.** Hot entries at 0 mod 4, no soft arithmetic or `mem*` builtins on runtime paths, no code in RAM.

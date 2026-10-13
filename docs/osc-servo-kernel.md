# Kernel and Scheduling Architecture

The scheduling architecture of the osc servo firmware: how one CH32V006 core (48 MHz, no cache, no hardware divide, a few KB of RAM) is shared between the 20 kHz motor kernel, the bus, the telemetry stream and the main loop, and why each piece runs where it does. The kernel lives in `firmware/lib/core/src/kernel/`; the chip side that binds it to the interrupts lives in `firmware/servo-ch32/src/runtime/`. Companion pillars: [osc-native-protocol.md](osc-native-protocol.md) (the wire), [osc-servo-transport.md](osc-servo-transport.md) (the receiver), [control-theory.md](control-theory.md) (the loops) and [driver-pattern.md](driver-pattern.md) (the layering).

Code is authoritative; when this doc and the code disagree, fix the doc.

---

## 1. Constraints

Every PWM period, 50 us, the servo takes the sample the ADC has just made, computes the motor command and writes the PWM before the next sample. Around that hard deadline sit the slower work (the velocity and position loops, the observer, the limits, the winding thermometer), the bus (frame service, replies, telemetry bursts) and housekeeping. All of it shares one core.

Three properties of the V006 constrain every decision in this doc:

- **One core, small RAM.** Every context shares the CPU. RAM is the first resource to run out, and the stack is the part of RAM that runs out ([v006-feature-budget.md](v006-feature-budget.md#budget-lines)).
- **Fetch-bound flash.** The core reads flash one 4-byte word at a time, three HCLK (core clock cycles) per word at 48 MHz, with no cache. Every taken branch, call and return restarts the fetch. An instruction costs roughly its fetch ([sec 8](#8-code-shape-on-a-fetch-bound-chip)).
- **No divide.** The core has a hardware multiply and no divide. Every divisor on a runtime path is a compile-time constant.

The design therefore treats CPU time as the scarce resource. The motor deadline is protected by interrupt priority ([sec 3](#3-the-kernel-above-the-bus)), every body is budgeted and simulated at its worst ([sec 4](#4-budgets-and-the-des)), data movement is handed to DMA ([sec 7](#7-dma-as-a-second-execution-unit)), and code size is treated as execution time ([sec 8](#8-code-shape-on-a-fetch-bound-chip)).

---

## 2. Static phased scheduling

### 2.1 One tick per PWM period

TIM1 drives the motor with center-aligned PWM at 20 kHz (`TICK_HZ`, `kernel/scheduler/table.rs`). Each period it triggers two ADC scans, one at the trough of the counter and one at the crest, and DMA1 CH1 lands both in the control table's tick block ([sec 7.2](#72-adc-scans-land-in-the-tick-block)). The DMA transfer-complete (TC) interrupt after the crest scan is the kernel tick.

The tick is hardware-driven end to end: TIM1, ADC, DMA, interrupt. No software timer starts or re-arms it, so it cannot drift against the PWM.

> **The tick index is the kernel's only clock.**

`sample_tick` counts executed ticks. Each executed tick advances every phase, filter, integrator and timer by exactly one period, and nothing in the kernel reads a wall clock. A tick that does not run is counted in `tick_lost_count` ([protocol sec 5.9](osc-native-protocol.md#59-health-osc-servo)) and is not compensated. With the kernel above the bus ([sec 3](#3-the-kernel-above-the-bus)), only a body that overruns two periods, a flash SAVE (torque off) or a debug halt loses a tick. A shunt burst borrows the ADC and runs no kernel tick for its window ([protocol sec 5.8](osc-native-protocol.md#58-limits-and-the-stall-permit-osc-servo)).

### 2.2 Three rates, ten phases

The kernel derives three rates from the one tick:

| Rate | Period | Work |
| --- | --- | --- |
| **FAST** | every tick (20 kHz) | measurement of the window the last command drove, the overcurrent trip, the back-EMF sums, the current loop, the motor write (`kernel/phases/fast.rs`) |
| **MEDIUM** | `DECIM_MED` = 10 ticks (2 kHz) | the outer loops, the estimators, the limits, the detectors and the publishes, cut into ten phases, one per tick |
| **SLOW** | `DECIM_SLOW` = 32 medium periods (62.5 Hz) | the stall permit lease, undervoltage and the winding thermometer, run inside the medium phases in stages, one stage per period |

The medium phases run in data-flow order, so each phase reads what the phases before it wrote in the same period:

| Phase | Name | Work |
| --- | --- | --- |
| 0 | CONTROL | the CONTROL read, the enable and mode edges, the drive kind; the configuration on the first tick |
| 1 | OBSERVER | back-EMF close, observer step, speed source switch |
| 2 | TRAJECTORY | trajectory and position loop; the hold updates the drive kind |
| 3 | LIMITS | limits fold, OpenLoop base duty, stall fault |
| 4 | VELOCITY | velocity loop or the current clamp; the current reference and band land |
| 5 | RAIL | rail estimate, sensor screen, position error timer |
| 6 | SLOW | the slow block when due; the thermometer's finish one period later |
| 7 | PUBLISH | estimates, mode, limits and sensors publish; the tick-load close |
| 8 | SPARE_A | the configuration take after a commit; otherwise a thermometer install |
| 9 | SPARE_B | the NTC (board thermistor) reading before a SLOW tick; the derate after a finish |

The schedule is stated once, in `kernel/scheduler/table.rs`, and the module tree mirrors it: `kernel/scheduler/` holds the tick and the table, `kernel/phases/` holds one file per phase, and `kernel/phases/loops/` holds the loops and detectors the phases run. The tree is meant to show the design; structure is admitted where it costs nothing at run time.

The two halves exchange two small structs. The medium half leaves the fast half a `Command` (the drive kind, the current reference and band, the rail clamp); the fast measurement reaches the same tick's medium phase as `Measured`. Only the CONTROL phase reads the host's CONTROL registers, so a host write lands at the next period's CONTROL phase. The delay from a sample to the command computed from it is fixed by the table, and the loop design depends on that constant ([control-theory.md, Why a Cascade](control-theory.md#why-a-cascade)).

### 2.3 Inside one tick

A tick runs in this order (`Kernel::on_tick`, called from `runtime::isr::on_adc_dma_tc`):

1. Count the tick in `sample_tick` and name the new scan in place in the tick block.
2. Spare-phase arbitration. The first tick loads the configuration. After that, SPARE_A runs the configuration take when one is due ([sec 6](#6-the-configuration-take)); a spare phase with no take runs a pending thermometer stage.
3. FAST measure. The window the previous command drove becomes this tick's current, voltage and flags, published in the tick block.
4. The TEL (telemetry stream) row, while a burst streams ([sec 5](#5-cut-through-tel)), copied while the tick block still holds this tick's scan.
5. This tick's medium phase.
6. FAST drive: the current loop (or open loop, brake or off) and the motor write.
7. At most one step of the TEL lane.
8. In the interrupt wrapper: the shunt-burst handshake poll and the tick-load accounting.

The order is the data flow. No stage reads the sample before it is measured, the row is copied before the block is overwritten, and the motor write never waits behind the lane.

The ADC keeps sampling while the tick runs. The next trough scan lands about halfway through the period and the next crest scan near its end, both in the same tick block. The tick therefore moves the trough's few samples out of the way early, and completes every read of the crest scan before the next one lands. The budget table carries both deadlines as rows (`TROUGH_DONE`, `FRAME_DONE`), and the DES (discrete-event simulator, [sec 4](#4-budgets-and-the-des)) pin `the_tick_reads_its_scan_before_the_next_lands` holds the most expensive tick to them.

### 2.4 Rare work gets a fixed slot

The periodic work is fixed by the table. The worst tick comes from rare events landing together: a configuration take, a slow-rate stage, a telemetry frame boundary, the tick-load close. Each rare event has a fixed home, so two meet on one tick only where the budget prices that combination:

- The configuration take owns the first SPARE_A after a commit ([sec 6](#6-the-configuration-take)).
- The slow block runs in stages one period apart, on the SLOW phase and the spare phases. A stage whose phase is taken waits a period on its pending bit.
- The tick-load close lands on PUBLISH, the lightest Closed phase that carries no stage and no take (`health_close_lands_only_on_publish_while_streaming`).
- The TEL lane takes at most one step per tick, and none on a tick that lands a frame head, banks a frame or runs a slow-rate stage.

Arbitration is a fixed priority in the first `match` of `on_tick`: boot, then the take, then the thermometer. There is no queue.

### 2.5 Why static

A static table is chosen over a dynamic scheduler for reasons specific to this chip:

- **The work is known in advance.** Three fixed rates in data-flow order and a short list of rare events. Nothing arrives at random and needs ranking.
- **Dispatch is one `match` on a phase counter.** A queue, deadlines or priority comparisons would add instructions and taken branches to every tick, which on this chip are the dominant cost.
- **Every tick can be priced.** A tick's worst case is its phase row plus the events allowed on that phase, so the budget table and the DES bound every tick. A dynamic scheduler's worst case depends on its run-time choices and is harder to bound and to pin.
- **Reordering creates no CPU time.** When a tick does not fit, a dynamic queue can only rank the same work differently. The remedy is to delete work, make it cheaper or move it to a phase with room; a static table makes that move explicit and reviewable.
- **One owner.** The tick is the only context that touches kernel state, so the kernel holds no locks. An RTOS (real-time operating system) would split that ownership and give each task a stack, on the chip where RAM is scarcest.
- **Fixed loop timing.** Every phase runs at a fixed offset from its sample, so the sample-to-command delay is a constant of the control design.

### 2.6 The main loop

Work with no deadline inside a tick runs in the main loop: the status LED, publishing the link counters and the dropped TEL rows, applying the clock trim, the shunt-burst page copy, the data job (a software CRC pass longer than any bus body may run), the deferred reboot and the rescue-pulse sampler. The loop then sleeps in `wfi` until the next interrupt.

Its reach-ins into bus state mask the bus level only (`pfic::mask_bus`). The main loop can hold off the bus briefly; it never holds off the kernel.

---

## 3. The kernel above the bus

### 3.1 Priority levels

The V006 PFIC (programmable fast interrupt controller) has two preemption levels. The kernel holds the upper one alone (`runtime::isr::install_irqs`):

| Vector | Level | Job |
| --- | --- | --- |
| DMA1 CH1 TC | HIGH | the kernel tick; the TEL lane while a burst streams |
| TIM2 | LOW | the bus break wake ([transport sec 7](osc-servo-transport.md#7-the-dma-priority-ladder-and-receive-side-discipline)) |
| USART1 | LOW | a TX arm drained |
| SysTick | LOW, taken last | the bus deadline mux, with dispatch inline |
| main loop | none (thread mode) | cold work; masks LOW only |

The bus vectors share LOW, so they never preempt each other and the bus driver needs no locks. SysTick is taken last within LOW, so a pending wake or TX completion resolves its frames before a pending deadline acts on them. The vector bodies are thin bindings in the sense of [driver-pattern sec 8](driver-pattern.md#8-hardware-demultiplexing). The kernel preempts all of them and nothing preempts the kernel, so a tick enters at its scan, held off only by the longest critical section.

> **The kernel never waits on the bus.**

### 3.2 Why the kernel is on top

One bus body can run longer than a tick period, and one host frame's bus work always does (the `Deadline` body and every `frame` row in the budget table exceed `TICK_PERIOD`). With the bus above the kernel, such a body covers the tick, the scan's completion flag merges with the next one, and the tick is lost. Each host frame then costs one or two ticks, and because the tick index is the kernel's clock, everything timed in ticks runs slow while the host polls. The DES holds that model against the bench with `kernel_lane_below_bus_reproduces_bench_loss`.

With the kernel on top, the bus is a background server. It receives the time the kernel leaves, so each microsecond of bus work costs 1 / (1 - U) microseconds of latency, U being the kernel's share of the CPU. A reply may pause between its arms for about one kernel body. The host's RESPONSE_DEADLINE ([protocol sec 7](osc-native-protocol.md#7-timing-rules)) covers the stretched path (`reply_bound_covers_turnaround_under_kernel_preemption`), and the kernel loses no tick to traffic (`kernel_on_top_loses_no_tick_under_polling`).

The motor deadline is hard and recurs every 50 us; the bus deadline is soft and measured in milliseconds. Under contention the bus yields.

Preemption creates hazards on the bus side, and the bus code closes them ([transport sec 2](osc-servo-transport.md#2-hardware-ledger)): each TX arm clears TC before its DMA starts, the frame projections read the ring cursor before the clock, and a configuration commit is bracketed by the count the kernel takes under ([sec 6](#6-the-configuration-take)).

### 3.3 Single-writer contracts

The priority split holds because every shared field has exactly one writer:

- **The bus** is the only writer of CONTROL, CONFIG, CALIB and the position table. The kernel only reads them, with volatile reads through `region_ptr`, and never forms a reference into them. CONTROL is read once per period at the CONTROL phase; a torn read across fields is accepted because each field is valid on its own. CONFIG and CALIB need whole-set consistency and arrive through the take ([sec 6](#6-the-configuration-take)).
- **The kernel** is the only writer of the tick block (after the DMA and the interrupt land the scan and the stamp), the estimates, the references, the mode, the limits and the fault byte.
- **The chip side** (the interrupt wrapper and the main loop) writes the health block.
- **The TEL channel** splits its fields between a bus half and a kernel half under the same rule ([sec 5](#5-cut-through-tel)).

The core is single and has no atomic read-modify-write, so these contracts are plain volatile loads and stores, with compiler fences where order matters.

---

## 4. Budgets and the DES

### 4.1 The budget table

`firmware/lib/core/src/budget/mod.rs` holds the most CPU each component may take, in HCLK ticks at 48 MHz. A tick period is 2400 of them (`TICK_PERIOD`). The table carries:

- kernel tick bodies by drive regime (Open or Closed) and phase, maximum and mean;
- one row per event a tick can carry on top: the take, the slow stages, the TEL row, head, bank and lane steps, the tick-load close;
- interrupt entry and exit;
- the bus vector bodies, and the bus CPU of each frame type at 3 Mbaud.

The table has three users. The bench probe records the same components on the chip. The hardware test `budgets` fails any window whose maximum or mean exceeds its row. The DES charges these values as its costs. While `budgets` is green, the DES runs no component cheaper than silicon. The rows also compose statically: a pin such as `the_take_tick_fits_at_budget_without_tel` sums the rows a tick can carry and checks the sum against the period.

Every row states its source. An **M** row is measured by the budget probe: the maximum is the largest of its runs plus 3%, at least 1 us of margin, and the mean is the measured mean plus 5%. An **E** row is estimated and names its instrument (usually the release disassembly priced with the fetch model) and its headroom.

### 4.2 The layout allowance

One source compiles to tick bodies that differ in register allocation and block placement between builds, and between the bench image the rows are measured on and the release image. The difference is real silicon time. Every kernel row, maximum and mean, therefore carries `LAYOUT` (35 HCLK) on top, and routine layout shifts pass with no row edit.

The allowance counts against every hard budget. Fit is judged with every row at its maximum plus the allowance, and the DES charges it the same way. The allowance never justifies exceeding the tick period, the turnaround deadline, a DES pin at budget or the stack reserve.

### 4.3 Rows move only on evidence

Raising a row relaxes the bench test and every DES pin at once, which makes it the cheapest way to fake a pass.

> **A row changes only with its evidence, in the same change.**

- A raised M row comes from the probe on the production image; bench images carry the probe's own overhead.
- A raised E row names its new instrument.
- The DES pins re-run against the new table.
- The PR states the old and new value, the measurement and the DES result.

A red bench window or a red pin is a finding. It is resolved by a root cause or a redesign, never by raising a row alone. Rows also go down when a refill measures them lower, so the table tracks the code instead of ratcheting upward.

### 4.4 DES pins run at budget

The DES in `firmware/lib/integration` runs the real bus driver and dispatch against simulated providers, with a kernel lane that preempts the bus at the table's costs (`sim/cpu.rs`). Its timing pins run at budget: every body at its maximum, with the layout allowance and interrupt entry and exit, and every injected backlog or lag at the worst the table allows. A pin is never moved to a typical mode or given a smaller backlog to pass. The typical lane (bodies drawn below their maxima around the mean rows) serves forecasts and gates nothing. [testing.md](testing.md) places the DES among the three test gears.

The DES is also held pessimistic against the bench. `step_soak_at_budget_drops_at_least_the_bench` and `window_load_at_budget_is_never_below_bench` fail when the simulator predicts less loss or less load than the bench measured. A DES result of zero loss is necessary; only the bench gate makes it sufficient.

### 4.5 Fit is an outcome

> **When a design does not fit at budget, the design changes and the pin stays.**

The remedies are to ask for less (fewer fields, a lower rate, a smaller batch), to make the work cheaper, or to find another mechanism.

The TEL row size is an instance. A frame of 16 rows must clear the 3 Mbaud wire within its 16 ticks, less the ticks the lane itself needs. A const assert in `firmware/lib/core/src/tel.rs` checks that 12-byte rows fit, and `twelve_byte_rows_fit_the_three_megabaud_batch` checks that 13-byte rows do not. The row maximum is 12 bytes because that is what fits, and the arm refuses anything larger ([protocol sec 5.6](osc-native-protocol.md#56-telemetry-stream-tel-bursts)).

The configuration take is another. Its tick has no room for a TEL row on top, so a burst waits for the take ([sec 6.3](#63-tel-waits-for-a-pending-take)) rather than the table assuming both fit.

### 4.6 Health on the chip

Every build measures itself. The tick interrupt is stamped at entry and exit, and the health block reports the mean tick load, the ticks over one period and the ticks lost ([protocol sec 5.9](osc-native-protocol.md#59-health-osc-servo)). With the kernel on top, bus traffic does not enter these counts; any count is the kernel's own.

---

## 5. Cut-through TEL

A TEL burst ([protocol sec 5.6](osc-native-protocol.md#56-telemetry-stream-tel-bursts)) is a bounded run of status frames the servo emits on its own schedule, 16 rows per frame and one row per tick. While a burst streams, the kernel tick is the transmitter. The bus lends the TX lane to the kernel once the arming ack is out (or at the arm, when no ack follows) and takes it back in its first body after the LAST frame has left the wire. No TX interrupt and no bus body runs for the burst. The code is `firmware/lib/drivers/src/tel/`.

The burst is cut-through: rows reach the wire as they are produced, with no stage-then-send step.

- Each row is copied once, to its final wire offset in one of two frame images ([sec 7.3](#73-the-two-chunk-tel-row)).
- A frame breaks onto the wire once enough of its rows have landed, and its DMA reads the rows behind them as the wire reaches them. The lead is set per frame from its wire time plus a margin of ticks (`LEAD_MARGIN_TICKS`), so the wire never overtakes the producer at budget (`tel_kernel_lane_never_underruns_at_budget`).
- The producer feeds the CRC engine as rows land, and the CRC is patched into the image before the DMA reaches it ([sec 7.4](#74-the-crc-beside-tx)).
- Ticks lost mid-frame let the DMA read rows that have not landed, and the frame fails its CRC. The host sees a sequence hole and the burst runs on to LAST (`tel_lost_ticks_mid_frame_fail_the_crc_and_leave_a_seq_hole`). A late frame never decodes as plausible data.

The lane is part of the static schedule ([sec 2.4](#24-rare-work-gets-a-fixed-slot)). It runs at the end of the tick and takes at most one step: a frame start, a CRC feed, the DMA arm or the CRC patch. It takes none on a tick that lands a frame head, banks a frame or runs a slow-rate stage (`tel_lane_never_steps_on_a_stage_tick`). Each kind of step has its own budget row.

The producer never blocks the tick. A row that finds both images still waiting for the wire is dropped and counted in `tel_drop_count`.

---

## 6. The configuration take

CONTROL can be read field by field because each field is valid on its own. CONFIG and CALIB cannot: a set of gains, a limit band or a thermal model is meaningful only whole, and a host often writes several fields in one commit. The kernel therefore never reads CONFIG or CALIB inside its loops. It works from its own copy, `KernelConfig` (`kernel/config.rs`), and replaces that copy whole, between commits.

### 6.1 The even/odd count

`Shared::config_seq` is a count the bus steps around every commit that reaches the kernel's inputs, the span `SPAN_START..SPAN_END`. The count goes odd before the commit's first store and even after its last, with compiler fences on both sides, so an even value always names one whole committed set. The bus is the count's only writer; the pair costs a commit a few instructions (the `CONFIG_COUNT` row).

### 6.2 The take at a fixed phase

On SPARE_A the kernel compares the count with the one it last took. If the count is even and new, the kernel takes the configuration on that tick (`Kernel::take_config`): one word copy of the span, laid out block for block as the table lays it out (const asserts check every offset), followed by the few values derived from it. If the count is odd, a commit is in progress; the kernel keeps its current set and checks again at the next period's SPARE_A.

The copy is consistent because the kernel preempts the bus: no commit store can land while the take runs, so an even count read at its start still holds at its end. The same argument requires the take to be a CPU copy inside the tick. A DMA copy would still be reading after the tick returns, and a commit could begin under it.

A configuration write takes effect within one medium period, or one more when the take's tick lands inside the next commit (`config_apply_lands_within_its_bound_at_budget`, `dispatched_config_write_lands_at_the_take`). The first tick takes the configuration with no write.

### 6.3 TEL waits for a pending take

The take's tick has no room for a TEL row on top (`the_take_tick_fits_at_budget_without_tel`), so the two never share a tick. A burst's first row waits while a committed write is not yet taken, and never starts on SPARE_A. The kernel evaluates this only when a burst is about to start, since a count read on every tick would cost every tick. Once a burst streams the host is silent, so no new take can arrive under it.

A burst armed in the same COMMIT as a configuration write streams its first row up to a medium period and a tick late (`a_burst_never_streams_through_a_config_take`, [protocol sec 5.6](osc-native-protocol.md#56-telemetry-stream-tel-bursts)).

A take does not hold the TX lane the way a slow-rate stage does: a take can run every period, and holding the lane on each would stall the stream.

---

## 7. DMA as a second execution unit

### 7.1 Rules

The DMA controller is treated as a second execution unit that runs in parallel with the CPU. On the hot paths (the tick, the bus frame and the TEL burst) the following rules apply:

- **Large copies go to DMA.** Data is laid out as contiguous runs (the control table may be reordered for it), and each run is copied by one DMA arm, fire and forget.
- **Overlap.** After an arm, the CPU does independent work before anything waits on the result. The gain is two movers working at once.
- **Poll, do not interrupt.** Completion is polled where possible instead of taking an interrupt.
- **Price the net saving.** DMA cycles on the AHB (the chip's internal bus) stall CPU fetch, and arming a channel costs instructions, so a copy below the arm cost stays on the CPU.
- **Cold paths use a plain small copy.** The arm cost and the code size do not pay off there.

> **Never queue two arms on one channel.** The second only waits for the first, and the parallelism is lost.

The channel map, and the DMA priority ladder that protects the RX ring under this traffic, are in [transport sec 2](osc-servo-transport.md#2-hardware-ledger) and [transport sec 7](osc-servo-transport.md#7-the-dma-priority-ladder-and-receive-side-discipline).

### 7.2 ADC scans land in the tick block

DMA1 CH1 writes both ADC scans of every PWM period directly into the control table's tick block. The table is laid out in scan order, so the crest scan's slots are the published fields themselves ([protocol sec 5.6](osc-native-protocol.md#56-telemetry-stream-tel-bursts)). The tick interrupt copies no sensor frame: it names the terminal taps in place, moves the trough's few samples out of the next scan's way and stamps `sample_tick`, and the kernel reads the rest where it landed. Bus reads and TEL rows copy from the same block.

While the CPU runs the tick, the ADC and DMA are already taking the next period's scans. This is the origin of the two deadlines in [sec 2.3](#23-inside-one-tick).

### 7.3 The two-chunk TEL row

A TEL row is up to two chunks of the table ([protocol sec 5.6](osc-native-protocol.md#56-telemetry-stream-tel-bursts)), so each row needs two copies, split between the two movers. The plan (`RowPlan`, `tel/rows.rs`) is built once at the arm, off the kernel's tick. The longer chunk goes to DMA1 CH6 as one memory-to-memory copy; the CPU copies the shorter chunk. The shorter chunk is at most half a row, three halfwords, which costs less as straight-line stores than a channel arm.

The CPU copies its chunk first and then arms the DMA, so the DMA copy runs while the CPU continues with the medium phase and the drive. Each row arms CH6 exactly once, and that copy lands before the next tick enters, so no arm ever queues behind another on the channel (`the_tick_reads_its_scan_before_the_next_lands`). CH6 belongs to the kernel for the whole burst, since no reply snapshots are taken while the lane is lent.

### 7.4 The CRC beside TX

SPI1 is the CRC engine ([protocol sec 3.2](osc-native-protocol.md#32-osc-crc-16)). DMA1 CH3 feeds it halfwords while other channels move the bytes, so the CPU computes no CRC on a hot path:

- **Reply:** the TX CRC is computed in parallel with the transmission and patched into the last arm before the DMA reaches it ([transport sec 4](osc-servo-transport.md#4-why-this-is-fast--first-principles)).
- **TEL frame:** the lane feeds the engine as rows land and patches the result into the image ahead of the wire ([sec 5](#5-cut-through-tel)).
- **Receive:** the feed starts at the covered checkpoint while the CRC bytes are still on the wire, and the verdict only polls a finished result ([transport sec 4](osc-servo-transport.md#4-why-this-is-fast--first-principles)).

---

## 8. Code shape on a fetch-bound chip

### 8.1 The fetch model

The V006 reads flash one aligned 4-byte word at a time, three HCLK per word at 48 MHz, with no cache. Every taken branch, call and return discards the prefetch and restarts; a branch not taken costs only its bytes. The budget table's E rows are priced with this model.

On this chip an instruction costs roughly its fetch. Fewer instruction bytes and fewer taken branches shorten a body far more than cheaper arithmetic does.

### 8.2 Lean code is speed

Because code bytes are execution time, deletion is the first tool: fewer conditionals, fewer special cases, one general path in place of several special ones. A fix that adds a branch to the tick is suspect until no deletion can make the branch unnecessary. The working rules:

- The hot path sits on the fall-through side of its branches; small hot bodies are branchless.
- Small hot helpers are inlined, since a call and its return each restart the fetch.
- Arithmetic is folded at compile time. The chip const-evaluates its timing constants (`KernelTiming`), so the kernel never divides at run time.
- A check that is provably redundant on a hot path may be removed, each removal with its safety argument and a test that pins the invariant. The firmware never panics.
- Structure is admitted only at zero run-time cost: no dynamic dispatch on hot paths, no layers that add instructions.

### 8.3 Hot functions start on a 4-byte boundary

A function entered at 2 mod 4 spends an extra fetch on every call. Every out-of-line function on a hot path (the tick interrupt and its callees, the bus vector bodies, the bus frame path, the TEL lane) therefore carries `#[rustc_align(4)]`, and `scripts/check-hot-align.sh` fails CI if any of them links at 2 mod 4 or disappears from the image. CI runs the check on both the release and the bench image.

The pin is applied whether or not the function is aligned today. Layout moves with every change; the pin keeps later changes out of the same trap. New hot code adds the attribute and the script entry in the same change.

The board builds with `-align-all-nofallthru-blocks=2` (`firmware/boards/osc-dev-v006/.cargo/config.toml`), which places every block not reached by fall-through on a 4-byte boundary. The flag does not cover function entries; the pins do. The four interrupt bodies share one `.trap.rust` input section, so each carries its own pin.

### 8.4 No soft arithmetic, no mem* builtins

The core has a multiply and no divide. A runtime `/` or `%`, or any 64-bit multiply or shift, pulls in a compiler-builtins routine that costs hundreds of cycles and flash. `.github/check-soft-arith.sh` fails CI if the image links any of them; divisors stay compile-time constants.

The same reasoning applies to `memcpy`, `memset`, `memcmp` and `memmove`: a builtin costs the call, a generic dispatch on alignment and length, and code the 4-byte pin cannot reach. Runtime paths must not call them. The replacements:

- a small fixed-size copy becomes typed word loads and stores, or an inlined copy the compiler does not lower to a call;
- `MaybeUninit` replaces a zero fill whose every byte is written before it is read;
- anything remaining gets its own routine, sized for its alignment and length and pinned like any hot function;
- implicit calls are removed too: struct moves, array zeroing, `copy_from_slice`, `fill` and `==` on arrays.

No CI check covers this rule; review enforces it ([sec 9](#9-trade-offs-to-accept)).

### 8.5 Nothing runs from RAM

Running hot code from RAM is the usual remedy for slow flash. On the V006, RAM is the first resource to run out, and the stack, the frame buffers and the control table already share it. Nothing runs from RAM; the remedy for a slow body is to make it smaller.

---

## 9. Trade-offs to accept

- **The thermometer exception.** The winding thermometer's stage rows report and do not gate fit, and the streaming case of `thermal_close_fits_at_budget_while_seated` is ignored, because those stages exceed the period at budget. The budget table's header names the exception so it does not spread.
- **Remaining mem* calls.** The bus frame path still calls a few `mem*` builtins (in dispatch, frame routing, the reply gather and the position table commit), and no CI check lists them.
- **Elastic replies.** With the kernel on top, a reply's latency depends on the kernel's load. RESPONSE_DEADLINE covers it, but a turnaround measured with the motor at rest does not hold while the motor moves ([transport sec 10](osc-servo-transport.md#10-measured-turnarounds)).
- **Tickless windows.** A shunt burst runs no kernel tick for its window ([control-theory.md, What It Does Not Cover](control-theory.md#what-it-does-not-cover)). A flash SAVE (torque off) or a debug halt stalls the whole core. SAVE and halts count in `tick_lost_count`; the burst counts nowhere, by design.

---

## 10. Summary

Nine rules:

1. **One tick per PWM period, started by hardware.** The tick index is the kernel's only clock; a lost tick is counted, never compensated.
2. **The schedule is a static table.** Three rates, ten medium phases in data-flow order, and a fixed slot for every rare event.
3. **The kernel is above the bus.** The bus is a background server whose latency stretches by 1 / (1 - U); the kernel never waits on it.
4. **Every body has a budget row.** The bench holds the chip to the table, and the DES charges it with every pin at budget, layout allowance included.
5. **Rows move only on evidence, and fit is an outcome.** A raise needs a production probe and a DES re-run in the same change; a design that does not fit changes, and the pin stays.
6. **TEL is cut-through and owned by the kernel.** Rows go to the wire as they are produced, one lane step per tick at most, and the producer never blocks.
7. **Configuration reaches the tick whole.** An even/odd count brackets every commit, the take runs at SPARE_A, and a burst never starts while a take is pending.
8. **DMA is a second execution unit.** Scans land in the table, the longer row chunk moves by DMA beside the CPU's work, and SPI computes the CRC beside TX; never two arms on one channel.
9. **Code size is execution time.** Hot functions start on a 4-byte boundary, no soft arithmetic or mem* builtins run on runtime paths, and nothing runs from RAM.

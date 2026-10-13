# osc servo kernel and scheduling

The servo runs on a CH32V006. It is a RISC-V chip under $0.20 with one core at 48 MHz, no cache, no hardware divide and very little RAM. On that one core the firmware reads the sensors, closes the current loop and writes the motor 20,000 times a second. Between those writes it runs the slower loops and estimators, answers the bus, and streams telemetry at 3 Mbaud. This doc is how the firmware shares the core between those jobs, and why it is built that way.

The wire is in `osc-native-protocol.md`, the receiver in `osc-servo-transport.md`, the loops in `control-theory.md`, and the code layering in `driver-pattern.md`. The code is the source of truth. When this doc and the code disagree, fix the doc.

If you only have a minute, here are the rules:

- One kernel tick per PWM period, 20 kHz, started by hardware. The schedule is a fixed table of ten phases, and each rare event has a fixed slot in it.
- The kernel interrupt sits above every bus interrupt. The bus gets the time the kernel leaves, so replies stretch and the kernel loses no tick.
- Every body has a row in one budget table. The bench holds the chip to it and the simulator charges it, every pin at budget. A row goes up only with a production measurement and a simulator re-run in the same change. When something does not fit, the design changes and the pin stays.
- Configuration reaches the tick whole, through an even/odd count and one take at a fixed phase. A telemetry burst never starts while a take is pending.
- DMA is a second worker beside the CPU. Scans land in the table, row chunks are copied, and CRCs are computed while the CPU does other work.
- Flash fetch is the bottleneck, so small code is fast code. Hot functions start on a 4-byte boundary, CI checks it, and nothing runs from RAM.

## 1. The job and the chip

Every 50 us the servo does the same thing. It takes the sample the ADC just made, decides what the motor should get, and writes the PWM before the next sample. Around that core sit the slower loops (velocity, position, the observer, the limits, the winding thermometer), the bus with its replies and telemetry bursts, and some housekeeping.

Three facts about the V006 shape every decision in this doc:

- **One core, small RAM.** Every context shares the one CPU. RAM is the resource that runs out first, and the stack is the part of it that runs out (`v006-feature-budget.md`).
- **Fetch-bound flash.** The core reads flash one 4-byte word at a time, three HCLK (core clock cycles) per word at 48 MHz, with no cache. Every taken branch, call and return starts the fetch over. With the core that fast and the flash that slow, an instruction costs about what it takes to fetch it (sec 8).
- **No divide.** The core has a hardware multiply and no divide, so every divisor on a runtime path is a compile-time constant.

The kernel is the code that runs on the tick. It lives in `firmware/lib/core/src/kernel/`, and the chip side that wires it to the interrupts lives in `firmware/servo-ch32/src/runtime/`.

## 2. Static phased scheduling

### 2.1 One tick per PWM period

TIM1 drives the motor with center-aligned PWM at 20 kHz (`TICK_HZ`). Every period it triggers two ADC scans, one at the trough of the counter and one at the crest. DMA1 channel 1 lands both scans straight in the control table's tick block (sec 7.2). The DMA's TC (transfer complete) interrupt after the crest scan is the kernel tick.

No software starts a tick or re-arms it. The chain is TIM1, ADC, DMA and the interrupt, all hardware, so the tick cannot drift against the PWM.

The tick index is the kernel's only clock. `sample_tick` counts the ticks that ran, every tick that runs advances everything by exactly one period, and nothing in the kernel reads a wall clock. A tick that does not run is counted in `tick_lost_count` (protocol sec 5.9), and nothing stretches to cover it. With the kernel above the bus (sec 3), only a body that overruns two periods, a flash SAVE with torque off, or a debug halt loses a tick. A shunt burst borrows the ADC and runs no kernel tick at all for its window (protocol sec 5.8).

### 2.2 Three rates, ten phases

The kernel runs three rates off the one tick:

- **FAST**, every tick (20 kHz). The measurement of the window the last command drove, the overcurrent trip, the back-EMF (the voltage the spinning motor generates) sums, the current loop and the motor write. This is `kernel/phases/fast.rs`.
- **MEDIUM**, once per period of `DECIM_MED` = 10 ticks (2 kHz). The outer loops, the estimators, the limits, the detectors and the publishes. The medium work is cut into ten phases, and each tick runs one of them.
- **SLOW**, once every `DECIM_SLOW` = 32 medium periods (62.5 Hz). The stall permit lease, undervoltage and the winding thermometer. It runs inside the medium phases in stages, one stage per period.

The phases are in data-flow order, so each one reads what the phases before it wrote in the same period:

| phase | name | what runs |
|---|---|---|
| 0 | CONTROL | the CONTROL read, the enable and mode edges, the drive kind (and the configuration, on the very first tick) |
| 1 | OBSERVER | back-EMF close, observer step, speed source switch |
| 2 | TRAJECTORY | trajectory and position loop, the hold's drive kind |
| 3 | LIMITS | limits fold, OpenLoop base duty, stall fault |
| 4 | VELOCITY | velocity loop or the current clamp, where the current reference and band land |
| 5 | RAIL | rail estimate, sensor screen, position error timer |
| 6 | SLOW | the slow block when it is due, the thermometer's finish one period later |
| 7 | PUBLISH | estimates, mode, limits and sensors publish, and the tick-load close |
| 8 | SPARE_A | the configuration take after a commit, else a thermometer install |
| 9 | SPARE_B | the NTC (board thermistor) reading before a SLOW tick, the derate after a finish |

The schedule is written down once, in `kernel/scheduler/table.rs`, and the tree follows it. `kernel/scheduler/` holds the tick and the table, `kernel/phases/` holds one file per phase, and the loops and detectors those phases run live in `kernel/phases/loops/`. I want a reader to see the design from the folders.

The two halves talk through two small structs. The medium half leaves the fast half a `Command` (the drive kind, the current reference and band, the rail clamp), and the fast measurement reaches the same tick's medium phase as `Measured`. Only the CONTROL phase reads the host's CONTROL registers, so a host write lands at the next period's CONTROL phase. The delay from a sample to the command computed from it is fixed by the table, and the loop design counts on that (`control-theory.md`, "Why a Cascade").

### 2.3 Inside one tick

One tick runs in this order (`Kernel::on_tick` inside `runtime::isr::on_adc_dma_tc`):

1. Count the tick in `sample_tick` and name the new scan in place in the tick block.
2. The spare-phase arbitration. The very first tick loads the configuration. After that, SPARE_A runs the configuration take when one is due (sec 6), and a spare phase with no take runs a pending thermometer stage.
3. FAST measure. The window the previous command drove becomes this tick's current, voltage and flags, published in the tick block.
4. The TEL (telemetry stream) row, if a burst is streaming (sec 5). It is copied now, while the tick block still holds this tick's scan.
5. This tick's medium phase.
6. FAST drive. The current loop (or open loop, brake or off) and the motor write.
7. One step of the TEL lane, at most.
8. Back in the interrupt wrapper, the shunt-burst handshake poll and the tick-load accounting.

The order is the data flow again. Nothing reads the sample before it is measured, the row is copied before anything overwrites the block, and the motor write never waits behind the lane.

The ADC does not stop while the tick runs. The next trough scan lands about halfway through the period and the next crest scan near its end, both in the same tick block. So the tick moves the trough's few samples out of the way early, and finishes every read of the crest scan before the next one lands. The budget table carries both deadlines as rows (`TROUGH_DONE`, `FRAME_DONE`), and the DES (discrete-event simulator, sec 4) pin `the_tick_reads_its_scan_before_the_next_lands` holds the most expensive tick to them.

### 2.4 Rare work gets a fixed slot

The periodic work is easy to place. The hard part is the rare events, like a configuration take, a slow-rate stage, a telemetry frame boundary or the tick-load close, because two of them landing on one tick make the worst tick. Each rare event has a fixed home, so two only meet on a tick when the budget prices that combination:

- The configuration take owns the first SPARE_A after a commit (sec 6).
- The slow block runs in stages one period apart, on the SLOW phase and the spare phases. A stage whose phase is taken waits a period on its pending bit.
- The tick-load close lands on PUBLISH, the lightest Closed phase that carries no stage and no take (`health_close_lands_only_on_publish_while_streaming`).
- The TEL lane takes at most one step per tick, and none on a tick that lands a frame head, banks a frame or runs a slow-rate stage.

The arbitration is a fixed priority in the first `match` of `on_tick`. Boot comes first, then the take, then the thermometer. There is no queue to look at.

### 2.5 Why static

I picked a static table over a dynamic scheduler on purpose, and the reasons are all about this chip:

- The work is known up front. Three fixed rates in data-flow order, plus a short list of rare events. Nothing arrives at random and needs to be ranked.
- The dispatch is one `match` on a phase counter. A queue, deadlines or priorities to compare would all be instructions and taken branches on every tick, and on this chip those are exactly what costs time.
- Every tick can be priced. A tick's worst case is its phase row plus the events allowed on that phase, so the budget table and the DES can bound every tick. A dynamic scheduler's worst case depends on what it chose to run, which is much harder to bound and to pin.
- Reordering the same work does not create CPU time. If a tick does not fit, a dynamic queue can only rank the same work differently. The fix is to delete work, make it cheaper or move it to a phase with room, and a static table makes that move explicit and reviewable.
- The tick is the only context that touches kernel state, so the kernel needs no locks. An RTOS (real-time operating system) would split that ownership and give every task its own stack, and RAM is the scarcest thing on the chip.
- The loops get fixed timing. Every phase runs at a fixed offset from its sample, so the sample-to-command delay is a constant the control design can count on.

### 2.6 The main loop

Everything with no deadline inside a tick runs in the main loop at the bottom. That is the status LED, publishing the link counters and the dropped TEL rows, applying the clock trim, the shunt-burst page copy, the data job (a software CRC pass that runs longer than any bus body should), the deferred reboot and the rescue-pulse sampler. Then the loop sleeps in `wfi` until the next interrupt.

Its reach-ins into bus state mask the bus level only (`pfic::mask_bus`). The main loop can hold off the bus for a moment, and it never holds off the kernel.

## 3. The kernel above the bus

### 3.1 Priority levels

The V006's PFIC (programmable fast interrupt controller) has two preemption levels. The kernel has the upper one to itself (`runtime::isr::install_irqs`):

| vector | level | job |
|---|---|---|
| DMA1 CH1 TC | HIGH | the kernel tick, and the TEL lane while a burst streams |
| TIM2 | LOW | the bus break wake (transport sec 7) |
| USART1 | LOW | a TX arm drained |
| SysTick | LOW, taken last | the bus deadline mux, with dispatch inline |
| main loop | none (thread mode) | cold work, masks LOW only |

The bus vectors share LOW, so they never preempt each other, and the bus driver needs no locks either. SysTick is taken last within LOW, so a pending wake or TX completion resolves its frames before a pending deadline acts on them. The kernel preempts all of them and nothing preempts the kernel, so a tick enters at its scan, held off only by the longest critical section.

### 3.2 Why the kernel is on top

One bus body can run longer than a whole tick period, and one host frame's bus work always does (the `Deadline` body and every `frame` row in the budget table are over `TICK_PERIOD`). With the bus above the kernel, such a body sits over the tick, the scan's completion flag merges with the next one, and the tick is gone. Every host frame then costs a tick or two, and since the tick index is the kernel's clock, everything timed in ticks runs slow while the host polls. The DES keeps that model honest against the bench with `kernel_lane_below_bus_reproduces_bench_loss`.

With the kernel on top, the bus becomes a background server. It gets the time the kernel leaves, so each microsecond of bus work costs 1 / (1 - U) microseconds of latency, where U is the kernel's share of the CPU. A reply can pause between its arms for about one kernel body. The host's RESPONSE_DEADLINE (protocol sec 7) covers the stretched path (`reply_bound_covers_turnaround_under_kernel_preemption`), and the kernel loses nothing to traffic (`kernel_on_top_loses_no_tick_under_polling`).

The trade is deliberate. The motor has a hard deadline every 50 us, and the bus has a soft one measured in milliseconds. So when the two compete, the bus waits.

Being preemptible has hazards on the bus side, and the bus code closes them (transport sec 2). Each TX arm clears TC before its DMA starts, the frame projections read the ring cursor before the clock, and a configuration commit is bracketed by the count the kernel takes under (sec 6).

### 3.3 Who writes what

The priority split works because every shared field has exactly one writer:

- The bus is the only writer of CONTROL, CONFIG, CALIB and the position table. The kernel only reads them, with volatile reads through `region_ptr`, and never forms a reference into them. CONTROL is read once per period at the CONTROL phase, and a torn read across fields is accepted because each field is sane on its own. CONFIG and CALIB need more than that, so they arrive through the take (sec 6).
- The kernel is the only writer of the tick block (after the DMA and the interrupt land the scan and the stamp), the estimates, the references, the mode, the limits and the fault byte.
- The chip side (the interrupt wrapper and the main loop) writes the health block.
- The TEL channel splits its fields between a bus half and a kernel half the same way (sec 5).

There is one core and no atomic read-modify-write on it, so these contracts are plain volatile loads and stores, with compiler fences where the order matters.

## 4. Budgets and the DES

### 4.1 The budget table

`firmware/lib/core/src/budget/mod.rs` holds the most CPU each piece may take, in HCLK ticks at 48 MHz. A tick period is 2400 of them (`TICK_PERIOD`). The table has these rows:

- Kernel tick bodies by drive regime (Open or Closed) and phase, maximum and mean.
- One row for each event a tick can carry on top. That is the take, the slow stages, the TEL row, head, bank and lane steps, and the tick-load close.
- Interrupt entry and exit.
- The bus vector bodies, and the bus CPU of each frame type at 3 Mbaud.

One table has three users. The bench probe records the same components on the chip. The hardware test `budgets` fails any window whose maximum or mean is over its row. And the DES charges these values as its costs. While `budgets` is green on the bench, the DES never runs a component cheaper than the silicon did. The rows also add up on paper. A static pin like `the_take_tick_fits_at_budget_without_tel` sums the rows a tick can carry and checks the sum against the period.

Each row says where it came from. An M row is measured by the budget probe. Its maximum is the largest of its runs plus 3%, with at least 1 us of margin, and its mean is the measured mean plus 5%. An E row is estimated, and it names its instrument (usually the release disassembly priced with the fetch model) and its headroom.

### 4.2 The layout allowance

The same source compiles to tick bodies that differ in register allocation and block placement from one build to the next, and between the bench image the rows are measured on and the release image. That difference is real silicon time. So every kernel row, maximum and mean, carries `LAYOUT` (35 HCLK) on top, and routine layout shifts pass with no row edit.

The allowance counts against every hard budget. Fit is judged with every row at its maximum plus the allowance, and the DES charges it the same way. The allowance is never a reason to exceed the tick period, the turnaround deadline, a DES pin at budget or the stack reserve.

### 4.3 Rows move only on evidence

Raising a row makes the bench test and every DES pin easier at the same time. That makes it the cheapest way to fake a pass, so a row changes only with its evidence, in the same change:

- A raised M row comes from the probe on the production image. Bench images carry the probe's own overhead.
- A raised E row names its new instrument.
- The DES pins re-run against the new table.
- The PR states the old and new value, the measurement and the DES result.

A red bench window or a red pin is a finding. It gets a root cause or a redesign, and it is never cleared by raising a row alone. Rows go down too, when a refill measures them lower, so the table follows the code and does not just ratchet up.

### 4.4 DES pins run at budget

The DES in `firmware/lib/integration` runs the real bus driver and dispatch against simulated providers, with a kernel lane that preempts the bus at the table's costs (`sim/cpu.rs`). Its timing pins run at budget. Every body is at its maximum, with the layout allowance and the interrupt entry and exit, and every injected backlog or lag is the worst the table allows. A pin is never moved to a typical mode or given a smaller backlog to make it pass. The typical lane (bodies drawn below their maxima around the mean rows) is there for forecasts and gates nothing. `testing.md` describes the three test gears this sits in.

The DES must also be pessimistic against the bench. `step_soak_at_budget_drops_at_least_the_bench` and `window_load_at_budget_is_never_below_bench` fail if the simulator predicts less loss or less load than the bench measured. So a DES result of nothing lost is necessary, and only the bench gate makes it enough.

### 4.5 Fit is an outcome

A design either fits at budget or it does not. When it does not, the answer is to ask for less (fewer fields, a lower rate, a smaller batch), make the work cheaper, or find another mechanism. The pin stays where it is.

The TEL row size is an example. A frame of 16 rows has to clear the 3 Mbaud wire inside its 16 ticks, less the ticks the lane itself needs. A const assert in `firmware/lib/core/src/tel.rs` checks that 12-byte rows fit, and `twelve_byte_rows_fit_the_three_megabaud_batch` checks that 13 bytes would not. So the row maximum is 12 bytes because that is what fits, and the arm refuses anything bigger (protocol sec 5.6).

The configuration take is another one. Its tick has no room for a TEL row on top, so a burst waits for the take (sec 6.3), and the table never has to pretend both fit.

### 4.6 Health on the chip

The chip measures itself in every build. The tick interrupt is stamped at entry and exit, and the health block reports the mean tick load, the ticks that ran over one period, and the ticks lost (protocol sec 5.9). Since the kernel is on top, bus traffic does not show up in these counts. Any count is the kernel's own.

## 5. Cut-through TEL

A TEL burst (the telemetry stream, protocol sec 5.6) is a bounded run of status frames the servo sends on its own, 16 rows to a frame and one row per tick. While a burst streams, the kernel tick is the transmitter. The bus lends the TX lane to the kernel once the arming ack is out (or at the arm, when no ack follows), and takes it back in its first body after the LAST frame has left the wire. No TX interrupt and no bus body runs for the burst. The code is in `firmware/lib/drivers/src/tel/`.

The burst is cut-through. Rows go to the wire as they are produced, with no stage-then-send step in between:

- Each row is copied once, straight to its final wire offset in one of two frame images (sec 7.3).
- A frame breaks onto the wire once enough of its rows have landed, and its DMA reads the rows behind them as the wire reaches them. The lead is set per frame from its wire time plus a margin of ticks (`LEAD_MARGIN_TICKS`), so the wire never catches the producer at budget (`tel_kernel_lane_never_underruns_at_budget`).
- The producer feeds the CRC engine as rows land, and the CRC is patched into the image before the DMA gets there (sec 7.4).
- If ticks are lost in the middle of a frame, its DMA reads rows that have not landed and the frame fails its CRC. The host sees a sequence hole and the burst runs on to LAST (`tel_lost_ticks_mid_frame_fail_the_crc_and_leave_a_seq_hole`). A late frame never decodes as plausible data.

The lane is part of the static schedule (sec 2.4). It runs at the end of the tick and takes at most one step, which is a frame start, a CRC feed, the DMA arm or the CRC patch. It takes none on a tick that lands a frame head, banks a frame or runs a slow-rate stage (`tel_lane_never_steps_on_a_stage_tick`). Each kind of step is its own budget row.

The producer never blocks the tick either. A row that finds both images still waiting for the wire is dropped and counted in `tel_drop_count`.

## 6. The configuration take

CONTROL can be read field by field, because each field is sane on its own. CONFIG and CALIB are different. A set of gains, a limit band or a thermal model only makes sense whole, and a host often writes several fields in one commit. So the kernel never reads CONFIG or CALIB inside its loops. It works from its own copy, `KernelConfig` (`kernel/config.rs`), and replaces that copy whole, between commits.

### 6.1 The even/odd count

`Shared::config_seq` is a count the bus steps around every commit that reaches the kernel's inputs, the span `SPAN_START..SPAN_END`. The count goes odd before the commit's first store and even after its last, with compiler fences on both sides. So an even value always names one whole committed set. The bus is the count's only writer, and the pair costs a commit a few instructions (the `CONFIG_COUNT` row).

### 6.2 The take at a fixed phase

On SPARE_A, the kernel compares the count with the one it last took. If the count is even and new, it takes the configuration on that tick (`Kernel::take_config`). The take is one word copy of the span, laid out block for block the way the table lays it out (const asserts check every offset), and then the few values derived from it. If the count is odd, a commit is under way. The kernel keeps its current set and looks again at the next period's SPARE_A. It never waits on the bus.

The copy is correct because the kernel preempts the bus. No commit store can land while the take runs, so an even count read at the start is still true at the end. This is also why the take is a CPU copy inside the tick. A DMA copy would still be reading after the tick returns, and a commit could start under it.

A configuration write takes effect within one medium period, or one more when the take's tick lands inside the next commit (`config_apply_lands_within_its_bound_at_budget`, `dispatched_config_write_lands_at_the_take`). The first tick takes the configuration with no write at all.

### 6.3 TEL waits for a pending take

The take's tick has no room for a TEL row on top (`the_take_tick_fits_at_budget_without_tel`), so the two never share a tick. A burst's first row waits while a committed write has not been taken yet, and it never starts on SPARE_A. The kernel asks this only when a burst is about to start, since reading the count on every tick would cost every tick. Once a burst streams the host is silent, so no new take can arrive under it.

A host that arms a burst in the same COMMIT as a configuration write sees its first row up to a medium period and a tick late (`a_burst_never_streams_through_a_config_take`, protocol sec 5.6).

A take does not hold the TX lane the way a slow-rate stage does. A take can run every period, and holding the lane for it every period would stall the stream.

## 7. DMA as a second execution unit

### 7.1 The rules

The V006 has one core and a DMA controller, and I treat the DMA as a second worker that runs in parallel with the CPU. On the hot paths (the tick, the bus frame and the TEL burst) the rules follow from that:

- A large copy goes to DMA. Lay the data out as contiguous runs (the control table can be reordered for it), arm one copy per run, and let it go.
- After arming, look for CPU work to do beside it before anything has to wait for the result. The whole point is two movers working at the same time.
- Never queue two arms on one channel. The second only waits for the first, and the parallel work is gone.
- Poll for completion where you can, instead of taking an interrupt.
- Price the net saving. DMA cycles on the AHB (the chip's internal bus) stall the CPU's fetch, and arming a channel costs instructions too, so a copy below the arm cost stays on the CPU.
- Cold paths use a plain small copy. The arm cost and the code size are not worth it there.

The channel map, and the DMA priority ladder that keeps the RX ring safe under all this traffic, are in transport sec 2 and sec 7.

### 7.2 ADC scans land in the tick block

DMA1 CH1 writes both ADC scans of every PWM period straight into the control table's tick block. The table is laid out in scan order, so the crest scan's slots are the published fields themselves (protocol sec 5.6). The tick interrupt does not copy a sensor frame. It names the terminal taps in place, moves the trough's few samples out of the next scan's way, and stamps `sample_tick`, and the kernel reads the rest where it landed. A bus read and a TEL row copy from the same block.

While the CPU runs the tick, the ADC and the DMA are already taking the next period's scans. That is where the two deadlines in sec 2.3 come from.

### 7.3 The two-chunk TEL row

A TEL row is up to two chunks of the table (protocol sec 5.6), so each row needs two copies, and the work is split between the two movers. The plan (`RowPlan` in `tel/rows.rs`) is built once at the arm, off the kernel's tick. The longer chunk goes to DMA1 CH6 as one memory-to-memory copy. The CPU copies the shorter chunk itself. The shorter chunk is at most half a row, three halfwords, and copying that straight-line costs less than arming a channel.

The CPU copies its chunk first and then arms the DMA, so the DMA's copy runs while the CPU goes on with the medium phase and the drive. Each row arms CH6 exactly once, and that copy lands before the next tick enters, so an arm never queues behind another on the channel (`the_tick_reads_its_scan_before_the_next_lands`). CH6 belongs to the kernel for the whole burst, since no reply snapshots are taken while the lane is lent.

### 7.4 The CRC beside TX

SPI1 is the CRC engine (protocol sec 3.2). DMA1 CH3 feeds it halfwords while other channels move the bytes, so the CPU never computes a CRC on a hot path:

- On a reply, the TX CRC is computed in parallel with the transmission and patched into the last arm before the DMA reaches it (transport sec 4).
- On a TEL frame, the lane feeds the engine as rows land and patches the result into the image ahead of the wire (sec 5).
- On receive, the feed starts at the covered checkpoint while the CRC bytes are still on the wire, and the verdict only polls a finished result (transport sec 4).

## 8. Code shape on a fetch-bound chip

### 8.1 The fetch model

The V006 reads flash one aligned 4-byte word at a time, three HCLK per word at 48 MHz, with no cache. Every taken branch, call and return throws away what was prefetched and starts over. A branch that is not taken costs only its bytes. The budget table's E rows are priced with this model.

So on this chip an instruction costs about what it takes to fetch it. Fewer instruction bytes and fewer taken branches make a body faster, much more than clever arithmetic does.

### 8.2 Lean code is speed

Since bytes are time, the first tool is deletion. Fewer conditionals, fewer special cases, and one general path where there were several special ones. A fix that adds a branch to the tick is a smell, and the first question is what deletion would make the branch unnecessary. In practice:

- Keep the hot path on the fall-through side of its branches, and keep small hot bodies branchless.
- Inline small hot helpers, since a call and its return each start the fetch over.
- Do the math at compile time. The chip const-evaluates its timing constants (`KernelTiming`), so the kernel never divides at run time.
- A check that is provably redundant on a hot path may go, each removal with its safety argument and a test that pins the invariant. The firmware still never panics.
- Structure is welcome when it costs nothing at run time. No dynamic dispatch on hot paths, and no layers that add instructions.

### 8.3 Hot functions start on a 4-byte boundary

A function entered at 2 mod 4 spends an extra fetch on every call. So every out-of-line function on a hot path (the tick interrupt and its callees, the bus vector bodies, the bus frame path, the TEL lane) carries `#[rustc_align(4)]`. `scripts/check-hot-align.sh` fails CI if any of them links at 2 mod 4 or disappears from the image, on both the release and the bench image.

The pin goes on whether or not the function happens to be aligned today. Layout moves with every change, and the pin keeps a later change from falling into the same trap. New hot code adds the attribute and the script entry in the same change.

The board also builds with `-align-all-nofallthru-blocks=2`, which puts every block not reached by fall-through on a 4-byte boundary. That flag does not cover function entries, which is why the pins exist. The four interrupt bodies share one `.trap.rust` input section, so each of them needs its own pin.

### 8.4 No soft arithmetic, no mem* builtins

The core has a multiply and no divide. A runtime `/` or `%`, or any 64-bit multiply or shift, pulls in a compiler-builtins routine that costs hundreds of cycles plus flash. `.github/check-soft-arith.sh` fails CI if the image links any of them, so divisors stay compile-time constants.

The same thinking applies to `memcpy`, `memset`, `memcmp` and `memmove`. A builtin costs the call, a generic dispatch on alignment and length, and code that the 4-byte pin cannot reach. Runtime paths must not call them, and the ways around them are:

- A small fixed-size copy becomes typed word loads and stores, or an inlined copy the compiler does not lower to a call.
- `MaybeUninit` replaces a zero fill whose every byte is written before it is read.
- Anything left gets its own routine, sized for its alignment and length and pinned like any hot function.
- Watch for the implicit calls too, like struct moves, array zeroing, `copy_from_slice`, `fill`, and `==` on arrays.

No CI check covers this rule today, so review is the gate (sec 9).

### 8.5 Nothing runs from RAM

On many chips the answer to slow flash is to run hot code from RAM. On the V006, RAM is the resource that runs out first, and the stack, the frame buffers and the control table already share it. So nothing runs from RAM, and the answer to a slow body is to make it smaller.

## 9. Caveats

A few things are still rough, and I want to be upfront about them.

- The winding thermometer's stages are the one exception to "every pin gates". Their rows report and do not gate fit, and the streaming case of `thermal_close_fits_at_budget_while_seated` is ignored, because those stages run over the period at budget today. The budget table's header names the exception, so it cannot spread quietly.
- The bus frame path still calls a few `mem*` builtins (in dispatch, frame routing, the reply gather and the position table commit), and no CI check lists them yet.
- With the kernel on top, a reply's latency depends on how busy the kernel is. RESPONSE_DEADLINE covers it, but a turnaround measured with the motor at rest will not hold while the motor moves (transport sec 10).
- A shunt burst runs no kernel tick for its window (`control-theory.md`, "What It Does Not Cover"). A flash SAVE (torque off) or a debug halt stalls the whole core. SAVE and halts show up in `tick_lost_count`, and the burst shows up nowhere, on purpose.

If something here does not match the code, or you have a better idea for any of it, leave a message on the [discussion board](https://github.com/OpenServoCore/open-servo-core/discussions). I'd love to hear it.

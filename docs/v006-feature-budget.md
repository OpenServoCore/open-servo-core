# V006 Feature Budget

This sheet shows what each firmware feature costs on the CH32V006 in flash, RAM, stack and CPU, and whether the whole feature set fits. The image measured is the `osc-dev-v006` app at 6d8d1472, built the way CI builds it (`cargo build --release -p osc-dev-v006-app`, opt-level 3, LTO, one codegen unit). Flash, RAM and static counts come from that ELF. CPU numbers are bench measurements where one exists and static counts where none does. Every row says which kind it is. The bench measurements were taken on earlier images and have not been re-run on this one. The Limits and Winding estimate rows carry one more measurement: the size delta between the release images at 013ec045 and 54698078, built the same way, which is what the thermal band cost over that base.

Short answer: everything fits. Flash is not the limit (6.7 KB left). CPU is not the limit at one host (load 31-35% with torque off). RAM is the limit, and the stack is the part that runs out. The planned fusion rings alone would use 84% of the RAM that can still be added.

## Budget lines

| Resource | Capacity | Used | Left | Notes |
|---|---|---|---|---|
| Flash (app region) | 58,112 B | 51,428 B | 6,684 B (11.5%) | The chip has 62 KB, but CONFIG A/B (1 KB), CALIB (4 KB) and META (256 B) are carved out of it. tinyboot lives in system flash. |
| RAM, static (.data + .bss) | 8,192 B | 5,896 B | 2,296 B stack region | .data 252 B, .bss 5,644 B; `_ebss` = 0x20001708 |
| RAM growth before the link fails | 2,296 - 1,536 B | - | 760 B | `osc-config.x` asserts at least 1,536 B above .bss. With `--features bench` the probes take 528 B of it, with `bench-window` 648 B. |
| Stack | 2,296 B region | ~1.25 KB measured high water (earlier image) | ~1.05 KB free at rest if the high water holds | Static worst nesting ~1.05 KB (frame sums below), so ~1.2 KB static margin |
| CPU, kernel at 20 kHz | 50 us per tick | 31-35% with torque off, ~60% driving at 20% duty | 65-69% with torque off | Measured (tick_load_mean_q15). Fast tick ~13 us idle, ~20 us driving while streaming. |
| CPU, transport | per host frame | 80-160 us per own frame in the bus ISRs | - | Below the kernel: no tick lost; the frame's latency stretches by 1 / (1 - U) |

Flash breakdown: `.vector_table` 11,976 B (the stubs plus the four ISR bodies, with the whole kernel tick inlined into the ADC DMA handler: 9,606 B), `.text` 37,410 B, `.rodata` 1,788 B, `.data` load image 252 B, `.tb_version` 2 B. The image has no divide instructions and only 132 multiply instructions in total (82 `mul`, 28 `mulh`, 22 `mulhu`).

## Features

Columns:

- **Flash**: bytes attributed from the DWARF line table. Inlined core/std helpers and the `math`/`bytes` helpers count toward the project file that called them. Out-of-line functions count where they are. This is a measurement of the image, but which feature a byte "belongs" to involves some judgment, so read each number as plus or minus 10-20%.
- **RAM**: exact, from the DWARF layout of each static.
- **CPU**: **M** = measured on the bench (idle points = percentage points of the 50 us tick with torque off; 1 pt = 0.5 us = 24 cycles). **S** = static count from the kernel ISR disassembly (instructions and multiplies on that feature's path; every instruction lies on some path, but one tick executes only a fraction of them). **E** = estimate.
- **Rate**: FAST = every tick. MEDIUM = one phase per tick, so each phase runs once every 10 ticks. SLOW = once every 320 ticks.
- **Separable**: how the feature turns off today. "flag" = cargo feature, "runtime" = a register or config field, "module" = cleanly removable code, "core" = the servo needs it.

### Kernel and estimators

| Feature | Flash B | RAM B | CPU | Rate | Separable |
|---|---|---|---|---|---|
| Kernel skeleton, config snapshot, faults, medium phase glue | 3,008 | ~260 (config snapshot 156, timing 36, command 36, latches) | M: the config snapshot saved 2.2-2.7 us/tick. S: 716 ins, 8 muls | FAST + MEDIUM | core |
| Fast path: window select, measure, drive (settle gain, floors) | 1,134 | ~30 | M: settle-gain table +2.4 pts, two drive-window floors +1.9 pts. S: 225 + 95 ins, 4 muls | FAST | core |
| Bias tracker (shunt zero, two zeros) | 214 | 12 | M: +4.9 pts as first built, -3.1 after the trim, ~+1.8 net. S: 70 ins | FAST | core |
| Current loop (PI) | 378 | 4 | S: 117 ins, 12 muls (closed-loop modes only) | FAST | core |
| Open-loop duty limiter | 200 | 4 | S: 7 ins inline + `DutyLimiter::step` 58 ins out of line (open loop only) | FAST | core |
| Back-EMF observer (boxcar) | 244 | 20 | S: 76 ins, 8 muls (sums every tick, close every medium half) | FAST | core (velocity source above the floor) |
| Omega switch | 164 | 8 | S: 56 ins | MEDIUM (OBSERVER) | core |
| Fusion (pot) observer | 666 | 12 | S: 197 ins, 10 muls (5 widening per the fusion plan) | MEDIUM (OBSERVER) | core (the only source under the floor) |
| Trajectory + position loop | 968 | ~20 | S: 141 ins inline + `step_position` 550 B out of line | MEDIUM (TRAJECTORY) | core |
| Velocity loop | 578 | 4 | S: 175 ins, 8 muls | MEDIUM (VELOCITY) | core |
| Limits: four-term min (cap, derate, rail, goal), press and collision detectors, endstops | 776 LIMITS body | 16 | S (five-term image): LIMITS 248 ins, 3 muls (193 ins, 5 muls before); E: +140 HCLK including headroom. Rail fold +17 ins at RAIL, E +59 HCLK. The I2t fold those counts include is retired and nothing has re-counted them; the heats it kept are the winding estimate's, whose row carries the measured delta. | MEDIUM (LIMITS, RAIL), SLOW | core |
| Rail estimator (vbus) | 130 | 16 | S: 41 ins, 2 muls | MEDIUM (RAIL) | core |
| Winding estimate (board NTC + two heats on the warmed input, NTC fault), in place of the winding thermometer | 32 | 18 | S: two multiplies on the SLOW due for the warmed input, one multiply and a shift per node on the finish, ~0 averaged. The one-node estimate measured 24 B / 6 B on the sheet's image; the band from 013ec045 (one node beside the I2t fold) to 54698078 (two heats on the warmed input, the derate as the one thermal path, the NTC fault) measured text +8 B, data 0, bss +12 B between the two release images. The thermometer's row was 2,868 B / 104 B; the band that replaced it measured text -2,844 B and bss -116 B on the release image | SLOW | runtime (no NTC curve = UNSET; both gains 0 reads the NTC alone), module |
| Shunt burst (ident) | 1,014 | 1,932 | 0 at rest; the kernel stops for ~1.04 ms per burst | on request | runtime, module (identification needs it) |
| Position table (`pos_lut`) | 748 | 514 | S: 53 ins, 3 muls per linearization | MEDIUM (OBSERVER) | runtime (`lut_live`) |
| Fast decay | ~100-150 (E, spread over the fast path, motor write, config) | 0 | a few branches per tick | FAST | runtime (`openloop_decay`) |
| Sense: injected tap B, tap swap by direction | inside HAL/scan | 0 | M: injected tap +0.26 pts; tap swap +0.4 pts idle, +1.8-2.9 pts at 20% drive (from register allocation, not the swap itself) | FAST | core for rev-2A sensing |
| Health: tick load, stack paint | 482 | ~20 | S: 81 ins per tick (~1 pt, E) | FAST | core (health is a regular table block in every build) |

### Transport, table and platform

| Feature | Flash B | RAM B | CPU | Separable |
|---|---|---|---|---|
| Transport core: framer, route, verify, reply, TX engine, SPI CRC engine, break wake | 7,780 | RX ring 512, CRC snapshot 256, bus state ~124 | M: 80-160 us of bus ISR per own frame (break wake 18 us + one SysTick body). S+sim: a foreign request ~63 us, a foreign status ~41 us | core |
| Protocol decode (frames, wire) | 2,292 | - | inside the frame cost | core |
| Group instructions (GREAD/GWRITE) | 1,980 | - | inside the frame cost | module in principle (fleet feature) |
| Software CRC-16 (odd-byte tail fold, ENUM key, persist) | 496 + 512 table in .rodata | - | one tail byte per frame | core; one out-of-line copy serves the call sites (see B below) |
| Chain snoop | 440 | 20 | per foreign frame (the fleet cost above) | core for chains; a chain-length limit saves CPU, not memory |
| Clock discipline: CAL ruler + trim loop | 1,760 | 64 + 16 stamp ring | 0 per tick; one DMA stamp per break, read only during a CAL train | core |
| TEL streaming | 1,744: out of line 1,256 (row 222, head 220, pickup 142, lane 672), the rest the inline gate and lane poll, the bus-side post and the row encoding | 436 (two 202 B frame images, the arm mailbox) + 48 lane state in `KERNEL` | A row is one DMA copy of its longer chunk (CH6, halfwords), armed first, and a CPU copy of the shorter while it runs, as the tick block is published: per streamed tick S: 65 ins over the gate's 25 (54 with one chunk), E 129-153 HCLK; a burst's first tick takes the bus-built copies, 178 ins in all over the plain tick (E 250 HCLK over a row). The ADC DMA lands the scans in the tick block and its publish runs every tick: E 0-58 HCLK a phase over the frame copy it replaced (`budget::TICK_PUBLISH`). Burst TX by DMA costs ~0 (M) | runtime (`tel_count` 0) |
| Dispatch + control table (map, rules, staging) | 6,032 | table 1,024, write staging 252, misc 34 | per frame | core |
| Persistence (SAVE/FACTORY, flash driver) | 3,126 | 8 | on SAVE only (blocking flash program) | core |
| Stamp + data state | 1,046 | - | off the tick | core |
| Chip HAL, init, scan, motor write | 6,430 | small statics ~44 | motor write runs every tick (38 ins, inline in the kernel ISR) | core |
| Runtime glue (ISRs, main loop, registry, statics, LED, own `memcpy`/`memset`) | 1,884 | LED 16 | - | core |
| Third party (core helpers with no project call site, qingke-rt, critical-section) | 742 | - | - | core |
| Startup and vector stubs (no line info) | 476 | - | - | core |
| .rodata (CRC table 512, control-table rule and jump tables, constants 1,276) | 1,788 | - | - | - |
| .data load image + `.tb_version` | 252 + 2 | (in RAM) | - | - |
| **Total** | **51,428** | **5,896** | | |
| `--features bench` (budget probe) | +2,448 | +528 (KERNEL_PROBE 440, BUS_PROBE 84, TEL bank count 4) | a masked stamp at each bus body's ends and a record update after the measured span (S: 100-130 instructions per bus body); after each tick's exit stamp the per-class rows' update, event ticks out of line (E: ~4.0 us a tick, fetch-model emulation) | flag (measured by a build diff) |
| `--features bench-window` (adds gamma(k), long ticks, entry lateness) | +2,848 | +648 (WINDOW_PROBE 120 on top) | the window record's update beside the rows' (E: ~7.3 us a tick in all) | flag (measured by a build diff) |

The two biggest RAM consumers are the shunt burst buffer (1,920 B, 23% of RAM, used only during identification) and the control table (1,024 B). The kernel state (636 B), the position table (514 B) and the RX ring (512 B) come next.

The two biggest CPU consumers are the host-frame service in the bus ISRs (80-160 us per own frame, plus 41-63 us per foreign frame on a shared bus) and the fast path's measure-and-drive glue (~13 us of the 50 us tick at idle). Multiplies are not a cost driver: the whole kernel ISR holds 74 multiply instructions, and the worst tick runs about 20-30 of them, roughly 1 pt. The cost is loads, stores, branches and flash wait states, which is why the measured deltas above come from register allocation and branching, not from arithmetic.

### RAM map

| Static | Bytes | Owner |
|---|---|---|
| `BURST_BUF` | 1,920 | shunt burst (960 ADC codes) |
| `SHARED.table` | 1,024 | control table, the ADC scans landing in its tick block |
| `SHARED.pos_lut` | 514 | position table, 257 x i16 |
| `RING` | 512 | transport RX ring |
| `KERNEL` | 538 | fast 72, medium 154 (winding estimate 6), config snapshot 156, command 36, timing 36, TEL lane 48, io 28, latch/phase 8 |
| `TEL_CHANNEL` | 436 | TEL frame images and arm mailbox |
| `CELLS` (.data) | 236 | ServoBus 220 (TX engine 40, clock discipline 64, chain 20, framer 16, pending 16, TEL burst 12, misc 52), LED 16 |
| `crc::SNAPSHOT` | 256 | reply payload span for the SPI CRC engine |
| `SESSION` | 252 | write staging 232, pending verdict 20 |
| merged small statics | 60 | burst FSM, tick load, scan sequence, break wake, break stamp ring 16 |
| `SHARED` rest | 34 | uid 16, store ref 8, generation |
| `.data` rest | 16 | config store, burst FSM |

### Stack

The stack region is 2,296 B. The worst case is three levels nested. Static frame sizes, read from the prologues:

- main: `__run` 256 B.
- 48 B hardware push.
- LOW, the bus (SysTick): `drive_framer` 64, `route_frame` 156, `dispatch` 164, `persist_table` 16, `ConfigStore::save` 72, `program` 96, plus 36 for SysTick itself.
- 48 B hardware push.
- HIGH, the kernel (ADC DMA ISR): 48 B, plus at most 48 B below it (`burst::handshake`, or `Kernel::configure` 28 + `take_config` 16).

The same frames as with the bus on top, nested in the other order. That sums to about 1,050 B. The measured free minimum on an earlier image was ~1.1 KB of a 2,356 B region at rest, or ~1.25 KB used. The 1,536 B link assertion only guards .bss growth, not deeper call chains. The biggest frames are `__run` and `dispatch`.

## Planned items

| Item | Flash B | RAM B | CPU | Basis |
|---|---|---|---|---|
| Fusion step 4, offset form | ~300-400 | **640** (two rings of 80 x u32 for a 40 ms window) + ~12 state | +2 muls and two ring writes in the OBSERVER phase: <0.1 pt averaged | RAM from the fusion plan; the rest is E |
| Encoder support | per board | per board | atan2 in software CORDIC costs ~3-5 us per sin/cos: ~0.6-1 pt at the medium rate, 6-10 pts at the fast rate | note only |
| **Sum** | **~0.4-0.7 KB** | **~670** | **<0.5 pt** | |

## Scenarios

| Scenario | Flash used / left | Static RAM | Stack region | Room to the link assertion | Stack free at rest (E) | CPU |
|---|---|---|---|---|---|---|
| Today (main) | 51,428 / 6,684 B | 5,896 B | 2,296 B | 760 B (92 B with bench) | ~1.05 KB | 31-35% idle, ~60% at 20% drive |
| A. Everything on (today + the planned items) | ~51,850-52,150 / ~5,950-6,250 B | ~6,565 B | ~1,625 B | ~90 B; **a `--features bench` build fails the assertion by ~580 B** | ~0.38 KB; static margin ~570 B | +<0.5 pt |
| B. Push the V006 further: A with the cheapest cuts | ~51,850-52,150 / ~5,950-6,250 B | ~6,005 B | ~2,185 B | ~650 B; a `--features bench` build fails by ~20 B | ~0.94 KB | same as A |
| C. The earlier six-item cut, applied to A | ~51,700-52,050 / ~6,050-6,400 B | ~6,565 B | ~1,625 B | ~90 B | ~0.38 KB | idle x 0.8 at 16 kHz (25-28%), less HIGH work per silent pair |

**A fits, but only just.** Flash and CPU have room. Only the stack is tight: the rings take the static margin from ~1.2 KB down to ~570 B, and the bench build no longer links.

**B: the cheapest cut is RAM layout. No feature is lost.**

1. **Store the fusion rings as 4 ms block sums.** That is 2 x 10 x u32 = 80 B instead of 640 B. A 40 ms window of ten 4 ms blocks is still a whole multiple of 4 ms, so the 250 Hz comb still cancels. The correction updates at 250 Hz instead of 2 kHz, which is still fast next to its tens-of-ms time constant. Alternatives: i16 storage (320 B), or overlay the rings on `BURST_BUF` and refill the window after a burst (0 B, since bursts run only during identification).
2. **The CRC dedupe is already in.** `osc_crc_continue` is one out-of-line function (40 B, the 512 B table lookup) behind 44 call sites. Only the USART1 handler and `verify` still inline the loop, so no flash is left to take here.
3. **Reserve lever if flash ever binds:** the last 2 KB of CALIB ("spare for tables to come") and the spare second 256 B page of each CONFIG window add up to 2.5 KB. Moving the region boundary is a layout change that moves saved images, so it needs a migration or a FACTORY.

**C does not reach the limit.** Two of its six items are already taken: rev 2A runs one wire flavour (HDSEL single wire, no buffered build), and the TEL row is capped by its packed bytes at the line rate (`tel::ROW_BYTES_MAX`, 11 B). Limiting chain length saves fleet CPU, not memory. The 16 kHz tick saves CPU nobody is short of, and it moves the PWM into the audible band unless PWM and ADC stay at 20 kHz with the kernel decimated (the reserve lever already on record). The drift tracker is already deleted, so dropping fast decay (~0.1-0.15 KB) is all C saves, and it does not touch RAM, which is where the limit is.

If RAM has to go further than B, these levers are next in order of cost: the position table read straight from its CALIB page (-514 B RAM, but a table can then only go LIVE after SAVE), and a smaller burst (-2 B per code, less identification bandwidth).

## Top 30 symbols

| Bytes | Symbol |
|---|---|
| 9,606 | `__qingke_rt_DMA1_CHANNEL1` (the whole kernel tick, inlined) |
| 7,596 | `runtime::run::__run` (main loop) |
| 5,366 | `BusDispatcher::dispatch` |
| 1,920 | `runtime::burst::BURST_BUF` (.bss) |
| 1,896 | `.L_MergedGlobals.128` (.bss: KERNEL, RING, TEL_CHANNEL, SESSION, small statics) |
| 1,680 | `bus::decode::decode` |
| 1,572 | `runtime::statics::SHARED` (.bss) |
| 1,342 | `ServoBus::route_frame` |
| 1,144 | `runtime::burst::handshake` |
| 1,048 | `ServoBus::drive_framer` |
| 870 | `ServoBus::route_chain` |
| 854 | `__qingke_rt_USART1` |
| 800 | `ServoBus::verify` |
| 704 | `ConfigStore::wipe` |
| 694 | `runtime::burst::on_dma_event` |
| 686 | `runtime::statics::install` |
| 664 | `config_store::program` |
| 650 | `control_table::rules::check_cmp` |
| 636 | `runtime::statics::KERNEL` (.bss) |
| 632 | `__qingke_rt_TIM2` |
| 550 | `TrajGen::step_position` |
| 540 | `__qingke_rt_SysTick` |
| 528 | `ConfigStore::save` |
| 512 | `.L.crctable` (.rodata, the LLVM-generated CRC table) |
| 512 | `providers::ring::RING` (.bss) |
| 504 | `BusDispatcher::commit` |
| 462 | `TxEngine::stage_gather` |
| 436 | `runtime::statics::TEL_CHANNEL` (.bss) |
| 412 | `Drivers::install` |
| 392 | `TelLane::service` |

## What only the bench can tell

- **The load each feature adds.** No cargo flag gates any kernel feature, so each delta needs an A/B image on the board. The deltas above that are marked M come from the bench. The S counts give an order of magnitude, not a percentage.
- **The stack high-water mark with the planned RAM in place.** Static frame sums are an upper bound. The stack-paint minimum (`stack_free_min`) is the real number, and it should be re-read after any .bss growth over ~100 B.
- **The TEL row and tick block publish on silicon, and the worst medium slice** (which phase plus the fast path makes the longest tick).
- **The fusion rings' real cost in the OBSERVER phase.**

## Method

- Build: `scripts/build-image.sh --bench <rev> osc-dev-v006`, which builds the release app and the `--features bench` app from one fixed path (the README's Firmware section says why the path matters); add `--window` for the `bench-window` app when a run needs gamma(k), the long ticks or the entry lateness.
- Sections: `llvm-size -A`. Symbols: `llvm-nm -S --size-sort -C`.
- Flash per module: a pyelftools script walks every CU's line program, drops sequences garbage-collected to address 0, and assigns each 2-byte slot to its innermost project file. If that file is core/std, compiler-builtins or the `math`/`bytes` helpers, the slot goes to the deepest `DW_TAG_inlined_subroutine` call site (or subprogram) that is project code. 48,910 of the 49,386 B of code carry line info. Each estimator and loop file counts to its own row; the phase files (`kernel/phases/*.rs` except `fast.rs`), the scheduler, the config snapshot and the fault latch count to the kernel skeleton.
- RAM: `llvm-nm` addresses for the statics and the DWARF struct layout for their members (`Shared`, `Kernel<..>`, `ServoBus<..>`, `Session`, `TelChannel`, `Cells`).
- CPU S counts: `llvm-objdump -d`, with each instruction of `__qingke_rt_DMA1_CHANNEL1` mapped to its owning file by the same attribution. Stack frames come from the first `addi sp, sp, -N` in each prologue.

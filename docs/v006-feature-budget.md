# V006 Feature Budget

This sheet shows what each firmware feature costs on the CH32V006 in flash, RAM, stack and CPU, and whether the whole feature set fits. The image measured is the `osc-dev-v006` app built the way CI builds it (`cargo build --release -p osc-dev-v006-app`, opt-level 3, LTO, one codegen unit). Flash and RAM numbers come from the ELF. CPU numbers are bench measurements where one exists and static counts where none does. Every row says which kind it is.

Short answer: everything fits. Flash is not the limit (3.5 KB left). CPU is not the limit at one host (load 31-35% with torque off). RAM is the limit, and the stack is the part that runs out. The planned fusion rings alone would use 78% of the RAM that can still be added.

## Budget lines

| Resource | Capacity | Used | Left | Notes |
|---|---|---|---|---|
| Flash (app region) | 58,112 B | 54,546 B | 3,566 B (6.1%) | The chip has 62 KB, but CONFIG A/B (1 KB), CALIB (4 KB) and META (256 B) are carved out of it. tinyboot lives in system flash. |
| RAM, static (.data + .bss) | 8,192 B | 5,836 B | 2,356 B stack region | .data 344 B, .bss 5,492 B; `_ebss` = 0x200016CC |
| RAM growth before the link fails | 2,356 - 1,536 B | - | 820 B | `osc-config.x` asserts at least 1,536 B above .bss. With `--features bench` the room is 684 B. |
| Stack | 2,356 B region | ~1.25 KB measured high water | ~1.1 KB free at rest, 848 B with the bringup probe | Static worst nesting ~1.5 KB (frame sums below), so ~850 B static margin |
| CPU, kernel at 20 kHz | 50 us per tick | 31-35% with torque off, ~60% driving at 20% duty | 65-69% with torque off | Measured (tick_load_mean_q15). Fast tick ~13 us idle, ~20 us driving while streaming. |
| CPU, transport | per host frame | 80-160 us per own frame in the bus ISRs | - | Below the kernel: no tick lost; the frame's latency stretches by 1 / (1 - U) |

Flash breakdown: `.vector_table` 14,252 B (the stubs plus the four ISR bodies, with the whole kernel tick inlined into the ADC DMA handler: 11,438 B), `.text` 38,188 B, `.rodata` 1,760 B, `.data` load image 344 B, `.tb_version` 2 B. The image has no divide instructions and only 124 multiply instructions in total (74 `mul`, 24 `mulh`, 26 `mulhu`).

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
| Kernel skeleton, config snapshot, faults, medium phase glue | 3,566 | ~240 (config snapshot 128, timing 40, command 32, latches) | M: the config snapshot saved 2.2-2.7 us/tick. S: CONTROL phase 572 ins | FAST + MEDIUM | core |
| Fast path: window select, measure, drive (settle gain, floors) | 1,462 | ~30 | M: settle-gain table +2.4 pts, two drive-window floors +1.9 pts. S: 344 + 70 ins, 3 muls | FAST | core |
| Bias tracker (shunt zero, two zeros) | 198 | 12 | M: +4.9 pts as first built, -3.1 after the trim, ~+1.8 net. S: 65 ins | FAST | core |
| Current loop (PI) | 516 | 8 | S: 160 ins, 10 muls (closed-loop modes only) | FAST | core |
| Open-loop duty limiter | 186 | 4 | S: 56 ins (open loop only) | FAST | core |
| Back-EMF observer (boxcar) | 340 | 20 | S: 102 ins, 9 muls (sums every tick, close every medium half) | FAST | core (velocity source above the floor) |
| Omega switch | 172 | 8 | S: 59 ins | MEDIUM (OBSERVER) | core |
| Fusion (pot) observer | 682 | 16 | S: 201 ins, 9 muls (5 widening per the fusion plan) | MEDIUM (OBSERVER) | core (the only source under the floor) |
| Trajectory + position loop | 1,116 | ~20 | S: 106 ins inline + `step_position` 566 B out of line | MEDIUM (TRAJECTORY) | core |
| Velocity loop | 610 | 4 | S: 183 ins, 8 muls | MEDIUM (VELOCITY) | core |
| Limits, stall, endstops, derate, detectors | 556 | 20 | S: 167 ins, 10 muls | MEDIUM (LIMITS) | core |
| Rail estimator (vbus) | 338 | 16 | S: 103 ins, 11 muls | MEDIUM (RAIL) | core |
| Winding thermometer | 286 | 8 | S: 87 ins, 8 muls, ~0 averaged | SLOW | runtime (gated by the `rtherm_*` thresholds), module |
| Ident aggregate | 256 | 20 | S: 78 ins, only while CONTROL `ident_agg` is set (off at boot) | FAST | runtime |
| Shunt burst (ident) | 1,066 | 1,932 | 0 at rest; the kernel stops for ~1.04 ms per burst | on request | runtime, module (identification needs it) |
| Position table (`pos_lut`) | 794 | 514 | S: 74 ins, 1 mul per linearization | MEDIUM (OBSERVER); FAST while TEL streams `pos_lin` | runtime (`lut_live`) |
| Fast decay | ~100-150 (E, spread over the fast path, motor write, config) | 0 | a few branches per tick | FAST | runtime (`openloop_decay`) |
| Sense: injected tap B, tap swap by direction | inside HAL/scan | 0 | M: injected tap +0.26 pts; tap swap +0.4 pts idle, +1.8-2.9 pts at 20% drive (from register allocation, not the swap itself) | FAST | core for rev-2A sensing |
| Health: tick load, stack paint | 474 | ~20 | S: 83 ins per tick (~1 pt, E) | FAST | core (health is a regular table block in every build) |

### Transport, table and platform

| Feature | Flash B | RAM B | CPU | Separable |
|---|---|---|---|---|
| Transport core: framer, route, verify, reply, TX engine, SPI CRC engine, break wake | 8,900 | RX ring 512, CRC snapshot 252, bus state ~184 | M: 80-160 us of bus ISR per own frame (break wake 18 us + one SysTick body). S+sim: a foreign request ~63 us, a foreign status ~41 us | core |
| Protocol decode (frames, wire) | 3,152 | - | inside the frame cost | core |
| Group instructions (GREAD/GWRITE) | 1,998 | - | inside the frame cost | module in principle (fleet feature) |
| Software CRC-16 (odd-byte tail fold, ENUM key, persist) | 1,026 + 512 table in .rodata | - | one tail byte per frame | core; see the dedupe lever below |
| Chain snoop | 640 | 20 | per foreign frame (the fleet cost above) | core for chains; a chain-length limit saves CPU, not memory |
| Clock discipline: CAL ruler + trim loop | 2,354 less the drift tracker, since deleted (its last form, with DMA break stamps, measured 1,568 B of .text) | 64 + 16 stamp ring | 0 per tick; one DMA stamp per break, read only during a CAL train | core |
| TEL streaming | 1,674 | 450 (double buffer 2 x 196, meta 18, feed 28, burst 12) | Staging runs at the tail of the tick. Sample encode while streaming is S: 370 ins. Burst TX by DMA costs ~0 (M) | runtime (`tel_mask` 0) |
| Dispatch + control table (map, rules, staging) | 5,918 | table 1,024, write staging 244, misc 34 | per frame | core |
| Persistence (SAVE/FACTORY, flash driver) | 3,108 | 8 | on SAVE only (blocking flash program) | core |
| Stamp + data state | 1,050 | - | off the tick | core |
| Chip HAL, init, scan, motor write | 6,666 | ADC DMA buffer 28, small statics ~48 | motor write runs every tick (484 B out of line) | core |
| Runtime glue (ISRs, main loop, registry, statics, LED) | 2,150 | LED 16 | - | core |
| Third party (memcpy/memset, qingke-rt, critical-section) | 746 | - | - | core |
| Startup and vector stubs (no line info) | 440 | - | - | core |
| .rodata (CRC table 512, control-table rule and jump tables 1,248) | 1,760 | - | - | - |
| .data load image + `.tb_version` | 344 + 2 | (in RAM) | - | - |
| **Total** | **54,546** | **5,836** | | |
| `--features bench` (probe counters) | +376 | +136 (HIGH_PROBE 52, TRIM_PROBE 84) | a few stores per probed event | flag (measured by a build diff) |

The two biggest RAM consumers are the shunt burst buffer (1,920 B, 23% of RAM, used only during identification) and the control table (1,024 B). The position table (514 B), the RX ring (512 B) and the kernel state (504 B) come next.

The two biggest CPU consumers are the host-frame service in the bus ISRs (80-160 us per own frame, plus 41-63 us per foreign frame on a shared bus) and the fast path's measure-and-drive glue (~13 us of the 50 us tick at idle). Multiplies are not a cost driver: the whole kernel ISR holds 93 multiply instructions, and the worst tick runs about 20-30 of them, roughly 1 pt. The cost is loads, stores, branches and flash wait states, which is why the measured deltas above come from register allocation and branching, not from arithmetic.

### RAM map

| Static | Bytes | Owner |
|---|---|---|
| `BURST_BUF` | 1,920 | shunt burst (960 ADC codes) |
| `SHARED.table` | 1,024 | control table |
| `SHARED.pos_lut` | 514 | position table, 257 x i16 |
| `RING` | 512 | transport RX ring |
| `KERNEL` | 504 | fast 104, medium 156, config snapshot 128, timing 40, command 32, TEL feed 28, io 8, latch/phase 8 |
| `TEL_CHANNEL` | 410 | TEL double buffer |
| `CELLS` (.data) | 296 | ServoBus 280 (TX engine 92, clock discipline 64, chain 20, framer 20, pending 16, TEL burst 12, misc 56), LED 16 |
| `crc::SNAPSHOT` | 252 | reply payload span for the SPI CRC engine |
| `SESSION` | 244 | write staging 232, pending verdict |
| `ADC_DMA_BUF` | 28 | scan |
| merged small statics | ~48 | burst FSM, tick load, scan sequence, break wake |
| `SHARED` rest | 34 | uid 16, store ref 8, generation |
| `.data` rest | 16 | config store, burst FSM |

### Stack

The stack region is 2,356 B. The worst case is three levels nested. Static frame sizes, read from the prologues:

- main: `__run` 352 B.
- 48 B hardware push.
- LOW, the bus (SysTick): `serve_break` 48, `drive_framer` 68, `route_frame` 156, `dispatch` 220, `ConfigStore::save` 68, `program` 92, plus 32 for SysTick itself.
- 48 B hardware push.
- HIGH, the kernel (ADC DMA ISR): 140 B, plus `Kernel::refresh` 212 B.

The same frames as with the bus on top, nested in the other order. That sums to about 1,480-1,500 B. The measured free minimum is ~1.1 KB at rest, or ~1.25 KB used. The 1,536 B link assertion only guards .bss growth, not deeper call chains. The biggest frames are `__run` (inlined bring-up locals), `dispatch` (a double 68 B Vec in `apply_commit`) and `Kernel::refresh`.

## Planned items

| Item | Flash B | RAM B | CPU | Basis |
|---|---|---|---|---|
| Fusion step 4, offset form | ~300-400 | **640** (two rings of 80 x u32 for a 40 ms window) + ~12 state | +2 muls and two ring writes in the OBSERVER phase: <0.1 pt averaged | RAM from the fusion plan; the rest is E |
| Thermometer fix | ~100-300 | ~8-16 | SLOW rate: ~0 | E |
| Encoder support | per board | per board | atan2 in software CORDIC costs ~3-5 us per sin/cos: ~0.6-1 pt at the medium rate, 6-10 pts at the fast rate | note only |
| **Sum** | **~0.4-0.7 KB** | **~670** | **<0.5 pt** | |

## Scenarios

| Scenario | Flash used / left | Static RAM | Stack region | Room to the link assertion | Stack free at rest (E) | CPU |
|---|---|---|---|---|---|---|
| Today (main) | 54,546 / 3,566 B | 5,836 B | 2,356 B | 820 B (684 B with bench) | ~1.1 KB (848 B with the probe) | 31-35% idle, ~60% at 20% drive |
| A. Everything on (today + the planned items) | ~55,100-55,500 / ~2,600-3,000 B | ~6,525 B | ~1,665 B | ~130 B; **a `--features bench` build fails the assertion by a few bytes** | ~0.4 KB (~0.16 KB with the probe); static margin ~170 B | +<0.5 pt |
| B. Push the V006 further: A with the cheapest cuts | ~54,200-54,800 / ~3,300-3,900 B | ~5,955 B | ~2,235 B | ~700 B | ~1.0 KB | same as A |
| C. The earlier six-item cut, applied to A | ~54,300-54,950 / ~3,150-3,800 B | ~6,485 B | ~1,705 B | ~170 B | ~0.45 KB | idle x 0.8 at 16 kHz (25-28%), less HIGH work per silent pair |

**A fits, but only just.** Flash and CPU have room. Only the stack is tight: the rings take the static margin from ~850 B down to ~170 B, and the bench build no longer links.

**B: the cheapest cuts are about RAM layout and duplicated code. No feature is lost.**

1. **Store the fusion rings as 4 ms block sums.** That is 2 x 10 x u32 = 80 B instead of 640 B. A 40 ms window of ten 4 ms blocks is still a whole multiple of 4 ms, so the 250 Hz comb still cancels. The correction updates at 250 Hz instead of 2 kHz, which is still fast next to its tens-of-ms time constant. Alternatives: i16 storage (320 B), or overlay the rings on `BURST_BUF` and refill the window after a burst (0 B, since bursts run only during identification).
2. **Make `osc_crc_continue` one out-of-line function.** LLVM turned the bitwise CRC loop into a 512 B table lookup and inlined it at about eight sites (1,026 B of code). One out-of-line copy that keeps the table should save ~0.7-0.9 KB (E) without changing the per-byte cost. Check turnaround on the bench, since the tail fold runs in the reply path.
3. **Reserve lever if flash ever binds:** the last 2 KB of CALIB ("spare for tables to come") and the spare second 256 B page of each CONFIG window add up to 2.5 KB. Moving the region boundary is a layout change that moves saved images, so it needs a migration or a FACTORY.

**C does not reach the limit.** Two of its six items are already taken: rev 2A runs one wire flavour (HDSEL single wire, no buffered build), and the TEL frame is capped at 6 fields (`STREAM_FIELDS_MAX`). Limiting chain length saves fleet CPU, not memory. The 16 kHz tick saves CPU nobody is short of, and it moves the PWM into the audible band unless PWM and ADC stay at 20 kHz with the kernel decimated (the reserve lever already on record). Dropping the drift tracker (~0.5 KB flash, ~40 B RAM) and fast decay (~0.1-0.15 KB) is all C saves, and neither touches RAM, which is where the limit is.

If RAM has to go further than B, these levers are next in order of cost: the position table read straight from its CALIB page (-514 B RAM, but a table can then only go LIVE after SAVE), and a smaller burst (-2 B per code, less identification bandwidth).

## Top 30 symbols

| Bytes | Symbol |
|---|---|
| 11,438 | `__qingke_rt_DMA1_CHANNEL1` (the whole kernel tick, inlined) |
| 7,668 | `runtime::run::__run` (main loop with init inlined) |
| 5,592 | `HighDispatcher::dispatch` |
| 2,350 | `ServoBus::route_frame` |
| 1,964 | `bus::decode::decode` |
| 1,920 | `control::burst::BURST_BUF` (.bss) |
| 1,748 | `.L_MergedGlobals.118` (.bss: KERNEL, TEL_CHANNEL, RING, SESSION, small statics) |
| 1,572 | `runtime::statics::SHARED` (.bss) |
| 1,394 | `TrimLoop::on_window` |
| 1,186 | `__qingke_rt_USART1` |
| 1,118 | `control::burst::handshake` |
| 1,084 | `ServoBus::drive_framer` |
| 1,052 | `Kernel::refresh` |
| 968 | `ServoBus::serve_break` |
| 778 | `ServoBus::verify` |
| 704 | `ConfigStore::wipe` |
| 672 | `__qingke_rt_TIM2` |
| 670 | `config_store::program` |
| 668 | `HighDispatcher::commit` |
| 636 | `control_table::rules::check_cmp` |
| 626 | `TxEngine::trigger` |
| 624 | `ConfigStore::save` |
| 606 | `__qingke_rt_SysTick` |
| 566 | `TrajGen::step_position` |
| 556 | `TxEngine::stage_gather` |
| 512 | `providers::ring::RING` (.bss) |
| 512 | `.L.crctable` (.rodata, the LLVM-generated CRC table) |
| 504 | `runtime::statics::KERNEL` (.bss) |
| 500 | `persist::Image::parse` |
| 484 | `Ch32Motor::write` |

## What only the bench can tell

- **The load each feature adds.** No cargo flag gates any kernel feature, so each delta needs an A/B image on the board. The deltas above that are marked M come from the bench. The S counts give an order of magnitude, not a percentage.
- **The stack high-water mark with the planned RAM in place.** Static frame sums are an upper bound. The stack-paint minimum (`stack_free_min`) is the real number, and it should be re-read after any .bss growth over ~100 B.
- **TEL encode cost per field, and the worst medium slice** (which phase plus the fast path makes the longest tick).
- **The fusion rings' real cost in the OBSERVER phase, and the CRC dedupe's effect on turnaround.**

## Method

- Build: in `firmware/boards/osc-dev-v006`, `CARGO_TARGET_DIR=<worktree>/target-v006 cargo build --release -p osc-dev-v006-app`, plus a second target directory with `--features bench` for the flag diff.
- Sections: `llvm-size -A`. Symbols: `llvm-nm -S --size-sort -C`.
- Flash per module: a pyelftools script walks every CU's line program, drops sequences garbage-collected to address 0, and assigns each 2-byte slot to its innermost project file. If that file is core/std, compiler-builtins or the `math`/`bytes` helpers, the slot goes to the deepest `DW_TAG_inlined_subroutine` call site (or subprogram) that is project code. 52,000 of the 52,440 B of code carry line info.
- RAM: `llvm-nm` addresses for the statics and the DWARF struct layout for their members (`Shared`, `Kernel<..>`, `ServoBus<..>`, `Session`, `TelChannel`, `Cells`).
- CPU S counts: `llvm-objdump -d`, with each instruction of `__qingke_rt_DMA1_CHANNEL1` mapped to its owning file by the same attribution. Stack frames come from the first `addi sp, sp, -N` in each prologue.

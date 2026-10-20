# V006 Performance

The performance rules for the osc servo firmware on the CH32V006, the facts they rest on, the layout traps, and how to measure a change. Read it before any performance or code-layout work on the servo image. The code-shape rules also appear in [kernel sec 8](osc-servo-kernel.md#8-code-shape-on-a-fetch-bound-chip) as part of the scheduling architecture; this doc carries the detail behind them.

Hot code means the kernel tick interrupt and its callees, the bus vector bodies, the bus frame path and the TEL (telemetry burst) lane. Everything else is cold.

Code is authoritative; when this doc and the code disagree, fix the doc.

---

## 1. The fetch model

The core is a QingKe V2 at HCLK (the core clock) 48 MHz, ISA RV32EC with Zmmul ([`riscv32ec_zmmul-unknown-none-elf.json`](../firmware/boards/osc-dev-v006/riscv32ec_zmmul-unknown-none-elf.json)). It runs every instruction from flash.

| Fact | Consequence |
| --- | --- |
| Flash is fetched one aligned 4-byte word per 3 HCLK | a 4-byte instruction costs about twice a 2-byte one |
| No instruction cache | a loop refetches its body on every pass |
| Every taken branch, jump, call and return restarts the fetch | a taken transfer costs a refetch; a branch not taken costs only its bytes |

> **Code bytes and taken transfers are execution time.**

The E (estimated) rows of the budget table price code with this model ([`firmware/lib/core/src/budget/mod.rs`](../firmware/lib/core/src/budget/mod.rs)), in one of two forms:

| Form | Rule |
| --- | --- |
| Straight runs | per run from a taken transfer to the next: 3 HCLK per flash word spanned plus 3 for the transfer into it, or 1 per instruction and 2 per load or store when that is more |
| Instruction classes | load or store 3.25, ALU 2, multiply 10, taken transfer 6, call and return 12 HCLK |

Arithmetic is rarely the cost: the whole kernel interrupt holds 74 multiply instructions, and its time goes to loads, stores, branches and flash waits ([v006-feature-budget.md](v006-feature-budget.md#transport-table-and-platform)).

### 1.1 Nothing runs from RAM

> **All code runs from flash.**

RAM holds data and the stack, and the stack is the binding limit ([v006-feature-budget.md](v006-feature-budget.md#budget-lines)). A slow body is made smaller, not moved ([kernel sec 8.5](osc-servo-kernel.md#85-nothing-runs-from-ram)).

---

## 2. Alignment

A transfer into a target at 2 mod 4 spends the first half of its first fetched word on bytes before the target, and a 4-byte instruction there spans two words: an extra fetch each time the target is reached. Two mechanisms keep targets at 0 mod 4.

### 2.1 Block alignment

The board app builds with `-C llvm-args=-align-all-nofallthru-blocks=2` (the `osc-dev-v006-app` profile in [`firmware/boards/osc-dev-v006/Cargo.toml`](../firmware/boards/osc-dev-v006/Cargo.toml); the bootloader builds without it). The value is a power of two: every basic block that its layout predecessor does not fall into starts at 0 mod 4. The padding costs flash. The flag does not touch function entries.

### 2.2 Function entries

> **Every out-of-line hot function carries `#[rustc_align(4)]` and is listed in [`scripts/check-hot-align.sh`](../scripts/check-hot-align.sh).**

- The attribute needs `#![feature(fn_align)]`, which the crates holding hot code enable (`osc-servo-core`, the drivers, the control table, the protocol, `osc-servo-ch32` and the app).
- The pin applies whether or not the function is aligned today. Layout drifts with unrelated edits and with the build path ([README, Firmware](../README.md#firmware)), so an entry aligned by luck does not stay aligned.
- The attribute does nothing on an inlined function. Pin only functions that exist as symbols in the ELF: `#[inline(never)]` bodies and the interrupt handlers.
- The four interrupt bodies share one `.trap.rust` section, so each carries its own pin ([`firmware/servo-ch32/src/runtime/isr.rs`](../firmware/servo-ch32/src/runtime/isr.rs)).
- New hot code adds the attribute and the script entry in the same change.

The script reads the linked image and fails when a listed function enters at 2 mod 4 or is missing (renamed or inlined). CI runs it on the release image and on the bench image.

---

## 3. Branch reach

A RISC-V conditional branch reaches ±4 KiB; an unconditional `j` reaches ±1 MiB. When LLVM's branch relaxation pass cannot prove a conditional branch's target is in reach, it rewrites the branch as an inverted branch over a jump:

```text
bne  a0, a1, cold           ->      beq  a0, a1, 1f
                                    j    cold
                                 1:
```

The pass works on estimated sizes, so spans a little under 4 KiB can be rewritten too. It is a compile-time pass, separate from the linker relaxation that `+relax` in the target spec enables for calls.

The cost lands on hot-to-cold branches. The original branch was not taken on the hot path and cost only its bytes; the inverted branch is taken on every execution of that path, one extra taken transfer per pass.

> **A large inlined interrupt body sits near this cliff.**

The kernel tick is inlined whole into the ADC DMA handler, 9,606 B ([v006-feature-budget.md](v006-feature-budget.md#budget-lines)). Its hot code falls through and its cold arms (`core::hint::cold_path`) are placed toward the end of the body, so many hot-to-cold spans run to several KiB. An edit that changes nothing on the hot path but grows the body can carry one of those spans past the limit, and the tick gets slower with no change to the code it runs.

### 3.1 Checking

Disassemble the handler with `llvm-objdump` from the `llvm-tools` rustup component:

```sh
llvm-objdump -d --no-show-raw-insn \
  --disassemble-symbols=__qingke_rt_DMA1_CHANNEL1 <elf> > isr.s
```

A relaxed pair is a conditional branch whose target is the instruction right after the `j` that follows it. This lists them with their spans:

```python
import re, sys
ins = []
for line in open(sys.argv[1]):
    m = re.match(r'\s+([0-9a-f]+):\s+(\S+)\s*([^<]*)', line)
    if m:
        ins.append((int(m[1], 16), m[2], m[3].split(',')[-1].strip()))
for (a, op, t), (_, op2, far), (c, _, _) in zip(ins, ins[1:], ins[2:]):
    if op.startswith('b') and op2 == 'j' and t == f'0x{c:x}':
        print(f'{a:x}: {op} over j {far} (span {int(far, 16) - a:+d} B)')
```

Compare the lists of the two images of an A/B. A pair present only in the slower image whose `j` leads to cold code (a fault arm, a rare drive kind, a slow-rate stage) is a relaxation cost on the hot path. Direct hot-to-cold branches with spans close to 4 KiB are the next to cross.

---

## 4. Code shape

The rules of [kernel sec 8.2](osc-servo-kernel.md#82-lean-code), with the forms the code uses for them.

### 4.1 The hot path falls through

The rare arm of a hot branch calls `core::hint::cold_path()`, so the common case is the fall-through and the rare case pays the taken transfer. The saturating helpers in [`firmware/lib/core/src/math.rs`](../firmware/lib/core/src/math.rs) and the drive selection in `Fast::drive` ([`firmware/lib/core/src/kernel/phases/fast.rs`](../firmware/lib/core/src/kernel/phases/fast.rs)) follow this form.

### 4.2 Clamping

Against run-time bounds, hot code clamps with `math::sat_clamp`, which is `x.max(lo).min(hi)` with each saturating arm cold, not with `i32::clamp`. `clamp` asserts `lo <= hi`, a panic path unless the bounds are constants; `sat_clamp` has none, and `hi` wins when `lo > hi`. The current loop is the user ([`firmware/lib/core/src/kernel/phases/loops/current.rs`](../firmware/lib/core/src/kernel/phases/loops/current.rs)).

### 4.3 Inline or out of line

| Form | When | Example |
| --- | --- | --- |
| `#[inline(always)]` | small hot helpers; a call and return costs 12 HCLK | `math::q_mul`, `math::sat_add` |
| `#[inline(never)]`, one body | code that inlined at several sites multiplies flash | `TelLane::feed` (inlined twice it doubles the lane's flash), `math::recip_div` |
| `#[inline(never)]`, rare work | a rare body out of a hot one leaves the hot code its registers | `TelLane::head`, once a frame, so the rows between keep their registers ([`firmware/lib/drivers/src/tel/rows.rs`](../firmware/lib/drivers/src/tel/rows.rs)) |
| `#[inline(never)]`, cold work | cold code inlined into a vector grows the vector | `ClockDiscipline::on_cal_break`, out of the TIM2 and SysTick vectors ([`firmware/lib/drivers/src/bus/clock.rs`](../firmware/lib/drivers/src/bus/clock.rs)) |

Each out-of-line hot body gets its alignment pin ([sec 2.2](#22-function-entries)).

### 4.4 Removing provably redundant checks

A bounds check, an `unwrap` or a checked operation on a hot path that can never fail may be replaced by its unchecked form, with a `SAFETY:` argument naming the invariant and a test pinning it. The registry accessors (`unwrap_unchecked` after bringup installs the cell, [`firmware/servo-ch32/src/runtime/registry.rs`](../firmware/servo-ch32/src/runtime/registry.rs)) are the pattern. Firmware never panics, and a removal never introduces undefined behaviour.

---

## 5. No `mem*` builtins

> **No runtime path calls `memcpy`, `memset`, `memcmp` or `memmove`.**

A builtin is a call, a generic dispatch on length and alignment, and an entry no `#[rustc_align]` can pin. [`.github/check-no-mem.sh`](../.github/check-no-mem.sh) fails the release or bench image when:

- any `compiler_builtins` mem routine is linked, or `memmove`, `memcmp` or `bcmp` is linked at all;
- a function other than the bringup installers (`statics::install`, `Drivers::install`) calls `memcpy` or `memset`.

LLVM lowers the aggregate moves that build the statics to `memcpy` and `memset`; [`firmware/servo-ch32/src/runtime/mem.rs`](../firmware/servo-ch32/src/runtime/mem.rs) provides both as byte loops for bringup alone.

Replacements:

| Need | Form | Example |
| --- | --- | --- |
| A word-aligned run | a word loop with volatile reads or writes | the configuration take (`KernelConfig::take`, [`firmware/lib/core/src/kernel/config.rs`](../firmware/lib/core/src/kernel/config.rs)) |
| A few halfwords | straight-line stores; a loop's setup alone outprices the copy | the TEL row's CPU chunk |
| Storage filled before it is read | `MaybeUninit`, so nothing is zeroed | the gather spans in `Dispatcher::read_profile` ([`firmware/lib/core/src/services/bus/dispatch.rs`](../firmware/lib/core/src/services/bus/dispatch.rs)) |
| A value another context owns | a borrow instead of a move or copy | |

A plain copy loop compiles back into `memcpy`; the volatile access keeps it a loop. Implicit calls count too: struct moves, array initialisation and zeroing, `copy_from_slice`, `fill`, and `==` on arrays.

---

## 6. DMA for large copies

On hot paths a large copy goes to DMA memory-to-memory, and the CPU works beside it ([kernel sec 7](osc-servo-kernel.md#7-dma-as-a-second-execution-unit)):

- one arm per contiguous run; data is laid out so runs are contiguous;
- never two arms queued on one channel;
- a copy smaller than the arm's cost stays on the CPU, and DMA cycles also stall CPU fetch on the AHB (the internal bus);
- cold paths use a plain loop.

The TEL row is the worked example: the longer chunk goes to DMA1 CH6 and the shorter, at most three halfwords, to the CPU while the DMA runs (`RowPlan`, [`firmware/lib/drivers/src/tel/rows.rs`](../firmware/lib/drivers/src/tel/rows.rs)). The ADC scans need no copy at all: DMA1 CH1 lands them in the tick block where they are read ([kernel sec 7.2](osc-servo-kernel.md#72-adc-scans-land-in-the-tick-block)).

---

## 7. Arithmetic

Zmmul is hardware 32x32 multiply, high half included, and no divide. A run-time `/` or `%`, or a 64-bit operation LLVM does not expand inline, links a `compiler_builtins` routine of hundreds of cycles. [`.github/check-soft-arith.sh`](../.github/check-soft-arith.sh) fails the image when any soft divide, modulo, multiply, or 64-bit shift, negate or compare routine is linked.

| Need | Form |
| --- | --- |
| A divisor known at build time | fold it into a constant (`KernelTiming`, `brr_for`, `tpb_for`) |
| A comparison of ratios | cross-multiply (`math::wide_ge`) |
| A fixed-point product | `math::q_mul` and `math::q_mul_u`: an i64 widening product LLVM lowers to `mul` plus `mulh` |
| A run-time quotient, slow rate or configuration only | `math::recip_div`: Newton reciprocal, accuracy pinned in its tests |

---

## 8. Measuring a change

### 8.1 Build

Every image that is flashed, priced or compared comes from [`scripts/build-image.sh`](../scripts/build-image.sh):

```sh
scripts/build-image.sh [--bench] <rev> osc-dev-v006
```

It exports the commit to one fixed path, so one commit always links one layout; a build from a working checkout can link a different one ([README, Firmware](../README.md#firmware)). Each image prints its sha1.

> **A release image byte-identical to its base needs no bench.** CI is the whole gate.

### 8.2 Static

- Sections: `llvm-size -A`; a firmware change reports its `.text` delta.
- Symbols: `llvm-nm -S --size-sort -C`.
- Disassembly: read the changed path against the fetch model ([sec 1](#1-the-fetch-model)) and price it as the E rows do; check the relaxed pairs ([sec 3.1](#31-checking)) and the alignment script on both images.

### 8.3 Bench

| Instrument | Image | Measures |
| --- | --- | --- |
| `tick_load_mean_q15` ([protocol sec 5.9](osc-native-protocol.md#59-health-osc-servo)) | release | the tick's mean share of the period; every build carries it |
| `budgets` hardware test ([`tools/bench/tests/hardware/budgets.rs`](../tools/bench/tests/hardware/budgets.rs)) | bench (`--bench`) | the budget probe's maximum and mean per kernel phase and drive arm, bus body and frame type, held against the budget table |

The budget table ([`firmware/lib/core/src/budget/mod.rs`](../firmware/lib/core/src/budget/mod.rs)) is shared by the probe, the hardware test and the DES (discrete-event simulator). A row changes only with its evidence, in the same change ([kernel sec 4.3](osc-servo-kernel.md#43-row-changes)). [testing.md](testing.md#gear-3-in-detail) describes how to run the test.

### 8.4 A/B

- Write the pass criterion and the result that would falsify it before the first run.
- Build both images with the script, flash them to the same board, and alternate A and B runs, so drift in the rig lands on both.
- Judge the difference against A's run-to-run spread.
- Layout alone moves a tick body by tens of HCLK between builds; the budget table carries `LAYOUT` (35 HCLK) on every kernel row for it ([kernel sec 4.2](osc-servo-kernel.md#42-the-layout-allowance)). A difference of that size is attributed to a code change only once the disassembly shows where it comes from.

---

## 9. Summary

1. **Code bytes and taken transfers are execution time.** Flash is fetched one 4-byte word per 3 HCLK with no cache; nothing runs from RAM.
2. **Hot targets sit at 0 mod 4.** Blocks by the board's alignment flag, function entries by `#[rustc_align(4)]` and the alignment script.
3. **Hot-to-cold spans stay under 4 KiB.** Past it the branch is relaxed into a taken branch on the hot path; count relaxed pairs in every A/B.
4. **The hot path falls through.** Rare arms are cold, clamps have no panic path, rare and cold bodies are out of line.
5. **No `mem*` builtins and no soft arithmetic.** Both are guarded in CI.
6. **Large hot copies go to DMA**, small ones to straight-line CPU stores.
7. **Measure from canonical images.** Byte-identical images need no bench; an A/B states its criterion first and reads its difference against layout noise.

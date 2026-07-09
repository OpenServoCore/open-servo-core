# Testing: the three gears

The transport is verified by three test gears, each proving something the others
structurally cannot. Green across all three — `scripts/gears.sh` — is what lets
us pronounce the tree good.

| gear | proves | where | gate |
|------|--------|-------|------|
| **1 — unit** | the pieces are individually correct: codec, CRC vectors, control-table rules, driver internals | in-crate `#[cfg(test)]` across `firmware/lib/*` and `tools/bench` | CI |
| **2 — DES** | the pieces compose correctly under adversarial sequencing, at every baud, deterministically | `firmware/lib/integration` (discrete-event sim) | CI |
| **3 — bench** | the real silicon meets timing and survives zero-gap load under real ISR/DMA/drift | `tools/bench/tests/hardware` (hardware-in-the-loop) | on-rig |

The division of labour is deliberate. **Gear 2 is wide but blind to time**: the
sim dispatches with a *zero* CPU-time model, so it proves logical correctness and
sequencing at every baud but cannot see ISR latency, wall-clock turnaround, HSI
drift, or the frame-N-deadline-vs-frame-N+1-break window. **Gear 3 is the deep-
on-timing, narrow-on-logic gear**: its whole job is the silicon-only failure
modes gear 2 erases. So for every property the sim is structurally blind to,
gear 3 carries one hardware assertion with a measured budget — it is complete on
silicon-only properties, not on logic.

## Running

```sh
scripts/gears.sh
```

Gears 1 and 2 are deterministic and need no hardware. Gear 3 needs a uart-pirate
and a flashed V006; the script auto-detects the pirate and **skips** gear 3
(rather than failing) when no rig is attached. Force-skip with `SKIP_BENCH=1`;
point at a specific pirate with `BENCH_PORT=/dev/tty…`.

The gears map to plain cargo invocations if you want to run one directly:

```sh
( cd firmware/lib && cargo test --workspace )            # gear 1 + 2
( cd tools/bench  && cargo test --lib )                  # gear 1 (bench units)
( cd tools/bench  && cargo test --test hardware -- --test-threads=1 )   # gear 3
```

## Gear 3 in detail

The hardware suite drives a single flashed V006 over the pirate and asserts on
the pirate's captured stamp stream — no wlink, no chip-counter reads; the wire is
the failure surface. It sweeps the full baud matrix (0.5M / 1M / 2M / 3M).

- **turnaround** (`turnaround.rs`) — THE metric: instruction wire-end → status
  break fall, per baud. 1 M is the tuned sweet spot (~35 µs); the ceiling is
  baud-aware because turnaround rises at both higher and lower baud on this
  silicon.
- **hot loop** (`hot_loop.rs`) — the production `[GWRITE(HOLD), COMMIT, GREAD]`
  zero-gap loop, the silicon twin of the DES `hot_loop` suite. The GREAD must
  read back the just-committed value every cycle; a stale read-back is a
  silently-dropped frame.
- **plain flood** (`hot_loop.rs`) — an aggressive `[WRITE(NOREPLY) × 8, READ]`
  flood that surfaces the low-baud framer floor.
- **ping / read / write / reg_write_action / silence** — the single-servo
  instruction-set happy path.

Longer soak: `BENCH_BURST_CYCLES=25000 scripts/gears.sh`.

### Budgets and the known floor

Burst tests gate on a **total failure rate** per scenario, not `stale == 0`. The
framer has a known low-baud frame-loss floor (task #32, OPEN) that leaves a rare
stale even on the production loop at 0.5M/1M; asserting zero would flake. The
budgets sit above the measured floor with margin — they catch a gross regression
and tighten toward zero when #32 lands. The printed per-cycle breakdown
(`stale` / `no-reply` / `other`) is the diagnostic; at 2M/3M stale is ~0.

The `tool-osc-*` binaries in `tools/bench/src/bin` are the forensic instruments
behind these tests — `tool-osc-burst` shares the exact cycle engine the hot-loop
test asserts on; `tool-reply-edges` dumps the IC edges when a reply artifact
needs root-causing.

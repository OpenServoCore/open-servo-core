#!/usr/bin/env bash
#
# Run the three test gears and pronounce the tree good only if every attached
# gear passes. See docs/testing.md for what each gear proves.
#
#   gear 1 -- unit           in-crate #[test]s: codec, CRC vectors, table rules
#   gear 2 -- DES            osc-integration discrete-event sim (deterministic)
#   gear 3 -- bench          hardware-in-the-loop against a flashed V006
#
# Gears 1 and 2 are deterministic and need no hardware. Gear 3 needs a
# uart-pirate + flashed V006; it is SKIPPED (not failed) when no rig is found.
# Force-skip with SKIP_BENCH=1; point at a specific pirate with BENCH_PORT.
set -euo pipefail
cd "$(dirname "$0")/.."

section() { printf '\n== %s ==\n' "$1"; }

# --- gear 1 + 2 : deterministic, no hardware ------------------------------
# The firmware/lib workspace test run covers every crate's in-module unit tests
# (gear 1) AND the osc-integration discrete-event suite (gear 2) in one pass.
section "gear 1+2: unit + DES (firmware/lib)"
( cd firmware/lib && cargo test --workspace )

# Chip-crate host-side unit tests (gear 1) behind the V006 feature.
section "gear 1: firmware/servo-ch32 lib"
( cd firmware/servo-ch32 && cargo test --features ch32v006x8x6 --lib )

# Bench host-library units (gear 1): wire layout + CRC vectors.
section "gear 1: bench lib"
( cd tools/bench && cargo test --lib )

# --- gear 3 : hardware-in-the-loop ----------------------------------------
section "gear 3: bench hardware"
if [ "${SKIP_BENCH:-0}" = "1" ]; then
  echo "  SKIP_BENCH=1; skipping gear 3"
elif [ -n "${BENCH_PORT:-}" ] \
  || ls /dev/cu.usbmodem* >/dev/null 2>&1 \
  || ls /dev/ttyACM* >/dev/null 2>&1; then
  # #[serial] already serialises the shared pirate; --test-threads=1 is belt
  # and suspenders. A longer soak: BENCH_BURST_CYCLES=25000.
  ( cd tools/bench && cargo test --test hardware -- --test-threads=1 )
else
  echo "  no bench rig detected (set BENCH_PORT to force); skipping gear 3"
fi

section "all attached gears green"

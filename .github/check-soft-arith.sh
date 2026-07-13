#!/usr/bin/env bash
#
# Gate: the firmware image must link NO soft integer arithmetic routine.
#
# The board targets rv32ec + zmmul: hardware 32x32 multiply, but NO hardware
# divide. A runtime `/` or `%`, or any 64-bit `*` / `<<`, therefore pulls in a
# compiler-builtins routine (__udivsi3, __umodsi3, __muldi3, __ashldi3, ...) --
# hundreds of wasted cycles plus extra flash. The transport keeps every divisor
# compile-time-constant (const{} folding: see `brr_for` / `tpb_for`) and all
# arithmetic <= 32-bit, so none should ever be linked. This locks that in.
#
# Usage: .github/check-soft-arith.sh [ELF]
#   Run from the repo root; ELF defaults to the release app image.
set -euo pipefail

elf="${1:-firmware/boards/osc-dev-v006/target/riscv32ec_zmmul-unknown-none-elf/release/osc-dev-v006-app}"
if [[ ! -f "$elf" ]]; then
  echo "error: ELF not found: $elf" >&2
  echo "  build it first: cargo build --release -p osc-dev-v006-app \\" >&2
  echo "    --manifest-path firmware/boards/osc-dev-v006/Cargo.toml" >&2
  exit 2
fi

# Prefer llvm-nm from the active toolchain's llvm-tools component, then
# cargo-binutils' rust-nm, then a system nm.
host="$(rustc -vV | sed -n 's/^host: //p')"
nm="$(rustc --print sysroot)/lib/rustlib/${host}/bin/llvm-nm"
if [[ ! -x "$nm" ]]; then
  nm="$(command -v rust-nm || command -v llvm-nm || command -v nm || true)"
fi
if [[ -z "$nm" ]]; then
  echo "error: no nm found (add the 'llvm-tools' rustup component)" >&2
  exit 2
fi

# compiler-builtins soft routines that must never appear:
#   div/rem (all widths), multiply (32-bit is hardware zmmul; 64/128-bit are
#   always soft), and 64/128-bit shift/negate/compare (they imply >32-bit math).
hits="$("$nm" "$elf" \
  | awk '{print $NF}' \
  | grep -E '^__(u?div|u?mod|mul)[a-z]*[0-9]+$|^__(ashl|ashr|lshr|neg|cmp|ucmp)(di|ti)[0-9]+$' \
  | sort -u || true)"

if [[ -n "$hits" ]]; then
  {
    echo "FAIL: $elf links soft integer arithmetic:"
    sed 's/^/    /' <<<"$hits"
    echo
    echo "rv32ec+zmmul has no hardware divide. Keep divisors compile-time"
    echo "constant (const{} -- see brr_for / tpb_for) and arithmetic <= 32-bit."
  } >&2
  exit 1
fi

echo "OK: no soft div/mod/mul or 64-bit arithmetic in $elf"

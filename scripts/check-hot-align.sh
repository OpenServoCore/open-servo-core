#!/usr/bin/env bash
#
# Gate: every hot out-of-line function of the tick and bus paths enters at
# 0 mod 4.
#
# The V006 fetches 4 B words with no cache, so an entry at 2 mod 4 can cost an
# extra fetch on every call. Each function below carries #[rustc_align(4)];
# this checks the linked image honors it and that no listed symbol is gone
# (a rename must update this list).
#
# Usage: scripts/check-hot-align.sh [ELF]
#   Run from the repo root; ELF defaults to the release app image.
set -euo pipefail

# Demangled path suffixes, generic arguments dropped.
hot=(
    Kernel::take_config
    Kernel::configure
    TelLane::pickup
    TelLane::head
    TelLane::row
    TelLane::service
    TelLane::start
    TelLane::feed
    scan::swap_taps
    burst::on_dma_event
    burst::handshake
    WindingTherm::install_reference
    TrajGen::step_position
    DutyLimiter::step
    __qingke_rt_DMA1_CHANNEL1
    __qingke_rt_TIM2
    __qingke_rt_USART1
    __qingke_rt_SysTick
)

elf="${1:-firmware/boards/osc-dev-v006/target/riscv32ec_zmmul-unknown-none-elf/release/osc-dev-v006-app}"
if [[ ! -f "$elf" ]]; then
  echo "error: ELF not found: $elf" >&2
  exit 2
fi

host="$(rustc -vV | sed -n 's/^host: //p')"
nm="$(rustc --print sysroot)/lib/rustlib/${host}/bin/llvm-nm"
if [[ ! -x "$nm" ]]; then
  nm="$(command -v rust-nm || command -v llvm-nm || true)"
fi
if [[ -z "$nm" ]]; then
  echo "error: no llvm-nm found (add the 'llvm-tools' rustup component)" >&2
  exit 2
fi

# "<addr> <name>" per text symbol, names demangled and generic-free.
syms="$("$nm" -C --defined-only "$elf" \
  | awk '$2 ~ /^[tT]$/ { a = $1; $1 = $2 = ""; sub(/^ +/, ""); print a, $0 }' \
  | perl -pe '1 while s/(\w)<[^<>]*>/$1/; s/ <(.*)>::/ $1::/')"

fail=0
for f in "${hot[@]}"; do
  hits="$(awk -v f="$f" '{ n = $2; if (n == f || substr(n, length(n) - length(f) - 1) == "::" f) print }' <<<"$syms")"
  if [[ -z "$hits" ]]; then
    echo "FAIL: $f not found in $elf (renamed or inlined?)" >&2
    fail=1
    continue
  fi
  while read -r addr name; do
    if (( 16#$addr % 4 != 0 )); then
      echo "FAIL: $name at 0x$addr, not 0 mod 4" >&2
      fail=1
    fi
  done <<<"$hits"
done

if (( fail )); then
  exit 1
fi
echo "OK: ${#hot[@]} hot callees enter at 0 mod 4 in $elf"

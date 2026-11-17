#!/usr/bin/env bash
#
# Gate: no firmware function reaches the panic handler.
#
# The firmware never panics: panic-halt freezes the servo with the bridge
# wherever it was. A panic path is an assert, an unwrap, `Ord::clamp` on
# bounds the compiler cannot prove ordered (use `max().min()`), or an index
# it cannot prove in range. A panic entry is a core function that reaches
# `rust_begin_unwind`; this fails when a firmware function calls one, except
# for the index checks listed in `allowed`, each at most its count of call
# sites. The listed ones are open debts: shrink the list as they go.
#
# Usage: .github/check-no-panic.sh [ELF]
#   Run from the repo root; ELF defaults to the release app image.
set -euo pipefail

# "caller|panic entry|max call sites"; caller is a demangled path suffix,
# generic arguments dropped. Only index checks belong here.
allowed=(
    "__qingke_rt_SysTick|core::panicking::panic_bounds_check|1"
    "__qingke_rt_TIM2|core::panicking::panic_bounds_check|2"
    "ServoBus::drive_framer|core::panicking::panic_bounds_check|1"
    "ServoBus::crc_feed|core::slice::index::slice_index_fail|3"
    "ServoBus::route_chain|core::slice::index::slice_index_fail|1"
    "ServoBus::route_frame|core::slice::index::slice_index_fail|2"
    "Dispatch::dispatch|core::slice::index::slice_index_fail|1"
)

elf="${1:-firmware/boards/osc-dev-v006/target/riscv32ec_zmmul-unknown-none-elf/release/osc-dev-v006-app}"
if [[ ! -f "$elf" ]]; then
  echo "error: ELF not found: $elf" >&2
  exit 2
fi

host="$(rustc -vV | sed -n 's/^host: //p')"
objdump="$(rustc --print sysroot)/lib/rustlib/${host}/bin/llvm-objdump"
if [[ ! -x "$objdump" ]]; then
  echo "error: llvm-objdump not found (add the 'llvm-tools' rustup component)" >&2
  exit 2
fi

# "<caller>|<panic entry>" per call or branch into a panic entry, demangled,
# caller generic-free.
calls="$("$objdump" -d --no-show-raw-insn -C "$elf" \
  | awk '
      /^[0-9a-f]+ <.*>:$/ { fn = $0; sub(/^[0-9a-f]+ </, "", fn); sub(/>:$/, "", fn); next }
      /^ +[0-9a-f]+:/ && match($0, /0x[0-9a-f]+ </) {
        t = substr($0, RSTART + RLENGTH); sub(/>$/, "", t)
        if (t ~ /\+0x[0-9a-f]+$/ || t == fn) next
        if (!((fn SUBSEP t) in seen)) { seen[fn, t] = 1; from[++n] = fn; to[n] = t }
        hits[fn, t]++
      }
      END {
        for (i = 1; i <= n; i++) if (to[i] ~ /(^|::)rust_begin_unwind$/) panic[to[i]] = 1
        do {
          grew = 0
          for (i = 1; i <= n; i++)
            if ((to[i] in panic) && !(from[i] in panic) && from[i] ~ /^(core|__rustc)::/) {
              panic[from[i]] = 1; grew = 1
            }
        } while (grew)
        for (i = 1; i <= n; i++)
          if ((to[i] in panic) && !(from[i] in panic))
            for (k = 0; k < hits[from[i], to[i]]; k++) print from[i] "|" to[i]
      }' \
  | perl -pe '1 while s/(\w)<[^<>]*>/$1/ || s/::<[^<>]*>//; s/^<(.*)>::/$1::/' \
  | sort | uniq -c)"

fail=0
while read -r count call; do
  [[ -z "${call:-}" ]] && continue
  f="${call%%|*}"
  t="${call#*|}"
  max=0
  for a in "${allowed[@]}"; do
    IFS='|' read -r ac at am <<<"$a"
    if [[ "$t" == "$at" && ( "$f" == "$ac" || "$f" == *"::$ac" ) ]]; then
      max=$am
    fi
  done
  if (( count > max )); then
    echo "FAIL: $f calls $t at $count site(s), allowed $max" >&2
    fail=1
  fi
done <<<"$calls"

if (( fail )); then
  echo >&2
  echo "Find the site with llvm-objdump -d -l on the ELF. Clamp with" >&2
  echo "max().min(), match on Option/Result, or bound the index so the" >&2
  echo "compiler can see it (a mask, a get, a fixed-size array)." >&2
  exit 1
fi
echo "OK: no panic path outside the listed index checks in $elf"

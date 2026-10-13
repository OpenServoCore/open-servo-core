#!/usr/bin/env bash
#
# Gate: no runtime path of the firmware calls a mem* routine.
#
# On the V006 a memcpy/memset/memcmp call is a call, a generic length and
# alignment dispatch, and an entry no #[rustc_align] can pin. Runtime copies
# are typed stores or project loops instead. The one exception is bringup:
# LLVM lowers the aggregate moves that build the statics to memcpy/memset,
# served by the byte loops in servo-ch32's runtime/mem.rs. So this fails when
#   - any compiler_builtins mem routine is linked (runtime/mem.rs must be the
#     one providing memcpy/memset), or memmove/memcmp/bcmp is linked at all;
#   - a function outside `allowed` calls memcpy or memset.
#
# Usage: .github/check-no-mem.sh [ELF]
#   Run from the repo root; ELF defaults to the release app image.
set -euo pipefail

# Bringup only, demangled path suffixes, generic arguments dropped.
allowed=(
    statics::install
    Drivers::install
)

elf="${1:-firmware/boards/osc-dev-v006/target/riscv32ec_zmmul-unknown-none-elf/release/osc-dev-v006-app}"
if [[ ! -f "$elf" ]]; then
  echo "error: ELF not found: $elf" >&2
  exit 2
fi

host="$(rustc -vV | sed -n 's/^host: //p')"
bin="$(rustc --print sysroot)/lib/rustlib/${host}/bin"
nm="$bin/llvm-nm"
objdump="$bin/llvm-objdump"
if [[ ! -x "$nm" || ! -x "$objdump" ]]; then
  echo "error: llvm-nm/llvm-objdump not found (add the 'llvm-tools' rustup component)" >&2
  exit 2
fi

fail=0

linked="$("$nm" -C --defined-only "$elf" \
  | awk '{ $1 = $2 = ""; sub(/^ +/, ""); print }' \
  | grep -E '^compiler_builtins::mem::|^(memmove|memcmp|bcmp)$' | sort -u || true)"
if [[ -n "$linked" ]]; then
  echo "FAIL: $elf links:" >&2
  sed 's/^/    /' <<<"$linked" >&2
  fail=1
fi

# "<caller>" per call or tail call to memcpy/memset, demangled, generic-free.
callers="$("$objdump" -d --no-show-raw-insn -C "$elf" \
  | awk '/^[0-9a-f]+ <.*>:$/ { fn = $0; sub(/^[0-9a-f]+ </, "", fn); sub(/>:$/, "", fn); next }
         ($2 == "jal" || $2 == "j") && $NF ~ /^<(memcpy|memset)>$/ { print fn }' \
  | perl -pe '1 while s/(\w)<[^<>]*>/$1/ || s/::<[^<>]*>//; s/^<(.*)>::/$1::/' \
  | sort -u)"
while read -r f; do
  [[ -z "$f" ]] && continue
  ok=0
  for a in "${allowed[@]}"; do
    if [[ "$f" == "$a" || "$f" == *"::$a" ]]; then
      ok=1
    fi
  done
  if (( ! ok )); then
    echo "FAIL: $f calls memcpy/memset" >&2
    fail=1
  fi
done <<<"$callers"

if (( fail )); then
  echo >&2
  echo "Write the copy as typed stores, borrow instead of moving, or use a" >&2
  echo "loop whose stores are volatile (a plain copy loop compiles to memcpy)." >&2
  exit 1
fi
echo "OK: no mem* call outside bringup in $elf"

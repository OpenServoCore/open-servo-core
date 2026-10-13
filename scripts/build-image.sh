#!/usr/bin/env bash
#
# Build the flash images of one commit from one canonical path.
#
#   scripts/build-image.sh [--bench] <rev> [board...]
#
# Cargo hashes the absolute path of every path dependency outside the
# workspace root into crate metadata. The hash lands in symbol names, and
# rustc orders functions by them, so the same commit built from two
# checkout paths links a different layout. Every release, bench and
# pricing build goes through this script so they share one path.
#
# Boards: osc-dev-v006 (app + boot), osc-adapter-wchlinke; default all.
# --bench adds the osc-dev-v006 app built with the bench feature.
# Images land in $OSC_BUILD_ROOT/out/<rev>/ as <image>.elf, .bin and .sha1;
# each prints as `<board> <image> <sha1>`.
set -euo pipefail

usage() { echo "usage: $0 [--bench] <rev> [osc-dev-v006|osc-adapter-wchlinke ...]" >&2; exit 2; }

bench=0
if [ "${1:-}" = "--bench" ]; then bench=1; shift; fi
[ $# -ge 1 ] || usage
rev_arg=$1; shift
boards=("$@")
[ ${#boards[@]} -gt 0 ] || boards=(osc-dev-v006 osc-adapter-wchlinke)
for board in "${boards[@]}"; do
    case $board in osc-dev-v006|osc-adapter-wchlinke) ;; *) usage ;; esac
done

repo=$(cd "$(dirname "$0")/.." && pwd)
root=${OSC_BUILD_ROOT:-$HOME/.cache/osc-build}
rev=$(git -C "$repo" rev-parse --short=8 "$rev_arg^{commit}")
src=$root/src
out=$root/out/$rev

mkdir -p "$root"
lock=$root/lock
mkdir "$lock" 2>/dev/null || { echo "$lock exists: another build is running (or remove it)" >&2; exit 1; }
trap 'rm -rf "$lock"' EXIT

rm -rf "$src"
mkdir -p "$src"
git -C "$repo" archive "$rev" | tar -x -C "$src"
mkdir -p "$out"

# build <board> <bin> <image> <triple> <cargo args...>
build() {
    local board=$1 bin=$2 image=$3 triple=$4; shift 4
    local dir=$src/firmware/boards/$board target=$root/target/$board
    (cd "$dir" && CARGO_TARGET_DIR=$target cargo build --release --locked "$@")
    local objcopy
    objcopy=$(find "$(cd "$dir" && rustc --print sysroot)" "$(cd / && rustc --print sysroot)" \
        -name llvm-objcopy -type f | head -1)
    [ -n "$objcopy" ] || { echo "no llvm-objcopy: rustup component add llvm-tools" >&2; exit 1; }
    cp "$target/$triple/release/$bin" "$out/$image.elf"
    "$objcopy" -O binary "$out/$image.elf" "$out/$image.bin"
    echo "$board $image $(shasum "$out/$image.bin" | cut -d' ' -f1)" > "$out/$image.sha1"
    lines+=("$(cat "$out/$image.sha1")")
}

lines=()

for board in "${boards[@]}"; do
    case $board in
        osc-dev-v006)
            t=riscv32ec_zmmul-unknown-none-elf
            build "$board" osc-dev-v006-app osc-dev-v006-app $t -p osc-dev-v006-app
            build "$board" osc-dev-v006-boot osc-dev-v006-boot $t -p osc-dev-v006-boot
            if [ $bench = 1 ]; then
                build "$board" osc-dev-v006-app osc-dev-v006-app-bench $t \
                    -p osc-dev-v006-app --features osc-dev-v006-app/bench
            fi
            ;;
        osc-adapter-wchlinke)
            build "$board" osc-adapter-wchlinke osc-adapter-wchlinke riscv32imc-unknown-none-elf
            ;;
    esac
done

echo "$rev -> $out"
printf '%s\n' "${lines[@]}"

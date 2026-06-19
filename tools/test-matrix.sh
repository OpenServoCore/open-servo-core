#!/usr/bin/env bash
# Local mirror of .github/workflows/ci.yml `baud-rdt-matrix`.
#
# Each combo gets its own CARGO_TARGET_DIR because the sim defaults
# (`firmware/lib/integration/src/sim/defaults.rs`) bake env vars in at
# compile time via `option_env!`. A shared target dir would either
# block on cargo's lock or rebuild on every iteration; a per-combo
# dir lets `JOBS > 1` actually run in parallel.
#
# Usage:
#   tools/test-matrix.sh             # JOBS = ceil(nproc/2)
#   JOBS=8 tools/test-matrix.sh      # explicit parallelism
#
# Edit RDTS / BAUDS below to change coverage.

set -u

ROOT=$(git rev-parse --show-toplevel)
NPROC=$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4)
JOBS=${JOBS:-$(( (NPROC + 1) / 2 ))}
[ "$JOBS" -ge 1 ] || JOBS=1

RDTS=(0 128 256 384 510)
BAUDS=(
    "0:9600"
    "1:57600"
    "2:115200"
    "3:1M"
    "4:2M"
    "5:3M"
)

TARGET_BASE=$ROOT/target/matrix
LOG_DIR=$TARGET_BASE/logs
mkdir -p "$LOG_DIR"

run_one() {
    local idx=$1 name=$2 rdt=$3
    local tag="b${idx}_r${rdt}"
    local target="$TARGET_BASE/$tag"
    local log="$LOG_DIR/$tag.log"
    mkdir -p "$target"
    if CARGO_TARGET_DIR="$target" \
       OSC_TEST_BAUD_IDX="$idx" \
       OSC_TEST_RDT_US="$rdt" \
       cargo test --workspace --manifest-path "$ROOT/firmware/lib/Cargo.toml" \
       >"$log" 2>&1; then
        printf 'PASS  baud=%-7s rdt=%4s\n' "$name" "$rdt"
    else
        printf 'FAIL  baud=%-7s rdt=%4s  log=%s\n' "$name" "$rdt" "$log"
        return 1
    fi
}
export -f run_one
export ROOT TARGET_BASE LOG_DIR

combos=()
for b in "${BAUDS[@]}"; do
    idx=${b%%:*}
    name=${b#*:}
    for rdt in "${RDTS[@]}"; do
        combos+=("$idx|$name|$rdt")
    done
done

printf 'running %d combos × %d jobs (target=%s)\n' \
    "${#combos[@]}" "$JOBS" "$TARGET_BASE"

if command -v parallel >/dev/null 2>&1; then
    printf '%s\n' "${combos[@]}" | \
        parallel --jobs "$JOBS" --colsep '\|' --line-buffer \
            'run_one {1} {2} {3}'
    rc=$?
else
    printf '%s\n' "${combos[@]}" | \
        xargs -P "$JOBS" -I{} bash -c '
            IFS="|" read -r i n r <<<"$0"
            run_one "$i" "$n" "$r"
        ' {}
    rc=$?
fi

printf '\ndone (exit=%d). logs in %s\n' "$rc" "$LOG_DIR"
exit $rc

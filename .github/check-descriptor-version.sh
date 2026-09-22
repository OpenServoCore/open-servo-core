#!/usr/bin/env bash
#
# Gate: a control-table change must carry the matching firmware_version
# bump (protocol sec 5.4). Per model directory under descriptors/, the
# newest major.minor in the working tree is compared with the newest on
# the baseline: fields removed or changed need a MAJOR bump, fields added
# need a MINOR bump, and a MINOR bump may only add. MAJOR 0 promises no
# compatibility, so it skips the gate; a model absent from the baseline
# is new and passes.
#
# Usage: .github/check-descriptor-version.sh
#   Run from the repo root. MAIN_REF (default origin/main) names the
#   baseline; MAIN_DIR reads the baseline descriptors from that directory
#   instead of git (local testing).
set -euo pipefail

main_ref="${MAIN_REF:-origin/main}"
main_json="$(mktemp)"
trap 'rm -f "$main_json"' EXIT

# Newest "MAJOR.MINOR" among "MAJOR.MINOR.json" names on stdin.
newest() { sed -n 's#^.*/##; s/^\([0-9]*\.[0-9]*\)\.json$/\1/p' | sort -t. -k1,1n -k2,2n | tail -1; }

baseline_list() {
  if [[ -n "${MAIN_DIR:-}" ]]; then
    ls "$MAIN_DIR/$1" 2>/dev/null || true
  else
    git ls-tree --name-only "$main_ref" "descriptors/$1/" 2>/dev/null || true
  fi
}

baseline_read() {
  if [[ -n "${MAIN_DIR:-}" ]]; then
    cat "$MAIN_DIR/$1/$2.json"
  else
    git show "$main_ref:descriptors/$1/$2.json"
  fi
}

# compare <same|minor> <baseline.json> <tree.json>: diff the field arrays
# and apply the bump rule for that relation.
compare() {
  python3 - "$1" "$2" "$3" <<'PY'
import json, sys

mode, main_path, tree_path = sys.argv[1:4]
KEYS = ("addr", "width", "kind", "access", "min", "max", "variants")

def fields(path):
    with open(path) as f:
        return {d["name"]: {k: d.get(k) for k in KEYS} for d in json.load(f)["fields"]}

main, tree = fields(main_path), fields(tree_path)
removed = sorted(set(main) - set(tree))
added = sorted(set(tree) - set(main))
changed = sorted(n for n in main if n in tree and main[n] != tree[n])
for label, names in (("removed", removed), ("changed", changed), ("added", added)):
    if names:
        print(f"  {label}: {', '.join(names)}")
breaking = bool(removed or changed)
if mode == "same":
    if breaking:
        sys.exit("FAIL: breaking table change requires a major bump")
    if added:
        sys.exit("FAIL: additive table change requires a minor bump")
    print("OK: table unchanged")
else:
    if breaking:
        sys.exit("FAIL: breaking change under a minor bump")
    print("OK: additive change under a minor bump")
PY
}

for dir in descriptors/*/; do
  model="$(basename "$dir")"
  tree="$(ls "$dir" | newest)"
  [[ -n "$tree" ]] || continue
  tree_major="${tree%%.*}" tree_minor="${tree##*.}"
  echo "$model: tree $tree"
  if [[ "$tree_major" == 0 ]]; then
    echo "OK: 0.x promises no compatibility, gate skipped"
    continue
  fi
  main="$(baseline_list "$model" | newest)"
  if [[ -z "$main" ]]; then
    echo "OK: no baseline descriptor on $main_ref, new model"
    continue
  fi
  main_major="${main%%.*}" main_minor="${main##*.}"
  echo "$model: baseline $main"
  baseline_read "$model" "$main" >"$main_json"
  if [[ "$tree" == "$main" ]]; then
    compare same "$main_json" "$dir$tree.json"
  elif (( tree_major > main_major )); then
    echo "OK: major bump"
  elif (( tree_major == main_major && tree_minor > main_minor )); then
    compare minor "$main_json" "$dir$tree.json"
  else
    echo "FAIL: tree $tree is behind baseline $main" >&2
    exit 1
  fi
done

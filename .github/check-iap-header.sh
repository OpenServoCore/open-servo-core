#!/usr/bin/env bash
# The WCH IAP loader boots the APP slot only when image offsets 0x50/0x54
# hold its validity words (0x12345678 / 0x39373533 LE). The board plants
# them through vector entries 20/21; this gate asserts nothing in the link
# ever moves them.
set -euo pipefail

ELF=firmware/boards/osc-adapter-wchlinke/target/riscv32imc-unknown-none-elf/release/osc-adapter-wchlinke
OBJCOPY=$(find "$(rustc --print sysroot)" -name llvm-objcopy | head -1)

BIN=$(mktemp)
trap 'rm -f "$BIN"' EXIT
"$OBJCOPY" -O binary "$ELF" "$BIN"

HEADER=$(od -An -tx1 -j 80 -N 8 "$BIN" | tr -d ' \n')
if [ "$HEADER" != "7856341233353739" ]; then
    echo "IAP magic missing at 0x50/0x54: got '$HEADER'" >&2
    exit 1
fi
echo "iap header OK: 0x50/0x54 hold the loader validity words"

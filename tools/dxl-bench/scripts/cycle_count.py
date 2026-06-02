#!/usr/bin/env python3
"""Count qingke V2 HCLK cycles from PFIC trap entry to first GPIO BSHR write.

Usage:
    cycle_count.py --elf <path> --start <symbol> [--highcode] [--hclk-mhz F]

Disassembles the given ELF with `rust-objdump`, walks instructions starting
at `--start` (the ISR trampoline symbol, e.g. `SysTick`), follows
unconditional jumps (jal / c.jal / j / jr / c.jr / c.j) through the
qingke-rt trampoline chain into the user ISR body, and stops at the first
store whose base register is NOT `sp` — assumed to be the GPIO BSHR write
emitted by `stat_high()` at the top of the ISR body.

Conditional branches are treated as fall-through (we count the fastest path
to stat_high). Function-call returns are not followed; the user ISR body is
inlined into the qingke-rt trampoline by the `#[interrupt]` macro at the
indirect-jump site, not at a call site.

Memory model (highcode vs lowcode):
    --highcode    ISR runs from SRAM (0x20000000); single-cycle fetch.
    (default)     ISR runs from flash; per CH32V006 RM §18.3.1, the flash
                  has wait states that depend on HCLK:
                      HCLK ≤ 15 MHz       → 0 WS
                      15 < HCLK ≤ 24 MHz  → 1 WS
                      24 < HCLK ≤ 48 MHz  → 2 WS
                  The I-Code bus has a prefetcher (RM §1: "prefetching is
                  done on this bus"), so sequential fetches amortize to zero
                  penalty — only TAKEN control-flow transfers (jal / j / jr
                  / jalr) pay the WS refill cost. Conditional branches we
                  walk through are by definition not-taken on our fast path
                  → no penalty. Override WS with --ws.

Hardware-prologue (HPE) cycles are added as a separate constant. The qingke
V2 core with `intsyscr=0x3` performs hardware context push on trap entry —
HPE saves x1, x5-x7, x10-x15 (10 regs) to sp[-48..0] (RM §3.4). Default: 7
cycles (conservative estimate; tune via --hpe).
"""

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass


CYCLES: dict[str, int] = {
    # ALU / logical / shift (1 cycle)
    "add": 1, "addi": 1, "sub": 1, "neg": 1,
    "and": 1, "andi": 1, "or": 1, "ori": 1, "xor": 1, "xori": 1, "not": 1,
    "sll": 1, "slli": 1, "srl": 1, "srli": 1, "sra": 1, "srai": 1,
    "slt": 1, "sltu": 1, "slti": 1, "sltiu": 1, "seqz": 1, "snez": 1,
    "lui": 1, "auipc": 1, "li": 1, "mv": 1, "nop": 1,
    # Loads — 2 cycles (1 issue + 1 wait)
    "lw": 2, "lh": 2, "lb": 2, "lhu": 2, "lbu": 2,
    # Stores — 1 cycle (pipelined writeback)
    "sw": 1, "sh": 1, "sb": 1,
    # Jumps — 2 cycles (pipeline flush)
    "jal": 2, "jalr": 2, "j": 2, "jr": 2, "ret": 2, "call": 2, "tail": 2,
    # Branches — taken=2, not-taken=1. We assume not-taken (fast path).
    "beq": 1, "bne": 1, "blt": 1, "bge": 1, "bltu": 1, "bgeu": 1,
    "beqz": 1, "bnez": 1, "bgez": 1, "bltz": 1, "bgtz": 1, "blez": 1,
    "bgt": 1, "ble": 1, "bgtu": 1, "bleu": 1,
    # CSR ops — 2 cycles
    "csrr": 2, "csrw": 2, "csrs": 2, "csrc": 2,
    "csrrw": 2, "csrrs": 2, "csrrc": 2,
    "csrwi": 2, "csrsi": 2, "csrci": 2,
    "csrrwi": 2, "csrrsi": 2, "csrrci": 2,
    # Privileged
    "mret": 4, "sret": 4, "wfi": 1, "wfe": 1, "fence": 1, "fence.i": 1,
    "ecall": 4, "ebreak": 4,
    # M-extension (V006 has Zmmul: multiplies only)
    "mul": 2, "mulh": 2, "mulhsu": 2, "mulhu": 2,
}


UNCOND_JUMPS = {"jal", "j", "jr", "jalr", "tail", "call"}
STORES = {"sw", "sh", "sb"}


@dataclass(frozen=True)
class Instr:
    addr: int
    mnemonic: str
    operands: str
    raw: str


LINE_RE = re.compile(
    r"^\s*([0-9a-fA-F]+):\s+[0-9a-fA-F]+\s+\t?([a-z][\w.]*)\s*(.*?)\s*$"
)
TARGET_RE = re.compile(r"0x([0-9a-fA-F]+)")
SYMBOL_RE = re.compile(r"^([0-9a-fA-F]+)\s+<([^>]+)>:\s*$")
SYMTAG_RE = re.compile(r"<([^>+]+)(?:\+0x([0-9a-fA-F]+))?>")


def disassemble(elf: str, tool: str) -> tuple[dict[int, Instr], dict[str, int]]:
    """Return (addr→Instr, symbol→addr) for every instruction in the ELF."""
    out = subprocess.check_output(
        [tool, "-d", elf], stderr=subprocess.DEVNULL,
    ).decode("utf-8", errors="replace")

    instrs: dict[int, Instr] = {}
    symbols: dict[str, int] = {}
    for line in out.splitlines():
        m_sym = SYMBOL_RE.match(line)
        if m_sym:
            symbols[m_sym.group(2)] = int(m_sym.group(1), 16)
            continue
        m = LINE_RE.match(line)
        if not m:
            continue
        addr = int(m.group(1), 16)
        mnemonic = m.group(2).lower()
        # Strip compressed prefix; cycles are identical.
        if mnemonic.startswith("c."):
            mnemonic = mnemonic[2:]
        operands = m.group(3).strip()
        instrs[addr] = Instr(addr, mnemonic, operands, line.strip())
    return instrs, symbols


def parse_target(operands: str, symbols: dict[str, int]) -> int | None:
    """Pull the target address from a jump's operands.

    `jal 0x… <sym>`            — `0x…` is the resolved absolute target.
    `j   0x… <sym+0xNN>`       — same; `0x…` is resolved.
    `jr  0x…(reg) <sym(+off)>` — PC-relative via register; use `<sym(+off)>`.

    Prefer the `<sym(+off)>` tag when present (already resolved by objdump);
    fall back to the first `0x…` literal otherwise.
    """
    m_sym = SYMTAG_RE.search(operands)
    if m_sym:
        name = m_sym.group(1)
        off = int(m_sym.group(2), 16) if m_sym.group(2) else 0
        if name in symbols:
            return symbols[name] + off
    m = TARGET_RE.search(operands)
    return int(m.group(1), 16) if m else None


def base_reg_of_store(operands: str) -> str | None:
    """For `sw rs, imm(rd)` return `rd` (the base register)."""
    m = re.search(r",\s*-?(?:0x)?[0-9a-fA-F]+\(([a-z][a-z0-9]+)\)", operands)
    return m.group(1) if m else None


@dataclass
class WalkResult:
    base_cycles: int          # excludes flash WS penalty
    branch_penalty_cycles: int  # WS × (#taken transfers)
    taken_transfers: int
    trace: list[Instr]
    end: Instr


def walk(
    instrs: dict[int, Instr],
    symbols: dict[str, int],
    start: int,
    ws: int,
    max_steps: int = 256,
) -> WalkResult:
    """Walk from `start` along the fast path; stop at first non-sp store.

    Counts base CPU cycles per instruction. For each unconditional jump
    taken, also accrues `ws` cycles of prefetch-refill penalty (relevant
    only when running from flash; caller passes 0 for highcode).
    """
    pc = start
    base = 0
    penalty = 0
    transfers = 0
    trace: list[Instr] = []

    for _ in range(max_steps):
        if pc not in instrs:
            raise RuntimeError(f"PC 0x{pc:x} not in disassembly")
        ins = instrs[pc]
        trace.append(ins)
        base += CYCLES.get(ins.mnemonic, 1)

        if ins.mnemonic in STORES:
            if base_reg_of_store(ins.operands) != "sp":
                return WalkResult(base, penalty, transfers, trace, ins)
            pc = next_addr(instrs, pc)
            continue

        if ins.mnemonic in ("ret", "mret"):
            raise RuntimeError(
                f"hit {ins.mnemonic} at 0x{pc:x} before any non-sp store"
            )

        if ins.mnemonic in UNCOND_JUMPS:
            target = parse_target(ins.operands, symbols)
            if target is None:
                raise RuntimeError(
                    f"unconditional jump at 0x{pc:x} with no parsable target: "
                    f"{ins.operands!r}"
                )
            penalty += ws
            transfers += 1
            pc = target
            continue

        # Conditional branch or anything else: fall through.
        pc = next_addr(instrs, pc)

    raise RuntimeError(f"walked {max_steps} instructions without finding marker")


def next_addr(instrs: dict[int, Instr], pc: int) -> int:
    """Return the next instruction address after pc, in address order."""
    addrs = sorted(instrs.keys())
    idx = addrs.index(pc)
    if idx + 1 >= len(addrs):
        raise RuntimeError(f"no instruction after 0x{pc:x}")
    return addrs[idx + 1]


def v006_default_ws(hclk_mhz: float) -> int:
    """Flash wait-state count per CH32V006 RM §18.3.1 LATENCY field."""
    if hclk_mhz <= 15.0:
        return 0
    if hclk_mhz <= 24.0:
        return 1
    return 2  # 24 < HCLK ≤ 48 MHz


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--elf", required=True, help="ELF path")
    ap.add_argument("--start", required=True, help="ISR entry symbol")
    ap.add_argument("--highcode", action="store_true",
                    help="ISR runs from SRAM (0 WS); default is flash")
    ap.add_argument("--hclk-mhz", type=float, default=48.0,
                    help="HCLK in MHz (default: 48); also picks default WS")
    ap.add_argument("--ws", type=int, default=None,
                    help="Flash wait states (default: from --hclk-mhz per V006 table)")
    ap.add_argument("--hpe", type=int, default=7,
                    help="Hardware-prologue cycles (default: 7)")
    ap.add_argument("--objdump", default=None,
                    help="objdump tool (default: rust-objdump)")
    ap.add_argument("--trace", action="store_true",
                    help="Print walked instructions")
    args = ap.parse_args()

    tool = args.objdump or shutil.which("rust-objdump")
    if not tool:
        print("error: rust-objdump not found in PATH "
              "(install with `cargo install cargo-binutils && "
              "rustup component add llvm-tools-preview`)",
              file=sys.stderr)
        return 1

    if args.highcode:
        ws = 0
        mem_label = "highcode (SRAM)"
    else:
        ws = args.ws if args.ws is not None else v006_default_ws(args.hclk_mhz)
        mem_label = f"flash, {ws} WS @ {args.hclk_mhz} MHz"

    instrs, symbols = disassemble(args.elf, tool)
    if args.start not in symbols:
        candidates = [s for s in symbols if args.start in s]
        print(f"error: symbol {args.start!r} not found", file=sys.stderr)
        for c in candidates[:5]:
            print(f"  did you mean: {c}", file=sys.stderr)
        return 1

    start = symbols[args.start]
    result = walk(instrs, symbols, start, ws=ws)

    if args.trace:
        for ins in result.trace:
            print(f"  {ins.raw}")
        print()

    total = args.hpe + result.base_cycles + result.branch_penalty_cycles
    us = total / args.hclk_mhz

    print(f"start symbol            : {args.start} @ 0x{start:08x}")
    print(f"end (first non-sp sw)   : 0x{result.end.addr:08x}  "
          f"{result.end.mnemonic} {result.end.operands}")
    print(f"instructions walked     : {len(result.trace)}  "
          f"({result.taken_transfers} taken transfers)")
    print(f"memory model            : {mem_label}")
    print()
    print(f"HPE (hardware prologue) : {args.hpe} cycles")
    print(f"base instr cycles       : {result.base_cycles} cycles")
    if ws:
        print(f"prefetch refill penalty : {result.branch_penalty_cycles} cycles  "
              f"({result.taken_transfers} × {ws} WS)")
    print(f"─" * 40)
    print(f"PFIC_ENTRY_TICKS        : {total} cycles  ≈ {us:.3f} µs @ {args.hclk_mhz} MHz")
    return 0


if __name__ == "__main__":
    sys.exit(main())

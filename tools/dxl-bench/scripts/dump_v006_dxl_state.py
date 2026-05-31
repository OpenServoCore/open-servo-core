"""Halt a wedged V006 and decode why the DXL bus is stuck.

Symptoms this targets:
  - Bus DATA line stuck at ~30% of Vcc (half-bridge driver fight between
    chip transceiver still asserting and another bus actor pulling).
  - Bus stuck HIGH while pirate timeouts (chip TE/transceiver enabled but
    DMA exhausted, idle bit = HIGH on the wire).
  - Bus stuck LOW (chip DMA underrun or stuck break condition).

What it dumps and decodes (rev_b pin map: PC0=DXL_TX, PC1=DXL_RX, PC2=TX_EN):
  - CPU PC (against the elf if --elf given)
  - USART1 CTLR1/CTLR3/STATR/BRR — TE/RE/UE/HDSEL/DMAT/DMAR/TC/TXE/RXNE/IDLE
  - DMA1 CH4 (USART1_TX) + CH5 (USART1_RX) — EN, CNTR (remaining bytes)
  - GPIOC PC0/PC1/PC2 — MODE/CNF + OUTDR + INDR (so we see who's driving)
  - AFIO PCFR1 USART1 remap field (verifies pins are actually routed)

Default flow: halt → dump → resume. Use --keep-halted for inspection.

Run:  python scripts/dump_v006_dxl_state.py
      python scripts/dump_v006_dxl_state.py --elf <path-to-elf>
      python scripts/dump_v006_dxl_state.py --keep-halted
"""

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys

# Standard CH32V0x peripheral bases.
USART1 = 0x40013800
DMA1 = 0x40020000
GPIOC = 0x40011000
AFIO = 0x40010000

# DMA channel N base = 0x08 + (N-1)*0x14.
DMA1_CH4 = DMA1 + 0x08 + 3 * 0x14   # USART1 TX
DMA1_CH5 = DMA1 + 0x08 + 4 * 0x14   # USART1 RX


def run(cmd: list[str]) -> str:
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        sys.exit(f"`{' '.join(cmd)}` failed: {r.stderr.strip()}")
    return r.stdout


def wlink_halt() -> None:
    run(["wlink", "halt"])


def wlink_resume() -> None:
    run(["wlink", "resume"])


def wlink_regs() -> str:
    return run(["wlink", "regs"])


_ANSI = re.compile(r"\x1b\[[0-9;]*[mGKHF]")


def wlink_dump(addr: int, length: int) -> bytes:
    """Returns `length` bytes starting at `addr`. wlink prints colorized hex
    in form `ADDR:   XX XX XX XX  XX XX ...   <ascii>` per line; we strip
    ANSI and parse the first `length` hex bytes after each address."""
    out = run(["wlink", "dump", f"0x{addr:08x}", str(length)])
    clean = _ANSI.sub("", out)
    bs = bytearray()
    for line in clean.splitlines():
        m = re.match(r"\s*(0x)?[0-9a-fA-F]+\s*:\s+(.*)", line)
        if not m:
            continue
        # Stop at 16 hex tokens per line (ascii column is after that).
        tokens = m.group(2).split()
        n_taken = 0
        for tok in tokens:
            if n_taken >= 16:
                break
            if re.fullmatch(r"[0-9a-fA-F]{2}", tok):
                bs.append(int(tok, 16))
                n_taken += 1
            if len(bs) >= length:
                break
        if len(bs) >= length:
            break
    if len(bs) < length:
        sys.exit(f"dump short: wanted {length}, parsed {len(bs)} from:\n{clean}")
    return bytes(bs[:length])


def u32_at(buf: bytes, offset: int) -> int:
    return int.from_bytes(buf[offset:offset + 4], "little")


def bit(value: int, n: int, name: str) -> str:
    return f"{name}={'1' if value & (1 << n) else '0'}"


def field(value: int, hi: int, lo: int) -> int:
    return (value >> lo) & ((1 << (hi - lo + 1)) - 1)


def decode_usart(buf: bytes) -> None:
    statr = u32_at(buf, 0x00)
    datar = u32_at(buf, 0x04)
    brr = u32_at(buf, 0x08)
    ctlr1 = u32_at(buf, 0x0C)
    ctlr2 = u32_at(buf, 0x10)
    ctlr3 = u32_at(buf, 0x14)
    print(f"\n[USART1] @ 0x{USART1:08x}")
    print(f"  CTLR1 = 0x{ctlr1:08x}  "
          f"{bit(ctlr1, 13, 'UE')} {bit(ctlr1, 3, 'TE')} {bit(ctlr1, 2, 'RE')} "
          f"{bit(ctlr1, 4, 'IDLEIE')} {bit(ctlr1, 5, 'RXNEIE')} "
          f"{bit(ctlr1, 6, 'TCIE')} {bit(ctlr1, 7, 'TXEIE')}")
    print(f"  CTLR2 = 0x{ctlr2:08x}  STOP={field(ctlr2, 13, 12)} "
          f"{bit(ctlr2, 14, 'LINEN')}")
    print(f"  CTLR3 = 0x{ctlr3:08x}  "
          f"{bit(ctlr3, 3, 'HDSEL')} {bit(ctlr3, 6, 'DMAR')} {bit(ctlr3, 7, 'DMAT')} "
          f"{bit(ctlr3, 0, 'EIE')}")
    print(f"  STATR = 0x{statr:08x}  "
          f"{bit(statr, 6, 'TC')} {bit(statr, 7, 'TXE')} {bit(statr, 5, 'RXNE')} "
          f"{bit(statr, 4, 'IDLE')} {bit(statr, 3, 'ORE')} {bit(statr, 2, 'NE')} "
          f"{bit(statr, 1, 'FE')} {bit(statr, 0, 'PE')}")
    print(f"  BRR   = 0x{brr:08x}  (mantissa={field(brr, 15, 4)} frac={field(brr, 3, 0)} "
          f"→ baud ≈ {48_000_000 // (brr if brr else 1)} bps @ 48 MHz PCLK)")
    print(f"  DATAR = 0x{datar:08x}")


def decode_dma_ch(name: str, base: int, buf: bytes) -> None:
    cfgr = u32_at(buf, 0x00)
    cntr = u32_at(buf, 0x04)
    paddr = u32_at(buf, 0x08)
    maddr = u32_at(buf, 0x0C)
    print(f"\n[{name}] @ 0x{base:08x}")
    print(f"  CFGR  = 0x{cfgr:08x}  "
          f"{bit(cfgr, 0, 'EN')} {bit(cfgr, 4, 'DIR')} {bit(cfgr, 5, 'CIRC')} "
          f"{bit(cfgr, 6, 'PINC')} {bit(cfgr, 7, 'MINC')} "
          f"{bit(cfgr, 1, 'TCIE')} {bit(cfgr, 2, 'HTIE')} {bit(cfgr, 3, 'TEIE')}")
    print(f"  CNTR  = 0x{cntr:08x}  ({cntr} bytes remaining)")
    print(f"  PADDR = 0x{paddr:08x}   MADDR = 0x{maddr:08x}")


_GPIO_MODE = {0: "in", 1: "out-10M", 2: "out-2M", 3: "out-50M"}
_GPIO_CNF_IN = {0: "analog", 1: "float", 2: "pupd", 3: "rsvd"}
_GPIO_CNF_OUT = {0: "PP", 1: "OD", 2: "AF_PP", 3: "AF_OD"}


def _decode_pin(cfg_nibble: int) -> str:
    mode = cfg_nibble & 0x3
    cnf = (cfg_nibble >> 2) & 0x3
    if mode == 0:
        return f"{_GPIO_MODE[mode]} {_GPIO_CNF_IN[cnf]}"
    return f"{_GPIO_MODE[mode]} {_GPIO_CNF_OUT[cnf]}"


def decode_gpioc(buf: bytes) -> None:
    cfglr = u32_at(buf, 0x00)
    indr = u32_at(buf, 0x08)
    outdr = u32_at(buf, 0x0C)
    print(f"\n[GPIOC] @ 0x{GPIOC:08x}  (PC0=DXL_TX, PC1=DXL_RX, PC2=TX_EN)")
    print(f"  CFGLR = 0x{cfglr:08x}")
    for pin in (0, 1, 2):
        nibble = (cfglr >> (pin * 4)) & 0xF
        in_lvl = (indr >> pin) & 1
        out_lvl = (outdr >> pin) & 1
        print(f"    PC{pin}: cfg=0x{nibble:x} [{_decode_pin(nibble)}]  "
              f"IN={in_lvl}  OUT={out_lvl}")


def decode_afio(buf: bytes) -> None:
    # PCFR1 sits at AFIO + 0x0C on V006 (memory note ch32v006_afio_pcfr1_offset).
    pcfr1 = u32_at(buf, 0x0C)
    # USART1_RM = bit 2; USART1_RM1 = bit 21; USART1_RM2 = bit 22.
    rm0 = (pcfr1 >> 2) & 1
    rm1 = (pcfr1 >> 21) & 1
    rm2 = (pcfr1 >> 22) & 1
    remap = (rm2 << 2) | (rm1 << 1) | rm0
    print(f"\n[AFIO] @ 0x{AFIO:08x}")
    print(f"  PCFR1 = 0x{pcfr1:08x}  USART1_RM=0b{remap:03b} ({remap}) "
          f"[expect 3 for PC0/PC1 on rev_b]")


def lookup_pc(elf: str, pc: int) -> str | None:
    if not shutil.which("riscv64-unknown-elf-addr2line"):
        return None
    r = subprocess.run(
        ["riscv64-unknown-elf-addr2line", "-fipe", elf, f"0x{pc:08x}"],
        capture_output=True, text=True,
    )
    if r.returncode == 0:
        return r.stdout.strip()
    return None


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--elf", default=None,
                    help="path to chip elf for PC symbol lookup (addr2line)")
    ap.add_argument("--keep-halted", action="store_true",
                    help="don't resume after dump (for follow-up inspection)")
    args = ap.parse_args()

    wlink_halt()
    try:
        print("=" * 70)
        regs_out = wlink_regs()
        print(regs_out.rstrip())
        if args.elf:
            m = re.search(r"\bpc\s*[:=]?\s*(0x[0-9a-fA-F]+)", regs_out)
            if m:
                pc = int(m.group(1), 16)
                sym = lookup_pc(args.elf, pc)
                if sym:
                    print(f"  PC 0x{pc:08x} → {sym}")

        decode_usart(wlink_dump(USART1, 0x20))
        decode_dma_ch("DMA1 CH4 (USART1 TX)", DMA1_CH4, wlink_dump(DMA1_CH4, 0x10))
        decode_dma_ch("DMA1 CH5 (USART1 RX)", DMA1_CH5, wlink_dump(DMA1_CH5, 0x10))
        decode_gpioc(wlink_dump(GPIOC, 0x10))
        decode_afio(wlink_dump(AFIO, 0x10))
        print()
        print("=" * 70)
    finally:
        if not args.keep_halted:
            wlink_resume()


if __name__ == "__main__":
    main()

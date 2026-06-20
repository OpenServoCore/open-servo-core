#!/usr/bin/env python3
"""Manual unicast-vs-broadcast smoke check — isolates the single-target bug.

No reboot, no drift, no trim. Discover the chip via broadcast Ping (sweeping
bauds), then hammer broadcast Ping, unicast Ping, and a unicast Read at the
discovered ID and tally replies. Directly answers "does unicast work now?".
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from dxl_packet import BROADCAST_ID, build_ping, build_read, parse_status
from pirate import Pirate
from pirate_chip_common import BAUD_INDEX, autodetect_pirate


def discover(pirate: Pirate):
    cands = [1_000_000] + [b for b in sorted(BAUD_INDEX, reverse=True) if b != 1_000_000]
    for b in cands:
        pirate.set_baud(b)
        time.sleep(0.05)
        pirate.drain_stamps()
        reply = pirate.xfer(build_ping(BROADCAST_ID), reply_us=200_000)
        if reply:
            return parse_status(reply).id, b, reply
    return None, None, None


def tally(pirate, label, pkt, dxl_id, n=20):
    ok = 0
    first = None
    for i in range(n):
        r = pirate.xfer(pkt, reply_us=200_000)
        if r:
            ok += 1
            if first is None:
                first = r
        time.sleep(0.005)
    print(f"{label:28s} {ok:>2}/{n} replied", flush=True)
    if first is not None:
        try:
            st = parse_status(first)
            print(f"    sample: id={st.id} err=0x{st.error:02X} "
                  f"params={st.params.hex()} raw={first.hex()}")
        except Exception as e:
            print(f"    sample raw={first.hex()} (parse: {e})")
    return ok


def main():
    port = autodetect_pirate()
    pirate = Pirate(port)
    try:
        dxl_id, baud, bcast = discover(pirate)
        if dxl_id is None:
            print("FAIL: no broadcast-Ping reply at any baud")
            return
        print(f"discovered id={dxl_id} at baud={baud}  raw={bcast.hex()}\n")
        # pirate is left at `baud` by discover()
        tally(pirate, "broadcast Ping (0xFE)", build_ping(BROADCAST_ID), dxl_id)
        tally(pirate, f"unicast Ping (id={dxl_id})", build_ping(dxl_id), dxl_id)
        tally(pirate, "unicast Read @0 len1", build_read(dxl_id, 0, 1), dxl_id, n=10)
    finally:
        pirate.close()


if __name__ == "__main__":
    main()

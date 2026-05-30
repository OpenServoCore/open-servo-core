"""Fast Bulk Read coalesce — verifies cal-applied chip fires inside the IDLE-coalesce window.

Pirate emits the predecessor INJ slot (via ARM after master IDLE); chip emits
the DUT slot. With HSI cal applied (clock_trim + clock_fine_trim_us per
docs/dxl-hsi-calibration.md), the bus should never IDLE between the two
slots — the chip's slot start lands within ~10 bit-times of the INJ slot end,
so the IDLE flag never asserts.

Each trial is classified:
  clean       — exactly 1 IDLE stamp at end of burst (coalesced)
  extra_idle  — >1 stamps (chip fired late; bus quieted)
  crc         — INJ tail clipped / CRC fail (chip fired early)
  other       — no frame / DUT slot bad / RX overflow

Pass criteria per (baud, INJ_LEN):
  - zero `other` (no lost frames — that's a real bug)
  - non-clean count ≤ max(2, trials // 10). At our measured ~1% wire wobble
    this gives ample headroom at any --trials; raise --trials to tighten.

Trials per (baud, INJ_LEN) is the `--trials` session option (default 10).
"""

import time

import pytest

from conftest import BAUD_INDEX, ensure_chip_baud
from dxl_link import clear_counters, print_counters, read_counters
from dxl_packet import (
    build_fast_bulk_read,
    build_fast_first_bytes,
    parse_fast_response,
)

INJ_ID = 50
DUT_LEN = 4
INJ_LENS = [4, 32, 64, 96, 128]
TEST_BAUDS = [1_000_000, 3_000_000]


def _run_trial(pirate, dut_id: int, inj_len: int) -> str:
    inj_data = b"\xAA" * inj_len
    packet_length = 1 + (2 + inj_len) + (2 + DUT_LEN) + 2
    inj_bytes = build_fast_first_bytes(
        packet_length=packet_length, err=0, slot_id=INJ_ID, data=inj_data,
    )
    request = build_fast_bulk_read([(INJ_ID, 0, inj_len), (dut_id, 0, DUT_LEN)])

    pirate.drain_stamps()
    b0 = pirate.bytes_count()
    pirate.arm(inj_bytes, after_idle_ticks=250 * 18)
    pirate.master(request)
    time.sleep(0.02)
    b1 = pirate.bytes_count()

    total = b1 - b0
    if total > 256:
        pirate.drain_stamps()
        return "other"
    all_rx = pirate.rx_range(b0, total)
    frame = all_rx[len(request):]
    if len(frame) < 11:
        pirate.drain_stamps()
        return "other"
    try:
        slots = parse_fast_response(frame, slot_lengths=[inj_len, DUT_LEN])
    except ValueError:
        pirate.drain_stamps()
        return "crc"
    if len(slots) != 2 or slots[0].id != INJ_ID or slots[0].data != inj_data:
        pirate.drain_stamps()
        return "crc"
    if slots[1].id != dut_id or slots[1].error != 0 or len(slots[1].data) != DUT_LEN:
        pirate.drain_stamps()
        return "other"
    return "clean" if len(pirate.drain_stamps()) == 1 else "extra_idle"


def _stress(pirate, dut_id: int, inj_len: int, n: int) -> dict:
    tally = {"clean": 0, "extra_idle": 0, "crc": 0, "other": 0}
    for _ in range(n):
        tally[_run_trial(pirate, dut_id, inj_len)] += 1
    return tally


@pytest.fixture
def at_test_baud(pirate, osc_id, baud, request):
    """Switch the chip to `request.param` baud for the test, then back."""
    target = request.param
    if target not in BAUD_INDEX:
        pytest.skip(f"unsupported test baud {target}")
    if baud != target:
        ensure_chip_baud(pirate, osc_id, target)
    yield target
    if baud != target:
        ensure_chip_baud(pirate, osc_id, baud)


@pytest.mark.parametrize(
    "at_test_baud",
    TEST_BAUDS,
    indirect=True,
    ids=[f"{b // 1_000_000}M" for b in TEST_BAUDS],
)
@pytest.mark.parametrize("inj_len", INJ_LENS)
def test_fast_coalesce(pirate, osc_id, at_test_baud, trials, inj_len):
    clear_counters(pirate, osc_id)
    tally = _stress(pirate, osc_id, inj_len, trials)
    link = read_counters(pirate, osc_id)

    total = sum(tally.values())
    clean_frac = tally["clean"] / total
    label = f"baud={at_test_baud} INJ_LEN={inj_len:>3d} n={trials:>3d}"
    print(
        f"\n  {label}  →  clean={tally['clean']:>3d}  extra={tally['extra_idle']:>3d}  "
        f"crc={tally['crc']:>3d}  other={tally['other']:>3d}  ({clean_frac*100:.1f}%)",
        flush=True,
    )
    print_counters(label, link)

    max_non_clean = max(2, trials // 10)
    non_clean = total - tally["clean"]
    assert tally["other"] == 0, f"lost frames at {label}: {tally}"
    assert non_clean <= max_non_clean, (
        f"{non_clean} non-clean trials at {label} (limit {max_non_clean}): {tally}"
    )

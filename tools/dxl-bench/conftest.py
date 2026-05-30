"""dxl-bench session setup.

The bench is pirate-only: one V203 (dxl-pirate firmware) is master, listener,
and IDLE-stamp source on the DXL bus, talking to a V006 DUT. There is no
external USB-UART path — pyserial + DynamixelSDK are gone. Tests drive the
bus through the `pirate` fixture; see `tools/dxl-bench/pirate.py`."""

import time

import pytest
import serial.tools.list_ports

from cal import Calibrator
from dxl_packet import build_ping, build_write, parse_status
from pirate import Pirate

PIRATE_VID = 0xC0DE
PIRATE_PID = 0xCAFE

# Mirror config::addr::comms::BAUD_RATE_IDX and the BaudRate enum.
BAUD_RATE_IDX_ADDR = 13
DEFAULT_BAUD = 1_000_000
BAUD_INDEX = {
    9600: 0,
    57600: 1,
    115200: 2,
    1_000_000: 3,
    2_000_000: 4,
    3_000_000: 5,
}


def pytest_addoption(parser):
    parser.addoption(
        "--pirate-port",
        default=None,
        help="dxl-pirate USB-CDC device path (default: auto-detect by VID/PID)",
    )
    parser.addoption("--baud", default=1_000_000, type=int, help="DXL wire baud")
    parser.addoption(
        "--tinyboot-baud",
        default=3_000_000,
        type=int,
        help="Baud the bootloader runs at; mirrors `boot/src/main.rs` and must "
             "be updated together if that file changes.",
    )
    parser.addoption("--id", default=1, type=int, help="OSC servo DXL ID")
    parser.addoption(
        "--no-dut",
        action="store_true",
        help="Skip V006 ping/retune at session start (for pirate-only smoke "
             "checks where the DUT is unplugged).",
    )
    parser.addoption(
        "--no-cal",
        action="store_true",
        help="Skip the boot-time HSI cal at session start. Default is to "
             "converge clock_trim via master-side CAL and apply the matching "
             "clock_fine_trim_us residual so every test runs against a fully "
             "calibrated chip.",
    )
    parser.addoption(
        "--trials",
        type=int,
        default=10,
        help="Trial multiplier for stress-style timing tests "
             "(e.g. test_timing_fast_coalesce). Default 10 keeps the suite "
             "fast; --trials=500 gives ~2500 trials per (baud, INJ_LEN).",
    )


def _autodetect_pirate() -> str:
    matches = [
        p.device
        for p in serial.tools.list_ports.comports()
        if p.vid == PIRATE_VID and p.pid == PIRATE_PID
    ]
    if len(matches) == 1:
        return matches[0]
    if not matches:
        pytest.fail(
            "no dxl-pirate found (VID 0xC0DE / PID 0xCAFE); "
            "pass --pirate-port=<path>",
            pytrace=False,
        )
    pytest.fail(
        f"multiple dxl-pirates found ({matches}); "
        "disambiguate with --pirate-port=<path>",
        pytrace=False,
    )


def _ping_at(pirate: Pirate, dxl_id: int, bps: int, attempts: int = 3) -> bytes:
    pirate.set_baud(bps)
    time.sleep(0.05)
    for _ in range(attempts):
        pirate.drain_stamps()
        reply = pirate.xfer(build_ping(dxl_id), reply_us=200_000)
        if reply:
            return reply
        time.sleep(0.02)
    return b""


def ensure_chip_baud(pirate: Pirate, dxl_id: int, target_bps: int) -> None:
    if target_bps not in BAUD_INDEX:
        pytest.fail(
            f"unsupported --baud {target_bps}; supported: {sorted(BAUD_INDEX)}",
            pytrace=False,
        )
    if _ping_at(pirate, dxl_id, target_bps):
        print(f"[bench] chip already at {target_bps} baud", flush=True)
        return

    current = next(
        (bps for bps in sorted(BAUD_INDEX, reverse=True)
         if bps != target_bps and _ping_at(pirate, dxl_id, bps)),
        None,
    )
    if current is None:
        pytest.fail(
            f"V006 not found at any known baud (target {target_bps}) — "
            "flash/wiring issue?",
            pytrace=False,
        )

    print(f"[bench] retune chip {current} → {target_bps}", flush=True)
    pirate.set_baud(current)
    time.sleep(0.05)
    reply = pirate.xfer(
        build_write(dxl_id, BAUD_RATE_IDX_ADDR, bytes([BAUD_INDEX[target_bps]])),
        reply_us=200_000,
    )
    if not reply:
        pytest.fail(f"BAUD write got no ACK at {current} baud", pytrace=False)
    st = parse_status(reply)
    if st.error != 0:
        pytest.fail(f"BAUD write rejected: err=0x{st.error:02x}", pytrace=False)
    time.sleep(0.05)  # let TC ISR finish the retune before pirate switches
    pirate.set_baud(target_bps)
    time.sleep(0.05)


def _cal_hsi(pirate: Pirate, dxl_id: int, baud: int, max_cycles: int = 6) -> tuple[int, int]:
    """Converge `clock_trim` via master-side CAL, then apply the
    `clock_fine_trim_us` residual measured AT the converged trim (post-trim
    residual is what the slot dispatcher needs — pre-trim residual is stale
    once trim shifts HSI). Returns (final_trim, final_q88).

    Per docs/dxl-hsi-calibration.md: chip's structural latency is carried
    by the compile-time `TX_LATENCY_TICKS` (=104); the runtime
    `clock_fine_trim_us` covers only the per-chip drift residual, which
    lands inside the 3 Mbaud coalesce window with no per-chip sweep."""
    c = Calibrator(pirate, dxl_id=dxl_id, baud=baud)
    c.write_clock_fine_trim_us(0)
    time.sleep(0.05)
    for _ in range(max_cycles):
        trim_before = c.read_clock_trim()
        m = c.measure(count=128)
        d = c.derive(m, current_trim=trim_before)
        if d.step == 0:
            c.write_clock_fine_trim_us(d.residual_q88)
            time.sleep(0.05)
            return trim_before, d.residual_q88
        c.write_clock_trim(d.new_trim)
        time.sleep(0.05)
    pytest.fail(
        f"HSI cal did not converge in {max_cycles} cycles",
        pytrace=False,
    )


def pytest_collection_modifyitems(config, items):
    """Run `test_system_*` last. System tests reboot the chip (e.g.
    tinyboot round-trip), which wipes RAM-backed control-table state
    including `clock_trim` and `clock_fine_trim_us` — every timing test
    that follows would then see an un-cal'd chip and fail."""
    system, other = [], []
    for item in items:
        (system if item.fspath.basename.startswith("test_system_") else other).append(item)
    items[:] = other + system


def pytest_sessionstart(session):
    config = session.config

    print("\n--- bench setup ---", flush=True)
    pirate_path = config.getoption("--pirate-port") or _autodetect_pirate()
    print(f"[bench] pirate: {pirate_path}", flush=True)

    pirate = Pirate(pirate_path)
    try:
        target_baud = config.getoption("--baud")
        dxl_id = config.getoption("--id")
        if config.getoption("--no-dut"):
            print("[bench] --no-dut: skipping V006 ping/retune + cal", flush=True)
            pirate.set_baud(target_baud)
        else:
            ensure_chip_baud(pirate, dxl_id, target_baud)
            if config.getoption("--no-cal"):
                print("[bench] --no-cal: skipping HSI cal", flush=True)
            else:
                final_trim, final_q88 = _cal_hsi(pirate, dxl_id, target_baud)
                print(f"[bench] HSI cal: clock_trim={final_trim:+d}, "
                      f"clock_fine_trim_us={final_q88:+d} "
                      f"(={final_q88/256:+.3f} µs)", flush=True)
    finally:
        pirate.close()

    config._bench_pirate_path = pirate_path
    print("--- bench setup complete ---\n", flush=True)


@pytest.fixture(scope="session")
def pirate(request):
    path = getattr(request.config, "_bench_pirate_path", None) or _autodetect_pirate()
    p = Pirate(path)
    yield p
    p.close()


@pytest.fixture(autouse=True)
def _drain_pirate_between_tests(pirate):
    # Drain stale stamps from the previous test so DRAIN-watching tests see
    # only their own IDLEs. The bytes counter is monotonic; tests snapshot it
    # at trip start, so no reset needed.
    pirate.drain_stamps()
    yield
    pirate.drain_stamps()


@pytest.fixture(scope="session")
def osc_id(request):
    return request.config.getoption("--id")


@pytest.fixture(scope="session")
def baud(request):
    return request.config.getoption("--baud")


@pytest.fixture(scope="session")
def tinyboot_baud(request):
    return request.config.getoption("--tinyboot-baud")


@pytest.fixture(scope="session")
def trials(request):
    return request.config.getoption("--trials")

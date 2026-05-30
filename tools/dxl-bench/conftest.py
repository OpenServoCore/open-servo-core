"""dxl-bench session setup.

The bench is pirate-only: one V203 (dxl-pirate firmware) is master, listener,
and IDLE-stamp source on the DXL bus, talking to a V006 DUT. There is no
external USB-UART path — pyserial + DynamixelSDK are gone. Tests drive the
bus through the `pirate` fixture; see `tools/dxl-bench/pirate.py`."""

import time

import pytest
import serial.tools.list_ports

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


def _ensure_chip_baud(pirate: Pirate, dxl_id: int, target_bps: int) -> None:
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


def pytest_sessionstart(session):
    config = session.config

    print("\n--- bench setup ---", flush=True)
    pirate_path = config.getoption("--pirate-port") or _autodetect_pirate()
    print(f"[bench] pirate: {pirate_path}", flush=True)

    pirate = Pirate(pirate_path)
    try:
        target_baud = config.getoption("--baud")
        if config.getoption("--no-dut"):
            print("[bench] --no-dut: skipping V006 ping/retune", flush=True)
            pirate.set_baud(target_baud)
        else:
            _ensure_chip_baud(pirate, config.getoption("--id"), target_baud)
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

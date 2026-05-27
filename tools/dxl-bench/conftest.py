import time

import pytest
import serial
import serial.tools.list_ports
from dynamixel_sdk import PacketHandler

from dxl_packet import build_ping, build_write, parse_status, read_status_frame
from echo_port_handler import EchoDrainingPortHandler

# WCH-LinkE USB IDs — used only to warn when its CDC serial bridge is the
# auto-selected DXL bus path. Its USART firmware drops/delays bytes and
# causes flaky tests; FT232H or similar dedicated USB-UART is preferred.
WCH_LINKE_VID = 0x1A86
WCH_LINKE_PID = 0x8010

# dxl-bus-injector USB CDC — host-commanded DXL bus injector + listener that
# shares the bus with the V006 DUT. See tools/dxl-bus-injector.
INJECTOR_VID = 0xC0DE
INJECTOR_PID = 0xCAFE

# Allowlist of USB vendor IDs for known USB-UART bridge chips. Filtering by
# vendor avoids picking up unrelated CDC-class devices (e.g. monitor "USB
# Controls", touchscreen controllers) that enumerate via `comports()` but
# aren't actual UART bridges. Pass `--port` for vendors not in this list.
USB_UART_VIDS = frozenset([
    0x0403,  # FTDI (FT232R, FT2232, FT4232, FT232H, FT231X, ...)
    0x067B,  # Prolific (PL2303 family)
    0x10C4,  # Silicon Labs (CP2102, CP2104, CP2105, CP2108, CP21xx)
    0x1A86,  # WCH (CH340/CH341/CH343 — and the WCH-LinkE serial bridge)
])

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
        "--port",
        default=None,
        help="Serial device path (default: auto-detect any USB-UART)",
    )
    parser.addoption("--baud", default=1000000, type=int, help="UART baud rate")
    parser.addoption(
        "--tinyboot-baud",
        default=3000000,
        type=int,
        help="Baud the bootloader runs at; mirrors `boot/src/main.rs` and must "
             "be updated together if that file changes.",
    )
    parser.addoption("--id", default=1, type=int, help="OSC servo DXL ID")
    parser.addoption(
        "--injector-port",
        default=None,
        help="V203 injector USB-CDC path (default: auto-detect by VID/PID; "
             "absent injector simply skips injector-dependent tests)",
    )


# `serial.tools.list_ports.comports()` is cross-platform: IOKit on macOS,
# sysfs on Linux, SetupDi on Windows. `p.device` returns the native path
# (`/dev/cu.*`, `/dev/ttyUSB*` / `/dev/ttyACM*`, `COMn`) which `serial.Serial`
# accepts unchanged. `p.vid` is None for non-USB or legacy ports on every
# platform, so the same filter works everywhere.
def _autodetect_port() -> str:
    """Pick the USB-UART device for the DXL bus. Filters to known UART chip
    VIDs (see `USB_UART_VIDS`), then de-prefers the WCH-LinkE bridge — when
    an FT232H/CP21xx/CH340 is also present, the dedicated bridge wins
    silently; the WCH-LinkE only auto-picks (with a warning) when it's the
    sole UART on the bus."""
    candidates = [
        p
        for p in serial.tools.list_ports.comports()
        if p.vid in USB_UART_VIDS
    ]
    preferred = [
        p for p in candidates
        if not (p.vid == WCH_LINKE_VID and p.pid == WCH_LINKE_PID)
    ] or candidates
    if not preferred:
        pytest.fail(
            "no USB-UART device found; pass --port=<path>",
            pytrace=False,
        )
    if len(preferred) > 1:
        listing = "\n".join(
            f"  {p.device}  vid={p.vid:04x} pid={(p.pid or 0):04x}  {p.description or ''}"
            for p in preferred
        )
        pytest.fail(
            f"multiple USB-UART devices found; disambiguate with --port=<path>:\n{listing}",
            pytrace=False,
        )
    return preferred[0].device


def _warn_if_wch_linke(device_path: str) -> None:
    """Print a flake warning if `device_path` is a WCH-LinkE CDC bridge. See
    `reference_wch_linke_dxl_bench_unreliable` — its USART firmware drops and
    delays bytes under sustained DXL traffic."""
    for p in serial.tools.list_ports.comports():
        if (
            p.device == device_path
            and p.vid == WCH_LINKE_VID
            and p.pid == WCH_LINKE_PID
        ):
            print(
                "[bench] WARNING: bus is on a WCH-LinkE CDC serial bridge. "
                "Its USART firmware drops/delays bytes and produces flaky "
                "DXL test results — prefer an FT232H or other dedicated "
                "USB-UART.",
                flush=True,
            )
            return


def _resolve_port(config) -> str:
    chosen = config.getoption("--port") or _autodetect_port()
    _warn_if_wch_linke(chosen)
    config._bench_port_path = chosen
    return chosen


def _autodetect_injector_port() -> str | None:
    """Find the V203 injector CDC device, or None if absent. The injector is
    optional — missing one degrades to skip-marking the tests that need it
    rather than failing the whole session. Multiple matches still fail so we
    don't silently pick the wrong device."""
    matches = [
        p.device
        for p in serial.tools.list_ports.comports()
        if p.vid == INJECTOR_VID and p.pid == INJECTOR_PID
    ]
    if len(matches) == 1:
        return matches[0]
    if not matches:
        return None
    pytest.fail(
        f"multiple V203 injectors found ({matches}); "
        "disambiguate with --injector-port <path>",
        pytrace=False,
    )


class Injector:
    """Thin USB-CDC client for the dxl-bus-injector's ASCII line protocol. See
    `tools/dxl-bus-injector/src/proto.rs` for the command grammar."""

    def __init__(self, port_path: str):
        # 115200 is nominal-only for CDC ACM; the wire side runs whatever
        # `BAUD <bps>` was last set to.
        self.ser = serial.Serial(port_path, 115200, timeout=0.2)
        self.port_path = port_path

    def close(self) -> None:
        self.ser.close()

    def command(self, line: str) -> str:
        """Send one line, return one stripped reply. Raises on timeout."""
        self.ser.reset_input_buffer()
        self.ser.write(line.encode() + b"\n")
        self.ser.flush()
        reply = self.ser.readline()
        if not reply:
            raise TimeoutError(f"injector did not reply to {line!r}")
        return reply.decode(errors="replace").rstrip()

    def set_baud(self, bps: int) -> None:
        reply = self.command(f"BAUD {bps}")
        if reply != "OK":
            pytest.fail(f"injector BAUD {bps} → {reply!r}", pytrace=False)

    def fire(self, *, dxl_id: int, err: int, packet_length: int,
             data: bytes, at_tick: int) -> None:
        reply = self.command(
            f"FIRE id={dxl_id} err={err} pl={packet_length} "
            f"data={data.hex()} at={at_tick}"
        )
        if reply != "OK":
            pytest.fail(f"injector FIRE → {reply!r}", pytrace=False)


def _ensure_injector_baud(port_path: str, target_bps: int) -> None:
    """Open the injector CDC, ping it, and retune both USART2 TX + USART3 RX
    to `target_bps`. Bus must be quiet — caller's responsibility."""
    inj = Injector(port_path)
    try:
        # TICK? is a cheap liveness probe that also gives the CDC stack a
        # round-trip before we start spamming BAUD.
        tick = inj.command("TICK?")
        if not tick.startswith("TICK "):
            pytest.fail(f"injector TICK? → {tick!r}", pytrace=False)
        inj.set_baud(target_bps)
    finally:
        inj.close()


def _drain_quiet(ser, quiet_ms: int = 25, timeout_ms: int = 300) -> None:
    """Consume bytes until the bus has been silent for `quiet_ms`. Cheap to call
    between tests; protects against stragglers from a previous reply landing in
    the next test's read window."""
    deadline = time.monotonic() + timeout_ms / 1000
    silent_since = None
    while time.monotonic() < deadline:
        n = ser.in_waiting
        if n:
            ser.read(n)
            silent_since = None
        elif silent_since is None:
            silent_since = time.monotonic()
        elif (time.monotonic() - silent_since) * 1000 >= quiet_ms:
            return
        time.sleep(0.001)


def _ping_at(port_path: str, dxl_id: int, bps: int, attempts: int = 3) -> bytes:
    """Ping `dxl_id` at `bps`; returns the raw Status frame or b'' if no reply
    after `attempts` tries. CDC/USB occasionally drops the first probe after a
    rate change, so retry a few times before giving up."""
    s = serial.Serial(port_path, bps, timeout=0.2)
    try:
        time.sleep(0.15)  # let CDC + WCH-LinkE rate switch settle
        for _ in range(attempts):
            s.reset_input_buffer()
            req = build_ping(dxl_id)
            s.write(req)
            s.flush()
            s.read(len(req))  # half-duplex echo
            frame = read_status_frame(s)
            if frame:
                return frame
            time.sleep(0.05)
        return b""
    finally:
        s.close()


def _ensure_chip_baud(port_path: str, dxl_id: int, target_bps: int) -> None:
    """Bring the V006 to `target_bps`. Already-there → no-op; otherwise sweep
    known rates to find the chip, then DXL Write BAUD_RATE_IDX from there.
    Host stays at the current rate to consume the ACK before the TC ISR retunes."""
    if target_bps not in BAUD_INDEX:
        pytest.fail(
            f"unsupported --baud {target_bps}; supported: {sorted(BAUD_INDEX)}",
            pytrace=False,
        )
    if _ping_at(port_path, dxl_id, target_bps):
        print(f"[bench] chip already at {target_bps} baud", flush=True)
        return

    # Sweep highest→lowest so post-test cleanups land first (most likely state).
    current = next(
        (bps for bps in sorted(BAUD_INDEX, reverse=True)
         if bps != target_bps and _ping_at(port_path, dxl_id, bps)),
        None,
    )
    if current is None:
        pytest.fail(
            f"V006 not found at any known baud (target {target_bps}) — "
            "flash/wiring issue?",
            pytrace=False,
        )

    print(f"[bench] retune chip {current} → {target_bps}", flush=True)
    s = serial.Serial(port_path, current, timeout=0.2)
    try:
        time.sleep(0.15)
        s.reset_input_buffer()
        req = build_write(dxl_id, BAUD_RATE_IDX_ADDR, bytes([BAUD_INDEX[target_bps]]))
        s.write(req)
        s.flush()
        s.read(len(req))
        frame = read_status_frame(s) or b""
    finally:
        s.close()
    if not frame:
        pytest.fail(f"BAUD write got no ACK at {current} baud", pytrace=False)
    st = parse_status(frame)
    if st.error != 0:
        pytest.fail(f"BAUD write rejected: err=0x{st.error:02x}", pytrace=False)
    time.sleep(0.05)  # let TC ISR finish the retune before the port fixture opens


def pytest_sessionstart(session):
    # Session hook (not a fixture) so output lands before pytest's test-name lines.
    config = session.config

    print("\n--- bench setup ---", flush=True)

    port_path = _resolve_port(config)
    print(f"[bench] port: {port_path}", flush=True)
    target_baud = config.getoption("--baud")
    _ensure_chip_baud(port_path, config.getoption("--id"), target_baud)

    inj_path = config.getoption("--injector-port") or _autodetect_injector_port()
    config._bench_injector_path = inj_path
    if inj_path is None:
        print("[bench] injector: not detected — injector tests will skip", flush=True)
    else:
        print(f"[bench] injector: {inj_path} (tuning to {target_baud} baud)", flush=True)
        _ensure_injector_baud(inj_path, target_baud)

    print("--- bench setup complete ---\n", flush=True)


@pytest.fixture(scope="session")
def port(request):
    path = getattr(request.config, "_bench_port_path", None) or _resolve_port(request.config)
    ph = EchoDrainingPortHandler(path)
    assert ph.openPort(), "openPort failed"
    assert ph.setBaudRate(request.config.getoption("--baud")), "setBaudRate failed"
    yield ph
    ph.closePort()


@pytest.fixture(autouse=True)
def _drain_serial_between_tests(port):
    # Drain both around each test: pre-test consumes any straggler from teardown
    # of the previous test or a slow ACK; post-test catches replies that arrived
    # after the test's read window closed.
    _drain_quiet(port.ser)
    yield
    _drain_quiet(port.ser)


@pytest.fixture(scope="session")
def injector(request):
    path = getattr(request.config, "_bench_injector_path", None)
    if path is None:
        pytest.skip("V203 injector not available")
    inj = Injector(path)
    yield inj
    inj.close()


@pytest.fixture(scope="session")
def packet_handler():
    return PacketHandler(2.0)


@pytest.fixture(scope="session")
def osc_id(request):
    return request.config.getoption("--id")


@pytest.fixture(scope="session")
def baud(request):
    return request.config.getoption("--baud")


@pytest.fixture(scope="session")
def tinyboot_baud(request):
    return request.config.getoption("--tinyboot-baud")

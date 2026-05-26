import subprocess
import tempfile
import time
from pathlib import Path

import pytest
import serial
import serial.tools.list_ports
from dynamixel_sdk import PacketHandler

from dxl_packet import build_ping, build_write, parse_status, read_status_frame
from echo_port_handler import EchoDrainingPortHandler

# WCH-LinkE USB IDs — also matches the WCH-Link debug + serial bridge that
# carries DXL bytes to/from the V006 in our bench rig.
WCH_LINKE_VID = 0x1A86
WCH_LINKE_PID = 0x8010

# nano-v203-injector USB CDC — host-commanded DXL Fast slot injector that
# shares the bus with the V006 DUT. See firmware/boards/nano-v203-injector.
INJECTOR_VID = 0xC0DE
INJECTOR_PID = 0xCAFE

REPO_ROOT = Path(__file__).resolve().parents[2]
BOARD_WS = REPO_ROOT / "firmware" / "boards" / "osc-dev-v006"
ELF_DIR = BOARD_WS / "target" / "riscv32ec_zmmul-unknown-none-elf" / "release"

# Mirror app/memory.x CALIB region.
CALIB_ADDR = 0x0800_0000 + 62 * 1024 - 4 * 1024 - 256
CALIB_LEN = 4 * 1024

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
        help="Serial device path (default: auto-detect WCH-LinkE)",
    )
    parser.addoption("--baud", default=1000000, type=int, help="UART baud rate")
    parser.addoption("--id", default=1, type=int, help="OSC servo DXL ID")
    parser.addoption(
        "--bin",
        default="osc-dev-v006-app-rev-a",
        help="Cargo bin target name to flash",
    )
    parser.addoption(
        "--boot-bin",
        default="osc-dev-v006-boot",
        help="Bootloader bin target (flashed to system flash @ 0x1FFF0000)",
    )
    parser.addoption(
        "--no-flash",
        action="store_true",
        help="Skip cargo build + probe-rs flash (use the firmware already on the chip)",
    )
    parser.addoption(
        "--no-calib",
        action="store_true",
        help="Skip CALIB save/wipe/restore (faster iteration; chip state preserved as-is)",
    )
    parser.addoption(
        "--injector-port",
        default=None,
        help="V203 injector USB-CDC path (default: auto-detect by VID/PID)",
    )
    parser.addoption(
        "--no-injector",
        action="store_true",
        help="Skip injector setup; tests that require it will be skipped",
    )


def _run(cmd, cwd=None):
    print(f"[bench] $ {' '.join(str(c) for c in cmd)}", flush=True)
    result = subprocess.run(cmd, cwd=cwd)
    if result.returncode != 0:
        pytest.fail(
            f"command failed (exit {result.returncode}): {' '.join(str(c) for c in cmd)}",
            pytrace=False,
        )
    return result


def _autodetect_port() -> str:
    """Find the unique WCH-LinkE serial device. Fails with a clear message if
    zero or multiple match — both ambiguous, both should require `--port`."""
    matches = [
        p.device
        for p in serial.tools.list_ports.comports()
        if p.vid == WCH_LINKE_VID and p.pid == WCH_LINKE_PID
    ]
    if len(matches) == 1:
        return matches[0]
    if not matches:
        pytest.fail(
            "no WCH-LinkE serial device found; pass --port <path> "
            "(check `ls /dev/cu.usbmodem*` or plug the probe in)",
            pytrace=False,
        )
    pytest.fail(
        f"multiple WCH-LinkE devices found ({matches}); "
        "disambiguate with --port <path>",
        pytrace=False,
    )


def _resolve_port(config) -> str:
    chosen = config.getoption("--port") or _autodetect_port()
    config._bench_port_path = chosen
    return chosen


def _autodetect_injector_port() -> str | None:
    """Find the unique V203 injector CDC device, or None if absent. Unlike the
    WCH-LinkE probe, the injector is optional — a missing one degrades to
    skip-marking the tests that need it instead of failing the whole session."""
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
    """Thin USB-CDC client for the V203 injector's ASCII line protocol. See
    `firmware/boards/nano-v203-injector/src/proto.rs` for the command grammar."""

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
    do_flash = not config.getoption("--no-flash")
    do_calib = not config.getoption("--no-calib")

    print("\n--- bench setup ---", flush=True)

    if do_flash:
        boot_bin = config.getoption("--boot-bin")
        boot_elf = ELF_DIR / boot_bin
        _run(["cargo", "build", "--release", "--bin", boot_bin], cwd=BOARD_WS)
        assert boot_elf.is_file(), f"boot ELF not found at {boot_elf}"
        _run(["probe-rs", "download", str(boot_elf)])

        bin_name = config.getoption("--bin")
        elf = ELF_DIR / bin_name
        _run(["cargo", "build", "--release", "--bin", bin_name], cwd=BOARD_WS)
        assert elf.is_file(), f"ELF not found at {elf}"
        _run(["probe-rs", "download", str(elf)])
        _run(["probe-rs", "reset"])
        time.sleep(0.3)

    config._bench_calib_backup = None
    if do_calib:
        tmp_root = Path(tempfile.mkdtemp(prefix="dxl-bench-calib-"))
        calib_backup = tmp_root / "calib_backup.bin"
        _run([
            "probe-rs", "read", "b8", hex(CALIB_ADDR), str(CALIB_LEN),
            "--format", "binary",
            "--output", str(calib_backup),
        ])
        _run(["probe-rs", "reset"])
        time.sleep(0.3)
        config._bench_calib_backup = calib_backup

    port_path = _resolve_port(config)
    print(f"[bench] port: {port_path}", flush=True)
    target_baud = config.getoption("--baud")
    _ensure_chip_baud(port_path, config.getoption("--id"), target_baud)

    config._bench_injector_path = None
    if not config.getoption("--no-injector"):
        inj_path = config.getoption("--injector-port") or _autodetect_injector_port()
        if inj_path is None:
            print(
                "[bench] injector: not detected — tests requiring it will skip "
                "(pass --no-injector to silence this message)",
                flush=True,
            )
        else:
            print(f"[bench] injector: {inj_path}", flush=True)
            _ensure_injector_baud(inj_path, target_baud)
            config._bench_injector_path = inj_path

    print("--- bench setup complete ---\n", flush=True)


def pytest_sessionfinish(session, exitstatus):
    calib_backup = getattr(session.config, "_bench_calib_backup", None)
    if calib_backup is None or not calib_backup.exists():
        return
    print("\n--- bench teardown ---", flush=True)
    # Don't switch to `wlink flash --erase` — that's a chip-wide mass erase.
    _run([
        "probe-rs", "download",
        "--binary-format", "bin",
        "--base-address", hex(CALIB_ADDR),
        str(calib_backup),
    ])
    _run(["probe-rs", "reset"])


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

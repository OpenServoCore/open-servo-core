import subprocess
import tempfile
import time
from pathlib import Path

import pytest
from dynamixel_sdk import PacketHandler

from echo_port_handler import EchoDrainingPortHandler

REPO_ROOT = Path(__file__).resolve().parents[2]
BOARD_WS = REPO_ROOT / "firmware" / "boards" / "osc-dev-v006"
ELF_DIR = BOARD_WS / "target" / "riscv32ec_zmmul-unknown-none-elf" / "release"

# Mirror app/memory.x CALIB region.
CALIB_ADDR = 0x0800_0000 + 62 * 1024 - 4 * 1024 - 256
CALIB_LEN = 4 * 1024


def pytest_addoption(parser):
    parser.addoption("--port", default="/dev/ttyACM0", help="Serial device path")
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


def _run(cmd, cwd=None):
    print(f"[bench] $ {' '.join(str(c) for c in cmd)}", flush=True)
    result = subprocess.run(cmd, cwd=cwd)
    if result.returncode != 0:
        pytest.fail(
            f"command failed (exit {result.returncode}): {' '.join(str(c) for c in cmd)}",
            pytrace=False,
        )
    return result


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
    ph = EchoDrainingPortHandler(request.config.getoption("--port"))
    assert ph.openPort(), "openPort failed"
    assert ph.setBaudRate(request.config.getoption("--baud")), "setBaudRate failed"
    yield ph
    ph.closePort()


@pytest.fixture(autouse=True)
def _drain_serial_between_tests(port):
    port.ser.reset_input_buffer()
    yield


@pytest.fixture(scope="session")
def packet_handler():
    return PacketHandler(2.0)


@pytest.fixture(scope="session")
def osc_id(request):
    return request.config.getoption("--id")


@pytest.fixture(scope="session")
def baud(request):
    return request.config.getoption("--baud")

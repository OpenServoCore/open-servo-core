# osc-adapter-wchlinke

The osc bus adapter runs on a stock WCH-LinkE (CH32V305FBP6). One physical
dongle plays two roles, and you flip between them entirely over USB - no
second probe, no button, no soldering:

- **Stock WCH-LinkE probe** (`1a86:8010` RV mode): flashes and debugs targets
  with `wlink`, provides SDI print, powers the DUT rails.
- **osc adapter** (`1209:0001`): the osc-host instrument this crate builds -
  bus scheduling, rail control, bench tooling via `tools/osc`.

## How the flip works

The CH32V305's factory ISP is unreachable on this package, but WCH's LinkE
firmware ships as an 8 KB IAP loader (flash `0x0000`, USB `4348:55e0`) plus
an APP image at `0x2000`. Both the stock firmware and this adapter are APP
images under that same preserved loader:

- The loader jumps to the APP only when the APP-validity words sit at image
  offsets `0x50`/`0x54` (`0x12345678` / `0x39373533`) AND its arm flag
  (byte 5 of the `@WCH` signature at `0x08037F08`) reads `0x5A`. This crate
  plants the validity words through vector-table entries 20/21 (`src/main.rs`;
  CI gates the placement via `.github/check-iap-header.sh`).
- Flashing through the loader re-arms the flag automatically, so a freshly
  flashed APP boots on the next reset without any manual step.

## Tooling (one-time setup)

- **wlink-iap** - cjacker's IAP flasher: <https://github.com/cjacker/wlink-iap>
  (`make && sudo make install`, or keep the binary on PATH).
- **Stock firmware image** - `FIRMWARE_CH32V305.bin`, extracted from WCH's
  [WCH-LinkUtility](https://www.wch.cn/downloads/WCH-LinkUtility_ZIP.html)
  install directory (the utility bundles per-probe firmware; v2.22 known
  good). Community mirrors exist but treat wch.cn as authoritative.
- **Adapter image** - build from this crate:

  ```sh
  cd firmware/boards/osc-adapter-wchlinke
  cargo build --release
  llvm-objcopy -O binary \
      target/riscv32imc-unknown-none-elf/release/osc-adapter-wchlinke \
      osc-adapter.bin
  ```

  (`llvm-objcopy` ships with the Rust `llvm-tools` component; the image is
  linked at `0x2000` under the loader - see `memory.x`.)

## Adapter -> stock LinkE

```sh
osc bootloader                        # adapter disarms the loader flag + resets
                                      # device re-enumerates as 4348:55e0
wlink-iap -f FIRMWARE_CH32V305.bin    # flash stock APP; loader re-arms itself
```

The dongle boots straight into the stock probe (`1a86:8010`). Verify with
`wlink list`. Two gotchas after a flip:

- Stock firmware boots with the **5V output OFF** (3V3 is on for SWD). If the
  DUT's motor rail rides the LinkE 5V: `wlink set-power enable5v`.
- No replug is needed; if the host USB stack sulks, replug is harmless.

## Stock LinkE -> adapter

```sh
wlink-iap -i                          # stock probe -> IAP mode (4348:55e0)
wlink-iap -f osc-adapter.bin          # flash the adapter APP
```

`wlink-iap -i` speaks the stock link-mode protocol to enter IAP; from the
adapter side that door is `osc bootloader` (the adapter does not implement
the stock protocol). Verify with `osc info` (enumerates as `1209:0001`).

## Recovery

The loader is never erased by either direction, so a broken or interrupted
APP flash leaves the dongle in (or bootable into) IAP mode - reflash with
`wlink-iap -f`. The loader cold-boots into IAP whenever the arm flag is
disarmed or the APP validity words are missing. Only a full-flash corruption
(e.g. SWD writes below `0x2000`) needs a second probe clipped to the SWD
header to restore a full image.

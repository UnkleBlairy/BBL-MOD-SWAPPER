# BBL Mod Swapper source

The Teensy 4.1 side of the BBL Mod Swapper, the ports it runs, and the FM-1 terminal firmware.

The Teensy runs FM-1 firmwares natively, compiled from their C source. The FM-1 runs a small terminal firmware and is the Teensy's screen, keys, knobs, LEDs and speaker over USB (`docs/LINK_PROTOCOL.md`).

## Layout

| Path | What |
|---|---|
| `src/` | The runtime (Rust): USB host link to the FM-1, the firmware menu, the SD card, the ROM library (`roms.rs`), save states, the Sample Lab, PSRAM |
| `build.rs` | The memory layout, and one `Fw` entry per port: how its C source is compiled for the Cortex-M7 |
| `build.sh` | Builds `teensy-fm1.hex` (Felucca + the menu) and every port for every flash slot: `roms/<STEM>.R1 .R2 .R3` |
| `ports/<name>/` | Each firmware's source. The Teensy changes are in `web/emu/felucca_teensy.c`, every one marked `teensy-fm1:`. `SOURCE_COMMIT.txt` names the upstream commit |
| `fm1-terminal/` | The FM-1 side: `terminal.c` and felucca-bsp. Builds `fm1-teensy-terminal.fwsc`, the package the installer puts on the FM-1 |
| `docs/LINK_PROTOCOL.md` | The Teensy ↔ FM-1 USB protocol |

## Build (Linux or WSL)

Needs Rust (with the `thumbv7em-none-eabihf` target), `arm-none-eabi-gcc` and Python 3.

```sh
rustup target add thumbv7em-none-eabihf
./build.sh
```

Out come `teensy-fm1.hex` (flash it with the installer or Teensy Loader) and `roms/`: copy a port's three files to the SD card's `ROMS` folder. Set `TFM1_COPY_TO=/some/folder` to have the results copied there too.

The FM-1 terminal (`fm1-terminal/build.sh`) needs the JieLi pi32v2 toolchain, JieLi's AC79 SDK and a checkout of [hugelton/Felucca](https://github.com/hugelton/Felucca): see the top of that script.

## Add a port

The short version is in `ports/PORTING_PROMPT.md` one folder up, written for an AI helper. `ports/x0x` (a firmware with its own browser build) and `ports/sloop` (a Felucca fork without one) are the best models.

## Licences and credits

- Runtime, build scripts, `teensy-fm1:` changes: GPL-3.0-only (`LICENSE`), by Blair (blairysbeatlab).
- Each port keeps its own licence and credits in its folder (`LICENSE`, `LICENSES/`, `NOTICE.md`, `CREDITS.md`):
  - Felucca by Leo Kuroshita (@kurogedelic), Hügelton Instruments: GPL-3.0
  - BBL Jam Box by Blair, a modified Felucca: GPL-3.0
  - JIANT FM by juanjiant-bit: GPL-3.0
  - SLOOP by isod89: GPL-3.0
  - X0X by Charles Vestal: GPL-3.0
- `fm1-terminal/`: MPL-2.0, derived from felucca-bsp's `examples/hwtest` by Leo Kuroshita. felucca-bsp's own licences are in `fm1-terminal/bsp/`.
- The original idea of running FM-1 firmwares off the FM-1 came from studying [fm1-emu](https://github.com/joakimeriksson/fm1-emu) by Joakim Eriksson.

Cleaned for publishing: other authors' `.fwsc` binaries are left out (link to their releases), and so are about 1,600 stray files in upstream JIANT's top folder (names like `00 17` that end in spaces, which Windows can't check out).

Not made or supported by M-VAVE or PJRC. No warranty.

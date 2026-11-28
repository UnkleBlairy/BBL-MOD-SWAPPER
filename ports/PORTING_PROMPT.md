# AI porting prompt

Copy everything inside the box into your AI coding helper (Claude Code, Codex, Cursor or similar). Change the first line to the firmware you want.
It works best with a helper that can run commands on your computer. On Windows, use WSL (Ubuntu).

```text
I want to port this FM-1 firmware to the BBL Mod Swapper: <FIRMWARE NAME AND ITS GITHUB LINK>

What the BBL Mod Swapper is:
- A Teensy 4.1 (NXP i.MX RT1062, Cortex-M7, 600 MHz, 1 MB RAM, 8 MB flash, optional PSRAM) runs FM-1 firmwares
  natively, compiled from their C source. The M-VAVE FM-1 runs a small terminal firmware and is the Teensy's screen,
  keys, knobs, LEDs and speaker, over a USB cable.
- Repo: https://github.com/UnkleBlairy/BBL-MOD-SWAPPER (branch gh-pages). The Teensy source is in source/.
  Read source/README.md, source/build.rs, source/build.sh, source/src/roms.rs and source/docs/LINK_PROTOCOL.md first.
- Finished ports to copy from are in source/ports/: x0x (a firmware with its own browser build) and sloop (a Felucca
  fork with no browser build) are the best models. Each port's changes are in web/emu/felucca_teensy.c and every change
  is marked "teensy-fm1:".

How a port works:
- Most FM-1 Hub firmwares are built on Felucca's HAL (hal/fm1_*.h) and many have a browser build (web/emu/*_web.c)
  that runs the firmware with the hardware faked. The Teensy port starts from that browser build: copy it to
  ports/<name>/web/emu/felucca_teensy.c and change only what the Teensy needs:
  - one core: the firmware's second-core jobs run inline
  - the screen: track changed rows as spans for the FM-1 link (copy x0x's or sloop's code)
  - the SPI NOR (saves): real flash in the ROM's own slot, below its tag (build.rs `nor`), or RAM if it has none
  - flash objects read in place (samples, presets) come from the image in program flash
  - no console, no getenv, no threads, no clock_gettime (stub them as the other ports do)
- MIDI (every port should have it): the Teensy's USB is a MIDI port and the FM-1's TRS jack (MIDI IN) is passed on.
  The port's felucca_teensy.c exports three functions (read source/docs/MIDI.md, copy from source/ports/jambox for a
  Felucca fork, or source/ports/x0x for a firmware with its own MIDI queues):
  - teensy_midi_in(status, d1, d2, source): one whole message, source 1 = USB, 2 = TRS: put it where the firmware's
    own MIDI input goes (Felucca forks: midi_enqueue in usb.c). Notes, CCs, clock, start, continue, stop.
  - teensy_midi_link(on): 1 while a computer has the USB MIDI port (Felucca forks: usb.config = on).
  - teensy_midi_out(out, cap): hand over the firmware's MIDI out as USB-MIDI packets (Felucca forks: drain midi_out_q).
  Without them the port still builds (empty defaults), but it has no MIDI.
- Add the firmware to source/build.rs (a `Fw` entry: dir, version, name, includes, extra C files, defines, stem,
  nor) and to the ROMS list in source/build.sh. The stem is its file name on the card: 8 capital letters at most.
- Memory is tight: about 384 KB ITCM (code), 128 KB DTCM (data and stack), 512 KB OCRAM. If the code is too big for
  ITCM, use `text_in_flash` or a list of functions that stay in flash, as JIANT does.
- Keep the original licence and every author credit. Only port firmwares whose licence allows it.

What I want from you:
1. Clone the repo and the firmware. Check its licence and tell me if it can't be ported.
2. Port it, build it with source/build.sh, and fix every error.
3. It must build for all three slots: roms/<STEM>.R1, .R2 and .R3.
4. Tell me which files to copy to my SD card's ROMS folder, and what to test, MIDI included: with the Teensy plugged
   into a computer it shows up as the MIDI device "BBL Mod Swapper (<name>)"; send it notes and clock, play the FM-1's
   keys and check MIDI comes back; and MIDI into the FM-1's TRS jack.
5. Write a short list of everything you changed, for my port post.

Ask me before you change anything outside ports/<name>/, build.rs and build.sh.
```

## When it works

Test it on your Mod Swapper, then post it: open a [New port](https://github.com/UnkleBlairy/BBL-MOD-SWAPPER/issues/new?template=new-port.yml) issue and drag in a zip with the three `.R` files and your `ports/<name>/` folder.

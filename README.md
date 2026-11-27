# BBL Mod Swapper

Turns an M-VAVE FM-1 into a Mod Swapper: a Teensy 4.1 runs open-source FM-1 firmwares (ported from the FM-1 Firmware Hub) and the FM-1 becomes its screen, keys and speaker. Swap firmwares from a menu, load samples and saves from an SD card. Optional audio in with PJRC's Teensy Audio Shield.

- **Build guide:** https://unkleblairy.github.io/BBL-MOD-SWAPPER/guide/
- **Installer:** https://unkleblairy.github.io/BBL-MOD-SWAPPER/ (Chrome or Edge on a computer). Step 1 flashes the Teensy, step 2 installs the Mod Swapper terminal on the FM-1, and "Back to stock" returns the FM-1 to official V15.
- **Ports and downloads:** https://unkleblairy.github.io/BBL-MOD-SWAPPER/ports/

## Ports

| Port | Version | By | Status |
|---|---|---|---|
| Felucca | built in | Leo Kuroshita | built into the Teensy |
| BBL Jam Box | 0.9 | Blairy (blairysbeatlab) | UNCONFIRMED |
| JIANT FM | 0.9.18 | juanjiant-bit | UNCONFIRMED |
| SLOOP | 2.8.5 | isod89 | UNCONFIRMED |
| X0X | 1.2.2 | Charles Vestal | UNCONFIRMED |

**UNCONFIRMED** means only its porter has tested it so far. A port becomes **CONFIRMED** once someone else reports it working.

## Tested one? Found a bug? Made a port?

Use the report form: https://unkleblairy.github.io/BBL-MOD-SWAPPER/ports/#report
Choose *It works*, *Bug*, *New port* or *Question*, say which FM-1 and Teensy (PSRAM or not), and what happened. It goes straight to Blairy.
Want to port a firmware yourself? The AI porting prompt is on the ports page: https://unkleblairy.github.io/BBL-MOD-SWAPPER/ports/#port

Not made or supported by M-VAVE or PJRC. GPL-3.0, no warranty. Each port keeps its own authors' credits and licence. The installer's FM-1 update code is the Felucca web installer by Leo Kuroshita (Hügelton Instruments).

# Ports

**Easier: the ports page, with direct downloads and a report form: https://unkleblairy.github.io/BBL-MOD-SWAPPER/ports/**

FM-1 firmwares ported to the BBL Mod Swapper. Each one goes on your SD card and shows up in the Mod Swapper's menu.

**Get them all at once: [ALL-PORTS.zip](ALL-PORTS.zip)**. Or open a port's folder below for just that one.

| Port | Version | Original by | Status | Download |
|---|---|---|---|---|
| [BBL Jam Box](JAMBOX/) | 0.9 | Blairy (blairysbeatlab) | UNCONFIRMED | [JAMBOX.zip](JAMBOX/JAMBOX.zip) |
| [JIANT FM](JIANT/) | 0.9.18 | juanjiant-bit | UNCONFIRMED | [JIANT.zip](JIANT/JIANT.zip) |
| [SLOOP](SLOOP/) | 2.8.5 | isod89 | UNCONFIRMED | [SLOOP.zip](SLOOP/SLOOP.zip) |
| [X0X](X0X/) | 1.2.2 | Charles Vestal | UNCONFIRMED | [X0X.zip](X0X/X0X.zip) |

Plus **113 more ports, untested on hardware yet**: search and download them on the ports page: https://unkleblairy.github.io/BBL-MOD-SWAPPER/ports/#more

Felucca (by Leo Kuroshita) is built into the Teensy, so it's always in the menu with no file needed.

A port stays **UNCONFIRMED** until someone other than its porter tests it. Tried one? [Report it](https://github.com/UnkleBlairy/BBL-MOD-SWAPPER/issues/new?template=test-report.yml).

## Put ports on your SD card

1. Click a zip above, then press the download button (the arrow at the top right of the file view).
2. Unzip it. Inside is a `ROMS` folder.
3. Copy that `ROMS` folder to the top of your SD card. If the card already has a `ROMS` folder, let the files go into it.
4. Card back in the Teensy, switch the FM-1 on, pick the port in the menu.

Each port is three files: `NAME.R1`, `NAME.R2` and `NAME.R3`, the same firmware built for each of the Teensy's three flash slots. Keep all three. The menu shows up to 160 firmwares, Felucca included (release 1.1; release 1 showed 16).

To remove a port, delete its three files from `ROMS`. Its saves are in `DATA` on the card: delete those too if you don't want them.

## Post your own port

Porting a firmware is a job you can hand to an AI coding helper (Claude Code, Codex, Cursor and the like):

1. Copy the prompt in **[PORTING_PROMPT.md](PORTING_PROMPT.md)** into your AI helper, with the name of the FM-1 firmware you want ported.
2. It builds the three `.R1` `.R2` `.R3` files. Test them on your Mod Swapper.
3. Open a **[New port](https://github.com/UnkleBlairy/BBL-MOD-SWAPPER/issues/new?template=new-port.yml)** issue, fill in the form and drag in a zip of the three files and your changed source. No git knowledge needed.

It gets added here as UNCONFIRMED, with your name on it.

Only firmwares whose licence allows it (GPL, MIT and the like). The original authors' credits stay on.

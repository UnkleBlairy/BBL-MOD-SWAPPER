# Teensy <-> FM-1 link protocol

Version 1. For now the PC relays between the two: Teensy USB serial, PC relay, FM-1 USB serial (CDC). Later the Teensy's USB host port connects to the FM-1 directly, using the same packets.

Every packet has the same framing:

    A5 5A type len_lo len_hi payload[len]

Multi-byte numbers are little-endian. A receiver skips any type it does not know.

## Teensy -> FM-1
| type | payload |
| --- | --- |
| `A` | Audio: 128 stereo frames, i16 L then i16 R (512 bytes). 44.1 kHz. |
| `L` | Screen spans, one after another: `y x n` (u8 each; n = 1..240 pixels, x + n <= 240), then RLE pixels until n pixels are made. RLE header byte `h`: `h & 0x80` means a run of `(h & 0x7F) + 1` copies of the next pixel; otherwise `h + 1` literal pixels follow. A pixel is 2 bytes in the panel's byte order (high byte first), written straight to the ST7789. |
| `D` | LEDs: `fm1_led[11] fm1_led_dim[11] fm1_led_breath[11] fm1_led_mid[11] dim_lo` (45 bytes), exactly as felucca-bsp's `fm1_input.h` uses them. Sent when they change. |
| `Z` | Sync: `gen u8`. The FM-1 empties its audio FIFO, sets `played` to 0, and echoes `gen` in every `I` from then on. The Teensy counts its sent frames from the same point, so `sent - played` is exact. The Teensy sends a new `Z` if no `I` with its `gen` comes back within 500 ms, for example after the FM-1 reboots. |

## FM-1 -> Teensy
| type | payload |
| --- | --- |
| `I` | Input and audio state, every 4 ms: `notes u32` (fm1_in.notes), `buttons u32` (fm1_in.buttons, raw matrix bits), `enc i8[7]` (steps since the last `I`, by hardware encoder), `master u16`, `batt u16` (ADC 0..1023), `played u32` (audio frames played since the last `Z`), `underruns u16`, `fifo u16` (frames waiting), `gen u8` (the last `Z`'s) = 30 bytes. |
| `R` | Ask for a full redraw (no payload). |

## Pacing
The FM-1's audio clock is the master. The Teensy renders while `sent - played < TARGET` (4096 frames, 93 ms, which covers the relay), so it follows the FM-1's clock exactly with no drift control. If no `I` arrives for 300 ms, the Teensy falls back to its own clock.

## PC -> Teensy (link mode)
`L\n` on the Teensy's serial port enters link mode: mux output (`A`, `L`, `D`, `T` text) plus packetised input. In link mode, everything sent to the Teensy is packets:
- `I` and `R` from the FM-1, passed through
- `C`: a text command line, the same commands as outside link mode (`B`, `K`, `E`, `V`, `I`)

Closing the port leaves link mode.

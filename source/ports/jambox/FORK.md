# Jam Box: Mod Swapper fork

From 2026-11-20 this copy is a **fork** of the native FM-1 Jam Box (`~/Felucca`, branch smk37). Features that need
the Teensy (line in, PSRAM track buffers, per-track recording) exist only here and can't go back to the native FM-1.

- **Fork point:** `~/Felucca` 598cff9 (Jam Box 0.9, private). The last native commit carried forward is in
  `SOURCE_COMMIT.txt`.
- **Native stays the master for shared code.** Bug fixes are made and tested on the native FM-1 in `~/Felucca`, then
  carried here: `git -C ~/Felucca diff <last synced>..HEAD -- firmware/src`, applied 3-way, then `SOURCE_COMMIT.txt`
  updated. Never the other way.
- **Keep the fork easy to merge into:** new features go in new files; the hooks in shared files are small and inside
  `#if defined(TEENSY_NOR_ADDR)` (the macro the port already builds with), so native patches still apply cleanly.
- **Saves:** fork-only fields (track buffers, R1-R6 slots) get their own version / tag, so a native jam still loads
  here (its sample slots moved up by 6) and a fork jam is never mistaken for a native one.

## Fork-only features
- Track recording phase 1 (docs/TRACK_REC.md): REC + REC IN pages, PSRAM pool, R1-R6 as sampler slots (trec.c,
  trec_ui.c)

# Track recording (Mod Swapper fork only) — agreed design, 2026-11-20

See FORK.md: fork-only code, new files + small hooks under `#if defined(TEENSY_NOR_ADDR)`.

## Pages (per track, FX button family, after SENDS: SENDS -> REC -> REC IN)
Hold REC stays as it is (quantized record on/off). Don't touch.
- REC:    K1 ARM OFF/ON | K2 START REC / PLAY / SYNC (SOUND: phase 2) | K3 LEN 1/4 1/2 1 2 4 8 PAT | K4 MODE ONE SHOT / REPEAT / LOOPER
- REC IN: K1 SRC LINE IN / TRACK / MASTER | K2 GAIN | K3 TRIG (SOUND start) | K4 DUB (looper feedback)
- LEN counts in the track's own pattern length (SLEN x SDIV), fixed when a take starts.
- SRC TRACK = the track's own sound, before the sends. MASTER = the whole mix with FX (resample).
- Monitoring: SRC LINE IN is heard through the track (its VOL/PAN/sends) while armed or recording.
- CLEAR on REC IN (turn right twice, a pause between, within 3 s) clears the track's buffer. (Was EDIT twice: it trapped EDIT on the REC pages.)
- REC settings save with the jam (the fork's save version). Not lockable.

## Modes
- ONE SHOT: one take of LEN, then ARM turns itself off.
- REPEAT: a new take every LEN until ARM is off. It records into fresh pool space, then swaps in on the bar line,
  so a sampler chopping the buffer never hears a half-written one.
- ONE SHOT / REPEAT do NOT play back. Another track's sampler plays the buffer (slot R1-R6).
- LOOPER: pass 1 records LEN. After that the loop plays back on this track (through its VOL, PAN and sends), and
  every pass overdubs: new = old * DUB + input. ARM off = stop overdubbing (the loop keeps playing), ARM on = resume.
  SRC TRACK + LOOPER = play notes freely, layered each pass (sequenced notes stack too: "for people to discover").
  The loop is still slot Rn, so other samplers can chop it live.

## Buffers / pool / slots
- 16-bit, 44.1 kHz, mono (up from 8-bit mu-law at 22 kHz). The sampler reads 16-bit buffers as well as mu-law chunks.
- Pool: 12 MB of PSRAM (about 2 min 20 s), shared dynamically by the 6 tracks; "POOL FULL" if it runs out.
  The other 4 MB (about 47 s) is for the Rust Sample Lab recorder (src/audio_in.rs, which uses all 16 MB today).
- Short fades (~2 ms) at loop edges.
- SLOT knob: R1-R6 (track buffers) first, then S01-S32. Native jams load with their slots moved up by 6.
- Timing: bar length to the sample from tempo + pattern length; codec vs FM-1 drift corrected; line-in LATENCY offset
  (~46 ms FM-1 output delay) with auto-calibrate (FM-1 out -> line in, click) in phase 2.

## Build order
1. Line in into the engine (C side reads the SAI ring in step with its 128-sample blocks), pool, R1-R6 + 16-bit
   playback, REC + REC IN pages, START REC/PLAY, LEN, ONE SHOT, SRC LINE IN + TRACK. Flash, test timing over USB,
   then the user tests on the FM-1.
2. REPEAT, LOOPER + DUB, SOUND start, SRC MASTER, latency calibration.
3. Buffers saved to the card (WAV in SAMPLES/RECORDED) and in save states.

Status: phase 1 built 2026-11-20 (firmware/src/trec.c audio + trec_ui.c pages; Teensy side src/audio_in.rs).

## Phase 1 as built
- Files: `firmware/src/trec.c` (state, takes, line in, the audio hooks), `trec_ui.c` (REC / REC IN pages). Hooks under
  `#if defined(TEENSY_NOR_ADDR)` in fx.c (mix_block: trec_begin; mix_part: trec_part), params.c (2 pages last in
  PAGES, GR_TREC), ui_gb.c, ui_input.c (knobs), eng_sampler.c (SMP_R, 16-bit reads), gb_store.c (slot
  numbering, the jam's "TRC1" section), web/emu/felucca_teensy.c (includes, trec_poll).
- The Jam Box swaps the FX and EDIT buttons (ui_input.c FAM_BTN): "FX button" / "EDIT" here are the Jam Box's names.
- REC: ARM, START REC/PLAY, LEN 1/4 1/2 1 2 4 8 PAT, MODE "1 SHOT" (ONE SHOT doesn't fit a dial). REC IN: SRC LINE /
  TRACK, GAIN -40..+24 dB. TRIG and DUB: phase 2. Title: REC Rn, Rn WAIT, Rn 45%. The buffer's waveform under the dials.
- START REC = the next bar line (master count, 16 x 1/16) while the sequencer records (REC on); PLAY = at PLAY.
- A bar = 16 of the track's steps (div_samples(SDIV)); PAT = the track's LEN. Fixed when the take starts.
- Pool: PSRAM after the Lab recorder's first 4 MB (its ring is now 47 s, was 190 s): 12 MB with both chips, first fit,
  a take into fresh space when there is room (its own old buffer only if nothing else fits). POOL FULL / NO PSRAM.
- Slots: SLOT 0..5 = R1..R6, 6..37 = S01..S32. Sounds, kits and the jam store the native order (R after S, 32..37), so
  native saves load unchanged (gb_store.c smp_sl_out / smp_sl_in). Buffers are not saved yet (phase 3).
- Line in: the main loop starts the codec when a track asks (audio_in.rs need / live_start; never inside the render).
  The engine reads it 1024 samples behind the receiver and resamples (linear): its speed follows the averaged lag
  (P + slow I, within 1 %), so no skips (STAT line: line_in_ppm resyncs lag in_lat). Measured on the Teensy over USB
  (no FM-1): settles in ~15 s at +381 ppm, lag 1022 of 1024, 0 resyncs, load 10.1 -> 10.9 %. A LINE IN take skips teensy_in_latency() samples first (output lag
  + read lag + ~1 ms): calibration is phase 2.
- On the Teensy over USB 2026-11-20: REC / REC IN pages, TRACK take on PLAY ("R5 RECORDED", ARM off, waveform),
  LINE IN arms the Audio Shield, a LINE take completes; REC settings come back with the jam after a reload.
- Host test (PC build of the same C, fake line in): 1 bar at 120 = 88192 samples, the latency skip lands to the
  sample, START REC starts on mst_step % 16 == 0, a sampler plays R1, EDIT twice clears.


## INPUT machine (2026-11-21, Blair: "no line in on tracks that already have synths and samples")
- The line in is a MACHINE now, not a REC IN source: INPUT = MONO's machine 1 (P_E7 = 1; MACHINES' last row, after SMPL;
  saves as MONO, so no save format change). PRESETS knob past SMPL.
- INPUT page (EDIT, the old OSC 1): MODE THRU / GATE, GAIN -40..+24 dB, SAT. Then MONO's FILTER (CUT RES TYPE AMT, DRIVE)
  and FILTER ENV pages.
  - THRU: always open, no note needed (eng_mono.c mono_thru, called from trec_part): SAT, DRIVE, CUT, RES, TYPE. No filter
    envelope or LFO (they need a voice).
  - GATE: steps and keys open it (MONO's voice with the line in for its oscillators): ENV, filter envelope, LFOs all work.
- Path: line in -> GAIN -> SAT -> DRIVE -> filter -> the track's REC (TRACK, before VOL / PAN / sends) and the mix.
- REC IN SRC: TRACK only (shows INPUT on an INPUT track); MASTER in phase 2. Old jams' LINE loads as TRACK.
- The codec starts when any track is INPUT (trec_poll); an INPUT take skips teensy_in_latency() first.
- 2026-11-21 on the FM-1: THRU works (heard, filtered). GATE: first try "oscillating weird noise" (track 1 had been a
  drum track: likely its steps chopping the gate; settings checked clean via `R`: no LFO/matrix, RES 0, no voices);
  second try: a noticeable DELAY between pressing a key and hearing it. Debug next session (the line in is read 1024
  samples (~23 ms) behind the receiver + the FM-1 output lag; THRU hides it, GATE makes it audible against the key).

## 2026-11-21 evening
- START SYNC (Blair: "starts at the beginning of the loop whether I press play or not"): armed, the take starts on the
  track's own loop's step 1 (seq_idx 0 with a new gb_loop), or at PLAY; no sequencer REC needed. trec.c trec_begin.
- Line in fixes (src/audio_in.rs): a flash erase (LAST JAM save after a take, interrupts off 45-150 ms) overran the SAI
  FIFO and the receiver stopped for good (INPUT "muted", the take silent, likely GATE's "oscillating noise" too: the
  reader looping 384 stale samples). Now FCONT (goes on after an overrun), a watchdog restart (no new sample for ~93 ms),
  the reader's speed reset on each resync. Read lag 1024 -> 384 (measured swing -135 / +330). Serial R prints LINEIN
  lines (lag range, overruns, restarts, link target, FM-1 underruns) and out_peak / MASTER.
- FM-1 link buffer: 1024 frames (Q1024) tested live, halves key latency (46 -> 23 ms); default still 2048 until
  checked while jamming (underruns 62 -> 67 in one session at 1024, maybe the save pauses).

## 2026-11-21 late: LOOPER, DUB, INPUT trigs (Blair)
- GATE dropped ("I won't use it"): INPUT is always open; its page is GAIN SAT (params.c; E0 no longer read).
- A step or key on an INPUT track plays the track's own take (Rn) from its start, over the line in (trec.c trec_trig,
  from eng_mono.c mono_note_on); middle C = as recorded, the note sets the speed. Raw buffer, no filter / ENV (it was
  recorded through them).
- REC MODE: STD (one take, ARM turns off) / LOOPER. LOOPER: pass 1 records LEN, then the loop plays round through the
  track (VOL PAN sends) while playing (PLAY = from its start); ARM on = every pass overdubs new = old x DUB + this,
  ARM off = it just plays. CLEAR (REC IN) clears it (and stops it). MODE back to STD stops the loop.
- REC IN K3 DUB 0..100 % (stored as 100 - DUB in the old rsv0 byte: old jams = 100 %). DUB 0 = a fresh take every pass
  (what REPEAT was going to be: no REPEAT mode).
- INPUT: the loop's playback and overdubs allow for the round trip (teensy_in_latency): a click played on the beat lands
  on the same sample every pass (tools/trec_test.c, PC test of the real C: 21 checks).
- Track squares: the red dot also while the LOOPER overdubs (trec_rec_now).
- 2026-11-21 (Blair): a new take WIPES the old one as it starts (sound, waveform, pool space); a trig while the slot
  records waits (counting time) and plays the new take from that point once it ends (step 1 comes ~33 ms before an
  INPUT take ends). LOOPER: ARM on with a loop still layers onto it (clear with CLEAR on REC IN for a fresh loop).
- The R slot's waveform is built while recording (trec_pk): reading the PSRAM back in a UI frame stalled the audio.
- A take records the track before its trig's playback (only the new input, not the old take).
- LOOPER, REC layer / PLAY layer (Blair: "you have to have a trig down to hear the loop, that keeps it uniform with
  how the other modes work"): the loop is never heard round on its own. Each trig / key plays the whole loop from its
  start at its note (INPUT tracks: trec_trig), as STANDARD's take. The REC layer keeps time and overdubs underneath per
  the REC settings.
- PLAYBACK page (INPUT's page 2, Blair 2026-11-21): PITCH (-24..+24 st on top of the note), STRCH (the sampler's grain
  stretch, 0 = 1.00x), LOFI (the sampler's sample-hold + bits), MODE ONCE / LOOP / PING / REV (the sampler's icons;
  LOOP / PING while the step or key is held, then a 256-sample fade). P_E2..P_E5, so saved, lockable, LFO / macro.
  trec.c trec_tp_go / trec_tp_run, the sampler's tables.
- INPUT's path split round the take (Blair: "the audio looped straight after the input page, the effects chain
  afterwards"): line in -> GAIN SAT (mono_in_pre) -> [recorded here] + the take playing -> DRIVE, filter (mono_in_post)
  -> VOL PAN sends. A take is the clean input; the loop goes through the filter like the input. Other tracks record
  their full sound as before.

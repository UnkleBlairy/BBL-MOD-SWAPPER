/* PC test of the track recording (Mod Swapper fork, docs/TRACK_REC.md): LOOPER, DUB, trig playback, overdub timing.
 *   gcc -std=gnu99 -O1 -w -ffp-contract=off -DFELUCCA_VERSION=\"0.9\" -DTEENSY_NOR_ADDR=1 -Ibuild/gen -Ifirmware/src \
 *       -Ifirmware/hal -Iweb/emu -o build/trec_test tools/trec_test.c -lm && build/trec_test
 * (build/gen from ./build.sh; -O0 does not assemble: the FM-1 HAL's inline asm) */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
static uint8_t fnor[1 << 21];
#undef TEENSY_NOR_ADDR
#define TEENSY_NOR_ADDR ((uintptr_t)fnor)
#include "felucca_teensy.c"
static int16_t *pool; static uint32_t lin_t; static int lin_amp = 8000;
int16_t *teensy_pool(uint32_t *n) { if (!pool) pool = calloc(6u << 20, 2); *n = 6u << 20; return pool; }
static uint32_t click_every;   /* nonzero: the line in is a click 1452 + 500 + m x this after lin_t 0 (played on the beat) */
uint32_t teensy_line_in(int16_t *d, uint32_t n) { if (click_every) { for (uint32_t i = 0; i < n; i++, lin_t++) d[i] = lin_t >= 1952 && (lin_t - 1952) % click_every == 0 ? 8000 : 0; return 1; } for (uint32_t i = 0; i < n; i++, lin_t++) d[i] = (int16_t)(lin_amp * sin(6.283185307 * 220.0 * lin_t / 44100.0)); return 1; }
uint32_t teensy_in_latency(void) { return 1452; }
uint32_t teensy_line_in_state(void) { return 1; }
void teensy_line_in_arm(void) {}
int32_t teensy_nor_erase(uint32_t off) { memset(fnor + (off & ((1 << 21) - 1)), 0xFF, 4096); return 0; }
int32_t teensy_nor_prog(uint32_t off, const uint8_t *s, uint32_t n) { memcpy(fnor + (off & ((1 << 21) - 1)), s, n); return 0; }
uint32_t teensy_nor_written(void) { return 0; }
static void run(uint32_t n) { for (uint32_t k = 0; k < n; k += 128) web_render(128); }
static int pk(const int16_t *w, uint32_t a, uint32_t b) { int p = 0; for (uint32_t i = a; i < b; i++) p = abs(w[i]) > p ? abs(w[i]) : p; return p; }
static voice_t tgv;
static void tg(int32_t p16) { tgv.pitch16 = p16; tgv.active = tgv.gate = 1; trec_trig(&trk[0], &tgv); }
static float outpk(uint32_t n) { float p = 0; for (uint32_t k = 0; k < n; k += 128) { web_render(128); for (int i = 0; i < 128; i++) p = fabsf(out_l[i]) > p ? fabsf(out_l[i]) : p; } return p; }
static int fails;
#define CHECK(c, ...) do { printf("%s: ", (c) ? "ok  " : "FAIL"); printf(__VA_ARGS__); printf("\n"); if (!(c)) fails++; } while (0)
int main(void)
{
    memset(fnor, 0xFF, sizeof fnor);
    web_boot(); run(44100 * 2);
    for (int k = 0; k < NTRK; k++) for (int i = 0; i < NSTEP; i++) memset(&trk[k].step[i], 0, sizeof trk[k].step[i]);   /* no steps */
    song.sel = 0;
    machine_set(&trk[0], MACH_INPUT); run(44100);
    CHECK(trk_in(&trk[0]), "track 1 is INPUT");
    float thru = outpk(4410);
    CHECK(thru > 0.01f, "INPUT always open with no note (out peak %.3f)", thru);
    /* LOOPER, 1 bar, START PLAY */
    trec_arm(0);
    trec_set[0] = (trec_set_t){.arm = 1, .start = TS_PLAY, .len = 2, .mode = TM_LOOP, .src = TSRC_TRACK};
    trec_gain_set(0);
    seq_start();
    uint32_t want = trec_len(&trk[0], &trec_set[0]);
    printf("1 bar = %u samples, skip %u\n", want, teensy_in_latency());
    if (getenv("CLICK")) goto click;
    run(256);
    CHECK(trec[0].st == 2, "the take started at PLAY (st %u)", trec[0].st);
    run(want + 1452 + 256);
    CHECK(trec[0].loop == 1 && smp_tab[0].len == want, "pass 1 done: looping (loop %u len %u)", trec[0].loop, smp_tab[0].len);
    CHECK(trec_set[0].arm == 1, "LOOPER keeps ARM on");
    int16_t *w = (int16_t *)smp_tab[0].w;
    int p1 = pk(w, 1000, want - 1000);
    CHECK(p1 > 1000, "the loop has the line in (peak %d)", p1);
    CHECK(trec_rec_now(0), "overdubbing shows as recording (red dot)");
    /* overdub at DUB 100 %: louder after a pass */
    run(want);
    int p2 = pk(w, 1000, want - 1000);
    CHECK(p2 > p1 * 13 / 10, "a pass of overdub at DUB 100%% adds up (peak %d -> %d)", p1, p2);
    /* ARM off: it plays, no more writes */
    trec_set[0].arm = 0;
    run(512);
    int16_t snap = w[want / 2]; int p3a = pk(w, 1000, want - 1000);
    run(want);
    CHECK(trec[0].loop == 1 && w[want / 2] == snap && pk(w, 1000, want - 1000) == p3a, "ARM off: the loop stays as it was");
    CHECK(!trec_rec_now(0), "ARM off: no red dot");
    lin_amp = 0;
    tg(60 * 16);
    float lo = outpk(want / 2);
    CHECK(lo > 0.01f, "a trig: the loop is heard through the track, the line in silent (out peak %.3f)", lo);
    /* DUB 0 %: a fresh take each pass */
    lin_amp = 3000; trec_set[0].dubx = 100; trec_set[0].arm = 1;
    run(want * 2 + 2048);
    int p4 = pk(w, 2000, want - 2000);
    CHECK(p4 > 2000 && p4 < 4000, "DUB 0%%: each pass replaces the loop with the input (peak %d, input 3000)", p4);
    /* LOOPER: the loop is heard only through trigs (the PLAY layer); the REC layer keeps time underneath */
    {
        trec_set[0].arm = 0;                       /* (no overdub: DUB 0 would wipe it with the silent input) */
        lin_amp = 0;
        trec_tp[0].on = 0;
        run(44100 * 3);                            /* (the trig before's reverb / delay tail) */
        float off = outpk(4096);
        CHECK(off < 0.002f, "no trig: the loop isn't heard (%.4f)", off);
        uint32_t lp0 = trec[0].lpos; run(4096);
        CHECK(trec[0].lpos != lp0, "the REC layer goes on in time underneath");
        tg(60 * 16);
        float tr2 = outpk(4096);
        CHECK(trec_tp[0].w == smp_tab[0].w && tr2 > 0.002f, "a trig plays the whole loop (%.4f)", tr2);
        lin_amp = 3000;
        trec_set[0].arm = 1;
    }
    /* PLAY again: the loop from its start */
    seq_stop(); run(1024);
    seq_start(); run(64);
    CHECK(trec[0].lpos < 256, "PLAY again: the loop restarts (lpos %u)", trec[0].lpos);
    /* clear, then STANDARD + a trig plays the take */
    trec_set[0].arm = 0; run(256);
    song.sel = 0; trec_clear(); run(256);
    CHECK(!trec[0].loop && !smp_tab[0].w, "EDIT twice: cleared, the loop stops");
    lin_amp = 8000;
    trec_set[0] = (trec_set_t){.arm = 1, .start = TS_PLAY, .len = 2, .mode = TM_STD, .src = TSRC_TRACK};
    trec_gain_set(0);
    seq_stop(); run(1024); seq_start(); run(want + 1452 + 1024);
    CHECK(smp_tab[0].len == want && !trec[0].loop && !trec_set[0].arm, "STANDARD: one take, ARM off, no loop");
    lin_amp = 0; run(4096);
    float quiet = outpk(4096);
    tg(60 * 16);
    CHECK(trec_tp[0].on && trec_tp[0].step == 65536, "middle C plays the take at its own speed (step %u)", trec_tp[0].step);
    float tr = outpk(8192);
    CHECK(tr > quiet + 0.01f, "the trig is heard (out %.3f, before %.3f)", tr, quiet);
    tg(72 * 16);
    CHECK(trec_tp[0].step > 130000 && trec_tp[0].step < 132000, "an octave up = twice as fast (step %u)", trec_tp[0].step);
    /* a trig at step 1 just before a new take ends (the round trip) goes on in the new take */
    lin_amp = 8000; trec_set[0].arm = 1; trec_set[0].start = TS_PLAY;
    seq_stop(); run(1024); seq_start(); run(128);
    CHECK(!smp_tab[0].w && trec[0].st == 2, "a new take wipes the old one as it starts");
    run(want - 128 - 512);
    tg(60 * 16);                   /* (as the step's note would, a little before the end) */
    CHECK(trec_tp[0].pend && !trec_tp[0].on, "a trig while it records waits");
    run(512 + 1452 + 256);
    CHECK(smp_tab[0].w && trec_tp[0].on && trec_tp[0].w == smp_tab[0].w && (trec_tp[0].c >> 8) > 1452,
          "the take ends: the trig plays the new one from where it has got to (at %d)", (int)(trec_tp[0].c >> 8));
    /* STANDARD: a new take while the old one plays from a trig has only the new input in it (not the old take) */
    {
        int pko = 0; for (uint32_t i = 0; i < smp_tab[0].len; i++) pko = abs(smp_tab[0].w[i]) > pko ? abs(smp_tab[0].w[i]) : pko;
        int pkc = 0; for (int i = 0; i < GB_WAVE; i++) pkc = trec_pk[0][i] > pkc ? trec_pk[0][i] : pkc;
        CHECK(pko > 1000 && abs(pkc * 128 - pko) < 256, "the screen's waveform matches the take (columns %d, take %d)", pkc * 128, pko);
        lin_amp = 0; trec_set[0].arm = 1;
        seq_stop(); run(1024); seq_start(); run(128);
        tg(60 * 16);                 /* the old take plays while the new one records a silent input */
        run(want + 1452 + 256);
        int pkn = 0; for (uint32_t i = 0; i < smp_tab[0].len; i++) pkn = abs(smp_tab[0].w[i]) > pkn ? abs(smp_tab[0].w[i]) : pkn;
        CHECK(pkn < 200, "the new take has only the (silent) input, not the old take playing (peak %d)", pkn);
    }
    /* PLAYBACK (INPUT's page 2) and the path: the take recorded before the filter, played back through it */
    {
        int16_t *tp_ = trk[0].p;
        seq_stop(); run(1024); trec_clear(); run(256);
        lin_amp = 8000; tp_[P_FLT_CUT] = 20; run(4096);
        trec_set[0] = (trec_set_t){.arm = 1, .start = TS_PLAY, .len = 2, .mode = TM_STD, .src = TSRC_TRACK};
        trec_gain_set(0);
        seq_start(); run(want + 1452 + 256);
        int tk = smp_tab[0].w ? pk(smp_tab[0].w, 0, smp_tab[0].len) : 0;
        CHECK(tk > 6000, "a take is the input before the filter (CUT 20: take peak %d)", tk);
        tp_[P_FLT_CUT] = 127; lin_amp = 0; run(44100);
        tg(60 * 16); float open = outpk(4096);
        tp_[P_FLT_CUT] = 5; run(44100 / 4);
        tg(60 * 16); float shut = outpk(4096);
        tp_[P_FLT_CUT] = 127; run(44100 / 4);
        CHECK(open > 0.01f && shut < open / 3, "the take plays through the filter (CUT open %.3f, shut %.3f)", open, shut);
        tp_[P_E2] = 12; tg(60 * 16); run(128);
        CHECK(trec_tp[0].step > 130000 && trec_tp[0].step < 132000, "PITCH +12: twice as fast (step %u)", trec_tp[0].step);
        tp_[P_E2] = 0;
        tg(60 * 16); run(8192); int32_t c1 = trec_tp[0].c >> 8;
        tp_[P_E3] = 63; tg(60 * 16); run(8192); int32_t c2 = trec_tp[0].c >> 8;
        tp_[P_E3] = -64; tg(60 * 16); run(8192); int32_t c3 = trec_tp[0].c >> 8;
        tp_[P_E3] = 0;
        CHECK(c1 > 7000 && c1 < 9400 && c2 < c1 / 3 && c3 > c1 * 3, "STRCH: 1.00x at 0, slower up, faster down (%d %d %d samples in)", c1, c2, c3);
        uint32_t L = smp_tab[0].len;
        tg(60 * 16); run(L + 4096);
        CHECK(!trec_tp[0].on, "ONCE: stops at the end");
        tp_[P_E5] = SM_REV; tg(60 * 16); run(256);
        float rv = outpk(2048);
        CHECK(trec_tp[0].on && rv > 0.01f, "REV plays (%.3f)", rv);
        run(L + 4096);
        CHECK(!trec_tp[0].on, "REV: stops at its end");
        tp_[P_E5] = SM_LOOP; tg(60 * 16); run(L * 2 + 4096);
        float lp2 = outpk(2048);
        CHECK(trec_tp[0].on && lp2 > 0.01f, "LOOP: goes round while the note is held (%.3f)", lp2);
        tgv.gate = 0; run(1024);
        CHECK(!trec_tp[0].on, "LOOP: let go, it fades out");
        tp_[P_E5] = SM_PING; tg(60 * 16); run(L * 3 + 512);
        CHECK(trec_tp[0].on, "PING: goes on while the note is held");
        tgv.gate = 0; run(1024); tp_[P_E5] = SM_ONCE;
        tp_[P_E4] = 127; tg(60 * 16);
        { int nz = 0, bad = 0; for (int i = 0; i < 64; i++) { run(128); nz += trec_tp[0].y != 0; bad += (trec_tp[0].y & 0xFFF) != 0; }
          CHECK(nz && !bad, "LOFI 127: 4 bits (%d blocks heard, %d off the steps)", nz, bad); }
        tp_[P_E4] = 0; trec_tp[0].on = 0; lin_amp = 8000;
    }
click:
    /* timing: a click played on the beat lands in the same place every pass (the round trip allowed for) */
    seq_stop(); run(1024); song.sel = 0; trec_clear(); run(256);
    trec_set[0] = (trec_set_t){.arm = 1, .start = TS_PLAY, .len = 2, .mode = TM_LOOP, .src = TSRC_TRACK};
    trec_gain_set(0);
    click_every = want; lin_t = 0;
    seq_start(); run(want + 1452 + 256);   /* (pass 1 just done: the click's first overdub comes 500 later) */
    w = (int16_t *)smp_tab[0].w;
    int at1 = 0; for (uint32_t i = 0; i < want; i++) if (abs(w[i]) > abs(w[at1])) at1 = (int)i;
    int v1 = abs(w[at1]);
    int16_t b1[16]; for (int i = 0; i < 16; i++) b1[i] = w[at1 - 6 + i];
    if (getenv("ZERO")) for (int i = 0; i < 16; i++) w[at1 - 6 + i] = 0;
    run(want);
    printf("pass1:   "); for (int i = 0; i < 16; i++) printf("%6d", b1[i]); printf("\noverdub: "); for (int i = 0; i < 16; i++) printf("%6d", w[at1 - 6 + i]); printf("\n");
    int at2 = 0; for (uint32_t i = 0; i < want; i++) if (abs(w[i]) > abs(w[at2])) at2 = (int)i;
    CHECK(at1 >= 495 && at1 <= 505, "pass 1: the click at sample %d of the loop (played at 500)", at1);
    CHECK(at2 == at1 && abs(w[at2]) > v1 * 3 / 2, "overdub: the click lands on itself (at %d, %d -> %d)", at2, v1, abs(w[at2]));
    printf("%s\n", fails ? "SOME FAILED" : "ALL OK");
    return fails != 0;
}

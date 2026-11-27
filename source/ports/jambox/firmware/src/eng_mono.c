/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2026 Blair (blairysbeatlab), BBL JAM BOX */
/* MONO: one analog-style voice in the vein of a Minimoog (not a copy of one). Two oscillators, a square sub, a drive
 * before the filter; the filter, its envelope and the ENV are DIGITAL's pages (FILTER / FILTER ENV / ENV).
 *   OSC 1: WAVE  the two oscillators' waves (SAW SQR TRI SIN in pairs, then each doubled), MIX  osc 1 .. osc 2,
 *          FINE  osc 2 -1 .. +1 semitone (centre: in tune), SEMI  osc 2 0 .. +24 semitones
 *   OSC 2: SUB  -1 / -2 / -3 octaves (a square), SUBVL  its level, PW  osc 1's shape (SQR: the pulse width,
 *          TRI: a wavefold, SIN: fewer and fewer levels to a square, SAW: a hidden oscillator it is hard synced to,
 *          1x .. 8x the note), SAT (P_LN0)  drive into a soft clip before the filter
 *   FILTER: its own (mono_filter): CUT RES, TYPE (P_LN1 on the FILTER page, params.c page_id_an) a blend LP .. BP .. HP,
 *          the filter envelope's AMT; RES from the top quarter rings, squeals and whistles on its own (MS-20-like)
 * Band-limited (polyBLEP) saws, pulses, sync. One voice (.poly 1); every press retriggers both envelopes (V_MONO).
 * The voice's state: ph[0] osc 1, ph[1] osc 2, ph[2] the sub; s[0..1] the filter, s[2..3] its envelope (as SAMPLER's),
 * s[4..5] osc 1 / 2's sync partner, s[6..7] their sync jump (the sample before the reset to the one after it). */
#define MONO_NW 10u
static const char *const N_MWAVE[MONO_NW] = {"SW+SQ", "SW+TR", "SW+SN", "SQ+TR", "SQ+SN", "TR+SN",
                                             "SW+SW", "SQ+SQ", "TR+TR", "SN+SN"};
enum { MW_SAW, MW_SQR, MW_TRI, MW_SIN };
static const uint8_t MONO_W[MONO_NW][2] = {{MW_SAW, MW_SQR}, {MW_SAW, MW_TRI}, {MW_SAW, MW_SIN}, {MW_SQR, MW_TRI},
                                          {MW_SQR, MW_SIN}, {MW_TRI, MW_SIN}, {MW_SAW, MW_SAW}, {MW_SQR, MW_SQR},
                                          {MW_TRI, MW_TRI}, {MW_SIN, MW_SIN}};
static const char *const N_MSUB[3] = {"-1", "-2", "-3"};
#if defined(TEENSY_NOR_ADDR)
/* teensy-fm1 fork: INPUT, MONO's machine 1 (P_E7 = 1): the Audio Shield's line in in place of the oscillators, into
 * MONO's SAT, DRIVE and filter (CUT RES TYPE), then the track (VOL, PAN, sends) and its REC.
 *   Always open (no note needed; mono_in_pre / mono_in_post, from trec.c trec_part; GATE mode was dropped, Blair
 *   2026-11-21). A step or key plays the track's own take (R slot, trec.c trec_trig) with the line in, both through
 *   DRIVE and the filter (the take is recorded before them). Page 2 PLAYBACK: PITCH STRCH LOFI MODE. GAIN: -40 .. +24 dB into the machine.
 * The line in arrives a block at a time (trec.c trec_begin fills in_buf). docs/TRACK_REC.md */
#define MONO_NM 2u
static const char *const N_MMACH[MONO_NM] = {"MONO", "INPUT"};
static const char *const N_INMODE[4] = {"ONCE", "LOOP", "PING", "REV"};   /* (the sampler's MODE: its icons) */
static const param_desc_t IN_D[7] = {   /* page 1 INPUT: GAIN (SAT P_LN0); page 2 PLAYBACK: the take's (trec.c trec_tp) */
    {"", F_INT, 0, 0, 0, 0, 0}, {"GAIN", F_INT, -40, 24, 0, 0, "dB"}, {"PITCH", F_INT, -24, 24, 0, 0, 0},
    {"STRCH", F_INT, -64, 63, 0, 0, 0}, {"LOFI", F_INT, 0, 127, 0, 0, 0}, {"MODE", F_ENUM, 0, 3, 0, N_INMODE, 0},
    {"", F_INT, 0, 0, 0, 0, 0}};
static int16_t in_buf[32];                              /* this block's line in (trec.c trec_begin) */
static uint8_t in_ok;                                   /* .. read (0: silence) */
#define MONO_IN(t) ((t)->p[P_E7] == 1)
static int32_t in_gq12(const track_t *t)               /* GAIN as Q12: 4096 x 10^(dB / 20) */
{
    int32_t g = 4096, d = clamp(t->p[P_E1], -40, 24);
    for (; d > 0; d--)
        g = (int32_t)(((int64_t)g * 73533) >> 16);
    for (; d < 0; d++)
        g = (int32_t)(((int64_t)g * 58409) >> 16);
    return g;
}
static const param_desc_t *mono_desc(const track_t *t, uint32_t k)
{
    return MONO_IN(t) && k < 7u ? &IN_D[k] : 0;
}
#else
#define MONO_NM 1u
static const char *const N_MMACH[MONO_NM] = {"MONO"};
#define MONO_IN(t) 0
#endif
static const param_desc_t MONO_SAT = {"SAT", F_INT, 0, 127, 0, 0, 0};   /* P_LN0 on a MONO track (params.c track_desc) */
static const param_desc_t MONO_TYPE = {"TYPE", F_INT, 0, 127, 0, 0, 0};  /* P_LN1: 0 LP, 64 BP, 127 HP, blended between */

/* the filter: eng_digital.c mono_filter (FM SYN's too) */

/* osc 2's tuning: Q16 ratio to osc 1 for d 1/64 semitones (0 .. +24 semitones + -1 .. +1) */
static uint32_t mono_ratio(int32_t d)
{
    uint32_t base = pitch_inc(60u * 16u), a, b, x;
    int32_t p = 60 * 16 * 4 + d;
    a = pitch_inc((uint32_t)(p >> 2));
    b = pitch_inc((uint32_t)(p >> 2) + 1u);
    x = a + (uint32_t)(((uint64_t)(b - a) * (uint32_t)(p & 3)) >> 2);
    return (uint32_t)(((uint64_t)x << 16) / base);
}

/* one oscillator's sample: wave w at phase *ph (advanced by inc), pw 0..127 << 8 its shape (Q8: smooth.c); *sp, *jp its sync partner and
 * jump (SAW) */
/* a / b with a quotient that fits 32 bits (hard sync's; the compiler's own 64-bit divide lives in the flash) */
static AUDIO_RAM uint32_t mono_div64(uint64_t a, uint32_t b)
{
#ifdef FELUCCA_WEB
    return (uint32_t)(a / b);
#else
    uint32_t hi = (uint32_t)(a >> 32), lo = (uint32_t)a, q = 0, i;
    for (i = 0; i < 32u; i++) {
        uint32_t top = hi >> 31;
        hi = (hi << 1) | (lo >> 31);
        lo <<= 1;
        q <<= 1;
        if (top || hi >= b) {
            hi -= b;
            q |= 1u;
        }
    }
    return q;
#endif
}

static inline __attribute__((always_inline)) int32_t mono_osc(uint32_t w, uint32_t *ph, uint32_t inc, uint32_t pw, int32_t *sp, int32_t *jp)
{
    uint32_t p = *ph, y;
    int32_t o;
    switch (w) {
    case MW_SAW:
        if (!pw) {
            o = osc_saw(p, inc);
            break;
        }
        {                                               /* hard sync: the partner at 1x .. 8x, reset as osc wraps */
            uint32_t si = (uint32_t)(((uint64_t)inc * (65536u + pw * 1792u / 127u)) >> 16), s = (uint32_t)*sp;   /* (pw Q8) */
            if (p > 0xFFFFFFFFu - inc) {               /* the wrap comes next sample: the partner's jump at it */
                uint32_t at = s + mono_div64((uint64_t)si * (0xFFFFFFFFu - p), inc);
                *jp = -(int32_t)(at >> 16);             /* (from its value there down to the reset's -32768) */
            }
            o = ((int32_t)(s >> 16) - 32768) - blep(s, si);
            if (p < inc || p > 0xFFFFFFFFu - inc)      /* the reset's own step, band-limited */
                o += (blep(p, inc) * *jp) >> 16;
            s += si;
            if (p + inc < p)                           /* osc wraps: the partner starts again where it would be */
                s = mono_div64((uint64_t)si * (p + inc), inc);
            *sp = (int32_t)s;
        }
        break;
    case MW_SQR:                                        /* PW: 50 % .. 5 % */
        o = osc_pulse(p, inc, 0x80000000u - pw * (0x73333333u / 127u / 256u));   /* (pw Q8) */
        break;
    case MW_TRI:                                        /* PW: folded back into range, more and more */
        o = osc_tri(p);
        if (pw) {
            o = (o * (int32_t)((32512u + pw * 3u) >> 4)) / 2032;   /* x1 .. x4 (pw Q8) */
            while (o > 32767 || o < -32768)
                o = o > 32767 ? 65534 - o : -65536 - o;
        }
        break;
    default:                                            /* MW_SIN, PW: 16 bits .. 1 (a square) */
        o = osc_sine(p);
        if (pw) {
            uint32_t sh = 1u + pw * 14u / (127u << 8), q = 1u << sh;    /* (a level every q: 2 .. 32768; pw Q8) */
            y = (uint32_t)(o + 32768);
            o = (int32_t)((y >> sh << sh) + (q >> 1)) - 32768;
            o = clamp(o, -32767, 32767);
        }
        break;
    }
    *ph = p + inc;
    return o;
}

#if defined(TEENSY_NOR_ADDR)
static void trec_trig(track_t *t, voice_t *v);    /* (trec.c: INPUT, a note plays the track's take) */
#endif
static void mono_note_on(track_t *t, voice_t *v)
{
    const int16_t *p = t->p;
#if defined(TEENSY_NOR_ADDR)
    if (MONO_IN(t))
        trec_trig(t, v);
#endif
    v->s[2] = !p[P_FE_ATK] ? 1 << 24 : v->env ? v->s[2] : 0;   /* the filter envelope: a retrigger's from where it is */
    v->s[3] = p[P_FE_ATK] ? 1 : 2;
}

static int32_t mono_cs[NTRK];                           /* each track's CUT as it glides to the knob (+1; 0: not yet) */

static AUDIO_RAM void mono_run(track_t *t, voice_t *v, int32_t *out, uint32_t n, const vmod_t *m, uint32_t ratio)
{
    const int16_t *p = t->p;
    uint32_t i, wv = (uint32_t)p[P_E0] % MONO_NW, w1 = MONO_W[wv][0], w2 = MONO_W[wv][1];
    uint32_t sub = (uint32_t)p[P_E4] % 3u, pw = (uint32_t)clamp(smq(t, SK_E6), 0, 127 << 8);   /* (smooth.c: Q8) */
    int32_t mixq = clamp(smq(t, SK_E1), 0, 127 << 8) * 64 / 127, sublv = clamp(smq(t, SK_E5), 0, 127 << 8);   /* (MIX Q14) */
    int32_t sat = clamp(smq(t, SK_LN0), 0, 127 << 8);
    uint32_t inc1 = m->inc, inc2, incs;
    int32_t drv = 32768 + ((sat * sat) >> 14);          /* SAT: x1 .. ~x2 squared, so the top screams (Q15) */
    inc2 = (uint32_t)(((uint64_t)inc1 * ratio) >> 16);   /* FINE -64..63: +-1 st */
    incs = inc1 >> (sub + 1u);
    /* the filter, with DIGITAL's envelope and DRIVE. CUT and its LFOs glide (~20 ms: a knob's steps are no jumps), the
     * key tracking and the envelope don't (a pluck keeps its snap) */
    int32_t *cs = &mono_cs[(uint32_t)(t - trk) % NTRK], ct = smq(t, SK_CUT) + m->cutoff;   /* (smooth.c: Q8) */
    int32_t cut = smq(t, SK_KTR) * (v->pitch16 - 60 * 16) >> 12;
    {
        int32_t fe = v->s[2], fs = v->s[3], sus = smq(t, SK_FESUS) << 9;
        if (!v->gate)
            fs = 4;
        if (fs == 1) {
            fe += (int32_t)SM_TABL(ENV_LIN, smq(t, SK_FEATK));
            if (fe >= (1 << 24)) { fe = 1 << 24; fs = 2; }
        } else if (fs == 2) {
            fe = p[P_FE_DEC] ? fe + mulq16(sus - fe, SM_TAB(ENV_EXP, smq(t, SK_FEDEC))) : sus;
        } else if (fs == 4) {
            fe -= mulq16(fe, SM_TAB(ENV_EXP, smq(t, SK_FEREL)));
        }
        fe = clamp(fe, 0, 1 << 24);
        v->s[2] = fe;
        v->s[3] = fs;
        cut += ((fe >> 9) * smq(t, SK_FEAMT)) >> 14;
        if (v == &t->v[t->m_vi])
            t->m_fenv = fe >= (1 << 24) ? 32767 : fe >> 9;   /* (FENV, a matrix source: mod.c) */
    }
    int32_t drive = 32768 + smq(t, SK_DRV) * 2, fdrv = smq(t, SK_DRV), ic1 = v->s[0], ic2 = v->s[1];
    int32_t mo = flt_mo8(smq(t, SK_LN1)), res = smq(t, SK_RES);
    mono_flt_t flt;
    for (i = 0; i < n; i++) {
        if (!(i & 7u)) {                                /* every 8 samples: a step of the glide */
            mono_glide(cs, ct);
            mono_fcoef(&flt, *cs - 1 + cut, res);
        }
        int32_t a = mono_osc(w1, &v->ph[0], inc1, pw, &v->s[4], &v->s[6]);
        int32_t b = mono_osc(w2, &v->ph[1], inc2, 0u, &v->s[5], &v->s[7]);   /* (PW: osc 1 only) */
        int32_t y = a + (((b - a) * mixq) >> 14);
        int32_t s = osc_pulse(v->ph[2], incs, 0x80000000u);
        v->ph[2] += incs;
        y = (y * 3 + ((s * sublv) >> 15) * 2) >> 2;     /* (osc 3/4, the sub up to 1/2: room before the clip) */
        if (sat)
            y = softclip((int32_t)(((int64_t)y * drv) >> 15));
        if (fdrv)
            y = softclip(((y >> 2) * (drive >> 2)) >> 11);
        y = mono_filter(&flt, mo, FW(y), &ic1, &ic2);
        out[i] += voice_amp_w(y, m, i);
    }
    v->s[0] = ic1;
    v->s[1] = ic2;
}

static const preset_t MONO_PRESETS[1] = {   /* a plain saw: the filter open, nothing else */
    {"SAW", {0, 0, 0, 0, 0, 0, 0, 0}, {0, 64, 127, 20}, 0, 1, FX(0, 0, 0, 0), PAT(0)},
};

static void mono_render(track_t *t, voice_t *v, int32_t *out, uint32_t n, const vmod_t *m)
{
    if (MONO_IN(t))                                     /* (INPUT: trec.c trec_part plays the line in and the take) */
        return;
    RAMCALL(mono_run)(t, v, out, n, m, mono_ratio(t->p[P_E3] * 64 + ((smq(t, SK_E2) + 128) >> 8)));   /* (FINE smooth.c's) */   /* (the ratio's sums stay in the flash) */
}

static const engine_t ENG_MONO = {
    .name = "MONO",
    .page_title = {"OSC 1", "OSC 2"},
    .edit = {{"WAVE", F_ENUM, 0, MONO_NW - 1, 0, N_MWAVE, 0}, {"MIX", F_INT, 0, 127, 0, 0, 0},
             {"FINE", F_BIPCT, -64, 63, 0, 0, 0}, {"SEMI", F_INT, 0, 24, 0, 0, 0},
             {"SUB", F_ENUM, 0, 2, 0, N_MSUB, 0}, {"SUBVL", F_INT, 0, 127, 0, 0, 0}, {"PW", F_INT, 0, 127, 0, 0, 0},
             {"MACH", F_ENUM, 0, MONO_NM - 1, 0, N_MMACH, 0}},
#if defined(TEENSY_NOR_ADDR)
    .desc = mono_desc,
#endif
    .presets = MONO_PRESETS,
    .npresets = 1,
    .note_on = mono_note_on,
    .render = mono_render,
    .knob = {P_E0, P_E1, P_E2, P_E3},
    .poly = 1,
    .keep = 0xF3u,                                      /* (a retrigger: the filter and the sync partners go on) */
};

#if defined(TEENSY_NOR_ADDR)
/* INPUT, in two halves round the take (Blair: "the audio looped straight after the input page, the effects chain
 * afterwards"): trec.c trec_part records between them and adds the take's playback there, so the loop goes through the
 * filter too and a take is the clean input.
 *   mono_in_pre:  the line in, GAIN, SAT (the INPUT page) -> pre (16-bit scale)
 *   mono_in_post: pre (+ the take playing), DRIVE, the filter (CUT RES TYPE; no envelope or LFO: no voice) -> b, at a
 *                 voice's level */
static int32_t thru_ic[NTRK][2], thru_cs[NTRK];
static AUDIO_RAM void mono_in_pre(track_t *t, int32_t *pre, uint32_t n)
{
    uint32_t i;
    int32_t ig = in_gq12(t), sat = clamp(smq(t, SK_LN0), 0, 127 << 8), drv = 32768 + ((sat * sat) >> 14);
    for (i = 0; i < n; i++) {
        int32_t y = in_ok ? clamp((in_buf[i & 31u] * ig) >> 12, -32768, 32767) : 0;
        if (sat)
            y = softclip((int32_t)(((int64_t)y * drv) >> 15));
        pre[i] = y;
    }
}
static AUDIO_RAM void mono_in_post(track_t *t, const int32_t *pre, int32_t *b, uint32_t n)
{
    uint32_t k = (uint32_t)(t - trk) % NTRK, i;
    int32_t drive = 32768 + smq(t, SK_DRV) * 2, fdrv = smq(t, SK_DRV);
    int32_t mo = flt_mo8(smq(t, SK_LN1)), res = smq(t, SK_RES), ct = smq(t, SK_CUT);
    int32_t ic1 = thru_ic[k][0], ic2 = thru_ic[k][1];
    mono_flt_t flt;
    for (i = 0; i < n; i++) {
        int32_t y = clamp(pre[i], -32768, 32767);   /* (the input and the take together: held to 16 bits) */
        if (!(i & 7u)) {
            mono_glide(&thru_cs[k], ct);
            mono_fcoef(&flt, thru_cs[k] - 1, res);
        }
        if (fdrv)
            y = softclip(((y >> 2) * (drive >> 2)) >> 11);
        y = mono_filter(&flt, mo, FW(y), &ic1, &ic2);
        b[i] += (int32_t)(((int64_t)y * VOICE_FS) >> (15 + FLT_SH - MIX_SH));   /* (voice_amp_w at full amp) */
    }
    thru_ic[k][0] = ic1;
    thru_ic[k][1] = ic2;
}
#endif

/* SPDX-License-Identifier: MIT
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments */
/* Shared DSP building blocks for the Felucca engines (all fixed point).
 * Voice output convention: add sample * amp to out[], where a full-scale
 * oscillator at amp = 1.0 (Q15 32767) contributes VOICE_FS. */
#define VOICE_FS 24000           /* per-voice level: one voice peaks near -6 dBFS before the master */
/* the mix bus (the voices' out[], the parts' sums, the dry mix, master_out) carries MIX_SH bits below Q15: a voice
 * turned down by its envelope keeps its detail (at Q15 an 808 kick's tail at 0.6 s had grit 28 dB under it). The
 * sends, the delay, the reverb and the comb's lines stay Q15 (16-bit memories) */
#define MIX_SH 8

static inline __attribute__((always_inline)) int32_t mulq15(int32_t a, int32_t b) { return (a * b) >> 15; }
/* a * k / 65536 without 64-bit: a up to 2^31, k Q16 */
static inline __attribute__((always_inline)) int32_t mulq16(int32_t a, uint32_t k)
{
    return (int32_t)(((a >> 16) * (int32_t)k) + (int32_t)(((uint32_t)(a & 0xFFFF) * k) >> 16));
}
static inline __attribute__((always_inline)) int32_t clamp(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : v > hi ? hi : v; }
/* a * k / 4096, rounded, keeping a's low bits (k up to 2^19). The mix and master use it: dropping 2 bits first, to make
 * room, was a faint bitcrusher that followed every sound (Blair heard it, an 808 kick's fade most of all) */
static inline __attribute__((always_inline)) int32_t mulq12r(int32_t a, int32_t k) { return (a >> 12) * k + (((a & 0xFFF) * k + 2048) >> 12); }

/* sine, linearly interpolated between the 1024 table points (plain lookup: THD -55 dB) */
static inline __attribute__((always_inline)) int32_t sine_i(uint32_t ph)
{
    uint32_t i = ph >> 22;
    int32_t a = SINE[i], b = SINE[(i + 1u) & 1023u];
    return a + (((b - a) * (int32_t)((ph >> 7) & 0x7FFFu)) >> 15);
}
static inline __attribute__((always_inline)) int32_t osc_sine(uint32_t ph) { return sine_i(ph); }

/* polyBLEP residual (Q15) around a wrap of a phase accumulator */
static inline __attribute__((always_inline)) int32_t blep(uint32_t ph, uint32_t inc)
{
    uint32_t d = inc >> 15;
    int32_t x;
    if (!d)
        return 0;
    if (ph < inc) {
        x = (int32_t)(ph / d);                       /* 0..32767 */
        return x + x - ((x * x) >> 15) - 32768;
    }
    if (ph > 0xFFFFFFFFu - inc) {
        x = -(int32_t)((0xFFFFFFFFu - ph) / d);       /* -32767..0 */
        return ((x * x) >> 15) + x + x + 32768;
    }
    return 0;
}

/* the same over 4 samples (the integral of a cubic B-spline): false tones ~14 dB lower than blep at middle C, a touch
 * less of the very top. Knows the wrap ahead from the phase, so no delay */
static inline __attribute__((always_inline)) int32_t blep4(uint32_t ph, uint32_t inc)
{
    int32_t sp = (int32_t)ph, x, x2, r;
    uint32_t a = sp < 0 ? 0u - (uint32_t)sp : (uint32_t)sp, d = inc >> 15, u;
    if (!d || (a >> 1) >= inc)
        return 0;
    u = a / d;                                       /* samples from the wrap, Q15: 0 .. 2 */
    if (u >= 32768u) {
        x = u >= 65536u ? 0 : 65536 - (int32_t)u;
        x2 = (x * x) >> 15;
        r = (((x2 * x2) >> 15) * 2731) >> 15;        /* x^4 / 24, Q16 */
    } else {
        x = 32768 - (int32_t)u;
        x2 = (x * x) >> 15;                          /* 1/24 + x/6 + x^2/4 + x^3/6 - x^4/8, Q16 */
        r = 2731 + ((x * 10923) >> 15) + (x2 >> 1) + ((((x2 * x) >> 15) * 10923) >> 15) - (((x2 * x2) >> 15) >> 2);
    }
    return sp < 0 ? r : -r;
}

static inline __attribute__((always_inline)) int32_t osc_saw(uint32_t ph, uint32_t inc)
{
    return ((int32_t)(ph >> 16) - 32768) - blep4(ph, inc);
}

static inline __attribute__((always_inline)) int32_t osc_pulse(uint32_t ph, uint32_t inc, uint32_t pw)
{
    return (osc_saw(ph, inc) - osc_saw(ph + pw, inc)) >> 1;
}

static inline __attribute__((always_inline)) int32_t osc_tri(uint32_t ph)
{
    int32_t s = (int32_t)(ph >> 15);                 /* 0..131071 */
    return s < 65536 ? s - 32768 : 98303 - s;
}

/* tanh(x / 32768) * 32767, linearly interpolated (a plain lookup gives
 * 256-unit steps, audible on quiet signals) */
static inline __attribute__((always_inline)) int32_t softclip(int32_t x)
{
    int32_t a = x < 0 ? -x : x, i = a >> 8, y;
    if (i >= 256)
        y = TANH_Q15[256];                           /* continuous with the table */
    else
        y = TANH_Q15[i] + (((TANH_Q15[i + 1] - TANH_Q15[i]) * (a & 255)) >> 8);
    return x < 0 ? -y : y;
}

/* the same at Q15 << FLT_SH, in and out (the FILTER page's filter): between the table's points a straight line, finely */
#define FLT_SH 8
#define FW(x) ((x) * (1 << FLT_SH))                   /* Q15 -> the filter's scale */
static inline __attribute__((always_inline)) int32_t softclip_w(int32_t x)
{
    int32_t a = x < 0 ? -x : x, i = a >> (8 + FLT_SH), y;
    if (i >= 256)
        y = TANH_Q15[256] << FLT_SH;
    else
        y = (TANH_Q15[i] << FLT_SH) + (((TANH_Q15[i + 1] - TANH_Q15[i]) * (a & ((256 << FLT_SH) - 1))) >> 8);
    return x < 0 ? -y : y;
}

/* xorshift32 */
static inline __attribute__((always_inline)) uint32_t noise32(int32_t *st)
{
    uint32_t s = (uint32_t)*st;
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    *st = (int32_t)s;
    return s;
}

/* Trapezoidal SVF (A. Simper), unconditionally stable. Coefficients per
 * block in Q13; signals stay within +-150000 so products fit in 32 bits. */
typedef struct { int32_t a1, a2, a3; } tsvf_t;
static inline void tsvf_coef_k(tsvf_t *c, int32_t cut, int32_t k)   /* cut: 0..127 << 8, k: damping Q12 */
{
    int32_t i, g, den;
    cut = clamp(cut, 0, 127 << 8);
    i = cut >> 8;
    g = SVF_G[i];
    if (i < 127)                                       /* between table points: sweeps without 128 steps */
        g += ((SVF_G[i + 1] - g) * (cut & 255)) >> 8;
    den = 4096 + ((g * (g + k)) >> 12);
    c->a1 = (int32_t)((4096u << 13) / (uint32_t)den);
    c->a2 = (c->a1 * g) >> 12;
    c->a3 = (c->a2 * g) >> 12;
}
static inline void tsvf_coef(tsvf_t *c, int32_t cut, int32_t reso)   /* reso 0..127: damping 2.0 .. ~0.15 */
{
    tsvf_coef_k(c, cut, 8192 - reso * 7600 / 127);
}

static inline __attribute__((always_inline)) int32_t tsvf_lp(const tsvf_t *c, int32_t in, int32_t *ic1, int32_t *ic2)
{
    int32_t v3 = in - *ic2;
    int32_t v1 = (c->a1 * *ic1 + c->a2 * v3) >> 13;
    int32_t v2 = *ic2 + ((c->a2 * *ic1 + c->a3 * v3) >> 13);
    *ic1 = clamp(2 * v1 - *ic1, -150000, 150000);
    *ic2 = clamp(2 * v2 - *ic2, -150000, 150000);
    return v2;
}

/* amplitude ramp over the block. Blocks are always CTL long, so x / CTL is a
 * shift (rounded towards zero, as a division); the compiler at -Os keeps
 * even constant divisions as a hardware divide, once per sample and voice */
#define CTL_LOG2 5
#if (1 << CTL_LOG2) != CTL
#error "CTL_LOG2 does not match CTL"
#endif
static inline __attribute__((always_inline)) int32_t amp_at(const vmod_t *m, uint32_t i)
{
    int32_t x = (m->amp1 - m->amp0) * (int32_t)i;
    return m->amp0 + ((x + ((x >> 31) & (CTL - 1))) >> CTL_LOG2);
}

/* a voice's sample s (Q15) at the amplitude ramp's sample i, scaled to the voice level (VOICE_FS), on the bus (Q15 << MIX_SH);
 * engines whose signal peaks near half scale add it << 1. (s * the level fits as s * amp did) */
static inline __attribute__((always_inline)) int32_t voice_amp(int32_t s, const vmod_t *m, uint32_t i)
{
    return (s * ((amp_at(m, i) * VOICE_FS) >> 15)) >> (15 - MIX_SH);
}

/* voice_amp for a sample at Q15 << FLT_SH (out of the FILTER page's filter): every bit on to the bus */
static inline __attribute__((always_inline)) int32_t voice_amp_w(int32_t s, const vmod_t *m, uint32_t i)
{
    return (int32_t)(((int64_t)s * ((amp_at(m, i) * VOICE_FS) >> 15)) >> (15 + FLT_SH - MIX_SH));
}

/* linear up to k, then a soft knee: only the peaks above k saturate (|y| - k must stay < 2^30) */
static inline __attribute__((always_inline)) int32_t soft_knee(int32_t y, int32_t k)
{
    int32_t a = y < 0 ? -y : y;
    if (a <= k)
        return y;
    a = k + (softclip((a - k) * 2) >> 1);
    return y < 0 ? -a : a;
}

/* phase increment of pitch16 (1/16 semitone) plus ct cents: whole 1/16 semitones from the table,
 * the rest (and fine, in 1/4096) as a factor (1 ct = 2.367 / 4096) */
static inline __attribute__((always_inline)) uint32_t cents_inc(int32_t pitch16, int32_t ct, int32_t fine)
{
    int32_t d16 = ct * 16 / 100, rem = ct * 16 - d16 * 100;          /* rem: 1/1600 semitone */
    uint32_t inc = pitch_inc(clamp(pitch16 + d16, 0, 2047));
    return inc + (uint32_t)((int32_t)(inc >> 12) * (rem * 2367 / 16000 + fine));
}

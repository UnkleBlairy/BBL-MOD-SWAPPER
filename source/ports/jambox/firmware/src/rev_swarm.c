/* SPDX-License-Identifier: GPL-3.0-only */
/* The reverb (Blair's design; the browser lab docs/swarmlab.html is its reference, the same maths in fixed point): two lines, left
 * and right, 4 evenly spaced taps each, half a spacing apart, each line's end coming back into itself. The tank runs on its own
 * clock and counts everything in ticks, so slowing the clock lowers the pitch of what's stored, as a slowed digital clock does.
 * Knobs (these ids keep old kits sounding like a reverb):
 *   CLOCK G_RSIZE  0..90 slows the clock 2.5 octaves; 90..127 shrinks the spacing from 63 ms to 1 ms: a pitched comb
 *   FDBK  G_RDAMP  how long it rings, per second not per trip (a comb rings like an echo); 127 holds
 *   FILT  G_RTYPE  -64..63, 0 open: a steep low pass one way, high pass the other, where the input and the feedback enter
 *                  the lines: the first repeat has it, every trip takes more
 *   SMEAR G_DMIX   0 clean echoes, up to a wash: allpasses before the lines, in the loop and on the way out; the loops mix,
 *                  their lengths differ and the taps leave the grid
 * Samples: Q19 (1.0 = 1 << 19: the memory's 16 bits + 4 below them). The memory rounds toward 0 near silence, so tails die to
 * exactly 0, and to the nearest at a hold. A soft limit on the way in to the memory: nothing can blow up.
 * Tables: tools/gen_swarm_tables.py */
#include "swarm_tables.h"

#define SW_ONE (1 << 19)
#define SW_LINE 3073                                     /* ceil(4.25 x the spacing + 90) + 8 ticks */
enum { SW_PRE_N = 5, SW_LOOP_N = 6, SW_OUT_N = 2, SW_NAP = SW_PRE_N + 2 * SW_LOOP_N + 2 * SW_OUT_N };
/* the allpasses: pre, the left loop, the right loop, the left out, the right out */
static const uint16_t SW_APL[SW_NAP] = {17, 29, 43, 67, 101, 61, 113, 211, 331, 449, 577, 71, 127, 197, 347, 431, 557, 89, 131, 97, 127};
#define SW_APSUM (17 + 29 + 43 + 67 + 101 + 61 + 113 + 211 + 331 + 449 + 577 + 71 + 127 + 197 + 347 + 431 + 557 + 89 + 131 + 97 + 127)
#define SW_DPRE (257 << 8)
static const int8_t SW_JIT[2][4] = {{37, -53, 71, -29}, {-41, 61, -31, 47}};   /* how far SMEAR's top moves each tap, ticks */
#define SW_SKEW 1966                                     /* the right loop this much shorter at SMEAR's top, Q15 */
#ifndef SW_IN_SHIFT
#define SW_IN_SHIFT 3                                    /* the send bus -> Q19 */
#endif
#ifndef SW_OUT_G
#define SW_OUT_G 12288                                   /* Q19 -> the wet bus, Q15 (the lab's 3, at this input level) */
#endif
#define SW_SQ2 46341                                     /* the filters' damping (Butterworth), Q15 */

static int16_t sw_mem[2 * SW_LINE + SW_APSUM] __attribute__((section(".pool")));
static struct {
    int32_t w;
    struct { int16_t *p, *start, *end; } ap[SW_NAP];
    int16_t *line[2];
    int32_t jit[2][4];                                   /* (SW_JIT, here: the tick reads no flash) */
    int32_t ph, inc;                                     /* the tick's phase and step per sample, Q24 */
    int32_t oct;                                         /* the clock, Q16 octaves: -2.5 .. 0 */
    int32_t pq, smq, wq, pt, smt, wt;                    /* with weight, per tick: the spacing Q16 ticks, smear Q27, the allpasses' way in Q27 */
    int32_t ts;                                          /* FILT, Q15, -1 .. 1 */
    int32_t g1, g2, g3, a[3], fbk, ins, damp, hpk, midc, co, si, fmix, fhp, near, gq;
    int32_t ma[3], fa[3], hf[6];
    int32_t lp[2], hp[2], y[2], yp[2], ms[2][2], fs[2][2], fi[4], fo[2][4], dr[2];
    uint32_t wob1, wob2;
    int32_t tick8, ready;
} sw __attribute__((section(".pool")));

/* The tick and the sample loop run from RAM (AUDIO_RAM): a loop this size, from flash, costs its own cache misses and pushes the voices'
 * code out of the cache as well. RAM code may make no calls (tools/build.py), so all of it is inlined */
#define SW_INL static inline __attribute__((always_inline))
SW_INL int32_t swq15(int32_t a, int32_t b) { return (int32_t)(((int64_t)a * b) >> 15); }
SW_INL int32_t swq24(int32_t a, int32_t b) { return (int32_t)(((int64_t)a * b) >> 24); }

/* a state-variable filter's step: x in Q19, states in Q25 (a low cut's small coefficients need the bits, or the state sticks on a
 * small value that goes on circling for ever). Sets the band and the low pass, Q19 */
SW_INL void sw_svf_run(int32_t *st, const int32_t *a, int32_t x, int32_t *band, int32_t *low)
{
    int32_t s0 = st[0], s1 = st[1], v3 = (x << 6) - s1;
    int32_t p = swq24(s0, a[1]), v1 = swq24(s0, a[0]) + swq24(v3, a[1]), v2 = s1 + p + swq24(v3, a[2]);
    st[0] = 2 * v1 - s0;
    st[1] = 2 * v2 - s1;
    *band = (v1 + 32) >> 6;
    *low = (v2 + 32) >> 6;
}
SW_INL int32_t sw_soft(int32_t x)                        /* straight up to .6, then a soft limit below 1 */
{
    int32_t a, u, i, y;
    if (x <= 314573 && x >= -314573)
        return x;
    a = x < 0 ? -x : x;
    u = (int32_t)(((int64_t)(a - 314573) * 5) >> 2);     /* (a - .6) / .4, Q19; 1/16 steps */
    i = u >> 15;
    if (i >= 64)
        y = SW_ONE - 512;
    else {
        int32_t t0 = SW_TANH[i], t1 = SW_TANH[i + 1], tv = t0 + (((t1 - t0) * (u & 32767)) >> 15);
        y = 314573 + ((tv * 819) >> 7);                  /* .4 x tanh, Q15 -> Q19 */
    }
    return x < 0 ? -y : y;
}

SW_INL int32_t sw_ap(uint32_t k, int32_t x, int32_t g, int32_t near)   /* the memory: to the nearest, or tails get eaten (many rounds a
                                                                         * trip); toward 0 when very quiet, or a faint tone circles for ever */
{
    int16_t *q = sw.ap[k].p;
    int32_t d = (int32_t)*q << 4, v = x - swq15(d, g), y = d + swq15(v, g), n = sw_soft(v);
    *q = (int16_t)(!near && n < (48 << 4) && n > -(48 << 4) ? n / 16 : (n + 8) >> 4);
    if (++q >= sw.ap[k].end)
        q = sw.ap[k].start;
    sw.ap[k].p = q;
    return y;
}
SW_INL int32_t sw_read(const int16_t *b, int32_t back, int32_t w)   /* `back` (Q8 ticks) before the next write, between samples too */
{
    int32_t di, fr, i0, i1, a;
    back = back < 256 ? 256 : back;
    di = back >> 8;
    fr = back & 255;
    i0 = w - di;
    if (i0 < 0)
        i0 += SW_LINE;
    i1 = i0 ? i0 - 1 : SW_LINE - 1;
    a = b[i0];
    return (a << 4) + (((b[i1] - a) * fr) >> 4);
}
SW_INL int32_t sw_sine(uint32_t ph)
{
    uint32_t i = ph >> 22;
    int32_t a = SINE[i], b = SINE[(i + 1u) & 1023u];
    return a + (((b - a) * (int32_t)((ph >> 7) & 0x7FFFu)) >> 15);
}

SW_INL void sw_tick(int32_t x)
{
    int32_t c, k, P, off, sm, w, m, dry, out[2], loop[2];
    const int32_t near = sw.near, wr = sw.w;
    sw.pq += (sw.pt - sw.pq) >> 12;                      /* with weight: the taps slide closer, and bend as they go */
    sw.smq += (sw.smt - sw.smq) >> 12;
    sw.wq += (sw.wt - sw.wq) >> 12;
    P = sw.pq >> 8;
    sm = sw.smq >> 12;
    w = sw.wq >> 12;
    off = P >> 2;
    dry = swq15(x >> 1, sw.ins);                         /* room above, in the memory */
    m = dry;
    for (k = 0; k < SW_PRE_N; k++)
        m = sw_ap((uint32_t)k, m, sw.g1, near);
    m = dry + swq15(m - dry, w);
    if (!(sw.tick8++ & 7)) {                             /* the loops wander, more at the top: less metallic */
        int32_t wand = swq15(sm, 32768 + 7 * sm) >> 7;   /* Q8 ticks */
        sw.wob1 += 1153000u;                             /* ~.37 Hz at the top clock */
        sw.wob2 += 1511000u;
        sw.dr[0] = (sw_sine(sw.wob1) * wand) >> 15;
        sw.dr[1] = (sw_sine(sw.wob2 + 0x40000000u) * wand) >> 15;
    }
    for (c = 0; c < 2; c++) {
        const int16_t *line = sw.line[c];
        const int32_t *jit = sw.jit[c];
        int32_t t0 = P - swq15(SW_DPRE + (c ? 224 << 8 : 220 << 8), w) + (c ? off : -off);
        int32_t sum = sw_read(line, t0 + ((jit[0] * sm) >> 7), wr);
        int32_t o1, len, y, yd, z, v1, v2;
        for (k = 1; k < 4; k++)
            sum += swq15(sw_read(line, t0 + k * P + ((jit[k] * sm) >> 7), wr), sw.a[k - 1]);
        o1 = sw_ap((uint32_t)(SW_PRE_N + 2 * SW_LOOP_N + 2 * c), sum, sw.g3, near);
        o1 = sw_ap((uint32_t)(SW_PRE_N + 2 * SW_LOOP_N + 2 * c + 1), o1, sw.g3, near);
        out[c] = sum + swq15(o1 - sum, w);
        /* what comes back: the end of the line, through the smear; the allpasses' own delay is taken off, so the grid stays exact */
        len = 4 * P;
        if (c)
            len -= swq15(len, swq15(SW_SKEW, sm));
        y = sw_read(line, len - (c ? 1730 << 8 : 1742 << 8) - 512 + sw.dr[c], wr);
        for (k = 0; k < SW_LOOP_N; k++)
            y = sw_ap((uint32_t)(SW_PRE_N + c * SW_LOOP_N + k), y, sw.g2, near);
        yd = sw_read(line, len - 512 + sw.dr[c], wr);    /* (the short way round) */
        y = yd + swq15(y - yd, w);
        sw_svf_run(sw.ms[c], sw.ma, y, &v1, &v2);        /* the mid cut: a shallow bell */
        z = y + swq15(swq15(v1, SW_SQ2), sw.midc);
        sw.lp[c] += swq15(z - sw.lp[c], sw.damp);        /* the high cut */
        sw.hp[c] += (int32_t)(((int64_t)(sw.lp[c] - (sw.hp[c] >> 8)) * sw.hpk) >> 7);   /* the low cut (Q27: slow, it needs the bits, or it sticks and circles) */
        loop[c] = sw.lp[c] - (sw.hp[c] >> 8);
    }
    {   /* SMEAR mixes the two loops: a rotation keeps the energy; FILT, then into the lines */
        int32_t b0 = swq15(swq15(loop[0], sw.co) + swq15(loop[1], sw.si), sw.fbk);
        int32_t b1 = swq15(swq15(loop[1], sw.co) - swq15(loop[0], sw.si), sw.fbk);
        for (c = 0; c < 2; c++) {
            int32_t q = m + (c ? b1 : b0), n;
            if (sw.fmix) {
                int32_t v1, v2, f;
                sw_svf_run(sw.fs[c], sw.fa, q, &v1, &v2);
                f = sw.fhp ? q - swq15(v1, SW_SQ2) - v2 : v2;
                q += swq15(f - q, sw.fmix);
            }
            n = sw_soft(q);
            sw.line[c][wr] = (int16_t)(near ? (n + 8) >> 4 : n / 16);   /* toward 0: tails die; at a hold, the nearest: it stays */
        }
    }
    sw.w = wr + 1 >= SW_LINE ? 0 : wr + 1;
    for (c = 0; c < 2; c++) {
        sw.yp[c] = sw.y[c];
        sw.y[c] = out[c];
    }
}

/* the send bus in; adds the mono wet to out, its stereo difference to rev_side (fx.c) */
static AUDIO_RAM void sw_run(const int32_t *rev_in, int32_t *out, uint32_t n)
{
    uint32_t i;
    for (i = 0; i < n; i++) {
        int32_t x = rev_in[i], j, c, pv, o[2], v1;
        const int32_t *h = sw.hf;
        int32_t *s = sw.fi;
        x = (x < -262143 ? -262143 : x > 262143 ? 262143 : x) << SW_IN_SHIFT;
        for (j = 0; j < 2; j++, h += 3, s += 2)          /* 4 poles, before the clock's rate */
            sw_svf_run(s, h, x, &v1, &x);
        sw.ph += sw.inc;
        if (sw.ph >= 1 << 24) {
            sw.ph -= 1 << 24;
            sw_tick(x);
        }
        pv = sw.ph >> 16;
        for (c = 0; c < 2; c++) {                        /* back to 44.1 kHz, between the ticks; 4 poles again */
            int32_t a = sw.yp[c] + (((sw.y[c] - sw.yp[c]) * pv) >> 8);
            h = sw.hf;
            s = sw.fo[c];
            for (j = 0; j < 2; j++, h += 3, s += 2)
                sw_svf_run(s, h, a, &v1, &a);
            o[c] = (int32_t)((((int64_t)a * (sw.gq >> 5)) + 16384) >> 15);
        }
        out[i] += (o[0] + o[1]) >> 1;
        rev_side[i] = (o[0] - o[1]) >> 1;
    }
}

static int32_t sw_log2(uint32_t v)                       /* Q10 */
{
    int32_t e = 31, i;
    uint32_t f;
    if (!v)
        return 0;
    while (!(v & 0x80000000u)) {
        v <<= 1;
        e--;
    }
    i = (int32_t)((v >> 26) & 31u);
    f = (v >> 10) & 0xFFFFu;
    return e * 1024 + SW_LOG[i] + (int32_t)(((uint32_t)(SW_LOG[i + 1] - SW_LOG[i]) * f) >> 16);
}
static int32_t sw_exp(int32_t y)                         /* exp(-y), y Q16 >= 0 -> Q15 */
{
    int32_t r = 32768, i;
    while (y >= 4 << 16) {
        r = (r * 600) >> 15;                             /* (exp(-4)) */
        y -= 4 << 16;
    }
    i = y >> 11;
    return swq15(r, SW_EXP[i] + (((SW_EXP[i + 1] - SW_EXP[i]) * (y & 2047)) >> 11));
}
static uint32_t sw_sqrt(uint32_t v)
{
    uint32_t r = 0, b = 1u << 30;
    while (b > v)
        b >>= 2;
    while (b) {
        if (v >= r + b) {
            v -= r + b;
            r = (r >> 1) + b;
        } else
            r >>= 1;
        b >>= 2;
    }
    return r;
}
static inline int32_t sw_ix(int32_t x)                   /* log2(fc / clock), Q10 -> the tables' place, Q6 */
{
    x += 9 << 10;
    return x < 0 ? 0 : x > 126 << 6 ? 126 << 6 : x;
}
static void sw_svf(int32_t *d, int32_t x)                /* a Butterworth's coefficients */
{
    int32_t ix = sw_ix(x), i = ix >> 6, f = ix & 63, j;
    if (i >= 126) {
        i = 125;
        f = 63;
    }
    for (j = 0; j < 3; j++) {
        int32_t t0 = SW_SVF[3 * i + j], t1 = SW_SVF[3 * i + 3 + j];
        d[j] = t0 + (((t1 - t0) >> 4) * f >> 2);
    }
}
static int32_t sw_opk(int32_t x)                         /* a one-pole's k */
{
    int32_t ix = sw_ix(x), i = ix >> 6, f = ix & 63;
    if (i >= 126)
        return SW_OP[126];
    return SW_OP[i] + (((SW_OP[i + 1] - SW_OP[i]) * f) >> 6);
}

/* each block: the knobs (they may be locks or LFOs' just now) -> what the tick uses */
static void sw_block(void)
{
    /* (smooth.c: the knobs at Q8, gliding as they are turned; the tables read in between) */
    int32_t clock = clamp(smg(SKG_RCLK), 0, 127 << 8), fd8 = clamp(smg(SKG_RFBK), 0, 127 << 8), fd = fd8 >> 8;
    int32_t filt = clamp(smg(SKG_RFLT), -64 * 256, 63 * 256), s = SM_TAB(SW_SMEAR, smg(SKG_SMEAR));
    int32_t octT = clock < 90 * 256 ? -640 * (90 * 256 - clock) / 90 : 0, t = filt >= 0 ? (filt * 520) >> 8 : filt * 2, d;
    int32_t r, up, dn, lc, f, f3, y, x, i, fr, smn, gT, lnf = SM_TAB(SW_LNF, fd8);
    i = clock > 90 * 256 ? clock - 90 * 256 : 0;
    sw.pt = SW_PT[i >> 8] + ((i >> 8) < 37 ? (int32_t)(((int64_t)(SW_PT[(i >> 8) + 1] - SW_PT[i >> 8]) * (i & 255)) >> 8) : 0);
    sw.smt = s << 12;
    sw.wt = (s >= 4096 ? 32768 : s * 8) << 12;
    if (!sw.ready) {
        int16_t *p = sw_mem;
        for (i = 0; i < 2; i++, p += SW_LINE)
            sw.line[i] = p;
        for (i = 0; i < SW_NAP; p += SW_APL[i], i++) {
            sw.ap[i].p = sw.ap[i].start = p;
            sw.ap[i].end = p + SW_APL[i];
        }
        for (i = 0; i < 2; i++)
            for (x = 0; x < 4; x++)
                sw.jit[i][x] = SW_JIT[i][x];
        sw.oct = octT;
        sw.pq = sw.pt;
        sw.smq = sw.smt;
        sw.wq = sw.wt;
        sw.ts = t;
        sw.gq = SW_OUT_G << 5;
        sw.ready = 1;
    }
    d = octT - sw.oct;                                   /* with weight: moving it warps the pitch of what's stored */
    sw.oct = d > -512 && d < 512 ? octT : sw.oct + (d >> 9);
    i = (sw.oct + 163840) / 2560;
    fr = (sw.oct + 163840) - i * 2560;
    if (i >= 64) {
        i = 63;
        fr = 2560;
    }
    sw.inc = SW_INC[i] + (SW_INC[i + 1] - SW_INC[i]) / 2560 * fr;
    for (x = 0; x < 6; x++)
        sw.hf[x] = SW_HF[6 * (i + (fr >= 1280)) + x];
    lc = SW_LTOP + (sw.oct >> 6);                        /* log2 of the clock, Q10 */
    d = t - sw.ts;                                       /* FILT glides */
    sw.ts = d > -64 && d < 64 ? t : sw.ts + (d >> 6);
    up = sw.ts > 0 ? sw.ts : 0;
    dn = sw.ts < 0 ? -sw.ts : 0;
    if (up) {
        int32_t hi = lc + SW_L40 < SW_LHPT ? lc + SW_L40 : SW_LHPT;
        x = SW_LHP0 + swq15(hi - SW_LHP0, up);
    } else {
        int32_t top = lc + SW_L42;
        x = top + swq15(SW_LLP0 - top, dn);
    }
    sw_svf(sw.fa, x - lc);
    sw.fhp = up > 0;
    sw.fmix = up + dn >= 4096 ? 32768 : (up + dn) * 8;   /* it comes in over the first few steps from the middle */
    /* a short loop makes more trips a second: less per trip, the same per second */
    r = (sw.pq >> 1) / 700;                              /* the spacing's part, Q15 */
    r = r > 32768 ? 32768 : r < 1 ? 1 : r;
    sw.fbk = fd >= 127 ? 32768 : fd8 == 0 ? 0 : sw_exp(swq15(lnf, r));
    if (sw.fbk > 32768)
        sw.fbk = 32768;
    sw.near = sw.fbk >= 32768;
    y = lnf < 68802 ? lnf : 68802;         /* the taps fall in step with the repeats, so the fade has no steps */
    for (x = 0; x < 3; x++)
        sw.a[x] = sw_exp((y * (x + 1)) >> 2);
    if (r > 32735)
        sw.ins = 32768;
    else {                                               /* (a comb's peaks don't blow up) */
        int32_t q = (int32_t)sw_sqrt((uint32_t)(1073741824 - sw.fbk * sw.fbk));
        q = q < 11469 ? 11469 : q;
        sw.ins = 32768 - swq15(32768 - q, 32768 - r);
    }
    /* the loop's tone: more low cut and mid cut as it smears, less near the top of FDBK. The side FILT keeps is opened too */
    f = (fd8 * 258) >> 8;
    f3 = swq15(swq15(f, f), f);
    y = (80 << 8) + swq15(120 << 8, s);
    y = swq15(y, 32768 - (f3 >> 1));
    y = swq15(y, 32768 - swq15(22938, dn));
    y = swq15(y, r);
    y = y < 25 << 8 ? 25 << 8 : y;
    sw.hpk = sw_opk(sw_log2((uint32_t)y) - (8 << 10) - lc);
    if (r < 9830)
        sw.damp = 32768;
    else {
        y = 5000 - swq15(800, s);
        y = swq15(y, 32768 + swq15(13107, f3));
        y = swq15(y, 32768 + 2 * up);
        x = sw_log2((uint32_t)y) - (sw_log2((uint32_t)r) - (15 << 10));
        x = x < lc + SW_L42 ? x : lc + SW_L42;
        sw.damp = sw_opk(x - lc);
    }
    x = lc + SW_L20 < SW_LMID ? lc + SW_L20 : SW_LMID;
    sw_svf(sw.ma, x - lc);
    y = 19661 + swq15(45875, s);                         /* .6 + 1.4 x smear, dB */
    y = swq15(y, 32768 - swq15(27853, f3));
    y = swq15(y, r);
    sw.midc = -swq15(y, 3441);                           /* dB -> how much of the band comes off */
    /* the wet's way out: the high pass all but vanishes, so some back at its end */
    y = swq15(105631, swq15(up, up));
    gT = (int32_t)(((int64_t)SW_OUT_G << 20) / sw_exp(y));   /* Q20 */
    sw.gq += (gT - sw.gq) >> 3;
    smn = sw.smq >> 12;
    gT = swq15(25559, s);                                /* the allpasses */
    sw.g1 += (gT - sw.g1) >> 3;
    gT = swq15(26214, s);
    sw.g2 += (gT - sw.g2) >> 3;
    gT = swq15(24576, s);
    sw.g3 += (gT - sw.g3) >> 3;
    {   /* the loops' rotation: up to 45 degrees */
        uint32_t ph = (uint32_t)smn * 16384u;
        sw.si = osc_sine(ph);
        sw.co = osc_sine(ph + 0x40000000u);
    }
}

/* (fx.c: sw_block as the block is taken, bus_take; the samples a block later, maybe on the second core) */
static __attribute__((noinline)) void rev_swarm(const int32_t *rev_in, int32_t *out, uint32_t n)
{
    RAMCALL(sw_run)(rev_in, out, n);
}
static void rev_swarm_clear(void)
{
    memset(sw_mem, 0, sizeof sw_mem);
    memset(&sw, 0, sizeof sw);
}

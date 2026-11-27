/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments */
/* Effects: per-track DIST and INSERT (1.5), then sends into three
 * shared buses (chorus, tempo delay, reverb). Mono buses, stereo dry mix. */
#define DLY_LEN 65536u           /* 1.49 s: 1/4 fits from 41 BPM, 1/4D from 61, 1/2 from 81 */
static int16_t dly_buf[DLY_LEN] __attribute__((section(".pool")));
static struct {
    uint32_t dly_w;
    int32_t dly_cur;                     /* SIMPLE: the delay time now (Q8 samples), gliding to the knob: the tape / BBD repitch */
    int32_t dly_lp, dly_lp2, dly_hp;     /* the delay's loop: two low-passes, a high-pass */
    uint32_t dly_ph;                     /* .. the tap's slow wobble (a BBD's clock drift) */
} fx;

static struct {
    uint8_t side;                        /* sd holds this block's stereo difference */
    int32_t sd[CTL];                     /* the block's left minus right (mix_block: rev_side_mix) */
} hl __attribute__((section(".pool")));
#define rev_side (hl.sd)


/* master (2026-11-12, measured with tools/snd/bench.py): a -6 dB trim (six hits at full MASTER summed to +3 dBFS),
 * then a look-ahead limiter, then a knee. The trim keeps its bit: master_out gives Q16 (65535 = full scale), and
 * MASTER is applied after it at 24 bits (audio.c), so quiet tails and a low MASTER keep their detail. The old stage (tanh over the whole range + a limiter reacting within a
 * few samples) added distortion 30 dB under a single kick and 12 dB under six hits, and reshaped bass waves.
 * Limiter: the signal is delayed LIM_LA samples (1.45 ms); every LIM_SB samples the gain needed for the loudest
 * of the LIM_LA samples waiting is worked out, and the gain ramps down to it over the next LIM_SB samples, so it
 * is there before the peak comes out: no wave is ever squashed. It comes back up ~90 ms (>> 12). The knee is
 * straight up to LIM_T and bends only above it (a safety, the limiter keeps the signal under it). */
#define LIM_T 49152                     /* -2.5 dBFS of the Q16 out: the limiter's ceiling and where the knee starts */
#define LIM_TB (LIM_T << MIX_SH)        /* .. on the bus (master_out works at Q16 << MIX_SH) */
#define LIM_LA 64
#define LIM_SB 16
static int32_t lim_dl[2 * LIM_LA], lim_pk[LIM_LA / LIM_SB], lim_cur, lim_g = 32768, lim_gt = 32768, lim_step;
static uint32_t lim_i;
static inline int32_t lim_knee(int32_t x)
{
    int32_t a = x < 0 ? -x : x;
    if (a <= LIM_TB)
        return x;
    a = LIM_TB + ((softclip(((a - LIM_TB) >> MIX_SH) * 2) >> 1) << MIX_SH);   /* tanh over the last 2.5 dB, slope 1 at the knee: <= 65535 */
    return x < 0 ? -a : a;
}
/* MENU > USB LEVEL FIXED (for #42: record over USB with the speaker turned down): the mix goes to master_out at the
 * full MASTER level, USB audio takes that (audio.c uac_tap), and only then does MASTER scale what the DAC gets
 * (usb_fixed_dac). MASTER (0, the default): MASTER before master_out, as always (USB follows the knob) */
static volatile uint8_t fx_usb_fixed;
#define OUT_SHIFT 7               /* (audio.c) Q15 -> 24-bit, -6 dBFS ceiling (M0f ran clean at -18 dBFS) */
#define MASTER_FULL 4096                /* main.c: the MASTER knob's top, Q12 */
/* the MASTER pot (ADC 0..1023, smoothed in main.c) to its level, square law: 0 .. 4088 (Q12) */
static inline uint32_t master_of_pot(uint32_t k10) { return (k10 * k10) >> 8; }
static __attribute__((noinline)) void usb_fixed_dac(int32_t *out, uint32_t n)   /* audio ISR, after uac_tap */
{
    uint32_t i;
    int32_t m = (int32_t)song.master_q12;
    for (i = 0; i < 2u * n; i++)
        out[i] = (int32_t)(((int64_t)out[i] * m) >> 12);   /* (the 24-bit word) */
}
static volatile uint8_t fx_lowcut;     /* settings: 1 LOWCUT 12 dB/oct ~110 Hz, 2 BASS+ (the small speaker):
                                        * 12 dB/oct ~220 Hz plus the harmonics of the bass (spk_bass) */
static int32_t lc_l1, lc_l2, lc_r1, lc_r2, dc_l, dc_r, dce_l, dce_r;

/* DC blocker (~2 Hz), always on: a leaky integrator of the input (Q6 state) subtracted from it.
 * The >> 12 step keeps its remainder (error feedback, 0..4095) and adds it to the next one, so no
 * part of the step is lost: the state follows the input exactly, down to 0 after the sound stops.
 * (A rounded step of (x - dc) / 4096 would stop moving at |x - dc| < 2048 and leave an offset of up
 * to +-31 at the output after silence.) */
static inline int32_t dc_block(int32_t x, int32_t *dc, int32_t *err)   /* (the bus's MIX_SH bits are its fraction now) */
{
    int32_t e = x - *dc + *err, d = e >> 12;
    *err = e - (d << 12);
    *dc += d;
    return x - *dc;
}

static int32_t lce[4];
static inline int32_t lowcut1(int32_t x, int32_t *lc, int32_t *err, uint32_t sh)   /* x minus its one-pole low-pass */
{
    int32_t e = x - *lc + *err, d = e >> sh;
    *err = e - (d << sh);
    *lc += d;
    return x - *lc;
}

/* BASS+: what the speaker cannot play, heard through its harmonics. The bass below ~150 Hz is clipped at its
 * own envelope (a level-following trapezoid: odd harmonics), then band-passed ~220 Hz..1 kHz and added.
 * The low-pass has 4 poles (#42: with 2, the trapezoid rebuilt the 300 .. 600 Hz of the mix itself, late,
 * and cancelled up to 6 dB of it; now under 0.5 dB) */
static int32_t sb_lp1, sb_lp2, sb_lp3, sb_lp4, sb_env, sb_h1, sb_h2, sb_hl;
static inline int32_t spk_bass(int32_t m)
{
    int32_t a, t, u;
    sb_lp1 += ((m - sb_lp1) * 692) >> 15;
    sb_lp2 += ((sb_lp1 - sb_lp2) * 692) >> 15;
    sb_lp3 += ((sb_lp2 - sb_lp3) * 692) >> 15;
    sb_lp4 += ((sb_lp3 - sb_lp4) * 692) >> 15;
    a = sb_lp4 < 0 ? -sb_lp4 : sb_lp4;
    if (a > sb_env)
        sb_env += (a - sb_env) >> 2;
    else if (sb_env > 0)
        sb_env -= (sb_env >> 11) + 1;
    t = clamp(sb_lp4 * 8, -sb_env, sb_env);
    sb_h1 += (t - sb_h1) >> 5;
    u = t - sb_h1;
    sb_h2 += (u - sb_h2) >> 5;
    u -= sb_h2;
    sb_hl += (u - sb_hl) >> 3;
    return sb_hl + (sb_hl >> 1);      /* x1.5 (#180: x3 peaked at 1.5 .. 3.5 x a full-scale kick, the limiter */
}                                     /* pulled the mix down up to 16 dB on each hit and the buzz took over) */

static inline __attribute__((always_inline)) void master_out(int32_t *l, int32_t *r)
{

#ifdef BENCH_TAP
    BENCH_TAP(*l, *r);                                  /* tools/emu_shot.c: the mix before the output stage (Q15 << MIX_SH) */
#endif
    *l = dc_block(*l, &dc_l, &dce_l);
    *r = dc_block(*r, &dc_r, &dce_r);
    if (fx_lowcut) {                  /* two one-pole high-passes, error feedback as dc_block (the */
        uint32_t sh = fx_lowcut == 2u ? 5u : 6u;    /* rounded step stopped at |x - lc| < 32: an offset) */
        int32_t b = fx_lowcut == 2u ? spk_bass((*l + *r) >> (1 + MIX_SH)) * (1 << MIX_SH) : 0;
        *l = lowcut1(*l, &lc_l1, &lce[0], sh);
        *l = lowcut1(*l, &lc_l2, &lce[1], sh) + b;
        *r = lowcut1(*r, &lc_r1, &lce[2], sh);
        *r = lowcut1(*r, &lc_r2, &lce[3], sh) + b;
    }
    {
        int32_t x = *l, y = *r, ax = x < 0 ? -x : x, ay = y < 0 ? -y : y, o;
        uint32_t k = lim_i % LIM_LA;
        if (ay > ax)
            ax = ay;
        if (ax > lim_cur)
            lim_cur = ax;
        *l = lim_dl[2u * k];                            /* LIM_LA samples ago */
        *r = lim_dl[2u * k + 1u];
        lim_dl[2u * k] = x;
        lim_dl[2u * k + 1u] = y;
        if (++lim_i % LIM_SB == 0u) {                    /* a sub-block in: the gain the waiting ones need */
            uint32_t j, b = (lim_i / LIM_SB) % (LIM_LA / LIM_SB);
            int32_t m = 0;
            lim_pk[b] = lim_cur;
            lim_cur = 0;
            for (j = 0; j < LIM_LA / LIM_SB; j++)
                if (lim_pk[j] > m)
                    m = lim_pk[j];
            lim_gt = m > LIM_TB ? (int32_t)(((uint32_t)LIM_T << 15) / ((uint32_t)m >> MIX_SH)) : 32768;
            lim_step = lim_gt < lim_g ? (lim_gt - lim_g) / LIM_SB - 1 : 0;   /* down: there in LIM_SB samples */
        }
        if (lim_step) {
            lim_g += lim_step;
            if (lim_g <= lim_gt)
                lim_g = lim_gt, lim_step = 0;
        } else if (lim_g < lim_gt)
            lim_g += ((lim_gt - lim_g) >> 12) + 1;      /* back up, ~90 ms */
        if (lim_g < 32768) {
            o = (int32_t)(((int64_t)*l * lim_g) >> 15);
            *l = o;
            *r = (int32_t)(((int64_t)*r * lim_g) >> 15);
        }
    }
    *l = lim_knee(*l);
    *r = lim_knee(*r);
}

/* length of one division (N_DIV order) in samples at the song tempo */
static const uint8_t DIV_DEN[6] = {1, 2, 4, 8, 3, 6};    /* original IDs stay fixed; slow rates append */
static uint32_t midi_beat_samples;                    /* zero until an external clock has a measured tempo */
static uint32_t beat_samples(void)
{
    uint32_t bpm = song.g[G_BPM] >= 40 ? (uint32_t)song.g[G_BPM] : 120u;   /* (BPM 0 for a moment, #282: no divide by 0) */
    return song.g[G_CLOCK] && midi_beat_samples ? midi_beat_samples : (uint32_t)FS * 60u / bpm;
}
static uint32_t div_samples(uint32_t div)
{
    uint32_t quarter = beat_samples();
    return div < 6u ? quarter / DIV_DEN[div] : div < 10u ? quarter << (div - 5u) : div == 10u ? quarter / 16u : quarter / DIV_DEN[div % 6u];   /* (10: 1/64, SIMPLE's SEQ) */
}

#include "repeat.c"                                  /* PERF's beat repeats (the master) */
#include "mfx.c"                                     /* the groovebox's master DJ filter and comb */
#include "click.c"                                   /* the metronome's click (after the master: audio.c) */

/* the delay's TIME (params.c N_DTIME): N_DIV's ten (div_samples), then 1.5.1's dotted 1/8D 1/16D 1/4D (10, 11, 12: 3/4,
 * 3/8, 3/2 of a quarter); at most DLY_LEN - 1 samples (1.486 s): 1/4D clamps at 60 BPM and below (1/2 below 81) */
static const uint8_t DIV_DOT[3] = {2, 3, 1};           /* 3 quarters >> this */
/* SIMPLE: the delay runs free of the tempo: TIME 0..127 is 20 ms .. 1.45 s, exponential (6.2 octaves). -> samples, Q8 */
static const uint32_t EXP2_Q15[17] = {32768, 34219, 35734, 37316, 38968, 40693, 42495, 44376, 46341, 48393, 50535, 52773, 55109, 57549, 60097, 62757, 65536};
static int32_t delay_q8(void)
{
    uint32_t e = ((uint32_t)clamp(smg(SKG_DTIME), 0, 127 << 8) * 3189u) >> 8, oct = e >> 16, fr = (e >> 12) & 15u, sub = e & 4095u;   /* (Q8: smooth.c) */
    uint32_t m = EXP2_Q15[fr] + (((uint32_t)EXP2_Q15[fr + 1u] - EXP2_Q15[fr]) * sub >> 12);
    uint32_t s = ((880u << oct) * m) >> 15;
    return (int32_t)(s >= DLY_LEN - 2u ? DLY_LEN - 2u : s) << 8;
}
static uint32_t delay_samples(void)
{
    uint32_t d = (uint32_t)song.g[G_DTIME];
    uint32_t s = d - 10u < 3u ? (beat_samples() * 3u) >> DIV_DOT[d - 10u] : div_samples(d);
    return s < 16u ? 16u : s >= DLY_LEN ? DLY_LEN - 1u : s;
}


#include "rev_swarm.c"                                  /* the reverb */

#if FELUCCA_CPU2
/* the dry mix and the sends, as each core adds its parts into them (after Jangada: mx[1] is the second core's,
 * mix_block), and a part's own block (b; mx[0].b also the fade of a reverb model change, after the parts) */
typedef struct { int32_t l[CTL], r[CTL], d[CTL], v[CTL], b[CTL]; } mixsum_t;
static mixsum_t mx[2];
#define mix_l (mx[0].l)
#define mix_r (mx[0].r)
#define send_d (mx[0].d)
#define send_r (mx[0].v)
#define part_buf (mx[0].b)
#define SM_DECL , mixsum_t *sm                          /* mix_part / mix_spread: the sums they add into */
#define SM_ARG(x) , (x)
#define SM(f) (sm->f)
#else
/* one core: the arrays of 1.6, the same code (SM(l) = mix_l, ...; send_d .. mix_r are declared below) */
static int32_t part_buf[CTL];                            /* a part's block (mix_part); the fade of a model change */
#define SM_DECL
#define SM_ARG(x)
#define SM(f) SM1_##f
#define SM1_l mix_l
#define SM1_r mix_r
#define SM1_d send_d
#define SM1_v send_r
#define SM1_b part_buf
#endif


/* process the two buses (the delay, the reverb) for one block; sends in, wet stereo-equal out */
#define GAIN_SLEW 256u                                  /* Q15 a sample at most: a whole swing in 128 samples (~3 ms) */
static void gain_ramp(int32_t *buf, uint32_t n, int32_t *cur, int32_t to)
{
    int32_t g = *cur, st, mx = (int32_t)(GAIN_SLEW * n);
    uint32_t i;
    if (g == to && to >= 32767)
        return;
    to = clamp(to, g - mx, g + mx);                     /* (a jump, SQR's: a ~3 ms fade, no click) */
    st = (to - g) / (int32_t)(n ? n : 1u);
    for (i = 0; i < n; i++) {
        g = i + 1u == n ? to : g + st;
        buf[i] = (int32_t)(((int64_t)buf[i] * g) >> 15);
    }
    *cur = to;
}
/* The buses run one block behind (2026-11-15, for both cores' sake): bus_take keeps the delay's and reverb's settings
 * for the block (the knobs, their glides, the LFOs: read where fx_buses always ran), and as the block ends bus_keep
 * parks its sends in wet (the delay's) and rev_side (the reverb's), whose own use is over by then; the next block
 * renders them there (bus_run: fx_buses, each sample read before its place is written), on the second core while the
 * first renders the parts, or on this one. So the delay and the reverb are what they were, sample for sample, one
 * block (CTL, 0.7 ms) later, and no memory is added. The render touches only the buses' own state (dly_buf, fx, sw,
 * fxs_cur[2..3], hl.sd, wet) */
static struct {
    int32_t dtarget, fb, col, hpk, g2, g3;              /* the delay's settings, D.OUT's and R.OUT's gains */
    uint32_t n;                                         /* the samples parked (0: nothing to render) */
} bus __attribute__((section(".pool")));                /* (zeroed at boot: main.c) */
/* PLAY twice (seq.c PANIC_ALL): the delay, the reverb and the comb emptied (audio side, between blocks) */
static void fx_panic(void)
{
    memset(dly_buf, 0, sizeof dly_buf);
    rev_swarm_clear();
    bus.n = 0;                                          /* (the block taken: not into the emptied buses) */
    memset(mfx_cb, 0, sizeof mfx_cb);
}
static int32_t fxs_cur[4] = {32767, 32767, 32767, 32767}, amp_cur = 32767, amp_cr = 32767;   /* (mod.c fxs_gain's, AMP's, as the last block ended) */
static void fx_buses(const int32_t *dly_in, const int32_t *rev_in, int32_t *wet,
                     uint32_t n)
{
    uint32_t i, dl;
    const int32_t fb = bus.fb, col = bus.col, hpk = bus.hpk, dtarget = bus.dtarget;
#define SIMPLE_DT(dl) (fx.dly_cur)
    int32_t dmix = 90 * 258;                            /* (the delay's level; G_DMIX is the reverb's SMEAR now) */
    int32_t rt;
    for (i = 0; i < n; i++) {
        int32_t y = 0, x;
        /* delay: a dub-style BBD echo. The tap wobbles slowly (a clock that drifts: +-20 samples at 0.8 Hz), every repeat goes
         * through two low-passes (COLR) and a low cut, and the loop saturates (tanh), so the echoes darken, thin out and
         * thicken as they pile up, and a high FDBK swells into self-oscillation instead of running away */
        {
            int32_t dm, pos, f, x0, x1, lp;
            uint32_t ri;
            {   /* free running: the time glides to the knob (a one-pole, ~90 ms), so sweeping TIME bends the echoes' pitch */
                int32_t d = dtarget - fx.dly_cur, step = d >> 12;
                if (!fx.dly_cur)
                    d = step = dtarget, fx.dly_cur = 0;
                else if (d > 0 && !step)
                    step = 1;
                step = step > 128 ? 128 : step < -128 ? -128 : step;   /* the pitch bends at most an octave down / half up */
                fx.dly_cur += step;
                dl = (uint32_t)fx.dly_cur >> 8;
            }
            dm = (int32_t)(dl >> 2) < 20 ? (int32_t)(dl >> 2) : 20;
            fx.dly_ph += 77900u;
            pos = (int32_t)(SIMPLE_DT(dl)) + ((osc_sine(fx.dly_ph) * dm) >> 7);
            ri = (uint32_t)pos >> 8;
            f = pos & 255;
            x0 = dly_buf[(fx.dly_w - ri) & (DLY_LEN - 1u)];
            x1 = dly_buf[(fx.dly_w - ri - 1u) & (DLY_LEN - 1u)];
            x = x0 + (((x1 - x0) * f) >> 8);
            fx.dly_lp += mulq15(x - fx.dly_lp, col);
            fx.dly_lp2 += mulq15(fx.dly_lp - fx.dly_lp2, col);
            fx.dly_hp += mulq15(fx.dly_lp2 - fx.dly_hp, hpk);
            lp = fx.dly_lp2 - fx.dly_hp;
            dly_buf[fx.dly_w & (DLY_LEN - 1u)] =
                (int16_t)clamp((dly_in[i] >> 1) + softclip(mulq15(lp, fb) * 5 >> 2), -32768, 32767);
            x = lp;
        }
        fx.dly_w++;
        y += mulq15(x << 1, dmix);
        wet[i] = y;
    }
    (void)rt;
    gain_ramp(wet, n, &fxs_cur[2], bus.g2);             /* (SIMPLE: the FX LFOs' D.OUT: the echoes) */
    if (bus.g3 >= 32767 && fxs_cur[3] >= 32767) {
        rev_swarm(rev_in, wet, n);                      /* SIMPLE: the one reverb */
    } else {                                            /* (.. R.OUT: the reverb's own, its stereo side too) */
        static int32_t rw[CTL];
        int32_t c = fxs_cur[3];
        for (i = 0; i < n; i++)
            rw[i] = 0;
        rev_swarm(rev_in, rw, n);
        gain_ramp(rev_side, n, &c, bus.g3);
        gain_ramp(rw, n, &fxs_cur[3], bus.g3);
        for (i = 0; i < n; i++)
            wet[i] += rw[i];
    }
}

/* after the buses: the reverb's stereo difference to the left and from the right of the dry mix (the wet bus adds the
 * same to both) */
static __attribute__((noinline)) void rev_side_mix(int32_t *l, int32_t *r, uint32_t n)
{
    uint32_t i;
    for (i = 0; i < n; i++) {
        l[i] += rev_side[i] * (1 << MIX_SH);
        r[i] -= rev_side[i] * (1 << MIX_SH);
    }
}

/* one block of the whole mix (shared with hostsim.c): events -> each part (with its modulation matrix)
 * -> dist -> INSERT -> SLICER -> level / pan / sends -> buses -> master; out: stereo Q15 */
static void events_block(uint32_t n);                    /* seq.c */
#if FELUCCA_CPU2
static int32_t wet[CTL];                                 /* (the dry mix and the sends: mx[0], above) */
#else
static int32_t send_d[CTL], send_r[CTL], wet[CTL], mix_l[CTL], mix_r[CTL];
#endif

/* SPREAD (#148): a POLY or UNISON part whose SPRD is above 0. Its voices start left and right of PAN in turn (voice.c
 * voice_start; UNISON's detuned voices alternate), at PAN -/+ SPRD / 2 (SPRD 127: hard left and right), with the pan
 * law PAN has (the far side down, the near one as it is). track_render gives every voice (b, as always) and the left
 * ones (spread_buf); here d = left - right, the side signal, so for a pair of pans L = b (gla + glb) / 2 + d (gla - glb)
 * / 2 (R alike): the mono sum is b's, the level of a centre pan less by SPRD / 256 at most. b goes through DIST, the
 * SLICER and the FX layer's mute key as always; d gets the SLICER's gate and the mute's ramp (slicer_track_sd,
 * perf_mute_sd) and passes DIST by (the spread of a distorted chord is its voices' clean difference). The sends
 * take b, mono, as before. SPRD 0, MONO or LEGATO: mix_part's own loop, bit for bit as before. The cost: a block's
 * clear and sum of spread_buf, the side and a wider mix loop, per part; per voice none (it picks its buffer) */
static AUDIO_RAM void mix_spread(track_t *t, int32_t *b, uint32_t n SM_DECL)
{
    int32_t *d = SPREAD_BUF(t);
    uint32_t i;
    int32_t lvl = SM_TAB(LEVEL_Q12, smq(t, SK_LEVEL)), pan = smq(t, SK_PAN), s = ((t->p[P_SPRD] + 1) >> 1) * 256;   /* (smooth.c: Q8) */
    int32_t pa = clamp(pan - s, -64 * 256, 63 * 256), pb = clamp(pan + s, -64 * 256, 63 * 256);
    int32_t gla = 4096 - (pa > 0 ? pa >> 2 : 0), gra = 4096 + (pa < 0 ? -(-pa >> 2) : 0);
    int32_t glb = 4096 - (pb > 0 ? pb >> 2 : 0), grb = 4096 + (pb < 0 ? -(-pb >> 2) : 0);
    int32_t glm = (gla + glb) >> 1, grm = (gra + grb) >> 1, gld = (gla - glb) >> 1, grd = (gra - grb) >> 1;
    int32_t dl = (smq(t, SK_DLY) * 258) >> 8, r = (smq(t, SK_REV) * 258) >> 8, pk = t->peak;
    int32_t xmax = dl;
    xmax = 0x7FFFFFFF / ((xmax > r ? xmax : r) | 1);
    for (i = 0; i < n; i++)
        d[i] = (d[i] << 1) - b[i];                      /* left - right */
    for (i = 0; i < n; i++) {
        int32_t x = mulq12r(b[i], lvl), a = x < 0 ? -x : x, y = mulq12r(d[i], lvl);
        int32_t xs = clamp(x >> MIX_SH, -xmax, xmax);    /* (the sends: Q15) */
        if (a > pk)
            pk = a;
        if (dl)
            SM(d)[i] += mulq15(xs, dl);
        if (r)
            SM(v)[i] += mulq15(xs, r);
        SM(l)[i] += mulq12r(x, glm) + mulq12r(y, gld);
        SM(r)[i] += mulq12r(x, grm) + mulq12r(y, grd);
    }
    t->peak = pk;
}

/* a part's cost for the second core's split (mix_split, after Jangada's): its voices x its engine's cost per
 * voice (ENG_VCOST: the host's instructions per sample, tests/regress.c CPU at 8 notes held less the idle mix, over the
 * engine's voices), and the DIST, INSERT and sends while it sounds or its tail runs. ENGINES[] order; DIGITAL as FM6
 * when built, nothing when retired. An estimate: it only steers the split, the sound does not depend on it */
#if FELUCCA_CPU2
static uint16_t part_cost[NTRK];                        /* the last block's, per part */
static const uint16_t ENG_VCOST[NENGINES] = {181, 181, 181, 181, 181};
#endif

/* one synth part into the dry mix and the sends of sm (mx[0]; mx[1] on the second core); a part with no voice sounding costs
 * the LFO tick and a cleared buffer only (after the DIST tail has run out) */
/* SIMPLE: the gains a track's block ended on: LEVEL, pan L R, sends. An LFO / lock moves them block by block, so without a
 * glide the steps make a buzz at the block rate (crunchy overtones on LFO > VOL, PAN, DLY, REV); while they move, the
 * block glides (mix_glide), still the plain loop below */
static int32_t mix_was[NTRK][5];
static AUDIO_RAM void mix_gains(const track_t *t, int32_t *g)   /* LEVEL, pan L R, sends, as the sound takes them (smooth.c) */
{
    int32_t pa = smq(t, SK_PAN);
    g[0] = SM_TAB(LEVEL_Q12, smq(t, SK_LEVEL));
    g[1] = 4096 - (pa > 0 ? pa >> 2 : 0);               /* (pa Q8: x 64 / 256) */
    g[2] = 4096 + (pa < 0 ? -(-pa >> 2) : 0);
    g[3] = (smq(t, SK_DLY) * 258) >> 8;
    g[4] = (smq(t, SK_REV) * 258) >> 8;
}
static AUDIO_RAM void mix_glide(track_t *t, const int32_t *b, uint32_t n, const int32_t *to SM_DECL)
{
    int32_t *w = mix_was[(uint32_t)(t - trk) % NTRK], c[5], st[5], pk = t->peak, sm0, xm;
    uint32_t i, k;
    sm0 = w[3] > w[4] ? w[3] : w[4];                    /* the sends' clamp, over the block (as mix_part) */
    sm0 = to[3] > sm0 ? to[3] : sm0;
    sm0 = to[4] > sm0 ? to[4] : sm0;
    xm = 0x7FFFFFFF / (sm0 | 1);
    for (k = 0; k < 5u; k++) {
        c[k] = w[k] << 8;                               /* (LEVEL, pan: up to 4096 or so; sends 127 x 258: fit) */
        st[k] = ((to[k] - w[k]) << 8) / (int32_t)n;
        w[k] = to[k];
    }
    for (i = 0; i < n; i++) {
        int32_t x, a, xs, dv, rv;
        for (k = 0; k < 5u; k++)
            c[k] += st[k];
        dv = c[3] >> 8;
        rv = c[4] >> 8;
        x = mulq12r(b[i], c[0] >> 8);
        a = x < 0 ? -x : x;
        xs = clamp(x >> MIX_SH, -xm, xm);
        if (a > pk)
            pk = a;
        if (dv)
            SM(d)[i] += mulq15(xs, dv);
        if (rv)
            SM(v)[i] += mulq15(xs, rv);
        SM(l)[i] += mulq12r(x, c[1] >> 8);
        SM(r)[i] += mulq12r(x, c[2] >> 8);
    }
    t->peak = pk;
}
static AUDIO_RAM void mix_plain(track_t *t, const int32_t *b, uint32_t n, int32_t lvl, int32_t gl, int32_t gr, int32_t d, int32_t r,
                                int32_t xmax SM_DECL)
{
    int32_t pk = t->peak;
    uint32_t i;
    for (i = 0; i < n; i++) {
        int32_t x = mulq12r(b[i], lvl), a = x < 0 ? -x : x;   /* pre-shift: 8 loud voices */
        int32_t xs = clamp(x >> MIX_SH, -xmax, xmax);   /* sends (Q15): mulq15 would overflow */
        if (a > pk)
            pk = a;
        if (d)
            SM(d)[i] += mulq15(xs, d);
        if (r)
            SM(v)[i] += mulq15(xs, r);
        SM(l)[i] += mulq12r(x, gl);
        SM(r)[i] += mulq12r(x, gr);
    }
    t->peak = pk;
}
#ifndef BENCH_PART                                     /* TEST: each track's time in this half (audio.c audio_trk_pk) */
uint32_t audio_trk_acc[ST_N];                          /* 0..5 the tracks, 6 events (note starts), 7 delay + reverb, 8 the rest */
static uint32_t pt_t0, pt_cur = ST_N;
static void part_mark(uint32_t k)
{
    uint32_t now;
#ifdef C1_CORE
    if (C1_CORE())                                      /* (the second core's parts: not timed here) */
        return;
#endif
    now = fm1_ticks();
    if (pt_cur < ST_N)
        audio_trk_acc[pt_cur] += now - pt_t0;
    pt_cur = k;
    pt_t0 = now;
}
#define BENCH_PART(t, k) part_mark((t) ? (uint32_t)((t) - trk) : ST_N)
#define ST_MARK(k) part_mark(k)
#endif
#ifndef ST_MARK
#define ST_MARK(k)
#endif
#if defined(TEENSY_NOR_ADDR)
static void trec_begin(uint32_t n);                     /* (teensy-fm1 fork, trec.c: the track recording) */
static uint32_t trec_part(track_t *t, int32_t *b, uint32_t n, uint32_t nr);
#endif
static void mix_part(track_t *t, uint32_t n SM_DECL)
{
    int32_t *b = SM(b);
#if FELUCCA_CPU2
    uint32_t nr;
#endif
#ifdef BENCH_PART
    BENCH_PART(t, 0);                                   /* (tools/bench_jam.c: the part's cost) */
#endif
    mod_begin(t);                                       /* the matrix's per-block values into t->p (mod.c) */
    RAMCALL(sm_track)(t, n);                            /* the knobs as the sound takes them (smooth.c) */
    T_SPREAD(t) = t->p[P_SPRD] > 0 && (trk_vmode(t) == V_POLY || trk_vmode(t) == V_UNISON);   /* (SPREAD, mix_spread) */
#if FELUCCA_CPU2
#ifdef C1_TICKS
    uint32_t c0 = C1_TICKS();
    nr = track_render(t, b, n);
    {                                                   /* the device: what it really took, us (both cores time
                                                         * their own parts; the mixing after it is cheap) */
        uint32_t us = (C1_TICKS() - c0) / FM1_TICKS_PER_US + (nr ? 2u : 0u);
        part_cost[(uint32_t)(t - trk) % NTRK] = (uint16_t)(us > 65535u ? 65535u : us);
    }
#else
    nr = track_render(t, b, n);
    part_cost[(uint32_t)(t - trk) % NTRK] = (uint16_t)(nr * ENG_VCOST[t->engine % NENGINES] + (nr ? 60u : 0u));
#endif
#if defined(TEENSY_NOR_ADDR)
    nr = trec_part(t, b, n, nr);                        /* (teensy-fm1 fork: the line in heard, the take written) */
#endif
    if (nr)
#else
    if (track_render(t, b, n))
#endif
        ;
    else {                                              /* no voice sounding: nothing to mix */
        RAMCALL(mix_gains)(t, mix_was[(uint32_t)(t - trk) % NTRK]);   /* (the next hit starts on them, no glide in) */
        if (MOD(t).on)
            mod_end(t);
        return;
    }
    if (T_SPREAD(t))
        RAMCALL(mix_spread)(t, b, n SM_ARG(sm));
    else {
    }
    if (!T_SPREAD(t)) {                                /* (the mix's values read after the calls: none is held over
                                                         * them, no spill in the loop: the ISR's cost as before 1.5) */
        int32_t g5[5];
        RAMCALL(mix_gains)(t, g5);
        int32_t lvl = g5[0], gl = g5[1], gr = g5[2], d = g5[3], r = g5[4];
        int32_t xmax = d;
        const int32_t *was = mix_was[(uint32_t)(t - trk) % NTRK];
        xmax = 0x7FFFFFFF / ((xmax > r ? xmax : r) | 1);   /* sends: loud chords at a high LEVEL */
        if (was[0] != lvl || was[1] != gl || was[2] != gr || was[3] != d || was[4] != r) {   /* (moving: glide) */
            int32_t to[5];
            to[0] = lvl;
            to[1] = gl;
            to[2] = gr;
            to[3] = d;
            to[4] = r;
            RAMCALL(mix_glide)(t, b, n, to SM_ARG(sm));
        } else
            RAMCALL(mix_plain)(t, b, n, lvl, gl, gr, d, r, xmax SM_ARG(sm));
    }
    if (MOD(t).on)
        mod_end(t);                                     /* the stored values back */
}

/* the HOME oscilloscope (ui_graph.c graph_scope): every other sample of the left channel. MENU > DISPLAY > SCOPE (1.2,
 * Discussion #165): OUT (0, the default) what goes out, after MASTER (audio.c audio_block, before the click); MIX the
 * mix before MASTER (and before the FX layer, SPEAKER EQ and the limiter), at the level MASTER at its top gives, so its
 * size does not follow the volume. The click is in neither */
#define SCOPE_N 512u
static int16_t scope_buf[SCOPE_N];
static uint32_t scope_w;
static volatile uint8_t scope_mix;
static __attribute__((noinline)) void scope_take_mix(uint32_t n)
{
    uint32_t i;
    for (i = 1; i < n; i += 2u)                         /* (((x >> 2) * MASTER_FULL) >> 10 is x) */
        scope_buf[scope_w++ & (SCOPE_N - 1u)] = (int16_t)clamp((mix_l[i] >> MIX_SH) + wet[i], -32767, 32767);
}

/* the master with the FX layer's effects between its level and master_out (perform.c) */
static __attribute__((noinline)) void perf_master(int32_t *out, uint32_t n)
{
    uint32_t i;
    int32_t mg = MASTER_FULL;                       /* (MASTER: audio.c audio_block, at 24 bits) */
    for (i = 0; i < n; i++) {
        mix_l[i] = mulq12r(mix_l[i] + wet[i] * (1 << MIX_SH), mg);
        mix_r[i] = mulq12r(mix_r[i] + wet[i] * (1 << MIX_SH), mg);
    }
    for (i = 0; i < n; i++) {                           /* (the beat repeats' 16-bit loop: Q15 while they run) */
        mix_l[i] >>= MIX_SH;
        mix_r[i] >>= MIX_SH;
    }
    perf_block(mix_l, mix_r, n);
    for (i = 0; i < n; i++) {
        mix_l[i] *= 1 << MIX_SH;
        mix_r[i] *= 1 << MIX_SH;
    }
    RAMCALL(mfx_block)(mix_l, mix_r, n);
    for (i = 0; i < n; i++) {
        int32_t l = mix_l[i], r = mix_r[i];
        master_out(&l, &r);
        out[2u * i] = l;
        out[2u * i + 1u] = r;
    }
}

/* the block's sends and the buses' settings for it, as it ends (on this core, where fx_buses used to run) */
static void bus_take(void)
{
    bus.fb = (clamp(smg(SKG_DFDBK), 0, 127 << 8) * 260) >> 8;   /* (smooth.c: Q8) */
    bus.dtarget = delay_q8();
    {   /* TONE: 64 the dub echo, below it the repeats go darker (a low-pass), above it thinner (a high-pass) */
        int32_t tn = clamp(smg(SKG_DTONE), 0, 127 << 8) - (64 << 8);
        bus.col = tn < 0 ? 14000 - ((-tn * 180) >> 8) : 14000 + ((tn * 127) >> 8);
        bus.hpk = tn > 0 ? 700 + ((tn * 130) >> 8) : 700;
    }
    bus.g2 = fxs_gain[2];
    bus.g3 = fxs_gain[3];
    sw_block();                                         /* (the reverb's settings: rev_swarm.c) */
}
static void bus_keep(uint32_t n)                        /* the block's last use of wet and rev_side is over */
{
    uint32_t i;
    for (i = 0; i < n; i++) {
        wet[i] = send_d[i];
        rev_side[i] = send_r[i];
    }
    bus.n = n;
}
#ifdef C1_TICKS
static uint16_t bus_cost;                               /* us, the last bus_run (mix_split) */
#endif
/* the block taken last time into wet and rev_side (either core) */
static void bus_run(void)
{
    uint32_t i;
#ifdef C1_TICKS
    uint32_t c0 = C1_TICKS(), us;
#endif
    fx_buses(wet, rev_side, wet, bus.n);                /* (in place: bus_keep) */
    for (i = bus.n; i < CTL; i++)
        wet[i] = rev_side[i] = 0;
#ifdef C1_TICKS
    us = (C1_TICKS() - c0) / FM1_TICKS_PER_US;
    bus_cost = (uint16_t)(us > 65535u ? 65535u : us);
#endif
}

/* ---- the second core (hal/fm1_cpu1.h; after Jangada's, github.com/zednaked/jangada, GPL-3.0) renders some of
 * the parts of each block into its own sums (mx[1]) while this core renders the others; then this core adds them in
 * and goes on (the LFOs' blocks, the FX layer, the buses, the master) as with one core. The sums are integers, so the
 * order they add up in changes nothing: the output is bit for bit one core's. A part's render touches its own state
 * only (its track_t with its SPREAD, ENV SYNC and LFO phase, its voices, its matrix: mod.c MOD, its SPREAD buffer,
 * its engine's memory) and what nobody writes meanwhile (song, the tables); what parts share is done before the
 * split (events_block: the notes, voice stealing; the FX layer's perf_begin) or after it (the LFOs' S&H draw from the
 * shared generator, part by part: voice.c track_lfo_end). tests/dualcore_test.c checks it, under ThreadSanitizer too.
 * Without a second core (the host, the browser, MENU > 2ND CORE OFF, one that did not answer or stopped) C1_RUN takes
 * nothing and this core renders them all */
#if FELUCCA_CPU2
#ifndef C1_RUN
#define C1_ON() 0                                       /* a second core is answering */
#define C1_RUN(fn, arg) 0                               /* hand it fn(arg): 1 = taken */
#define C1_WAIT() 0                                     /* wait for it: 0 = done */
#endif
#ifndef C1_SPLIT
#define C1_SPLIT() mix_split()
#endif
static uint32_t c1_blocks;                              /* blocks the second core took parts of (console) */

#define C1_BUS (1u << 24)                               /* c1_parts: the buses too (bus_run) */
static void c1_parts(uint32_t arg)                      /* arg: the parts (bit k = part k) | n << 8 | C1_BUS */
{
    uint32_t i, n = (arg & ~C1_BUS) >> 8;
    if (arg & C1_BUS)
        bus_run();
    mixsum_t *sm = &mx[1];
    for (i = 0; i < n; i++)
        sm->l[i] = sm->r[i] = sm->d[i] = sm->v[i] = 0;
    for (i = 0; i < NPART; i++)
        if (arg >> i & 1u)
            mix_part(&trk[i], n, sm);
}

/* which parts go to the second core: the split of the last block's costs that leaves the busier core the least; none
 * when that saves less than handing them over costs (C1_HAND) */
#ifdef C1_TICKS
#define C1_HAND 6u                                      /* (us: the device times the parts) */
#else
#define C1_HAND 40u
#endif
static uint32_t mix_split(void)
{
    uint32_t m, i, best = 0, all = 0, lo, b = 0;
#ifdef C1_TICKS
    b = bus_cost;                                       /* (the second core has the buses first) */
#endif
    for (i = 0; i < NPART; i++)
        all += part_cost[i];
    lo = (all > b ? all : b) + C1_HAND;
    for (m = 1; m < (1u << NPART) - 1u; m++) {
        uint32_t a = b, w;
        for (i = 0; i < NPART; i++)
            if (m >> i & 1u)
                a += part_cost[i];
        w = (a > all + b - a ? a : all + b - a) + C1_HAND;
        if (w < lo) {
            lo = w;
            best = m;
        }
    }
    return best;
}
#endif

/* SIMPLE: buf's n by a gain going from *cur to to (Q15, 32767 none: a block at it is left alone), no steps (no clicks) */
static AUDIO_RAM void mix_gain(const int32_t *wet, int32_t mg, uint32_t n)
{
    uint32_t i;
    for (i = 0; i < n; i++) {
        mix_l[i] = mulq12r(mix_l[i] + wet[i] * (1 << MIX_SH), mg);
        mix_r[i] = mulq12r(mix_r[i] + wet[i] * (1 << MIX_SH), mg);
    }
}
static AUDIO_RAM void mix_out(int32_t *out, uint32_t n)
{
    uint32_t i;
    for (i = 0; i < n; i++) {
        int32_t l = mix_l[i], r = mix_r[i];
        master_out(&l, &r);
        out[2u * i] = l;
        out[2u * i + 1u] = r;
    }
}
static void amp_block(uint32_t n)                       /* SIMPLE: AMP (G_AMP, as modulated), the whole mix; squared */
{
    int32_t v = clamp(smg(SKG_AMP), 0, 127 << 8), p = clamp(smg(SKG_PAN), -64 * 256, 64 * 256);   /* (smooth.c: Q8) */
    int32_t to = v >= 127 << 8 ? 32767 : (int32_t)(((int64_t)v * v * 32767) / (127 * 127 * 65536));
    gain_ramp(mix_l, n, &amp_cur, p > 0 ? to * (64 * 256 - p) / (64 * 256) : to);   /* PAN: the other side quieter (a balance) */
    gain_ramp(mix_r, n, &amp_cr, p < 0 ? to * (64 * 256 + p) / (64 * 256) : to);
}

#define HEARD_Q15 8                             /* ~-72 dB: below it nothing an erase cuts is heard */
static volatile uint32_t audio_heard_ms;        /* when the output was last above that */
static void mix_block(int32_t *out, uint32_t n)
{
    uint32_t i;
    int perf;
    int32_t mg = MASTER_FULL;                       /* (MASTER: audio.c audio_block, at 24 bits) */
    for (i = 0; i < n; i++)
        send_d[i] = send_r[i] = mix_l[i] = mix_r[i] = 0;
    ST_MARK(NTRK);
    events_block(n);
#if defined(TEENSY_NOR_ADDR)
    trec_begin(n);                                      /* (teensy-fm1 fork: takes starting, the line in) */
#endif
    perf = perf_begin(n);                               /* the FX hold layer at work (perform.c) */
#if FELUCCA_CPU2
    uint32_t bn = bus.n, c1 = C1_ON(), m = c1 ? C1_SPLIT() & ((1u << NPART) - 1u) : 0u;
    uint32_t job = m || bn ? m | n << 8 | (bn ? C1_BUS : 0u) : 0u;
    if (job && !(c1 && C1_RUN(c1_parts, job)))
        job = m = 0;
    c1_blocks += m != 0;
    if (bn && !(job & C1_BUS))
        bus_run();                                      /* (one core: the buses here, the same order) */
    for (i = 0; i < NPART; i++)
        if (!(m >> i & 1u))
            mix_part(&trk[i], n, &mx[0]);
    if (job && C1_WAIT())                               /* it stopped answering (held from now on): its work here */
        c1_parts(job);
    if (m) {
        const mixsum_t *sm = &mx[1];
        for (i = 0; i < n; i++) {
            mix_l[i] += sm->l[i];
            mix_r[i] += sm->r[i];
            send_d[i] += sm->d[i];
            send_r[i] += sm->v[i];
        }
    }
    for (i = 0; i < NPART; i++)                         /* the LFOs' blocks, part by part (voice.c) */
        track_lfo_end(&trk[i]);
#else
    uint32_t bn = bus.n;
    if (bn)
        bus_run();
    for (i = 0; i < NPART; i++)                         /* (one core: as 1.6) */
        mix_part(&trk[i], n);
#endif
    hl.side = bn != 0;                                  /* (wet, rev_side: the block taken last time) */
    if (!bn)
        for (i = 0; i < n; i++)
            wet[i] = 0;
#ifdef BUS_TAP
    if (bn)
        BUS_TAP(wet, rev_side, n);                      /* (tools/emu_shot.c: the buses' output, to compare) */
#endif
#ifdef BENCH_PART
    BENCH_PART((track_t *)0, 0);                        /* (the rest: the master's) */
#endif
    ST_MARK(NTRK + 2u);
    mod_g_begin(n);                                     /* (SIMPLE: the LFOs on the master's: mod.c) */
    gain_ramp(send_d, n, &fxs_cur[0], fxs_gain[0]);     /* (SIMPLE: the FX LFOs' D.SND R.SND) */
    gain_ramp(send_r, n, &fxs_cur[1], fxs_gain[1]);
    ST_MARK(NTRK + 1u);
    bus_take();                                         /* (rendered next block: bus_run) */
    ST_MARK(NTRK + 2u);
    if (hl.side)
        rev_side_mix(mix_l, mix_r, n);                  /* the reverb's stereo */
    if (scope_mix)
        scope_take_mix(n);
    if (perf) {
        perf_master(out, n);
        mod_g_end();
        bus_keep(n);
        ST_MARK(ST_N);
        return;
    }
    RAMCALL(mix_gain)(wet, mg, n);
    RAMCALL(mfx_block)(mix_l, mix_r, n);
    amp_block(n);
    mod_g_end();
    RAMCALL(mix_out)(out, n);
    for (i = 0; i < n; i++)                             /* still heard? (reverb and delay tails: flash saves wait, persist.c) */
        if (out[2u * i] > HEARD_Q15 << MIX_SH || out[2u * i] < -(HEARD_Q15 << MIX_SH) || out[2u * i + 1u] > HEARD_Q15 << MIX_SH || out[2u * i + 1u] < -(HEARD_Q15 << MIX_SH)) {
            audio_heard_ms = fm1_ms;
            break;
        }
    bus_keep(n);
    ST_MARK(ST_N);
    return;
    for (i = 0; i < n; i++) {
        int32_t l = mulq12r(mix_l[i] + wet[i], mg);
        int32_t r = mulq12r(mix_r[i] + wet[i], mg);
        master_out(&l, &r);
        out[2u * i] = l;
        out[2u * i + 1u] = r;
    }
}

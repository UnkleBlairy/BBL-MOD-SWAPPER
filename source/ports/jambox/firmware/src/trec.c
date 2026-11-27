/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2026 Blair (blairysbeatlab), BBL JAM BOX */
/* teensy-fm1 fork only (FORK.md, docs/TRACK_REC.md): per-track recording into the Teensy's PSRAM. Phase 1:
 *   REC page    K1 ARM OFF/ON  K2 START REC/PLAY/SYNC  K3 LEN 1/4 1/2 1 2 4 8 (bars of 16 of the track's steps) or PAT (its
 *               LEN)  K4 MODE STD (one take, ARM turns off) / LOOPER (the take then plays round with the track, ARM on =
 *               every pass overdubs: new = old x DUB + this; ARM off = it just plays; REC IN's CLEAR, turned twice, clears it)
 *   REC IN page K1 SRC TRACK  K2 GAIN (dB)  K3 DUB % (LOOPER: how much of the loop stays each pass; 0 = a fresh take)
 * On an INPUT track a step or key plays the track's take from its start (trec_trig; middle C as recorded), over the line in.
 * A take is 16 bits at 44.1 kHz, mono, into fresh pool space; when it is done it becomes the track's slot Rn (another
 * track's sampler plays it; eng_sampler.c SMP_R) and ARM turns itself off. LINE IN is heard through the track (its
 * VOL, PAN and sends) while armed or recording. TRACK records the track's own sound before VOL, PAN and the sends.
 *
 * When a take starts: START REC = on a bar line (the master's count, every 16 1/16s) while the sequencer records
 * (REC); START PLAY = on PLAY; START SYNC = the
 * track's own loop's step 1, whether it was already playing (the next loop) or PLAY was pressed (no REC needed). Its length is fixed then (tempo, the track's speed). A LINE IN take is read late by the
 * round trip (teensy_in_latency: the FM-1's output lag + the line in's read lag), so what was played along with the
 * bar line lands at the take's start.
 *
 * Audio side (trec_begin, trec_part: fx.c mix_block / mix_part, the audio render) and the pages' side (trec_knob,
 * trec_view: ui_gb.c, ui_input.c) below. The line in and the pool come from the Teensy (src/audio_in.rs). */

#define TREC_MAGIC 0x31435254u                     /* "TRC1": the jam's section (gb_store.c) */
enum { TS_REC, TS_PLAY, TS_SYNC, TS_N };            /* START (SYNC: the track's loop's step 1, playing or at PLAY) */
enum { TSRC_LINE, TSRC_TRACK, TSRC_N };             /* SRC (MASTER: phase 2) */
enum { TM_STD, TM_LOOP, TM_N };                     /* MODE: STANDARD (one take, ARM off) / LOOPER (plays, overdubs) */
#define TL_N 7u                                     /* LEN: 1/4 1/2 1 2 4 8 PAT */
#define TG_MIN (-40)
#define TG_MAX 24
typedef struct { uint8_t arm, start, len, mode, src, dubx, rsv1; int8_t gain; } trec_set_t;   /* (saved: the jam; dubx =
                                                                                                 * 100 - DUB %: old jams 0 = 100 %) */
static trec_set_t trec_set[NTRK];

typedef struct {
    uint32_t at, want, pos, skip;                  /* the take: where in the pool, its length, written, still to skip */
    uint8_t st;                                    /* 0 idle, 1 armed and waiting, 2 recording */
    uint8_t loop;                                  /* LOOPER: the buffer plays round (and overdubs while ARM is on) */
    uint32_t lpos;                                 /* .. where it is */
    uint32_t sk0;                                  /* the take's skip at its start (INPUT: the round trip) */
} trec_t;
/* INPUT: a step or key plays the track's take (trec_trig), as the sampler plays a chunk (eng_sampler.c, its tables):
 * two Hann grains half a grain apart read at the note's speed (PITCH: tape), each next one starting half a grain /
 * STRCH further on; LOFI's sample-hold and bits; MODE ONCE / REV play to the end, LOOP / PING while the note is held.
 * The PLAYBACK page (INPUT's page 2): E2 PITCH -24..+24, E3 STRCH (-64..63: 0 = 1.00x), E4 LOFI, E5 MODE */
#define TP_FADE 256u                               /* LOOP / PING let go: the fade out, samples */
typedef struct {
    const int16_t *w;
    uint32_t len;
    voice_t *v;                                    /* the note's voice (LOOP / PING: while its gate is held) */
    int32_t c, pv;                                 /* the grain playing's start and the one before's, Q8 (virtual: past
                                                    * the end for LOOP / PING) */
    uint32_t gpos, step, el;                       /* in the grain (Q16); the note's speed (Q16, 1.0 = middle C); pend: the
                                                    * samples it has waited */
    int32_t p16, y;                                /* the note (pitch16, PITCH added each block); LOFI's held value */
    uint32_t hc, fade;                             /* LOFI's hold count; the let-go fade left (0: none) */
    uint8_t gb, rel;
    volatile uint8_t on;
    uint8_t pend;                                  /* the slot was recording at the trig: it plays the take from where it
                                                    * would have got to once it ends */
} trec_tp_t;
static trec_tp_t trec_tp[NTRK];
static void trec_tp_go(const track_t *t, trec_tp_t *tp, const int16_t *w, uint32_t len, uint32_t el);
static void trec_tp_run(const track_t *t, trec_tp_t *tp, int32_t *b, uint32_t n);
/* each R slot's waveform for the screen (gb_store.c smp_peaks), built as it is written: reading a whole buffer back
 * from the PSRAM in a UI frame stalled the audio (~0.1 s at "R1 RECORDED"). trec_tkpk: a take's, while it records */
static uint8_t trec_pk[NTRK][GB_WAVE], trec_tkpk[NTRK][GB_WAVE];
static uint32_t trec_dcol[NTRK];                   /* LOOPER: the column the overdub is in (its peak restarts there) */
static trec_t trec[NTRK];
static uint32_t trec_buf_at[NTRK];                 /* each R slot's buffer: where in the pool (smp_tab[k]: its length) */
static int16_t *trec_pool;
static uint32_t trec_pool_n;                       /* samples */
/* this block's line in: eng_mono.c in_buf / in_ok (INPUT, the machine that plays it; trec_begin reads it) */
static uint8_t trec_was_play;
static uint16_t trec_gbl[NTRK];                   /* each track's loop count last block (SYNC: a new loop) */
static uint32_t trec_mst_was;
static int32_t trec_g[NTRK];                       /* GAIN, Q12 */
static volatile uint8_t trec_done, trec_full;
static int32_t trec_out_pk;                        /* (the 'R' report: the output after MASTER, 24 bit) */
static int32_t trec_in_pk, trec_src_pk[NTRK];      /* (the serial 'R' report: the loudest since it was last asked) */      /* bit k: track k's take is done / found no room (the pages' poll) */

extern int16_t *teensy_pool(uint32_t *n);
extern uint32_t teensy_line_in(int16_t *dst, uint32_t n);
extern uint32_t teensy_in_latency(void);
extern uint32_t teensy_line_in_state(void);
extern void teensy_line_in_arm(void);

/* track t plays the line in (INPUT, eng_mono.c): the engine it sounds with now */
static inline int trk_in(const track_t *t) { return t->engine == ENGI_MONO && MONO_IN(t); }

static const uint8_t TL_STEPS[TL_N - 1u] = {4, 8, 16, 32, 64, 128};
static const char *const TL_NAME[TL_N] = {"1/4", "1/2", "1", "2", "4", "8", "PAT"};

static void trec_gain_set(uint32_t k)              /* 4096 x 10^(dB / 20), 1 dB at a time */
{
    int32_t g = 4096, d = trec_set[k].gain;
    for (; d > 0; d--)
        g = (int32_t)(((int64_t)g * 73533) >> 16);  /* (+1 dB: x 1.12202) */
    for (; d < 0; d++)
        g = (int32_t)(((int64_t)g * 58409) >> 16);  /* (-1 dB: x 0.89125) */
    trec_g[k] = g;
}
static void trec_defaults(void)
{
    uint32_t k;
    for (k = 0; k < NTRK; k++) {
        trec_set[k] = (trec_set_t){0};
        trec_set[k].len = 2;                       /* 1 bar */
        trec_set[k].src = TSRC_TRACK;              /* (LINE: the INPUT machine now, not a SRC) */
        trec_gain_set(k);
    }
}
static void trec_loaded(void)                      /* (gb_store.c jam_apply) a jam's settings: in range, nothing armed */
{
    uint32_t k;
    for (k = 0; k < NTRK; k++) {
        trec_set_t *s = &trec_set[k];
        s->arm = 0;
        s->start %= TS_N;
        s->len = (uint8_t)(s->len < TL_N ? s->len : 2u);
        s->mode %= TM_N;
        s->dubx = (uint8_t)(s->dubx <= 100u ? s->dubx : 0u);
        s->src = TSRC_TRACK;                       /* (a jam from before INPUT: its LINE is TRACK) */
        s->gain = (int8_t)clamp(s->gain, TG_MIN, TG_MAX);
        trec_gain_set(k);
    }
}

/* the take's length, samples: LEN in the track's steps */
static uint32_t trec_len(const track_t *t, const trec_set_t *s)
{
    uint32_t steps = s->len < TL_N - 1u ? TL_STEPS[s->len] : (uint32_t)(t->p[P_SLEN] > 0 ? t->p[P_SLEN] : 1);
    return steps * div_samples((uint32_t)t->p[P_SDIV] & 0xFFu);
}
/* room in the pool for n samples (first fit), not over any other slot's buffer nor a take on its way; own: track k's
 * own buffer may be used (only when nothing else fits). 0xFFFFFFFF: none */
static uint32_t trec_alloc(uint32_t k, uint32_t n, int own)
{
    uint32_t at = 0;
    for (;;) {
        uint32_t j, moved = 0;
        if (at + n > trec_pool_n || at + n < at)
            return 0xFFFFFFFFu;
        for (j = 0; j < NTRK; j++) {
            uint32_t a, l;
            if (j != k || !own) {                  /* a slot's buffer */
                a = trec_buf_at[j];
                l = smp_tab[j].len;
                if (l && at < a + l && a < at + n) {
                    at = a + l;
                    moved = 1;
                }
            }
            if (j != k && trec[j].st == 2u) {      /* another take on its way */
                a = trec[j].at;
                l = trec[j].want;
                if (at < a + l && a < at + n) {
                    at = a + l;
                    moved = 1;
                }
            }
        }
        if (!moved)
            return at;
    }
}
static void trec_start(uint32_t k)
{
    track_t *t = &trk[k];
    trec_t *r = &trec[k];
    uint32_t n = trec_len(t, &trec_set[k]), at;
    smp_tab[k].len = 0;                            /* a new take wipes the old one (Blair): its sound and its pool space */
    smp_tab[k].w = 0;
    if (trec_tp[k].on) {                           /* (a trig this same block, on the old take: it waits for the new) */
        trec_tp[k].on = 0;
        trec_tp[k].pend = 1;
        trec_tp[k].el = 0;
    }
    r->loop = 0;
    memset(trec_pk[k], 0, GB_WAVE);
    at = trec_alloc(k, n, 0);
    if (at == 0xFFFFFFFFu || !trec_pool) {
        r->st = 0;
        trec_set[k].arm = 0;
        trec_full |= (uint8_t)(1u << k);
        return;
    }
    r->at = at;
    r->want = n;
    r->pos = 0;
    r->skip = trk_in(t) ? teensy_in_latency() : 0u;   /* (INPUT: the line in arrives late, the round trip) */
    r->sk0 = r->skip;
    memset(trec_tkpk[k], 0, GB_WAVE);
    r->st = 2;
}
static void trec_finish(uint32_t k)
{
    trec_t *r = &trec[k];
    trec_tp_t *tp = &trec_tp[k];
    smp_chunk_t c;
    memset(&c, 0, sizeof c);
    c.off = 0x01000000u * (k + 1u);                /* (a name only: eng_sampler.c reads c.w) */
    c.len = r->want;
    c.rate = (uint16_t)FS;
    c.name[0] = 'R';
    c.name[1] = (char)('1' + k);
    c.w = trec_pool + r->at;
    memcpy(trec_pk[k], trec_tkpk[k], GB_WAVE);
    trec_dcol[k] = 0xFFFFFFFFu;
    smp_tab[k] = c;                                /* (a voice playing the one before keeps reading it) */
    if (tp->pend) {                                /* INPUT: a trig while it recorded (its step 1 comes just before the
                                                    * take ends: the round trip): the new take from where it has got to */
        tp->pend = 0;
        trec_tp_go(&trk[k], tp, c.w, c.len, tp->el);
    }
    trec_buf_at[k] = r->at;
    r->st = 0;
    if (trec_set[k].mode == TM_LOOP) {             /* LOOPER: it plays round from here (the loop's start), ARM on = */
        r->loop = 1;                               /* overdubbing */
        r->lpos = r->sk0 % r->want;                /* (the track is that far into its next pass: the skip) */
    } else
        trec_set[k].arm = 0;                       /* STANDARD: done */
    trec_done |= (uint8_t)(1u << k);
}

/* the block's start (fx.c mix_block, after the events): the bar lines, PLAY, the takes that start, the line in */
static void trec_begin(uint32_t n)
{
    uint32_t k, need = 0, bar = 0, play = song.playing && !trec_was_play;
    if (song.playing && (play || mst_step != trec_mst_was) && !(mst_step & 15u))
        bar = 1;
    trec_was_play = song.playing;
    trec_mst_was = mst_step;
    for (k = 0; k < NTRK; k++) {
        trec_t *r = &trec[k];
        const trec_set_t *s = &trec_set[k];
        if (r->loop && (s->mode != TM_LOOP || !smp_tab[k].w))
            r->loop = 0;                           /* (MODE turned to STANDARD, the buffer cleared: the loop stops) */
        if (r->loop && play)
            r->lpos = 0;                           /* PLAY: the loop from its start, with the tracks */
        if (!s->arm && r->st)
            r->st = 0;                             /* ARM off: the take is dropped (the buffer before stays) */
        else if (s->arm && !r->st && !r->loop)
            r->st = 1;                             /* (LOOPER playing: ARM on overdubs, no new take) */
        {                                          /* SYNC: the track's own loop is at its step 1 (a new loop, or PLAY) */
            const track_t *t = &trk[k];
            uint32_t top = song.playing && !t->seq_idx && (play || t->gb_loop != trec_gbl[k]);
            trec_gbl[k] = t->gb_loop;
            if (r->st == 1u && ((s->start == TS_REC && bar && song.rec) || (s->start == TS_PLAY && play) ||
                                (s->start == TS_SYNC && top)))
                trec_start(k);
        }
        if (trk_in(&trk[k]))
            need = 1;
    }
    in_ok = (uint8_t)(need && n <= 32u && teensy_line_in(in_buf, n));
    if (in_ok)
        for (k = 0; k < n; k++)
            trec_in_pk = in_buf[k] > trec_in_pk ? in_buf[k] : -in_buf[k] > trec_in_pk ? -in_buf[k] : trec_in_pk;
}

/* a track's block (fx.c mix_part, after track_render: b its sound, before VOL, PAN, the sends; nr its voices): the
 * line in heard through it, the take written. Returns what mix_part takes as "sounding" */
static uint32_t trec_part(track_t *t, int32_t *b, uint32_t n, uint32_t nr)
{
    uint32_t k = (uint32_t)(t - trk) % NTRK, i;
    trec_t *r = &trec[k];
    const trec_set_t *s = &trec_set[k];
    int32_t g = trec_g[k];
    trec_tp_t *tp = &trec_tp[k];
    uint32_t lp = r->loop && song.playing && smp_tab[k].w && smp_tab[k].len, dub = s->arm && lp;
    int32_t dq = (100 - (int32_t)s->dubx) * 128 / 100;   /* DUB, Q7: how much of the loop stays each pass */
    uint32_t lat = lp && trk_in(t) ? teensy_in_latency() % smp_tab[k].len : 0u;   /* (INPUT: what comes in now was
                                                         * played along with the loop that far back) */
    int32_t pre[CTL];                              /* INPUT: after its INPUT page (GAIN SAT), before DRIVE and the filter */
    uint32_t in = trk_in(t) && n <= CTL;
    if (in) {                                      /* INPUT: always open (eng_mono.c) */
        RAMCALL(mono_in_pre)(t, pre, n);
        nr |= 1u;
    } else
        tp->on = tp->pend = 0;                     /* (not INPUT now: its take doesn't play) */
    for (i = 0; (s->arm || r->st || lp) && i < n; i++) {   /* (a take: INPUT's input before the filter, else the track's
                                                    * sound; never the take playing) */
        int32_t x, y = in ? pre[i] : (int32_t)(((int64_t)b[i] * 1398101) >> (20 + MIX_SH));   /* (x 32768 / VOICE_FS) */
        x = clamp((y * g) >> 12, -32768, 32767);
        if (x > trec_src_pk[k] || -x > trec_src_pk[k])
            trec_src_pk[k] = x < 0 ? -x : x;
        if (r->st == 2u) {
            if (r->skip)
                r->skip--;
            else if (r->pos < r->want) {
                int16_t v = (int16_t)x;
                if (r->pos < 88u)                  /* ~2 ms fades at its edges */
                    v = (int16_t)(x * (int32_t)r->pos / 88);
                else if (r->want - r->pos <= 88u)
                    v = (int16_t)(x * (int32_t)(r->want - r->pos - 1u) / 88);
                {                                  /* (the screen's waveform, as it goes) */
                    uint32_t col = r->pos * GB_WAVE / r->want, m = (uint32_t)(v < 0 ? -v : v) >> 7;
                    if (m > trec_tkpk[k][col])
                        trec_tkpk[k][col] = (uint8_t)m;
                }
                trec_pool[r->at + r->pos++] = v;
                if (r->pos >= r->want) {
                    trec_finish(k);
                    lp = r->loop && song.playing;  /* (LOOPER: round from the next sample) */
                    dub = s->arm && lp;
                    lat = lp && trk_in(t) ? teensy_in_latency() % smp_tab[k].len : 0u;
                }
                continue;
            }
        }
        if (lp) {                                  /* LOOPER's REC layer: it keeps time; ARM on: new = old x DUB + this.
                                                    * Heard only through trigs and keys (the PLAY layer, trec_trig; Blair:
                                                    * "you have to have a trig down to hear the loop", as STANDARD) */
            int16_t *w = (int16_t *)smp_tab[k].w;
            if (dub) {
                uint32_t at = r->lpos >= lat ? r->lpos - lat : r->lpos + smp_tab[k].len - lat;
                uint32_t col = at * GB_WAVE / smp_tab[k].len, m;
                w[at] = (int16_t)clamp(((w[at] * dq) >> 7) + x, -32768, 32767);
                m = (uint32_t)(w[at] < 0 ? -w[at] : w[at]) >> 7;
                if (col != trec_dcol[k]) {         /* (each column's peak afresh each pass: DUB fades show) */
                    trec_dcol[k] = col;
                    trec_pk[k][col] = 0;
                }
                if (m > trec_pk[k][col])
                    trec_pk[k][col] = (uint8_t)m;
            }
            if (++r->lpos >= smp_tab[k].len)
                r->lpos = 0;
        }
    }
    if (tp->pend) {                                /* (waiting for the take: the time counts on) */
        tp->el += n;
        if (r->st != 2u)                           /* (the take dropped: ARM off) */
            tp->pend = 0;
    }
    if (tp->on)                                    /* INPUT: the take a step or key started (the PLAYBACK page), */
        trec_tp_run(t, tp, pre, n);                /* with the input, into the effects */
    if (in)
        RAMCALL(mono_in_post)(t, pre, b, n);
    return nr;
}

/* the PLAYBACK page's MODE and STRCH (Q12 x), LOFI */
#define TP_MODE(t) ((uint32_t)(t)->p[P_E5] & 3u)
#define TP_STR(t) ((uint32_t)SMP_STRETCH[clamp((t)->p[P_E3], -64, 63) + 64])
/* the take's sample at virtual place u, by MODE (ONCE: silence past the end; REV: from the end; LOOP; PING: there
 * and back) */
static inline int32_t tp_src(const trec_tp_t *tp, uint32_t mode, int32_t u)
{
    int32_t L = (int32_t)tp->len;
    if (mode == SM_LOOP)
        u = u < 0 ? 0 : u % L;
    else if (mode == SM_PING) {
        int32_t per = 2 * (L - 1);
        u = u < 0 ? 0 : u % (per > 0 ? per : 1);
        u = u < L ? u : per - u;
    } else if (u < 0 || u >= L)
        return 0;
    else if (mode == SM_REV)
        u = L - 1 - u;
    return tp->w[u];
}
/* the note's speed now: its pitch + PITCH (semitones, live: knob, lock, LFO) */
static void tp_speed(const track_t *t, trec_tp_t *tp)
{
    int32_t p = clamp(tp->p16 + clamp(t->p[P_E2], -24, 24) * 16, 0, 2047);
    tp->step = (uint32_t)(((uint64_t)pitch_inc((uint32_t)p) << 16) / pitch_inc(60u * 16u));
}
/* start playing w (len samples), el samples in (a trig that waited for the take: where it would have got to) */
static void trec_tp_go(const track_t *t, trec_tp_t *tp, const int16_t *w, uint32_t len, uint32_t el)
{
    uint32_t gb = 11u;                             /* grains of 2048 (~46 ms, as the sampler's ~40), half the take at most */
    tp->on = 0;
    if (!w || len < 64u)
        return;
    while (gb > 6u && (1u << gb) > len / 2u)
        gb--;
    tp->w = w;
    tp->len = len;
    tp->gb = (uint8_t)gb;
    tp_speed(t, tp);
    tp->c = (int32_t)(((uint64_t)el * tp->step >> 8) * 4096u / TP_STR(t));   /* (Q8: el x speed / STRCH) */
    tp->pv = tp->c - (int32_t)(1u << (gb - 1u + 8u));   /* the grain "before" lines up: no fade in */
    tp->gpos = 0;
    tp->hc = 0;
    tp->fade = 0;
    tp->rel = 0;
    tp->on = 1;
}
/* a block of the take into INPUT's pre (trec_part: then DRIVE and the filter) */
static void trec_tp_run(const track_t *t, trec_tp_t *tp, int32_t *b, uint32_t n)
{
    uint32_t i, mode = TP_MODE(t), lf = (uint32_t)clamp(t->p[P_E4], 0, 127), hold = SMP_HOLD[lf], near = lf >= 16u;
    uint32_t sh = lf >= 2u ? 16u - (16u * 127u - 12u * lf + 63u) / 127u : 0u;   /* LOFI's bits: 16 .. 4 (the sampler's) */
    uint32_t hb = tp->gb - 1u, hq = 1u << (hb + 16u), adv = (1u << (hb + 20u)) / TP_STR(t);   /* half a grain (Q16), the
                                                                                               * next grain's start (Q8) */
    int32_t half = 1 << hb;
    tp_speed(t, tp);
    if ((mode == SM_LOOP || mode == SM_PING) && !tp->rel && !(tp->v && tp->v->active && tp->v->gate)) {
        tp->rel = 1;                               /* (let go: a short fade) */
        tp->fade = TP_FADE;
    }
    for (i = 0; i < n; i++) {
        if (!tp->hc) {                             /* LOFI's sample-hold: a new value every hold samples */
            uint32_t k = tp->gpos >> 16, wi = tp->gb >= 8u ? k >> (tp->gb - 8u) : k << (8u - tp->gb);
            int32_t c0 = tp->c >> 8, p0 = tp->pv >> 8, y;
            y = (tp_src(tp, mode, p0 + (int32_t)k + half) * SMP_HANN[wi + 128u] +
                 tp_src(tp, mode, c0 + (int32_t)k) * SMP_HANN[wi]) >> 15;
            if (!near) {                           /* (clean: between this sample and the next) */
                uint32_t k1 = k + 1u, w1 = tp->gb >= 8u ? k1 >> (tp->gb - 8u) : k1 << (8u - tp->gb);
                int32_t y1;
                w1 = w1 > 127u ? 127u : w1;
                y1 = (tp_src(tp, mode, p0 + (int32_t)k1 + half) * SMP_HANN[w1 + 128u] +
                      tp_src(tp, mode, c0 + (int32_t)k1) * SMP_HANN[w1]) >> 15;
                y += (int32_t)(((int64_t)(y1 - y) * (tp->gpos & 0xFFFFu)) >> 16);
            }
            if (sh)
                y = ((y + (1 << (sh - 1u))) >> sh) << sh;
            tp->y = y;
        }
        if (++tp->hc >= hold)
            tp->hc = 0;
        {
            int32_t y = tp->y;
            if (tp->fade) {
                y = (int32_t)((int64_t)y * (int32_t)tp->fade / (int32_t)TP_FADE);
                if (!--tp->fade) {
                    tp->on = 0;
                    return;
                }
            }
            b[i] += y;                             /* (INPUT's pre: 16-bit scale, before DRIVE and the filter) */
        }
        tp->gpos += tp->step;
        while (tp->gpos >= hq) {                   /* the next grain */
            tp->gpos -= hq;
            tp->pv = tp->c;
            tp->c += (int32_t)adv;
            if (mode == SM_LOOP || mode == SM_PING) {   /* (round: back by a period, so the places never overflow) */
                int32_t per = mode == SM_LOOP ? (int32_t)tp->len : 2 * ((int32_t)tp->len - 1);
                if (tp->pv >> 8 >= per) {
                    tp->pv -= per << 8;
                    tp->c -= per << 8;
                }
            }
        }
        if ((mode == SM_ONCE || mode == SM_REV) && tp->pv >> 8 >= (int32_t)tp->len) {
            tp->on = 0;                            /* (both grains past the end) */
            return;
        }
    }
}
/* INPUT (eng_mono.c mono_note_on): a step or key plays the track's take from its start, middle C = as recorded */
static void trec_trig(track_t *t, voice_t *v)
{
    uint32_t k = (uint32_t)(t - trk) % NTRK;
    trec_tp_t *tp = &trec_tp[k];
    tp->on = 0;
    tp->pend = 0;
    tp->v = v;
    tp->p16 = v->pitch16;
    if (trec[k].st == 2u) {                        /* recording (the old take wiped): played once it's done */
        tp_speed(t, tp);
        tp->el = 0;
        tp->pend = 1;
        return;
    }
    trec_tp_go(t, tp, smp_tab[k].w, smp_tab[k].len, 0);
}
/* track k records now: a take on its way, or the LOOPER overdubbing (the screen's red dot) */
static inline int trec_rec_now(uint32_t k)
{
    return trec[k].st == 2u || (trec[k].loop && trec_set[k].arm && song.playing);
}

/* the pages (trec_ui.c, after gb_store.c: ui_gb.c and ui_input.c call them) */
static void trec_knob(uint32_t rec_in, uint32_t slot, int32_t steps);
static void trec_clear(void);
static void trec_poll(void);
static void trec_view(uint32_t rec_in, char *title, char lab[][8], char val[][12], int16_t *ratio, uint8_t *mask,
                      uint8_t *wv, uint8_t *wa, uint8_t *wb, uint16_t *wkey);

/* SPDX-License-Identifier: GPL-3.0-only */
/* SIMPLE, step slides (Blair's design): hold two steps (one page, ui_input.c sl_*) and a menu sets a slide from the first
 * to the second over the steps between them: TYPE (PITCH PARAM BOTH), CURVE (-64 fast then slow .. 0 straight .. 63 slow
 * then fast), MODE (SMOOTH, or STEP: a staircase). A note between the two is a stop on the way (first > it > second): up
 * and down modulation. So are a lock's values: a step in the slide holding a lock is a stop for that parameter. A
 * parameter not locked at an end has the sound's own value there.
 * PITCH: the notes are legato, one envelope all the way, the last note held a whole step, and each voice travels to its note of the next stop (notes in
 * pitch order, a chord's first to the next one's first). Different sizes: several voices converge on one note and all
 * but one fade out on the way, or one voice spreads into several, the new ones fading in. Drums, and the ARP on: every
 * trig still hits. PARAM alone: notes play as usual, the values glide.
 * Timing is by steps, so the tempo, SPD and swing stretch it. A step that fails its CHANCE doesn't start a slide. Clearing
 * either end removes it. Kept in the last jam, not in sounds or kits (patterns never are) */
#define SL_PITCH 1u
#define SL_PARAM 2u
#define SL_STEP 4u
#define SL_NP 8u                                         /* parameters gliding at once, a track */
typedef struct { uint8_t to, ty; int8_t curve; } slide_t;   /* a slide starts on this step: to = its last step + 1 (0 none) */
static slide_t slide[NTRK][NSTEP];
typedef struct {
    uint8_t on, a, b, ty, pa, pb, np;                    /* the slide playing; pa..pb the notes' stretch now (pb 0: none) */
    int8_t curve;
    uint8_t id[SL_NP], sa[SL_NP], sb[SL_NP];             /* each parameter's stretch now, its values at the two ends */
    int8_t vf[SL_NP], vt[SL_NP];
} sl_rt_t;
static sl_rt_t sl_rt[NTRK];

/* a new slide takes the place of any it overlaps; two can share an end (a chain) */
static void sl_set(uint32_t k, uint32_t a, uint32_t b, uint32_t ty, int32_t curve)
{
    uint32_t i;
    for (i = 0; i < NSTEP; i++) {
        uint32_t to = slide[k][i].to;
        if (to && i < b && to - 1u > a)
            slide[k][i].to = 0;
    }
    slide[k][a].ty = (uint8_t)ty;
    slide[k][a].curve = (int8_t)curve;
    slide[k][a].to = (uint8_t)(b + 1u);
}
static void sl_forget(uint32_t k, uint32_t step)         /* the step's gone: the slides from it and to it */
{
    uint32_t i;
    slide[k][step].to = 0;
    for (i = 0; i < step; i++)
        if (slide[k][i].to == step + 1u)
            slide[k][i].to = 0;
}
/* x 0..32768 through the curve: powers of x blended, up to x^16; the left side is the same turned over */
static int32_t sl_curve(int32_t x, int32_t c)
{
    int32_t lv[5];
    uint32_t i, u;
    if (c < 0)
        return 32768 - sl_curve(32768 - x, -c);
    if (!c)
        return x;
    lv[0] = x;
    for (i = 1; i < 5u; i++)
        lv[i] = (lv[i - 1u] * lv[i - 1u]) >> 15;
    u = (uint32_t)c * 4u;
    i = u >> 6;
    return i >= 4u ? lv[4] : lv[i] + (((lv[i + 1u] - lv[i]) * (int32_t)(u & 63u)) >> 6);
}
/* how far along stretch sa..sb the playhead is (idx, pos15 of the step): 0..32768 */
static int32_t sl_phase(const sl_rt_t *rt, uint32_t sa, uint32_t sb, uint32_t idx, uint32_t pos15)
{
    uint32_t n = sb - sa, x;
    if (idx < sa)
        x = 0;
    else if (idx >= sb)
        x = 32768u;
    else
        x = ((idx - sa) * 32768u + pos15) / n;
    if (rt->ty & SL_STEP) {                              /* a stair a step (short ones: 4 a step) */
        uint32_t st = n < 4u ? n * 4u : n;
        x = (x * st >> 15) * 32768u / st;
    }
    return sl_curve((int32_t)x, rt->curve);
}
/* the slide's voices end where they were going (arrived), the faded ones go */
static void sl_finish(track_t *t)
{
    uint32_t i;
    for (i = 0; i < NVOICE; i++) {
        voice_t *v = &t->v[i];
        if (!v->sl)
            continue;
        if (v->sl == 2u)
            voice_kill(v);
        v->pitch_cur = v->pitch16;
        v->sl = 0;
    }
}
static int sl_can(const track_t *t)                      /* its notes can be legato: not a drum, not through the ARP */
{
    return trk_machine(t)->drum != 1u && !arp_am(t);
}
/* the voices sounding set off for step `to`'s notes */
static void sl_aim(track_t *t, const step_t *to)
{
    voice_t *src[NVOICE], *f;
    uint8_t nn[4];
    uint32_t i, j, n = 0, m = to->n < 4u ? to->n : 4u, drum = trk_machine(t)->drum == 1u, one = drum || trk_vmode(t) != V_POLY;
    int32_t last = -1;
    for (i = 0; i < m; i++) {                            /* the notes low to high */
        for (j = i; j > 0u && nn[j - 1u] > to->note[i]; j--)
            nn[j] = nn[j - 1u];
        nn[j] = to->note[i];
    }
    if (one)
        m = 1u;
    for (i = 0; i < NVOICE; i++) {                       /* the voices low to high */
        voice_t *v = &t->v[i];
        if (!v->active || !v->stage || v->stage == 4u || (!v->gate && !drum))
            continue;
        for (j = n; j > 0u && src[j - 1u]->pitch_cur > v->pitch_cur; j--)
            src[j] = src[j - 1u];
        src[j] = v;
        n++;
    }
    if (!n || !m)
        return;
    if (n >= m) {                                        /* the same, or converging: all but one of each note fade out */
        for (i = 0; i < n; i++) {
            voice_t *v = src[i];
            j = i * m / n;
            v->sfrom = v->pitch_cur;
            v->pitch16 = voice_pitch16(t, nn[j]);
            v->note = nn[j];
            v->sl = (int32_t)j == last ? 2u : 1u;
            last = (int32_t)j;
        }
    } else {                                             /* spreading: a voice's first note is its own, the rest copies */
        for (j = 0; j < m; j++) {
            voice_t *v;
            i = j * n / m;
            v = src[i];
            if ((int32_t)i == last) {
                uint32_t q, np = trk_nvoice(t);
                for (q = 0; q < np && t->v[q].active; q++)
                    ;
                if (q == np || voices_busy() >= NVOICE_ALL)
                    continue;                            /* (no room: this note stays out) */
                f = &t->v[q];
                *f = *v;
                f->age = ++vage;
                f->env_out = 0;                          /* (from silence) */
                f->sl = 3u;
                v = f;
            } else {
                v->sfrom = v->pitch_cur;
                v->sl = 1u;
            }
            v->pitch16 = voice_pitch16(t, nn[j]);
            v->note = nn[j];
            last = (int32_t)i;
        }
    }
    if (t->seq_n) {                                      /* the note-offs to come are the new notes' */
        for (i = 0; i < m; i++)
            t->seq_notes[i] = nn[i];
        t->seq_n = (uint8_t)m;
    }
    if (one && !drum) {
        t->mono_stack[0] = nn[0];
        t->nmono = 1u;
        t->mono_note = nn[0];
    }
}
/* step idx: the notes' next stretch, to the next step with notes in the slide */
static void sl_seg(track_t *t, sl_rt_t *rt, uint32_t idx)
{
    uint32_t q;
    sl_finish(t);
    rt->pb = 0;
    for (q = idx + 1u; q <= rt->b; q++)
        if (t->step[q].time == ST_NOTE && t->step[q].n) {
            rt->pa = (uint8_t)idx;
            rt->pb = (uint8_t)q;
            sl_aim(t, &t->step[q]);
            return;
        }
}
/* step idx: each parameter's stretch: the lock (or the slide's start, the sound's own value) before it to the next */
static void sl_params(track_t *t, sl_rt_t *rt, uint32_t idx)
{
    uint32_t k = trk_index(t), i, j, np = 0;
    rt->np = 0;
    if (!(rt->ty & SL_PARAM))
        return;
    for (i = 0; i < motion.count && np < SL_NP; i++) {
        const motion_event_t *e = &motion.event[i];
        uint32_t id = MOTION_ID(e), s = e->place & 63u;
        if ((e->place >> 6) != k || !(e->param & MOTION_LOCK) || s < rt->a || s > rt->b || id == P_E7 ||
            motion_desc(t, id)->fmt == F_ENUM)           /* (the ones that switch stay plain locks) */
            continue;
        for (j = 0; j < np && rt->id[j] != id; j++)
            ;
        if (j == np)
            rt->id[np++] = (uint8_t)id;
    }
    for (j = 0; j < np; j++) {
        uint32_t id = rt->id[j], sa = rt->a, sb = rt->b;
        int32_t base = motion_base_value(t, id), vf = base, vt = base;
        for (i = 0; i < motion.count; i++) {
            const motion_event_t *e = &motion.event[i];
            uint32_t s = e->place & 63u;
            if ((e->place >> 6) != k || e->param != (id | MOTION_LOCK) || s < rt->a || s > rt->b)
                continue;
            if (s <= idx && s >= sa) {
                sa = s;
                vf = e->value;
            } else if (s > idx && s <= sb) {
                sb = s;
                vt = e->value;
            }
        }
        rt->sa[j] = (uint8_t)sa;
        rt->sb[j] = (uint8_t)sb;
        rt->vf[j] = (int8_t)vf;
        rt->vt[j] = (int8_t)vt;
    }
    rt->np = (uint8_t)np;
}
static void sl_new(sl_rt_t *rt, const slide_t *sd, uint32_t idx)
{
    rt->on = 1;
    rt->a = (uint8_t)idx;
    rt->b = (uint8_t)(sd->to - 1u);
    rt->ty = sd->ty;
    rt->curve = sd->curve;
    rt->pb = 0;
    rt->np = 0;
}
static void sl_arrive(track_t *t, sl_rt_t *rt)           /* the parameters at their last stop's values */
{
    uint32_t j;
    for (j = 0; j < rt->np; j++)
        *motion_vp(t, rt->id[j]) = rt->vt[j];
    rt->np = 0;
    rt->on = 0;
}
/* a slide cut short (stop, ACID, a jump, LEN made shorter): each voice stays where it has got to */
static void sl_stop(track_t *t)
{
    uint32_t i;
    sl_rt[trk_index(t)].on = 0;
    for (i = 0; i < NVOICE; i++) {
        voice_t *v = &t->v[i];
        if (!v->sl)
            continue;
        if (v->sl == 2u)
            voice_kill(v);
        v->pitch16 = v->pitch_cur;
        v->sl = 0;
    }
}
/* step idx plays (seq_play: in place of seq_step). Nothing to do with a slide: seq_step, as ever */
static __attribute__((noinline)) void sl_play(track_t *t, const step_t *s, uint32_t idx, uint32_t period, uint32_t skip)
{
    uint32_t k = trk_index(t), len = t->p[P_SLEN] ? (uint32_t)t->p[P_SLEN] : 1u, ok = (skip & SEQ_ROLLED) != 0u, in, start, end;
    sl_rt_t *rt = &sl_rt[k];
    const slide_t *sd = &slide[k][idx];
    if (!rt->on && !sd->to) {
        seq_step(t, s, idx, period, skip);
        return;
    }
    in = rt->on && idx > rt->a && idx <= rt->b;
    if (rt->on && !in) {
        sl_stop(t);
        t->seq_hold = 0;
    }
    start = ok && sd->to > idx + 1u && sd->to <= len && (s->time == ST_LOCK || (s->time == ST_NOTE && s->n));
    end = in && idx == rt->b;
    if (in && (rt->ty & SL_PITCH) && sl_can(t) && t->seq_n) {   /* legato: no new note */
        if (end) {
            sl_finish(t);
            sl_arrive(t, rt);
            if (start)
                sl_new(rt, sd, idx);
            if (rt->on && (rt->ty & SL_PITCH)) {
                sl_seg(t, rt, idx);
            } else {
                t->seq_hold = 0;
                t->seq_off = step_samples(t, period, idx);   /* (the landing note: a whole step, or GATE's ~40 ms hides it) */
            }
        } else if (idx == rt->pb) {
            sl_seg(t, rt, idx);
        }
        if (rt->on)
            sl_params(t, rt, idx);
        return;
    }
    if (end)
        sl_arrive(t, rt);
    seq_step(t, s, idx, period, skip);
    if (in && !end && (rt->ty & SL_PITCH) && (idx == rt->pb || !rt->pb))
        sl_seg(t, rt, idx);                              /* (a drum's hit, or the notes had ended) */
    if (start && !rt->on) {
        sl_new(rt, sd, idx);
        if (rt->ty & SL_PITCH) {
            sl_seg(t, rt, idx);
            if (sl_can(t) && t->seq_n)
                t->seq_hold = 1;                         /* (held to the slide's end, not the step's GATE) */
        }
    }
    if (rt->on)
        sl_params(t, rt, idx);
}
/* every block while one plays (seq_tick): how far along it is. By steps, so tempo and swing stretch it */
static __attribute__((noinline)) void sl_block(track_t *t, uint32_t k, uint32_t period, uint32_t ac)
{
    sl_rt_t *rt = &sl_rt[k];
    uint32_t idx = t->seq_idx, cur = step_samples(t, period, idx), pos, j;
    if (ac) {
        sl_stop(t);
        return;
    }
    if (idx > rt->b)                                     /* (its first step played early, by its NUDGE, at the loop's end) */
        idx = rt->a;
    pos = t->seq_pos >= cur ? 32767u : ((t->seq_pos >> 4) * 32768u) / ((cur >> 4) | 1u);
    sl_ph[k] = rt->pb ? sl_phase(rt, rt->pa, rt->pb, idx, pos) : 32768;
    for (j = 0; j < rt->np; j++) {
        uint32_t id = rt->id[j];
        int32_t ph = sl_phase(rt, rt->sa[j], rt->sb[j], idx, pos);
        *motion_vp(t, id) = (int16_t)(rt->vf[j] + (((rt->vt[j] - rt->vf[j]) * ph + 16384) >> 15));
        motion_active[k][id / 32u] |= 1u << (id % 32u);
        motion_locked[k][id / 32u] |= 1u << (id % 32u);
    }
}

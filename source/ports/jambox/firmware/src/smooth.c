/* SPDX-License-Identifier: GPL-3.0-only */
/* Knob smoothing (2026-11-12, Blair: every knob that isn't stepped by design "smooth as butter, like a pot on the
 * actual circuitry of a synth"). The knobs are detents, 128 values: a slow turn of CUT was a staircase you could hear.
 *
 * Every knob that moves smoothly reaches the sound through here, as a Q8 value (256 in-betweens a step: smq / smg):
 *   - its stored value glides to each new one at the pace it is being turned: the time between the last two turns
 *     of that knob (sm_turn stamps them, before the value is written), so a steady turn is one straight line, a slow
 *     turn as smooth as a fast one. The first click of a turn glides SM_FIRST_MS; then 3/4 of the averaged pace (SM_MIN_MS .. SM_LONG_MS). A change no hand made (a lock,
 *     a sound loaded, a reset) only de-clicks: SM_SNAP_MS. Macros, faders, the crossfader and MIDI CCs stamp too
 *   - the LFOs and the matrix (mod.c) add their offset on top at Q8, not rounded to whole steps
 * The engines read smq() where they used t->p (the integer stays for the screen, saves, locks: nothing else changes).
 * Per block (fx.c mix_part: sm_track after mod_begin; the globals in mod.c mod_g_begin) */
#define SMOOTH_ON 1
enum { SK_CUT, SK_RES, SK_DRV, SK_KTR, SK_FEAMT, SK_FESUS, SK_SUS, SK_LEVEL, SK_PAN, SK_DLY, SK_REV,
       SK_E0, SK_E1, SK_E2, SK_E3, SK_E4, SK_E5, SK_E6, SK_FB, SK_DT, SK_LN0, SK_LN1,
       SK_ATK, SK_DEC, SK_REL, SK_FEATK, SK_FEDEC, SK_FEREL, SK_CATK, SK_CDEC, SK_DATK, SK_DDEC,   /* the envelopes' times */
       SK_LRATE, SK_A1, SK_A2, SK_A3, SK_A4, SK_NT };                                             /* the LFO's speed, the 4 slots' AMTs */
static const uint8_t SM_TID[SK_NT] = {P_FLT_CUT, P_FLT_RES, P_FLT_DRV, P_FLT_KTR, P_FE_AMT, P_FE_SUS, P_SUS, P_LEVEL,
    P_PAN, P_DLY, P_REV, P_E0, P_E0 + 1, P_E0 + 2, P_E0 + 3, P_E0 + 4, P_E0 + 5, P_E0 + 6, P_FM3_LEVEL, P_FM4_LEVEL,
    P_LN0, P_LN1, P_ATK, P_DEC, P_REL, P_FE_ATK, P_FE_DEC, P_FE_REL, P_FM3_ATK, P_FM3_DEC, P_FM4_ATK, P_FM4_DEC,
    P_LRATE, P_M1AMT, P_M2AMT, P_M3AMT, P_M4AMT};
_Static_assert(P_M2AMT == P_M1AMT + 3 && P_M3AMT == P_M1AMT + 6 && P_M4AMT == P_M1AMT + 9, "SK_A1..: the slots' AMTs 3 apart");
/* the globals: the DJ filter, the comb, the delay, the reverb, AMP / PAN, and ALL's knobs (not its PITCH: semitones) */
enum { SKG_DJ, SKG_CMIX, SKG_CTUNE, SKG_CFDBK, SKG_CDAMP, SKG_DTIME, SKG_DFDBK, SKG_DTONE, SKG_RCLK, SKG_RFBK, SKG_RFLT,
       SKG_SMEAR, SKG_AMP, SKG_PAN, SKG_F1FREE, SKG_F2FREE, SKG_A1, SKG_N = SKG_A1 + (G_A_REV - G_A_TYP + 1) };
static const uint8_t SM_GID[SKG_N] = {G_MIDI, G_SYNC, G_CRATE, G_CDEPTH, G_INFO, G_DTIME, G_DFDBK, G_DCOLOR, G_RSIZE,
    G_RDAMP, G_RTYPE, G_DMIX, G_AMP, G_PAN, G_F1FREE, G_F2FREE, G_A_TYP, G_A_TYP + 1, G_A_TYP + 2, G_A_TYP + 3, G_A_TYP + 4, G_A_TYP + 5,
    G_A_TYP + 6, G_A_TYP + 7, G_A_TYP + 8, G_A_TYP + 9, G_A_TYP + 10, G_A_TYP + 11, G_A_TYP + 12, G_A_TYP + 13};
_Static_assert(G_A_TYP + 13 == G_A_REV, "SM_GID: ALL's knobs");

#define SM_SNAP_MS 3u      /* a change no hand made: just no click */
#define SM_FIRST_MS 75u    /* the first click of a turn (we can't know when the next comes: a pot's pace) */
#define SM_TURN_MS 2000u   /* clicks further apart than this are a new turn (Blair turns the Elite's knobs very slowly) */
#define SM_HAND_MS 300u    /* a change this soon after a stamp is a hand's */
#define SM_MIN_MS 40u      /* a hand's glide at the least: clicks arrive unevenly (the main loop draws between reads) */
#define SM_LONG_MS 300u    /* .. at the most (Blair 2026-11-14: 3 s lagged far behind the knob, 400 ms nearly right) */
typedef struct {
    int32_t v;             /* the stored value gliding (Q16); its step a block: what's left / the blocks left (no stored
                            * step: 4 bytes a slot, the Elite's RAM) */
    int32_t out;           /* what the sound takes: v + the modulation, Q8 */
    uint16_t left;         /* the glide's blocks left */
    uint16_t tms, gap;     /* sm_turn: when it was last turned (ms, low 16 bits), the turn's pace: the time between its
                            * clicks, averaged (0: its first click) */
    int16_t base;          /* the stored value the glide goes to */
} sm_s;
static sm_s sm_tr[NTRK][SK_NT], sm_gl[SKG_N];
static uint8_t sm_tslot[P_COUNT], sm_gslot[G_COUNT];  /* id -> its slot + 1 (0: not smoothed) */
static uint8_t sm_ready;                                /* (the first block: every value at once) */

static void sm_init(void)
{
    uint32_t i;
    for (i = 0; i < SK_NT; i++)
        sm_tslot[SM_TID[i]] = (uint8_t)(i + 1u);
    for (i = 0; i < SKG_N; i++)
        sm_gslot[SM_GID[i]] = (uint8_t)(i + 1u);
}

/* the sound's value of slot s of track t / global slot s, Q8 */
static inline __attribute__((always_inline)) int32_t smq(const track_t *t, uint32_t s) { return sm_tr[(uint32_t)(t - trk) % NTRK][s].out; }
static inline __attribute__((always_inline)) int32_t smg(uint32_t s) { return sm_gl[s].out; }
/* a table of 128 read at a Q8 index, in between linearly */
#define SM_TAB(tab, q8) ({ int32_t q_ = clamp((q8), 0, 127 << 8), i_ = q_ >> 8; \
    (int32_t)(tab)[i_] + (i_ < 127 ? ((((int32_t)(tab)[i_ + 1] - (int32_t)(tab)[i_]) * (q_ & 255)) >> 8) : 0); })

/* .. a table of 32-bit values (ENV_LIN, LFO_INC) */
#define SM_TABL(tab, q8) ({ int32_t q_ = clamp((q8), 0, 127 << 8), i_ = q_ >> 8; \
    (uint32_t)(tab)[i_] + (i_ < 127 ? (uint32_t)(((int64_t)((int64_t)(tab)[i_ + 1] - (int64_t)(tab)[i_]) * (q_ & 255)) >> 8) : 0u); })

/* UI: the value at vp is about to be changed by a hand (a knob, a fader, a macro, a CC): its pace */
static void sm_turn(const int16_t *vp)
{
    sm_s *s = 0;
    uint32_t k, now;
    for (k = 0; k < NTRK && !s; k++)
        if (vp >= trk[k].p && vp < trk[k].p + P_COUNT && sm_tslot[vp - trk[k].p])
            s = &sm_tr[k][sm_tslot[vp - trk[k].p] - 1u];
    if (!s && vp >= song.g && vp < song.g + G_COUNT && sm_gslot[vp - song.g])
        s = &sm_gl[sm_gslot[vp - song.g] - 1u];
    if (!s)
        return;
    now = fm1_ms & 0xFFFFu;
    k = (now - s->tms) & 0xFFFFu;                       /* (k: the time since the last click) */
    s->gap = (uint16_t)(k > SM_TURN_MS ? 0u : s->gap ? (s->gap + k) / 2u : k);
    s->tms = (uint16_t)now;
}

/* audio ISR: one value's block. base: its stored value, off: the modulation's (Q8), nb: the block's samples */
static inline __attribute__((always_inline)) void sm_step(sm_s *s, int32_t base, int32_t off, uint32_t nb)
{
    if (base != s->base || !sm_ready) {
        uint32_t since = (fm1_ms - s->tms) & 0xFFFFu, ms = s->gap, bl;
        s->base = (int16_t)base;
        if (!sm_ready)
            ms = 0u;
        else if (since > SM_HAND_MS)                   /* no hand: a lock, a load */
            ms = SM_SNAP_MS;
        else if (!ms)
            ms = SM_FIRST_MS;
        else {                                          /* 3/4 of a click (1.5 lagged behind the knob, 1 nearly right: Blair 2026-11-14) */
            ms = ms * 3u / 4u;
            ms = ms < SM_MIN_MS ? SM_MIN_MS : ms > SM_LONG_MS ? SM_LONG_MS : ms;
        }
        bl = ms * (uint32_t)(FS / 1000) / nb;
        if (!bl) {
            s->v = base * 65536;
            s->left = 0;
        } else {
            s->left = (uint16_t)(bl > 0xFFFFu ? 0xFFFFu : bl);
        }
    }
    if (s->left) {
        s->v += ((base * 65536) - s->v) / (int32_t)s->left;   /* (a straight line: the same step each block) */
        if (!--s->left)
            s->v = base * 65536;
    }
    s->out = ((s->v + 128) >> 8) + off;
}

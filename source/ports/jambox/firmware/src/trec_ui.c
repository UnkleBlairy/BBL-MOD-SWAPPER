/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2026 Blair (blairysbeatlab), BBL JAM BOX */
/* teensy-fm1 fork only: the REC and REC IN pages of the track recording (trec.c has the rest) */
/* ------------------------------------------------------------- the pages --- */

/* ARM on: the pool (PSRAM) and, for LINE IN, the codec asked for. 0: no pool */
static int trec_arm(uint32_t k)
{
    if (!trec_pool) {
        uint32_t n = 0;
        trec_pool = teensy_pool(&n);
        trec_pool_n = trec_pool ? n : 0u;
    }
    if (!trec_pool) {
        ui_message("NO PSRAM");
        return 0;
    }
    return 1;
}
static uint32_t trec_clr_t;                        /* CLEAR's first turn (fm1_ms | 1), 0: none */
static void trec_knob(uint32_t rec_in, uint32_t slot, int32_t steps)
{
    uint32_t k = song.sel % NTRK;
    trec_set_t *s = &trec_set[k];
    int32_t d = steps > 0 ? 1 : -1;
    switch (rec_in * 4u + slot) {
    case 0:
        if (d > 0 && !s->arm && trec_arm(k))
            s->arm = 1;
        else if (d < 0)
            s->arm = 0;
        break;
    case 1: s->start = (uint8_t)clamp((int32_t)s->start + d, 0, TS_N - 1); break;
    case 2: s->len = (uint8_t)clamp((int32_t)s->len + d, 0, TL_N - 1); break;
    case 3: s->mode = (uint8_t)clamp((int32_t)s->mode + d, 0, TM_N - 1); break;
    case 4:
        s->src = TSRC_TRACK;                       /* (TRACK only for now: MASTER in phase 2; the line in is INPUT) */
        break;
    case 5:
        s->gain = (int8_t)clamp((int32_t)s->gain + accel(EN_K1 + slot, steps, TG_MAX - TG_MIN), TG_MIN, TG_MAX);
        trec_gain_set(k);
        break;
    case 6:                                        /* DUB % (stored as 100 - it) */
        s->dubx = (uint8_t)(100 - clamp(100 - (int32_t)s->dubx + accel(EN_K1 + slot, steps, 100), 0, 100));
        break;
    case 7:                                        /* CLEAR (REC IN's 4th dial; was EDIT twice, which trapped EDIT on the
                                                    * REC pages, Blair): right once = SURE?, right again after a pause
                                                    * (not the same spin) within 3 s = cleared; left = no */
        if (d < 0 || !smp_tab[k].len)
            trec_clr_t = 0;
        else if (trec_clr_t && (int32_t)(fm1_ms - trec_clr_t) > 300 && (int32_t)(fm1_ms - trec_clr_t) < 3000) {
            trec_clr_t = 0;
            trec_clear();
        } else if (!trec_clr_t || (int32_t)(fm1_ms - trec_clr_t) >= 3000) {
            trec_clr_t = fm1_ms | 1u;
            ui_message("TURN AGAIN = CLEAR");
        }
        break;
    default: return;
    }
}
/* CLEAR turned twice (REC IN): the track's buffer (slot Rn) emptied */
static void trec_clear(void)
{
    uint32_t k = song.sel % NTRK;
    fm1_irq_off();
    smp_tab[k].len = 0;
    smp_tab[k].w = 0;
    trec[k].st = 0;
    trec[k].loop = 0;
    trec_tp[k].on = 0;
    memset(trec_pk[k], 0, GB_WAVE);
    trec_set[k].arm = 0;
    fm1_irq_on();
    smp_pk_of = 0xFFFFFFFFu;
    ui_message("BUFFER CLEARED");
}
/* every frame: the takes done since */
static void trec_poll(void)
{
    static char m[16];
    uint32_t d = trec_done, f = trec_full, k, in = 0;
    static uint8_t in_was;
    for (k = 0; k < NTRK; k++)                     /* INPUT on a track: the codec started (again, if it is new) */
        if (eng_idx(trk[k].eng_req) == ENGI_MONO && trk[k].p[P_E7] == 1)
            in |= 1u << k;
    if (in && ((in & ~in_was) || teensy_line_in_state() == 2u))
        teensy_line_in_arm();
    in_was = (uint8_t)in;
    if (!d && !f)
        return;
    trec_done = trec_full = 0;
    smp_pk_of = 0xFFFFFFFFu;                       /* (the waveform: the new take's) */
    for (k = 0; k < NTRK; k++) {
        if ((f >> k) & 1u) {
            ui_message("POOL FULL");
            return;
        }
        if ((d >> k) & 1u) {
            str_cpy(m, "R1 RECORDED", sizeof m);
            m[1] = (char)('1' + k);
            ui_message(m);
            return;
        }
    }
}
/* the dials: REC (rec_in 0) ARM START LEN MODE, REC IN (1) SRC GAIN; the title says what it's doing, the buffer's
 * waveform under them (wv, wkey: ui_gb.c's sampler waveform) */
static void trec_view(uint32_t rec_in, char *title, char lab[][8], char val[][12], int16_t *ratio, uint8_t *mask,
                      uint8_t *wv, uint8_t *wa, uint8_t *wb, uint16_t *wkey)
{
    static const char *const LB[8] = {"ARM", "START", "LEN", "MODE", "SRC", "GAIN", "DUB", "CLEAR"};
    static const char *const MN[TM_N] = {"STD", "LOOPER"};   /* (STANDARD: too wide for a dial) */
    static const char *const SN[TS_N] = {"REC", "PLAY", "SYNC"};
    static const char *const RN[TSRC_N] = {"LINE", "TRACK"};   /* (LINE IN: too wide) */
    uint32_t k = song.sel % NTRK, i;
    const trec_set_t *s = &trec_set[k];
    const trec_t *r = &trec[k];
    char nb[8];
    str_cpy(title, "REC R1", 12);
    title[5] = (char)('1' + k);
    if (rec_in)
        str_cpy(title, "REC IN", 12);
    else if (r->st == 2u) {                              /* "R1 45%" */
        title[0] = 'R';
        title[1] = (char)('1' + k);
        title[2] = ' ';
        fmt_int(nb, (int32_t)(r->skip ? 0u : (uint32_t)((uint64_t)r->pos * 100u / (r->want ? r->want : 1u))));
        str_cpy(title + 3, nb, 9);
        str_cpy(title + str_len(title), "%", 12 - str_len(title));
    } else if (r->st == 1u) {
        str_cpy(title, "R1 WAIT", 12);
        title[1] = (char)('1' + k);
    } else if (r->loop) {                                /* LOOPER: "R1 DUB" overdubbing, "R1 LOOP" just playing */
        str_cpy(title, trec_rec_now(k) ? "R1 DUB" : "R1 LOOP", 12);
        title[1] = (char)('1' + k);
    }
    if (trk_in(&trk[k]) && teensy_line_in_state() == 0u)
        str_cpy(title, "NO LINE IN", 12);
    for (i = 0; i < 4u; i++) {
        uint32_t j = rec_in * 4u + i;
        char *o = val[i];
        int32_t q = 0;
        str_cpy(lab[i], LB[j], 8);
        *mask |= (uint8_t)(1u << i);
        switch (j) {
        case 0: str_cpy(o, s->arm ? "ON" : "OFF", 12); q = s->arm ? 1000 : 0; break;
        case 1: str_cpy(o, SN[s->start % TS_N], 12); q = s->start * 1000 / (TS_N - 1); break;
        case 2: str_cpy(o, TL_NAME[s->len % TL_N], 12); q = s->len * 1000 / (TL_N - 1); break;
        case 3: str_cpy(o, MN[s->mode % TM_N], 12); q = s->mode * 1000 / (TM_N - 1); break;
        case 4: str_cpy(o, trk_in(&trk[k]) ? "INPUT" : RN[s->src % TSRC_N], 12); q = 1000; break;   /* (INPUT: its own sound) */
        case 7:
            if (trec_clr_t && (int32_t)(fm1_ms - trec_clr_t) >= 3000)
                trec_clr_t = 0;
            str_cpy(o, !smp_tab[k].len ? "EMPTY" : trec_clr_t ? "SURE?" : "R1", 12);
            if (smp_tab[k].len && !trec_clr_t)
                o[1] = (char)('1' + k);
            q = trec_clr_t ? 1000 : 0;
            break;
        case 6:
            fmt_int(o, 100 - (int32_t)s->dubx);
            str_cpy(o + str_len(o), "%", 12 - str_len(o));
            q = (100 - s->dubx) * 10;
            break;
        default:
            if (s->gain > 0)
                o[0] = '+';
            fmt_int(o + (s->gain > 0), s->gain);
            str_cpy(o + str_len(o), "DB", 12 - str_len(o));
            q = (s->gain - TG_MIN) * 1000 / (TG_MAX - TG_MIN);
            break;
        }
        ratio[i] = (int16_t)q;
    }
    *wv = smp_peaks(k) ? 2u : 1u;                   /* the buffer, or NO SAMPLE */
    *wa = 0;
    *wb = GB_WAVE - 1u;
    *wkey = (uint16_t)(k | smp_gen << 6);
}

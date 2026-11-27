/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments */
/* MASTER (#137): the pot read before the audio starts, so the first block plays at its level (it was 2048, about
 * 70 % of the knob, until the main loop's first read after the splash, then a 1/8-per-frame glide from mid-scale:
 * the keys, live from the audio ISR at once, sounded loud under the splash with the knob turned down).
 * master_knob: the pot x16, smoothed per UI frame (master_poll) */
static int32_t master_knob = 512 * 16;
#if BOARD_SMK37
static void master_poll(void)                           /* the Elite: K8, no noise to smooth; a ~8 ms glide (no clicks), */
{                                                       /* by the clock, not the frames (it lagged a busy screen) */
    static uint32_t t;
    int32_t a = fm1_adc_read(FM1_ADC_MASTER);
    uint32_t n = fm1_ms - t;
    t = fm1_ms;
    if (a < 0)
        return;
    for (n = n > 32u ? 32u : n; n; n--)
        master_knob += (a * 16 - master_knob) / 8;
    song.master_q12 = master_of_pot((uint32_t)(master_knob / 16));
}
#else
static void master_poll(void)
{
    int32_t a = fm1_adc_read(FM1_ADC_MASTER);
    if (a >= 0) {
        master_knob += (a * 16 - master_knob) / 8;
        song.master_q12 = master_of_pot((uint32_t)(master_knob / 16));
    }
}
#endif
/* power-on (main.c), before audio_init: the knob as it stands, no glide */
static void master_boot(void)
{
    uint32_t i;
    (void)fm1_adc_read(FM1_ADC_MASTER);                 /* (the first conversion after the pin setup: dropped) */
    for (i = 0; i < 3u; i++) {
        int32_t a = fm1_adc_read(FM1_ADC_MASTER);
        if (a >= 0) {
            master_knob = a * 16;
            song.master_q12 = master_of_pot((uint32_t)a);
            return;
        }
    }                                                   /* (no reading: felucca_init's level, the loop glides on) */
}

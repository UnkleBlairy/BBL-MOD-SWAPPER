/* SPDX-License-Identifier: GPL-3.0-only */
/* teensy-fm1: X0X (charlesvestal/fm1-x0x 512c0da) on the Teensy 4.1. Derived from X0X's browser build,
 * web/emu/x0x_web.c: the host simulator (host/x0x_host.c: the whole app, UI, sequencer, engines, against a simulated
 * FM-1) with the device clock driven by the audio the Teensy asks for. The 909 / 808 render that runs on the FM-1's
 * second core runs inline here (the host's plat_cpu2_*: one core). The changes are marked "teensy-fm1:"; the one in
 * host/x0x_host.c is X0X_LCD_PUT (lcd_fill / lcd_blit mark the changed pixels for the display link).
 * What the Teensy side (src/main.rs) uses is the same set of exports as the Felucca ports: web_render, web_boot,
 * teensy_fb / teensy_spans / teensy_redraw (the screen), teensy_raw_input / teensy_raw_enc / teensy_raw_adc (the
 * FM-1's panel, as matrix ids), teensy_leds (as the matrix columns). Saves stay in RAM for now (the host's store[]). */
#include <stdint.h>
#include <string.h>

#define X0X_WEB 1
#ifndef X0X_VERSION
#define X0X_VERSION FELUCCA_VERSION
#endif

/* teensy-fm1: the screen, with the rows' changed spans (lo > hi: clean) */
static uint8_t dirty_lo[240], dirty_hi[240];
static uint32_t teensy_px;
static void teensy_mark(uint32_t i)
{
    uint32_t y = i / 240u, x = i % 240u;
    if (dirty_lo[y] > dirty_hi[y]) { dirty_lo[y] = dirty_hi[y] = (uint8_t)x; return; }
    if (x < dirty_lo[y]) dirty_lo[y] = (uint8_t)x;
    if (x > dirty_hi[y]) dirty_hi[y] = (uint8_t)x;
}
#define X0X_LCD_PUT(i, v) do { uint32_t i_ = (i); uint16_t v_ = (v); teensy_px++; \
    if (fb[i_] != v_) { fb[i_] = v_; teensy_mark(i_); } } while (0)

/* teensy-fm1: the host's timing (its own test main, never on the Teensy) wants clock_gettime */
#include <time.h>
#ifndef CLOCK_MONOTONIC
#define CLOCK_MONOTONIC 1
static int clock_gettime(int c, struct timespec *t) { (void)c; t->tv_sec = 0; t->tv_nsec = 0; return 0; }
#endif

static void web_audio(const int32_t *blk, uint32_t n);
static void web_said(const uint8_t *p, uint32_t n) { (void)p; (void)n; }   /* no screen reader */
#include "../../host/x0x_host.c"

#undef __attribute__
#define EXPORT __attribute__((used, visibility("default")))

/* output ring, stereo float, filled by run_ms, drained by web_render */
#define RING 4096u
static float ring[RING * 2];
static uint32_t ring_w, ring_r;
static void web_audio(const int32_t *blk, uint32_t n)
{
    uint32_t k;
    for (k = 0; k < n; k++) {
        uint32_t i = (ring_w++ % RING) * 2u;
        ring[i] = (float)blk[2 * k] / 8388608.0f;
        ring[i + 1] = (float)blk[2 * k + 1] / 8388608.0f;
    }
}

static float out_l[1024], out_r[1024];
EXPORT float *web_out_l(void) { return out_l; }
EXPORT float *web_out_r(void) { return out_r; }
/* n frames (<= 1024) into out_l / out_r: the device runs until they exist */
EXPORT void web_render(uint32_t n)
{
    uint32_t k;
    if (n > 1024u)
        n = 1024u;
    while (ring_w - ring_r < n)
        run_ms(1);
    for (k = 0; k < n; k++) {
        uint32_t i = (ring_r++ % RING) * 2u;
        out_l[k] = ring[i];
        out_r[k] = ring[i + 1];
    }
}

static uint32_t ui_frames;
EXPORT void web_boot(void) { boot(); }
EXPORT void web_nor_erase(void) {}                /* (the saves are the host's RAM objects) */

/* ------------------------------------------------------------ the screen --- */
EXPORT const uint16_t *teensy_fb(void) { return fb; }
EXPORT uint32_t teensy_draws(void) { return blits; }
EXPORT uint32_t teensy_pixels(void) { return teensy_px; }
EXPORT void teensy_redraw(void) { uint32_t y; for (y = 0; y < 240u; y++) { dirty_lo[y] = 0; dirty_hi[y] = 239; } }
EXPORT uint32_t teensy_ui_frames(void) { return blits; }
static uint32_t span_row;
/* the changed spans, RLE (docs/LINK_PROTOCOL.md 'L'), as ports/felucca's */
EXPORT uint32_t teensy_spans(uint8_t *out, uint32_t cap)
{
    uint32_t len = 0, k;
    for (k = 0; k < 240u; k++) {
        uint32_t y = (span_row + k) % 240u, x0, n, i = 0, o;
        const uint16_t *p;
        if (dirty_lo[y] > dirty_hi[y])
            continue;
        x0 = dirty_lo[y];
        n = dirty_hi[y] - x0 + 1u;
        if (len + 3u + n * 3u > cap) {
            span_row = y;
            return len;
        }
        p = fb + y * 240u + x0;
        o = len;
        out[o++] = (uint8_t)y; out[o++] = (uint8_t)x0; out[o++] = (uint8_t)n;
        while (i < n) {
            uint32_t r = 1, l;
            while (i + r < n && r < 128u && p[i + r] == p[i]) r++;
            if (r >= 2u) {
                out[o++] = (uint8_t)(0x80u | (r - 1u));
                memcpy(out + o, &p[i], 2); o += 2;
                i += r;
                continue;
            }
            l = 1;
            while (i + l < n && l < 128u && !(i + l + 1u < n && p[i + l] == p[i + l + 1u])) l++;
            out[o++] = (uint8_t)(l - 1u);
            memcpy(out + o, &p[i], 2u * l); o += 2u * l;
            i += l;
        }
        len = o;
        dirty_lo[y] = 1; dirty_hi[y] = 0;
    }
    span_row = 0;
    return len;
}

/* ------------------------------------------------------------- the panel --- */
/* the FM-1's matrix ids to X0X's labels and roles: its default panel map (firmware/src/app/panel.c PANEL_DEFAULT) */
static const uint8_t PANEL_BTN[NB] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 0, 1};   /* matrix id per B_* */
static const uint8_t PANEL_ENC[NE] = {0, 1, 6, 2, 3, 4, 5};                            /* matrix encoder per EN_* */
static const int8_t FM1_KEYMAP[6][11] = {      /* firmware/hal/fm1_input.h: key id at (column, row bit) */
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 5, 11,  4, 10,  3,  9,  2,  8, -1, -1, -1},
    {34, 35, 36, 37, 38, 40, 39, 13,  7,  6, 12},
    {23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33},
    { 0,  1, 15, 14, 17, 16, 19, 18, 20, 21, 22},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
};
EXPORT void teensy_raw_input(uint32_t notes, uint32_t buttons)
{
    uint32_t i, b = 0;
    for (i = 0; i < NB; i++)
        if (buttons >> PANEL_BTN[i] & 1u)
            b |= 1u << i;
    held_btn = b;
    set_keys(notes & ((1u << NKEYS) - 1u));
}
EXPORT void teensy_raw_enc(uint32_t hw, int32_t n)
{
    uint32_t r;
    for (r = 0; r < NE; r++)
        if (PANEL_ENC[r] == hw)
            enc_acc[r] += n;
}
EXPORT void teensy_raw_adc(uint32_t m, uint32_t batt)   /* MASTER 0..1023 -> X0X's gain Q12 (plat_fm1.c: squared) */
{
    (void)batt;
    if (m > 1023u)
        m = 1023u;
    master = (m * m) >> 8;
}
/* the lights as the matrix columns: lit, dim (glow / dim keys); PLAY green at its keyless spot (plat_fm1.c) */
static int8_t led_at(uint32_t id)
{
    uint32_t c, r;
    for (c = 0; c < 11u; c++)
        for (r = 1; r < 5u; r++)
            if (FM1_KEYMAP[r][c] == (int8_t)id)
                return (int8_t)(c << 3 | r);
    return -1;
}
EXPORT uint32_t teensy_leds(uint8_t *out)
{
    uint32_t i;
    memset(out, 0, 45);
    for (i = 0; i < NB + NKEYS; i++) {
        uint32_t id = i < NB ? PANEL_BTN[i] : 14u + (i - NB);
        int on = i < NB ? (int)(lit_btn >> i & 1u) : (int)(lit_keys >> (i - NB) & 1u);
        int dim = i < NB ? 1 : (int)(dim_keys >> (i - NB) & 1u);
        int8_t q = led_at(id);
        if (i == B_PLAY)
            q = (int8_t)(8 << 3 | 1);
        if (q < 0)
            continue;
        if (on)
            out[q >> 3] |= (uint8_t)(1u << (q & 7));
        else if (dim)
            out[11 + (q >> 3)] |= (uint8_t)(1u << (q & 7));
    }
    return 45;
}

/* the Felucca ports' label / role inputs (main.rs uses them for the PC viewer): X0X's own labels */
EXPORT void web_buttons(uint32_t m) { held_btn = m; }
EXPORT void web_keys(uint32_t m) { set_keys(m & ((1u << NKEYS) - 1u)); }
EXPORT void web_enc(uint32_t role, int32_t n)
{
    if (role < NE)
        enc_acc[role] += n;
}
EXPORT void web_master(uint32_t v) { teensy_raw_adc(v, 0); }

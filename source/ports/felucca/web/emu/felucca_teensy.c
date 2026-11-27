/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments
 * The design (the device clock driven by the audio the worklet asks for, the exports below, the saved
 * flash handed back before boot) follows X0X's browser build, web/emu/x0x_web.c of charlesvestal/fm1-x0x
 * (GPL-3.0), a Felucca fork. */
/* teensy-fm1: Felucca on the Teensy 4.1. Derived from Felucca's web/emu/felucca_web.c (hugelton/Felucca f4cbb6a,
 * the last commit that has it); the changes are marked "teensy-fm1:". The device loop is the browser build's: the
 * Teensy calls web_render() from its audio path and reads the screen from web_fb. */
/* Felucca in the browser: the firmware's sources, as the host tests build them (tests/hostsim.c: libc,
 * engines, voices, FX, sequencer; then the UI, main.c, storage and projects, in src/felucca.c's order),
 * compiled to WebAssembly and run in an AudioWorklet against a simulated FM-1:
 *   - the device clock: TIMER4 ticks (24 MHz) and fm1_ms advance with the audio the worklet asks for; the
 *     audio ISR (audio.c fm1_alnk0_irq, the real one) renders each I2S half (HALF_FRAMES) as the DMA would
 *     ask for it, and the main loop (main.c's, below in web_frame) runs every 15 ms, with ui_input on every
 *     millisecond in between as main.c's wait loop calls it;
 *   - the panel: buttons and encoders by their printed labels (panel.c's table maps them to the matrix),
 *     the keys as fm1_in.notes, the LEDs read back from fm1_led / fm1_led_dim through the key matrix;
 *   - the screen: lcd_fill / lcd_blit into a 240 x 240 RGB565 frame (the panel's byte order);
 *   - the flash: a 1 MiB NOR image behind storage.c's hooks (settings, projects, user presets, the user
 *     sample slots read through it as XIP); the page keeps its storage sectors and gives them back at boot.
 * Not here: USB (MIDI, audio, the editor, updates), TRS MIDI, the serial console, the battery (full), the
 * crash / UBOOT paths. Nothing of the firmware changes for this file.
 *   web/emu/build.sh  ->  build/emu/{felucca.wasm, worklet.js, index.html} */
#include <stdint.h>

#define FELUCCA_WEB 1
#ifndef FELUCCA_CDC
#define FELUCCA_CDC 1                    /* as src/felucca.c: MENU > USB SERIAL is there (nothing behind it) */
#endif
#define FELUCCA_FLASH 1
#define FELUCCA_OTA 0
#define FELUCCA_UAC 0
#define FELUCCA_UART 0
#ifndef FELUCCA_VERSION
#define FELUCCA_VERSION "WEB"
#endif

/* teensy-fm1: the 1 MiB SPI NOR can't sit in the Teensy's RAM. It reads as erased flash (a 1 MiB constant in the
 * Teensy's program flash) except the 4 KiB sectors written since boot, which get a RAM copy (NOR_SLOTS of them).
 * The user sample slots read straight from the constant (no user samples yet). */
#define NOR_SIZE 0x100000u
#define NOR_SLOTS 8u
static const uint8_t nor_base[NOR_SIZE] = {[0 ... NOR_SIZE - 1] = 0xFF};
static uint8_t nor_slot[NOR_SLOTS][4096];
static int16_t nor_map[NOR_SIZE / 4096u];   /* sector -> slot + 1 (0: not written) */
static uint32_t nor_used;
#define SMP_USER_XIP(k) (nor_base + SMP_USER_BASE + (k) * SMP_USER_SIZE)   /* eng_sample.c */
/* teensy-fm1: hostsim.c's timing (used by its own test main, never on the Teensy) wants clock_gettime */
#include <time.h>
#ifndef CLOCK_MONOTONIC
#define CLOCK_MONOTONIC 1
static int clock_gettime(int c, struct timespec *t) { (void)c; t->tv_sec = 0; t->tv_nsec = 0; return 0; }
#endif
#define main hostsim_main
#include "../../tests/hostsim.c"
#undef main

/* ------------------------------------------------------------ the clock --- */
#define FM1_TICKS_PER_US 24u             /* TIMER4: 24 MHz */
static uint32_t web_ticks;
static uint32_t fm1_ticks(void) { return web_ticks; }

/* ------------------------------------------------------ audio (ALNK0) --- */
static uint32_t web_half;                /* the half the next render fills */
#define FM1_AUDIO_HALF 1u
static uint8_t fm1_audio_pending(void) { return FM1_AUDIO_HALF; }
static void fm1_audio_ack_aux(uint8_t p) { (void)p; }
static uint32_t fm1_audio_free_half(void) { return web_half; }
static void fm1_audio_ack_half(void) {}
static void fm1_audio_init(int32_t *buf, uint32_t words, void (*isr)(void), uint32_t prio)
{ (void)buf; (void)words; (void)isr; (void)prio; }
static void fm1_audio_stop(void) {}
void isr_alnk0(void) {}
#include "../../firmware/src/audio.c"

/* ------------------------------------------------------------ the panel --- */
#define FM1_NCOL 11u
static const int8_t FM1_KEYMAP[6][FM1_NCOL] = {      /* as bsp/boards/fm1/fm1_input.h: key id at (column, row bit) */
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 5, 11,  4, 10,  3,  9,  2,  8, -1, -1, -1},
    {34, 35, 36, 37, 38, 40, 39, 13,  7,  6, 12},
    {23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33},
    { 0,  1, 15, 14, 17, 16, 19, 18, 20, 21, 22},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
};
static uint8_t fm1_led[FM1_NCOL], fm1_led_dim[FM1_NCOL], fm1_led_breath[FM1_NCOL], fm1_led_mid[FM1_NCOL];
static uint32_t web_dim_lo;
static void fm1_led_dim_level(uint32_t lo) { web_dim_lo = lo; }
/* the power-on LED sweep (bsp/boards/fm1/fm1_input.h fm1_led_anim_start, the picture bsp/boards/fm1/fm1_led_anim.h): its frame from the
 * clock (a scan frame 1.1 ms), the page draws each LED's level (web_anim_levels); at its end, or on a key or a button
 * down, the last picture (the glow everywhere, or dark) and the UI's LEDs from the next frame, as the scan does */
#include "../../firmware/bsp/boards/fm1/fm1_led_anim.h"
static uint32_t web_anim_t0, web_anim_glow, web_anim_run, web_anim_idle;   /* (idle: 1.2's idle animation, MENU >
                                                                              * ANIM IDLE: fm1_idle_level, endless) */
static uint32_t web_anim_frame(void) { return (web_ticks - web_anim_t0) / (1100u * FM1_TICKS_PER_US); }
static void fm1_led_anim_start(uint32_t glow)
{
    web_anim_idle = 0;
    memset(fm1_led, 0, sizeof fm1_led);
    memset(fm1_led_dim, 0, sizeof fm1_led_dim);
    memset(fm1_led_breath, 0, sizeof fm1_led_breath);
    memset(fm1_led_mid, 0, sizeof fm1_led_mid);
    web_anim_t0 = web_ticks;
    web_anim_glow = glow != 0u;
    web_anim_run = 1;
}
static int fm1_led_anim_on(void)
{
    if (web_anim_run && web_anim_idle && (fm1_in.notes || fm1_in.buttons))
        web_anim_run = 0;                       /* (the idle animation: ended, the UI's LEDs from now) */
    if (web_anim_run && !web_anim_idle && (web_anim_frame() >= FM1_ANIM_FRAMES || fm1_in.notes || fm1_in.buttons)) {
        uint32_t c, r;
        web_anim_run = 0;
        for (c = 0; c < FM1_NCOL; c++)
            for (r = 1; r < 5u; r++)
                if (web_anim_glow && FM1_KEYMAP[r][c] >= 0)
                    fm1_led_dim[c] |= (uint8_t)(1u << r);
    }
    return (int)web_anim_run;
}
static void fm1_led_idle_start(uint32_t glow)
{
    fm1_led_anim_start(glow);
    web_anim_idle = 1;
}
static int fm1_led_idle_on(void) { return fm1_led_anim_on() && web_anim_idle; }
static void fm1_led_anim_stop(void) { web_anim_run = 0; }
static uint32_t web_pressed, web_released, web_notes_pressed;
static int32_t web_enc_steps[7];
static uint32_t fm1_input_edges(uint32_t *released)
{
    uint32_t p = web_pressed;
    if (released)
        *released = web_released;
    web_pressed = web_released = 0;
    return p;
}
static uint32_t fm1_input_note_edges(void) { uint32_t n = web_notes_pressed; web_notes_pressed = 0; return n; }
static int32_t fm1_enc_take(uint32_t e) { int32_t s = web_enc_steps[e % 7u]; web_enc_steps[e % 7u] = 0; return s; }
static void fm1_input_init(void) {}
static void fm1_input_tick(void) {}
static void fm1_wdt_feed(void) {}
static void fm1_irq_off(void) {}
static void fm1_irq_on(void) {}

/* ----------------------------------------------------------- the screen --- */
static uint16_t web_fb[240 * 240];       /* RGB565 as the panel gets it (high byte first) */
static uint32_t web_draws;
static uint32_t teensy_px;   /* teensy-fm1: pixels sent to the panel (what a display link has to carry) */
/* teensy-fm1: the FM-1 link sends only what changed: per row, the first and last changed column (lo > hi: clean) */
static uint8_t dirty_lo[240], dirty_hi[240];
static void mark(uint32_t y, uint32_t x)
{
    if (dirty_lo[y] > dirty_hi[y]) { dirty_lo[y] = dirty_hi[y] = (uint8_t)x; return; }
    if (x < dirty_lo[y]) dirty_lo[y] = (uint8_t)x;
    if (x > dirty_hi[y]) dirty_hi[y] = (uint8_t)x;
}
static void lcd_fill(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint16_t c)
{
    uint32_t i, j;
    uint16_t sw = (uint16_t)((c >> 8) | (c << 8));
    for (j = 0; j < h && y + j < 240u; j++)
        for (i = 0; i < w && x + i < 240u; i++)
            if (web_fb[(y + j) * 240u + x + i] != sw) {
                web_fb[(y + j) * 240u + x + i] = sw;
                mark(y + j, x + i);
            }
    teensy_px += w * h;
    web_draws++;
}
static void lcd_blit(uint32_t x, uint32_t y, uint32_t w, uint32_t h, const uint16_t *p)
{
    uint32_t i, j;
    for (j = 0; j < h && y + j < 240u; j++)
        for (i = 0; i < w && x + i < 240u; i++)
            if (web_fb[(y + j) * 240u + x + i] != p[j * w + i]) {
                web_fb[(y + j) * 240u + x + i] = p[j * w + i];
                mark(y + j, x + i);
            }
    teensy_px += w * h;
    web_draws++;
}
static void lcd_sync(void) {}
static void lcd_power(uint32_t s)              /* MENU > SCREEN OFF (lcd.c): off, the page dark; on: redrawn before */
{
    if (!s) {
        uint32_t y;
        memset(web_fb, 0, sizeof web_fb);
        for (y = 0; y < 240u; y++) { dirty_lo[y] = 0; dirty_hi[y] = 239; }   /* teensy-fm1 */
        web_draws++;
    }
}
static void lcd_wake_now(void) {}
static void lcd_init(void) {}

#include "../../firmware/src/gfx.c"
#include "../../firmware/src/panel.c"
#include "../../firmware/src/ui.c"
#include "../../firmware/src/menu_items.c"
#include "../../firmware/src/icons.c"
#include "../../firmware/src/ui_graph.c"
#include "../../firmware/src/ui_draw.c"
#include "../../firmware/src/ui_menu.c"
#include "../../firmware/src/ui_input.c"
#include "../../firmware/src/ui_layer.c"

/* ------------------------------------------------------------ the flash --- */
static uint32_t web_flash_writes;
static uint8_t flash_ok;
static uint8_t *nor_sector(uint32_t sec, int make)   /* teensy-fm1: the RAM copy of a sector (made on demand) */
{
    if (nor_map[sec])
        return nor_slot[nor_map[sec] - 1];
    if (!make || nor_used >= NOR_SLOTS)
        return 0;
    memcpy(nor_slot[nor_used], nor_base + sec * 4096u, 4096);
    nor_map[sec] = (int16_t)(++nor_used);
    return nor_slot[nor_used - 1];
}
static int st_read(uint32_t off, void *dst, uint32_t n)
{
    uint8_t *d = dst;
    if (off >= NOR_SIZE || n > NOR_SIZE - off)
        return -1;
    while (n) {
        uint32_t sec = off / 4096u, o = off % 4096u, k = 4096u - o < n ? 4096u - o : n;
        const uint8_t *p = nor_sector(sec, 0);
        memcpy(d, (p ? p : nor_base + sec * 4096u) + o, k);
        d += k, off += k, n -= k;
    }
    return 0;
}
static int st_erase(uint32_t off)
{
    uint8_t *p;
    if (off >= NOR_SIZE || 4096u > NOR_SIZE - off)
        return -8;
    if (!(p = nor_sector(off / 4096u, 1)))
        return -8;                                   /* teensy-fm1: out of RAM sectors */
    memset(p, 0xFF, 4096);
    web_flash_writes++;
    return 0;
}
static int st_prog(uint32_t off, const void *src, uint32_t n)       /* NOR: a write only clears bits */
{
    const uint8_t *s = src;
    uint32_t i;
    if (off >= NOR_SIZE || n > NOR_SIZE - off)
        return -8;
    for (i = 0; i < n; i++) {
        uint8_t *p = nor_sector((off + i) / 4096u, 1);
        if (!p)
            return -8;                               /* teensy-fm1: out of RAM sectors */
        p[(off + i) % 4096u] &= s[i];
    }
    web_flash_writes++;
    return 0;
}
static uint32_t irq_save(void) { return 0; }
static void irq_restore(uint32_t f) { (void)f; }
static uint32_t fl_jedec_ram(void) { return 0x856014u; }             /* the FM-1's part (project.c) */
static void fl_plain_window_init(void) {}
#define FL_FAR(fn) (fn)
#include "../../firmware/src/storage.c"
#include "../../firmware/src/upreset.c"
#include "../../firmware/src/project.c"

/* ------------------------------------------------- main.c and its HAL --- */
#if FELUCCA_CPU2
typedef struct { uint32_t magic, count, vec, pc, emu, dbg, rets, core; } fm1_crash_t;
#else
typedef struct { uint32_t magic, count, vec, pc, emu, dbg, rets; } fm1_crash_t;
#endif
#define FM1_CRASH_MAGIC 0x43525348u
static fm1_crash_t fm1_crash;                     /* (none: main.c boot_leds runs the sweep) */
static struct { uint8_t p3_rst, wdt_con; uint32_t rst_src; } fm1_boot;
uint32_t _data_start[1], _data_end[1], _data_load[1], _bss_start[1], _bss_end[1];
uint32_t _pool_start[1], _pool_end[1], _rt_start[1], _rt_end[1], _rt_load[1];
static uint32_t web_master_adc = 700;   /* MASTER, 0..1023 as the ADC reads it */
enum { FM1_ADC_BATT = 3, FM1_ADC_MASTER = 4 };
static uint32_t web_batt_adc;   /* teensy-fm1: the FM-1's battery reading over the link */
static int32_t fm1_adc_read(uint32_t ch) { return ch == FM1_ADC_MASTER ? (int32_t)web_master_adc : (int32_t)(web_batt_adc ? web_batt_adc : 620u); }
static void fm1_adc_init(void) {}
static void fm1_timer5_ack(void) {}
static void fm1_timer5_start(void (*isr)(void), uint32_t prio) { (void)isr; (void)prio; }
void isr_timer5(void) {}
static void fm1_reboot(void) {}
static void fm1_enter_uboot(void) {}
static void fm1_guard_lock_top(void) {}
static void fm1_irq_enable_all(void) {}
static void fm1_irq_init(void) {}
static void fm1_time_init(void) {}
static void fm1_reset_reason(void) {}
static void fm1_wdt_arm(uint32_t v) { (void)v; }
static void fm1_mailbox_clear(void) {}
static void fm1_guard_enable(uint32_t m) { (void)m; }
enum { FM1_GUARD_STACK = 1, FM1_GUARD_WRITE = 2, FM1_GUARD_BUS = 4, FM1_GUARD_PC = 8 };
#if FELUCCA_CPU2
/* the second core (felucca-bsp chip/fm1_cpu1.h): none here, one core whatever MENU > 2ND CORE says (fx.c mix_block's
 * C1_ON() is 0 without felucca.c's macros) */
static uint8_t fm1_cpu1_on, fm1_cpu1_gave_up;
static void fm1_cpu1_stop(void) {}
static int fm1_cpu1_start(void) { return -1; }
#endif
#if FELUCCA_CDC
static void cdc_task(void) {}
#endif
/* main.c's calls into the USB driver (registers): nothing here (usb.c's own functions stay as they are) */
#define usb_poll() ((void)0)
#define usb_start() ((void)0)
#define usb_retry(ms) ((void)(ms))
#define usb_detach() ((void)0)
#include "../../firmware/src/master.c"
#include "../../firmware/src/main.c"
#undef usb_poll
#undef usb_start
#undef usb_retry
#undef usb_detach

/* --------------------------------------------------------- the device --- */
static uint32_t web_booted, web_boot_ms, web_last_frame;

/* main.c fm1_main's boot, up to the main loop (no USB, no UART, no panel setup) */
static void web_power_on(void)
{
    persist_boot();
    settings_init();
    usb_serial_apply();
    lcd_init();
    draw_splash();                        /* main.c: the splash (ui_draw.c) */
    memset(&felucca_dbg, 0, sizeof felucca_dbg);
    felucca_dbg.magic = DBG_MAGIC;
    fm1_input_init();
    fm1_adc_init();
    panel_init();
    felucca_init();
    master_boot();                        /* main.c (#137): the pot's level from the first block */
    kb_boot_hold = 1;                     /* main.c (#137): the keys silent under the splash */
    audio_init();
    boot_leds();                          /* main.c: the power-on LED sweep, from the scan's start */
    autosave_boot(boot_clean);            /* main.c: the last session's music (1.2, project.c) */
    web_boot_ms = fm1_ms + 430u;         /* main.c: 30 + 400 ms before the first frame */
    web_booted = 1;
}

/* one pass of main.c fm1_main's loop (what the browser has: no USB, editor, UBOOT, console) */
static void web_frame(void)
{
    int32_t b = fm1_adc_read(FM1_ADC_BATT);
    if (b > 0)
        song.batt_raw = song.batt_raw ? song.batt_raw + (b - song.batt_raw) / 32 : b;
    master_poll();
    felucca_dbg.ui_frames++;
    ui_input();
    settings_poll();
    autosave_poll();
    ui_leds();
    ui_draw();
}

/* output ring, stereo float, filled by web_ms, drained by web_render */
#define RING 4096u
static float ring[RING * 2];
static uint32_t ring_w, ring_r, web_samples_due;

/* one millisecond of the device: the clock, the audio halves due by now, the main loop */
static void web_ms(void)
{
    fm1_ms++;
    web_ticks += 1000u * FM1_TICKS_PER_US;
    web_samples_due += FS;
    while (web_samples_due >= 1000u * HALF_FRAMES) {      /* an I2S half: the audio ISR renders it */
        uint32_t k;
        const int32_t *o = &abuf[web_half * HALF_WORDS];
        web_samples_due -= 1000u * HALF_FRAMES;
        fm1_alnk0_irq();
        for (k = 0; k < HALF_FRAMES; k++) {
            uint32_t i = (ring_w++ % RING) * 2u;
            ring[i] = (float)o[2u * k] / 8388608.0f;
            ring[i + 1] = (float)o[2u * k + 1u] / 8388608.0f;
        }
        web_half ^= 1u;
    }
    if (!web_booted || (int32_t)(fm1_ms - web_boot_ms) < 0)
        return;
    if (web_boot_ms) {                                    /* main.c: the splash goes, the loop starts */
        web_boot_ms = 0;
        kb_boot_hold = 0;
        lcd_fill(0, 0, 240, 240, T_BG);
        web_last_frame = fm1_ms - 15u;
    }
    if (fm1_ms - web_last_frame >= 15u) {                /* main.c: a frame, then ui_input until 15 ms are over */
        web_last_frame = fm1_ms;
        web_frame();
    } else {
        ui_input();                                       /* main.c's wait loop */
    }
}

#define EXPORT __attribute__((used, visibility("default")))
#undef __attribute__

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
        web_ms();
    for (k = 0; k < n; k++) {
        uint32_t i = (ring_r++ % RING) * 2u;
        out_l[k] = ring[i];
        out_r[k] = ring[i + 1];
    }
}

/* the flash: erased, then the page's saved sectors (web_nor), then boot */
EXPORT uint32_t web_nor_size(void) { return NOR_SIZE; }
EXPORT void web_nor_erase(void) { memset(nor_map, 0, sizeof nor_map); nor_used = 0; }   /* teensy-fm1 */
/* teensy-fm1: the screen, and how many draws have touched it, for the Teensy's display link */
EXPORT const uint16_t *teensy_fb(void) { return web_fb; }
EXPORT uint32_t teensy_draws(void) { return web_draws; }
EXPORT uint32_t teensy_nor_sectors(void) { return nor_used; }
EXPORT uint32_t teensy_pixels(void) { return teensy_px; }
/* teensy-fm1: the whole screen counts as changed (the FM-1 asked for a redraw) */
EXPORT void teensy_redraw(void) { uint32_t y; for (y = 0; y < 240u; y++) { dirty_lo[y] = 0; dirty_hi[y] = 239; } }
/* teensy-fm1: changed rows as spans (docs/LINK_PROTOCOL.md 'L') into out, up to cap bytes; those rows become clean.
 * Returns the bytes written (0: nothing changed). Rows are taken round-robin so a busy area cannot starve the rest. */
static uint32_t span_row;
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
        if (len + 3u + n * 3u > cap) {           /* worst case: every pixel a 1-pixel literal */
            span_row = y;
            return len;
        }
        p = web_fb + y * 240u + x0;
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
/* teensy-fm1: the FM-1's raw input, as its own scan reports it (fm1_in.notes, fm1_in.buttons matrix bits, encoder
 * steps by hardware index, the ADCs) */
EXPORT void teensy_raw_input(uint32_t notes, uint32_t buttons)
{
    uint32_t oldb = fm1_in.buttons, oldn = fm1_in.notes;
    notes &= (1u << 27) - 1u;
    web_pressed |= buttons & ~oldb;
    web_released |= oldb & ~buttons;
    fm1_in.buttons = buttons;
    web_notes_pressed |= notes & ~oldn;
    fm1_in.notes = notes;
}
EXPORT void teensy_raw_enc(uint32_t hw, int32_t n) { web_enc_steps[hw % 7u] += n; }
EXPORT void teensy_raw_adc(uint32_t master, uint32_t batt) { web_master_adc = master > 1023u ? 1023u : master; web_batt_adc = batt > 1023u ? 1023u : batt; }
/* teensy-fm1: the LEDs for the link ('D', 45 bytes); returns 1 when they differ from the last call */
EXPORT uint32_t teensy_leds(uint8_t *out)
{
    static uint8_t last[45];
    memcpy(out, fm1_led, 11); memcpy(out + 11, fm1_led_dim, 11); memcpy(out + 22, fm1_led_breath, 11);
    memcpy(out + 33, fm1_led_mid, 11); out[44] = (uint8_t)web_dim_lo;
    if (!memcmp(out, last, 45))
        return 0;
    memcpy(last, out, 45);
    return 1;
}
EXPORT uint32_t teensy_ui_frames(void) { return felucca_dbg.ui_frames; }
/* teensy-fm1: the firmware menu (src/main.rs), drawn into the screen before any firmware boots, in the look of the
 * terminal's BBL MOD SWAPPER splash: a chunky bold block font (6 x 7 dots, M and W 7 wide; bit w-1 the leftmost dot),
 * hot pink on near black. A title, n entries (name, and a detail on the right) with entry sel in a pink box, up to 3
 * shown (scrolled to keep sel in view), a hint */
static const struct { char ch; uint8_t w; uint8_t row[7]; } MB7[] = {
    {' ', 4, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
    {'-', 6, {0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x00}},
    {'.', 2, {0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03}},
    {'0', 6, {0x1E, 0x33, 0x37, 0x3F, 0x3B, 0x33, 0x1E}},
    {'1', 6, {0x0C, 0x1C, 0x0C, 0x0C, 0x0C, 0x0C, 0x1E}},
    {'2', 6, {0x1E, 0x33, 0x03, 0x06, 0x0C, 0x18, 0x3F}},
    {'3', 6, {0x3E, 0x03, 0x03, 0x1E, 0x03, 0x03, 0x3E}},
    {'4', 6, {0x33, 0x33, 0x33, 0x3F, 0x03, 0x03, 0x03}},
    {'5', 6, {0x3F, 0x30, 0x30, 0x3E, 0x03, 0x03, 0x3E}},
    {'6', 6, {0x1E, 0x30, 0x30, 0x3E, 0x33, 0x33, 0x1E}},
    {'7', 6, {0x3F, 0x03, 0x06, 0x0C, 0x0C, 0x0C, 0x0C}},
    {'8', 6, {0x1E, 0x33, 0x33, 0x1E, 0x33, 0x33, 0x1E}},
    {'9', 6, {0x1E, 0x33, 0x33, 0x1F, 0x03, 0x03, 0x1E}},
    {'A', 6, {0x1E, 0x33, 0x33, 0x3F, 0x33, 0x33, 0x33}},
    {'B', 6, {0x3E, 0x33, 0x33, 0x3E, 0x33, 0x33, 0x3E}},
    {'C', 6, {0x1F, 0x30, 0x30, 0x30, 0x30, 0x30, 0x1F}},
    {'D', 6, {0x3E, 0x33, 0x33, 0x33, 0x33, 0x33, 0x3E}},
    {'E', 6, {0x3F, 0x30, 0x30, 0x3E, 0x30, 0x30, 0x3F}},
    {'F', 6, {0x3F, 0x30, 0x30, 0x3E, 0x30, 0x30, 0x30}},
    {'G', 6, {0x1F, 0x30, 0x30, 0x3F, 0x33, 0x33, 0x1F}},
    {'H', 6, {0x33, 0x33, 0x33, 0x3F, 0x33, 0x33, 0x33}},
    {'I', 6, {0x3F, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x3F}},
    {'J', 6, {0x0F, 0x06, 0x06, 0x06, 0x06, 0x36, 0x1C}},
    {'K', 6, {0x33, 0x36, 0x3C, 0x38, 0x3C, 0x36, 0x33}},
    {'L', 6, {0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x3F}},
    {'M', 7, {0x63, 0x77, 0x7F, 0x6B, 0x63, 0x63, 0x63}},
    {'N', 6, {0x33, 0x3B, 0x3F, 0x3F, 0x3F, 0x37, 0x33}},
    {'O', 6, {0x1E, 0x33, 0x33, 0x33, 0x33, 0x33, 0x1E}},
    {'P', 6, {0x3E, 0x33, 0x33, 0x3E, 0x30, 0x30, 0x30}},
    {'Q', 6, {0x1E, 0x33, 0x33, 0x33, 0x3F, 0x36, 0x1F}},
    {'R', 6, {0x3E, 0x33, 0x33, 0x3E, 0x3C, 0x36, 0x33}},
    {'S', 6, {0x1F, 0x30, 0x30, 0x1E, 0x03, 0x03, 0x3E}},
    {'T', 6, {0x3F, 0x3F, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C}},
    {'U', 6, {0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x1E}},
    {'V', 6, {0x33, 0x33, 0x33, 0x33, 0x33, 0x1E, 0x0C}},
    {'W', 7, {0x63, 0x63, 0x63, 0x6B, 0x7F, 0x77, 0x63}},
    {'X', 6, {0x33, 0x33, 0x1E, 0x0C, 0x1E, 0x33, 0x33}},
    {'Y', 6, {0x33, 0x33, 0x1E, 0x0C, 0x0C, 0x0C, 0x0C}},
    {'Z', 6, {0x3F, 0x03, 0x06, 0x0C, 0x18, 0x30, 0x3F}},
    {'%', 6, {0x31, 0x32, 0x04, 0x08, 0x10, 0x13, 0x23}}, /* teensy-fm1: ROM copy progress */
    /* teensy-fm1: the Sample Lab's symbols (src/lab.rs glyph), for the menu's bottom bar */
    {'\x01', 6, {0x20, 0x38, 0x3E, 0x3F, 0x3E, 0x38, 0x20}}, /* play (start) */
    {'\x02', 7, {0x1C, 0x1C, 0x7F, 0x3E, 0x1C, 0x08, 0x7F}}, /* download (start) */
    {'\x04', 6, {0x0C, 0x1E, 0x3F, 0x0C, 0x0C, 0x0C, 0x0C}}, /* up */
    {'\x05', 6, {0x0C, 0x0C, 0x0C, 0x0C, 0x3F, 0x1E, 0x0C}}, /* down */
};
static uint32_t mb_w(const char *t, uint32_t sx, uint32_t gap)
{
    uint32_t w = 0, i;
    for (; *t; t++)
        for (i = 0; i < sizeof MB7 / sizeof MB7[0]; i++)
            if (MB7[i].ch == (*t >= 'a' && *t <= 'z' ? *t - 32 : *t))
                w += MB7[i].w * sx + (t[1] ? gap : 0u);
    return w;
}
static void mb_text(int32_t x, int32_t y, const char *t, uint32_t sx, uint32_t sy, uint32_t gap, uint16_t c)
{
    uint32_t i, r, k, n;
    for (; *t; t++)
        for (i = 0; i < sizeof MB7 / sizeof MB7[0]; i++)
            if (MB7[i].ch == (*t >= 'a' && *t <= 'z' ? *t - 32 : *t)) {
                n = MB7[i].w;
                for (r = 0; r < 7u; r++)
                    for (k = 0; k < n; k++)
                        if (MB7[i].row[r] & (1u << (n - 1u - k)))
                            cv_rect(x + (int32_t)(k * sx), y + (int32_t)(r * sy), (int32_t)sx, (int32_t)sy, c);
                x += (int32_t)(n * sx + gap);
            }
}
/* teensy-fm1: the firmware menu, drawn as the Sample Lab (src/lab.rs layout): a pink banner with the title in black,
 * four big rows (the selected one on pink, the others bare), a scroll bar, a white bottom banner with the controls in
 * black. hint: "left\tright" (the controls at both ends) or one centred line. */
#define MN_ROW_Y 48u
#define MN_ROW_P 42u
#define MN_ROWS 4u
static const uint16_t M_BG = 0x0841, M_INK = 0xFFDF, M_DIM = 0x8C71, M_BOX = 0x2945, M_PINK = 0xF8B2;
static void mn_top(const char *title)
{
    cv_begin(240, MN_ROW_Y, M_BG);
    cv_rect(0, 0, 240, 42, M_PINK);
    mb_text(8, 6, title, 2, 4, 2, M_BG);
    cv_blit(0, 0);
}
static void mn_bottom(const char *hint)
{
    char l[48];
    const char *tab = hint;
    uint32_t m;
    cv_begin(240, 240u - 212u, M_BG);
    cv_rect(0, 1, 240, 27, M_INK);
    while (*tab && *tab != '\t')
        tab++;
    m = (uint32_t)(tab - hint) < sizeof l - 1u ? (uint32_t)(tab - hint) : sizeof l - 1u;
    memcpy(l, hint, m);
    l[m] = 0;
    if (*tab) {
        mb_text(8, 5, l, 2, 3, 2, M_BG);
        mb_text((int32_t)(232u - mb_w(tab + 1, 2, 2)), 5, tab + 1, 2, 3, 2, M_BG);
    } else {
        uint32_t sx = mb_w(l, 2, 2) <= 224u ? 2u : 1u;
        mb_text((int32_t)(240u - mb_w(l, sx, sx)) / 2, 5, l, sx, 3, sx, M_BG);
    }
    cv_blit(0, 212);
}
/* one row strip (y0 .. y0 + MN_ROW_P): a name, and on the right the details ("version\nstatus", small) */
static void mn_row(uint32_t r, const char *name, const char *detail, int on, int big_detail)
{
    uint16_t ink = on ? M_BG : M_INK, dim = on ? M_BG : M_DIM;
    cv_begin(240, MN_ROW_P, M_BG);
    if (on)
        cv_rect(8, 0, 218, (int32_t)MN_ROW_P - 4, M_PINK);
    if (name)
        mb_text(16, 5, name, 2, 4, 2, ink);
    if (detail && big_detail)
        mb_text(16, 5, detail, 2, 4, 2, ink);
    else if (detail) {
        const char *nl = detail;
        while (*nl && *nl != '\n')
            nl++;
        if (*nl) {
            char top[24];
            uint32_t m = (uint32_t)(nl - detail);
            if (m > sizeof top - 1u)
                m = sizeof top - 1u;
            memcpy(top, detail, m);
            top[m] = 0;
            mb_text((int32_t)(220u - mb_w(top, 1, 1)), 6, top, 1, 2, 1, dim);
            mb_text((int32_t)(220u - mb_w(nl + 1, 1, 1)), 22, nl + 1, 1, 2, 1, dim);
        } else
            mb_text((int32_t)(220u - mb_w(detail, 1, 1)), 14, detail, 1, 2, 1, dim);
    }
    cv_blit(0, MN_ROW_Y + r * MN_ROW_P);
}
EXPORT void teensy_menu(const char *title, const char *const *names, const char *const *details, uint32_t n,
                        uint32_t sel, const char *hint)
{
    uint32_t i, top = sel >= MN_ROWS / 2u ? sel - MN_ROWS / 2u : 0u;
    if (n > MN_ROWS && top > n - MN_ROWS)
        top = n - MN_ROWS;
    if (n <= MN_ROWS)
        top = 0;
    mn_top(title);
    for (i = 0; i < MN_ROWS; i++) {
        uint32_t k = top + i;
        mn_row(i, k < n ? names[k] : 0, k < n && details ? details[k] : 0, k == sel, 0);
    }
    /* the scroll bar, over the rows' right edge (as the Lab's) */
    if (n > MN_ROWS) {
        uint32_t h0 = MN_ROWS * MN_ROW_P - 4u, h = h0 * MN_ROWS / n;
        if (h < 8u)
            h = 8u;
        cv_begin(4, h0, M_BOX);
        cv_rect(0, (int32_t)((h0 - h) * top / (n - MN_ROWS)), 4, (int32_t)h, M_PINK);
        cv_blit(230, MN_ROW_Y);
    }
    mn_bottom(hint);
}
/* teensy-fm1: a ROM being copied: the title, its name on pink, under it the progress at the same size (the pink
 * filling as it goes, as the Lab's send), the hint at the bottom */
EXPORT void teensy_loading(const char *title, const char *name, const char *status, uint32_t pct, const char *hint)
{
    uint32_t r;
    mn_top(title);
    mn_row(0, name, 0, 1, 0);
    cv_begin(240, MN_ROW_P, M_BG);
    cv_rect(8, 0, 218, (int32_t)MN_ROW_P - 4, M_BOX);
    cv_rect(8, 0, (int32_t)(218u * (pct > 100u ? 100u : pct) / 100u), (int32_t)MN_ROW_P - 4, M_PINK);
    mb_text(16, 5, status, 2, 4, 2, M_INK);
    cv_blit(0, MN_ROW_Y + MN_ROW_P);
    for (r = 2; r < MN_ROWS; r++)
        mn_row(r, 0, 0, 0, 0);
    cv_begin(240, 4, M_BG);
    cv_blit(0, MN_ROW_Y + MN_ROWS * MN_ROW_P);   /* (the strip under the rows, above the bottom banner) */
    mn_bottom(hint);
}
EXPORT uint32_t web_flash_writes_count(void) { return web_flash_writes; }
EXPORT void web_boot(void)
{
    if (!web_booted)
        web_power_on();
}

/* input, by the printed labels: buttons bit b = B_FX .. B_OCTUP (panel.c), keys bit k = key k (F3 .. G5),
 * encoders by role (EN_SELECT .. EN_K4), MASTER 0..1023 */
static uint32_t web_btn_labels, web_key_mask;
EXPORT void web_buttons(uint32_t labels)
{
    uint32_t m = 0, b, old = fm1_in.buttons;
    web_btn_labels = labels;
    for (b = 0; b < NB; b++)
        if ((labels >> b) & 1u)
            m |= 1u << panel.btn[b];
    web_pressed |= m & ~old;
    web_released |= old & ~m;
    fm1_in.buttons = m;
}
EXPORT void web_keys(uint32_t mask)
{
    uint32_t old = fm1_in.notes;
    mask &= (1u << 27) - 1u;
    web_key_mask = mask;
    web_notes_pressed |= mask & ~old;
    fm1_in.notes = mask;
}
EXPORT void web_enc(uint32_t role, int32_t n)
{
    if (role < NE)
        web_enc_steps[panel.enc[role] % 7u] += n * panel.dir[role];
}
EXPORT void web_master(uint32_t v) { web_master_adc = v > 1023u ? 1023u : v; }
/* a channel message from Web MIDI, as a USB-MIDI packet arrives (usb.c midi_enqueue): 0 = the ring was full */
EXPORT uint32_t web_midi(uint32_t status, uint32_t d1, uint32_t d2)
{
    if (status < 0x80u || status >= 0xF0u)
        return 1;
    return (uint32_t)midi_enqueue((status >> 4) | status << 8 | (d1 & 127u) << 16 | (d2 & 127u) << 24, 1);
}

/* the LEDs: bit b = the button labelled b lit (bit 14: PLAY's green), keys bit k; the dim glow and the breath alike */
static uint32_t led_on(const uint8_t *l, uint32_t id)
{
    uint32_t c, r;
    for (c = 0; c < FM1_NCOL; c++)
        for (r = 1; r < 5u; r++)
            if (FM1_KEYMAP[r][c] == (int8_t)id)
                return (l[c] >> r) & 1u;
    return 0;
}
static uint32_t led_buttons(const uint8_t *l)
{
    uint32_t b, m = 0;
    for (b = 0; b < NB; b++)
        m |= led_on(l, panel.btn[b]) << b;
    return m;
}
static uint32_t led_keys(const uint8_t *l)
{
    uint32_t k, m = 0;
    for (k = 0; k < 27u; k++)
        m |= led_on(l, 14u + k) << k;
    return m;
}
EXPORT uint32_t web_lit_buttons(void)
{
    return led_buttons(fm1_led) | ((fm1_led[LED_PLAY_GREEN >> 3] >> (LED_PLAY_GREEN & 7u)) & 1u) << 14;
}
EXPORT uint32_t web_lit_keys(void) { return led_keys(fm1_led); }
EXPORT uint32_t web_dim_buttons(void) { return led_buttons(fm1_led_dim); }
EXPORT uint32_t web_dim_keys(void) { return led_keys(fm1_led_dim); }
EXPORT uint32_t web_dim_level(void) { return web_dim_lo; }
EXPORT uint32_t web_breath_buttons(void) { return led_buttons(fm1_led_breath); }   /* #119: dark .. ~60 % of lit */
EXPORT uint32_t web_breath_keys(void) { return led_keys(fm1_led_breath); }
EXPORT uint32_t web_mid_keys(void) { return led_keys(fm1_led_mid); }   /* the DRUM grid's beats: a steady mid level */
/* the power-on sweep (or 1.2's idle animation) running: each LED's level now (bsp/boards/fm1/fm1_led_anim.h: 0..64 /64 of lit, | 0x80 the glow under
 * it), 0..13 the buttons by label, 14..40 the keys F3..G5; 0 when it is not */
EXPORT uint8_t *web_anim_levels(void)
{
    static uint8_t lv[FM1_ANIM_NBTN + FM1_ANIM_NKEY];
    uint32_t f = web_anim_frame(), i;
    if (!fm1_led_anim_on())
        return 0;
    for (i = 0; i < FM1_ANIM_NBTN + FM1_ANIM_NKEY; i++)
        lv[i] = (uint8_t)(web_anim_idle ? fm1_idle_level : fm1_anim_level)(f, i < FM1_ANIM_NBTN ? (i < NB ? panel.btn[i] : i) : i,
                                                                          web_anim_glow);
    return lv;
}

/* for the test and the bench (web/emu/emu_test.mjs), a heavy song straight into the tracks (as hostsim's
 * renders set them): T1 FM6 PAD, T2 PHYS DRONE STRING (SYMP), T3 GRAIN CLOUD PAD, 4-note chords on all 16
 * steps, held; T4 DRUM, kick snare and hats. PLAY is the page's (or the test's) to press */
EXPORT void web_test_heavy(void)
{
    static const uint8_t ENG[3][2] = {{12, 4}, {ENGI_PHYS, 7}, {8, 0}};
    static const uint8_t CH[4][4] = {{48, 55, 60, 64}, {45, 52, 57, 60}, {41, 48, 53, 57}, {43, 50, 55, 59}};
    uint32_t p, i;
    for (p = 0; p < 3u; p++) {
        track_t *t = &trk[p];
        host_preset(t, ENG[p][0], ENG[p][1]);
        t->p[P_VOICE] = V_POLY;
        t->p[P_SUS] = 127;
        for (i = 0; i < 16u; i++) {
            uint8_t n[4], k;
            for (k = 0; k < 4u; k++)
                n[k] = (uint8_t)(CH[i / 4u][k] + 12u * p);
            put_step(t, i, 4, n, ST_NOTE, 0);
        }
    }
    host_drums(&trk[3]);
    for (i = 0; i < 16u; i++) {
        uint8_t n[4];
        uint32_t k = 0;
        if (i % 4u == 0u)
            n[k++] = 36;
        if (i == 4u || i == 12u)
            n[k++] = 38;
        n[k++] = i % 2u ? 42 : 46;
        put_step(&trk[3], i, k, n, ST_NOTE, 0);
    }
    ui.force = 1;
}

EXPORT uint32_t web_test_user_preset(uint32_t i) { return (uint32_t)up_used(i); }

EXPORT uint16_t *web_screen(void) { return web_fb; }
EXPORT uint32_t web_screen_draws(void) { return web_draws; }
EXPORT uint32_t web_now_ms(void) { return fm1_ms; }
EXPORT uint32_t web_playing(void) { return song.playing; }
EXPORT uint32_t web_sample_rate(void) { return FS; }

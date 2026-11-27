/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments
 * The design (the device clock driven by the audio the worklet asks for, the exports below, the saved
 * flash handed back before boot) follows X0X's browser build, web/emu/x0x_web.c of charlesvestal/fm1-x0x
 * (GPL-3.0), a Felucca fork. */
/* teensy-fm1: BBL Jam Box (a Felucca fork) on the Teensy 4.1. Derived from its web/emu/felucca_web.c (release/0.6,
 * f057a4f); the changes are marked "teensy-fm1:", the same as ports/felucca's. */
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
 * The sample memory reads straight from the constant (nothing recorded into it yet). */
#define NOR_SIZE 0x100000u
#ifdef TEENSY_NOR_ADDR
/* teensy-fm1: a slot ROM: the 1 MiB is real flash in this ROM's slot, below its tag (build.rs), read in place as on
 * the FM-1 and written by the Teensy's flash writer (src/nor.rs: teensy_nor_erase / teensy_nor_prog, which also mirror
 * every written sector to the card's DATA/<STEM>.NOR; the menu copies that back when it loads the ROM into a slot). */
#define nor_base ((const uint8_t *)TEENSY_NOR_ADDR)
extern int32_t teensy_nor_erase(uint32_t off);
extern int32_t teensy_nor_prog(uint32_t off, const uint8_t *src, uint32_t n);
extern uint32_t teensy_nor_written(void);
#else
#define NOR_SLOTS 8u
static const uint8_t nor_base[NOR_SIZE] = {[0 ... NOR_SIZE - 1] = 0xFF};
static uint8_t nor_slot[NOR_SLOTS][4096];
static int16_t nor_map[NOR_SIZE / 4096u];   /* sector -> slot + 1 (0: not written) */
static uint32_t nor_used;
#endif
#define SMP_XIP(off) (nor_base + (off))   /* eng_sampler.c: the sample memory */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#define __attribute__(x)
#define memset felucca_memset
#define memcpy felucca_memcpy
#define memcmp felucca_memcmp
#include "felucca_tables.h"
#include "../../firmware/src/libc.c"
#undef memset
#undef memcpy
#undef memcmp
static struct { volatile uint32_t notes, buttons; } fm1_in;
static uint32_t fm1_ticks(void);                    /* teensy-fm1: 0.9 fx.c times its parts (below, the clock) */
#include "../../firmware/src/core.h"
#include "../../firmware/src/engines.c"
#include "../../firmware/src/params.c"
#include "../../firmware/src/mod.c"
#include "../../firmware/src/voice.c"
#include "../../firmware/src/fx.c"
static void fm1_delay_ms(uint32_t ms) { (void)ms; }
#include "../../firmware/src/usb.c"
#include "../../firmware/src/midi_uart.c"
#include "../../firmware/src/seq.c"
#ifdef TEENSY_NOR_ADDR
#include "../../firmware/src/trec.c"                /* teensy-fm1 fork: track recording (docs/TRACK_REC.md) */
#endif

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
static const int8_t FM1_KEYMAP[6][FM1_NCOL] = {      /* as hal/fm1_input.h: key id at (column, row bit) */
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 5, 11,  4, 10,  3,  9,  2,  8, -1, -1, -1},
    {34, 35, 36, 37, 38, 40, 39, 13,  7,  6, 12},
    {23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33},
    { 0,  1, 15, 14, 17, 16, 19, 18, 20, 21, 22},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
};
static uint8_t fm1_led_k2[FM1_NCOL], fm1_led_k4[FM1_NCOL];
static uint8_t fm1_led[FM1_NCOL], fm1_led_dim[FM1_NCOL], fm1_led_breath[FM1_NCOL], fm1_led_mid[FM1_NCOL];
static uint32_t web_dim_lo;
static void fm1_led_dim_level(uint32_t lo) { web_dim_lo = lo; }
/* the power-on LED sweep (hal/fm1_input.h fm1_led_anim_start, the picture hal/fm1_led_anim.h): its frame from the
 * clock (a scan frame 1.1 ms), the page draws each LED's level (web_anim_levels); at its end, or on a key or a button
 * down, the last picture (the glow everywhere, or dark) and the UI's LEDs from the next frame, as the scan does */
#include "../../firmware/hal/fm1_led_anim.h"
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
#include "../../firmware/src/icons.c"
#include "../../firmware/src/ui_graph.c"
#include "../../firmware/src/ui_gb.c"
#include "../../firmware/src/ui_draw.c"
#include "../../firmware/src/ui_input.c"
#include "../../firmware/src/ui_buttons.c"          /* the page buttons' taps */

/* ------------------------------------------------------------ the flash --- */
static uint32_t web_flash_writes;
static uint8_t flash_ok;
#ifdef TEENSY_NOR_ADDR
static int st_read(uint32_t off, void *dst, uint32_t n)
{
    if (off >= NOR_SIZE || n > NOR_SIZE - off)
        return -1;
    memcpy(dst, nor_base + off, n);
    return 0;
}
static int st_erase(uint32_t off)
{
    if (off >= NOR_SIZE || 4096u > NOR_SIZE - off || teensy_nor_erase(off))
        return -8;
    web_flash_writes++;
    return 0;
}
static int st_prog(uint32_t off, const void *src, uint32_t n)       /* NOR: a write only clears bits (so does the chip) */
{
    if (off >= NOR_SIZE || n > NOR_SIZE - off || teensy_nor_prog(off, src, n))
        return -8;
    web_flash_writes++;
    return 0;
}
#else
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
#endif
static uint32_t irq_save(void) { return 0; }
static void irq_restore(uint32_t f) { (void)f; }
static uint32_t fl_jedec_ram(void) { return 0x856014u; }             /* the FM-1's part (project.c) */
#define FL_JEDEC_OK(id) ((id) == 0x856014u)                          /* as hal/fm1_flash_map.h (FM-1) */
#define FM1_OCT_BOTH 3u                                              /* OCT- + OCT+, as hal/fm1_input.h (FM-1) */
static void fl_plain_window_init(void) {}
#define FL_FAR(fn) (fn)
#include "../../firmware/src/storage.c"
#include "../../firmware/src/persist.c"            /* the device's settings */
#include "../../firmware/src/gb_store.c"         /* the groovebox's sounds and kits */
#ifdef TEENSY_NOR_ADDR
#include "../../firmware/src/trec_ui.c"             /* teensy-fm1 fork: the REC pages */
#endif

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
/* the second core (hal/fm1_cpu1.h): none here, one core whatever MENU > 2ND CORE says (fx.c mix_block's C1_ON() is 0
 * without felucca.c's macros) */
uint32_t _c1_start[1], _c1_end[1], _c1_load[1];
static uint8_t fm1_c1_on, fm1_c1_gave_up;
static void fm1_cpu1_halt(void) {}
static int fm1_cpu1_start(void) { return -1; }
static void fm1_guard_pc_open(void) {}
static void fm1_supply_stock(void) {}                /* (hal/fm1_sys.h: the chip's supply; none here) */
static uint32_t fm1_vdc14(void) { return 0; }
static uint32_t fm1_sysvdd(void) { return 0; }
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
#ifdef TEENSY_NOR_ADDR
    trec_defaults();                      /* teensy-fm1 fork: the REC pages (the last jam may have its own) */
#endif
    jam_boot();                           /* gb_store.c: the last jam back */
    kb_boot_hold = 1;                     /* main.c (#137): the keys silent under the splash */
    audio_init();
    boot_leds();                          /* main.c: the power-on LED sweep, from the scan's start */
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
    ui_leds();
#ifdef TEENSY_NOR_ADDR
    trec_poll();                          /* teensy-fm1 fork: takes done */
#endif
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

#ifdef TEENSY_NOR_ADDR
/* teensy-fm1 fork: the serial 'R' report of the track recording (trec.c): the line in, each track's REC, each R slot */
static uint32_t tr_put(char *o, uint32_t at, uint32_t cap, const char *s)
{
    while (*s && at + 1u < cap)
        o[at++] = *s++;
    o[at] = 0;
    return at;
}
static uint32_t tr_num(char *o, uint32_t at, uint32_t cap, const char *name, int32_t v)
{
    char nb[16];
    fmt_int(nb, v);
    at = tr_put(o, at, cap, name);
    at = tr_put(o, at, cap, nb);
    return tr_put(o, at, cap, " ");
}
EXPORT uint32_t teensy_trec_info(char *o, uint32_t cap)
{
    uint32_t at = 0, k;
    at = tr_put(o, at, cap, "TREC ");
    at = tr_num(o, at, cap, "line=", (int32_t)teensy_line_in_state());
    at = tr_num(o, at, cap, "in_ok=", in_ok);
    at = tr_num(o, at, cap, "in_peak=", trec_in_pk);
    at = tr_num(o, at, cap, "pool=", (int32_t)trec_pool_n);
    at = tr_num(o, at, cap, "playing=", song.playing);
    at = tr_num(o, at, cap, "rec=", song.rec);
    at = tr_num(o, at, cap, "mst=", (int32_t)mst_step);
    at = tr_num(o, at, cap, "sel=", song.sel + 1);
    at = tr_num(o, at, cap, "master_q12=", (int32_t)song.master_q12);
    at = tr_num(o, at, cap, "knob=", master_knob / 16);
    at = tr_num(o, at, cap, "usb_fixed=", fx_usb_fixed);
    at = tr_num(o, at, cap, "out_peak=", trec_out_pk >> OUT_SHIFT);
    trec_out_pk = 0;
    at = tr_put(o, at, cap, "\r\n");
    trec_in_pk = 0;
    for (k = 0; k < NTRK; k++) {
        const trec_set_t *s = &trec_set[k];
        const trec_t *r = &trec[k];
        int32_t pk = 0;
        uint32_t i;
        for (i = 0; smp_tab[k].w && i < GB_WAVE; i++)   /* (the waveform's columns: reading the PSRAM back stalls) */
            pk = trec_pk[k][i] * 128 > pk ? trec_pk[k][i] * 128 : pk;
        at = tr_num(o, at, cap, "  T", (int32_t)k + 1);
        at = tr_num(o, at, cap, "arm=", s->arm);
        at = tr_num(o, at, cap, "start=", s->start);
        at = tr_num(o, at, cap, "len=", s->len);
        at = tr_num(o, at, cap, "src=", s->src);
        at = tr_num(o, at, cap, "gain=", s->gain);
        at = tr_num(o, at, cap, "st=", r->st);
        at = tr_num(o, at, cap, "pos=", (int32_t)r->pos);
        at = tr_num(o, at, cap, "want=", (int32_t)r->want);
        at = tr_num(o, at, cap, "skip=", (int32_t)r->skip);
        at = tr_num(o, at, cap, "src_peak=", trec_src_pk[k]);
        at = tr_num(o, at, cap, "| R_len=", (int32_t)smp_tab[k].len);
        at = tr_num(o, at, cap, "R_peak=", pk);
        {                                          /* (the sound: what a machine switch keeps, INPUT's MODE) */
            const track_t *t = &trk[k];
            uint32_t j, nv = 0;
            for (j = 0; j < NVOICE; j++)
                nv += t->v[j].active ? 1u : 0u;
            at = tr_num(o, at, cap, "| mach=", (int32_t)trk_mach(t));
            at = tr_num(o, at, cap, "eng=", t->engine);
            at = tr_num(o, at, cap, "e0=", t->p[P_E0]);
            at = tr_num(o, at, cap, "e1=", t->p[P_E1]);
            at = tr_num(o, at, cap, "nv=", (int32_t)nv);
            at = tr_num(o, at, cap, "cut=", t->p[P_FLT_CUT]);
            at = tr_num(o, at, cap, "res=", t->p[P_FLT_RES]);
            at = tr_num(o, at, cap, "fea=", t->p[P_FE_AMT]);
            at = tr_num(o, at, cap, "ktr=", t->p[P_FLT_KTR]);
            at = tr_num(o, at, cap, "ld_flt=", t->p[P_LD_FLT]);
            at = tr_num(o, at, cap, "ld_amp=", t->p[P_LD_AMP]);
            at = tr_num(o, at, cap, "ed_flt=", t->p[P_ED_FLT]);
            at = tr_num(o, at, cap, "mod=", MOD(t).on);
            at = tr_num(o, at, cap, "atk=", t->p[P_ATK]);
            at = tr_num(o, at, cap, "sus=", t->p[P_SUS]);
            at = tr_num(o, at, cap, "rel=", t->p[P_REL]);
            at = tr_num(o, at, cap, "sat=", t->p[P_LN0]);
            at = tr_num(o, at, cap, "drv=", t->p[P_FLT_DRV]);
        }
        at = tr_put(o, at, cap, "\r\n");
        trec_src_pk[k] = 0;
    }
    return at;
}
#endif
/* the flash: erased, then the page's saved sectors (web_nor), then boot */
EXPORT uint32_t web_nor_size(void) { return NOR_SIZE; }
#ifdef TEENSY_NOR_ADDR
EXPORT void web_nor_erase(void) {}                /* teensy-fm1: the flash keeps what was saved */
EXPORT uint32_t teensy_nor_sectors(void) { return teensy_nor_written(); }
#else
EXPORT void web_nor_erase(void) { memset(nor_map, 0, sizeof nor_map); nor_used = 0; }   /* teensy-fm1 */
EXPORT uint32_t teensy_nor_sectors(void) { return nor_used; }
#endif
/* teensy-fm1: the screen, and how many draws have touched it, for the Teensy's display link */
EXPORT const uint16_t *teensy_fb(void) { return web_fb; }
EXPORT uint32_t teensy_draws(void) { return web_draws; }
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
/* the sample uploader wrote the sample memory (nor 0xA0000..): read its table again */
EXPORT void web_smp_scan(void) { smp_scan(); }
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
/* the power-on sweep (or 1.2's idle animation) running: each LED's level now (hal/fm1_led_anim.h: 0..64 /64 of lit, | 0x80 the glow under
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

EXPORT uint16_t *web_screen(void) { return web_fb; }
EXPORT uint32_t web_screen_draws(void) { return web_draws; }
EXPORT uint32_t web_now_ms(void) { return fm1_ms; }
EXPORT uint32_t web_playing(void) { return song.playing; }
EXPORT uint32_t web_sample_rate(void) { return FS; }

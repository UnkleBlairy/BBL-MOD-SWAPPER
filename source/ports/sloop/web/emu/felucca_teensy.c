/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments
 * The design (the device clock driven by the audio the worklet asks for, the exports below, the saved
 * flash handed back before boot) follows X0X's browser build, web/emu/x0x_web.c of charlesvestal/fm1-x0x
 * (GPL-3.0), a Felucca fork. */
/* teensy-fm1: SLOOP (isod89/sloop-fm1 bac9c05, 2.8.5, a Felucca fork) on the Teensy 4.1. SLOOP has no browser build,
 * so this is ports/felucca's Teensy file (from Felucca's web/emu/felucca_web.c) with SLOOP's sources, boot and main
 * loop (firmware/src/felucca.c's order, main.c fm1_main). The changes from Felucca's are marked "teensy-fm1:".
 * One core: FELUCCA_CPU1 0 (SLOOP's own one-core build, as 2.5; tests/cpu1_host.c runs the render inline). */
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
#define FELUCCA_CDC 0                    /* teensy-fm1: no console */
#endif
#define FELUCCA_FLASH 1
#define FELUCCA_OTA 0
#define FELUCCA_UAC 0
#define FELUCCA_UART 0
#define FELUCCA_CPU1 0                   /* teensy-fm1: one core */
#define FELUCCA_ARRANGER 1               /* teensy-fm1: as src/felucca.c */
#define FELUCCA_ID "FM-1_900"
#ifndef FELUCCA_VERSION
#define FELUCCA_VERSION "WEB"
#endif

#define NOR_SIZE 0x100000u
#define fm1_xip_ptr fm1_xip_ptr_device
#include "../../firmware/hal/fm1_xip.h"
#undef fm1_xip_ptr
#ifdef TEENSY_NOR_ADDR
/* teensy-fm1: a slot ROM: the 1 MiB is real flash in this ROM's slot, below its tag (build.rs), read in place as on
 * the FM-1 (fm1_xip.h: samples, user presets, the FM6 bank) and written by the Teensy's flash writer (src/nor.rs:
 * teensy_nor_erase / teensy_nor_prog, which also mirror every written sector to the card's DATA/SLOOP.NOR; the menu
 * copies that back when it loads the ROM into a slot). As the Jam Box's (ports/jambox). */
#define nor_base ((const uint8_t *)TEENSY_NOR_ADDR)
extern int32_t teensy_nor_erase(uint32_t off);
extern int32_t teensy_nor_prog(uint32_t off, const uint8_t *src, uint32_t n);
extern uint32_t teensy_nor_written(void);
static const uint8_t *fm1_xip_ptr(uint32_t off) { return nor_base + (off < NOR_SIZE ? off : 0); }
#else
/* teensy-fm1: the 1 MiB SPI NOR can't sit in the Teensy's RAM. It reads as erased flash (a 1 MiB constant in the
 * Teensy's program flash) except the 4 KiB sectors written since boot, which get a RAM copy (NOR_SLOTS of them).
 * The user sample slots read straight from the constant (no user samples yet). */
#define NOR_SLOTS 8u
static const uint8_t nor_base[NOR_SIZE] = {[0 ... NOR_SIZE - 1] = 0xFF};
static uint8_t nor_slot[NOR_SLOTS][4096];
static int16_t nor_map[NOR_SIZE / 4096u];   /* sector -> slot + 1 (0: not written) */
static uint32_t nor_used;
/* teensy-fm1: SLOOP reads flash objects in place (fm1_xip.h: samples, user presets, the FM6 bank): from the image
 * here, the RAM copy of a sector written since boot if there is one (an object spanning sectors written this
 * session reads its first sector only; saves to the SD card replace this) */
static uint8_t *nor_sector(uint32_t sec, int make);
static const uint8_t *fm1_xip_ptr(uint32_t off)
{
    const uint8_t *p;
    if (off >= NOR_SIZE)
        return nor_base;
    p = nor_sector(off / 4096u, 0);
    return p ? p + off % 4096u : nor_base + off;
}
#endif
#define ST_XIP_PTR(off) fm1_xip_ptr(off)
/* teensy-fm1: hostsim.c's timing (used by its own test main, never on the Teensy) wants clock_gettime */
#include <time.h>
#ifndef CLOCK_MONOTONIC
#define CLOCK_MONOTONIC 1
static int clock_gettime(int c, struct timespec *t) { (void)c; t->tv_sec = 0; t->tv_nsec = 0; return 0; }
#endif
/* teensy-fm1: tests/cpu1_host.c's second-core thread (CORES=2) is never started here (one core: the jobs run inline);
 * newlib declares the pthread types but not the calls */
#include <stdlib.h>
void abort(void) { for (;;) ; }         /* teensy-fm1: cpu1_host.c's "two jobs at once" (never with one core) */
#define getenv(name) ((char *)0)         /* teensy-fm1: cpu1_host.c's CORES / CHAOS: unset, one core inline */
#define pthread_mutex_lock(m) ((void)(m), 0)
#define pthread_mutex_unlock(m) ((void)(m), 0)
#define pthread_mutex_init(m, a) ((void)(m), 0)
#define pthread_cond_wait(c, m) ((void)(c), (void)(m), 0)
#define pthread_cond_signal(c) ((void)(c), 0)
#define pthread_cond_init(c, a) ((void)(c), 0)
#define pthread_create(t, a, f, x) ((void)(t), (void)(f), -1)
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
/* teensy-fm1: SLOOP's LEDs (hal/fm1_input.h): lit, the glow, the backlight (labels in the dark) and its pulse */
static uint8_t fm1_led[FM1_NCOL], fm1_led_dim[FM1_NCOL], fm1_led_bg[FM1_NCOL];
static volatile uint16_t fm1_led_bg_ns;
static void fm1_led_key(uint32_t id, int on)
{
    uint32_t c, r;
    for (c = 0; c < FM1_NCOL; c++)
        for (r = 0; r < 6u; r++)
            if (FM1_KEYMAP[r][c] == (int8_t)id) {
                if (on) fm1_led[c] |= (uint8_t)(1u << r);
                else fm1_led[c] &= (uint8_t)~(1u << r);
            }
}
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
#include "../../firmware/src/ui_song.c"
#include "../../firmware/src/ui_studio.c"
#include "../../firmware/src/icons.c"
#include "../../firmware/src/ui_draw.c"
#include "../../firmware/src/ui_vis.c"
#include "../../firmware/src/ui_layers.c"
#include "../../firmware/src/ui_menu.c"
#include "../../firmware/src/ui_input.c"

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
static void fl_plain_window_init(void) {}
#define FL_FAR(fn) (fn)
#include "../../firmware/hal/fm1_flash_id.h"   /* teensy-fm1: FL_JEDEC_OK (project.c) */
#include "../../firmware/src/storage.c"
#include "../../firmware/src/upreset.c"
#include "../../firmware/src/project.c"

/* ------------------------------------------------- main.c and its HAL --- */
/* teensy-fm1: SLOOP's main.c wants its crash record (hal/fm1_irq.h), the stacks and RAM regions of app.ld, the guard
 * registers and the second core (hal/fm1_cpu1.h): none of it runs here (web_power_on and web_frame below are
 * fm1_main's boot and loop); only names for it to compile against */
typedef struct {
    uint32_t magic, count, vec, pc, rets, emu, dbg, sp, psr, icfg, uptime_ms;
    uint32_t etm[4];
    uint32_t early;
} fm1_crash_t;
#define FM1_CRASH_MAGIC 0x43525348u
static fm1_crash_t fm1_crash;
static struct { uint8_t p3_rst, wdt_con; uint32_t rst_src; } fm1_boot;
uint32_t _data_start[1], _data_end[1], _data_load[1], _bss_start[1], _bss_end[1];
uint32_t _pool_start[1], _pool_end[1], _rt_start[1], _rt_end[1], _rt_load[1];
uint32_t _c1_start[1], _c1_end[1], _c1_load[1], _lr_start[1], _lr_end[1];
char _ustack_lo[4], _ustack_top[4], _sstack_lo[4], _sstack_top[4];
static uint32_t fm1_sp(void) { return 0; }
static uint32_t fm1_ssp(void) { return 0; }
static uint32_t fm1_usp(void) { return 0; }
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
static void fm1_supplies_stock(void) {}
enum { FM1_GUARD_STACK = 1, FM1_GUARD_WRITE = 2, FM1_GUARD_BUS = 4, FM1_GUARD_PC = 8 };
/* the second core: none (cpu1.c's FELUCCA_CPU1=0 part; its job API is tests/cpu1_host.c's, inline) */
static uint8_t fm1_c1_on;
static struct { uint32_t bad; } fm1_c1_mb;
static void fm1_cpu1_bar(void) {}
static void fm1_cpu1_halt(void) {}
enum { CPU1_OFF, CPU1_NOSTART, CPU1_FAILED, CPU1_OK };   /* cpu1.c */
static uint8_t cpu1_state = CPU1_OFF;
static uint32_t cpu1_timeouts;
static void cpu1_boot(void) {}
static const char *cpu1_label(void) { return "1 CORE"; }
static void cpu1_supplies(char *b) { b[0] = 0; }
static uint32_t cpu1_stack_free(void) { return 0; }
typedef struct { uint8_t pending; } bootguard_stub_t;
#if FELUCCA_CDC
static void cdc_task(void) {}
#endif
/* main.c's calls into the USB driver (registers): nothing here (usb.c's own functions stay as they are) */
#define usb_poll() ((void)0)
#define usb_start() ((void)0)
#define usb_retry(ms) ((void)(ms))
#define usb_detach() ((void)0)
#include "../../firmware/src/splash.c"
#include "../../firmware/src/main.c"
#undef usb_poll
#undef usb_start
#undef usb_retry
#undef usb_detach

/* --------------------------------------------------------- the device --- */
static uint32_t web_booted, web_boot_ms, web_last_frame;

/* main.c fm1_main's boot, up to the main loop (no USB, no UART, no panel setup, no update paths) */
static void web_power_on(void)
{
    persist_boot();
    settings_init();
    lcd_init();
    sloop_splash();                       /* main.c: the SLOOP logo (splash.c) */
    memset(&felucca_dbg, 0, sizeof felucca_dbg);
    felucca_dbg.magic = DBG_MAGIC;
    fm1_input_init();
    fm1_adc_init();
    panel_init();
    felucca_init();
    cpu1_boot();
    audio_init();
    web_boot_ms = fm1_ms + 930u;          /* main.c: 30 + 900 ms with the logo before the loop */
    web_booted = 1;
}

/* one pass of main.c fm1_main's loop (what the Teensy has: no USB, editor, UBOOT, console) */
static void web_frame(void)
{
    static int32_t knob = 512 * 16;
    int32_t b = fm1_adc_read(FM1_ADC_BATT), a = fm1_adc_read(FM1_ADC_MASTER);
    if (b > 0)
        song.batt_raw = song.batt_raw ? song.batt_raw + (b - song.batt_raw) / 32 : b;
    if (a >= 0) {                                         /* main.c: the MASTER pot, and the LIVE page's level */
        uint32_t k10;
        knob += (a * 16 - knob) / 8;
        k10 = (uint32_t)(knob / 16);
        if (ed_master) {
            if (ed_master_k == 0xFFFFu)
                ed_master_k = (uint16_t)k10;
            if (k10 + 48u < ed_master_k || k10 > ed_master_k + 48u)
                ed_master = 0;
            else
                k10 = (ed_master - 1u) * 1023u / 126u;
        }
        song.master_q12 = (k10 * k10) >> 8;
    }
    felucca_dbg.ui_frames++;
    felucca_dbg.page = ui.page;
    felucca_dbg.home = ui.home;
    ui_input();
    ui_leds();
    ui_draw();
    autosave_tick();
    sections_flush();
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
        lcd_fill(0, 0, 240, 240, C_BLACK);
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
#ifdef TEENSY_NOR_ADDR
EXPORT void web_nor_erase(void) {}                /* teensy-fm1: the flash keeps what was saved */
EXPORT uint32_t teensy_nor_sectors(void) { return teensy_nor_written(); }
#else
EXPORT void web_nor_erase(void) { memset(nor_map, 0, sizeof nor_map); nor_used = 0; }   /* teensy-fm1 */
EXPORT uint32_t teensy_nor_sectors(void) { return nor_used; }
#endif
/* teensy-fm1: the Sample Lab (src/hooks/sloop.rs) wrote user sample slot k (0..3): read its header again */
EXPORT void teensy_smp_user_scan(uint32_t k) { if (k < SMP_USER_SLOTS) smp_user_scan(k); }
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
EXPORT uint32_t teensy_leds(uint8_t *out)   /* teensy-fm1: lit, glow | backlight, (no breath / mid levels) */
{
    uint32_t c;
    memset(out, 0, 45);
    memcpy(out, fm1_led, 11);
    for (c = 0; c < 11u; c++)
        out[11 + c] = fm1_led_dim[c] | fm1_led_bg[c];
    return 45;
}
EXPORT uint32_t teensy_ui_frames(void) { return felucca_dbg.ui_frames; }
/* teensy-fm1: the firmware menu (src/main.rs), drawn into the screen before any firmware boots, in the look of the
 * terminal's BBL MOD SWAPPER splash: a chunky bold block font (6 x 7 dots, M and W 7 wide; bit w-1 the leftmost dot),
 * hot pink on near black. A title, n entries (name, and a detail on the right) with entry sel in a pink box, up to 3
 * shown (scrolled to keep sel in view), a hint */
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
EXPORT uint16_t *web_screen(void) { return web_fb; }
EXPORT uint32_t web_screen_draws(void) { return web_draws; }
EXPORT uint32_t web_now_ms(void) { return fm1_ms; }
EXPORT uint32_t web_playing(void) { return song.playing; }
EXPORT uint32_t web_sample_rate(void) { return FS; }

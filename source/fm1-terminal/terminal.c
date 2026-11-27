/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments (felucca-bsp, examples/hwtest: the base)
 * Teensy terminal changes: teensy-fm1 project, 2026 (MPL-2.0, as the file it derives from). */
/* FM-1 Teensy terminal: the FM-1 as the screen, keys, knobs, LEDs and audio out of a Teensy 4.1 that runs the
 * firmware. Derived from felucca-bsp's examples/hwtest (same BSP calls, boot guard, update entry).
 *
 *   USB    CDC serial (1209:000B), packets A5 5A type len16 payload (~/teensy-fm1/docs/LINK_PROTOCOL.md):
 *          in:  'A' audio (128 stereo i16 frames), 'L' screen spans (RLE), 'D' LEDs, 'Z' sync
 *          out: 'I' every 4 ms: keys, buttons (raw matrix), encoder steps, ADCs, audio played / underruns / FIFO,
 *               'R' when it wants the whole screen (boot, or after a gap)
 *          The installer still replaces it over USB-MIDI like any firmware (the update entry, as hwtest).
 *   audio  a 6144-frame FIFO into ALNK0, Felucca's own level (Q15 << 7); it starts once 1024 frames wait
 *   input  felucca-bsp's scan (TIMER5, 10 kHz), as Felucca reads it
 *   screen spans straight to the ST7789 at 12 MHz, as Felucca drives it (lcd.c)
 *
 * USB data in: usb.c polls at 2 kHz and takes one 64-byte packet a poll (128 KB/s, less than the audio alone), so
 * TIMER5 also takes the CDC endpoint on every tick (10 kHz, up to 640 KB/s) and moves it into a 16 KB ring. A full
 * ring leaves the packet in the endpoint (NAK): the host waits, nothing is lost.
 *
 * Two boots in a row that never reach the main loop's "healthy" point drop into UBOOT, as hwtest. */
#include <stdint.h>
#define FELUCCA_CDC 1
#define FELUCCA_OTA 1             /* the update entry: the installer can replace the terminal over USB-MIDI */
#define FELUCCA_ID "FM-1_1300"    /* the identity the installer sees, == the package's */
#define FELUCCA_USB_PID 0x000B    /* not Felucca's 0x0001: the PC tells the FM-1 and the Teensy apart */
#define RING_PUBLISH() __asm__ volatile("" ::: "memory")
#include "fm1_time.h"
#include "fm1_sys.h"
#include "fm1_irq.h"
#include "fm1_guard.h"
#include "fm1_timer.h"
#include "fm1_input.h"
#include "fm1_adc.h"
#include "fm1_lcd_hw.h"
#include "fm1_audio.h"
#include "fm1_flash.h"
#include "libc.c"
#include "usb.c"
#include "ota.c"

extern uint32_t _data_start[], _data_end[], _data_load[], _bss_start[], _bss_end[];
extern uint32_t _rt_start[], _rt_end[], _rt_load[];   /* code that runs from RAM: the flash driver (fm1_flash.h) */

static volatile uint32_t ms;
static uint32_t splash_until;                  /* ms: the Teensy's screen is not drawn before this (packet 'L', splash) */
static uint32_t boots_ok;
static struct { uint32_t magic, pending, failed, rst, boots; } guard __attribute__((section(".noinit")));   /* rst: this start's reset reason (P3_RST_SRC), boots: starts since power-up (both in the 'I' report, bytes 27-28) */

/* ---- USB data in: the endpoint, drained every TIMER5 tick into rx ---- */
#define RX_N 16384u
static uint8_t rx[RX_N];
static volatile uint32_t rx_w, rx_r;
static volatile uint32_t st_ticks, st_rxb, st_full;   /* stats ('T' each second): ticks, bytes in, ticks with rx full */
static void rx_take(void)                       /* TIMER5 ISR */
{
    st_ticks++;
    if (!usb.up || !usb.config)
        return;
    ep3_rx();                                   /* one packet, if the endpoint has one and cdc_in has room */
    while (ci_r != ci_w && rx_w - rx_r < RX_N) {
        rx[rx_w % RX_N] = cdc_in[ci_r % CI_N];
        ci_r++;
        rx_w++;
        st_rxb++;
    }
    if (rx_w - rx_r >= RX_N)
        st_full++;
}

void fm1_timer5_irq(void)                       /* 10 kHz: key scan, ms, USB at 2 kHz, CDC data in every tick */
{
    static uint32_t sub, last, acc;
    uint32_t t = fm1_ticks();
    fm1_timer5_ack();
    fm1_input_tick();
    acc += t - last;
    last = t;
    while (acc >= 1000u * FM1_TICKS_PER_US) {
        acc -= 1000u * FM1_TICKS_PER_US;
        ms++;
    }
    if (++sub == 5u) {
        sub = 0;
        usb_poll();
    }
    rx_take();
}
extern void isr_timer5(void);

/* ---- audio: FIFO -> ALNK0 ---- */
#define HALF_FRAMES 128u
#define HALF_WORDS (HALF_FRAMES * 2u)
#define AF_N 6144u                              /* frames */
#define PRIME 1024u                             /* frames waiting before playing starts (again) */
static int32_t abuf[2u * HALF_WORDS] __attribute__((aligned(4)));
static int16_t afifo[AF_N][2];
static volatile uint32_t af_w, af_r;            /* written by the main loop, read by the ISR */
static volatile uint32_t played, underruns;
static volatile uint8_t primed;
static uint8_t link_gen;                        /* the last 'Z' */

void fm1_alnk0_irq(void)                        /* via isr_alnk0 (fm1_isr.S) */
{
    uint8_t p = fm1_audio_pending();
    fm1_audio_ack_aux(p);
    if (p & FM1_AUDIO_HALF) {
        int32_t *o = &abuf[fm1_audio_free_half() * HALF_WORDS];
        uint32_t i, r = af_r, w = af_w;
        if (!primed && w - r >= PRIME)
            primed = 1;
        for (i = 0; i < HALF_FRAMES; i++) {
            if (primed && r != w) {
                const int16_t *s = afifo[r % AF_N];
                o[2u * i] = (int32_t)s[0] * 128;           /* Felucca's DAC word: Q15 << 7 */
                o[2u * i + 1u] = (int32_t)s[1] * 128;
                r++;
            } else {
                o[2u * i] = o[2u * i + 1u] = 0;
                if (primed) {
                    primed = 0;
                    underruns++;
                }
            }
        }
        played += r - af_r;
        af_r = r;
        fm1_audio_ack_half();
    }
}
extern void isr_alnk0(void);

/* ---- screen: ST7789 240x240 on SPI1, as Felucca's lcd.c ---- */
#define C(x) ((uint16_t)((((x) >> 8) | ((x) << 8)) & 0xFFFFu))
#define LCD_BAUD 4u                             /* 12 MHz */
static uint8_t lcd_small[16];
static uint8_t lcd_busy;
static void lcd_spin(uint32_t n)
{
    for (volatile uint32_t i = 0; i < n; i++)
        ;
}
static void lcd_sync(void)
{
    if (!lcd_busy)
        return;
    fm1_lcd_wait();
    fm1_lcd_deselect();
    lcd_busy = 0;
}
static void lcd_cmd(uint8_t c)
{
    lcd_sync();
    fm1_lcd_send_cmd(c);
    fm1_lcd_wait();
    fm1_lcd_deselect();
}
static void lcd_data(const void *p, uint32_t n)
{
    if (!n)
        return;
    lcd_sync();
    fm1_lcd_send_data(p, n);
    lcd_busy = 1;
}
static void lcd_window(uint32_t x0, uint32_t y0, uint32_t x1, uint32_t y1)
{
    lcd_sync();
    lcd_small[0] = 0; lcd_small[1] = (uint8_t)x0; lcd_small[2] = 0; lcd_small[3] = (uint8_t)x1;
    lcd_cmd(0x2A);
    lcd_data(lcd_small, 4);
    lcd_small[4] = 0; lcd_small[5] = (uint8_t)y0; lcd_small[6] = 0; lcd_small[7] = (uint8_t)y1;
    lcd_cmd(0x2B);
    lcd_data(lcd_small + 4, 4);
    lcd_cmd(0x2C);
}
static uint16_t fillbuf[240];
static void lcd_rect(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint16_t c)   /* c in panel byte order */
{
    uint32_t i;
    lcd_sync();
    for (i = 0; i < w; i++)
        fillbuf[i] = c;
    lcd_window(x, y, x + w - 1u, y + h - 1u);
    for (i = 0; i < h; i++)
        lcd_data(fillbuf, w * 2u);
    lcd_sync();
}
static const uint8_t LCD_SEQ[] = {
    0x01, 0, 0x00, 1, 150,                      /* SWRESET, wait */
    0x11, 0, 0x00, 1, 120,                      /* SLPOUT, wait */
    0x3A, 1, 0x55,                              /* RGB565 */
    0x36, 1, 0x00,                              /* top-left origin, RGB */
    0x21, 0,                                    /* INVON: IPS */
    0x13, 0,                                    /* NORON */
};
static void lcd_init(void)
{
    uint32_t r, x;
    fm1_lcd_hw_init();
    lcd_spin(2000000u);
    for (r = 0; r < sizeof LCD_SEQ; r += 2u + LCD_SEQ[r + 1u]) {
        if (LCD_SEQ[r] == 0x00u) {
            lcd_spin(LCD_SEQ[r + 2u] * 25000u);
            continue;
        }
        lcd_cmd(LCD_SEQ[r]);
        for (x = 0; x < LCD_SEQ[r + 1u]; x++)
            lcd_small[x] = LCD_SEQ[r + 2u + x];
        if (LCD_SEQ[r + 1u])
            lcd_data(lcd_small, LCD_SEQ[r + 1u]);
    }
    fm1_lcd_baud(LCD_BAUD);
    lcd_rect(0, 0, 240, 240, 0);
    lcd_cmd(0x29);                              /* DISPON */
}

/* ---- start-up steps and the crash screen: what failed, readable on the panel itself ---- */
static uint8_t lcd_ready;
static const uint16_t HEX3x5[16] = {            /* 3 x 5 digits, row-major, bit 14 = top left */
    0x7B6F, 0x2C97, 0x73E7, 0x73CF, 0x5BC9, 0x79CF, 0x79EF, 0x7249,
    0x7BEF, 0x7BCF, 0x7BED, 0x6BAE, 0x7927, 0x6B6E, 0x79E7, 0x79E4};
static void diag_hex(uint32_t x, uint32_t y, uint32_t v, uint16_t fg)   /* 8 digits, 4-px dots */
{
    uint32_t d, b;
    for (d = 0; d < 8u; d++) {
        uint32_t g = HEX3x5[(v >> (28u - 4u * d)) & 15u];
        for (b = 0; b < 15u; b++)
            if (g & (0x4000u >> b))
                lcd_rect(x + d * 16u + (b % 3u) * 4u, y + (b / 3u) * 4u, 4, 4, fg);
    }
}
static void diag_step(uint32_t k)               /* start-up steps: no longer drawn (they were the green blocks along the
                                                 * top while start-up was being debugged; a crash still shows its red page) */
{
    (void)k;
}

/* 'L': spans queue here (whole packets, u16 length first) and are drawn a span at a time between everything else,
 * each only once the panel has taken the last one: the sound never waits behind a page of screen */
#define LQ_N 32768u
static uint8_t lq[LQ_N];
static uint32_t lq_w, lq_r, lq_left;           /* bytes in, bytes out, bytes of the packet being drawn still queued */
static uint16_t rowbuf[2][240];
static uint32_t spans_bad, row_which;
static uint32_t lq_get(void)
{
    uint32_t c = lq[lq_r % LQ_N];
    lq_r++;
    return c;
}
static int lq_put(const uint8_t *b, uint32_t len)   /* 0: no room (the caller tries again later) */
{
    uint32_t i;
    if (LQ_N - (lq_w - lq_r) < len + 2u)
        return 0;
    lq[lq_w % LQ_N] = (uint8_t)len;
    lq[(lq_w + 1u) % LQ_N] = (uint8_t)(len >> 8);
    for (i = 0; i < len; i++)
        lq[(lq_w + 2u + i) % LQ_N] = b[i];
    lq_w += len + 2u;
    return 1;
}
static void lq_skip(void)                       /* a bad span: drop the rest of its packet */
{
    spans_bad++;
    lq_r += lq_left;
    lq_left = 0;
}
static void lq_clear(void)
{
    lq_r = lq_w;
    lq_left = 0;
}
static void lcd_pump(void)                      /* one span, if the panel is free */
{
    uint32_t y, x, n, k = 0;
    uint16_t *row;
    if (lcd_busy && !(FM1_LCD_SPI_CON & 0x8000u))
        return;                                 /* still sending the last span */
    if (!lq_left) {
        if (lq_w - lq_r < 2u)
            return;
        lq_left = lq_get();
        lq_left |= lq_get() << 8;
        if (!lq_left)
            return;
    }
    if (lq_left < 3u) {
        lq_skip();
        return;
    }
    y = lq_get();
    x = lq_get();
    n = lq_get();
    lq_left -= 3u;
    if (!n || y >= 240u || x + n > 240u) {
        lq_skip();
        return;
    }
    row = rowbuf[row_which];
    while (k < n && lq_left) {
        uint32_t h = lq_get();
        lq_left--;
        if (h & 0x80u) {
            uint32_t r = (h & 0x7Fu) + 1u;
            uint16_t px;
            if (lq_left < 2u)
                break;
            px = (uint16_t)lq_get();            /* bytes as they come: the panel's order */
            px |= (uint16_t)(lq_get() << 8);
            lq_left -= 2u;
            while (r-- && k < n)
                row[k++] = px;
        } else {
            uint32_t l = h + 1u;
            while (l-- && k < n) {
                uint16_t px;
                if (lq_left < 2u)
                    break;
                px = (uint16_t)lq_get();
                px |= (uint16_t)(lq_get() << 8);
                lq_left -= 2u;
                row[k++] = px;
            }
        }
    }
    if (k != n) {
        lq_skip();
        return;
    }
    lcd_window(x, y, x + n - 1u, y);
    lcd_data(row, n * 2u);
    row_which ^= 1u;
}

/* ---- the update (ota.c): as hwtest ---- */
static uint8_t flash_ok;
static uint32_t ota_now_ms(void) { return ms; }
static void ota_idle(void) { fm1_wdt_feed(); }
static int ota_in_area(uint32_t off, uint32_t n) { return FL_IN(off, n, OTA_AREA, OTA_AREA + OTA_AREA_LEN); }
static int ota_erase(uint32_t off)
{
    uint32_t took;
    if (!ota_in_area(off, 0x1000u) || (off & 0xFFFu))
        return -8;
    return fl_erase4k(off, &took);
}
static int ota_prog(uint32_t off, const void *p, uint32_t n)
{
    if (!ota_in_area(off, n))
        return -8;
    return fl_write(off, p, n);
}
static int ota_fread(uint32_t off, void *p, uint32_t n)
{
    uint8_t *d = p;
    while (n) {
        uint32_t k = n > 256u ? 256u : n, f = irq_save();
        int rc = FL_FAR(fl_read_ram)(off, d, k);
        irq_restore(f);
        if (rc)
            return rc;
        off += k;
        d += k;
        n -= k;
    }
    return 0;
}
static void ota_show(uint32_t step, int32_t code)   /* the update's steps as a bar across the bottom */
{
    lcd_rect(0, 200, 240, 24, 0);
    if (step < 9u)
        lcd_rect(0, 200, step * 40u, 24, C(0x07FF));
    else
        lcd_rect(0, 200, 240, 24, code == 1 ? C(0x07E0) : C(0xF800));
}
static void ota_commit(const uint8_t *parm)
{
    guard.pending = 0;
    guard.failed = 0;
    usb_detach();
    fm1_delay_ms(30);
    fm1_audio_stop();
    fm1_enter_update(parm);
}
static void fm1_fault(const fm1_crash_t *c)    /* red, then vec pc rets emu dbg sp in hex; 20 s, then reset */
{
    uint32_t i;
    fm1_audio_stop();
    if (!lcd_ready)
        lcd_init();
    lcd_ready = 1;
    fm1_lcd_backlight(1);
    lcd_rect(0, 20, 240, 220, C(0xF800));
    {
        const uint32_t v[6] = {c->vec, c->pc, c->rets, c->emu, c->dbg, c->sp};
        for (i = 0; i < 6u; i++)
            diag_hex(56, 40u + i * 32u, v[i], 0xFFFFu);
    }
    for (i = 0; i < 200u; i++) {
        fm1_wdt_feed();
        fm1_delay_ms(100);
    }
    fm1_reboot();
}

/* ---- USB data out ---- */
static int send_packet(uint8_t ty, const uint8_t *p, uint32_t n)
{
    uint32_t i;
    if (!cdc.dtr || CO_N - (co_w - co_r) < n + 5u)
        return 0;
    {
        const uint8_t h[5] = {0xA5, 0x5A, ty, (uint8_t)n, (uint8_t)(n >> 8)};
        for (i = 0; i < 5u; i++)
            cdc_out[(co_w + i) % CO_N] = h[i];
    }
    for (i = 0; i < n; i++)
        cdc_out[(co_w + 5u + i) % CO_N] = p[i];
    RING_PUBLISH();
    co_w += n + 5u;
    return 1;
}
static void put32(uint8_t *b, uint32_t v) { b[0] = (uint8_t)v; b[1] = (uint8_t)(v >> 8); b[2] = (uint8_t)(v >> 16); b[3] = (uint8_t)(v >> 24); }
static void put16(uint8_t *b, uint32_t v) { b[0] = (uint8_t)v; b[1] = (uint8_t)(v >> 8); }

/* ---- packets in ---- */
#define PK_MAX 4096u
static uint8_t pk[5u + PK_MAX];
static uint32_t pk_len;
static uint32_t last_rx_ms, link_up;
static uint32_t st_lcd, st_l, st_a, st_loops, st_maxloop;
static int packet(uint8_t ty, const uint8_t *b, uint32_t len)   /* 0: not taken yet (the screen queue is full) */
{
    if (ty == 'A')
        st_a++;
    uint32_t i;
    if (ty == 'A' && len == 512u) {
        uint32_t w = af_w;
        if (AF_N - (w - af_r) >= 128u) {        /* (pacing keeps it from filling: a full FIFO drops the block) */
            for (i = 0; i < 128u; i++) {
                afifo[(w + i) % AF_N][0] = (int16_t)(b[4u * i] | b[4u * i + 1u] << 8);
                afifo[(w + i) % AF_N][1] = (int16_t)(b[4u * i + 2u] | b[4u * i + 3u] << 8);
            }
            RING_PUBLISH();
            af_w = w + 128u;
        }
    } else if (ty == 'L') {
        if ((int32_t)(ms - splash_until) < 0)
            return 1;                           /* the splash stays up: dropped, the whole screen is asked for after */
        if (!lq_put(b, len))
            return 0;
        if (lq_w - lq_r > st_lcd)
            st_lcd = lq_w - lq_r;
        st_l++;
    } else if (ty == 'D' && len >= 45u) {
        for (i = 0; i < FM1_NCOL; i++) {
            fm1_led[i] = b[i];
            fm1_led_dim[i] = b[11u + i];
            fm1_led_breath[i] = b[22u + i];
            fm1_led_mid[i] = b[33u + i];
        }
        fm1_led_dim_level(b[44]);
    } else if (ty == 'Z' && len >= 1u) {        /* sync: an empty FIFO, played from 0, this generation */
        uint32_t f = irq_save();
        af_r = af_w;
        played = 0;
        primed = 0;
        link_gen = b[0];
        irq_restore(f);
    }
    return 1;
}
#ifdef TERMINAL_SELFTEST                         /* emulator profiling: a recorded Teensy stream instead of USB */
#include "selftest_stream.h"                    /* const uint8_t st_stream[]; ST_STREAM_N */
static uint32_t st_pos;
static void selftest_feed(void)
{
    uint32_t f = irq_save();
    while (rx_w - rx_r < RX_N && st_pos < ST_STREAM_N) {
        rx[rx_w % RX_N] = st_stream[st_pos];
        rx_w++;
        st_rxb++;
#ifndef TERMINAL_SELFTEST_ONCE                  /* once: st_pos stays at the end, the feed stops */
        if (++st_pos == ST_STREAM_N)
            st_pos = 0;
#else
        st_pos++;
#endif
    }
    irq_restore(f);
}
#endif
static uint8_t pk_held;
static void rx_service(void)
{
#ifdef TERMINAL_SELFTEST
    selftest_feed();
#endif
    if (pk_held) {
        if (!packet(pk[2], pk + 5u, pk[3] | (uint32_t)pk[4] << 8))
            return;
        pk_held = 0;
        pk_len = 0;
    }
    uint32_t budget = 8192u;                    /* bytes per call: the rest of the loop keeps running */
    while (rx_r != rx_w && budget--) {
        uint8_t c = rx[rx_r % RX_N];
        rx_r++;
        if ((pk_len == 0u && c != 0xA5u) || (pk_len == 1u && c != 0x5Au)) {
            pk_len = c == 0xA5u ? 1u : 0u;
            continue;
        }
        pk[pk_len++] = c;
        if (pk_len >= 5u) {
            uint32_t len = pk[3] | (uint32_t)pk[4] << 8;
            if (len > PK_MAX) {
                pk_len = 0;
            } else if (pk_len == 5u + len) {
                if (!packet(pk[2], pk + 5u, len)) {
                    pk_held = 1;                /* complete, waiting for the screen queue: rx stops (the host waits) */
                    return;
                }
                pk_len = 0;
                if (ms - last_rx_ms > 1000u || !link_up) {   /* the Teensy (back) after a gap: ask for it all */
                    link_up = 1;
                    send_packet('R', 0, 0);
                }
                last_rx_ms = ms;
            }
        }
    }
}

/* waiting for the Teensy: a dark screen and a small mark at the bottom (red: no USB host, amber: no Teensy yet) */
/* the start-up screen (what shows while the FM-1 starts and waits for the Teensy, instead of black): BBL MOD SWAPPER
 * in a chunky bold block font (the letters it needs only), filling the screen; BBL in hot pink. Each row: the glyph's
 * width in dots w, then 7 rows, bit (w - 1) the leftmost dot. Shown for at least SPLASH_MS after power-on */
static const struct { char ch; uint8_t w; uint8_t row[7]; } BOLD7[] = {
    {'B', 6, {0x3E, 0x33, 0x33, 0x3E, 0x33, 0x33, 0x3E}},
    {'L', 6, {0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x3F}},
    {'M', 7, {0x63, 0x77, 0x7F, 0x6B, 0x63, 0x63, 0x63}},
    {'O', 6, {0x1E, 0x33, 0x33, 0x33, 0x33, 0x33, 0x1E}},
    {'D', 6, {0x3E, 0x33, 0x33, 0x33, 0x33, 0x33, 0x3E}},
    {'S', 6, {0x1F, 0x30, 0x30, 0x1E, 0x03, 0x03, 0x3E}},
    {'W', 7, {0x63, 0x63, 0x63, 0x6B, 0x7F, 0x77, 0x63}},
    {'A', 6, {0x1E, 0x33, 0x33, 0x3F, 0x33, 0x33, 0x33}},
    {'P', 6, {0x3E, 0x33, 0x33, 0x3E, 0x30, 0x30, 0x30}},
    {'E', 6, {0x3F, 0x30, 0x30, 0x3E, 0x30, 0x30, 0x3F}},
    {'R', 6, {0x3E, 0x33, 0x33, 0x3E, 0x3C, 0x36, 0x33}},
};
#define SPLASH_MS 3000u
static uint32_t big_line(uint32_t y, const char *t, uint32_t sx, uint32_t sy, uint32_t gap, uint16_t c)
{
    uint32_t n, w = 0, x, i, r, k;
    const char *u;
    for (u = t; *u; u++)
        for (i = 0; i < sizeof BOLD7 / sizeof BOLD7[0]; i++)
            if (BOLD7[i].ch == *u)
                w += BOLD7[i].w * sx + (u[1] ? gap : 0u);
    x = (240u - w) / 2u;
    for (; *t; t++)
        for (i = 0; i < sizeof BOLD7 / sizeof BOLD7[0]; i++)
            if (BOLD7[i].ch == *t) {
                n = BOLD7[i].w;
                for (r = 0; r < 7u; r++)
                    for (k = 0; k < n; k++)
                        if (BOLD7[i].row[r] & (1u << (n - 1u - k)))
                            lcd_rect(x + k * sx, y + r * sy, sx, sy, c);
                x += n * sx + gap;
            }
    return y + 7u * sy;
}
static void splash(void)
{
    uint32_t y;
    lcd_rect(0, 0, 240, 240, C(0x0841));
    y = big_line(4, "BBL", 11, 11, 8, C(0xF8B2));
    y = big_line(y + 6u, "MOD", 11, 11, 8, C(0xFFDF));
    big_line(y + 6u, "SWAPPER", 5, 9, 3, C(0xFFDF));
}

static void show_waiting(uint32_t now)
{
    uint16_t c = !usb.config ? C(0xF800) : ((now >> 9) & 1u) ? C(0xFD20) : C(0x0841);
    lcd_rect(108, 235, 24, 4, c);
}

static uint32_t put_u(char *o, const char *name, uint32_t v)
{
    char t[10];
    uint32_t n = 0, k = 0;
    while (*name)
        o[n++] = *name++;
    do t[k++] = (char)('0' + v % 10u); while ((v /= 10u) && k < 10u);
    while (k)
        o[n++] = t[--k];
    o[n++] = ' ';
    return n;
}
static void send_stats(void)                   /* 'T': one line a second, for the PC */
{
    static uint32_t ticks0, rxb0, full0;
    char o[160];
    uint32_t n = 0, t = st_ticks, r = st_rxb, f = st_full;
    n += put_u(o + n, "FM-1 rx_KB/s=", (r - rxb0) / 1024u);
    n += put_u(o + n, "rx_full_ticks=", f - full0);
    n += put_u(o + n, "t5_hz=", t - ticks0);
    n += put_u(o + n, "loops=", st_loops);
    n += put_u(o + n, "max_loop_us=", st_maxloop / FM1_TICKS_PER_US);
    n += put_u(o + n, "screen_q_max=", st_lcd);
    n += put_u(o + n, "L=", st_l);
    n += put_u(o + n, "A=", st_a);
    n += put_u(o + n, "fifo=", af_w - af_r);
    n += put_u(o + n, "under=", underruns);
    if (send_packet('T', (const uint8_t *)o, n)) {
        ticks0 = t; rxb0 = r; full0 = f;
        st_loops = st_maxloop = st_lcd = st_l = st_a = 0;
    }
}

static void app_main(void)
{
    uint32_t last_stats = 0, loop_t = 0;
    uint32_t i, last_report = 0, last_adc = 0, last_wait = 0;
    int32_t master = 0, batt = 0, enc_acc[FM1_NENC] = {0};
    uint8_t rep[30];
    uint32_t steps = 10;
    /* the screen first, lit: each start-up step then leaves a green block along the top (a crash shows its address) */
    lcd_init();
    lcd_ready = 1;
    splash();                                   /* until the Teensy's screen arrives, and at least SPLASH_MS */
    splash_until = ms + SPLASH_MS;
    fm1_lcd_backlight(1);
    diag_step(0);
    {
        uint32_t f = irq_save();
        flash_ok = FL_FAR(fl_jedec_ram)() == 0x856014u;
        irq_restore(f);
        diag_step(1);
        if (flash_ok)
            ota_boot_cleanup();
        diag_step(2);
    }
    fm1_input_init();
    diag_step(3);
    fm1_adc_init();
    diag_step(4);
    for (i = 0; i < 2u * HALF_WORDS; i++)
        abuf[i] = 0;
    fm1_audio_init(abuf, HALF_WORDS, isr_alnk0, 3);
    diag_step(5);
    usb_start();
    diag_step(6);
    fm1_timer5_start(isr_timer5, 4);           /* above ALNK0 (3), as Felucca */
    diag_step(7);
    fm1_guard_lock_top();
    fm1_irq_enable_all();
    diag_step(8);
    for (;;) {
        uint32_t now = ms;
        {
            uint32_t t = fm1_ticks();
            if (loop_t && t - loop_t > st_maxloop)
                st_maxloop = t - loop_t;
            loop_t = t;
            st_loops++;
        }
        if (now - last_stats >= 1000u) {
            last_stats = now;
            send_stats();
        }
        if (steps < 13u && now >= (steps - 10u) * 1000u)   /* blocks 9..12: 0, 1, 2, 3 s into the main loop */
            diag_step(steps++ - 1u);
        fm1_wdt_feed();
        usb_retry(now);
        if (usb.uboot_req) {                    /* the SysEx soft key */
            usb_detach();
            fm1_audio_stop();
            fm1_enter_uboot();
        }
        ota_service();
        if (usb.ota_req) {
            usb.ota_req = 0;
            if (flash_ok)
                ota_session();
            lq_clear();
            splash();
            link_up = 0;
        }
        if (!boots_ok && now > 2000u) {
            boots_ok = 1;
            guard.pending = 0;
            guard.failed = 0;
        }
        rx_service();
        if (splash_until && (int32_t)(now - splash_until) >= 0) {
            if (!link_up || send_packet('R', 0, 0))   /* the Teensy's whole screen, now the splash is done */
                splash_until = 0;
        }
        lcd_pump();
        if (link_up && ms - last_rx_ms > 1000u) {    /* the Teensy went quiet (ms now: rx_service may have moved last_rx_ms past this pass's now) */
            link_up = 0;
            lq_clear();
            splash();
        }
        if (!link_up && now - last_wait >= 100u) {
            last_wait = now;
            show_waiting(now);
        }
        if (now - last_adc >= 40u) {
            last_adc = now;
            master = fm1_adc_read(FM1_ADC_MASTER);
            batt = fm1_adc_read(FM1_ADC_BATT);
        }
        for (i = 0; i < FM1_NENC; i++)
            enc_acc[i] += fm1_enc_take(i);
        if (now - last_report >= 4u) {
            uint32_t w = af_w, r = af_r;
            int8_t e[FM1_NENC];
            for (i = 0; i < FM1_NENC; i++) {
                int32_t s = enc_acc[i] > 127 ? 127 : enc_acc[i] < -127 ? -127 : enc_acc[i];
                e[i] = (int8_t)s;
            }
            put32(rep, fm1_in.notes);
            put32(rep + 4, fm1_in.buttons);
            for (i = 0; i < 7u; i++)
                rep[8u + i] = i < FM1_NENC ? (uint8_t)e[i] : 0u;
            put16(rep + 15, (uint32_t)master);
            put16(rep + 17, (uint32_t)batt);
            put32(rep + 19, played);
            put16(rep + 23, underruns);
            put16(rep + 25, w - r);
            rep[27] = (uint8_t)guard.rst;
            rep[28] = (uint8_t)guard.boots;
            rep[29] = link_gen;
            if (send_packet('I', rep, 30)) {
                last_report = now;
                for (i = 0; i < FM1_NENC; i++)
                    enc_acc[i] -= e[i];
            }
        }
    }
}

void fm1_cstart(void)
{
    uint32_t *s, *d;
    fm1_time_init();
    fm1_wdt_arm(0x0D);
    if (guard.magic != 0x54524D31u) {           /* "TRM1" */
        guard.magic = 0x54524D31u;
        guard.failed = 0;
        guard.pending = 0;
        guard.boots = 0;
    }
    fm1_reset_reason();
    guard.rst = fm1_boot.p3_rst;
    guard.boots++;
    /* only a hang (WDT, bit 2) or a soft reset (bit 6: the crash screen's) counts as a failed start. Switching the
     * FM-1 off and on (power-on, VDDIO low, long press: bits 0, 1, 4) never does: its RAM keeps this struct on USB
     * power, and two quick power cycles had sent it to UBOOT (2026-11-13) */
    if (guard.pending && (fm1_boot.p3_rst & 0x44u) && !(fm1_boot.p3_rst & 0x13u))
        guard.failed++;
    else if (!(fm1_boot.p3_rst & 0x44u))
        guard.failed = 0;
    guard.pending = 1;
    if (guard.failed >= 3u) {
        guard.failed = 0;
        guard.pending = 0;
        fm1_enter_uboot();
    }
    fm1_irq_init();
    for (d = _bss_start; d < _bss_end; d++)
        *d = 0;
    for (s = _data_load, d = _data_start; d < _data_end; s++, d++)
        *d = *s;
    for (s = _rt_load, d = _rt_start; d < _rt_end; s++, d++)
        *d = *s;
    fm1_guard_enable(FM1_GUARD_STACK | FM1_GUARD_WRITE | FM1_GUARD_BUS | FM1_GUARD_PC);
    app_main();
    for (;;)
        ;
}

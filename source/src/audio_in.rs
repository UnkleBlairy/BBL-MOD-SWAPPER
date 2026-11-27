//! Line in for the Sample Lab's recorder: the PJRC Audio Shield (Rev D, SGTL5000) on SAI1, as the Teensy Audio
//! library wires it: I2C on pins 18 (SDA) / 19 (SCL), MCLK 23, BCLK 21, LRCLK 20, the codec's data out on pin 8.
//! The Teensy is the I2S master at exactly 44.1 kHz (the audio PLL at 677.376 MHz, /4 /15 = 11.2896 MHz MCLK).
//! The codec's pins are touched only once the codec has answered on I2C (pins 21 / 23 are the splitter's on a kit).
//! The kit's own line in (BBL Audio In, ES8388 on SAI2) gets its own `start` later; the recorder only needs
//! `start`, `stop`, `written`, `at` and `set_gain`.
//!
//! The SAI1 interrupt empties the receive FIFO (32 words: 0.36 ms) into a ring of mono samples: 8192 here (OCRAM)
//! then 8192 in the Lab's audition ring, 16384 in all (371 ms), so a slow card write never loses sound.
//! The headphone jack on the shield hears the line in straight through (the codec's analog bypass).

use core::sync::atomic::{AtomicU32, Ordering};

const HALF: usize = 8192;
#[unsafe(link_section = ".uninit.lab_rec")]
static mut RING_A: [i16; HALF] = [0; HALF];
static mut RING_B: *mut i16 = core::ptr::null_mut();
static W: AtomicU32 = AtomicU32::new(0); // samples written, ever
static OVER: AtomicU32 = AtomicU32::new(0); // FIFO overruns (the interrupt came too late)
static mut PH: u8 = 0; // 0: the next word is a left one
static mut LV: i32 = 0;
static mut READY: bool = false; // the codec is set up (once per power-up)

static mut BIG: u32 = 0; // samples in the PSRAM ring (0: no PSRAM, the small ring above)
static RUN: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false); // the receiver is on
static NEED: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false); // the Jam Box asked for it

/// The Lab recorder's part of the PSRAM: the first 4 MB (2 M samples, 47 s). The rest (12 MB with both chips) is the
/// Jam Box's track recording pool (ports/jambox/docs/TRACK_REC.md, `teensy_pool`).
pub const RING_BIG: u32 = 2 * 1024 * 1024;

/// How many samples the ring keeps: 16384 (371 ms), or with PSRAM 2 M (47 s): a whole take, so nothing goes to the
/// card until it ends.
pub fn ring() -> u32 {
    let b = unsafe { BIG };
    if b > 0 { b } else { 2 * HALF as u32 }
}
pub fn big() -> bool {
    unsafe { BIG > 0 }
}

fn rd(a: usize) -> u32 {
    unsafe { (a as *const u32).read_volatile() }
}
fn wr(a: usize, v: u32) {
    unsafe { (a as *mut u32).write_volatile(v) }
}
fn rmw(a: usize, clear: u32, set: u32) {
    wr(a, rd(a) & !clear | set);
}

const SAI: usize = 0x4038_4000;
const RCSR: usize = SAI + 0x88;
const RCR1: usize = SAI + 0x8C;
const RCR2: usize = SAI + 0x90;
const RCR3: usize = SAI + 0x94;
const RCR4: usize = SAI + 0x98;
const RCR5: usize = SAI + 0x9C;
const RDR0: usize = SAI + 0xA0;
const RFR0: usize = SAI + 0xC0;
const RMR: usize = SAI + 0xE0;
const IOMUXC: usize = 0x401F_8000;
const GPR: usize = 0x400A_C000;
const CCM: usize = 0x400F_C000;
const ANALOG: usize = 0x400D_8000;
const NVIC_ISER1: usize = 0xE000_E104;
const NVIC_ICER1: usize = 0xE000_E184;
const IRQ_BIT: u32 = 1 << (56 - 32); // SAI1 = IRQ 56

// ---- I2C, bit-banged on pins 18 (GPIO1 17, SDA) and 19 (GPIO1 16, SCL); the shield has the pull-ups ----
const SDA: u32 = 1 << 17;
const SCL: u32 = 1 << 16;
fn gpio() -> usize {
    // GPR26 hands each GPIO1 pin to the fast GPIO6 when its bit is set
    if rd(GPR + 0x68) & (SDA | SCL) != 0 { 0x4200_0000 } else { 0x401B_8000 }
}
fn dly() {
    cortex_m::asm::delay(3000); // ~5 us: ~100 kHz
}
struct I2c(usize);
impl I2c {
    fn open() -> I2c {
        // pads: GPIO (ALT5), hysteresis, 22k pull-up; driven only low (open drain by the direction register)
        for (mux, pad) in [(0xFC, 0x2EC), (0x100, 0x2F0)] {
            wr(IOMUXC + mux, 5);
            wr(IOMUXC + pad, 0x1_F0B0);
        }
        let g = gpio();
        rmw(g, SDA | SCL, 0); // DR low
        rmw(g + 4, SDA | SCL, 0); // both released
        dly();
        I2c(g)
    }
    fn line(&self, bit: u32, hi: bool) {
        rmw(self.0 + 4, bit, if hi { 0 } else { bit });
    }
    fn sda_in(&self) -> bool {
        rd(self.0 + 8) & SDA != 0
    }
    fn scl_hi(&self) {
        self.line(SCL, true);
        for _ in 0..1000 {
            if rd(self.0 + 8) & SCL != 0 {
                break; // (the codec may hold it low a moment)
            }
            dly();
        }
    }
    fn start(&self) {
        self.line(SDA, true);
        self.scl_hi();
        dly();
        self.line(SDA, false);
        dly();
        self.line(SCL, false);
        dly();
    }
    fn stop(&self) {
        self.line(SDA, false);
        dly();
        self.scl_hi();
        dly();
        self.line(SDA, true);
        dly();
    }
    fn put(&self, b: u8) -> bool {
        for i in (0..8).rev() {
            self.line(SDA, b >> i & 1 != 0);
            dly();
            self.scl_hi();
            dly();
            self.line(SCL, false);
        }
        self.line(SDA, true);
        dly();
        self.scl_hi();
        dly();
        let ack = !self.sda_in();
        self.line(SCL, false);
        dly();
        ack
    }
    fn get(&self, ack: bool) -> u8 {
        self.line(SDA, true);
        let mut b = 0u8;
        for _ in 0..8 {
            dly();
            self.scl_hi();
            dly();
            b = b << 1 | self.sda_in() as u8;
            self.line(SCL, false);
        }
        self.line(SDA, !ack);
        dly();
        self.scl_hi();
        dly();
        self.line(SCL, false);
        self.line(SDA, true);
        dly();
        b
    }
}

const ADDR: u8 = 0x0A; // the SGTL5000 (CTRL_ADR0_CS low)
fn write(i: &I2c, reg: u16, v: u16) -> bool {
    i.start();
    let ok = i.put(ADDR << 1) && i.put((reg >> 8) as u8) && i.put(reg as u8) && i.put((v >> 8) as u8) && i.put(v as u8);
    i.stop();
    ok
}
fn read(i: &I2c, reg: u16) -> Option<u16> {
    i.start();
    let ok = i.put(ADDR << 1) && i.put((reg >> 8) as u8) && i.put(reg as u8);
    if !ok {
        i.stop();
        return None;
    }
    i.start();
    if !i.put(ADDR << 1 | 1) {
        i.stop();
        return None;
    }
    let hi = i.get(true);
    let lo = i.get(false);
    i.stop();
    Some((hi as u16) << 8 | lo as u16)
}

/// The audio PLL, SAI1's clock and MCLK out on pin 23 (the codec needs MCLK to do anything but answer).
fn clocks() {
    rmw(CCM + 0x7C, 0, 3 << 18); // CCGR5: SAI1 on
    // PLL4: 24 MHz x (28 + 2240/10000) = 677.376 MHz, post divider 1 (POST_DIV_SELECT 2)
    if rd(ANALOG + 0x70) & (1 << 13) == 0 || rd(ANALOG + 0x70) & 0x7F != 28 {
        wr(ANALOG + 0x70, 1 << 16 | 1 << 13 | 2 << 19 | 28 | 1 << 12);
        wr(ANALOG + 0x80, 2240);
        wr(ANALOG + 0x90, 10000);
        rmw(ANALOG + 0x70, 1 << 12, 0); // power up
        for _ in 0..1_000_000 {
            if rd(ANALOG + 0x70) & (1 << 31) != 0 {
                break;
            }
        }
        rmw(ANALOG + 0x170, 1 << 23 | 1 << 15, 0); // MISC2: audio divider 1
        rmw(ANALOG + 0x70, 1 << 16, 0); // bypass off
    }
    // SAI1 clock root = PLL4 / 4 / 15 = 11.2896 MHz
    rmw(CCM + 0x1C, 3 << 10, 2 << 10);
    rmw(CCM + 0x28, 0x7 << 6 | 0x3F, 3 << 6 | 14);
    rmw(GPR + 0x04, 0x7, 1 << 19); // MCLK1 = the clock root, pin out
    wr(IOMUXC + 0x120, 3); // pin 23: SAI1_MCLK
}

/// The SAI1 receiver, master: BCLK 64 fs on pin 21, LRCLK on 20, data in on pin 8; interrupt at 16 words.
fn sai_on() {
    wr(RCSR, 0);
    for _ in 0..100_000 {
        if rd(RCSR) & (1 << 31) == 0 {
            break;
        }
    }
    wr(IOMUXC + 0x128, 3); // pin 21: SAI1_RX_BCLK
    wr(IOMUXC + 0x124, 3); // pin 20: SAI1_RX_SYNC
    wr(IOMUXC + 0x17C, 3); // pin 8: SAI1_RX_DATA00
    wr(IOMUXC + 0x594, 2); // ... its input select
    wr(RMR, 0);
    wr(RCR1, 15);
    wr(RCR2, 1 << 26 | 1 << 25 | 1 << 24 | 1); // async, MCLK1, BCLK rising, BCLK out, MCLK / 4
    wr(RCR3, 1 << 16); // channel 0
    // 2 words a frame, 32-bit sync, MSB first, early, low, out; FCONT: an overrun (a flash erase holds the interrupts
    // off for 45-150 ms) doesn't stop the receiver (it stopped for good on the FM-1, 2026-11-21: the INPUT track went
    // quiet after a take, the LAST JAM save)
    wr(RCR4, 1 << 28 | 1 << 16 | 31 << 8 | 1 << 4 | 1 << 3 | 1 << 1 | 1);
    wr(RCR5, 31 << 24 | 31 << 16 | 31 << 8);
    unsafe {
        PH = 0;
    }
    wr(RCSR, 1 << 25 | 1 << 20 | 1 << 19 | 1 << 18); // FIFO reset, error flags cleared (a FIFO error stops it)
    wr(RCSR, 1 << 31 | 1 << 28 | 1 << 8); // on, BCLK on, FIFO request interrupt
    wr(NVIC_ISER1, IRQ_BIT);
}

/// Start recording into the ring (`b`: the second half, HALF samples). False: no codec answered.
pub fn start(b: *mut i16) -> bool {
    unsafe {
        RING_B = b;
        if BIG == 0 {
            // the Teensy 4.1's PSRAM chips (set up once; none fitted: the small ring)
            let n = if crate::psram::found() > 0 { crate::psram::found() } else {
                let (a, b) = crate::psram::init();
                (a == 0x5D0D) as u32 + (b == 0x5D0D && a == 0x5D0D) as u32
            };
            BIG = if n > 0 { RING_BIG } else { 0 };
        }
    }
    let i = I2c::open();
    let mut found = read(&i, 0).is_some_and(|id| id >> 8 == 0xA0);
    if !found {
        clocks(); // (some codecs answer only with MCLK running)
        cortex_m::asm::delay(600_000);
        found = read(&i, 0).is_some_and(|id| id >> 8 == 0xA0);
        if !found {
            wr(IOMUXC + 0x120, 5); // pin 23 back to a GPIO (an input)
            return false;
        }
    }
    clocks();
    if !unsafe { READY } {
        // as the Teensy Audio library's AudioControlSGTL5000::enable(), then line in
        let ms = |n: u32| cortex_m::asm::delay(600_000 * n);
        write(&i, 0x0030, 0x4060); // ANA_POWER: VDDD from outside
        write(&i, 0x0026, 0x006C); // LINREG
        write(&i, 0x0028, 0x01F2); // REF_CTRL
        write(&i, 0x002C, 0x0F22); // LINE_OUT_CTRL
        write(&i, 0x003C, 0x4446); // SHORT_CTRL
        write(&i, 0x0024, 0x0137); // ANA_CTRL
        write(&i, 0x0030, 0x40FF); // ANA_POWER: all up
        write(&i, 0x0002, 0x0073); // DIG_POWER
        ms(400);
        write(&i, 0x002E, 0x1D1D); // LINE_OUT_VOL
        write(&i, 0x0004, 0x0004); // CLK_CTRL: 44.1 kHz, 256 fs
        write(&i, 0x0006, 0x0030); // I2S_CTRL: 64 fs, 16 bits, I2S, slave
        write(&i, 0x000A, 0x0000); // SSS_CTRL: I2S out = ADC, DAC = ADC
        write(&i, 0x000E, 0x0000); // ADCDAC_CTRL: unmuted
        write(&i, 0x0010, 0x3C3C); // DAC_VOL 0 dB
        write(&i, 0x0022, 0x3E3E); // ANA_HP_CTRL: about half
        write(&i, 0x0020, 0x0055); // ANA_ADC_CTRL: line in level 5
        write(&i, 0x0024, 0x0066); // ANA_CTRL: ADC = line in, headphones = line in (bypass), unmuted
        unsafe {
            READY = true;
        }
    }
    sai_on();
    RUN.store(true, Ordering::Release);
    true
}

/// Stop the interrupt (the ring's second half goes back to the Lab).
pub fn stop() {
    RUN.store(false, Ordering::Release);
    wr(NVIC_ICER1, IRQ_BIT);
    wr(RCSR, 0); // the receiver off (left on, its FIFO overflows and stops it)
    cortex_m::asm::dsb();
    unsafe {
        RING_B = core::ptr::null_mut();
    }
}

/// The line in level, 0 (quietest, 3.12 V p-p full scale) .. 15 (loudest, 0.24 V p-p).
pub fn set_gain(g: u8) {
    let g = g.min(15) as u16;
    let i = I2c::open();
    write(&i, 0x0020, g << 4 | g);
}

pub fn written() -> u32 {
    W.load(Ordering::Acquire)
}
pub fn overruns() -> u32 {
    OVER.load(Ordering::Relaxed)
}

/// Sample `n` (it must be one of the last RING written).
pub fn at(n: u32) -> i16 {
    let k = n as usize & (HALF - 1);
    unsafe {
        if BIG > 0 {
            return ((crate::psram::BASE as *const i16).add((n & (BIG - 1)) as usize)).read_volatile();
        }
        if n as usize & HALF == 0 || RING_B.is_null() {
            (&raw const RING_A as *const i16).add(k).read_volatile()
        } else {
            RING_B.add(k).read_volatile()
        }
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn SAI1() {
    let cs = rd(RCSR);
    if cs & (1 << 18) != 0 {
        wr(RCSR, cs & !(1 << 19 | 1 << 20) | 1 << 25); // overrun: clear it (W1C), empty the FIFO; it goes on (FCONT)
        OVER.fetch_add(1, Ordering::Relaxed);
        unsafe {
            PH = 0;
        }
        return;
    }
    let f = rd(RFR0);
    let n = ((f >> 16).wrapping_sub(f)) & 0x3F;
    let mut w = W.load(Ordering::Relaxed);
    unsafe {
        for _ in 0..n {
            let v = (rd(RDR0) >> 16) as i16 as i32;
            if PH == 0 {
                LV = v;
                PH = 1;
            } else {
                PH = 0;
                let m = ((LV + v) >> 1) as i16;
                let k = w as usize & (HALF - 1);
                if BIG > 0 {
                    ((crate::psram::BASE as *mut i16).add((w & (BIG - 1)) as usize)).write_volatile(m);
                } else if w as usize & HALF == 0 || RING_B.is_null() {
                    (&raw mut RING_A as *mut i16).add(k).write_volatile(m);
                } else {
                    RING_B.add(k).write_volatile(m);
                }
                w = w.wrapping_add(1);
            }
        }
    }
    W.store(w, Ordering::Release);
}

// ---- the Jam Box's live line in (track recording, ports/jambox/firmware/src/trec.c) ----
// The engine takes the line in a block at a time, in step with its own blocks, TARGET samples behind the receiver.
// The codec runs on the Teensy's crystal, the engine on its timer or as the FM-1 plays: they differ (~0.15 % measured)
// and the engine renders in bursts. So the reader resamples: its speed (Q16, 1.0 = 65536) follows the averaged lag
// (proportional + a slow integral, ~3 s), each sample interpolated between its two neighbours: no skips, no clicks.

// Measured 2026-11-21 on the FM-1 link: the lag swings ~-135 / +330 round its average (the engine's bursts), so 384
// keeps ~250 samples of room below and ~15 ms less delay than the 1024 it was.
const TARGET: u32 = 384; // 8.7 ms behind the receiver
static mut POS: u64 = 0; // the read position, Q16 (its whole part wraps with `written`)
static mut STEP: i64 = 65536;
static mut INTEG: i64 = 0;
static mut LAG_AVG: u32 = 0; // the lag, averaged (Q8)
static mut LIVE: bool = false;
static LAG_MIN: AtomicU32 = AtomicU32::new(u32::MAX); // the lag's least / most since the STAT line last read it
static LAG_MAX: AtomicU32 = AtomicU32::new(0);
static FIXES: AtomicU32 = AtomicU32::new(0);
static mut W_LAST: u32 = 0; // `written` at the reader's last call
static mut W_SAME: u32 = 0; // the reader's calls since `written` last moved
static RESTART: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false); // stuck: the main loop restarts it
static RESTARTS: AtomicU32 = AtomicU32::new(0);
static RCSR_STUCK: AtomicU32 = AtomicU32::new(0); // RCSR when it was last found stuck (what stopped it) // the speed now, ppm off 1.0 (signed, as u32)
static RESYNCS: AtomicU32 = AtomicU32::new(0);
/// Frames between rendering and the FM-1 playing them (main.rs: the link's target; ~2 blocks without it).
pub static OUT_LAT: AtomicU32 = AtomicU32::new(256);

/// The receiver stopped (no new samples for ~93 ms while on): the main loop calls `restart` (outside the render).
pub fn stuck() -> bool {
    RESTART.load(Ordering::Relaxed) && RUN.load(Ordering::Acquire)
}
pub fn restart() {
    RESTART.store(false, Ordering::Relaxed);
    RCSR_STUCK.store(rd(RCSR), Ordering::Relaxed);
    RESTARTS.fetch_add(1, Ordering::Relaxed);
    wr(NVIC_ICER1, IRQ_BIT);
    cortex_m::asm::dsb();
    sai_on();
    unsafe {
        LIVE = false; // the reader starts over at TARGET behind
    }
}
/// For the serial R report: samples written, overruns, restarts, RCSR now and when last found stuck.
pub fn health() -> (u32, u32, u32, u32, u32) {
    (written(), OVER.load(Ordering::Relaxed), RESTARTS.load(Ordering::Relaxed), rd(RCSR), RCSR_STUCK.load(Ordering::Relaxed))
}

/// The Jam Box wants the line in and it is off: the main loop starts it (outside the audio render).
pub fn need() -> bool {
    NEED.load(Ordering::Relaxed) && !RUN.load(Ordering::Acquire) && !FAIL.load(Ordering::Relaxed)
}
static FAIL: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false); // no codec answered

/// The line in's state for the Jam Box's screen: 0 no codec, 1 on, 2 off (it starts when asked for).
#[unsafe(no_mangle)]
pub extern "C" fn teensy_line_in_state() -> u32 {
    if RUN.load(Ordering::Acquire) { 1 } else if FAIL.load(Ordering::Relaxed) { 0 } else { 2 }
}
/// A track was armed: try the codec again (it may have been plugged in since).
#[unsafe(no_mangle)]
pub extern "C" fn teensy_line_in_arm() {
    FAIL.store(false, Ordering::Relaxed);
    NEED.store(true, Ordering::Relaxed);
}
/// Start the line in for the Jam Box (no Lab ring: the small ring, or the PSRAM one). False: no codec.
pub fn live_start() -> bool {
    NEED.store(false, Ordering::Relaxed);
    unsafe {
        LIVE = false;
    }
    let ok = start(core::ptr::null_mut());
    FAIL.store(!ok, Ordering::Relaxed);
    ok
}
/// For the STAT line: the reader's speed (ppm off 1.0), resyncs, the averaged lag.
pub fn fixes() -> (i32, u32, u32) {
    (FIXES.load(Ordering::Relaxed) as i32, RESYNCS.load(Ordering::Relaxed), unsafe { LAG_AVG >> 8 })
}
/// For the STAT line: the lag's least and most since the last call (the margin the reader really needs).
pub fn lag_span() -> (u32, u32) {
    (LAG_MIN.swap(u32::MAX, Ordering::Relaxed), LAG_MAX.swap(0, Ordering::Relaxed))
}

/// The next `n` samples of the line in into `dst` (the engine's block). 0: the line in is off (silence), it is asked for.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn teensy_line_in(dst: *mut i16, n: u32) -> u32 {
    let out = unsafe { core::slice::from_raw_parts_mut(dst, n as usize) };
    if !RUN.load(Ordering::Acquire) {
        NEED.store(true, Ordering::Relaxed);
        out.fill(0);
        unsafe {
            LIVE = false;
        }
        return 0;
    }
    let w = written();
    unsafe {
        if w == W_LAST {
            W_SAME += 1;
            if W_SAME > 128 {
                // ~4096 samples (93 ms) of calls and nothing new (more than any render burst): the receiver stopped
                RESTART.store(true, Ordering::Relaxed);
            }
        } else {
            W_SAME = 0;
            W_LAST = w;
        }
        let room = ring().min(65536) - 256;
        let lag = w.wrapping_sub((POS >> 16) as u32);
        if !LIVE || lag < n + 2 || lag > room {
            if LIVE {
                RESYNCS.fetch_add(1, Ordering::Relaxed);
            }
            POS = (w.wrapping_sub(TARGET) as u64) << 16;
            LAG_AVG = TARGET << 8;
            STEP = 65536; // (every resync: a stall's error left in INTEG pinned it at +1 % for minutes)
            INTEG = 0;
            LIVE = true;
        } else {
            LAG_MIN.fetch_min(lag, Ordering::Relaxed);
            LAG_MAX.fetch_max(lag, Ordering::Relaxed);
            LAG_AVG = LAG_AVG - (LAG_AVG >> 7) + (lag << 1); // ~128 blocks (90 ms)
            let err = (LAG_AVG as i64 - (TARGET << 8) as i64) >> 8; // samples, + = behind
            INTEG = (INTEG + err).clamp(-(1 << 26), 1 << 26);
            STEP = (65536 + err / 2 + (INTEG >> 14)).clamp(65536 - 655, 65536 + 655); // within 1 %
            FIXES.store((((STEP - 65536) * 1_000_000) >> 16) as i32 as u32, Ordering::Relaxed);
        }
        for o in out.iter_mut() {
            let i = (POS >> 16) as u32;
            let f = (POS & 0xFFFF) as i32;
            let a = at(i) as i32;
            let b = at(i.wrapping_add(1)) as i32;
            *o = (a + (((b - a) * f) >> 16)) as i16;
            POS = POS.wrapping_add(STEP as u64);
        }
    }
    1
}

/// Frames from the engine rendering a block to hearing what was played along with it back in the line in: the
/// output's lag (the FM-1 link) + the read lag + the codec (~1 ms).
#[unsafe(no_mangle)]
pub extern "C" fn teensy_in_latency() -> u32 {
    OUT_LAT.load(Ordering::Relaxed) + TARGET + 44
}

/// The track recording pool in PSRAM (after the Lab recorder's 4 MB): its first sample, `*n` samples. Null: no PSRAM.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn teensy_pool(n: *mut u32) -> *mut i16 {
    let mut chips = crate::psram::found();
    if chips == 0 {
        let (a, b) = crate::psram::init();
        chips = (a == 0x5D0D) as u32 + (b == 0x5D0D && a == 0x5D0D) as u32;
    }
    let bytes = chips as usize * crate::psram::CHIP;
    let skip = RING_BIG as usize * 2;
    if bytes <= skip {
        unsafe { *n = 0 };
        return core::ptr::null_mut();
    }
    unsafe { *n = ((bytes - skip) / 2) as u32 };
    (crate::psram::BASE + skip) as *mut i16
}

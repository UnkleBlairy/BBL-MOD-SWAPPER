//! External PSRAM on FlexSPI2 (Teensy 4.1 underside pads, APS6404L 8 MB each: chip 1 = CS0 EMC_24 at 0x7000_0000,
//! chip 2 = CS1 EMC_22 at 0x7080_0000). Same set-up as PJRC's startup.c configure_external_ram(): pins ALT8,
//! 88 MHz (PLL2 528 / 6), QPI read 0xEB / write 0x38. Only called on request (serial `O`), never at boot.

const F2: usize = 0x402A_4000; // FLEXSPI2
const MCR0: usize = 0x00; const MCR1: usize = 0x04; const MCR2: usize = 0x08; const AHBCR: usize = 0x0C;
const INTEN: usize = 0x10; const INTR: usize = 0x14; const LUTKEY: usize = 0x18; const LUTCR: usize = 0x1C;
const AHBRXBUF0: usize = 0x20; const FLSHA1CR0: usize = 0x60; const FLSHA2CR0: usize = 0x64;
const FLSHA1CR1: usize = 0x70; const FLSHA2CR1: usize = 0x74; const FLSHA1CR2: usize = 0x80; const FLSHA2CR2: usize = 0x84;
const IPCR0: usize = 0xA0; const IPCR1: usize = 0xA4; const IPCMD: usize = 0xB0; const IPRXFCR: usize = 0xB8;
const IPTXFCR: usize = 0xBC; const RFDR0: usize = 0x100; const LUT: usize = 0x200;

pub const BASE: usize = 0x7000_0000;
pub const CHIP: usize = 8 * 1024 * 1024;

#[inline(always)]
fn w(off: usize, v: u32) { unsafe { core::ptr::write_volatile((F2 + off) as *mut u32, v) } }
#[inline(always)]
fn r(off: usize) -> u32 { unsafe { core::ptr::read_volatile((F2 + off) as *const u32) } }
#[inline(always)]
fn wa(addr: usize, v: u32) { unsafe { core::ptr::write_volatile(addr as *mut u32, v) } }
#[inline(always)]
fn ra(addr: usize) -> u32 { unsafe { core::ptr::read_volatile(addr as *const u32) } }

// LUT instruction: operand | pads << 8 | opcode << 10 (two per 32-bit word)
const CMD_SDR: u32 = 0x01; const ADDR_SDR: u32 = 0x02; const WRITE_SDR: u32 = 0x08; const READ_SDR: u32 = 0x09;
const DUMMY_SDR: u32 = 0x0C; const P1: u32 = 0; const P4: u32 = 2;
const fn ins(op: u32, pads: u32, opnd: u32) -> u32 { (opnd & 0xFF) | (pads << 8) | (op << 10) }
const fn lut(a: u32, b: u32) -> u32 { a | (b << 16) }

fn ip(seq: u32, addr: u32, datsz: u32) {
    w(IPCR0, addr);
    w(IPCR1, (seq << 16) | datsz);
    w(IPCMD, 1);
    let mut n = 0u32;
    while r(INTR) & 1 == 0 {
        n += 1;
        if n > 10_000_000 { break; } // no hang if the bus is dead
    }
    w(INTR, 1 | (1 << 3)); // done + error only: IPRXWA (bit 5) pops the RX FIFO, so read first
}

fn read_id(addr: u32) -> u32 {
    w(IPRXFCR, r(IPRXFCR) | 1); // empty the RX FIFO first
    ip(11, addr, 4);
    let id = r(RFDR0) & 0xFFFF;
    w(INTR, 1 << 5); // now pop it
    id
}

static mut READY: u32 = 0; // chips found (0, 1 or 2)

/// Set up FlexSPI2 and both chip selects. Returns the IDs read from chip 1 and chip 2 (0x5D0D = APS6404).
pub fn init() -> (u32, u32) {
    unsafe {
        // pads: EMC_22..29 ALT8 (+SION), drive/pull as PJRC
        let mux = |n: usize| 0x401F_8014 + 4 * n;
        let pad = |n: usize| 0x401F_8204 + 4 * n;
        wa(pad(22), 0x1B0F9); wa(pad(23), 0x110F9); wa(pad(24), 0x1B0F9); wa(pad(25), 0x100F9);
        for n in 26..=29 { wa(pad(n), 0x170F9); }
        for n in 22..=29 { wa(mux(n), 8 | 0x10); }
        // daisy chain: DQS_FA, IO_FA_BIT0..3, SCK_FA -> the EMC pads
        for a in [0x401F_872Cusize, 0x401F_8730, 0x401F_8734, 0x401F_8738, 0x401F_873C, 0x401F_8750] { wa(a, 1); } // PJRC imxrt.h
        // clock: FLEXSPI2_CLK_SEL = 3 (PLL2 528 MHz), PODF = 5 (/6) = 88 MHz; gate on
        let cbcmr = 0x400F_C018usize; let ccgr7 = 0x400F_C080usize;
        wa(ccgr7, ra(ccgr7) & !(3 << 8));
        wa(cbcmr, (ra(cbcmr) & !((7 << 29) | (3 << 8))) | (5 << 29) | (3 << 8));
        wa(ccgr7, ra(ccgr7) | (3 << 8));

        w(MCR0, r(MCR0) | 2); // MDIS
        let clr0 = (0xFF << 24) | (0xFF << 16) | (1 << 14) | (1 << 13) | (1 << 12) | (1 << 11) | (1 << 7) | (1 << 6) | (3 << 4) | 1;
        w(MCR0, (r(MCR0) & !clr0) | (0xFF << 24) | (0xFF << 16) | (1 << 4) | 2);
        w(MCR1, (0xFFFF << 16) | 0xFFFF);
        let clr2 = (0xFF << 24) | (1 << 19) | (1 << 15) | (1 << 14) | (1 << 11);
        w(MCR2, (r(MCR2) & !clr2) | (0x20 << 24));
        w(AHBCR, r(AHBCR) & !((1 << 6) | (1 << 5) | (1 << 4) | (1 << 3)));
        let mask = (1 << 31) | (3 << 24) | (0xF << 16) | 0xFF;
        w(AHBRXBUF0, (r(AHBRXBUF0) & !mask) | (1 << 31) | 64);
        w(AHBRXBUF0 + 4, (r(AHBRXBUF0) & !mask) | (1 << 31) | 64);
        w(AHBRXBUF0 + 8, mask);
        w(AHBRXBUF0 + 12, mask);
        w(IPRXFCR, (r(IPRXFCR) & 0xFFFF_FFC0) | 1);
        w(IPTXFCR, (r(IPTXFCR) & 0xFFFF_FFC0) | 1);
        w(INTEN, 0);
        let cr1 = (1 << 5) | 1; // CSINTERVAL 0, TCSH 1, TCSS 1 (PJRC)
        let cr2 = (6 << 8) | 5; // AHB write = seq 6, AHB read = seq 5
        w(FLSHA1CR0, 0x2000); w(FLSHA1CR1, cr1); w(FLSHA1CR2, cr2); // 8 MB (KB units)
        w(FLSHA2CR0, 0x2000); w(FLSHA2CR1, cr1); w(FLSHA2CR2, cr2);
        w(MCR0, r(MCR0) & !2);

        w(LUTKEY, 0x5AF0_5AF0); w(LUTCR, 2);
        for i in 0..64 { w(LUT + 4 * i, 0); }
        w(MCR0, r(MCR0) | 1);
        let mut n = 0; while r(MCR0) & 1 != 0 && n < 1_000_000 { n += 1; }
        w(LUTKEY, 0x5AF0_5AF0); w(LUTCR, 2);
        let s = |seq: usize, k: usize, v: u32| w(LUT + 4 * (seq * 4 + k), v);
        s(8, 0, lut(ins(CMD_SDR, P4, 0xF5), 0));                          // exit QPI
        s(9, 0, lut(ins(CMD_SDR, P1, 0x66), 0));                          // reset enable
        s(10, 0, lut(ins(CMD_SDR, P1, 0x99), 0));                         // reset
        s(11, 0, lut(ins(CMD_SDR, P1, 0x9F), ins(DUMMY_SDR, P1, 24)));    // read ID (PJRC: 24 dummy cycles)
        s(11, 1, lut(ins(READ_SDR, P1, 1), 0));
        s(12, 0, lut(ins(CMD_SDR, P1, 0x35), 0));                         // enter QPI
        s(5, 0, lut(ins(CMD_SDR, P4, 0xEB), ins(ADDR_SDR, P4, 24)));      // QPI read
        s(5, 1, lut(ins(DUMMY_SDR, P4, 6), ins(READ_SDR, P4, 1)));
        s(6, 0, lut(ins(CMD_SDR, P4, 0x38), ins(ADDR_SDR, P4, 24)));      // QPI write
        s(6, 1, lut(ins(WRITE_SDR, P4, 1), 0));

        let mut ids = (0, 0);
        for (k, addr) in [0u32, 0x80_0000].into_iter().enumerate() {
            ip(8, addr, 0); ip(9, addr, 0); ip(10, addr, 0);
            let id = read_id(addr);
            if k == 0 { ids.0 = id } else { ids.1 = id }
        }
        let mut found = 0;
        if ids.0 == 0x5D0D { ip(12, 0, 0); found = 1; }
        if ids.1 == 0x5D0D { ip(12, 0x80_0000, 0); found += 1; }
        READY = found;
        ids
    }
}

/// Write then read back one chip with an address-derived pattern (two passes, the second inverted).
/// Returns (bad words, first bad address or 0, MB/s write, MB/s read) for `chip` 0 or 1.
pub fn test_chip(chip: usize, cycles_per_us: u32) -> (u32, u32, u32, u32) {
    let base = BASE + chip * CHIP;
    let words = CHIP / 4;
    let pat = |i: usize, inv: bool| {
        let mut x = (i as u32).wrapping_mul(0x9E37_79B1) ^ 0xA5A5_5A5A;
        x ^= x >> 15;
        if inv { !x } else { x }
    };
    let mut bad = 0u32; let mut first = 0u32;
    let (mut tw, mut tr) = (0u32, 0u32);
    let dwt = || cortex_m::peripheral::DWT::cycle_count();
    for inv in [false, true] {
        let t0 = dwt();
        for i in 0..words { wa(base + 4 * i, pat(i, inv)); }
        cortex_m::asm::dsb();
        clean_invalidate();
        let t1 = dwt();
        for i in 0..words {
            let v = ra(base + 4 * i);
            if v != pat(i, inv) {
                if bad == 0 { first = (base + 4 * i) as u32; }
                bad = bad.saturating_add(1);
            }
        }
        let t2 = dwt();
        tw = t1.wrapping_sub(t0); tr = t2.wrapping_sub(t1);
    }
    let mbps = |c: u32| if c == 0 { 0 } else { ((CHIP as u64 * cycles_per_us as u64) / c as u64) as u32 };
    (bad, first, mbps(tw), mbps(tr))
}

/// Clean + invalidate the whole D-cache (by set/way), so read-back really comes from the chip.
fn clean_invalidate() {
    unsafe {
        // Cortex-M7 32 KB D-cache: 4 ways x 256 sets x 32 B lines
        for set in 0..256u32 {
            for way in 0..4u32 {
                core::ptr::write_volatile(0xE000_EF74 as *mut u32, (way << 30) | (set << 5)); // DCCISW
            }
        }
        cortex_m::asm::dsb();
        cortex_m::asm::isb();
    }
}

pub fn found() -> u32 { unsafe { READY } }

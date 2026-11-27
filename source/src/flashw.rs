//! Writing the Teensy's own program flash (the W25Q64 behind FlexSPI1) at run time: 64 KB block erase and 256-byte
//! page program, as IP commands through LUT sequence 15 (the boot header's LUT leaves it free; PJRC's eeprom.c uses
//! the same one). Nothing may read the flash while it is busy, so every function here runs from ITCM (all of this
//! crate's .text is in ITCM in the menu build), touches only registers and the stack, and keeps interrupts off.
//! After a write the AHB read buffers and the CPU's data cache are cleared, so reads see the new bytes.
//!
//! Only the slot area is writable: slot 0 (the menu, this code) and PJRC's top 256 KB are refused.

const BASE: usize = 0x402A_8000; // FlexSPI1
const MCR0: usize = 0x00;
const INTR: usize = 0x14;
const LUTKEY: usize = 0x18;
const LUTCR: usize = 0x1C;
const IPCR0: usize = 0xA0;
const IPCR1: usize = 0xA4;
const IPCMD: usize = 0xB0;
const IPRXFCR: usize = 0xB8;
const IPTXFCR: usize = 0xBC;
const RFDR0: usize = 0x100;
const TFDR0: usize = 0x180;
const LUT60: usize = 0x200 + 60 * 4;

const IPCMDDONE: u32 = 1 << 0;
const IPTXWE: u32 = 1 << 6;

// LUT instructions: opcode << 10 | pads << 8 | operand
const CMD_SDR: u32 = 0x01;
const ADDR_SDR: u32 = 0x02;
const WRITE_SDR: u32 = 0x08;
const READ_SDR: u32 = 0x09;
const fn ins(op: u32, pads: u32, opnd: u32) -> u32 {
    op << 10 | pads << 8 | opnd
}
const fn lut(a: u32, b: u32) -> u32 {
    a | b << 16
}

/// The writable range: slots 1.. up to PJRC's reserved top 256 KB.
pub const LO: u32 = 0x6020_0000;
pub const HI: u32 = 0x607C_0000;
pub const BLOCK: u32 = 0x1_0000;
pub const PAGE: u32 = 256;

#[inline(always)]
fn rd(o: usize) -> u32 {
    unsafe { ((BASE + o) as *const u32).read_volatile() }
}
#[inline(always)]
fn wr(o: usize, v: u32) {
    unsafe { ((BASE + o) as *mut u32).write_volatile(v) }
}

#[inline(always)]
fn seq(a: u32, b: u32, c: u32, d: u32) {
    wr(LUTKEY, 0x5AF0_5AF0);
    wr(LUTCR, 2); // unlock
    wr(LUT60, a);
    wr(LUT60 + 4, b);
    wr(LUT60 + 8, c);
    wr(LUT60 + 12, d);
}

#[inline(always)]
fn run(addr: u32, len: u32) {
    wr(IPCR0, addr & 0x00FF_FFFF);
    wr(IPCR1, 15 << 16 | (len & 0xFFFF));
    wr(IPCMD, 1);
}

#[inline(always)]
fn done() {
    while rd(INTR) & IPCMDDONE == 0 {}
    wr(INTR, IPCMDDONE);
}

#[inline(always)]
fn write_enable() {
    seq(lut(ins(CMD_SDR, 0, 0x06), 0), 0, 0, 0);
    run(0, 0);
    done();
}

/// Poll the status register until the flash is no longer busy, then drop FlexSPI's stale read buffers.
#[inline(always)]
fn wait_ready() {
    seq(lut(ins(CMD_SDR, 0, 0x05), ins(READ_SDR, 0, 1)), 0, 0, 0);
    loop {
        wr(IPRXFCR, 1); // clear the rx FIFO
        run(0, 1);
        done();
        if rd(RFDR0) & 1 == 0 {
            break;
        }
    }
    wr(MCR0, rd(MCR0) | 1); // software reset: purges the AHB read buffers
    while rd(MCR0) & 1 != 0 {}
}

fn invalidate(addr: u32, len: u32) {
    let mut a = addr & !31;
    while a < addr + len {
        unsafe { (0xE000_EF5C as *mut u32).write_volatile(a) }; // DCIMVAC
        a += 32;
    }
    cortex_m::asm::dsb();
    cortex_m::asm::isb();
}

/// Erase the 64 KB block at `addr` (aligned). ~150 ms, interrupts off.
#[inline(never)]
pub fn erase_block(addr: u32) -> bool {
    if addr % BLOCK != 0 || addr < LO || addr >= HI {
        return false;
    }
    cortex_m::interrupt::free(|_| {
        write_enable();
        seq(lut(ins(CMD_SDR, 0, 0xD8), ins(ADDR_SDR, 0, 24)), 0, 0, 0);
        run(addr, 0);
        done();
        wait_ready();
    });
    invalidate(addr, BLOCK);
    true
}

/// Erase the 4 KB sector at `addr` (aligned). ~45 ms, interrupts off.
#[inline(never)]
pub fn erase_sector(addr: u32) -> bool {
    if addr % 0x1000 != 0 || addr < LO || addr >= HI {
        return false;
    }
    cortex_m::interrupt::free(|_| {
        write_enable();
        seq(lut(ins(CMD_SDR, 0, 0x20), ins(ADDR_SDR, 0, 24)), 0, 0, 0);
        run(addr, 0);
        done();
        wait_ready();
    });
    invalidate(addr, 0x1000);
    true
}

/// Program one page (`data` <= 256 bytes, not crossing a 256-byte boundary) at `addr`. The page must be erased.
#[inline(never)]
pub fn program(addr: u32, data: &[u8]) -> bool {
    let len = data.len() as u32;
    if len == 0 || len > PAGE || (addr % PAGE) + len > PAGE || addr < LO || addr + len > HI {
        return false;
    }
    cortex_m::interrupt::free(|_| {
        write_enable();
        seq(lut(ins(CMD_SDR, 0, 0x32), ins(ADDR_SDR, 0, 24)), lut(ins(WRITE_SDR, 2, 1), 0), 0, 0); // quad page program
        wr(IPTXFCR, 1); // clear the tx FIFO (watermark: 8 bytes)
        run(addr, len);
        let mut i = 0usize;
        loop {
            let st = rd(INTR);
            if st & IPCMDDONE != 0 {
                break;
            }
            if st & IPTXWE != 0 {
                let mut w = [0u32; 2];
                for k in 0..8 {
                    if i + k < data.len() {
                        w[k / 4] |= (data[i + k] as u32) << (8 * (k % 4));
                    }
                }
                wr(TFDR0, w[0]);
                wr(TFDR0 + 4, w[1]);
                i += 8;
                wr(INTR, IPTXWE);
            }
        }
        wr(INTR, IPCMDDONE | IPTXWE);
        wait_ready();
    });
    invalidate(addr, len);
    true
}

/// Program any length (page by page) at `addr`; the range must be erased.
pub fn write(mut addr: u32, mut data: &[u8]) -> bool {
    while !data.is_empty() {
        let n = ((PAGE - addr % PAGE) as usize).min(data.len());
        if !program(addr, &data[..n]) {
            return false;
        }
        addr += n as u32;
        data = &data[n..];
    }
    true
}

/// 'W' test: erase the first block of slot 3, write 64 KB of a pattern, read it back. Returns (erase us, write us,
/// wrong bytes, erased-check wrong bytes).
pub fn self_test() -> (u32, u32, u32, u32) {
    const AT: u32 = 0x6060_0000;
    let t = cortex_m::peripheral::DWT::cycle_count();
    if !erase_block(AT) {
        return (0, 0, u32::MAX, u32::MAX);
    }
    let te = cortex_m::peripheral::DWT::cycle_count().wrapping_sub(t) / 600;
    let p = unsafe { core::slice::from_raw_parts(AT as *const u8, BLOCK as usize) };
    let not_erased = p.iter().filter(|&&b| b != 0xFF).count() as u32;
    let pat = |i: u32| (i.wrapping_mul(7) ^ (i >> 8)) as u8;
    let t = cortex_m::peripheral::DWT::cycle_count();
    let mut buf = [0u8; 256];
    for pg in 0..BLOCK / PAGE {
        for (k, b) in buf.iter_mut().enumerate() {
            *b = pat(pg * PAGE + k as u32);
        }
        if !program(AT + pg * PAGE, &buf) {
            return (te, 0, u32::MAX, not_erased);
        }
    }
    let tw = cortex_m::peripheral::DWT::cycle_count().wrapping_sub(t) / 600;
    let bad = p.iter().enumerate().filter(|(i, b)| **b != pat(*i as u32)).count() as u32;
    (te, tw, bad, not_erased)
}

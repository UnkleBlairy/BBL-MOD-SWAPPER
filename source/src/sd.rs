//! The Teensy 4.1's built-in micro SD slot: uSDHC1, polled, 4-bit at ~25 MHz, PIO through the data port.
//! Pins GPIO_SD_B0_00..05 = CMD CLK D0 D1 D2 D3 (ALT0). Clock: PLL2 PFD0 (396 MHz) / 2 = 198 MHz into the
//! controller, divided down to ~390 kHz for identification, ~25 MHz after. SDHC/SDXC (block addressed) and
//! SDSC (byte addressed) cards. Implements embedded-sdmmc's BlockDevice for the FAT layer.

use core::cell::Cell;
use embedded_sdmmc::{Block, BlockCount, BlockDevice, BlockIdx};

const BASE: usize = 0x402C_0000; // USDHC1
const BLK_ATT: usize = 0x04;
const CMD_ARG: usize = 0x08;
const CMD_XFR_TYP: usize = 0x0C;
const RSP0: usize = 0x10;
const RSP1: usize = 0x14;
const RSP2: usize = 0x18;
const RSP3: usize = 0x1C;
const DATA: usize = 0x20;
const PRES: usize = 0x24;
const PROT: usize = 0x28;
const SYS: usize = 0x2C;
const INT_ST: usize = 0x30;
const INT_EN: usize = 0x34;
const INT_SIG: usize = 0x38;
const WTMK: usize = 0x44;
const MIX: usize = 0x48;
const VEND: usize = 0xC0;

fn rd(o: usize) -> u32 {
    unsafe { ((BASE + o) as *const u32).read_volatile() }
}
fn wr(o: usize, v: u32) {
    unsafe { ((BASE + o) as *mut u32).write_volatile(v) }
}
fn reg(a: usize) -> *mut u32 {
    a as *mut u32
}

// interrupt status bits
const CC: u32 = 1 << 0;
const TC: u32 = 1 << 1;
const BWR: u32 = 1 << 4;
const BRR: u32 = 1 << 5;
const CMD_ERR: u32 = 0xF << 16; // CTOE CCE CEBE CIE
const DATA_ERR: u32 = 0x7 << 20; // DTOE DCE DEBE

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Error {
    NoCard,      // no answer to CMD8/ACMD41 (empty slot)
    Cmd(u8, u32), // command index, interrupt status
    Data(u32),   // interrupt status
    Timeout(u8), // where
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        write!(f, "{:?}", self)
    }
}
impl core::error::Error for Error {}

#[derive(Clone, Copy)]
enum Rsp {
    None,
    R2,  // 136 bits, crc
    R1,  // 48, crc + index
    R1b, // 48 busy, crc + index
    R3,  // 48, no crc / index (OCR)
}

pub struct Sd {
    pub blocks: u32,   // card size in 512-byte blocks
    hc: bool,          // block addressed (SDHC/SDXC)
    rca: u32,
    pub last_err: Cell<Option<Error>>,
}

fn wait(mask: u32, limit: u32, at: u8) -> Result<u32, Error> {
    for _ in 0..limit {
        let s = rd(INT_ST);
        if s & (mask | CMD_ERR | DATA_ERR) != 0 {
            return Ok(s);
        }
    }
    Err(Error::Timeout(at))
}

fn cmd(idx: u8, arg: u32, rsp: Rsp, data: u32) -> Result<u32, Error> {
    // command (and data) line free
    for _ in 0..1_000_000 {
        if rd(PRES) & (if data != 0 { 3 } else { 1 }) == 0 {
            break;
        }
    }
    wr(INT_ST, 0xFFFF_FFFF);
    wr(CMD_ARG, arg);
    let (typ, crc, ix) = match rsp {
        Rsp::None => (0, 0, 0),
        Rsp::R2 => (1, 1, 0),
        Rsp::R1 => (2, 1, 1),
        Rsp::R1b => (3, 1, 1),
        Rsp::R3 => (2, 0, 0),
    };
    wr(CMD_XFR_TYP, (idx as u32) << 24 | data << 21 | ix << 20 | crc << 19 | typ << 16);
    let s = wait(CC, 10_000_000, idx)?;
    if s & CMD_ERR != 0 {
        wr(INT_ST, s);
        return Err(Error::Cmd(idx, s));
    }
    wr(INT_ST, CC);
    Ok(rd(RSP0))
}

fn set_clock(sdclkfs: u32, dvs: u32) {
    let s = rd(SYS) & !(0xFFF0);
    wr(SYS, s | sdclkfs << 8 | dvs << 4);
    for _ in 0..1_000_000 {
        if rd(PRES) & (1 << 3) != 0 {
            break; // SDSTB
        }
    }
}

impl Sd {
    /// Clocks, pins, the card's identification and its switch to 4-bit, ~25 MHz.
    pub fn init() -> Result<Sd, Error> {
        unsafe {
            // PLL2 PFD0 = 528 * 18 / 24 = 396 MHz, ungated
            let pfd = reg(0x400D_8100);
            pfd.write_volatile((pfd.read_volatile() & !0xBF) | 24);
            // USDHC1 clock: PFD0 (CSCMR1 bit 16), / 2 (CSCDR1 PODF bits 11..13 = 1), gate on (CCGR6 CG1)
            let cscmr1 = reg(0x400F_C01C);
            cscmr1.write_volatile(cscmr1.read_volatile() | 1 << 16);
            let cscdr1 = reg(0x400F_C024);
            cscdr1.write_volatile((cscdr1.read_volatile() & !(7 << 11)) | 1 << 11);
            let ccgr6 = reg(0x400F_C080);
            ccgr6.write_volatile(ccgr6.read_volatile() | 3 << 2);
            // pins: ALT0; CLK: keeper, DSE 1, speed 2; CMD and data: also a 47k pull-up
            const CLK_PAD: u32 = 1 << 12 | 1 << 3 | 2 << 6;
            const DAT_PAD: u32 = CLK_PAD | 1 << 13 | 1 << 14;
            for i in 0..6 {
                reg(0x401F_81BC + 4 * i).write_volatile(0);
                reg(0x401F_83AC + 4 * i).write_volatile(if i == 1 { CLK_PAD } else { DAT_PAD });
            }
        }
        // controller reset, data timeout max, ~390 kHz, 1-bit, little endian, all status bits on (no interrupts)
        wr(SYS, rd(SYS) | 1 << 24);
        for _ in 0..1_000_000 {
            if rd(SYS) & (1 << 24) == 0 {
                break;
            }
        }
        wr(VEND, rd(VEND) & !(1 << 8)); // SD clock gated when idle (FRC_SDCLK_ON off)
        wr(SYS, (rd(SYS) & !(0xF << 16)) | 0xE << 16);
        set_clock(0x80, 1); // 198 MHz / 256 / 2 = 387 kHz
        wr(PROT, 2 << 4);
        wr(INT_EN, 0xFFFF_FFFF);
        wr(INT_SIG, 0);
        wr(MIX, 0);
        // 80 clocks to the card
        wr(SYS, rd(SYS) | 1 << 27);
        for _ in 0..1_000_000 {
            if rd(SYS) & (1 << 27) == 0 {
                break;
            }
        }

        cmd(0, 0, Rsp::None, 0)?;
        let v2 = match cmd(8, 0x1AA, Rsp::R1, 0) {
            Ok(r) => r & 0xFFF == 0x1AA,
            Err(Error::Cmd(..)) | Err(Error::Timeout(_)) => false,
            Err(e) => return Err(e),
        };
        // ACMD41 until the card is ready (up to ~1 s)
        let mut ocr = 0;
        let mut ready = false;
        for _ in 0..2000 {
            if cmd(55, 0, Rsp::R1, 0).is_err() {
                return Err(Error::NoCard);
            }
            ocr = cmd(41, if v2 { 0x4030_0000 } else { 0x0030_0000 }, Rsp::R3, 0).map_err(|_| Error::NoCard)?;
            if ocr & (1 << 31) != 0 {
                ready = true;
                break;
            }
            cortex_m::asm::delay(300_000); // ~0.5 ms
        }
        if !ready {
            return Err(Error::NoCard);
        }
        let hc = ocr & (1 << 30) != 0;
        cmd(2, 0, Rsp::R2, 0)?; // CID
        let rca = cmd(3, 0, Rsp::R1, 0)? & 0xFFFF_0000;
        cmd(9, rca, Rsp::R2, 0)?; // CSD: the size
        // R2 in RSP0..3 is the CSD without its CRC byte, shifted right 8: CSD bit n is at bit n - 8
        let csd = [rd(RSP0), rd(RSP1), rd(RSP2), rd(RSP3)];
        let bit = |n: u32| (csd[((n - 8) / 32) as usize] >> ((n - 8) % 32)) & 1;
        let field = |hi: u32, lo: u32| (lo..=hi).rev().fold(0u32, |v, n| v << 1 | bit(n));
        let blocks = if field(127, 126) == 1 {
            (field(69, 48) + 1) * 1024 // CSD v2: (C_SIZE + 1) * 512 KB
        } else {
            let c_size = field(73, 62);
            let mult = field(49, 47);
            let bl_len = field(83, 80);
            ((c_size + 1) << (mult + 2)) << bl_len >> 9
        };
        cmd(7, rca, Rsp::R1b, 0)?; // select
        cmd(55, rca, Rsp::R1, 0)?;
        cmd(6, 2, Rsp::R1, 0)?; // 4-bit
        wr(PROT, 2 << 4 | 1 << 1);
        cmd(16, 512, Rsp::R1, 0)?;
        set_clock(0x04, 0); // 198 / 8 = 24.75 MHz
        wr(WTMK, 128 << 16 | 128);
        Ok(Sd { blocks, hc, rca, last_err: Cell::new(None) })
    }

    fn addr(&self, block: u32) -> u32 {
        if self.hc { block } else { block * 512 }
    }

    pub fn read_blocks(&self, start: u32, out: &mut [[u8; 512]]) -> Result<(), Error> {
        let n = out.len() as u32;
        if n == 0 {
            return Ok(());
        }
        wr(BLK_ATT, n << 16 | 512);
        let multi = n > 1;
        // read, block count, multi + auto CMD12
        wr(MIX, 1 << 4 | (multi as u32) << 5 | 1 << 1 | (multi as u32) << 2);
        cmd(if multi { 18 } else { 17 }, self.addr(start), Rsp::R1, 1)?;
        for b in out.iter_mut() {
            let s = wait(BRR, 10_000_000, 17)?;
            if s & DATA_ERR != 0 {
                return Err(Error::Data(s));
            }
            wr(INT_ST, BRR);
            for w in 0..128 {
                b[w * 4..w * 4 + 4].copy_from_slice(&rd(DATA).to_le_bytes());
            }
        }
        let s = wait(TC, 10_000_000, 18)?;
        if s & DATA_ERR != 0 {
            return Err(Error::Data(s));
        }
        wr(INT_ST, 0xFFFF_FFFF);
        Ok(())
    }

    pub fn write_blocks(&self, start: u32, data: &[[u8; 512]]) -> Result<(), Error> {
        let n = data.len() as u32;
        if n == 0 {
            return Ok(());
        }
        wr(BLK_ATT, n << 16 | 512);
        let multi = n > 1;
        wr(MIX, (multi as u32) << 5 | 1 << 1 | (multi as u32) << 2);
        cmd(if multi { 25 } else { 24 }, self.addr(start), Rsp::R1, 1)?;
        for b in data {
            let s = wait(BWR, 10_000_000, 24)?;
            if s & DATA_ERR != 0 {
                return Err(Error::Data(s));
            }
            wr(INT_ST, BWR);
            for w in 0..128 {
                wr(DATA, u32::from_le_bytes([b[w * 4], b[w * 4 + 1], b[w * 4 + 2], b[w * 4 + 3]]));
            }
        }
        let s = wait(TC, 50_000_000, 25)?;
        if s & DATA_ERR != 0 {
            return Err(Error::Data(s));
        }
        wr(INT_ST, 0xFFFF_FFFF);
        // the card programs: wait until it is out of busy (CMD13 state back to transfer)
        for _ in 0..200_000 {
            if let Ok(st) = cmd(13, self.rca, Rsp::R1, 0) {
                if (st >> 9) & 0xF == 4 && st & (1 << 8) != 0 {
                    return Ok(());
                }
            }
        }
        Err(Error::Timeout(13))
    }
}

impl BlockDevice for Sd {
    type Error = Error;
    fn read(&self, blocks: &mut [Block], start: BlockIdx) -> Result<(), Error> {
        // Block is a 512-byte array wrapper: read one at a time into its contents
        for (i, b) in blocks.iter_mut().enumerate() {
            let one: &mut [[u8; 512]; 1] = unsafe { &mut *(b.contents.as_mut_ptr() as *mut [[u8; 512]; 1]) };
            self.read_blocks(start.0 + i as u32, one).inspect_err(|e| self.last_err.set(Some(*e)))?;
        }
        Ok(())
    }
    fn write(&self, blocks: &[Block], start: BlockIdx) -> Result<(), Error> {
        for (i, b) in blocks.iter().enumerate() {
            let one: &[[u8; 512]; 1] = unsafe { &*(b.contents.as_ptr() as *const [[u8; 512]; 1]) };
            self.write_blocks(start.0 + i as u32, one).inspect_err(|e| self.last_err.set(Some(*e)))?;
        }
        Ok(())
    }
    fn num_blocks(&self) -> Result<BlockCount, Error> {
        Ok(BlockCount(self.blocks))
    }
}

/// No clock on the Teensy yet: files get this date.
pub struct Clock;
impl embedded_sdmmc::TimeSource for Clock {
    fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
        embedded_sdmmc::Timestamp { year_since_1970: 56, zero_indexed_month: 10, zero_indexed_day: 12, hours: 0, minutes: 0, seconds: 0 }
    }
}

/// 'D': start the card, then its size, file system and the files in its root (and in FIRMWARE/ if there is one).
#[unsafe(link_section = ".xip.sd_report")]
#[inline(never)]
pub fn report(out: &mut impl core::fmt::Write) {
    let t = cortex_m::peripheral::DWT::cycle_count();
    let sd = match Sd::init() {
        Ok(s) => s,
        Err(e) => {
            let _ = write!(out, "SD: {:?}\r\n", e);
            return;
        }
    };
    let ms = cortex_m::peripheral::DWT::cycle_count().wrapping_sub(t) / 600_000;
    let _ = write!(out, "SD: card {} MB ({} blocks), {} addressing, ready in {} ms\r\n",
        sd.blocks / 2048, sd.blocks, if sd.hc { "block" } else { "byte" }, ms);
    // read speed: 2 MB from the start of the card
    let mut buf = [[0u8; 512]; 16];
    let t = cortex_m::peripheral::DWT::cycle_count();
    let mut ok = true;
    for i in 0..256 {
        if let Err(e) = sd.read_blocks(i * 16, &mut buf) {
            let _ = write!(out, "SD: read error {:?} at block {}\r\n", e, i * 16);
            ok = false;
            break;
        }
    }
    if ok {
        let us = cortex_m::peripheral::DWT::cycle_count().wrapping_sub(t) / 600;
        let _ = write!(out, "SD: read 2 MB in {} ms = {} KB/s\r\n", us / 1000, 2048 * 1_000_000 / us.max(1));
    }
    let vm: embedded_sdmmc::VolumeManager<_, _, 2, 2, 1> = embedded_sdmmc::VolumeManager::new_with_limits(sd, Clock, 0);
    let vol = match vm.open_volume(embedded_sdmmc::VolumeIdx(0)) {
        Ok(v) => v,
        Err(e) => {
            let _ = write!(out, "SD: no FAT volume: {:?}\r\n", e);
            return;
        }
    };
    let root = match vol.open_root_dir() {
        Ok(r) => r,
        Err(e) => {
            let _ = write!(out, "SD: root: {:?}\r\n", e);
            return;
        }
    };
    let mut n = 0;
    let _ = root.iterate_dir(|e| {
        if n < 40 {
            let _ = write!(out, "SD:   {}{} {}\r\n", e.name, if e.attributes.is_directory() { "/" } else { "" }, e.size);
        }
        n += 1;
        core::ops::ControlFlow::Continue(())
    });
    let _ = write!(out, "SD: {} entries in the root\r\n", n);
}

/// 'D1': set the card up: ROMS/, SAMPLES/, DATA/ and a README.TXT, then a write test (1 MB written, timed,
/// read back and checked, deleted). Existing files are left alone.
#[cfg(fw_menu)]
#[unsafe(link_section = ".xip.sd_setup")]
#[inline(never)]
pub fn setup(out: &mut impl core::fmt::Write) {
    use embedded_sdmmc::{Error as FsErr, Mode};
    let sd = match Sd::init() {
        Ok(s) => s,
        Err(e) => {
            let _ = write!(out, "SD: {:?}\r\n", e);
            return;
        }
    };
    let vm: embedded_sdmmc::VolumeManager<_, _, 2, 2, 1> = embedded_sdmmc::VolumeManager::new_with_limits(sd, Clock, 0);
    let vol = match vm.open_volume(embedded_sdmmc::VolumeIdx(0)) {
        Ok(v) => v,
        Err(e) => {
            let _ = write!(out, "SD: no FAT volume: {:?}\r\n", e);
            return;
        }
    };
    let root = match vol.open_root_dir() {
        Ok(r) => r,
        Err(e) => {
            let _ = write!(out, "SD: root: {:?}\r\n", e);
            return;
        }
    };
    for d in ["ROMS", "SAMPLES", "DATA"] {
        match root.make_dir_in_dir(d) {
            Ok(()) => { let _ = write!(out, "SD: made {}/\r\n", d); }
            Err(FsErr::DirAlreadyExists) => { let _ = write!(out, "SD: {}/ already there\r\n", d); }
            Err(e) => { let _ = write!(out, "SD: mkdir {}: {:?}\r\n", d, e); return; }
        }
    }
    const README: &[u8] = b"BBL Mod Swapper card\r\n\r\nROMS\\     firmware images the menu lists\r\nSAMPLES\\  the shared sample library, any folders inside, every firmware can pull from it\r\nDATA\\     each firmware's own saves (presets, patterns, projects), one folder per firmware\r\n";
    match root.open_file_in_dir("README.TXT", Mode::ReadWriteCreateOrTruncate) {
        Ok(f) => {
            let r = f.write(README).and_then(|_| f.close());
            let _ = write!(out, "SD: README.TXT {:?}\r\n", r);
        }
        Err(e) => { let _ = write!(out, "SD: README.TXT: {:?}\r\n", e); return; }
    }
    // write test in DATA/: 1 MB in 4 KB pieces, a pattern that depends on the position
    let data = match root.open_dir("DATA") {
        Ok(d) => d,
        Err(e) => { let _ = write!(out, "SD: open DATA: {:?}\r\n", e); return; }
    };
    let mut buf = [0u8; 4096];
    let fill = |buf: &mut [u8; 4096], k: u32| {
        for (i, b) in buf.iter_mut().enumerate() { *b = ((i as u32).wrapping_mul(31) ^ k.wrapping_mul(97)) as u8; }
    };
    let t = cortex_m::peripheral::DWT::cycle_count();
    match data.open_file_in_dir("WTEST.BIN", Mode::ReadWriteCreateOrTruncate) {
        Ok(f) => {
            for k in 0..256u32 {
                fill(&mut buf, k);
                if let Err(e) = f.write(&buf) { let _ = write!(out, "SD: write error {:?} at {} KB\r\n", e, k * 4); return; }
            }
            if let Err(e) = f.close() { let _ = write!(out, "SD: close: {:?}\r\n", e); return; }
        }
        Err(e) => { let _ = write!(out, "SD: create WTEST.BIN: {:?}\r\n", e); return; }
    }
    let us = cortex_m::peripheral::DWT::cycle_count().wrapping_sub(t) / 600;
    let _ = write!(out, "SD: wrote 1 MB in {} ms = {} KB/s\r\n", us / 1000, 1024 * 1_000_000 / us.max(1));
    let mut bad = 0u32;
    let mut got = 0u32;
    match data.open_file_in_dir("WTEST.BIN", Mode::ReadOnly) {
        Ok(f) => {
            let mut want = [0u8; 4096];
            for k in 0..256u32 {
                fill(&mut want, k);
                let mut n = 0;
                while n < buf.len() {
                    match f.read(&mut buf[n..]) {
                        Ok(0) => break,
                        Ok(m) => n += m,
                        Err(e) => { let _ = write!(out, "SD: read back error {:?}\r\n", e); return; }
                    }
                }
                got += n as u32;
                bad += buf[..n].iter().zip(want[..n].iter()).filter(|(a, b)| a != b).count() as u32;
            }
            let _ = f.close();
        }
        Err(e) => { let _ = write!(out, "SD: reopen WTEST.BIN: {:?}\r\n", e); return; }
    }
    let _ = write!(out, "SD: read back {} bytes, {} wrong\r\n", got, bad);
    let _ = write!(out, "SD: delete test file {:?}\r\n", data.delete_entry_in_dir("WTEST.BIN"));
}

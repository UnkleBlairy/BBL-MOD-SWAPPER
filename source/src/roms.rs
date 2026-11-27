//! The ROM library (menu slot only): firmware images on the SD card in ROMS/, copied into the Teensy's flash slots
//! when picked, and started from there.
//!
//! Flash: slot 0 (0x60000000) is Felucca plus this menu. Slots 1..3 are a cache of ROMs from the card. Each image is
//! linked for one slot address, so the card holds one file per slot: ROMS/<STEM>.R1, .R2, .R3. A file is a 512-byte
//! header (below) then the image, from the slot's first byte.
//!
//! The last 4 KB sector of each slot is its tag: a copy of the header of the ROM in it, then a log of u32 use stamps
//! (each start programs the next free word with a number higher than any before, so the least recently used slot is
//! the one with the lowest last stamp; when the log is full the sector is erased and rewritten). A slot is valid only
//! with a tag and a boot header whose self pointer is the slot's own address. The tag is written last, so a copy cut
//! short by power loss leaves no valid slot.

use embedded_sdmmc::{Mode, RawDirectory, RawFile, VolumeIdx, VolumeManager};

use crate::{flashw, sd};

pub const BASE: [u32; 4] = [0x6000_0000, 0x6020_0000, 0x6040_0000, 0x6060_0000];
pub const END: [u32; 4] = [0x6020_0000, 0x6040_0000, 0x6060_0000, 0x607C_0000];
pub const NSLOT: usize = 4;
const MAGIC: &[u8; 8] = b"FM1ROM01";
const HDR: usize = 512;

pub fn tag_at(s: usize) -> u32 {
    END[s] - 0x1000
}
/// The biggest image a slot holds (everything below its tag).
pub fn room(s: usize) -> u32 {
    tag_at(s) - BASE[s]
}

/// A ROM header: 0 magic, 8 slot, 12 image length, 16 image CRC-32, 20 base address, 24 name[32], 56 version[16],
/// 72 stem[8] (the card file name without its extension), 80 the save memory's size (0: none). Strings are NUL padded.
#[derive(Clone, Copy)]
pub struct Header {
    pub slot: u32,
    pub len: u32,
    pub crc: u32,
    pub base: u32,
    pub name: [u8; 32],
    pub version: [u8; 16],
    pub stem: [u8; 8],
    pub nor: u32, // the save memory's size: it sits just below the tag (nor.rs); 0 = none
}
impl Header {
    pub fn parse(b: &[u8]) -> Option<Header> {
        if b.len() < 80 || &b[..8] != MAGIC {
            return None;
        }
        let u = |o: usize| u32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]]);
        let mut h = Header { slot: u(8), len: u(12), crc: u(16), base: u(20), name: [0; 32], version: [0; 16], stem: [0; 8], nor: u(80) };
        h.name.copy_from_slice(&b[24..56]);
        h.version.copy_from_slice(&b[56..72]);
        h.stem.copy_from_slice(&b[72..80]);
        let s = h.slot as usize;
        if s == 0 || s >= NSLOT || h.base != BASE[s] || h.len == 0 || h.len > room(s) || h.nor % 4096 != 0
            || h.len + h.nor > room(s)
        {
            return None;
        }
        Some(h)
    }
}

// CRC-32 (IEEE, as zlib)
const fn crc_table() -> [u32; 256] {
    let mut t = [0u32; 256];
    let mut i = 0;
    while i < 256 {
        let mut c = i as u32;
        let mut k = 0;
        while k < 8 {
            c = if c & 1 != 0 { 0xEDB8_8320 ^ (c >> 1) } else { c >> 1 };
            k += 1;
        }
        t[i] = c;
        i += 1;
    }
    t
}
static CRC_T: [u32; 256] = crc_table();
/// Running CRC: start with 0, feed pieces, the value is the CRC of everything so far.
pub fn crc32(crc: u32, data: &[u8]) -> u32 {
    let mut c = !crc;
    for &b in data {
        c = CRC_T[((c ^ b as u32) & 0xFF) as usize] ^ (c >> 8);
    }
    !c
}

/// What a flash slot holds: its header (None: empty or not valid) and its last use stamp (0: never).
#[derive(Clone, Copy)]
pub struct SlotInfo {
    pub hdr: Option<Header>,
    pub last_use: u32,
}

fn word(a: u32) -> u32 {
    unsafe { (a as *const u32).read_volatile() }
}

pub fn slot_info(s: usize) -> SlotInfo {
    let none = SlotInfo { hdr: None, last_use: 0 };
    if s == 0 || s >= NSLOT {
        return none;
    }
    let tag = unsafe { core::slice::from_raw_parts(tag_at(s) as *const u8, HDR) };
    let Some(h) = Header::parse(tag) else { return none };
    // boot header (IVT) at +0x1000, with its self pointer at +0x14: linked for this slot
    let ivt = BASE[s] + 0x1000;
    if word(ivt) != 0x4020_00D1 || word(ivt + 0x14) != ivt || h.slot as usize != s {
        return none;
    }
    let mut last = 0;
    let mut a = tag_at(s) + HDR as u32;
    while a < END[s] {
        let w = word(a);
        if w == 0xFFFF_FFFF {
            break;
        }
        last = w;
        a += 4;
    }
    SlotInfo { hdr: Some(h), last_use: last }
}

/// The next use stamp: one more than any in the slots.
fn next_stamp() -> u32 {
    (1..NSLOT).map(|s| slot_info(s).last_use).max().unwrap_or(0) + 1
}

/// Record a start of slot `s` (it becomes the most recently used).
pub fn stamp(s: usize) {
    let Some(_) = slot_info(s).hdr else { return };
    let v = next_stamp();
    let mut a = tag_at(s) + HDR as u32;
    while a < END[s] && word(a) != 0xFFFF_FFFF {
        a += 4;
    }
    if a >= END[s] {
        // the log is full: rewrite the sector with the header and this one stamp
        let mut h = [0u8; HDR];
        h.copy_from_slice(unsafe { core::slice::from_raw_parts(tag_at(s) as *const u8, HDR) });
        flashw::erase_sector(tag_at(s));
        flashw::write(tag_at(s), &h);
        a = tag_at(s) + HDR as u32;
    }
    flashw::write(a, &v.to_le_bytes());
}

// ---- the card ----

pub type Vm = VolumeManager<sd::Sd, sd::Clock, 4, 4, 1>;
pub struct Lib {
    pub vm: Vm,
    pub root: RawDirectory,
    pub roms: RawDirectory,
}
impl Lib {
    pub fn open() -> Option<Lib> {
        let card = sd::Sd::init().ok()?;
        let vm: Vm = VolumeManager::new_with_limits(card, sd::Clock, 0);
        let vol = vm.open_raw_volume(VolumeIdx(0)).ok()?;
        let root = vm.open_root_dir(vol).ok()?;
        let roms = match vm.open_dir(root, "ROMS") {
            Ok(d) => d,
            Err(_) => {
                vm.make_dir_in_dir(root, "ROMS").ok()?;
                vm.open_dir(root, "ROMS").ok()?
            }
        };
        Some(Lib { vm, root, roms })
    }
}

/// One menu entry: a ROM by its stem, where it is (flash slot, card files) and its texts for the menu.
#[derive(Clone, Copy)]
pub struct Entry {
    pub stem: [u8; 8],
    pub name: [u8; 33],    // C string
    pub detail: [u8; 33],  // C string
    pub slot: Option<u8>,  // in this flash slot
    pub slot_crc: u32,     // ... with this image
    pub card: u8,          // bit s: the card has a file for slot s
    pub card_crc: [u32; 4],
    pub card_ver: [u8; 16],
}
impl Entry {
    const fn empty() -> Entry {
        Entry { stem: [0; 8], name: [0; 33], detail: [0; 33], slot: None, slot_crc: 0, card: 0, card_crc: [0; 4], card_ver: [0; 16] }
    }
    /// In flash and up to date with the card (or the card has nothing newer for that slot).
    pub fn ready(&self) -> Option<usize> {
        let s = self.slot? as usize;
        if self.card & 1 << s != 0 && self.card_crc[s] != self.slot_crc {
            return None;
        }
        Some(s)
    }
}

pub const MAX: usize = 16;
/// The menu's SAMPLE LAB: starts the mod that hosts it (the Jam Box) with its Sample Lab open.
pub const LAB_STEM: [u8; 8] = *b"\0SMPLAB\0";
pub const LAB_HOST: [u8; 8] = *b"JAMBOX\0\0";
pub struct Catalog {
    pub n: usize,
    pub e: [Entry; MAX],
}

fn cstr_copy(dst: &mut [u8], src: &[u8]) {
    let n = src.iter().position(|&c| c == 0).unwrap_or(src.len()).min(dst.len() - 1);
    dst[..n].copy_from_slice(&src[..n]);
    dst[n..].fill(0);
}
fn upper(b: &mut [u8]) {
    for c in b.iter_mut() {
        c.make_ascii_uppercase();
    }
}

impl Catalog {
    pub const fn new() -> Catalog {
        Catalog { n: 0, e: [Entry::empty(); MAX] }
    }
    fn find_or_add(&mut self, stem: &[u8; 8]) -> Option<&mut Entry> {
        let i = match self.e[..self.n].iter().position(|e| &e.stem == stem) {
            Some(i) => i,
            None => {
                if self.n == MAX {
                    return None;
                }
                self.e[self.n] = Entry::empty();
                self.e[self.n].stem = *stem;
                self.n += 1;
                self.n - 1
            }
        };
        Some(&mut self.e[i])
    }

    /// Entry 0 is the built-in firmware (slot 0); then the flash slots, then the card's ROMS/.
    pub fn build(&mut self, builtin: (&[u8], &[u8]), lib: Option<&Lib>) {
        self.n = 1;
        self.e[0] = Entry::empty();
        self.e[0].stem = *b"\0BUILTIN";
        self.e[0].slot = Some(0);
        cstr_copy(&mut self.e[0].name, builtin.0);
        cstr_copy(&mut self.e[0].detail, builtin.1);
        for s in 1..NSLOT {
            let si = slot_info(s);
            if let Some(h) = si.hdr {
                if let Some(e) = self.find_or_add(&h.stem) {
                    e.slot = Some(s as u8);
                    e.slot_crc = h.crc;
                    cstr_copy(&mut e.name, &h.name);
                    e.card_ver = h.version;
                }
            }
        }
        if let Some(lib) = lib {
            // file names first (the directory is locked while it is walked), then their headers
            let mut found: [([u8; 12], u8); 24] = [([0; 12], 0); 24];
            let mut nf = 0;
            let _ = lib.vm.iterate_dir(lib.roms, |d| {
                let ext = d.name.extension();
                if !d.attributes.is_directory() && ext.len() == 2 && ext[0] == b'R' && (b'1'..=b'3').contains(&ext[1]) && nf < found.len() {
                    let mut nm = [0u8; 12];
                    let b = d.name.base_name();
                    nm[..b.len()].copy_from_slice(b);
                    nm[b.len()] = b'.';
                    nm[b.len() + 1..b.len() + 3].copy_from_slice(ext);
                    found[nf] = (nm, (ext[1] - b'0'));
                    nf += 1;
                }
                core::ops::ControlFlow::Continue(())
            });
            for &(nm, s) in &found[..nf] {
                let len = nm.iter().position(|&c| c == 0).unwrap_or(12);
                let Ok(name) = core::str::from_utf8(&nm[..len]) else { continue };
                let Ok(f) = lib.vm.open_file_in_dir(lib.roms, name, Mode::ReadOnly) else { continue };
                let mut b = [0u8; HDR];
                let got = lib.vm.read(f, &mut b).unwrap_or(0);
                let flen = lib.vm.file_length(f).unwrap_or(0);
                let _ = lib.vm.close_file(f);
                let Some(h) = Header::parse(&b[..got]) else { continue };
                if h.slot != s as u32 || flen != HDR as u32 + h.len {
                    continue;
                }
                if let Some(e) = self.find_or_add(&h.stem) {
                    e.card |= 1 << s;
                    e.card_crc[s as usize] = h.crc;
                    if e.name[0] == 0 {
                        cstr_copy(&mut e.name, &h.name);
                    }
                    e.card_ver = h.version;
                }
            }
        }
        for e in self.e[1..self.n].iter_mut() {
            upper(&mut e.name);
            let mut d = [0u8; 33];
            let v = e.card_ver.iter().position(|&c| c == 0).unwrap_or(16);
            let mut w = Cursor { b: &mut d, n: 0 };
            w.put(&e.card_ver[..v]);
            w.put(if e.ready().is_some() { b"\nREADY" } else if e.card != 0 { b"\nON CARD" } else { b"" });
            e.detail = d;
        }
        // SAMPLE LAB (record, trim, send), when the Jam Box is in flash or on the card
        if self.n < MAX && self.e[..self.n].iter().any(|e| e.stem == LAB_HOST) {
            let e = &mut self.e[self.n];
            *e = Entry::empty();
            e.stem = LAB_STEM;
            cstr_copy(&mut e.name, b"SAMPLE LAB");
            cstr_copy(&mut e.detail, b"RECORD + TRIM\nIN THE JAM BOX");
            self.n += 1;
        }
    }

    /// The flash slot a ROM from the card goes to: the stale one it is already in, else an empty one, else the least
    /// recently used, among the slots the card has a file for. None: no file fits.
    pub fn target(&self, i: usize) -> Option<usize> {
        let e = &self.e[i];
        if let Some(s) = e.slot {
            if e.card & 1 << s != 0 {
                return Some(s as usize);
            }
        }
        let cands = (1..NSLOT).filter(|&s| e.card & 1 << s != 0);
        if let Some(s) = cands.clone().find(|&s| slot_info(s).hdr.is_none()) {
            return Some(s);
        }
        cands.min_by_key(|&s| slot_info(s).last_use)
    }
}

struct Cursor<'a> {
    b: &'a mut [u8],
    n: usize,
}
impl Cursor<'_> {
    fn put(&mut self, s: &[u8]) {
        for &c in s {
            if self.n + 1 < self.b.len() {
                self.b[self.n] = c;
                self.n += 1;
            }
        }
    }
}

// ---- copying a ROM from the card into a slot, a step at a time (the main loop keeps the FM-1's screen going) ----

#[derive(Clone, Copy, PartialEq)]
pub enum Phase {
    Erase,
    Copy,
    Check,
    Restore, // the save memory from DATA/<STEM>.NOR (nor.rs), before the tag
    Done,
    Failed(&'static str),
}

pub struct Load {
    pub slot: usize,
    file: RawFile,
    hdr: [u8; HDR],
    len: u32,
    crc: u32,
    run_crc: u32,
    at: u32, // Erase: the next block; Copy: the next image byte; Restore: the next save memory byte
    nor: u32,
    save: Option<RawFile>,
    pub phase: Phase,
}

impl Load {
    pub fn start(lib: &Lib, stem: &[u8; 8], slot: usize) -> Result<Load, &'static str> {
        let mut nm = [0u8; 12];
        let sl = stem.iter().position(|&c| c == 0).unwrap_or(8);
        nm[..sl].copy_from_slice(&stem[..sl]);
        nm[sl..sl + 3].copy_from_slice(&[b'.', b'R', b'0' + slot as u8]);
        let name = core::str::from_utf8(&nm[..sl + 3]).map_err(|_| "NAME")?;
        let f = lib.vm.open_file_in_dir(lib.roms, name, Mode::ReadOnly).map_err(|_| "OPEN")?;
        let mut hdr = [0u8; HDR];
        if lib.vm.read(f, &mut hdr).unwrap_or(0) != HDR {
            let _ = lib.vm.close_file(f);
            return Err("READ");
        }
        let Some(h) = Header::parse(&hdr) else {
            let _ = lib.vm.close_file(f);
            return Err("HEADER");
        };
        if h.slot as usize != slot {
            let _ = lib.vm.close_file(f);
            return Err("SLOT");
        }
        Ok(Load { slot, file: f, hdr, len: h.len, crc: h.crc, run_crc: 0, at: BASE[slot], nor: h.nor, save: None, phase: Phase::Erase })
    }

    /// 0..100
    pub fn percent(&self) -> u32 {
        let blocks = (END[self.slot] - BASE[self.slot]) as u64;
        match self.phase {
            Phase::Erase => ((self.at - BASE[self.slot]) as u64 * 30 / blocks) as u32,
            Phase::Copy => 30 + ((self.at as u64) * 68 / self.len.max(1) as u64) as u32,
            Phase::Check => 98,
            Phase::Restore => 98 + (self.at as u64 * 2 / self.nor.max(1) as u64) as u32,
            Phase::Done => 100,
            Phase::Failed(_) => 0,
        }
    }

    /// One step: erase one 64 KB block (skipped if blank), or copy 4 KB, or check and tag. Returns the phase after it.
    pub fn step(&mut self, lib: &Lib) -> Phase {
        let s = self.slot;
        match self.phase {
            Phase::Erase => {
                let blk = unsafe { core::slice::from_raw_parts(self.at as *const u32, flashw::BLOCK as usize / 4) };
                if blk.iter().any(|&w| w != 0xFFFF_FFFF) && !flashw::erase_block(self.at) {
                    return self.fail(lib, "ERASE");
                }
                self.at += flashw::BLOCK;
                if self.at >= END[s] {
                    self.at = 0;
                    self.phase = Phase::Copy;
                }
            }
            Phase::Copy => {
                let mut buf = [0u8; 4096];
                let want = ((self.len - self.at) as usize).min(buf.len());
                let mut n = 0;
                while n < want {
                    match lib.vm.read(self.file, &mut buf[n..want]) {
                        Ok(0) | Err(_) => return self.fail(lib, "CARD READ"),
                        Ok(m) => n += m,
                    }
                }
                self.run_crc = crc32(self.run_crc, &buf[..n]);
                if !flashw::write(BASE[s] + self.at, &buf[..n]) {
                    return self.fail(lib, "WRITE");
                }
                self.at += n as u32;
                if self.at >= self.len {
                    self.phase = Phase::Check;
                }
            }
            Phase::Check => {
                let _ = lib.vm.close_file(self.file);
                let img = unsafe { core::slice::from_raw_parts(BASE[s] as *const u8, self.len as usize) };
                if self.run_crc != self.crc || crc32(0, img) != self.crc {
                    self.phase = Phase::Failed("CHECK");
                    return self.phase;
                }
                // its saves from the card, if it has a save memory and the card a copy
                self.at = 0;
                self.save = None;
                if self.nor > 0 {
                    if let Ok(data) = lib.vm.open_dir(lib.root, "DATA") {
                        let mut nm = [0u8; 12];
                        let st = &self.hdr[72..80];
                        let l = st.iter().position(|&c| c == 0).unwrap_or(8);
                        nm[..l].copy_from_slice(&st[..l]);
                        nm[l..l + 4].copy_from_slice(b".NOR");
                        if let Ok(name) = core::str::from_utf8(&nm[..l + 4]) {
                            self.save = lib.vm.open_file_in_dir(data, name, Mode::ReadOnly).ok();
                        }
                        let _ = lib.vm.close_dir(data);
                    }
                }
                self.phase = if self.save.is_some() { Phase::Restore } else { return self.tag() };
            }
            Phase::Restore => {
                let f = self.save.unwrap();
                let mut buf = [0u8; 4096];
                let mut n = 0;
                while n < buf.len() {
                    match lib.vm.read(f, &mut buf[n..]) {
                        Ok(0) | Err(_) => break,
                        Ok(m) => n += m,
                    }
                }
                let at = tag_at(s) - self.nor + self.at;
                if n > 0 && buf[..n].iter().any(|&b| b != 0xFF) && !flashw::write(at, &buf[..n]) {
                    let _ = lib.vm.close_file(f);
                    self.phase = Phase::Failed("SAVE WRITE");
                    return self.phase;
                }
                self.at += buf.len() as u32;
                if n < buf.len() || self.at >= self.nor {
                    let _ = lib.vm.close_file(f);
                    self.save = None;
                    return self.tag();
                }
            }
            Phase::Done | Phase::Failed(_) => {}
        }
        self.phase
    }

    /// The tag last: the slot is valid from here on.
    fn tag(&mut self) -> Phase {
        let s = self.slot;
        if !flashw::write(tag_at(s), &self.hdr) {
            self.phase = Phase::Failed("TAG");
            return self.phase;
        }
        self.phase = if slot_info(s).hdr.is_some() { Phase::Done } else { Phase::Failed("BOOT HEADER") };
        self.phase
    }

    fn fail(&mut self, lib: &Lib, why: &'static str) -> Phase {
        let _ = lib.vm.close_file(self.file);
        self.phase = Phase::Failed(why);
        self.phase
    }
}

// ---- 'U' upload: a file from the PC onto the card ----

pub trait Io {
    fn read(&mut self, buf: &mut [u8]) -> usize;
    fn write(&mut self, b: &[u8]);
}

/// `U <DIR>/<NAME.EXT> <len> <crc32 hex>`: replies "U GO", then takes the bytes in 4 KB pieces, answering "k" after
/// each, and ends with "U OK <crc> <ms>" or "U ERR <why>". DIR is ROMS, SAMPLES or DATA.
pub fn upload(lib: &Lib, args: &[u8], io: &mut impl Io) {
    let mut it = args.split(|&c| c == b' ').filter(|a| !a.is_empty());
    let (Some(path), Some(len), Some(crc)) = (it.next(), it.next(), it.next()) else {
        io.write(b"U ERR usage: U DIR/NAME.EXT len crc\r\n");
        return;
    };
    let len = len.iter().take_while(|c| c.is_ascii_digit()).fold(0u32, |v, &c| v * 10 + (c - b'0') as u32);
    let crc = crc.iter().fold(0u32, |v, &c| v << 4 | (c as char).to_digit(16).unwrap_or(0));
    let Some(slash) = path.iter().position(|&c| c == b'/') else {
        io.write(b"U ERR path\r\n");
        return;
    };
    let (Ok(dir), Ok(name)) = (core::str::from_utf8(&path[..slash]), core::str::from_utf8(&path[slash + 1..])) else {
        io.write(b"U ERR path\r\n");
        return;
    };
    if !["ROMS", "SAMPLES", "DATA"].contains(&dir) {
        io.write(b"U ERR dir\r\n");
        return;
    }
    let d = match lib.vm.open_dir(lib.root, dir) {
        Ok(d) => d,
        Err(_) => {
            io.write(b"U ERR open dir\r\n");
            return;
        }
    };
    let f = match lib.vm.open_file_in_dir(d, name, Mode::ReadWriteCreateOrTruncate) {
        Ok(f) => f,
        Err(_) => {
            let _ = lib.vm.close_dir(d);
            io.write(b"U ERR create\r\n");
            return;
        }
    };
    let t0 = cortex_m::peripheral::DWT::cycle_count();
    io.write(b"U GO\r\n");
    let mut buf = [0u8; 4096];
    let mut got = 0u32;
    let mut run = 0u32;
    let mut err: Option<&[u8]> = None;
    while got < len {
        let want = ((len - got) as usize).min(buf.len());
        let mut n = 0;
        let mut last = cortex_m::peripheral::DWT::cycle_count();
        while n < want {
            let m = io.read(&mut buf[n..want]);
            if m > 0 {
                n += m;
                last = cortex_m::peripheral::DWT::cycle_count();
            } else if cortex_m::peripheral::DWT::cycle_count().wrapping_sub(last) > 3 * 600_000_000 {
                break;
            }
        }
        if n < want {
            err = Some(b"U ERR timeout\r\n");
            break;
        }
        run = crc32(run, &buf[..n]);
        if lib.vm.write(f, &buf[..n]).is_err() {
            err = Some(b"U ERR write\r\n");
            break;
        }
        got += n as u32;
        io.write(b"k");
    }
    let _ = lib.vm.close_file(f);
    if err.is_none() && run != crc {
        err = Some(b"U ERR crc\r\n");
    }
    if let Some(e) = err {
        let _ = lib.vm.delete_entry_in_dir(d, name);
        let _ = lib.vm.close_dir(d);
        io.write(e);
        return;
    }
    // read it back from the card
    let mut back = 0u32;
    if let Ok(f) = lib.vm.open_file_in_dir(d, name, Mode::ReadOnly) {
        loop {
            match lib.vm.read(f, &mut buf) {
                Ok(0) | Err(_) => break,
                Ok(m) => back = crc32(back, &buf[..m]),
            }
        }
        let _ = lib.vm.close_file(f);
    }
    let _ = lib.vm.close_dir(d);
    let ms = cortex_m::peripheral::DWT::cycle_count().wrapping_sub(t0) / 600_000;
    let mut l = [0u8; 64];
    let mut w = Cursor { b: &mut l, n: 0 };
    w.put(if back == crc { b"U OK " } else { b"U ERR readback " });
    put_hex(&mut w, back);
    w.put(b" ");
    put_dec(&mut w, ms);
    w.put(b"\r\n");
    let n = w.n;
    io.write(&l[..n]);
}

fn put_hex(w: &mut Cursor, v: u32) {
    for i in (0..8).rev() {
        let d = (v >> (i * 4)) & 0xF;
        w.put(&[b"0123456789abcdef"[d as usize]]);
    }
}
fn put_dec(w: &mut Cursor, v: u32) {
    let mut d = [0u8; 10];
    let mut n = 0;
    let mut x = v;
    loop {
        d[n] = b'0' + (x % 10) as u8;
        n += 1;
        x /= 10;
        if x == 0 {
            break;
        }
    }
    for i in (0..n).rev() {
        w.put(&[d[i]]);
    }
}

/// 'Z': `Z<n>` forgets flash slot n (erases its tag; the menu stops listing it), `Z DIR/NAME` deletes a card file.
pub fn remove(lib: Option<&Lib>, args: &[u8]) -> &'static str {
    let a = args.trim_ascii();
    if a.len() == 1 && (b'1'..=b'3').contains(&a[0]) {
        return if flashw::erase_sector(tag_at((a[0] - b'0') as usize)) { "Z OK slot" } else { "Z ERR erase" };
    }
    let Some(lib) = lib else { return "Z ERR no card" };
    let Some(slash) = a.iter().position(|&c| c == b'/') else { return "Z ERR path" };
    let (Ok(dir), Ok(name)) = (core::str::from_utf8(&a[..slash]), core::str::from_utf8(&a[slash + 1..])) else { return "Z ERR path" };
    let Ok(d) = lib.vm.open_dir(lib.root, dir) else { return "Z ERR dir" };
    let r = lib.vm.delete_entry_in_dir(d, name);
    let _ = lib.vm.close_dir(d);
    if r.is_ok() { "Z OK file" } else { "Z ERR delete" }
}

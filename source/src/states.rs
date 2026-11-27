//! Save states: the whole running mod, as an emulator saves a game. The mod's own memory is three ranges the linker
//! keeps apart (build.rs: libfelucca.a's .data, .bss and OCRAM buffers between __smod_* and __emod_*); only the mod
//! writes them, and only when the main loop calls it. With the mod paused (the overlay is open), a save copies them to
//! the card and a load copies them back: the mod goes on exactly where it was saved (its sound, pattern, screen).
//! Not in a state: the mod's save memory (nor.rs, its own projects and samples in flash), like a cartridge's battery save,
//! EXCEPT the samples of a mod whose hook says which part holds them (smem: the Jam Box's table and the data in use).
//! Those are kept once per set on the card, in a pack named by their CRC (STATES/<STEM>/<CRC>.SMP), which the state
//! names: a save writes the pack only when the card hasn't got that set yet (usually it has: no extra wait). A load
//! checks both files, then writes the pack back into the flash a sector at a time, skipping every sector the flash
//! already has the same, then the RAM.
//!
//! On the card: STATES/<STEM>/01.SST .. 16.SST, a 512-byte header, then the three ranges one after the other.
//!   0 "FM1STATE", 8 u32 version 1, 12 u32 the ROM's CRC (its tag, roms.rs), 16 u32 the body's CRC-32,
//!   20 3 x (u32 address, u32 length), 44 char stem[8], 52 u32 smem offset (in the save memory), 56 u32 smem length,
//!   60 u32 the pack's CRC (its name; smem length 0: no pack)
//! A state is used only by the very same build (the ROM CRC and the three ranges): anything else reads as OLD.
//! A load reads the file twice: once to check the CRC (nothing changed yet), then into the mod's memory.

use embedded_sdmmc::{Mode, RawDirectory, RawFile, RawVolume};

use crate::lab::Vm;

include!(concat!(env!("OUT_DIR"), "/fwcfg.rs"));

pub const SLOTS: usize = 16;
const HDR: usize = 512;
const MAGIC: &[u8; 8] = b"FM1STATE";
const SAVE_CHUNK: usize = 8192; // ~35 ms of card writing a step: the FM-1 link goes on between steps
const LOAD_CHUNK: usize = 16384;

unsafe extern "C" {
    static mut __smod_data: u8;
    static mut __emod_data: u8;
    static mut __smod_bss: u8;
    static mut __emod_bss: u8;
    static mut __smod_uninit: u8;
    static mut __emod_uninit: u8;
}

fn ranges() -> [(u32, u32); 3] {
    unsafe {
        let r = |a: *mut u8, b: *mut u8| (a as u32, b as u32 - a as u32);
        [
            r(&raw mut __smod_data, &raw mut __emod_data),
            r(&raw mut __smod_bss, &raw mut __emod_bss),
            r(&raw mut __smod_uninit, &raw mut __emod_uninit),
        ]
    }
}
fn body_len() -> u32 {
    ranges().iter().map(|r| r.1).sum()
}
const SEC: u32 = 4096;
/// Sample packs only in builds whose hook has them (the code stays out of the others: SLOOP is tight in slot 3).
const PACKS: bool = cfg!(fw_hook_jambox);
/// The save memory's part a state carries (offset, length; whole sectors). (0, 0): none.
#[cfg(fw_hook_jambox)]
fn smem() -> (u32, u32) {
    if NOR_SIZE == 0 { (0, 0) } else { crate::hooks::jambox::state_mem() }
}
#[cfg(not(fw_hook_jambox))]
fn smem() -> (u32, u32) {
    (0, 0)
}
fn u32at(h: &[u8], o: usize) -> u32 {
    u32::from_le_bytes([h[o], h[o + 1], h[o + 2], h[o + 3]])
}
/// A header's smem, if it is a sane one for this build.
fn hdr_smem(h: &[u8]) -> Option<(u32, u32)> {
    let (o, n) = (u32at(h, 52), u32at(h, 56));
    (o % SEC == 0 && n % SEC == 0 && o <= NOR_SIZE && n <= NOR_SIZE - o).then_some((o, n))
}
/// The running ROM's CRC, from its tag (0 without one).
fn rom_crc() -> u32 {
    let t = unsafe { core::slice::from_raw_parts(TAG_ADDR as *const u8, 20) };
    if &t[..8] == b"FM1ROM01" { u32::from_le_bytes([t[16], t[17], t[18], t[19]]) } else { 0 }
}
fn header(crc: u32, sm: (u32, u32), pcrc: u32) -> [u8; HDR] {
    let mut h = [0u8; HDR];
    h[..8].copy_from_slice(MAGIC);
    h[8..12].copy_from_slice(&1u32.to_le_bytes());
    h[12..16].copy_from_slice(&rom_crc().to_le_bytes());
    h[16..20].copy_from_slice(&crc.to_le_bytes());
    for (i, (a, n)) in ranges().iter().enumerate() {
        h[20 + i * 8..24 + i * 8].copy_from_slice(&a.to_le_bytes());
        h[24 + i * 8..28 + i * 8].copy_from_slice(&n.to_le_bytes());
    }
    let l = STEM.len().min(8);
    h[44..44 + l].copy_from_slice(&STEM.as_bytes()[..l]);
    h[52..56].copy_from_slice(&sm.0.to_le_bytes());
    h[56..60].copy_from_slice(&sm.1.to_le_bytes());
    h[60..64].copy_from_slice(&pcrc.to_le_bytes());
    h
}
/// The header is this build's (all but the body's CRC).
fn ours(h: &[u8]) -> bool {
    let me = header(0, (0, 0), 0);
    h.len() >= HDR && h[..16] == me[..16] && h[20..52] == me[20..52]
}

const fn crc_table() -> [u32; 256] {
    let mut t = [0u32; 256];
    let mut n = 0;
    while n < 256 {
        let mut c = n as u32;
        let mut k = 0;
        while k < 8 {
            c = if c & 1 != 0 { 0xEDB8_8320 ^ (c >> 1) } else { c >> 1 };
            k += 1;
        }
        t[n] = c;
        n += 1;
    }
    t
}
static CRC: [u32; 256] = crc_table();
fn crc_add(mut c: u32, b: &[u8]) -> u32 {
    for &x in b {
        c = CRC[((c ^ x as u32) & 255) as usize] ^ (c >> 8);
    }
    c
}

fn name(slot: usize) -> [u8; 6] {
    let mut n = *b"00.SST";
    n[0] += ((slot + 1) / 10) as u8;
    n[1] += ((slot + 1) % 10) as u8;
    n
}

/// STATES/<STEM>/, made when `make` and missing.
fn dir(vm: &Vm, vol: RawVolume, make: bool) -> Option<RawDirectory> {
    let root = vm.open_root_dir(vol).ok()?;
    let st = match vm.open_dir(root, "STATES") {
        Ok(d) => Some(d),
        Err(_) if make => vm.make_dir_in_dir(root, "STATES").ok().and_then(|_| vm.open_dir(root, "STATES").ok()),
        Err(_) => None,
    };
    let _ = vm.close_dir(root);
    let st = st?;
    let l = STEM.len().min(8);
    let stem = core::str::from_utf8(&STEM.as_bytes()[..l]).unwrap_or("MOD");
    let d = match vm.open_dir(st, stem) {
        Ok(d) => Some(d),
        Err(_) if make => vm.make_dir_in_dir(st, stem).ok().and_then(|_| vm.open_dir(st, stem).ok()),
        Err(_) => None,
    };
    let _ = vm.close_dir(st);
    d
}

#[derive(Clone, Copy, PartialEq)]
pub enum Slot {
    Empty,
    Used,
    Old, // a state of another build of this mod: not loadable
}

/// What the 16 slots hold.
pub fn scan(vm: &Vm, vol: RawVolume) -> [Slot; SLOTS] {
    let mut out = [Slot::Empty; SLOTS];
    let Some(d) = dir(vm, vol, false) else { return out };
    for (k, o) in out.iter_mut().enumerate() {
        let n = name(k);
        let Ok(f) = vm.open_file_in_dir(d, core::str::from_utf8(&n).unwrap(), Mode::ReadOnly) else { continue };
        let mut h = [0u8; HDR];
        let mut got = 0;
        while got < HDR {
            match vm.read(f, &mut h[got..]) {
                Ok(0) | Err(_) => break,
                Ok(m) => got += m,
            }
        }
        let len = vm.file_length(f).unwrap_or(0);
        let sm = if got == HDR { hdr_smem(&h) } else { None };
        *o = match sm {
            Some(_) if ours(&h) && len == HDR as u32 + body_len() => Slot::Used,
            _ => Slot::Old,
        };
        let _ = vm.close_file(f);
    }
    let _ = vm.close_dir(d);
    out
}

#[derive(Clone, Copy, PartialEq)]
pub enum Kind {
    Save,
    Check, // a load's first pass: the CRCs
    Load,
}

pub enum Res {
    Busy,
    Done,
    Failed(&'static str),
    /// The load failed half way: the mod's memory is part old, part new (restart it)
    Broken,
}

#[derive(Clone, Copy, PartialEq)]
enum Stage {
    PackOut,   // save: the samples into a new pack (only when the card hasn't got this set)
    RamOut,    // save: the state file
    RamCheck,  // load: the state file's CRC
    PackCheck, // load: the pack's CRC
    PackIn,    // load: the pack into the flash
    RamIn,     // load: the state file into the RAM
}

/// "XXXXXXXX.SMP": a pack's name, its CRC.
fn pack_name(crc: u32) -> [u8; 12] {
    let mut n = *b"00000000.SMP";
    for (i, c) in n[..8].iter_mut().enumerate() {
        let d = (crc >> (28 - 4 * i)) as u8 & 15;
        *c = if d < 10 { b'0' + d } else { b'A' + d - 10 };
    }
    n
}
fn open_in(vm: &Vm, vol: RawVolume, name: &[u8], mode: Mode) -> Result<RawFile, &'static str> {
    let d = dir(vm, vol, mode != Mode::ReadOnly).ok_or("CARD ERROR")?;
    let f = vm.open_file_in_dir(d, core::str::from_utf8(name).unwrap_or("X"), mode);
    let _ = vm.close_dir(d);
    f.map_err(|_| "CARD ERROR")
}

pub struct Job {
    pub kind: Kind,
    pub slot: usize,
    vol: RawVolume,
    stage: Stage,
    file: Option<RawFile>,
    part: usize,
    off: u32,
    done: u32,
    crc: u32,
    want: u32,      // the state file's body CRC
    sm: (u32, u32), // the save memory's part (smem) ..
    pcrc: u32,      // .. and its CRC: the pack's name
}

impl Job {
    pub fn percent(&self) -> u32 {
        let all = (body_len() + self.sm.1).max(1) as u64;
        let total = all * if self.kind == Kind::Save { 1 } else { 2 };
        let done = self.done as u64 + if self.kind == Kind::Load { all } else { 0 };
        (done * 100 / total).min(100) as u32
    }

    pub fn save(vm: &Vm, vol: RawVolume, slot: usize) -> Result<Job, &'static str> {
        let sm = smem();
        let pcrc = if !PACKS || sm.1 == 0 {
            0
        } else {
            !crc_add(!0, unsafe { core::slice::from_raw_parts((NOR_ADDR + sm.0) as *const u8, sm.1 as usize) })
        };
        let mut j = Job { kind: Kind::Save, slot, vol, stage: Stage::RamOut, file: None, part: 0, off: 0, done: 0, crc: !0, want: 0, sm, pcrc };
        if PACKS && sm.1 > 0 {
            let pn = pack_name(pcrc);
            let have = match open_in(vm, vol, &pn, Mode::ReadOnly) {
                Ok(f) => {
                    let ok = vm.file_length(f).unwrap_or(0) == sm.1;
                    let _ = vm.close_file(f);
                    ok
                }
                Err(_) => false,
            };
            if have {
                j.done = sm.1; // this set is on the card already
            } else {
                j.file = Some(open_in(vm, vol, &pn, Mode::ReadWriteCreateOrTruncate)?);
                j.stage = Stage::PackOut;
                return Ok(j);
            }
        }
        j.ram_out(vm)?;
        Ok(j)
    }

    /// Open the state file for writing (in place when one of this size is there: no finding the card's clusters
    /// again, the slow part), its header with no CRC yet: a save cut off reads as damaged on a load.
    fn ram_out(&mut self, vm: &Vm) -> Result<(), &'static str> {
        let d = dir(vm, self.vol, true).ok_or("CARD ERROR")?;
        let n = name(self.slot);
        let nm = core::str::from_utf8(&n).unwrap();
        let size = HDR as u32 + body_len();
        let f = match vm.open_file_in_dir(d, nm, Mode::ReadWriteAppend) {
            Ok(f) if vm.file_length(f).unwrap_or(0) == size && vm.file_seek_from_start(f, 0).is_ok() => Ok(f),
            Ok(f) => {
                let _ = vm.close_file(f);
                vm.open_file_in_dir(d, nm, Mode::ReadWriteCreateOrTruncate)
            }
            Err(_) => vm.open_file_in_dir(d, nm, Mode::ReadWriteCreateOrTruncate),
        };
        let _ = vm.close_dir(d);
        let f = f.map_err(|_| "CARD ERROR")?;
        if vm.write(f, &header(0, self.sm, self.pcrc)).is_err() {
            let _ = vm.close_file(f);
            return Err("CARD WRITE");
        }
        self.file = Some(f);
        self.stage = Stage::RamOut;
        self.part = 0;
        self.off = 0;
        self.crc = !0;
        Ok(())
    }

    pub fn load(vm: &Vm, vol: RawVolume, slot: usize) -> Result<Job, &'static str> {
        let n = name(slot);
        let f = open_in(vm, vol, &n, Mode::ReadOnly).map_err(|_| "EMPTY")?;
        let mut h = [0u8; HDR];
        let got = read_all(vm, f, &mut h);
        let sm = if got { hdr_smem(&h) } else { None };
        let Some(sm) = sm.filter(|_| ours(&h) && vm.file_length(f).unwrap_or(0) == HDR as u32 + body_len()) else {
            let _ = vm.close_file(f);
            return Err("OTHER VERSION");
        };
        Ok(Job { kind: Kind::Check, slot, vol, stage: Stage::RamCheck, file: Some(f), part: 0, off: 0, done: 0, crc: !0,
                 want: u32at(&h, 16), sm, pcrc: u32at(&h, 60) })
    }

    /// Stop now (the file closed). A load stopped in its Load pass leaves the mod broken.
    pub fn abort(&mut self, vm: &Vm) {
        if let Some(f) = self.file.take() {
            let _ = vm.close_file(f);
        }
    }
    fn fail(&mut self, vm: &Vm, why: &'static str) -> Res {
        self.abort(vm);
        if self.kind == Kind::Load { Res::Broken } else { Res::Failed(why) }
    }

    /// One piece: up to SAVE_CHUNK written or LOAD_CHUNK read (a pack: one sector).
    pub fn step(&mut self, vm: &Vm) -> Res {
        let Some(f) = self.file else { return Res::Failed("NO FILE") };
        match self.stage {
            Stage::PackOut | Stage::PackCheck | Stage::PackIn if PACKS => self.step_pack(vm, f),
            _ => self.step_ram(vm, f),
        }
    }

    fn step_ram(&mut self, vm: &Vm, f: RawFile) -> Res {
        let r = ranges();
        if self.part >= r.len() {
            return self.ram_end(vm, f);
        }
        let (a, n) = r[self.part];
        let k = (n - self.off).min(if self.stage == Stage::RamOut { SAVE_CHUNK } else { LOAD_CHUNK } as u32) as usize;
        if k == 0 {
            self.part += 1;
            self.off = 0;
            return Res::Busy;
        }
        let at = (a + self.off) as *mut u8;
        match self.stage {
            Stage::RamOut => {
                let s = unsafe { core::slice::from_raw_parts(at, k) };
                if vm.write(f, s).is_err() {
                    return self.fail(vm, "CARD WRITE");
                }
                self.crc = crc_add(self.crc, s);
            }
            Stage::RamCheck => {
                let mut buf = [0u8; 2048];
                let mut left = k;
                while left > 0 {
                    let m = left.min(buf.len());
                    if !read_all(vm, f, &mut buf[..m]) {
                        return self.fail(vm, "CARD READ");
                    }
                    self.crc = crc_add(self.crc, &buf[..m]);
                    left -= m;
                }
            }
            _ => {
                let d = unsafe { core::slice::from_raw_parts_mut(at, k) };
                if !read_all(vm, f, d) {
                    return self.fail(vm, "CARD READ");
                }
            }
        }
        self.off += k as u32;
        self.done += k as u32;
        Res::Busy
    }

    fn ram_end(&mut self, vm: &Vm, f: RawFile) -> Res {
        match self.stage {
            Stage::RamOut => {
                // the header again, now with the body's CRC
                let ok = vm.file_seek_from_start(f, 0).is_ok() && vm.write(f, &header(!self.crc, self.sm, self.pcrc)).is_ok();
                self.abort(vm);
                if ok { Res::Done } else { Res::Failed("CARD WRITE") }
            }
            Stage::RamCheck => {
                self.abort(vm);
                if !self.crc != self.want {
                    return Res::Failed("SAVE DAMAGED");
                }
                if !PACKS || self.sm.1 == 0 {
                    return self.ram_in(vm);
                }
                // the pack next: there, of the right size and CRC, before anything is changed
                match open_in(vm, self.vol, &pack_name(self.pcrc), Mode::ReadOnly) {
                    Ok(p) if vm.file_length(p).unwrap_or(0) == self.sm.1 => {
                        self.file = Some(p);
                        self.stage = Stage::PackCheck;
                        self.off = 0;
                        self.crc = !0;
                        Res::Busy
                    }
                    Ok(p) => {
                        let _ = vm.close_file(p);
                        Res::Failed("SAMPLES MISSING")
                    }
                    Err(_) => Res::Failed("SAMPLES MISSING"),
                }
            }
            _ => {
                self.abort(vm);
                Res::Done
            }
        }
    }

    /// The state file again, from its body, into the RAM (the load's last stage).
    fn ram_in(&mut self, vm: &Vm) -> Res {
        self.kind = Kind::Load;
        self.stage = Stage::RamIn;
        self.part = 0;
        self.off = 0;
        let n = name(self.slot);
        match open_in(vm, self.vol, &n, Mode::ReadOnly) {
            Ok(f) if vm.file_seek_from_start(f, HDR as u32).is_ok() => {
                self.file = Some(f);
                Res::Busy
            }
            Ok(f) => {
                let _ = vm.close_file(f);
                self.fail(vm, "CARD READ")
            }
            Err(_) => self.fail(vm, "CARD READ"),
        }
    }

    /// One sector of a pack: written from the flash, checked, or loaded into the flash (erased and written only when
    /// it differs).
    fn step_pack(&mut self, vm: &Vm, f: RawFile) -> Res {
        if self.off >= self.sm.1 {
            return match self.stage {
                Stage::PackOut => {
                    let ok = vm.flush_file(f).is_ok();
                    self.abort(vm);
                    if !ok {
                        return Res::Failed("CARD WRITE");
                    }
                    match self.ram_out(vm) {
                        Ok(()) => Res::Busy,
                        Err(e) => Res::Failed(e),
                    }
                }
                Stage::PackCheck => {
                    if !self.crc != self.pcrc || vm.file_seek_from_start(f, 0).is_err() {
                        self.abort(vm);
                        return Res::Failed("SAMPLES DAMAGED");
                    }
                    self.kind = Kind::Load; // from here on the mod is being changed
                    self.stage = Stage::PackIn;
                    self.off = 0;
                    self.done = 0;
                    Res::Busy
                }
                _ => {
                    self.abort(vm);
                    self.ram_in(vm)
                }
            };
        }
        let at = self.sm.0 + self.off;
        let flash = unsafe { core::slice::from_raw_parts((NOR_ADDR + at) as *const u8, SEC as usize) };
        let mut buf = [0u8; SEC as usize];
        match self.stage {
            Stage::PackOut => {
                buf.copy_from_slice(flash);
                if vm.write(f, &buf).is_err() {
                    return self.fail(vm, "CARD WRITE");
                }
            }
            Stage::PackCheck => {
                if !read_all(vm, f, &mut buf) {
                    return self.fail(vm, "CARD READ");
                }
                self.crc = crc_add(self.crc, &buf);
            }
            _ => {
                if !read_all(vm, f, &mut buf) {
                    return self.fail(vm, "CARD READ");
                }
                if buf[..] != *flash
                    && (crate::nor::teensy_nor_erase(at) != 0
                        || unsafe { crate::nor::teensy_nor_prog(at, buf.as_ptr(), SEC) } != 0)
                {
                    return self.fail(vm, "FLASH WRITE");
                }
            }
        }
        self.off += SEC;
        self.done += SEC;
        Res::Busy
    }
}

fn read_all(vm: &Vm, f: RawFile, buf: &mut [u8]) -> bool {
    let mut got = 0;
    while got < buf.len() {
        match vm.read(f, &mut buf[got..]) {
            Ok(0) | Err(_) => return false,
            Ok(m) => got += m,
        }
    }
    true
}

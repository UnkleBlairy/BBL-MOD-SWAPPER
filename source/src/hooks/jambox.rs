//! Sample hook for the BBL Jam Box (the Sample Lab, lab.rs, talks to it through hooks/mod.rs): its sample memory, its 32 chunks (S01..S32) as the Jam Box reads them,
//! and a new chunk written into one of them. The layout is the Jam Box's own (firmware/src/eng_sampler.c, and the web
//! uploader's web/smpimg.js, which this follows), in its save memory (nor.rs, flash in the ROM's slot):
//!   0x00000  the table (4 KiB): u32 magic "FSM2", u16 version 1, u16 count, u32 data length, u32 data CRC32, 12 zero
//!            bytes, u32 table CRC32 (bytes 0..27, then the entries), then count entries of 24 bytes: char name[12],
//!            u32 offset (in the data), u32 length (samples), u16 rate (Hz), u16 coding (0)
//!   0x01000  the data, to 0x97000: 8-bit mu-law (G.711), one byte a sample
//!
//! Writing a chunk (Put): the new sound goes after the data in use. When it doesn't fit there, the chunks still in use
//! slide down first, over the space of the one replaced (only ever down, so a sector at a time with one 4 KiB buffer is
//! safe). The table is written last, then the Jam Box reads it again (its smp_scan). Every step is one sector (~50 ms:
//! an erase); the main loop runs between them, so the FM-1 link stays alive.

use crate::nor;

unsafe extern "C" {
    fn web_smp_scan();
}

pub const BASE: u32 = 0x00000; // the Teensy builds' place (eng_sampler.c TEENSY_NOR_ADDR): no app in the NOR
pub const DATA: u32 = BASE + 0x1000;
pub const TOP: u32 = 0x97000; // to the projects (storage.c 0x97000)
pub const CAP: u32 = TOP - DATA; // 614400 samples: ~27.9 s at 22 kHz
// where the samples were before 0.9 on the Teensy (and still are on the FM-1): moved once by `Move`
const OLD_BASE: u32 = 0xA0000;
const OLD_CAP: u32 = 0xDA000 - OLD_BASE - 0x1000;
pub const MAX: usize = 32;
pub const RATE: u32 = 22050; // stored at most at this rate (the uploader's default)
const MAGIC: u32 = 0x324D_5346; // "FSM2"
const SEC: u32 = 4096;

fn nor(off: u32, n: u32) -> &'static [u8] {
    unsafe { core::slice::from_raw_parts((nor::NOR_ADDR + off) as *const u8, n as usize) }
}
fn u32le(b: &[u8], o: usize) -> u32 {
    u32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]])
}
fn u16le(b: &[u8], o: usize) -> u32 {
    u16::from_le_bytes([b[o], b[o + 1]]) as u32
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

/// A 16-bit sample as a mu-law byte (the uploader's ulawEnc).
pub fn ulaw(s: i32) -> u8 {
    let (sign, mut s) = if s < 0 { (0x80, -s) } else { (0, s) };
    s = s.min(32635) + 0x84;
    let mut e = 7;
    while e > 0 && s & (0x4000 >> (7 - e)) == 0 {
        e -= 1;
    }
    !(sign | (e << 4) | ((s >> (e + 3)) & 15)) as u8
}

/// The slot's name on the Lab's screen: S01..S32.
pub fn label(slot: usize) -> ([u8; 4], usize) {
    let mut l = *b"S00 ";
    l[1] += ((slot + 1) / 10) as u8;
    l[2] += ((slot + 1) % 10) as u8;
    (l, 3)
}

#[derive(Clone, Copy)]
pub struct Chunk {
    pub name: [u8; 12],
    pub off: u32,
    pub len: u32,
    pub rate: u16,
}
const NO_CHUNK: Chunk = Chunk { name: [0; 12], off: 0, len: 0, rate: 0 };

/// The table as the Jam Box checks it (gb_store.c smp_scan): anything wrong and there are no chunks.
#[derive(Clone, Copy)]
pub struct Table {
    pub n: usize,
    pub dlen: u32,
    pub c: [Chunk; MAX],
}
impl Table {
    pub const EMPTY: Table = Table { n: 0, dlen: 0, c: [NO_CHUNK; MAX] };

    pub fn read() -> Table {
        Table::read_at(BASE, CAP)
    }

    /// The table of a sample memory at `base` holding up to `cap` bytes of data (also the old place, OLD_BASE).
    fn read_at(base: u32, cap: u32) -> Table {
        let mut t = Table::EMPTY;
        let h = nor(base, SEC);
        let cnt = u16le(h, 6) as usize;
        let dlen = u32le(h, 8);
        if u32le(h, 0) != MAGIC || u16le(h, 4) != 1 || cnt > MAX || dlen > cap {
            return t;
        }
        if !crc_add(crc_add(!0, &h[..28]), &h[32..32 + cnt * 24]) != u32le(h, 28) {
            return t;
        }
        for i in 0..cnt {
            let e = &h[32 + i * 24..56 + i * 24];
            let (off, len, rate) = (u32le(e, 12), u32le(e, 16), u16le(e, 20));
            if len < 64 || off > dlen || len > dlen - off || !(4000..=48000).contains(&rate) || u16le(e, 22) != 0 {
                return Table::EMPTY;
            }
            let mut name = [0u8; 12];
            name.copy_from_slice(&e[..12]);
            t.c[i] = Chunk { name, off, len, rate: rate as u16 };
        }
        if !crc_add(!0, nor(base + SEC, dlen)) != u32le(h, 12) {
            return Table::EMPTY;
        }
        t.n = cnt;
        t.dlen = dlen;
        t
    }

    /// Slot `slot` holds a sound.
    pub fn full(&self, slot: usize) -> bool {
        slot < self.n
    }
    /// The last slot the Lab can pick: the chunks in use, and one more (a new one).
    pub fn top(&self) -> usize {
        self.n.min(MAX - 1)
    }

    /// Samples a sound sent to `slot` can have (the memory less every other chunk).
    pub fn room(&self, slot: usize) -> u32 {
        CAP - self.used_but(slot)
    }

    /// Samples held by every chunk but `skip`.
    fn used_but(&self, skip: usize) -> u32 {
        (0..self.n).filter(|&i| i != skip).map(|i| self.c[i].len).sum()
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Phase {
    Invalidate,      // the table erased (the chunks are about to move)
    Compact(u32),    // rewriting data sector n (chunks sliding down)
    Write(u32),      // the new chunk: its next sample (index)
    Table,           // the data's CRC, then the table
    Scan,            // the Jam Box reads the table again
    Done,
    Failed(&'static str),
}

/// Writing one chunk into slot `slot`: `plan`, then `step` until Done / Failed.
pub struct Put {
    pub slot: usize,
    pub n: u32,  // samples it will write (the sound may have been longer: cut to what fits)
    pub rate: u16,
    name: [u8; 12],
    old: Table,
    new: Table, // offsets after the move, the new chunk included
    dst: u32,   // where the new chunk starts in the data
    first: u32, // the first data sector the move rewrites (u32::MAX: no move)
    pub phase: Phase,
    steps: u32,
    total: u32,
    buf: [u8; SEC as usize],
}

impl Put {
    pub const fn idle() -> Put {
        Put {
            slot: 0,
            n: 0,
            rate: 0,
            name: [0; 12],
            old: Table::EMPTY,
            new: Table::EMPTY,
            dst: 0,
            first: u32::MAX,
            phase: Phase::Done,
            steps: 0,
            total: 1,
            buf: [0; SEC as usize],
        }
    }

    /// Plan a chunk of `want` samples at `rate` into `slot` (0.. the chunks there are: one past the end adds one).
    /// Nothing is written yet. `self.n` is what will be written (less than `want` when the memory is short).
    pub fn plan(&mut self, slot: usize, name: &[u8], rate: u32, want: u32) -> Result<(), &'static str> {
        let t = Table::read();
        if slot > t.n || slot >= MAX {
            return Err("NO SUCH SLOT");
        }
        if !(4000..=48000).contains(&rate) {
            return Err("RATE NOT USABLE");
        }
        let free = CAP - t.used_but(slot);
        let n = want.min(free);
        if n < 64 {
            return Err("SAMPLE MEMORY FULL");
        }
        let mut nm = [0u8; 12];
        for (d, &c) in nm.iter_mut().zip(name.iter()) {
            *d = if (b' '..127).contains(&c) { c } else { b'?' };
        }
        let mut new = t;
        let mut first = u32::MAX;
        let dst;
        if CAP - t.dlen >= n {
            dst = t.dlen; // room after the data in use
        } else {
            // slide the chunks kept down, in the order they lie, over the gaps (the replaced one's space included)
            let mut order = [0usize; MAX];
            let mut k = 0;
            for i in (0..t.n).filter(|&i| i != slot) {
                order[k] = i;
                k += 1;
            }
            order[..k].sort_unstable_by_key(|&i| t.c[i].off);
            let mut at = 0u32;
            for &i in &order[..k] {
                if t.c[i].off != at {
                    first = first.min(at / SEC);
                }
                new.c[i].off = at;
                at += t.c[i].len;
            }
            dst = at;
        }
        new.c[slot] = Chunk { name: nm, off: dst, len: n, rate: rate as u16 };
        if slot == t.n {
            new.n = t.n + 1;
        }
        new.dlen = dst + n;
        self.slot = slot;
        self.n = n;
        self.rate = rate as u16;
        self.name = nm;
        self.old = t;
        self.new = new;
        self.dst = dst;
        self.first = first;
        let moved = if first == u32::MAX { 0 } else { (dst + SEC - 1) / SEC - first };
        self.total = moved + 1 + (dst + n).div_ceil(SEC) - dst / SEC + 2;
        self.steps = 0;
        self.phase = if first == u32::MAX { Phase::Write(0) } else { Phase::Invalidate };
        Ok(())
    }

    pub fn percent(&self) -> u32 {
        (self.steps * 100 / self.total.max(1)).min(100)
    }

    /// Erase data sector `s` (from DATA) and write `self.buf` into it.
    fn burn(&mut self, s: u32) -> bool {
        let off = DATA + s * SEC;
        nor::teensy_nor_erase(off) == 0 && unsafe { nor::teensy_nor_prog(off, self.buf.as_ptr(), SEC) } == 0
    }

    /// One step (one sector at most). `pcm` gives the new chunk's samples (16-bit, at `self.rate`), as many as asked
    /// for, fewer at the sound's end (the rest is silence).
    pub fn step(&mut self, pcm: &mut dyn FnMut(&mut [i16]) -> usize) -> Phase {
        self.phase = match self.phase {
            Phase::Invalidate => {
                if nor::teensy_nor_erase(BASE) != 0 {
                    Phase::Failed("FLASH ERASE")
                } else {
                    Phase::Compact(self.first)
                }
            }
            Phase::Compact(s) => {
                let (a, b) = (s * SEC, s * SEC + SEC);
                if a >= self.dst {
                    Phase::Write(0)
                } else {
                    self.buf.copy_from_slice(nor(DATA + a, SEC));
                    for i in 0..self.old.n {
                        if i == self.slot {
                            continue;
                        }
                        let (no, oo, len) = (self.new.c[i].off, self.old.c[i].off, self.old.c[i].len);
                        let (x0, x1) = (no.max(a), (no + len).min(b));
                        if x0 < x1 {
                            let src = nor(DATA + oo + (x0 - no), x1 - x0);
                            self.buf[(x0 - a) as usize..(x1 - a) as usize].copy_from_slice(src);
                        }
                    }
                    if self.buf[..] == *nor(DATA + a, SEC) || self.burn(s) {
                        Phase::Compact(s + 1)
                    } else {
                        Phase::Failed("FLASH WRITE")
                    }
                }
            }
            Phase::Write(j) => {
                if j >= self.n {
                    Phase::Table
                } else {
                    let at = self.dst + j; // the next sample's place in the data
                    let s = at / SEC;
                    let a = s * SEC;
                    self.buf.copy_from_slice(nor(DATA + a, SEC));
                    let k = ((a + SEC - at).min(self.n - j)) as usize;
                    let o = (at - a) as usize;
                    let mut tmp = [0i16; 512];
                    let mut done = 0;
                    while done < k {
                        let want = (k - done).min(tmp.len());
                        let got = pcm(&mut tmp[..want]);
                        for x in &mut tmp[got..want] {
                            *x = 0;
                        }
                        for (d, &v) in self.buf[o + done..o + done + want].iter_mut().zip(tmp[..want].iter()) {
                            *d = ulaw(v as i32);
                        }
                        done += want;
                    }
                    if self.burn(s) { Phase::Write(j + k as u32) } else { Phase::Failed("FLASH WRITE") }
                }
            }
            Phase::Table => {
                let t = &self.new;
                let dcrc = !crc_add(!0, nor(DATA, t.dlen));
                self.buf.fill(0);
                for i in 0..t.n {
                    let e = &mut self.buf[32 + i * 24..56 + i * 24];
                    e[..12].copy_from_slice(&t.c[i].name);
                    e[12..16].copy_from_slice(&t.c[i].off.to_le_bytes());
                    e[16..20].copy_from_slice(&t.c[i].len.to_le_bytes());
                    e[20..22].copy_from_slice(&t.c[i].rate.to_le_bytes());
                }
                self.buf[0..4].copy_from_slice(&MAGIC.to_le_bytes());
                self.buf[4..6].copy_from_slice(&1u16.to_le_bytes());
                self.buf[6..8].copy_from_slice(&(t.n as u16).to_le_bytes());
                self.buf[8..12].copy_from_slice(&t.dlen.to_le_bytes());
                self.buf[12..16].copy_from_slice(&dcrc.to_le_bytes());
                let tcrc = !crc_add(crc_add(!0, &self.buf[..28]), &self.buf[32..32 + t.n * 24]);
                self.buf[28..32].copy_from_slice(&tcrc.to_le_bytes());
                if nor::teensy_nor_erase(BASE) == 0 && unsafe { nor::teensy_nor_prog(BASE, self.buf.as_ptr(), SEC) } == 0 {
                    Phase::Scan
                } else {
                    Phase::Failed("TABLE WRITE")
                }
            }
            Phase::Scan => {
                unsafe { web_smp_scan() };
                let t = Table::read();
                if t.n == self.new.n && t.c[self.slot].len == self.n && t.c[self.slot].off == self.dst {
                    Phase::Done
                } else {
                    Phase::Failed("CHECK FAILED")
                }
            }
            p => p,
        };
        self.steps += 1;
        self.phase
    }
}

/// The samples a card's DATA/JAMBOX.NOR kept at the old place (OLD_BASE, before the Teensy builds moved them to BASE)
/// copied over once: the data a sector a step, the table last (a cut copy just starts again), then the old table
/// erased so it never runs again, then the Jam Box reads its slots. Nothing to do when BASE already has a table.
pub struct Move {
    at: u32, // the next data sector's offset; past `end`: the table
    end: u32,
    done: bool,
}
impl Move {
    pub const fn new() -> Move {
        Move { at: 0, end: 0, done: false }
    }
    /// One sector (~50 ms) if there is one to move. True when it did something.
    pub fn step(&mut self) -> bool {
        if self.done {
            return false;
        }
        if self.end == 0 {
            let old = Table::read_at(OLD_BASE, OLD_CAP);
            if old.n == 0 || Table::read().n != 0 {
                self.done = true;
                return false;
            }
            self.end = (old.dlen + SEC - 1) / SEC * SEC;
        }
        let mut buf = [0u8; SEC as usize];
        let (src, dst) = if self.at < self.end { (OLD_BASE + SEC + self.at, DATA + self.at) } else { (OLD_BASE, BASE) };
        buf.copy_from_slice(nor(src, SEC));
        let ok = nor::teensy_nor_erase(dst) == 0 && unsafe { nor::teensy_nor_prog(dst, buf.as_ptr(), SEC) } == 0;
        if !ok || self.at >= self.end {
            if ok {
                nor::teensy_nor_erase(OLD_BASE);
                unsafe { web_smp_scan() };
            }
            self.done = true; // (a failed write: tried again at the next start)
        } else {
            self.at += SEC;
        }
        true
    }
}

/// The part of the save memory a save state carries (states.rs): the table and the data in use, whole sectors, as
/// (offset, length). With no good table, just the table sector (a state with no samples loads as none).
pub fn state_mem() -> (u32, u32) {
    let t = Table::read();
    let d = if t.n == 0 { 0 } else { (t.dlen + SEC - 1) / SEC * SEC };
    (BASE, SEC + d)
}

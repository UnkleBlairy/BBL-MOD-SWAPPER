//! Sample hook for SLOOP (the Sample Lab, lab.rs, talks to it through hooks/mod.rs): its four user sample slots,
//! USR1..USR4, as SLOOP reads them (firmware/src/eng_sample.c smp_user_scan), and a sound written into one of them.
//! The layout is SLOOP's own (web/EDITOR_PROTOCOL.md "User sample slot"; tools/sampleio.py user_slot, which this
//! follows), in its save memory (nor.rs, flash in the ROM's slot, mirrored to the card's DATA/SLOOP.NOR):
//!   USR1..USR3 at 0xA0000, 0xB4000, 0xC8000, USR4 at 0xE7000, 80 KiB each:
//!   0    u32 magic "FSMP", u16 version 1, u8 zone count 1..16, u8 0, char name[8], u32 data length (bytes),
//!        u32 CRC-32 of the data, 8 zero bytes, then 16 zones of 28 bytes: u32 off, n, loop start, loop end,
//!        rate (Hz / 44100 x 65536); i16 root x 16, ADPCM predictor at the loop start; u8 step index there, lo, hi, looped
//!   512  the data: IMA ADPCM, 4 bit, low nibble first, from predictor 0 and step index 0
//!
//! The Lab sends one sound as one zone over the whole keyboard (C4 plays it as recorded), one-shot: on SLOOP, a synth
//! track with engine SAMPLE and SET = USRn plays it (or GRAIN with SRC = USRn, or SLICE).
//!
//! Writing (Put): sector 0 (the header) is erased first and SLOOP reads the slot again, so the slot is empty while it
//! changes; then the data sectors, encoded as they go; sector 0 last, with the header; then SLOOP reads it again.
//! Every step is one sector (~50 ms: an erase); the main loop runs between them, so the FM-1 link stays alive.

use crate::nor;

unsafe extern "C" {
    fn teensy_smp_user_scan(k: u32);
}

pub const MAX: usize = 4;
pub const RATE: u32 = 22050; // as SLOOP's editor stores them
const SIZE: u32 = 0x14000;
const DOFF: u32 = 512;
const DMAX: u32 = SIZE - DOFF; // bytes of ADPCM a slot holds
const CAP: u32 = DMAX * 2; // 162816 samples: ~7.4 s at 22 kHz
const MAGIC: u32 = 0x504D_5346; // "FSMP"
const SEC: u32 = 4096;
const ROOT: i16 = 60; // C4

fn base(k: usize) -> u32 {
    if k < 3 { 0xA0000 + k as u32 * SIZE } else { 0xE7000 }
}
fn nor(off: u32, n: u32) -> &'static [u8] {
    unsafe { core::slice::from_raw_parts((nor::NOR_ADDR + off) as *const u8, n as usize) }
}
fn u32le(b: &[u8], o: usize) -> u32 {
    u32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]])
}

/// The slot's name on the Lab's screen.
pub fn label(slot: usize) -> ([u8; 4], usize) {
    let mut l = *b"USR1";
    l[3] += slot as u8;
    (l, 4)
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

const IMA_STEP: [i32; 89] = [
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31, 34, 37, 41, 45, 50, 55, 60, 66, 73, 80, 88, 97, 107,
    118, 130, 143, 157, 173, 190, 209, 230, 253, 279, 307, 337, 371, 408, 449, 494, 544, 598, 658, 724, 796, 876, 963,
    1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066, 2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358, 5894,
    6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899, 15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794,
    32767,
];
const IMA_IDX: [i32; 8] = [-1, -1, -1, -1, 2, 4, 6, 8];

/// The IMA encoder (sampleio.ima_encode): one sample in, its 4-bit code out.
struct Ima {
    pred: i32,
    idx: i32,
}
impl Ima {
    fn code(&mut self, x: i32) -> u8 {
        let step = IMA_STEP[self.idx as usize];
        let mut diff = x - self.pred;
        let mut code = 0u8;
        if diff < 0 {
            code = 8;
            diff = -diff;
        }
        let mut vd = step >> 3;
        if diff >= step {
            code |= 4;
            diff -= step;
            vd += step;
        }
        if diff >= step >> 1 {
            code |= 2;
            diff -= step >> 1;
            vd += step >> 1;
        }
        if diff >= step >> 2 {
            code |= 1;
            vd += step >> 2;
        }
        self.pred = (if code & 8 != 0 { self.pred - vd } else { self.pred + vd }).clamp(-32768, 32767);
        self.idx = (self.idx + IMA_IDX[(code & 7) as usize]).clamp(0, 88);
        code
    }
}

/// The four slots as SLOOP checks them (smp_user_scan's header test).
#[derive(Clone, Copy)]
pub struct Table {
    pub n: usize, // slots holding a sound
    used: [bool; MAX],
}
impl Table {
    pub const EMPTY: Table = Table { n: 0, used: [false; MAX] };

    pub fn read() -> Table {
        let mut t = Table::EMPTY;
        for k in 0..MAX {
            let h = nor(base(k), 32);
            let nz = h[6];
            if u32le(h, 0) == MAGIC && h[4] == 1 && h[5] == 0 && (1..=16).contains(&nz) && u32le(h, 16) <= DMAX {
                t.used[k] = true;
                t.n += 1;
            }
        }
        t
    }
    pub fn full(&self, slot: usize) -> bool {
        slot < MAX && self.used[slot]
    }
    /// Any of the four.
    pub fn top(&self) -> usize {
        MAX - 1
    }
    /// A slot is the same size whatever the others hold.
    pub fn room(&self, _slot: usize) -> u32 {
        CAP
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Phase {
    Invalidate, // the header sector erased: the slot is empty while it changes
    Write(u32), // slot sector s (1..): the data in it
    Table,      // sector 0: the header and the first data bytes
    Scan,       // SLOOP reads the slot again
    Done,
    Failed(&'static str),
}

/// Writing one sound into slot `slot`: `plan`, then `step` until Done / Failed.
pub struct Put {
    pub slot: usize,
    pub n: u32, // samples it will write (the sound may have been longer: cut to what fits)
    pub rate: u16,
    name: [u8; 8],
    dlen: u32, // ADPCM bytes
    ima: Ima,
    crc: u32,
    pub phase: Phase,
    steps: u32,
    total: u32,
    done: u32, // samples encoded
    sec0: [u8; SEC as usize],
    buf: [u8; SEC as usize],
}

impl Put {
    pub const fn idle() -> Put {
        Put {
            slot: 0,
            n: 0,
            rate: 0,
            name: [0; 8],
            dlen: 0,
            ima: Ima { pred: 0, idx: 0 },
            crc: !0,
            phase: Phase::Done,
            steps: 0,
            total: 1,
            done: 0,
            sec0: [0; SEC as usize],
            buf: [0; SEC as usize],
        }
    }

    /// Plan a sound of `want` samples at `rate` into `slot` (0..3). Nothing is written yet. `self.n` is what will be
    /// written (less than `want` when the slot is short).
    pub fn plan(&mut self, slot: usize, name: &[u8], rate: u32, want: u32) -> Result<(), &'static str> {
        if slot >= MAX {
            return Err("NO SUCH SLOT");
        }
        if !(4000..=48000).contains(&rate) {
            return Err("RATE NOT USABLE");
        }
        let n = want.min(CAP);
        if n < 64 {
            return Err("SOUND TOO SHORT");
        }
        let mut nm = [0u8; 8];
        let mut k = 0;
        for &c in name {
            if k == 8 {
                break;
            }
            let c = c.to_ascii_uppercase();
            if (b' '..127).contains(&c) {
                nm[k] = c;
                k += 1;
            }
        }
        self.slot = slot;
        self.n = n;
        self.rate = rate as u16;
        self.name = nm;
        self.dlen = n.div_ceil(2);
        self.ima = Ima { pred: 0, idx: 0 };
        self.crc = !0;
        self.done = 0;
        self.sec0.fill(0);
        self.steps = 0;
        self.total = 1 + (DOFF + self.dlen).div_ceil(SEC) + 1;
        self.phase = Phase::Invalidate;
        Ok(())
    }

    pub fn percent(&self) -> u32 {
        (self.steps * 100 / self.total.max(1)).min(100)
    }

    /// Encode the data bytes `d0..d1` into `out` (pcm gives the samples; past the sound's end, silence).
    fn encode(&mut self, out: &mut [u8], pcm: &mut dyn FnMut(&mut [i16]) -> usize) {
        let mut tmp = [0i16; 512];
        let mut o = 0;
        while o < out.len() {
            let bytes = (out.len() - o).min(tmp.len() / 2);
            let want = ((self.n - self.done.min(self.n)) as usize).min(bytes * 2);
            let got = if want > 0 { pcm(&mut tmp[..want]) } else { 0 };
            for x in &mut tmp[got..bytes * 2] {
                *x = 0;
            }
            self.done += (bytes * 2) as u32;
            for b in 0..bytes {
                let lo = self.ima.code(tmp[2 * b] as i32);
                let hi = self.ima.code(tmp[2 * b + 1] as i32);
                out[o + b] = lo | hi << 4;
            }
            o += bytes;
        }
    }

    /// One step (one sector at most). `pcm` gives the sound's samples (16-bit, at `self.rate`), as many as asked for,
    /// fewer at the sound's end (the rest is silence).
    pub fn step(&mut self, pcm: &mut dyn FnMut(&mut [i16]) -> usize) -> Phase {
        let b = base(self.slot);
        self.phase = match self.phase {
            Phase::Invalidate => {
                if nor::teensy_nor_erase(b) != 0 {
                    Phase::Failed("FLASH ERASE")
                } else {
                    unsafe { teensy_smp_user_scan(self.slot as u32) };
                    // sector 0 holds the data's first bytes after the header: encoded now, written last
                    let k = (SEC - DOFF).min(self.dlen) as usize;
                    let mut first = [0u8; (SEC - DOFF) as usize];
                    self.encode(&mut first[..k], pcm);
                    self.crc = crc_add(self.crc, &first[..k]);
                    self.sec0[DOFF as usize..DOFF as usize + k].copy_from_slice(&first[..k]);
                    Phase::Write(1)
                }
            }
            Phase::Write(s) => {
                let a = s * SEC - DOFF; // the data byte at the sector's start
                if a >= self.dlen {
                    Phase::Table
                } else {
                    let k = (self.dlen - a).min(SEC) as usize;
                    let mut buf = core::mem::replace(&mut self.buf, [0; SEC as usize]);
                    buf.fill(0xFF);
                    self.encode(&mut buf[..k], pcm);
                    self.crc = crc_add(self.crc, &buf[..k]);
                    let ok = nor::teensy_nor_erase(b + s * SEC) == 0
                        && unsafe { nor::teensy_nor_prog(b + s * SEC, buf.as_ptr(), SEC) } == 0;
                    self.buf = buf;
                    if ok { Phase::Write(s + 1) } else { Phase::Failed("FLASH WRITE") }
                }
            }
            Phase::Table => {
                let h = &mut self.sec0;
                h[0..4].copy_from_slice(&MAGIC.to_le_bytes());
                h[4..6].copy_from_slice(&1u16.to_le_bytes());
                h[6] = 1; // one zone
                h[7] = 0;
                h[8..16].copy_from_slice(&self.name);
                h[16..20].copy_from_slice(&self.dlen.to_le_bytes());
                h[20..24].copy_from_slice(&(!self.crc).to_le_bytes());
                let rate = ((self.rate as u64 * 65536 + 22050) / 44100) as u32;
                let z = &mut h[32..60];
                z[0..4].copy_from_slice(&0u32.to_le_bytes()); // off
                z[4..8].copy_from_slice(&self.n.to_le_bytes());
                z[8..12].copy_from_slice(&0u32.to_le_bytes()); // loop start
                z[12..16].copy_from_slice(&(self.n - 1).to_le_bytes()); // loop end
                z[16..20].copy_from_slice(&rate.to_le_bytes());
                z[20..22].copy_from_slice(&(ROOT * 16).to_le_bytes());
                z[22..24].copy_from_slice(&0i16.to_le_bytes()); // predictor at the loop start (= the start)
                z[24] = 0; // step index there
                z[25] = 0; // lo
                z[26] = 127; // hi
                z[27] = 0; // one-shot
                if self.dlen < SEC - DOFF {
                    let e = (DOFF + self.dlen) as usize;
                    h[e..].fill(0xFF);
                }
                if unsafe { nor::teensy_nor_prog(b, self.sec0.as_ptr(), SEC) } == 0 {
                    Phase::Scan
                } else {
                    Phase::Failed("HEADER WRITE")
                }
            }
            Phase::Scan => {
                unsafe { teensy_smp_user_scan(self.slot as u32) };
                let h = nor(b, 24);
                if Table::read().full(self.slot) && u32le(h, 16) == self.dlen && u32le(h, 20) == !self.crc {
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

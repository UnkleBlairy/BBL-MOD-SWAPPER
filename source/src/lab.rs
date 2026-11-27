//! The Sample Lab: one sample browser for every mod that has sample slots (a hook in hooks/), part of the Mod Swapper
//! runtime and drawn in its style (the menu's MB7 block font, hot pink). Step 2 of docs/STATUS.md "Sample Lab":
//! browse the card's SAMPLES/, audition, send to a slot.
//!
//! FX + GLO opens it (main.rs holds those presses back briefly, Gate, so the mod never sees the combo). The mod
//! pauses while it is open (no audio rendered, no input), so the Lab has the sound; MASTER sets its level. Controls:
//!   SELECT knob   the file (faster the faster it turns)       PLAY   hear it / open a folder
//!   ALGORITHM     the slot to send to                         keys   hear it at that pitch (C = as recorded)
//!   REC           the trim page of the file                   HOME   back: out of a folder, then to the mod
//!   OCT- / OCT+   the keys' octave (-2 .. +2)
//! The trim page (step 3; the user: no manual, you look at the screen and get it): REC on a file always lands here,
//! REC again sends (twice to replace a slot in use), then back to the list. The four knobs under the screen are named
//! above them: START
//! (K1, the left bracket), ZOOM (K2: x6 a detent round the bracket last turned, or the view's middle when that is out of
//! view; down to one frame a column), POS (K3: the zoomed view left / right), END (K4, the right bracket). START / END
//! move a column of the view a detent (faster the faster they turn). The slot box alternates ALGO / the slot
//! every 2 s (held 3 s on the slot after ALGORITHM turns). HOME (a back arrow) is always shown. The length is the title, pink when the slot can't hold it all: a pink line marked FULL where
//! the memory runs out, the wave past it grey. HOME back to the list. The trim stays with that file until another is
//! trimmed or the folder changes.
//! The page (the user's rules: LESS IS MORE, BIGGER IS BETTER): the title and the slot, four names in the menu's
//! chunky letters (a long one shows its start and end; the selected one scrolls), and the buttons with symbols.
//!
//! The screen has no frame buffer (some mods have ~1 KB of RAM to spare): the page is a short list of rectangles and
//! texts, each row is drawn from it when the link wants rows, and a row goes out only when its hash changed.
//! Audition streams from the card through a 2048-frame ring. WAV (8/16/24/32-bit, float) and AIFF, any rate, mono
//! mix of the channels.

use core::ops::ControlFlow;
use embedded_sdmmc::{LfnBuffer, Mode, RawDirectory, RawFile, RawVolume, ShortFileName, VolumeIdx, VolumeManager};

use crate::audio_in;
use crate::hooks::active as hook;
use crate::sd;
use crate::states;

pub type Vm = VolumeManager<sd::Sd, sd::Clock, 4, 3, 1>;

// ---- the panel: buttons by printed label (panel.c's enum), the matrix ids of PANEL_DEFAULT, encoders by role ----
pub const FX: u32 = 1 << 0;
pub const GLO: u32 = 1 << 5;
pub const HOME: u32 = 1 << 6;
pub const PLAY: u32 = 1 << 10;
pub const REC: u32 = 1 << 11;
pub const OCTDN: u32 = 1 << 12;
pub const OCTUP: u32 = 1 << 13;
pub const SAVE: u32 = 1 << 7;
pub const EDIT: u32 = 1 << 4; // deletes a take (SAMPLES/RECORDED only), twice
pub const COMBO: u32 = FX | GLO; // the Sample Lab (mods with a sample hook)
pub const SAVES: u32 = FX | SAVE; // save states (every mod)
pub const MENU: u32 = FX | HOME; // back to the firmware menu, held MENU_HOLD_MS (every mod)
pub const MENU_HOLD_MS: u32 = 1000;
/// The combos this mod has: the Lab only with a sample hook.
const fn combos() -> u32 {
    if hook::MAX > 0 { COMBO | SAVES | MENU } else { SAVES | MENU }
}
const BTN_MATRIX: [u8; 14] = [2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 0, 1];
const ROLE_OF_HW: [u8; 7] = [0, 1, 3, 4, 5, 6, 2]; // SELECT ALGORITHM PRESETS K1..K4 = 0 1 2 3..6
const SELECT: u32 = 0;
const ALGO: u32 = 1;
const PRESETS: u32 = 2;
const K1: u32 = 3; // START: the left knob, the left bracket
const K2: u32 = 4; // ZOOM (K2 and K3, the middle knobs)
const K3: u32 = 5;
const K4: u32 = 6; // END: the right knob, the right bracket

pub fn to_matrix(labels: u32) -> u32 {
    (0..14).filter(|b| labels >> b & 1 != 0).fold(0, |m, b| m | 1 << BTN_MATRIX[b])
}
fn to_labels(matrix: u32) -> u32 {
    (0..14).filter(|&b| matrix >> BTN_MATRIX[b] & 1 != 0).fold(0, |m, b| m | 1 << b)
}

/// Between the FM-1's input and the mod: presses of the combo's buttons wait up to HOLD_MS (dropped if the combo
/// completes), and what is held when the Lab opens or closes stays away from the mod until it is let go.
pub struct Gate {
    held: u32, // combo buttons pressed, not yet passed on (matrix bits)
    t0: u32,   // when the first of them went down (ms)
    mute_b: u32,
    mute_n: u32,
    last: (u32, u32), // what the mod was last given
    raw: (u32, u32),  // the FM-1's last report
}
const HOLD_MS: u32 = 150;
const HOLD_SHOW: u32 = 3000; // a knob's value / name held this long after it turns, before the alternation goes on
impl Gate {
    pub const fn new() -> Gate {
        Gate { held: 0, t0: 0, mute_b: 0, mute_n: 0, last: (u32::MAX, u32::MAX), raw: (0, 0) }
    }
    fn emit(&mut self, n: u32, b: u32, out: &mut dyn FnMut(u32, u32)) {
        if (n, b) != self.last {
            self.last = (n, b);
            out(n, b);
        }
    }
    fn state(&self, n: u32, b: u32) -> (u32, u32) {
        (n & !self.mute_n, b & !self.held & !self.mute_b)
    }
    /// A report (notes, matrix buttons). `out` gets what the mod should see, in order. Returns the combo just
    /// completed: 0 none, 1 the Sample Lab (FX + GLO), 2 save states (FX + SAVE), 3 FX + HOME down (the menu, once held).
    #[inline(never)]
    pub fn input(&mut self, notes: u32, buttons: u32, ms: u32, out: &mut dyn FnMut(u32, u32)) -> u8 {
        self.raw = (notes, buttons);
        self.mute_b &= buttons;
        self.mute_n &= notes;
        let c = to_matrix(combos());
        for (k, cb) in [(1u8, COMBO), (2u8, SAVES), (3u8, MENU)] {
            let m = to_matrix(cb);
            if combos() & cb == cb && buttons & m == m {
                self.held = 0;
                self.mute_b = buttons;
                self.mute_n = notes;
                return k;
            }
        }
        let new = buttons & c & !self.held & !self.last.1 & !self.mute_b;
        if new != 0 {
            if self.held == 0 {
                self.t0 = ms;
            }
            self.held |= new;
        }
        let tapped = self.held & !buttons; // pressed and let go inside the wait: a whole press for the mod
        if tapped != 0 {
            self.held &= !tapped;
            let (n, b) = self.state(notes, buttons | tapped);
            self.emit(n, b, out);
        }
        let (n, b) = self.state(notes, buttons);
        self.emit(n, b, out);
        0
    }
    /// Time passing: presses that waited long enough go to the mod.
    #[inline(never)]
    pub fn tick(&mut self, ms: u32, out: &mut dyn FnMut(u32, u32)) {
        if self.held != 0 && ms.wrapping_sub(self.t0) >= HOLD_MS {
            self.held = 0;
            let (n, b) = self.state(self.raw.0, self.raw.1);
            self.emit(n, b, out);
        }
    }
    /// The FM-1's keys and buttons (matrix) at its last report.
    #[inline(never)]
    pub fn raw(&self) -> (u32, u32) {
        self.raw
    }
    /// The Lab closed: what is held now is the Lab's, the mod gets it only after a release.
    #[inline(never)]
    pub fn closed(&mut self, out: &mut dyn FnMut(u32, u32)) {
        self.mute_b = self.raw.1;
        self.mute_n = self.raw.0;
        let (n, b) = self.state(self.raw.0, self.raw.1);
        self.emit(n, b, out);
    }
}

// ---- the look: the menu's palette and its MB7 block font (ports/felucca's felucca_teensy.c teensy_menu) ----
const PAL: [u16; 6] = [0x0841, 0xFFDF, 0x8C71, 0x2945, 0xF8B2, 0x47E8];
const BG: u8 = 0;
const INK: u8 = 1;
const DIM: u8 = 2;
const BOX: u8 = 3;
const PINK: u8 = 4;
const GREEN: u8 = 5;

fn glyph(c: u8) -> (u8, [u8; 7]) {
    let c = c.to_ascii_uppercase();
    match c {
        b' ' => (4, [0, 0, 0, 0, 0, 0, 0]),
        b'-' => (6, [0, 0, 0, 0x3F, 0, 0, 0]),
        b'.' => (2, [0, 0, 0, 0, 0, 3, 3]),
        b'0' => (6, [0x1E, 0x33, 0x37, 0x3F, 0x3B, 0x33, 0x1E]),
        b'1' => (6, [0x0C, 0x1C, 0x0C, 0x0C, 0x0C, 0x0C, 0x1E]),
        b'2' => (6, [0x1E, 0x33, 0x03, 0x06, 0x0C, 0x18, 0x3F]),
        b'3' => (6, [0x3E, 0x03, 0x03, 0x1E, 0x03, 0x03, 0x3E]),
        b'4' => (6, [0x33, 0x33, 0x33, 0x3F, 0x03, 0x03, 0x03]),
        b'5' => (6, [0x3F, 0x30, 0x30, 0x3E, 0x03, 0x03, 0x3E]),
        b'6' => (6, [0x1E, 0x30, 0x30, 0x3E, 0x33, 0x33, 0x1E]),
        b'7' => (6, [0x3F, 0x03, 0x06, 0x0C, 0x0C, 0x0C, 0x0C]),
        b'8' => (6, [0x1E, 0x33, 0x33, 0x1E, 0x33, 0x33, 0x1E]),
        b'9' => (6, [0x1E, 0x33, 0x33, 0x1F, 0x03, 0x03, 0x1E]),
        b'A' => (6, [0x1E, 0x33, 0x33, 0x3F, 0x33, 0x33, 0x33]),
        b'B' => (6, [0x3E, 0x33, 0x33, 0x3E, 0x33, 0x33, 0x3E]),
        b'C' => (6, [0x1F, 0x30, 0x30, 0x30, 0x30, 0x30, 0x1F]),
        b'D' => (6, [0x3E, 0x33, 0x33, 0x33, 0x33, 0x33, 0x3E]),
        b'E' => (6, [0x3F, 0x30, 0x30, 0x3E, 0x30, 0x30, 0x3F]),
        b'F' => (6, [0x3F, 0x30, 0x30, 0x3E, 0x30, 0x30, 0x30]),
        b'G' => (6, [0x1F, 0x30, 0x30, 0x3F, 0x33, 0x33, 0x1F]),
        b'H' => (6, [0x33, 0x33, 0x33, 0x3F, 0x33, 0x33, 0x33]),
        b'I' => (6, [0x3F, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x3F]),
        b'J' => (6, [0x0F, 0x06, 0x06, 0x06, 0x06, 0x36, 0x1C]),
        b'K' => (6, [0x33, 0x36, 0x3C, 0x38, 0x3C, 0x36, 0x33]),
        b'L' => (6, [0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x3F]),
        b'M' => (7, [0x63, 0x77, 0x7F, 0x6B, 0x63, 0x63, 0x63]),
        b'N' => (6, [0x33, 0x3B, 0x3F, 0x3F, 0x3F, 0x37, 0x33]),
        b'O' => (6, [0x1E, 0x33, 0x33, 0x33, 0x33, 0x33, 0x1E]),
        b'P' => (6, [0x3E, 0x33, 0x33, 0x3E, 0x30, 0x30, 0x30]),
        b'Q' => (6, [0x1E, 0x33, 0x33, 0x33, 0x3F, 0x36, 0x1F]),
        b'R' => (6, [0x3E, 0x33, 0x33, 0x3E, 0x3C, 0x36, 0x33]),
        b'S' => (6, [0x1F, 0x30, 0x30, 0x1E, 0x03, 0x03, 0x3E]),
        b'T' => (6, [0x3F, 0x3F, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C]),
        b'U' => (6, [0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x1E]),
        b'V' => (6, [0x33, 0x33, 0x33, 0x33, 0x33, 0x1E, 0x0C]),
        b'W' => (7, [0x63, 0x63, 0x63, 0x6B, 0x7F, 0x77, 0x63]),
        b'X' => (6, [0x33, 0x33, 0x1E, 0x0C, 0x1E, 0x33, 0x33]),
        b'Y' => (6, [0x33, 0x33, 0x1E, 0x0C, 0x0C, 0x0C, 0x0C]),
        b'Z' => (6, [0x3F, 0x03, 0x06, 0x0C, 0x18, 0x30, 0x3F]),
        1 => (6, [0x20, 0x38, 0x3E, 0x3F, 0x3E, 0x38, 0x20]), // play
        2 => (7, [0x1C, 0x1C, 0x7F, 0x3E, 0x1C, 0x08, 0x7F]), // download
        3 => (7, [0x08, 0x18, 0x3F, 0x7F, 0x3F, 0x18, 0x08]), // back
        b'%' => (6, [0x31, 0x32, 0x04, 0x08, 0x10, 0x13, 0x23]),
        b'_' => (6, [0, 0, 0, 0, 0, 0, 0x3F]),
        b'/' => (6, [0x03, 0x03, 0x06, 0x0C, 0x18, 0x30, 0x30]),
        b'(' => (4, [0x3, 0x6, 0xC, 0xC, 0xC, 0x6, 0x3]),
        b')' => (4, [0xC, 0x6, 0x3, 0x3, 0x3, 0x6, 0xC]),
        b'[' => (4, [0xF, 0xC, 0xC, 0xC, 0xC, 0xC, 0xF]),
        b']' => (4, [0xF, 0x3, 0x3, 0x3, 0x3, 0x3, 0xF]),
        b':' => (2, [0, 3, 3, 0, 3, 3, 0]),
        b'+' => (6, [0, 0x0C, 0x0C, 0x3F, 0x0C, 0x0C, 0]),
        b'=' => (6, [0, 0, 0x3F, 0, 0x3F, 0, 0]),
        b'\'' => (2, [3, 3, 2, 0, 0, 0, 0]),
        b',' => (2, [0, 0, 0, 0, 0, 3, 2]),
        b'!' => (2, [3, 3, 3, 3, 3, 0, 3]),
        b'#' => (7, [0x36, 0x36, 0x7F, 0x36, 0x7F, 0x36, 0x36]),
        b'&' => (7, [0x1C, 0x36, 0x36, 0x1C, 0x3B, 0x36, 0x1F]),
        b'>' => (5, [0x18, 0x0C, 0x06, 0x03, 0x06, 0x0C, 0x18]),
        5 => (7, [0x1C, 0x22, 0x49, 0x49, 0x41, 0x22, 0x1C]), // a dial: its ring and pointer
        6 => (7, [0x1C, 0x3E, 0x7F, 0x7F, 0x7F, 0x3E, 0x1C]), // record
        7 => (7, [0, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0]), // stop
        b'<' => (5, [0x03, 0x06, 0x0C, 0x18, 0x0C, 0x06, 0x03]),
        _ => (6, [0x1E, 0x33, 0x03, 0x06, 0x0C, 0x00, 0x0C]), // '?'
    }
}
fn text_w(s: &[u8], sx: u32) -> u32 {
    s.iter().map(|&c| glyph(c).0 as u32 * sx + sx).sum::<u32>().saturating_sub(sx)
}

#[derive(Clone, Copy)]
enum Op {
    Rect { x: u8, y: u8, w: u8, h: u8, c: u8 },
    Text { x: i16, y: u8, sx: u8, sy: u8, c: u8, cl: u8, cr: u8, s: u16, n: u8 }, // drawn only in cl..cr
    Wave { y: u8, h: u8, a: i16, b: i16 }, // the trim page's waveform (columns a..=b pink)
}

// ---- the trim page's waveform: GW columns of the loudest sample in each (0..255), as the Jam Box's smp_peaks ----
const GW: usize = 224;
const WX: u32 = 8;
const WY: u32 = 52;
const WH: u32 = 118;
#[derive(Clone, Copy, PartialEq)]
enum Page {
    List,
    Trim,
    Saves,
    Rec,  // the recorder (the RECORD row at the top of SAMPLES)
    Name, // a new recording's name
}
/// The whole file (0) or the zoomed view (1): frames x0 .. x0 + w over the columns, frames at .. end still to read
/// (from the end down when `back`: the view moved left, the new part is on the left).
#[derive(Clone, Copy)]
struct Wave {
    pk: [u8; GW],
    x0: u32,
    w: u32,
    at: u32,
    end: u32,
    back: bool,
}
impl Wave {
    const NEW: Wave = Wave { pk: [0; GW], x0: 0, w: 1, at: 0, end: 0, back: false };
    fn view(x0: u32, w: u32) -> Wave {
        Wave { pk: [0; GW], x0, w: w.max(1), at: x0, end: x0 + w, back: false }
    }
    /// The column of frame f (outside 0..GW when it is out of view).
    fn col(&self, f: u32) -> i32 {
        ((f as i64 - self.x0 as i64) * GW as i64).div_euclid(self.w as i64) as i32
    }
    fn done(&self) -> bool {
        self.at >= self.end
    }
}

// ---- the card: SAMPLES/ and folders under it ----
const WIN: usize = 12; // entries kept around the selection
const ROWS: usize = 4; // shown
const NAME: usize = 48;
#[derive(Clone, Copy)]
struct Ent {
    sfn: ShortFileName,
    dir: bool,
    name: [u8; NAME], // as shown: upper case, no extension
    len: u8,
}
const NO_ENT: Ent = Ent { sfn: unsafe { core::mem::zeroed() }, dir: false, name: [0; NAME], len: 0 };
/// The first row at the top of SAMPLES: the recorder (shown as a folder, so the keys and REC leave it alone).
const REC_ENT: Ent = {
    let t = b"\x06 RECORD";
    let mut name = [0u8; NAME];
    let mut i = 0;
    while i < t.len() {
        name[i] = t[i];
        i += 1;
    }
    Ent { sfn: unsafe { core::mem::zeroed() }, dir: true, name, len: t.len() as u8 }
};
// the recorder: where the takes go, their names, the longest take, the knob ranges
/// The recorder is built in (the Jam Box; the others have no room or no slots).
const RECORDER: bool = cfg!(fw_rec);
const REC_DIR: &str = "RECORDED";
const REC_MAX: u32 = 10 * 60 * 44_100;
const NAME_CH: &[u8] = b" ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_";
const NAME_N: usize = 8; // the card's short names: 8 letters
const TRIG_N: u8 = 17; // the sound that starts a take: -54 dB .. -6 dB, 3 dB a step
#[derive(Clone, Copy, PartialEq)]
enum Rst {
    Idle,
    Armed, // waiting for sound
    On,
    Saving, // the take (in PSRAM) going to the card
}

/// A sound file's format, from its header.
#[derive(Clone, Copy, Default)]
struct Fmt {
    ok: bool,
    ch: u16,
    bits: u16,
    float: bool,
    be: bool,     // big endian (AIFF)
    signed8: bool, // 8-bit samples are signed (AIFF)
    rate: u32,
    off: u32,    // the first sample's byte
    frames: u32,
}
impl Fmt {
    fn fb(&self) -> u32 {
        self.ch as u32 * self.bits as u32 / 8
    }
    /// One frame as a mono 16-bit sample.
    fn mono(&self, f: &[u8]) -> i32 {
        let bs = (self.bits / 8) as usize;
        let mut acc = 0i32;
        for c in 0..self.ch as usize {
            let s = &f[c * bs..c * bs + bs];
            let v = match (bs, self.float) {
                (4, true) => {
                    let u = if self.be { u32::from_be_bytes([s[0], s[1], s[2], s[3]]) } else { u32::from_le_bytes([s[0], s[1], s[2], s[3]]) };
                    (f32::from_bits(u) * 32767.0).clamp(-32768.0, 32767.0) as i32
                }
                (1, _) => if self.signed8 { (s[0] as i8 as i32) << 8 } else { (s[0] as i32 - 128) << 8 },
                _ => {
                    let (hi, lo) = if self.be { (s[0], s[1]) } else { (s[bs - 1], s[bs - 2]) };
                    i16::from_be_bytes([hi, lo]) as i32
                }
            };
            acc += v;
        }
        acc / self.ch.max(1) as i32
    }
}

/// Read a WAV or AIFF header.
fn parse(vm: &Vm, f: RawFile) -> Fmt {
    let mut fm = Fmt::default();
    let size = vm.file_length(f).unwrap_or(0);
    let mut h = [0u8; 12];
    if vm.file_seek_from_start(f, 0).is_err() || vm.read(f, &mut h).unwrap_or(0) < 12 {
        return fm;
    }
    let riff = &h[0..4] == b"RIFF" && &h[8..12] == b"WAVE";
    let aiff = &h[0..4] == b"FORM" && (&h[8..12] == b"AIFF" || &h[8..12] == b"AIFC");
    if !riff && !aiff {
        return fm;
    }
    fm.be = aiff;
    let (mut at, mut have_fmt, mut data) = (12u32, false, None);
    for _ in 0..64 {
        let mut c = [0u8; 8];
        if at + 8 > size || vm.file_seek_from_start(f, at).is_err() || vm.read(f, &mut c).unwrap_or(0) < 8 {
            break;
        }
        let len = if aiff { u32::from_be_bytes([c[4], c[5], c[6], c[7]]) } else { u32::from_le_bytes([c[4], c[5], c[6], c[7]]) };
        let mut b = [0u8; 40];
        let k = (len as usize).min(40);
        let _ = vm.read(f, &mut b[..k]);
        match &c[0..4] {
            b"fmt " if riff && k >= 16 => {
                let mut tag = u16::from_le_bytes([b[0], b[1]]);
                if tag == 0xFFFE && k >= 26 {
                    tag = u16::from_le_bytes([b[24], b[25]]); // extensible: the sub-format's first word
                }
                fm.ch = u16::from_le_bytes([b[2], b[3]]);
                fm.rate = u32::from_le_bytes([b[4], b[5], b[6], b[7]]);
                fm.bits = u16::from_le_bytes([b[14], b[15]]);
                fm.float = tag == 3;
                have_fmt = (tag == 1 || tag == 3) && (fm.bits + 7) / 8 * 8 == fm.bits;
            }
            b"COMM" if aiff && k >= 18 => {
                fm.ch = u16::from_be_bytes([b[0], b[1]]);
                fm.bits = (u16::from_be_bytes([b[6], b[7]]) + 7) / 8 * 8;
                let e = (u16::from_be_bytes([b[8], b[9]]) & 0x7FFF) as i32;
                let m = u64::from_be_bytes([b[10], b[11], b[12], b[13], b[14], b[15], b[16], b[17]]);
                let sh = 16383 + 63 - e;
                fm.rate = if (0..64).contains(&sh) { (m >> sh) as u32 } else { 0 };
                fm.signed8 = true;
                have_fmt = true;
                if &h[8..12] == b"AIFC" && k >= 22 {
                    match &b[18..22] {
                        b"NONE" | b"twos" => {}
                        b"sowt" => fm.be = false,
                        b"fl32" | b"FL32" => fm.float = true,
                        _ => have_fmt = false,
                    }
                }
            }
            b"data" if riff => data = Some((at + 8, len)),
            b"SSND" if aiff && k >= 4 => data = Some((at + 16 + u32::from_be_bytes([b[0], b[1], b[2], b[3]]), len.saturating_sub(8))),
            _ => {}
        }
        if have_fmt && data.is_some() {
            break;
        }
        at = at.saturating_add(8 + len + (len & 1 & riff as u32));
    }
    if let Some((off, len)) = data {
        let fb = fm.ch as u32 * fm.bits as u32 / 8;
        let ok = have_fmt && (1..=8).contains(&fm.ch) && (8..=32).contains(&fm.bits) && (!fm.float || fm.bits == 32)
            && (1000..=192_000).contains(&fm.rate) && fb > 0;
        if ok {
            fm.off = off;
            fm.frames = len.min(size.saturating_sub(off)) / fb;
            fm.ok = fm.frames > 0;
        }
    }
    fm
}

// ---- audition: the file streamed into a ring of mono samples, played at the key's pitch ----
// (~190 ms at 44.1 kHz: a slow card read or a long page doesn't break the sound; a mod without samples: no audition)
const RING: usize = if hook::MAX > 0 { 8192 } else { 64 };
const FS: u64 = 44_100;
const SEMI: [f32; 27] = [
    0.667420, 0.707107, 0.749154, 0.793701, 0.840896, 0.890899, 0.943874, 1.000000, 1.059463, 1.122462, 1.189207,
    1.259921, 1.334840, 1.414214, 1.498307, 1.587401, 1.681793, 1.781797, 1.887749, 2.000000, 2.118926, 2.244924,
    2.378414, 2.519842, 2.669680, 2.828427, 2.996614,
];
const KEY_C: usize = 7; // the keys run F3 .. G5: key 7 is middle C, the sound as recorded

struct Play {
    file: Option<RawFile>,
    on: bool,     // sounding (render reads it)
    fm: Fmt,
    next: u32,    // the next frame to read from the file
    w: u32,       // frames in the ring so far (absolute)
    pos: u64,     // the read head, frames Q32
    step: u64,
    env: u32,     // fade in (0..64)
    stop: u32,    // fading out: samples left (0: not stopping)
    from: u32,    // the first frame (the trim's START)
    end: u32,     // the frame it stops at
}

// ---- sending: the file, converted to the hook's rate (a box filter, as the Jam Box uploader) and normalised ----
#[derive(PartialEq, Clone, Copy)]
enum Send {
    No,
    Peak,
    Write,
}
struct Conv {
    file: Option<RawFile>,
    fm: Fmt,
    next: u32,  // next source frame
    start: u32, // the first source frame (the trim's START)
    end: u32,   // the source frame it stops at
    want: u32,  // samples the trimmed sound has at the slot's rate (more than put.n: cut)
    own: bool,  // the file is the conversion's (closed at the end), not the trim page's
    ratio: u64, // source frames per output sample, Q32
    j: u64,     // output samples made
    bound: u32, // the source frame where output j ends
    acc: i64,
    cnt: u32,
    peak: i32,
    gain: f32,
}

pub struct Lab {
    pub open: bool,
    // the folder
    dir: Option<RawDirectory>,
    path: [ShortFileName; 4],
    depth: usize,
    n: usize,
    sel: usize,
    win0: usize,
    winn: usize,
    win: [Ent; WIN],
    fm: Fmt,
    fm_of: usize,
    moved_at: u32,
    // sound
    play: Play,
    ring: [i16; RING],
    pub out_l: [f32; 128],
    pub out_r: [f32; 128],
    // slots
    tab: hook::Table,
    slot: usize,
    arm: bool,
    darm: bool, // EDIT once on a take: EDIT again deletes it
    send: Send,
    conv: Conv,
    put: hook::Put,
    // input
    btn: u32,
    keys: u32,
    sel_t: u32,
    sel_dir: i32,
    // the screen
    msg: [u8; 40],
    msg_c: u8,
    msg_at: u32,
    ops: [Op; 48],
    nops: usize,
    pool: [u8; 512],
    npool: usize,
    sent: [u32; 240],
    pending: bool,
    row: usize,
    dirty: bool,
    drawn: u32,
    pct: u32,
    now: u32,
    master: f32, // the MASTER pot, as a gain
    oct: i32,    // the keys' octave (OCT- / OCT+)
    sel_at: u32, // when the selection last moved (the name's scroll)
    // the trim page
    page: Page,
    tf: Option<RawFile>, // the file, open while the page is
    t_of: usize,         // the entry the trim belongs to (usize::MAX: none)
    ta: u32,             // START (frame)
    tb: u32,             // END (frame, not included)
    hot: u8,             // the knob last turned: 0 START, 1 END (the zoom is round it)
    zoom: u32,           // 0: the whole file, k: 1/6^k of it
    wv: [Wave; 2],
    pkmax: u8,           // the whole file's loudest column (the drawing's scale)
    blinked: u8,         // the slot box's alternation at the last layout
    algo_at: u32,        // when ALGORITHM last turned (the slot box holds the slot a while)
    starved: u32,        // audition samples the ring ran dry (J?)
    // save states (the SAVES page, FX + SAVE)
    st: [states::Slot; states::SLOTS],
    ssel: usize,
    sarm: bool,          // REC once on a used slot: REC again replaces it
    job: Option<states::Job>,
    pub close_now: bool, // a load is done: back to the mod (the main loop closes)
    pub broken: bool,    // a load failed half way: the mod must restart
    gap: u32,            // ms between two turns of the main loop, the longest while a save / load ran (J?)
    gap_at: u32,
    // the recorder (Page::Rec) and the take's name (Page::Name)
    codec: bool,         // the line in answered
    r_sound: bool,       // the take starts on sound (else on REC)
    r_gain: u8,          // the line in level 0..15
    r_trig: u8,          // the sound that starts it: -54 + 3 x r_trig dB
    rst: Rst,
    rf: Option<RawFile>, // the take's file
    r_at: u32,           // the next ring sample to go into the file
    r_seen: u32,         // the next ring sample the meter / trigger / picture looked at
    r_len: u32,          // samples in the file
    r_drop: u32,         // samples lost (the card fell behind)
    r_end: u32,          // the take's end (saving from PSRAM)
    r_pk: u32,           // the loudest sample since the last picture
    r_meter: u32,        // the meter (falls back slowly)
    r_cpk: u8,           // the picture's column being filled
    r_cn: u32,           // ... samples in it so far
    r_ovr: u32,          // codec overruns at the take's start
    mon: u32,            // the monitor's place in the ring
    rname: [u8; NAME_N], // the take's file name (no extension), as on the card
    rn: usize,
    nb: [u8; NAME_N],    // the name being edited (spaces: nothing)
    ncur: usize,
    cp: Option<(RawFile, RawFile, u32, u32)>, // renaming: from, to, bytes copied, of
}

static mut OWN: Option<(Vm, RawVolume)> = None;

/// The card for the Lab: the save-memory mirror's (nor.rs), when the mod has one open, else the Lab's own.
pub fn fs<'a>(ext: Option<(&'a Vm, RawVolume)>) -> Option<(&'a Vm, RawVolume)> {
    if ext.is_some() {
        return ext;
    }
    let own = unsafe { &mut *(&raw mut OWN) };
    if own.is_none() {
        let card = sd::Sd::init().ok()?;
        let vm: Vm = VolumeManager::new_with_limits(card, sd::Clock, 0);
        let vol = vm.open_raw_volume(VolumeIdx(0)).ok()?;
        *own = Some((vm, vol));
    }
    own.as_ref().map(|(v, x)| (v, *x))
}

impl Lab {
    pub const fn new() -> Lab {
        Lab {
            open: false,
            dir: None,
            path: [unsafe { core::mem::zeroed() }; 4],
            depth: 0,
            n: 0,
            sel: 0,
            win0: 0,
            winn: 0,
            win: [NO_ENT; WIN],
            fm: Fmt { ok: false, ch: 0, bits: 0, float: false, be: false, signed8: false, rate: 0, off: 0, frames: 0 },
            fm_of: usize::MAX,
            moved_at: 0,
            play: Play { file: None, on: false, fm: Fmt { ok: false, ch: 0, bits: 0, float: false, be: false, signed8: false, rate: 0, off: 0, frames: 0 }, next: 0, w: 0, pos: 0, step: 0, env: 0, stop: 0, from: 0, end: 0 },
            ring: [0; RING],
            out_l: [0.0; 128],
            out_r: [0.0; 128],
            tab: hook::Table::EMPTY,
            slot: 0,
            arm: false,
            darm: false,
            send: Send::No,
            conv: Conv { file: None, fm: Fmt { ok: false, ch: 0, bits: 0, float: false, be: false, signed8: false, rate: 0, off: 0, frames: 0 }, next: 0, start: 0, end: 0, want: 0, own: true, ratio: 0, j: 0, bound: 0, acc: 0, cnt: 0, peak: 0, gain: 1.0 },
            put: hook::Put::idle(),
            btn: 0,
            keys: 0,
            sel_t: 0,
            sel_dir: 0,
            msg: [0; 40],
            msg_c: DIM,
            msg_at: 0,
            ops: [Op::Rect { x: 0, y: 0, w: 0, h: 0, c: 0 }; 48],
            nops: 0,
            pool: [0; 512],
            npool: 0,
            sent: [0; 240],
            pending: false,
            row: 0,
            dirty: false,
            drawn: 0,
            pct: 0,
            now: 0,
            master: 1.0,
            oct: 0,
            sel_at: 0,
            page: Page::List,
            tf: None,
            t_of: usize::MAX,
            ta: 0,
            tb: 0,
            hot: 0,
            zoom: 0,
            wv: [Wave::NEW; 2],
            pkmax: 0,
            blinked: 0,
            algo_at: 0,
            starved: 0,
            st: [states::Slot::Empty; states::SLOTS],
            ssel: 0,
            sarm: false,
            job: None,
            close_now: false,
            broken: false,
            gap: 0,
            gap_at: 0,
            codec: false,
            r_sound: false,
            r_gain: 5,
            r_trig: 8,
            rst: Rst::Idle,
            rf: None,
            r_at: 0,
            r_seen: 0,
            r_len: 0,
            r_drop: 0,
            r_end: 0,
            r_pk: 0,
            r_meter: 0,
            r_cpk: 0,
            r_cn: 0,
            r_ovr: 0,
            mon: 0,
            rname: [0; NAME_N],
            rn: 0,
            nb: [b' '; NAME_N],
            ncur: 0,
            cp: None,
        }
    }

    fn say(&mut self, c: u8, parts: &[&[u8]]) {
        self.msg = [0; 40];
        let mut k = 0;
        for p in parts {
            for &b in p.iter() {
                if k < 39 && b != 0 {
                    self.msg[k] = b;
                    k += 1;
                }
            }
        }
        self.msg_c = c;
        self.msg_at = self.now;
        self.dirty = true;
    }

    /// Open the Lab (the mod has just paused): the slots, the folder it was in last.
    /// `held`: the FM-1's keys and buttons (matrix) now: still held from the combo, so not presses.
    #[inline(never)]
    pub fn open(&mut self, fs: Option<(&Vm, RawVolume)>, ms: u32, held: (u32, u32)) {
        self.now = ms;
        self.open = true;
        self.keys = held.0 & ((1 << 27) - 1);
        self.btn = to_labels(held.1);
        self.arm = false;
        self.send = Send::No;
        self.msg = [0; 40];
        self.tab = hook::Table::read();
        self.slot = self.slot.min(self.tab.top());
        self.sent = [0x5EED_0000; 240]; // every row goes out
        self.row = 0;
        self.out_l = [0.0; 128];
        self.out_r = [0.0; 128];
        self.enter(fs);
        self.algo_at = ms; // (the slot shown first, a while)
        self.dirty = true;
    }

    /// Open the SAVES page (the mod has just paused): the 16 slots as the card has them.
    #[inline(never)]
    pub fn open_saves(&mut self, fs: Option<(&Vm, RawVolume)>, ms: u32, held: (u32, u32)) {
        self.now = ms;
        self.open = true;
        self.page = Page::Saves;
        self.keys = held.0 & ((1 << 27) - 1);
        self.btn = to_labels(held.1);
        self.sarm = false;
        self.send = Send::No;
        self.job = None;
        self.close_now = false;
        self.msg = [0; 40];
        self.sent = [0x5EED_0000; 240];
        self.row = 0;
        self.out_l = [0.0; 128];
        self.out_r = [0.0; 128];
        match fs {
            Some((vm, vol)) => self.st = states::scan(vm, vol),
            None => self.say(PINK, &[b"NO CARD"]),
        }
        self.dirty = true;
    }

    fn saves_buttons(&mut self, fs: Option<(&Vm, RawVolume)>, down: u32) -> bool {
        if self.job.is_some() {
            return false; // nothing until it is done
        }
        if down & HOME != 0 {
            return true;
        }
        let Some((vm, vol)) = fs else { return false };
        let save = down & (SAVE | REC) != 0; // SAVE (or REC) saves, PLAY loads
        if down & !(SAVE | REC) != 0 {
            self.sarm = false;
        }
        let k = self.ssel;
        let (nm, nn) = save_name(k);
        let sl = &nm[..nn];
        if save {
            if self.st[k] != states::Slot::Empty && !self.sarm {
                self.sarm = true;
                self.say(PINK, &[b"SAVE AGAIN: REPLACE ", sl]);
                return false;
            }
            self.sarm = false;
            match states::Job::save(vm, vol, k) {
                Ok(j) => {
                    self.job = Some(j);
                    self.gap = 0;
                    self.gap_at = self.now;
                    self.pct = 0;
                    self.msg = [0; 40];
                }
                Err(why) => self.say(PINK, &[b"FAILED: ", why.as_bytes()]),
            }
            self.dirty = true;
        }
        if down & PLAY != 0 {
            match self.st[k] {
                states::Slot::Empty => self.say(PINK, &[sl, b" IS EMPTY"]),
                states::Slot::Old => self.say(PINK, &[sl, b": OLD VERSION"]),
                states::Slot::Used => match states::Job::load(vm, vol, k) {
                    Ok(j) => {
                        self.job = Some(j);
                        self.gap = 0;
                        self.gap_at = self.now;
                        self.pct = 0;
                        self.msg = [0; 40];
                    }
                    Err(why) => self.say(PINK, &[b"FAILED: ", why.as_bytes()]),
                },
            }
            self.dirty = true;
        }
        false
    }

    fn saves_step(&mut self, vm: &Vm, vol: RawVolume) {
        let Some(j) = self.job.as_mut() else { return };
        let r = j.step(vm);
        self.pct = j.percent();
        self.dirty = true;
        let (kind, k) = (j.kind, j.slot);
        let (nm, nn) = save_name(k);
        let sl = &nm[..nn];
        match r {
            states::Res::Busy => {}
            states::Res::Done => {
                self.job = None;
                if kind == states::Kind::Save {
                    self.st[k] = states::Slot::Used; // (no scan: 16 files opened, the slow part)
                    self.say(GREEN, &[b"SAVED: ", sl]);
                } else {
                    self.close_now = true; // loaded: straight back to the mod, where it was
                }
            }
            states::Res::Failed(why) => {
                self.job = None;
                self.st = states::scan(vm, vol);
                self.say(PINK, &[b"FAILED: ", why.as_bytes()]);
            }
            states::Res::Broken => {
                self.job = None;
                self.broken = true;
            }
        }
    }

    /// The SAVES page: 16 slots, four rows at a time; the bottom says the buttons.
    fn layout_saves(&mut self) {
        self.rect(0, 0, 240, 42, PINK);
        self.text(8, 6, 2, 4, BG, 0, 240, b"SAVE STATES");
        let top = self.ssel.saturating_sub(ROWS / 2).min(states::SLOTS - ROWS);
        for r in 0..ROWS {
            let i = top + r;
            let y = Self::ROW_Y + r as u32 * Self::ROW_P;
            let on = i == self.ssel;
            if on {
                self.rect(8, y, 218, Self::ROW_P - 4, PINK);
            }
            // the slot's number on the far left, then what it holds: SAVE or EMPTY (less is more)
            let used = self.st[i] == states::Slot::Used;
            let c = if on || used { INK } else { DIM };
            let mut num = *b"00";
            let nn = if i + 1 >= 10 {
                num[0] = b'0' + ((i + 1) / 10) as u8;
                num[1] = b'0' + ((i + 1) % 10) as u8;
                2
            } else {
                num[0] = b'0' + (i + 1) as u8;
                1
            };
            self.text(Self::NAME_L as i32, y + 5, 2, 4, c, Self::NAME_L, Self::NAME_R, &num[..nn]);
            let what: &[u8] = match self.st[i] {
                states::Slot::Used => b"SAVE",
                states::Slot::Old => b"OLD",
                states::Slot::Empty => b"EMPTY",
            };
            self.text(Self::NAME_L as i32 + 44, y + 5, 2, 4, c, Self::NAME_L, Self::NAME_R, what);
        }
        // where the list is
        let (y0, h0) = (Self::ROW_Y, ROWS as u32 * Self::ROW_P - 4);
        self.rect(230, y0, 4, h0, BOX);
        let h = h0 * ROWS as u32 / states::SLOTS as u32;
        self.rect(230, y0 + (h0 - h) * top as u32 / (states::SLOTS - ROWS) as u32, 4, h, PINK);
        let y = 217;
        if let Some(kind) = self.job.as_ref().map(|j| j.kind) {
            self.rect(8, 216, 226 * self.pct / 100, 24, PINK);
            let mut t = [b' '; 12];
            let what: &[u8] = if kind == states::Kind::Save { b"SAVING" } else { b"LOADING" };
            t[..what.len()].copy_from_slice(what);
            let p = self.pct.min(100);
            t[8] = if p >= 100 { b'1' } else { b' ' };
            t[9] = if p >= 10 { b'0' + (p / 10 % 10) as u8 } else { b' ' };
            t[10] = b'0' + (p % 10) as u8;
            t[11] = b'%';
            let w: &[u8] = &t;
            self.text(12, y, 2, 3, INK, 8, 234, w);
        } else {
            self.rect(0, 213, 240, 27, INK);
            if self.msg[0] != 0 {
                let m = self.msg;
                let n = m.iter().position(|&b| b == 0).unwrap_or(40);
                let sx = if text_w(&m[..n], 2) <= 224 { 2 } else { 1 };
                let w = text_w(&m[..n], sx);
                self.text((120 - w.min(232) / 2) as i32, y, sx, 3, BG, 4, 236, &m[..n]);
            } else {
                // each button's name, then (alternating, 2 s each) what it does here
                let what = self.now / 2000 % 2 == 1;
                self.text(8, y, 2, 3, BG, 0, 240, b"\x03HOME");
                let rec: &[u8] = b"SAVE\x02";
                let w = text_w(rec, 2);
                self.text((232 - w) as i32, y, 2, 3, BG, 0, 240, rec);
                if self.st[self.ssel] == states::Slot::Used {
                    let play: &[u8] = if what { b"LOAD\x01" } else { b"PLAY\x01" };
                    let w2 = text_w(play, 2);
                    self.text((232 - w - 10 - w2) as i32, y, 2, 3, BG, 0, 240, play);
                }
            }
        }
        self.pending = true;
        self.row = 0;
    }

    /// The FM-1 asked for the whole screen (it showed its splash after a quiet link): every row goes again.
    #[inline(never)]
    pub fn redraw(&mut self) {
        self.sent = [0x5EED_0000; 240];
        self.dirty = true;
    }

    /// Close (back to the mod): files and folder closed.
    #[inline(never)]
    pub fn close(&mut self, fs: Option<(&Vm, RawVolume)>) {
        if let (true, Some((vm, _))) = (RECORDER, fs) {
            self.rec_abort(vm);
        }
        audio_in::stop();
        self.stop_now(fs.map(|f| f.0));
        if let (Some(mut j), Some((vm, _))) = (self.job.take(), fs) {
            j.abort(vm);
        }
        self.close_now = false;
        self.page = Page::List;
        if let Some((vm, _)) = fs {
            if let Some(f) = self.conv.file.take() {
                if self.conv.own {
                    let _ = vm.close_file(f);
                }
            }
            if let Some(f) = self.tf.take() {
                let _ = vm.close_file(f);
            }
            if let Some(d) = self.dir.take() {
                let _ = vm.close_dir(d);
            }
        }
        self.dir = None;
        self.tf = None;
        self.send = Send::No;
        self.open = false;
    }

    /// (Re)open the folder in `path[..depth]` under SAMPLES/ and count its entries.
    fn enter(&mut self, fs: Option<(&Vm, RawVolume)>) {
        self.stop_now(fs.map(|f| f.0));
        self.n = 0;
        self.sel = 0;
        self.winn = 0;
        self.fm_of = usize::MAX;
        self.t_of = usize::MAX;
        let Some((vm, vol)) = fs else {
            self.say(PINK, &[b"NO CARD"]);
            return;
        };
        if let Some(d) = self.dir.take() {
            let _ = vm.close_dir(d);
        }
        let Ok(root) = vm.open_root_dir(vol) else {
            self.say(PINK, &[b"CARD ERROR"]);
            return;
        };
        if RECORDER && vm.open_dir(root, "SAMPLES").map(|d| vm.close_dir(d)).is_err() {
            let _ = vm.make_dir_in_dir(root, "SAMPLES"); // (a new card: the recorder needs somewhere to put takes)
        }
        let mut d = match vm.open_dir(root, "SAMPLES") {
            Ok(d) => d,
            Err(_) => {
                let _ = vm.close_dir(root);
                self.say(PINK, &[b"NO SAMPLES FOLDER"]);
                return;
            }
        };
        let _ = vm.close_dir(root);
        for i in 0..self.depth {
            match vm.open_dir(d, &self.path[i]) {
                Ok(d2) => {
                    let _ = vm.close_dir(d);
                    d = d2;
                }
                Err(e) => {
                    self.depth = i;
                    let mut w = Buf { b: [0; 40], n: 0 };
                    let _ = core::fmt::Write::write_fmt(&mut w, format_args!("CAN'T OPEN FOLDER {:?}", e));
                    let m = w.b;
                    self.say(PINK, &[&m[..w.n]]);
                    break;
                }
            }
        }
        self.dir = Some(d);
        self.n = self.walk(vm, usize::MAX) + self.vrow();
        self.fill(vm);
        self.dirty = true;
    }

    /// Walk the folder: entries shown (folders, .WAV, .AIF), in the card's order. Those in the window from `from`
    /// are kept. Returns how many were seen (all of them when `from` is usize::MAX: just counting).
    fn walk(&mut self, vm: &Vm, from: usize) -> usize {
        let Some(d) = self.dir else { return 0 };
        let mut store = [0u8; 256];
        let mut lfn = LfnBuffer::new(&mut store);
        let mut i = 0usize;
        let mut kept = 0usize;
        let win = &mut self.win;
        let _ = vm.iterate_dir_lfn(d, &mut lfn, |e, long| {
            let a = e.attributes;
            if a.is_volume() || a.is_hidden() || a.is_system() || e.name.base_name().first() == Some(&b'.') {
                return ControlFlow::Continue(());
            }
            let dir = a.is_directory();
            if !dir && !matches!(e.name.extension(), b"WAV" | b"AIF") {
                return ControlFlow::Continue(());
            }
            if long.is_some_and(|l| l.starts_with('.')) {
                return ControlFlow::Continue(());
            }
            if from != usize::MAX && i >= from {
                if kept >= WIN {
                    return ControlFlow::Break(());
                }
                let w = &mut win[kept];
                w.sfn = e.name;
                w.dir = dir;
                w.name = [0; NAME];
                let mut k = 0;
                let mut put = |c: u8| {
                    if k < NAME {
                        w.name[k] = if (b' '..127).contains(&c) { c.to_ascii_uppercase() } else { b'?' };
                        k += 1;
                    }
                };
                match long {
                    Some(l) => {
                        let b = l.as_bytes();
                        let stem = if dir { b } else { b.iter().rposition(|&c| c == b'.').map_or(b, |p| &b[..p]) };
                        for ch in core::str::from_utf8(stem).unwrap_or("?").chars() {
                            put(if ch.is_ascii() { ch as u8 } else { b'?' });
                        }
                    }
                    None => {
                        for &c in e.name.base_name() {
                            put(c);
                        }
                        if dir && !e.name.extension().is_empty() {
                            put(b'.');
                            for &c in e.name.extension() {
                                put(c);
                            }
                        }
                    }
                }
                w.len = k as u8;
                kept += 1;
            }
            i += 1;
            ControlFlow::Continue(())
        });
        if from != usize::MAX {
            self.winn = kept;
        }
        i
    }

    /// Where an entry is in the list (the same filter as walk).
    fn index_of(&self, vm: &Vm, sfn: ShortFileName) -> Option<usize> {
        let d = self.dir?;
        let (mut i, mut at) = (0usize, None);
        let _ = vm.iterate_dir(d, |e| {
            let a = e.attributes;
            if a.is_volume() || a.is_hidden() || a.is_system() || e.name.base_name().first() == Some(&b'.') {
                return ControlFlow::Continue(());
            }
            if !a.is_directory() && !matches!(e.name.extension(), b"WAV" | b"AIF") {
                return ControlFlow::Continue(());
            }
            if e.name == sfn {
                at = Some(i);
                return ControlFlow::Break(());
            }
            i += 1;
            ControlFlow::Continue(())
        });
        at
    }

    /// Make sure the selection (and the rows shown around it) are in the window.
    fn fill(&mut self, vm: &Vm) {
        let v = self.vrow();
        let top = self.top().saturating_sub(v);
        if self.winn > 0 && top >= self.win0 && (top + ROWS).min(self.n - v) <= self.win0 + self.winn {
            return;
        }
        self.win0 = top.saturating_sub((WIN - ROWS) / 2);
        self.walk(vm, self.win0);
    }
    fn top(&self) -> usize {
        self.sel.saturating_sub(ROWS / 2).min(self.n.saturating_sub(ROWS))
    }
    /// The recorder's row (the first at the top of SAMPLES): 1, else 0.
    fn vrow(&self) -> usize {
        (RECORDER && self.depth == 0 && self.dir.is_some()) as usize
    }
    fn ent(&self, i: usize) -> Option<&Ent> {
        let v = self.vrow();
        if i < v {
            return Some(&REC_ENT);
        }
        let i = i - v;
        if i >= self.win0 && i < self.win0 + self.winn { Some(&self.win[i - self.win0]) } else { None }
    }

    fn open_sel(&self, vm: &Vm) -> Option<RawFile> {
        let e = self.ent(self.sel)?;
        if e.dir {
            return None;
        }
        vm.open_file_in_dir(self.dir?, &e.sfn, Mode::ReadOnly).ok()
    }

    /// The selected file's format (read once it has been selected a moment).
    fn info(&mut self, vm: &Vm) {
        if self.fm_of == self.sel {
            return;
        }
        self.fm_of = self.sel;
        self.fm = Fmt::default();
        if let Some(f) = self.open_sel(vm) {
            self.fm = parse(vm, f);
            let _ = vm.close_file(f);
        }
        self.dirty = true;
    }

    fn stop_now(&mut self, vm: Option<&Vm>) {
        self.play.on = false;
        self.play.stop = 0;
        if let (Some(f), Some(vm)) = (self.play.file.take(), vm) {
            if Some(f) != self.tf {
                let _ = vm.close_file(f);
            }
        }
        self.play.file = None;
    }

    /// Start the selected file at key `k` (KEY_C: as recorded).
    fn audition(&mut self, vm: &Vm, k: usize, oct: i32) {
        self.stop_now(Some(vm));
        self.info(vm);
        if !self.fm.ok {
            self.say(PINK, &[b"CAN'T READ IT"]);
            return;
        }
        let Some(f) = self.tf.or_else(|| self.open_sel(vm)) else { return };
        let (a, b) = self.range();
        let p = &mut self.play;
        p.fm = self.fm;
        p.from = a;
        p.end = b;
        p.next = a;
        p.w = 0;
        p.pos = 0;
        p.step = ((self.fm.rate as f32 * SEMI[k.min(26)]) as u64) * (1u64 << 32) / FS;
        p.step = if oct >= 0 { p.step << oct } else { p.step >> -oct };
        p.env = 0;
        p.stop = 0;
        p.file = Some(f);
        for _ in 0..4 {
            self.feed(vm);
        }
        self.play.on = true;
        self.dirty = true;
    }

    /// More of the playing file into the ring, if there is room.
    fn feed(&mut self, vm: &Vm) {
        let p = &mut self.play;
        let Some(f) = p.file else { return };
        let fb = p.fm.fb();
        let head = (p.pos >> 32) as u32;
        let room = RING as u32 - 2 - (p.w - head.min(p.w));
        let left = p.end.saturating_sub(p.next);
        let n = (2048 / fb).min(room).min(left);
        if n < 64.min(left) {
            return; // (the ring nearly full: later, in a bigger piece)
        }
        if n == 0 {
            return;
        }
        let mut buf = [0u8; 2048];
        let len = (n * fb) as usize;
        if vm.file_seek_from_start(f, p.fm.off + p.next * fb).is_err() {
            p.next = p.fm.frames;
            return;
        }
        let got = vm.read(f, &mut buf[..len]).unwrap_or(0) as u32 / fb;
        for i in 0..got {
            let s = p.fm.mono(&buf[(i * fb) as usize..((i + 1) * fb) as usize]);
            self.ring[(p.w as usize) % RING] = s.clamp(-32768, 32767) as i16;
            p.w += 1;
        }
        p.next = if got < n { p.end } else { p.next + got };
    }

    /// The MASTER pot (0..1023): the audition's level (squared, as a volume knob feels).
    #[inline(never)]
    pub fn master(&mut self, v: u32) {
        let x = v.min(1023) as f32 / 1023.0;
        self.master = x * x;
    }

    #[inline(never)]
    pub fn out_ptrs(&self) -> (*const f32, *const f32) {
        (self.out_l.as_ptr(), self.out_r.as_ptr())
    }

    /// One audio block (128 frames) into out_l / out_r.
    #[inline(never)]
    pub fn render(&mut self) {
        if RECORDER && self.page == Page::Rec {
            // the line in, as it comes (a little behind, so the FM-1's clock and the codec's never cross)
            let w = audio_in::written();
            let lag = w.wrapping_sub(self.mon);
            if lag > 4096 || lag < 256 {
                self.mon = w.wrapping_sub(1024);
            }
            let g = 0.8 / 65536.0 * self.master;
            for i in 0..128 {
                let v = if self.codec { audio_in::at(self.mon.wrapping_add(i as u32)) as f32 * g } else { 0.0 };
                self.out_l[i] = v;
                self.out_r[i] = v;
            }
            self.mon = self.mon.wrapping_add(128);
            return;
        }
        let p = &mut self.play;
        for i in 0..128 {
            let mut v = 0.0f32;
            if p.on {
                let ip = (p.pos >> 32) as u32;
                if ip + 1 < p.w {
                    let a = self.ring[ip as usize % RING] as f32;
                    let b = self.ring[(ip + 1) as usize % RING] as f32;
                    let fr = (p.pos & 0xFFFF_FFFF) as f32 * (1.0 / 4_294_967_296.0);
                    let mut g = 0.8 / 65536.0 * self.master;
                    if p.env < 64 {
                        p.env += 1;
                        g *= p.env as f32 / 64.0;
                    }
                    if p.stop > 0 {
                        p.stop -= 1;
                        g *= p.stop as f32 / 64.0;
                        if p.stop == 0 {
                            p.on = false;
                        }
                    }
                    v = (a + (b - a) * fr) * g;
                    p.pos += p.step;
                } else if p.next >= p.end {
                    p.on = false; // the end
                } else {
                    self.starved += 1; // the card fell behind: a gap in the sound
                }
            }
            self.out_l[i] = v;
            self.out_r[i] = v;
        }
    }

    // ---- input ----

    /// The FM-1's report (notes, matrix buttons).
    #[inline(never)]
    pub fn raw(&mut self, fs: Option<(&Vm, RawVolume)>, notes: u32, buttons: u32, ms: u32) -> bool {
        let close = self.buttons(fs, to_labels(buttons), ms);
        self.keys_in(fs, notes, ms);
        close
    }
    /// An encoder of the FM-1 (hardware index).
    #[inline(never)]
    pub fn raw_enc(&mut self, fs: Option<(&Vm, RawVolume)>, hw: u32, n: i32, ms: u32) {
        self.enc(fs, ROLE_OF_HW[(hw % 7) as usize] as u32, n, ms);
    }

    /// Buttons by label. True: HOME was pressed, the Lab should close.
    #[inline(never)]
    pub fn buttons(&mut self, fs: Option<(&Vm, RawVolume)>, labels: u32, ms: u32) -> bool {
        self.now = ms;
        let down = labels & !self.btn;
        self.btn = labels;
        if down == 0 {
            return false;
        }
        if self.page == Page::Saves {
            return self.saves_buttons(fs, down);
        }
        if self.send != Send::No {
            return false; // nothing until the send is done
        }
        if down & HOME != 0 && self.depth == 0 && self.page == Page::List {
            return true; // at the top: back to the mod
        }
        let Some((vm, vol)) = fs else { return false };
        if down & !REC != 0 {
            self.arm = false;
        }
        if down & !EDIT != 0 {
            self.darm = false;
        }
        if RECORDER && down & EDIT != 0 && matches!(self.page, Page::List | Page::Trim) {
            self.del_button(vm, vol);
            return false;
        }
        if self.page == Page::Trim {
            self.trim_buttons(vm, down);
            return false;
        }
        if RECORDER && self.page == Page::Rec {
            self.rec_buttons(vm, down);
            return false;
        }
        if RECORDER && self.page == Page::Name {
            self.name_buttons(vm, vol, down);
            return false;
        }
        if RECORDER && down & (PLAY | REC) != 0 && self.sel < self.vrow() {
            self.rec_open(vm);
            return false;
        }
        if down & (OCTDN | OCTUP) != 0 {
            self.set_oct(self.oct + if down & OCTUP != 0 { 1 } else { -1 });
        }
        if down & HOME != 0 && self.depth > 0 {
            self.depth -= 1;
            let back = self.path[self.depth];
            self.enter(Some((vm, vol)));
            // land on the folder we came out of
            self.sel = self.index_of(vm, back).map_or(0, |i| i + self.vrow());
            self.fill(vm);
            self.dirty = true;
        }
        if down & PLAY != 0 {
            match self.ent(self.sel).copied() {
                Some(e) if e.dir => {
                    if self.depth < 4 {
                        self.path[self.depth] = e.sfn;
                        self.depth += 1;
                        self.enter(Some((vm, vol)));
                    }
                }
                Some(_) => {
                    if self.play.on && self.play.stop == 0 {
                        self.play.stop = 64;
                        self.dirty = true;
                    } else {
                        self.audition(vm, KEY_C, 0);
                    }
                }
                None => {}
            }
        }
        if down & REC != 0 && self.ent(self.sel).is_some_and(|e| !e.dir) {
            self.trim_open(vm); // (REC again there sends)
        }
        false
    }

    #[inline(never)]
    pub fn keys_in(&mut self, fs: Option<(&Vm, RawVolume)>, notes: u32, ms: u32) {
        self.now = ms;
        let notes = notes & ((1 << 27) - 1);
        let down = notes & !self.keys;
        self.keys = notes;
        if down == 0 || self.send != Send::No || matches!(self.page, Page::Saves | Page::Rec | Page::Name) {
            return;
        }
        let Some((vm, _)) = fs else { return };
        if self.ent(self.sel).is_some_and(|e| !e.dir) {
            self.arm = false;
            self.audition(vm, down.trailing_zeros() as usize, self.oct);
        }
    }

    /// An encoder by role, + = clockwise.
    #[inline(never)]
    pub fn enc(&mut self, fs: Option<(&Vm, RawVolume)>, role: u32, n: i32, ms: u32) {
        self.now = ms;
        if n == 0 || self.send != Send::No {
            return;
        }
        if self.page == Page::Saves {
            if self.job.is_none() && matches!(role, SELECT | ALGO | PRESETS) {
                let s = (self.ssel as i32 + n.signum()).clamp(0, states::SLOTS as i32 - 1) as usize;
                if s != self.ssel {
                    self.ssel = s;
                    self.sarm = false;
                    self.msg = [0; 40];
                    self.dirty = true;
                }
            }
            return;
        }
        if RECORDER && self.page == Page::Rec {
            return self.rec_knob(role, n);
        }
        if RECORDER && self.page == Page::Name {
            return self.name_knob(role, n);
        }
        let Some((vm, _)) = fs else { return };
        match role {
            K1 | K4 if self.page == Page::Trim => self.trim_knob(role == K4, n, ms),
            K2 if self.page == Page::Trim => self.zoom_by(n > 0),
            K3 if self.page == Page::Trim => self.pos_knob(n, ms),
            SELECT | PRESETS if self.page == Page::Trim => {}
            SELECT | PRESETS => {
                // as the Jam Box's knobs (ui_input.c accel): slow is one a detent, faster 3, fastest 8
                let dt = ms.wrapping_sub(self.sel_t) / n.unsigned_abs();
                let m = if self.sel_dir != n.signum() || dt >= 200 || dt >= 28 { 1 } else if dt >= 11 { 3 } else { 8 };
                self.sel_t = ms;
                self.sel_dir = n.signum();
                let m = if role == PRESETS { ROWS as i32 } else { m };
                let s = (self.sel as i32 + n * m).clamp(0, self.n.max(1) as i32 - 1) as usize;
                if s != self.sel {
                    self.sel = s;
                    self.sel_at = ms;
                    self.arm = false;
                    self.stop_now(Some(vm));
                    self.fill(vm);
                    self.moved_at = ms;
                    self.dirty = true;
                }
            }
            ALGO => {
                let top = self.tab.top() as i32; // the last slot ALGORITHM can pick (hooks/mod.rs)
                let s = (self.slot as i32 + n.signum()).clamp(0, top) as usize;
                self.algo_at = ms;
                self.dirty = true;
                if s != self.slot {
                    self.slot = s;
                    self.arm = false;
                    self.dirty = true;
                }
            }
            _ => {}
        }
    }

    // ---- the trim page ----
    fn set_oct(&mut self, o: i32) {
        self.oct = o.clamp(-2, 2);
        let mut t = *b"OCT +0";
        t[4] = if self.oct < 0 { b'-' } else { b'+' };
        t[5] = b'0' + self.oct.unsigned_abs() as u8;
        self.say(INK, &[&t]);
    }

    /// The frames heard and sent: the trim, when it is this file's, else all of it.
    fn range(&self) -> (u32, u32) {
        if self.t_of == self.sel && self.fm_of == self.sel && self.ta < self.tb && self.tb <= self.fm.frames {
            (self.ta, self.tb)
        } else {
            (0, self.fm.frames)
        }
    }

    /// The source frame where the slot's memory runs out, counting from START.
    fn cut(&self) -> u32 {
        let rate = self.fm.rate.min(hook::RATE).max(1);
        let room = self.tab.room(self.slot.min(hook::MAX.saturating_sub(1))) as u64 * self.fm.rate as u64 / rate as u64;
        (self.ta as u64 + room).min(u32::MAX as u64) as u32
    }

    fn trim_open(&mut self, vm: &Vm) {
        self.stop_now(Some(vm));
        self.info(vm);
        if !self.fm.ok {
            self.say(PINK, &[b"CAN'T READ IT"]);
            return;
        }
        let Some(f) = self.open_sel(vm) else {
            self.say(PINK, &[b"CAN'T OPEN IT"]);
            return;
        };
        self.tf = Some(f);
        if self.t_of != self.sel || self.tb > self.fm.frames {
            self.t_of = self.sel;
            self.ta = 0;
            self.tb = self.fm.frames;
            self.zoom = 0;
            self.hot = 0;
        }
        self.wv[0] = Wave::view(0, self.fm.frames);
        self.pkmax = 0;
        self.view();
        self.page = Page::Trim;
        self.msg = [0; 40];
        self.dirty = true;
    }

    fn trim_close(&mut self, vm: &Vm) {
        self.stop_now(Some(vm));
        if let Some(f) = self.tf.take() {
            let _ = vm.close_file(f);
        }
        self.page = Page::List;
        self.msg = [0; 40];
        self.dirty = true;
    }

    fn trim_buttons(&mut self, vm: &Vm, down: u32) {
        if down & HOME != 0 {
            self.trim_close(vm);
            return;
        }
        if down & (OCTDN | OCTUP) != 0 {
            self.set_oct(self.oct + if down & OCTUP != 0 { 1 } else { -1 });
        }
        if down & PLAY != 0 {
            if self.play.on && self.play.stop == 0 {
                self.play.stop = 64;
                self.dirty = true;
            } else {
                self.audition(vm, KEY_C, 0);
            }
        }
        if down & REC != 0 {
            self.rec(vm);
        }
    }

    /// The zoomed view's width (frames): 1/6^zoom of the file, a whole number of frames a column (one at most).
    fn zoom_w(&self) -> u32 {
        let fr = self.fm.frames;
        let mut w = fr;
        for _ in 0..self.zoom {
            w /= 6;
        }
        if self.zoom == 0 { fr } else { (w / GW as u32 * GW as u32).max(GW as u32).min(fr) }
    }
    /// The frame of the marker last turned (the zoom is round it).
    fn mark(&self) -> u32 {
        if self.hot == 0 { self.ta } else { self.tb.saturating_sub(1) }
    }

    /// The ZOOM knob: a level a detent (clockwise in), round the bracket last turned when it is in view, else round
    /// the middle of the view (POS moved it away).
    fn zoom_by(&mut self, inn: bool) {
        let v = self.wv[1];
        let mid = if self.zoom > 0 && !(0..GW as i32).contains(&v.col(self.mark())) { v.x0 + v.w / 2 } else { self.mark() };
        if inn {
            if self.fm.frames <= GW as u32 || (self.zoom > 0 && self.zoom_w() <= GW as u32) {
                return; // (one frame a column already)
            }
            self.zoom += 1;
        } else {
            if self.zoom == 0 {
                return;
            }
            self.zoom -= 1;
        }
        self.view_at(mid);
        self.dirty = true;
    }

    /// The zoomed view round the marker, read again.
    fn view(&mut self) {
        self.view_at(self.mark());
    }
    fn view_at(&mut self, mid: u32) {
        if self.zoom == 0 {
            return;
        }
        let w = self.zoom_w();
        let x0 = mid.saturating_sub(w / 2).min(self.fm.frames - w);
        self.wv[1] = Wave::view(x0, w);
    }

    /// The zoomed view moved k columns (+ right): the peaks still in view kept, only the new part read.
    fn pan(&mut self, k: i32) -> bool {
        let fr = self.fm.frames;
        let v = &mut self.wv[1];
        let fpc = v.w / GW as u32;
        if self.zoom == 0 || fpc == 0 {
            return false;
        }
        let k = (k as i64).clamp(-((v.x0 / fpc) as i64), ((fr - v.x0 - v.w) / fpc) as i64);
        if k == 0 {
            return false;
        }
        let x1 = (v.x0 as i64 + k * fpc as i64) as u32;
        if k.unsigned_abs() as usize >= GW {
            *v = Wave::view(x1, v.w);
            return true;
        }
        let kc = k.unsigned_abs() as usize;
        // what is still to read: the new part, and what was still to read of the old (max of peaks: a frame read
        // twice changes nothing)
        let (a, b) = (v.at.max(x1), v.end.min(x1 + v.w));
        if k > 0 {
            v.pk.copy_within(kc.., 0);
            v.pk[GW - kc..].fill(0);
            v.at = if a < b { a } else { (v.x0 + v.w).max(x1) };
            v.end = x1 + v.w;
            v.back = false;
        } else {
            v.pk.copy_within(..GW - kc, kc);
            v.pk[..kc].fill(0);
            v.at = x1;
            v.end = if a < b { b } else { v.x0.min(x1 + v.w) };
            v.back = true;
        }
        v.x0 = x1;
        true
    }

    /// The marker moved: the zoomed view follows it (centred on it again, the peaks still in view kept).
    fn follow(&mut self) {
        if self.zoom == 0 {
            return;
        }
        let c = self.wv[1].col(self.mark());
        if (24..200).contains(&c) {
            return;
        }
        if (c - GW as i32 / 2).abs() < GW as i32 {
            self.pan(c - GW as i32 / 2);
        }
        if !(0..GW as i32).contains(&self.wv[1].col(self.mark())) {
            self.view();
        }
    }

    /// The knobs' speed (the Jam Box's ui_input.c accel): slow one a detent, faster 3, fastest 8.
    fn accel(&mut self, n: i32, ms: u32) -> i64 {
        let dt = ms.wrapping_sub(self.sel_t) / n.unsigned_abs();
        let m = if self.sel_dir != n.signum() || dt >= 28 { 1 } else if dt >= 11 { 3 } else { 8 };
        self.sel_t = ms;
        self.sel_dir = n.signum();
        m
    }

    /// POS (K3): the zoomed view left / right, 4 columns a detent (faster the faster it turns).
    fn pos_knob(&mut self, n: i32, ms: u32) {
        let m = self.accel(n, ms) as i32;
        if self.zoom == 0 {
            self.say(INK, &[b"ZOOM IN FIRST"]);
            return;
        }
        if self.pan(n * m * 4) {
            self.dirty = true;
        }
    }

    /// START (K1) or END (K4) turned: a column of the view a detent, faster the faster it turns.
    fn trim_knob(&mut self, end: bool, n: i32, ms: u32) {
        let m = self.accel(n, ms);
        self.hot = end as u8;
        let fr = self.fm.frames as i64;
        let w = if self.zoom > 0 { self.wv[1].w } else { self.fm.frames } as i64;
        let mut d = n as i64 * m * w / GW as i64;
        if d == 0 {
            d = n.signum() as i64;
        }
        let minl = 64.min(fr / 2).max(1);
        if end {
            self.tb = (self.tb as i64 + d).clamp(self.ta as i64 + minl, fr) as u32;
            if self.play.on {
                self.play.end = self.tb.max(self.play.next);
            }
        } else {
            self.ta = (self.ta as i64 + d).clamp(0, self.tb as i64 - minl) as u32;
        }
        self.arm = false;
        self.follow();
        self.dirty = true;
    }

    /// More of the waveform from the card: the view shown first.
    fn scan(&mut self, vm: &Vm) {
        let Some(f) = self.tf else { return };
        let i = if self.zoom > 0 && !self.wv[1].done() { 1 } else if !self.wv[0].done() { 0 } else { return };
        let fm = self.fm;
        let fb = fm.fb();
        let mut buf = [0u8; 2048];
        for _ in 0..4 {
            let v = &mut self.wv[i];
            let n = (2048 / fb).min(v.end - v.at);
            if n == 0 {
                break;
            }
            let from = if v.back { v.end - n } else { v.at };
            if vm.file_seek_from_start(f, fm.off + from * fb).is_err() {
                v.at = v.end;
                break;
            }
            let got = vm.read(f, &mut buf[..(n * fb) as usize]).unwrap_or(0) as u32 / fb;
            if got < n {
                v.at = v.end;
                break;
            }
            for k in 0..got {
                let c = ((from + k - v.x0) as u64 * GW as u64 / v.w as u64) as usize;
                if c < GW {
                    let s = (fm.mono(&buf[(k * fb) as usize..((k + 1) * fb) as usize]).unsigned_abs() >> 7).min(255) as u8;
                    v.pk[c] = v.pk[c].max(s);
                }
            }
            if v.back {
                v.end -= got;
            } else {
                v.at += got;
            }
        }
        if i == 0 {
            self.pkmax = self.wv[0].pk.iter().copied().max().unwrap_or(0);
        }
        self.dirty = true;
    }

    // ---- send ----
    fn rec(&mut self, vm: &Vm) {
        match self.ent(self.sel) {
            Some(e) if !e.dir => {}
            _ => return,
        }
        if self.tab.full(self.slot) && !self.arm {
            self.arm = true;
            self.say(PINK, &[b"REC AGAIN: REPLACE"]);
            return;
        }
        self.arm = false;
        self.stop_now(Some(vm));
        if self.tf.is_none() {
            self.fm_of = usize::MAX; // (read again; on the trim page the file is open and already read)
        }
        self.info(vm);
        let fm = self.fm;
        if !fm.ok {
            self.say(PINK, &[b"CAN'T READ IT"]);
            return;
        }
        if fm.rate < 4000 {
            self.say(PINK, &[b"RATE TOO LOW"]);
            return;
        }
        let rate = fm.rate.min(hook::RATE);
        let (a, b) = self.range();
        let want = ((b - a) as u64 * rate as u64 / fm.rate as u64) as u32;
        let e = *self.ent(self.sel).unwrap();
        if let Err(why) = self.put.plan(self.slot, &e.name[..(e.len as usize).min(12)], rate, want) {
            self.say(PINK, &[why.as_bytes()]);
            return;
        }
        let (f, own) = match self.tf {
            Some(f) => (f, false),
            None => match self.open_sel(vm) {
                Some(f) => (f, true),
                None => {
                    self.say(PINK, &[b"CAN'T OPEN IT"]);
                    return;
                }
            },
        };
        let ratio = ((fm.rate as u64) << 32) / rate as u64;
        let c = &mut self.conv;
        c.file = Some(f);
        c.own = own;
        c.fm = fm;
        c.start = a;
        c.want = want;
        c.next = a;
        c.end = (a + ((self.put.n as u64 * ratio) >> 32) as u32 + 1).min(b);
        c.ratio = ratio;
        c.peak = 1;
        self.send = Send::Peak;
        self.pct = 0;
        self.msg = [0; 40];
    }

    /// Source frames `next..` as mono samples, up to `n` (a read of 2 KB at most).
    fn src_chunk(c: &mut Conv, vm: &Vm, buf: &mut [u8; 2048]) -> u32 {
        let Some(f) = c.file else { return 0 };
        let fb = c.fm.fb();
        let n = (2048 / fb).min(c.end - c.next);
        if n == 0 || vm.file_seek_from_start(f, c.fm.off + c.next * fb).is_err() {
            return 0;
        }
        vm.read(f, &mut buf[..(n * fb) as usize]).unwrap_or(0) as u32 / fb
    }

    fn send_step(&mut self, vm: &Vm) {
        match self.send {
            Send::Peak => {
                let mut buf = [0u8; 2048];
                for _ in 0..8 {
                    let got = Self::src_chunk(&mut self.conv, vm, &mut buf);
                    let fb = self.conv.fm.fb() as usize;
                    for i in 0..got as usize {
                        let s = self.conv.fm.mono(&buf[i * fb..i * fb + fb]).abs();
                        self.conv.peak = self.conv.peak.max(s);
                    }
                    self.conv.next += got;
                    if got == 0 || self.conv.next >= self.conv.end {
                        let c = &mut self.conv;
                        c.gain = 0.97 * 32767.0 / c.peak as f32;
                        c.next = c.start;
                        c.j = 0;
                        c.bound = c.start + (c.ratio >> 32) as u32;
                        c.acc = 0;
                        c.cnt = 0;
                        self.send = Send::Write;
                        break;
                    }
                }
                let c = &self.conv;
                self.pct = 10 * (c.next - c.start) / (c.end - c.start).max(1);
            }
            Send::Write => {
                let conv = &mut self.conv;
                let mut buf = [0u8; 2048];
                let (mut have, mut at) = (0u32, 0u32); // frames in buf, the next one to use
                let mut pull = |out: &mut [i16]| -> usize {
                    let mut k = 0;
                    while k < out.len() {
                        if at >= have {
                            conv.next += have;
                            have = Self::src_chunk(conv, vm, &mut buf);
                            at = 0;
                            if have == 0 {
                                break;
                            }
                        }
                        let fb = conv.fm.fb() as usize;
                        conv.acc += conv.fm.mono(&buf[at as usize * fb..at as usize * fb + fb]) as i64;
                        conv.cnt += 1;
                        at += 1;
                        if conv.next + at >= conv.bound {
                            let v = conv.acc as f32 / conv.cnt.max(1) as f32 * conv.gain;
                            out[k] = v.clamp(-32767.0, 32767.0) as i16;
                            k += 1;
                            conv.j += 1;
                            conv.bound = conv.start + (((conv.j + 1) * conv.ratio) >> 32) as u32;
                            conv.acc = 0;
                            conv.cnt = 0;
                        }
                    }
                    k
                };
                let ph = self.put.step(&mut pull);
                // the frames read but not used go back (the next step reads from conv.next again)
                conv.next += at;
                self.pct = 10 + self.put.percent() * 90 / 100;
                match ph {
                    hook::Phase::Done => self.sent_ok(vm),
                    hook::Phase::Failed(why) => {
                        self.end_send(vm);
                        self.say(PINK, &[b"FAILED: ", why.as_bytes()]);
                    }
                    _ => {}
                }
            }
            Send::No => {}
        }
        self.dirty = true;
    }
    fn end_send(&mut self, vm: &Vm) {
        if let Some(f) = self.conv.file.take() {
            if self.conv.own {
                let _ = vm.close_file(f);
            }
        }
        self.send = Send::No;
        self.tab = hook::Table::read();
    }
    fn sent_ok(&mut self, vm: &Vm) {
        self.end_send(vm);
        if self.page == Page::Trim {
            self.trim_close(vm); // loaded: back to the list, the message there
        }
        let (sl, sln) = hook::label(self.slot);
        let cut = self.put.n < self.conv.want;
        self.say(GREEN, &[b"SENT TO ", &sl[..sln], if cut { b" - CUT" } else { b"" }]);
    }

    // ---- the main loop's turn ----
    #[inline(never)]
    pub fn step(&mut self, fs: Option<(&Vm, RawVolume)>, ms: u32) {
        self.now = ms;
        if self.page == Page::Saves {
            if self.job.is_some() {
                self.gap = self.gap.max(ms.wrapping_sub(self.gap_at));
            }
            if let Some((vm, vol)) = fs {
                self.saves_step(vm, vol);
            }
            self.gap_at = ms;
        } else if let Some((vm, vol)) = fs {
            if RECORDER && self.page == Page::Rec {
                self.rec_step(vm);
            } else if RECORDER && self.page == Page::Name {
                self.name_step(vm, vol);
            } else if self.send != Send::No {
                self.send_step(vm);
            } else {
                if self.play.on {
                    for _ in 0..4 {
                        let at = self.play.next;
                        self.feed(vm); // (the ring kept full: a few reads a pass when it needs them)
                        if self.play.next == at {
                            break;
                        }
                    }
                } else if self.play.file.is_some() {
                    self.stop_now(Some(vm));
                    self.dirty = true;
                }
                if self.fm_of != self.sel && ms.wrapping_sub(self.moved_at) > 150 && self.ent(self.sel).is_some_and(|e| !e.dir) {
                    self.info(vm);
                }
                if self.page == Page::Trim && !self.play.on {
                    self.scan(vm); // (not while it plays: the card is the sound's)
                }
            }
        }
        if self.msg[0] != 0 && self.send == Send::No && !self.arm && !self.sarm && !self.darm && ms.wrapping_sub(self.msg_at) > 4000 {
            self.msg = [0; 40];
            self.dirty = true;
        }
        if self.page == Page::Rec && ms.wrapping_sub(self.drawn) >= 50 {
            self.dirty = true; // the meter, the picture, the time
        }
        if self.page == Page::Trim && self.play.on && ms.wrapping_sub(self.drawn) >= 40 {
            self.dirty = true; // the play head
        }
        if self.blinks() != self.blinked {
            self.blinked = self.blinks();
            self.dirty = true; // a name that alternates with what it does
        }
        if self.scrolls() && ms.wrapping_sub(self.drawn) >= 30 {
            self.dirty = true; // the selected name scrolling
        }
        // a new picture only once the last has all gone to the FM-1 (no half old, half new rows: the tear)
        if self.dirty && !self.pending && ms.wrapping_sub(self.drawn) >= 20 {
            self.dirty = false;
            self.drawn = ms;
            self.layout();
        }
    }

    // ---- the page ----
    fn rect(&mut self, x: u32, y: u32, w: u32, h: u32, c: u8) {
        if self.nops < self.ops.len() {
            self.ops[self.nops] = Op::Rect { x: x as u8, y: y as u8, w: w.min(240 - x) as u8, h: h as u8, c };
            self.nops += 1;
        }
    }
    /// Text from x (may start left of the clip, to scroll), drawn only between cl and cr.
    fn text(&mut self, x: i32, y: u32, sx: u32, sy: u32, c: u8, cl: u32, cr: u32, s: &[u8]) {
        let n = s.iter().position(|&b| b == 0).unwrap_or(s.len()).min(255).min(self.pool.len() - self.npool);
        if self.nops < self.ops.len() && n > 0 {
            self.pool[self.npool..self.npool + n].copy_from_slice(&s[..n]);
            self.ops[self.nops] = Op::Text { x: x as i16, y: y as u8, sx: sx as u8, sy: sy as u8, c, cl: cl as u8, cr: cr.min(240) as u8, s: self.npool as u16, n: n as u8 };
            self.npool += n;
            self.nops += 1;
        }
    }

    // the list's rows: boxes from y ROW_Y, ROW_P apart; names in the menu's 2 x 4 letters between NAME_L and NAME_R
    const ROW_Y: u32 = 48;
    const ROW_P: u32 = 42;
    const NAME_L: u32 = 16;
    const NAME_R: u32 = 218;

    /// The selected name, when it is too long for its row: how far it has scrolled (px), and how far it goes.
    fn scroll_of(&self) -> Option<(u32, u32)> {
        let e = self.ent(self.sel)?;
        let w = text_w(&e.name[..e.len as usize], 2) + if e.dir { 14 } else { 0 };
        let room = Self::NAME_R - Self::NAME_L;
        if w <= room {
            return None;
        }
        let over = w - room;
        let (hold, px) = (1200u32, 25u32); // a pause at each end, 25 ms a pixel
        let period = 2 * hold + over * px;
        let t = self.now.wrapping_sub(self.sel_at) % period;
        Some(((t.saturating_sub(hold) / px).min(over), over))
    }
    /// The slot box's alternation: the knob's name (true) or the slot (false). The slot is held a while after ALGORITHM
    /// turns (you see what you just changed), then they alternate, 2 s each.
    fn blink(&self, at: u32) -> bool {
        let e = self.now.wrapping_sub(at);
        e >= HOLD_SHOW && (e - HOLD_SHOW) / 2000 % 2 == 0
    }
    fn blinks(&self) -> u8 {
        if self.page == Page::Saves {
            return (self.now / 2000 % 2) as u8; // the bottom's words alternate
        }
        if self.page == Page::Name {
            return (self.now / 2000 % 2) as u8;
        }
        if self.page == Page::Rec {
            return (self.rst == Rst::Armed && self.now / 500 % 2 == 1) as u8;
        }
        self.blink(self.algo_at) as u8 | ((self.can_delete() && self.now / 2000 % 2 == 1) as u8) << 1
    }
    fn scrolls(&self) -> bool {
        self.send == Send::No && self.page == Page::List && self.scroll_of().is_some()
    }

    fn layout(&mut self) {
        self.nops = 0;
        self.npool = 0;
        if self.page == Page::Saves {
            return self.layout_saves();
        }
        if RECORDER && self.page == Page::Rec {
            return self.layout_rec();
        }
        if RECORDER && self.page == Page::Name {
            return self.layout_name();
        }
        let sending = self.send != Send::No;
        let trim = self.page == Page::Trim;
        // the title as the menu's (the trim page: the length), and the slot it sends to (ALGORITHM): pink when it holds a sound
        // the top banner: pink, the title black on it
        self.rect(0, 0, 240, 42, PINK);
        if trim {
            self.layout_trim();
        } else {
            self.text(8, 6, 2, 4, BG, 0, 176, b"SAMPLE LAB");
        }
        let full = self.tab.full(self.slot);
        self.rect(182, 4, 52, 32, if full { PINK } else { BOX });
        let (sl, sln) = hook::label(self.slot);
        let sl = &sl[..sln];
        // too small to have its knob's name beside it: the box says the knob (ALGO), then the slot, over and over
        let shown: &[u8] = if self.blink(self.algo_at) { b"ALGO" } else { sl };
        let sw = text_w(shown, 2);
        self.text((208 - sw / 2) as i32, 9, 2, 3, INK, 182, 234, shown);
        // four names
        let top = self.top();
        for r in 0..if trim { 0 } else { ROWS } {
            let i = top + r;
            let y = Self::ROW_Y + r as u32 * Self::ROW_P;
            let Some(e) = self.ent(i).copied() else { continue };
            let on = i == self.sel;
            if on {
                self.rect(8, y, 218, Self::ROW_P - 4, if self.play.on { GREEN } else { PINK });
            }
            let c = INK; // (all text white but the title: the user)
            let mut nm = [0u8; NAME + 1];
            let mut l = e.len as usize;
            nm[..l].copy_from_slice(&e.name[..l]);
            if e.dir && i >= self.vrow() {
                nm[l] = b'/';
                l += 1;
            }
            let room = Self::NAME_R - Self::NAME_L;
            if on {
                let off = self.scroll_of().map_or(0, |s| s.0);
                self.text(Self::NAME_L as i32 - off as i32, y + 5, 2, 4, c, Self::NAME_L, Self::NAME_R, &nm[..l]);
            } else {
                let mut out = [0u8; NAME + 3];
                let k = fit(&nm[..l], room, &mut out);
                self.text(Self::NAME_L as i32, y + 5, 2, 4, c, Self::NAME_L, Self::NAME_R, &out[..k]);
            }
        }
        if self.n > ROWS && !trim {
            let (y0, h0) = (Self::ROW_Y, ROWS as u32 * Self::ROW_P - 4);
            self.rect(230, y0, 4, h0, BOX);
            let h = (h0 * ROWS as u32 / self.n as u32).max(8);
            self.rect(230, y0 + (h0 - h) * top as u32 / (self.n - ROWS) as u32, 4, h, PINK);
        }
        if self.n == 0 && self.msg[0] == 0 && !trim {
            self.text(16, Self::ROW_Y + 5, 2, 4, INK, 16, 226, b"NO SAMPLES");
        }
        // the bottom: sending, a message, or the buttons
        let y = 217;
        if !sending {
            self.rect(0, 213, 240, 27, INK); // the bottom banner: white, the buttons (or a message) black on it
        }
        if sending {
            self.rect(8, 216, 226 * self.pct / 100, 24, PINK);
            let mut t = *b"SENDING 000%";
            t[8] = if self.pct >= 100 { b'1' } else { b' ' };
            t[9] = b'0' + (self.pct / 10 % 10) as u8;
            t[10] = b'0' + (self.pct % 10) as u8;
            self.text(12, y, 2, 3, INK, 8, 234, &t);
        } else if self.msg[0] != 0 {
            let m = self.msg;
            let n = m.iter().position(|&b| b == 0).unwrap_or(40);
            let sx = if text_w(&m[..n], 2) <= 224 { 2 } else { 1 };
            let w = text_w(&m[..n], sx);
            self.text((120 - w.min(232) / 2) as i32, y, sx, 3, BG, 4, 236, &m[..n]);
        } else if self.can_delete() && self.now / 2000 % 2 == 1 {
            // a take (RECORDED): every other 2 s the bar says EDIT deletes it
            let t: &[u8] = b"EDIT: DELETE";
            self.text((120 - text_w(t, 2) / 2) as i32, y, 2, 3, BG, 0, 240, t);
        } else if trim {
            self.text(8, y, 2, 3, BG, 0, 240, b"\x03HOME");
            let w = text_w(b"REC\x02", 2);
            self.text((232 - w) as i32, y, 2, 3, BG, 0, 240, b"REC\x02");
            let w2 = text_w(b"PLAY\x01", 2);
            self.text((232 - w - 10 - w2) as i32, y, 2, 3, BG, 0, 240, b"PLAY\x01");
        } else {
            // each button's word, then its symbol
            // HOME: out of the folder, or (at the top) back to the mod
            self.text(8, y, 2, 3, BG, 0, 240, b"\x03HOME");
            let w = text_w(b"REC\x02", 2);
            self.text((232 - w) as i32, y, 2, 3, BG, 0, 240, b"REC\x02");
            let play: &[u8] = if self.ent(self.sel).is_some_and(|e| e.dir) { b"OPEN\x01" } else { b"PLAY\x01" };
            let w2 = text_w(play, 2);
            self.text((232 - w - 10 - w2) as i32, y, 2, 3, BG, 0, 240, play);
        }
        self.pending = true;
        self.row = 0;
    }

    /// The trim page: the length (the title), the waveform with START [ and END ], the knobs' names.
    fn layout_trim(&mut self) {
        let (a, b) = (self.ta, self.tb);
        let cut = self.cut();
        let over = cut < b;
        // the length, in seconds
        let cs = (b - a) as u64 * 100 / self.fm.rate.max(1) as u64;
        let mut w = Buf { b: [0; 40], n: 0 };
        let _ = if cs < 10_000 {
            core::fmt::Write::write_fmt(&mut w, format_args!("{}.{:02} S", cs / 100, cs % 100))
        } else {
            core::fmt::Write::write_fmt(&mut w, format_args!("{}.{} S", cs / 100, cs / 10 % 10))
        };
        let t = w.b;
        self.text(8, 6, 3, 4, BG, 0, 176, &t[..w.n]);
        // the waveform: the trim (up to the cut) pink
        let v = self.wv[if self.zoom > 0 { 1 } else { 0 }];
        let lim = |c: i32| c.clamp(-1, GW as i32) as i16;
        let ca = lim(v.col(a));
        let cb = lim(v.col(b.min(cut).max(a + 1) - 1));
        if self.nops < self.ops.len() {
            self.ops[self.nops] = Op::Wave { y: WY as u8, h: WH as u8, a: ca, b: cb };
            self.nops += 1;
        }
        // the brackets: START [ and END ] (the knob last turned bold)
        for (k, f) in [(0u8, a), (1, b - 1)] {
            let c = v.col(f);
            if !(0..GW as i32).contains(&c) {
                continue;
            }
            let c = c as u32;
            let bw = if self.hot == k { 3 } else { 1 };
            self.rect(WX + c - bw / 2, WY - 4, bw, WH + 8, INK);
            let fx = if k == 1 { WX + c - 4 } else { WX + c + 1 };
            self.rect(fx, WY - 4, 4, 2, INK);
            self.rect(fx, WY + WH + 2, 4, 2, INK);
        }
        // where the slot's memory runs out: a pink line, FULL beside it
        if over {
            let c = v.col(cut);
            if (0..GW as i32).contains(&c) {
                let c = c as u32;
                self.rect(WX + c, WY, 2, WH, PINK);
                let fw = text_w(b"FULL", 2);
                let fx = if c + 6 + fw <= GW as u32 { WX + c + 6 } else { WX + c - 4 - fw };
                self.rect(fx - 2, WY + 2, fw + 4, 18, BG);
                self.text(fx as i32, WY + 4, 2, 2, INK, 0, 240, b"FULL");
            }
        }
        // the play head
        if self.play.on {
            let c = v.col(self.play.from + (self.play.pos >> 32) as u32);
            if (0..GW as i32).contains(&c) {
                // under the wave, so playing redraws a few rows, not the whole wave
                self.rect((WX + c as u32).saturating_sub(1), WY + WH + 4, 3, 4, GREEN);
            }
        }
        // over the knobs, what they do: START, ZOOM, POS, END
        let y = WY + WH + 10;
        // (as near over their knobs as they fit)
        for (x, t) in [(4, &b"START"[..]), (78, b"ZOOM"), (142, b"POS"), (190, b"END")] {
            self.text(x, y, 2, 3, INK, 0, 240, t);
        }
    }

    /// Row `y` of the page, as the panel takes it (RGB565, high byte first).
    fn draw_row(&self, y: u32, line: &mut [u16; 240]) {
        line.fill(PAL[BG as usize]);
        for op in &self.ops[..self.nops] {
            match *op {
                Op::Rect { x, y: ry, w, h, c } => {
                    if y >= ry as u32 && y < ry as u32 + h as u32 {
                        line[x as usize..(x as usize + w as usize).min(240)].fill(PAL[c as usize]);
                    }
                }
                Op::Text { x, y: ty, sx, sy, c, cl, cr, s, n } => {
                    let (sx, sy, ty) = (sx as i32, sy as u32, ty as u32);
                    if y < ty || y >= ty + 7 * sy {
                        continue;
                    }
                    let r = ((y - ty) / sy) as usize;
                    let (cl, cr) = (cl as i32, cr as i32);
                    let mut cx = x as i32;
                    for &ch in &self.pool[s as usize..s as usize + n as usize] {
                        if cx >= cr {
                            break;
                        }
                        let (gw, rows) = glyph(ch);
                        for k in 0..gw as i32 {
                            if rows[r] >> (gw as i32 - 1 - k) & 1 != 0 {
                                for a in cx + k * sx..cx + k * sx + sx {
                                    if a >= cl && a < cr {
                                        line[a as usize] = PAL[c as usize];
                                    }
                                }
                            }
                        }
                        cx += gw as i32 * sx + sx;
                    }
                }
                Op::Wave { y: wy, h, a, b } => {
                    let (wy, h) = (wy as u32, h as u32);
                    if y < wy || y >= wy + h {
                        continue;
                    }
                    let v = &self.wv[if self.zoom > 0 { 1 } else { 0 }];
                    let (mid, yy, top) = ((wy + h / 2) as i32, y as i32, self.pkmax.max(24) as i32);
                    for x in 0..GW {
                        let h2 = (v.pk[x] as i32 * h as i32 / (2 * top)).clamp(1, h as i32 / 2);
                        if yy >= mid - h2 && yy < mid + h2 {
                            let c = if (a as i32..=b as i32).contains(&(x as i32)) { INK } else { DIM }; // the trim white, the rest grey
                            line[WX as usize + x] = PAL[c as usize];
                        }
                    }
                }
            }
        }
    }

    /// Changed rows as the link's spans (docs/LINK_PROTOCOL.md 'L', the same coding as the glue's teensy_spans).
    #[inline(never)]
    pub fn spans(&mut self, out: &mut [u8]) -> usize {
        if !self.pending {
            return 0;
        }
        let mut len = 0;
        let mut line = [0u16; 240];
        while self.row < 240 {
            let y = self.row;
            self.draw_row(y as u32, &mut line);
            let h = line.iter().fold(0x811C_9DC5u32, |h, &p| (h ^ p as u32).wrapping_mul(0x0100_0193));
            if h == self.sent[y] {
                self.row += 1;
                continue;
            }
            if len + 3 + 240 * 3 > out.len() {
                return len;
            }
            let o = &mut out[len..];
            o[0] = y as u8;
            o[1] = 0;
            o[2] = 240;
            let mut k = 3;
            let mut i = 0;
            while i < 240 {
                let mut r = 1;
                while i + r < 240 && r < 128 && line[i + r] == line[i] {
                    r += 1;
                }
                if r >= 2 {
                    o[k] = 0x80 | (r - 1) as u8;
                    o[k + 1] = (line[i] >> 8) as u8;
                    o[k + 2] = line[i] as u8;
                    k += 3;
                    i += r;
                    continue;
                }
                let mut l = 1;
                while i + l < 240 && l < 128 && !(i + l + 1 < 240 && line[i + l] == line[i + l + 1]) {
                    l += 1;
                }
                o[k] = (l - 1) as u8;
                k += 1;
                for p in &line[i..i + l] {
                    o[k] = (p >> 8) as u8;
                    o[k + 1] = *p as u8;
                    k += 2;
                }
                i += l;
            }
            len += k;
            self.sent[y] = h;
            self.row += 1;
        }
        self.pending = false;
        len
    }

    /// Row `y` as bytes (the 'S' screen dump).
    #[inline(never)]
    pub fn row_bytes(&self, y: u32, out: &mut [u8; 480]) {
        let mut line = [0u16; 240];
        self.draw_row(y, &mut line);
        for (i, p) in line.iter().enumerate() {
            out[2 * i] = (p >> 8) as u8;
            out[2 * i + 1] = *p as u8;
        }
    }

    /// One line for 'J': where it is and what it shows.
    #[inline(never)]
    pub fn status(&self, out: &mut impl core::fmt::Write) {
        let _ = write!(out, "LAB open={} depth={} n={} sel={} slot={} slots={} send={} pct={} play={} fmt={}:{}ch/{}bit/{}Hz/{}fr msg=",
            self.open, self.depth, self.n, self.sel, self.slot, self.tab.n, self.send as u8, self.pct, self.play.on,
            self.fm.ok, self.fm.ch, self.fm.bits, self.fm.rate, self.fm.frames);
        for &c in self.msg.iter().take_while(|&&c| c != 0) {
            let _ = out.write_char(c as char);
        }
        let _ = write!(out, " page={} trim={}..{} of={} zoom={} hot={} view={}+{} scan={}/{}",
            self.page == Page::Trim, self.ta, self.tb, self.t_of as i32, self.zoom, self.hot, self.wv[1].x0, self.wv[1].w,
            self.wv[0].done(), self.wv[1].done());
        let _ = write!(out, " starved={} saves_gap={}ms", self.starved, self.gap);
        if !RECORDER {
            let _ = write!(out, "\r\n");
            return;
        }
        let _ = write!(out, " psram_ring={}", audio_in::ring());
        let _ = write!(out, " rec: page={} codec={} state={} sound={} gain={} trig={} written={} len={} drop={} overruns={} meter={} name=",
            self.page == Page::Rec, self.codec, self.rst as u8, self.r_sound, self.r_gain, self.r_trig, audio_in::written(),
            self.r_len, self.r_drop, audio_in::overruns(), self.r_meter);
        for &c in &self.rname[..self.rn] {
            let _ = out.write_char(c as char);
        }
        if self.page == Page::Name {
            let _ = write!(out, " editing=");
            for &c in &self.nb {
                let _ = out.write_char(c as char);
            }
            let _ = write!(out, " cur={}", self.ncur);
        }
        let _ = write!(out, " sel_name=");
        if let Some(e) = self.ent(self.sel) {
            for &c in &e.name[..e.len as usize] {
                let _ = out.write_char(c as char);
            }
        }
        let _ = write!(out, "\r\n");
    }
}



// ---- the recorder: line in -> SAMPLES/RECORDED/AUDIOnnn.WAV (44.1 kHz mono 16-bit), then its name, then the trim page
impl Lab {
    /// RECORD on the list: the recorder's page, the line in on (and heard: the monitor, at MASTER).
    fn rec_open(&mut self, vm: &Vm) {
        self.stop_now(Some(vm));
        self.page = Page::Rec;
        self.rst = Rst::Idle;
        self.msg = [0; 40];
        self.wv[0] = Wave::NEW;
        self.zoom = 0;
        self.pkmax = 255;
        self.codec = audio_in::start(self.ring.as_mut_ptr());
        if self.codec {
            audio_in::set_gain(self.r_gain);
        } else {
            self.say(PINK, &[b"NO AUDIO IN"]);
        }
        let w = audio_in::written();
        self.r_seen = w;
        self.mon = w.wrapping_sub(1024);
        self.r_meter = 0;
        self.r_pk = 0;
        self.r_cn = 0;
        self.r_cpk = 0;
        self.dirty = true;
    }

    /// SAMPLES/RECORDED (made the first time): the recorder's page is only at the top, so `dir` is SAMPLES.
    fn rec_dir(&self, vm: &Vm) -> Option<RawDirectory> {
        let s = self.dir?;
        if vm.open_dir(s, REC_DIR).map(|d| vm.close_dir(d)).is_err() {
            let _ = vm.make_dir_in_dir(s, REC_DIR);
        }
        vm.open_dir(s, REC_DIR).ok()
    }

    /// A new take's file: the next free AUDIOnnn.WAV, its header (the lengths written at the end).
    fn rec_new(&mut self, vm: &Vm) -> bool {
        let Some(d) = self.rec_dir(vm) else {
            self.say(PINK, &[b"CARD ERROR"]);
            return false;
        };
        let mut top = 0u32;
        let _ = vm.iterate_dir(d, |e| {
            let b = e.name.base_name();
            if b.len() == 8 && &b[..5] == b"AUDIO" && b[5..].iter().all(|c| c.is_ascii_digit()) {
                top = top.max(b[5..].iter().fold(0, |v, &c| v * 10 + (c - b'0') as u32));
            }
            ControlFlow::Continue(())
        });
        if top >= 999 {
            let _ = vm.close_dir(d);
            self.say(PINK, &[b"AUDIO999 IS THE LAST"]);
            return false;
        }
        let k = top + 1;
        let mut nm = *b"AUDIO000.WAV";
        nm[5] = b'0' + (k / 100) as u8;
        nm[6] = b'0' + (k / 10 % 10) as u8;
        nm[7] = b'0' + (k % 10) as u8;
        let f = vm.open_file_in_dir(d, core::str::from_utf8(&nm).unwrap_or("AUDIO000.WAV"), Mode::ReadWriteCreate);
        let _ = vm.close_dir(d);
        let Ok(f) = f else {
            self.say(PINK, &[b"CAN'T MAKE THE FILE"]);
            return false;
        };
        if vm.write(f, &wav_header(0x0FFF_0000)).is_err() {
            let _ = vm.close_file(f);
            self.say(PINK, &[b"CARD ERROR"]);
            return false;
        }
        self.rf = Some(f);
        self.rname.copy_from_slice(&nm[..8]);
        self.rn = 8;
        true
    }

    /// `n` samples from the ring into the take's file (2048 at most).
    fn rec_put(&mut self, vm: &Vm, n: u32) -> bool {
        let Some(f) = self.rf else { return false };
        let n = n.min(2048);
        let mut buf = [0u8; 4096];
        for j in 0..n {
            let v = audio_in::at(self.r_at.wrapping_add(j)).to_le_bytes();
            buf[2 * j as usize] = v[0];
            buf[2 * j as usize + 1] = v[1];
        }
        let ok = vm.write(f, &buf[..2 * n as usize]).is_ok();
        self.r_at = self.r_at.wrapping_add(n);
        self.r_len += n;
        ok
    }

    /// The main loop's turn on the recorder's page: what came in (meter, the start on sound, the picture), and the
    /// take to the card, 2048 samples (46 ms) at a time.
    fn rec_step(&mut self, vm: &Vm) {
        if self.rst == Rst::Saving {
            return self.rec_save_step(vm);
        }
        if !self.codec {
            return;
        }
        let w = audio_in::written();
        let keep = audio_in::ring().saturating_sub(512); // (the oldest still safe to read)
        if w.wrapping_sub(self.r_seen) > keep {
            self.r_seen = w.wrapping_sub(keep);
        }
        let thr = trig_amp(self.r_trig);
        let mut n = 0;
        while self.r_seen != w && n < 8192 {
            let s = audio_in::at(self.r_seen).unsigned_abs() as u32;
            self.r_pk = self.r_pk.max(s);
            if self.rst == Rst::Armed && s >= thr {
                // sound: the take starts, 50 ms before it (so the attack is all there)
                let back = 2205.min(keep - w.wrapping_sub(self.r_seen));
                self.r_at = self.r_seen.wrapping_sub(back);
                self.rst = Rst::On;
                self.dirty = true;
            }
            // the picture: a column every 1/30 s, scrolling left
            self.r_cpk = self.r_cpk.max((s >> 7).min(255) as u8);
            self.r_cn += 1;
            if self.r_cn >= 1470 {
                let pk = &mut self.wv[0].pk;
                pk.copy_within(1.., 0);
                pk[GW - 1] = self.r_cpk;
                self.r_cn = 0;
                self.r_cpk = 0;
            }
            self.r_seen = self.r_seen.wrapping_add(1);
            n += 1;
        }
        if self.rst != Rst::On {
            return;
        }
        let lag = w.wrapping_sub(self.r_at);
        if audio_in::big() {
            // PSRAM: the whole take stays there, the card only after it (a slow card can't cut it)
            if lag >= REC_MAX.min(keep) {
                self.rec_stop(vm);
            }
            return;
        }
        if lag > keep {
            // the card fell more than the ring behind: a gap (counted, said at the end)
            self.r_drop += lag - audio_in::ring() / 2;
            self.r_at = w.wrapping_sub(audio_in::ring() / 2);
        }
        if w.wrapping_sub(self.r_at) >= 2048 && !self.rec_put(vm, 2048) {
            self.rec_stop(vm);
            self.say(PINK, &[b"CARD FULL OR ERROR"]);
            return;
        }
        if self.r_len >= REC_MAX {
            self.rec_stop(vm);
        }
    }

    /// Saving a take from PSRAM: 16 KB a turn (the FM-1's link between), then its name.
    fn rec_save_step(&mut self, vm: &Vm) {
        for _ in 0..4 {
            let left = self.r_end.wrapping_sub(self.r_at);
            if left == 0 {
                return self.rec_finish(vm);
            }
            if !self.rec_put(vm, left) {
                self.rec_finish(vm);
                return self.say(PINK, &[b"CARD FULL OR ERROR"]);
            }
        }
        let left = self.r_end.wrapping_sub(self.r_at);
        self.pct = (self.r_len as u64 * 100 / (self.r_len + left).max(1) as u64) as u32;
        self.dirty = true;
    }

    /// The take ends: (PSRAM) saved from there a piece a turn, or the rest of it to the card now; then its name.
    fn rec_stop(&mut self, vm: &Vm) {
        let w = audio_in::written();
        audio_in::stop();
        if audio_in::big() {
            self.r_end = w;
            self.rst = Rst::Saving;
            self.pct = 0;
            self.dirty = true;
            return;
        }
        for _ in 0..8 {
            let left = w.wrapping_sub(self.r_at);
            if left == 0 || left > audio_in::ring() || !self.rec_put(vm, left) {
                break;
            }
        }
        self.rec_finish(vm);
    }

    /// The take is on the card: the lengths into its header, then its name (too short: gone).
    fn rec_finish(&mut self, vm: &Vm) {
        self.rst = Rst::Idle;
        let Some(f) = self.rf.take() else { return };
        let _ = vm.file_seek_from_start(f, 0);
        let _ = vm.write(f, &wav_header(self.r_len));
        let _ = vm.close_file(f);
        if self.r_len < 441 {
            self.rec_delete(vm);
            self.codec = audio_in::start(self.ring.as_mut_ptr());
            self.say(PINK, &[b"TOO SHORT"]);
            return;
        }
        // the name page: the default name, PLAY keeps it
        self.page = Page::Name;
        self.nb = [b' '; NAME_N];
        self.nb[..self.rn].copy_from_slice(&self.rname[..self.rn]);
        self.ncur = 0;
        self.msg = [0; 40];
        if self.r_drop > 0 {
            let mut b = Buf { b: [0; 40], n: 0 };
            let _ = core::fmt::Write::write_fmt(&mut b, format_args!("CARD TOO SLOW: LOST {} MS", self.r_drop / 44));
            let m = b.b;
            self.say(PINK, &[&m[..b.n]]);
        }
        self.dirty = true;
    }

    /// The take's file, deleted (armed but nothing came, or too short).
    fn rec_delete(&mut self, vm: &Vm) {
        if let Some(f) = self.rf.take() {
            let _ = vm.close_file(f);
        }
        if let Some(d) = self.rec_dir(vm) {
            let mut nm = [0u8; 12];
            nm[..self.rn].copy_from_slice(&self.rname[..self.rn]);
            nm[self.rn..self.rn + 4].copy_from_slice(b".WAV");
            let _ = vm.delete_entry_in_dir(d, core::str::from_utf8(&nm[..self.rn + 4]).unwrap_or("?"));
            let _ = vm.close_dir(d);
        }
        self.rn = 0;
    }

    /// The Lab closes in the middle: a take is kept (its header written), an empty one deleted, a rename undone.
    fn rec_abort(&mut self, vm: &Vm) {
        match self.rst {
            Rst::On | Rst::Saving => {
                if self.rst == Rst::On {
                    self.rec_stop(vm);
                }
                while self.rst == Rst::Saving {
                    self.rec_save_step(vm);
                }
            }
            Rst::Armed => self.rec_delete(vm),
            Rst::Idle => {}
        }
        self.rst = Rst::Idle;
        if let Some((a, b, _, _)) = self.cp.take() {
            let _ = vm.close_file(a);
            let _ = vm.close_file(b);
            if let Some(d) = self.dir {
                if let Ok(rd) = vm.open_dir(d, REC_DIR) {
                    let nm = self.name_now();
                    let _ = vm.delete_entry_in_dir(rd, core::str::from_utf8(&nm.0[..nm.1]).unwrap_or("?"));
                    let _ = vm.close_dir(rd);
                }
            }
        }
        if matches!(self.page, Page::Rec | Page::Name) {
            self.page = Page::List;
        }
    }

    fn rec_buttons(&mut self, vm: &Vm, down: u32) {
        if self.rst == Rst::Saving {
            return;
        }
        if down & HOME != 0 {
            match self.rst {
                Rst::On => self.rec_stop(vm),
                Rst::Armed => {
                    self.rec_delete(vm);
                    self.rst = Rst::Idle;
                }
                Rst::Saving => {}
                Rst::Idle => {
                    audio_in::stop();
                    self.page = Page::List;
                    self.msg = [0; 40];
                }
            }
            self.dirty = true;
            return;
        }
        if down & REC != 0 {
            match self.rst {
                Rst::Idle => {
                    if !self.codec {
                        self.say(PINK, &[b"NO AUDIO IN"]);
                        return;
                    }
                    if !self.rec_new(vm) {
                        return;
                    }
                    self.r_len = 0;
                    self.r_drop = 0;
                    self.r_ovr = audio_in::overruns();
                    self.msg = [0; 40];
                    if self.r_sound {
                        self.rst = Rst::Armed;
                    } else {
                        self.rst = Rst::On;
                        self.r_at = audio_in::written();
                    }
                }
                Rst::Armed => {
                    self.rec_delete(vm);
                    self.rst = Rst::Idle;
                }
                Rst::On => self.rec_stop(vm),
                Rst::Saving => {}
            }
            self.dirty = true;
        }
    }

    /// MODE (K1): starts on REC / on sound; GAIN (K2): the line in level; TRIG (K3): how loud starts it.
    fn rec_knob(&mut self, role: u32, n: i32) {
        match role {
            K1 if self.rst == Rst::Idle => {
                self.r_sound = n > 0;
                self.say(INK, &[if self.r_sound { b"START ON SOUND" } else { b"START ON REC" }]);
            }
            K2 => {
                self.r_gain = (self.r_gain as i32 + n.signum()).clamp(0, 15) as u8;
                if self.codec {
                    audio_in::set_gain(self.r_gain);
                }
                let mut b = Buf { b: [0; 40], n: 0 };
                let _ = core::fmt::Write::write_fmt(&mut b, format_args!("GAIN {}", self.r_gain));
                let m = b.b;
                self.say(INK, &[&m[..b.n]]);
            }
            K3 if self.r_sound => {
                self.r_trig = (self.r_trig as i32 + n.signum()).clamp(0, TRIG_N as i32 - 1) as u8;
                let mut b = Buf { b: [0; 40], n: 0 };
                let _ = core::fmt::Write::write_fmt(&mut b, format_args!("START AT {} DB", -54 + 3 * self.r_trig as i32));
                let m = b.b;
                self.say(INK, &[&m[..b.n]]);
            }
            _ => {}
        }
        self.dirty = true;
    }

    /// The recorder's page: what it is doing (the title), the line in as it comes (a picture ~7 s wide, white while
    /// it records), the meter (and the TRIG line), the knobs' names.
    fn layout_rec(&mut self) {
        self.rect(0, 0, 240, 42, PINK);
        let mut t = Buf { b: [0; 40], n: 0 };
        let _ = match self.rst {
            Rst::Idle => core::fmt::Write::write_str(&mut t, "RECORD"),
            Rst::Armed => core::fmt::Write::write_str(&mut t, if self.now / 500 % 2 == 0 { "WAITING" } else { "" }),
            Rst::Saving => core::fmt::Write::write_str(&mut t, "SAVING"),
            Rst::On => {
                let cs = (self.r_len + audio_in::written().wrapping_sub(self.r_at).min(audio_in::ring())) / 441;
                core::fmt::Write::write_fmt(&mut t, format_args!("REC {}.{} S", cs / 100, cs / 10 % 10))
            }
        };
        let tb = t.b;
        self.text(8, 6, 2, 4, BG, 0, 240, &tb[..t.n]);
        // how it starts, and the level
        let mode: &[u8] = if self.r_sound { b"START ON SOUND" } else { b"START ON REC" };
        self.text(8, 50, 2, 2, INK, 0, 240, mode);
        // the picture
        let (a, b) = if self.rst == Rst::On { (0, GW as i16 - 1) } else { (-1, -1) };
        if self.nops < self.ops.len() {
            self.ops[self.nops] = Op::Wave { y: 70, h: 80, a, b };
            self.nops += 1;
        }
        // the meter: -60 .. 0 dB across, pink in the top 6 dB; the TRIG line when it starts on sound
        self.r_meter = self.r_pk.max(self.r_meter * 3 / 4);
        self.r_pk = 0;
        self.rect(8, 158, 224, 14, BOX);
        let px = db_px(self.r_meter);
        self.rect(8, 158, px, 14, if px >= 202 { PINK } else { GREEN });
        if self.r_sound {
            let x = db_px(trig_amp(self.r_trig));
            self.rect(8 + x.saturating_sub(1), 154, 3, 22, INK);
        }
        let y = 182;
        self.text(4, y, 2, 3, INK, 0, 240, b"MODE");
        self.text(78, y, 2, 3, INK, 0, 240, b"GAIN");
        if self.r_sound {
            self.text(142, y, 2, 3, INK, 0, 240, b"TRIG");
        }
        if self.rst == Rst::Saving {
            self.rect(0, 213, 240, 27, BG);
            self.rect(8, 216, 226 * self.pct.min(100) / 100, 24, PINK);
            let mut t = *b"SAVING 000%";
            t[7] = if self.pct >= 100 { b'1' } else { b' ' };
            t[8] = b'0' + (self.pct / 10 % 10) as u8;
            t[9] = b'0' + (self.pct % 10) as u8;
            self.text(12, 217, 2, 3, INK, 8, 234, &t);
            self.pending = true;
            self.row = 0;
            return;
        }
        self.bottom(if self.rst == Rst::Idle { b"REC\x06" } else { b"REC\x07" }, None);
    }

    /// The white bottom banner: a message, or HOME on the left and the right one(s).
    fn bottom(&mut self, right: &[u8], mid: Option<&[u8]>) {
        let y = 217;
        self.rect(0, 213, 240, 27, INK);
        if self.msg[0] != 0 {
            let m = self.msg;
            let n = m.iter().position(|&b| b == 0).unwrap_or(40);
            let sx = if text_w(&m[..n], 2) <= 224 { 2 } else { 1 };
            let w = text_w(&m[..n], sx);
            self.text((120 - w.min(232) / 2) as i32, y, sx, 3, BG, 4, 236, &m[..n]);
        } else {
            self.text(8, y, 2, 3, BG, 0, 240, b"\x03HOME");
            let w = text_w(right, 2);
            self.text((232 - w) as i32, y, 2, 3, BG, 0, 240, right);
            if let Some(m) = mid {
                let w2 = text_w(m, 2);
                self.text((232 - w - 10 - w2) as i32, y, 2, 3, BG, 0, 240, m);
            }
        }
        self.pending = true;
        self.row = 0;
    }

    // ---- the take's name: MOVE (K1) picks the letter, LETTER (K2) changes it, PLAY saves it, HOME keeps AUDIOnnn ----
    fn name_knob(&mut self, role: u32, n: i32) {
        if self.cp.is_some() {
            return;
        }
        match role {
            K1 => self.ncur = (self.ncur as i32 + n.signum()).clamp(0, NAME_N as i32 - 1) as usize,
            K2 => {
                let i = NAME_CH.iter().position(|&c| c == self.nb[self.ncur]).unwrap_or(0) as i32;
                self.nb[self.ncur] = NAME_CH[(i + n).rem_euclid(NAME_CH.len() as i32) as usize];
            }
            _ => return,
        }
        self.msg = [0; 40];
        self.dirty = true;
    }

    /// The name being edited, as the card will have it: the letters (spaces dropped) + ".WAV".
    fn name_now(&self) -> ([u8; 12], usize) {
        let mut nm = [0u8; 12];
        let mut k = 0;
        for &c in self.nb.iter().filter(|&&c| c != b' ') {
            nm[k] = c;
            k += 1;
        }
        if k > 0 {
            nm[k..k + 4].copy_from_slice(b".WAV");
            k += 4;
        }
        (nm, k)
    }

    fn name_buttons(&mut self, vm: &Vm, vol: RawVolume, down: u32) {
        if self.cp.is_some() {
            return;
        }
        if down & HOME != 0 {
            return self.name_done(vm, vol); // (the name it has)
        }
        if down & PLAY == 0 {
            return;
        }
        let (nm, k) = self.name_now();
        if k == 0 {
            return self.say(PINK, &[b"NAME IS EMPTY"]);
        }
        if nm[..k - 4] == self.rname[..self.rn] {
            return self.name_done(vm, vol);
        }
        let name = core::str::from_utf8(&nm[..k]).unwrap_or("?");
        let mut old = [0u8; 12];
        old[..self.rn].copy_from_slice(&self.rname[..self.rn]);
        old[self.rn..self.rn + 4].copy_from_slice(b".WAV");
        let Some(d) = self.rec_dir(vm) else { return self.say(PINK, &[b"CARD ERROR"]) };
        // the card can't rename: a copy under the new name, then the old one deleted
        let taken = vm.open_file_in_dir(d, name, Mode::ReadOnly).map(|f| vm.close_file(f)).is_ok();
        let r = if taken {
            Err("NAME TAKEN")
        } else {
            match vm.open_file_in_dir(d, core::str::from_utf8(&old[..self.rn + 4]).unwrap_or("?"), Mode::ReadOnly) {
                Ok(a) => match vm.open_file_in_dir(d, name, Mode::ReadWriteCreate) {
                    Ok(b) => {
                        let n = vm.file_length(a).unwrap_or(0);
                        self.cp = Some((a, b, 0, n));
                        Ok(())
                    }
                    Err(_) => {
                        let _ = vm.close_file(a);
                        Err("BAD NAME")
                    }
                },
                Err(_) => Err("CARD ERROR"),
            }
        };
        let _ = vm.close_dir(d);
        if let Err(why) = r {
            self.say(PINK, &[why.as_bytes()]);
        }
        self.pct = 0;
        self.dirty = true;
    }

    /// The rename's copy: 16 KB a turn.
    fn name_step(&mut self, vm: &Vm, vol: RawVolume) {
        let Some((a, b, mut done, of)) = self.cp else { return };
        let mut buf = [0u8; 4096];
        let mut end = false;
        for _ in 0..4 {
            let got = vm.read(a, &mut buf).unwrap_or(0);
            if got == 0 || vm.write(b, &buf[..got]).is_err() {
                end = true;
                break;
            }
            done += got as u32;
        }
        self.pct = (done as u64 * 100 / of.max(1) as u64) as u32;
        self.dirty = true;
        if !end && done < of {
            self.cp = Some((a, b, done, of));
            return;
        }
        self.cp = None;
        let _ = vm.close_file(a);
        let _ = vm.close_file(b);
        if done < of {
            // the copy failed: the new one goes, the take keeps its name
            if let Some(d) = self.rec_dir(vm) {
                let (nm, k) = self.name_now();
                let _ = vm.delete_entry_in_dir(d, core::str::from_utf8(&nm[..k]).unwrap_or("?"));
                let _ = vm.close_dir(d);
            }
            return self.say(PINK, &[b"CARD FULL OR ERROR"]);
        }
        self.rec_delete(vm);
        let (nm, k) = self.name_now();
        self.rname = [0; NAME_N];
        self.rname[..k - 4].copy_from_slice(&nm[..k - 4]);
        self.rn = k - 4;
        self.name_done(vm, vol);
    }

    /// Named: the RECORDED folder, the take selected, its trim page (PLAY hears it, REC sends it to the slot).
    fn name_done(&mut self, vm: &Vm, vol: RawVolume) {
        let mut nm = [0u8; 12];
        nm[..self.rn].copy_from_slice(&self.rname[..self.rn]);
        nm[self.rn..self.rn + 4].copy_from_slice(b".WAV");
        let (n0, nn) = (nm, self.rn);
        self.page = Page::List;
        let (Ok(dir), Ok(file)) = (
            ShortFileName::create_from_str(REC_DIR),
            ShortFileName::create_from_str(core::str::from_utf8(&nm[..self.rn + 4]).unwrap_or("?")),
        ) else {
            return;
        };
        self.path[0] = dir;
        self.depth = 1;
        self.enter(Some((vm, vol)));
        self.sel = self.index_of(vm, file).unwrap_or(0);
        self.fill(vm);
        self.sel_at = self.now;
        self.moved_at = self.now;
        self.fm_of = usize::MAX;
        self.t_of = usize::MAX;
        self.trim_open(vm);
        self.say(GREEN, &[b"SAVED: ", &n0[..nn]]);
    }

    /// The name page: the 8 letters big, the one MOVE is on pink; MOVE and LETTER over their knobs.
    fn layout_name(&mut self) {
        self.rect(0, 0, 240, 42, PINK);
        self.text(8, 6, 2, 4, BG, 0, 240, b"NAME IT");
        for i in 0..NAME_N {
            let x = 8 + 28 * i as u32;
            if i == self.ncur {
                self.rect(x - 2, 82, 28, 56, PINK);
            }
            let c = self.nb[i];
            if c == b' ' {
                self.text(x as i32, 118, 4, 2, if i == self.ncur { INK } else { DIM }, 0, 240, b"_");
            } else {
                self.text(x as i32, 89, 4, 6, INK, 0, 240, &[c]);
            }
        }
        self.text(8, 146, 2, 2, DIM, 0, 240, b"ON THE CARD AS .WAV");
        self.text(4, 182, 2, 3, INK, 0, 240, b"MOVE");
        self.text(78, 182, 2, 3, INK, 0, 240, b"LETTER");
        if self.cp.is_some() {
            self.rect(0, 213, 240, 27, BG);
            self.rect(8, 216, 226 * self.pct.min(100) / 100, 24, PINK);
            let mut t = *b"SAVING 000%";
            t[7] = if self.pct >= 100 { b'1' } else { b' ' };
            t[8] = b'0' + (self.pct / 10 % 10) as u8;
            t[9] = b'0' + (self.pct % 10) as u8;
            self.text(12, 217, 2, 3, INK, 8, 234, &t);
            self.pending = true;
            self.row = 0;
            return;
        }
        let right: &[u8] = if self.now / 2000 % 2 == 1 { b"SAVE\x01" } else { b"PLAY\x01" };
        self.bottom(right, None);
    }
}

/// A WAV header: 44.1 kHz, mono, 16-bit, `n` samples.
fn wav_header(n: u32) -> [u8; 44] {
    let mut h = [0u8; 44];
    let d = n.saturating_mul(2);
    h[0..4].copy_from_slice(b"RIFF");
    h[4..8].copy_from_slice(&d.saturating_add(36).to_le_bytes());
    h[8..16].copy_from_slice(b"WAVEfmt ");
    h[16..20].copy_from_slice(&16u32.to_le_bytes());
    h[20..22].copy_from_slice(&1u16.to_le_bytes());
    h[22..24].copy_from_slice(&1u16.to_le_bytes());
    h[24..28].copy_from_slice(&44_100u32.to_le_bytes());
    h[28..32].copy_from_slice(&88_200u32.to_le_bytes());
    h[32..34].copy_from_slice(&2u16.to_le_bytes());
    h[34..36].copy_from_slice(&16u16.to_le_bytes());
    h[36..40].copy_from_slice(b"data");
    h[40..44].copy_from_slice(&d.to_le_bytes());
    h
}

/// The level that starts a take on sound: -54 + 3 k dB of full scale.
fn trig_amp(k: u8) -> u32 {
    let mut a = 32768.0f32;
    for _ in 0..(TRIG_N + 1 - k.min(TRIG_N - 1)) {
        a *= 0.707_946; // -3 dB
    }
    a as u32
}

/// A level on the meter: -60 dB at 0 px .. 0 dB at 224 px.
fn db_px(amp: u32) -> u32 {
    if amp < 33 {
        return 0;
    }
    // log2 from the float's exponent and a fit of its mantissa (within 0.01)
    let b = (amp as f32).to_bits();
    let e = ((b >> 23) & 255) as f32 - 127.0;
    let m = f32::from_bits(b & 0x7F_FFFF | 0x3F80_0000) - 1.0;
    let db = 6.0206 * (e + m * (1.3466 - 0.3466 * m) - 15.0);
    (((db + 60.0) * 224.0 / 60.0).clamp(0.0, 224.0)) as u32
}

// ---- deleting a take: only in SAMPLES/RECORDED (what the PC put on the card stays), EDIT twice ----
impl Lab {
    fn can_delete(&self) -> bool {
        RECORDER
            && self.depth == 1
            && ShortFileName::create_from_str(REC_DIR).is_ok_and(|d| d == self.path[0])
            && self.ent(self.sel).is_some_and(|e| !e.dir)
    }

    fn del_button(&mut self, vm: &Vm, vol: RawVolume) {
        if !self.can_delete() {
            return;
        }
        let e = *self.ent(self.sel).unwrap();
        let name = &e.name[..(e.len as usize).min(16)];
        if !self.darm {
            self.darm = true;
            return self.say(PINK, &[b"EDIT AGAIN: DELETE ", name]);
        }
        self.darm = false;
        if self.page == Page::Trim {
            self.trim_close(vm);
        } else {
            self.stop_now(Some(vm));
        }
        let Some(d) = self.dir else { return };
        let ok = vm.delete_entry_in_dir(d, e.sfn).is_ok();
        let sel = self.sel;
        self.enter(Some((vm, vol)));
        self.sel = sel.min(self.n.saturating_sub(1));
        self.fill(vm);
        self.sel_at = self.now;
        self.moved_at = self.now;
        if ok {
            self.say(GREEN, &[b"DELETED ", name]);
        } else {
            self.say(PINK, &[b"CAN'T DELETE IT"]);
        }
    }
}

/// "SAVE 1" .. "SAVE 16"
fn save_name(i: usize) -> ([u8; 7], usize) {
    let mut t = *b"SAVE   ";
    let k = i + 1;
    if k >= 10 {
        t[5] = b'0' + (k / 10) as u8;
        t[6] = b'0' + (k % 10) as u8;
        (t, 7)
    } else {
        t[5] = b'0' + k as u8;
        (t, 6)
    }
}

fn copy(out: &mut [u8], s: &[u8]) -> usize {
    let n = s.len().min(out.len());
    out[..n].copy_from_slice(&s[..n]);
    n
}
fn fmt_num(out: &mut [u8], mut v: u32) -> usize {
    let mut d = [0u8; 10];
    let mut n = 0;
    loop {
        d[n] = b'0' + (v % 10) as u8;
        n += 1;
        v /= 10;
        if v == 0 {
            break;
        }
    }
    for i in 0..n.min(out.len()) {
        out[i] = d[n - 1 - i];
    }
    n.min(out.len())
}

struct Buf {
    b: [u8; 40],
    n: usize,
}
impl core::fmt::Write for Buf {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for &c in s.as_bytes() {
            if self.n < 39 {
                self.b[self.n] = c.to_ascii_uppercase();
                self.n += 1;
            }
        }
        Ok(())
    }
}

/// A name that doesn't fit `room` px (in 2 x letters): its start and its end, ".." between.
fn fit(name: &[u8], room: u32, out: &mut [u8]) -> usize {
    if text_w(name, 2) <= room {
        out[..name.len()].copy_from_slice(name);
        return name.len();
    }
    let (mut a, mut b) = (1usize, 1usize);
    let w = |a: usize, b: usize| text_w(&name[..a], 2) + 2 + text_w(b"..", 2) + 2 + text_w(&name[name.len() - b..], 2);
    loop {
        let grew_a = a + b < name.len() && w(a + 1, b) <= room && a <= b + 2;
        if grew_a {
            a += 1;
            continue;
        }
        if a + b < name.len() && w(a, b + 1) <= room {
            b += 1;
            continue;
        }
        if a + b < name.len() && w(a + 1, b) <= room {
            a += 1;
            continue;
        }
        break;
    }
    out[..a].copy_from_slice(&name[..a]);
    out[a..a + 2].copy_from_slice(b"..");
    out[a + 2..a + 2 + b].copy_from_slice(&name[name.len() - b..]);
    a + 2 + b
}

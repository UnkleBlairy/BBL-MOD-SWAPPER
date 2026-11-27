//! A slot ROM's save memory: the FM-1's 1 MiB SPI NOR (settings, sounds, kits, the last jam, the sample memory) as
//! real flash in the ROM's own slot, just below its tag (build.rs: NOR_ADDR). The firmware reads it in place, as the
//! FM-1 reads its NOR through XIP, and writes it through `teensy_nor_erase` / `teensy_nor_prog` (its glue's st_erase
//! / st_prog), which run the flash writer (flashw.rs: interrupts off, ~45 ms a 4 KB erase; the FM-1's own NOR stops
//! the sound for as long).
//!
//! The card is the copy that lasts: every sector written is marked, and the main loop writes the marked ones into
//! DATA/<STEM>.NOR (the whole memory, sector n at n * 4096; a short file reads as erased past its end), one sector a
//! step (~20 ms). When the menu copies the ROM into a slot again (roms.rs Load) it copies that file back first, so
//! nothing saved is lost when another ROM takes the slot.

use embedded_sdmmc::{Mode, RawFile, RawVolume, VolumeIdx, VolumeManager};

use crate::{flashw, sd};

include!(concat!(env!("OUT_DIR"), "/fwcfg.rs"));

const SECTOR: u32 = 4096;
const NSEC: usize = 256; // 1 MiB at most
static mut DIRTY: [u32; NSEC / 32] = [0; NSEC / 32];

fn dirty() -> &'static mut [u32; NSEC / 32] {
    unsafe { &mut *core::ptr::addr_of_mut!(DIRTY) }
}
fn mark(sec: u32) {
    dirty()[(sec / 32) as usize] |= 1 << (sec % 32);
}
fn next_dirty() -> Option<u32> {
    dirty().iter().enumerate().find(|(_, w)| **w != 0).map(|(i, w)| i as u32 * 32 + w.trailing_zeros())
}

/// Erase the 4 KB sector at `off` (from the save memory's start). 0 = done.
#[unsafe(no_mangle)]
pub extern "C" fn teensy_nor_erase(off: u32) -> i32 {
    if NOR_SIZE == 0 || off % SECTOR != 0 || off >= NOR_SIZE || !flashw::erase_sector(NOR_ADDR + off) {
        return -1;
    }
    mark(off / SECTOR);
    ERASES.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    LAST_ERASE.store(off, core::sync::atomic::Ordering::Relaxed);
    0
}
/// For the serial R report: sector erases since power-up, the last one's offset (what wrote to the flash).
pub static ERASES: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
pub static LAST_ERASE: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// Program `n` bytes at `off` (a NOR write: bits only go from 1 to 0). 0 = done. The bytes are copied to the stack
/// a page at a time first: nothing may be read from the flash while it programs (`src` may point into it).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn teensy_nor_prog(off: u32, src: *const u8, n: u32) -> i32 {
    if NOR_SIZE == 0 || off >= NOR_SIZE || n > NOR_SIZE - off {
        return -1;
    }
    let mut done = 0u32;
    while done < n {
        let a = off + done;
        let k = (flashw::PAGE - a % flashw::PAGE).min(n - done);
        let mut buf = [0u8; 256];
        unsafe { core::ptr::copy_nonoverlapping(src.add(done as usize), buf.as_mut_ptr(), k as usize) };
        if !flashw::program(NOR_ADDR + a, &buf[..k as usize]) {
            return -1;
        }
        mark(a / SECTOR);
        done += k;
    }
    0
}

/// Sectors written that the card doesn't have yet.
#[unsafe(no_mangle)]
pub extern "C" fn teensy_nor_written() -> u32 {
    dirty().iter().map(|w| w.count_ones()).sum()
}

pub type Vm = VolumeManager<sd::Sd, sd::Clock, 4, 3, 1>;

/// The card side: DATA/<STEM>.NOR, open while the firmware runs.
pub struct Mirror {
    card: Option<(Vm, RawFile, u32)>, // the card, the file, its length
    vol: Option<RawVolume>,           // the card's volume (the Sample Lab reads SAMPLES/ through the same manager)
    pub state: &'static str,          // for 'I': "OK", "NO CARD", "WRITE" ..
    pub sectors: u32,                 // written to the card since boot
}

impl Mirror {
    /// The card's file system, for the Sample Lab (src/lab.rs): the manager and its volume, while the card is good.
    pub fn fs(&self) -> Option<(&Vm, RawVolume)> {
        match (&self.card, self.vol) {
            (Some((vm, _, _)), Some(v)) => Some((vm, v)),
            _ => None,
        }
    }

    pub const fn new() -> Mirror {
        Mirror { card: None, vol: None, state: "-", sectors: 0 }
    }

    /// Open the card and the file (at boot: it can take a few hundred ms). Nothing to do without a save memory.
    #[inline(never)]
    pub fn open(&mut self) {
        if NOR_SIZE == 0 || self.card.is_some() {
            return;
        }
        self.state = "NO CARD";
        let Ok(card) = sd::Sd::init() else { return };
        let vm: Vm = VolumeManager::new_with_limits(card, sd::Clock, 0);
        self.state = "NO FAT";
        let Ok(vol) = vm.open_raw_volume(VolumeIdx(0)) else { return };
        let Ok(root) = vm.open_root_dir(vol) else { return };
        let data = match vm.open_dir(root, "DATA") {
            Ok(d) => d,
            Err(_) => {
                if vm.make_dir_in_dir(root, "DATA").is_err() {
                    return;
                }
                match vm.open_dir(root, "DATA") {
                    Ok(d) => d,
                    Err(_) => return,
                }
            }
        };
        let _ = vm.close_dir(root);
        let mut nm = [0u8; 12];
        let l = STEM.len().min(8);
        nm[..l].copy_from_slice(&STEM.as_bytes()[..l]);
        nm[l..l + 4].copy_from_slice(b".NOR");
        let name = core::str::from_utf8(&nm[..l + 4]).unwrap_or("SAVE.NOR");
        self.state = "OPEN";
        let Ok(f) = vm.open_file_in_dir(data, name, Mode::ReadWriteCreateOrAppend) else { return };
        let _ = vm.close_dir(data);
        let len = vm.file_length(f).unwrap_or(0);
        self.card = Some((vm, f, len));
        self.vol = Some(vol);
        self.state = "OK";
    }

    /// One piece of the copy to the card, if a sector waits: grow the file by an erased sector, or write one sector.
    /// Returns true when it did something (it took ~20 ms).
    #[inline(never)]
    pub fn step(&mut self) -> bool {
        let Some(sec) = next_dirty() else { return false };
        let Some((vm, f, len)) = self.card.as_mut() else { return false };
        let at = sec * SECTOR;
        let ok = if at > *len {
            // the file stops short of this sector: one erased sector on its end
            let ff = [0xFFu8; 512];
            let mut ok = vm.file_seek_from_start(*f, *len).is_ok();
            for _ in 0..SECTOR / 512 {
                ok = ok && vm.write(*f, &ff).is_ok();
            }
            if ok {
                *len += SECTOR;
            }
            ok && vm.flush_file(*f).is_ok()
        } else {
            let mut buf = [0u8; SECTOR as usize];
            let p = unsafe { core::slice::from_raw_parts((NOR_ADDR + at) as *const u8, SECTOR as usize) };
            buf.copy_from_slice(p);
            let ok = vm.file_seek_from_start(*f, at).is_ok() && vm.write(*f, &buf).is_ok() && vm.flush_file(*f).is_ok();
            if ok {
                *len = (*len).max(at + SECTOR);
                dirty()[(sec / 32) as usize] &= !(1 << (sec % 32));
                self.sectors += 1;
            }
            ok
        };
        if !ok {
            // stop trying (the sectors stay marked; the flash still has them)
            self.state = "WRITE FAILED";
            self.card = None;
        }
        true
    }
}

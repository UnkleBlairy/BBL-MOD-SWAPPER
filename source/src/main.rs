//! teensy-fm1: FM-1 firmwares running natively on the Teensy 4.1.
//!
//! Two images share the flash (build.rs, TFM1_FW): slot 0 at 0x60000000 (the one the boot ROM starts) holds
//! Felucca and the firmware menu, slot 1 at 0x60400000 the BBL Jam Box. At power-on slot 0 shows the menu on the
//! FM-1's screen (a knob scrolls, a button or key starts the one shown). Felucca starts in place; another slot is
//! started by a reset with its number in SNVS_LPGPR0, which slot 0 reads first thing and jumps to (once: the next
//! reset is the menu again). When the FM-1 goes away for 2 s and comes back (it was switched off and on), the Teensy
//! resets into the menu, as it will when both share one power switch.
//!
//! Then real time: one 128-frame audio block every 2.9 ms (or as the FM-1 plays them), the CPU load reported each
//! second over USB serial.
//!
//! Serial commands (one per line): `S` screen dump ("SCREEN 240 240\n" + 115200 bytes RGB565 as
//! the panel gets it), `B<hex>` buttons by label (bit 10 = PLAY), `K<hex>` keys, `E<role> <steps>`
//! encoder, `V<0..1023>` the MASTER volume pot, `I` what is running. In the menu: `E` scrolls, `B`/`K` start the
//! one shown, `P<n>` starts slot n. `X` resets to the menu. Opening the port at 134 baud reboots into the
//! bootloader (no button press to flash).
//!
//! `M` switches to mux mode (`m` back; closing the port also leaves it): everything the Teensy
//! sends is then packets `A5 5A type len_lo len_hi payload`, from a ring that never blocks the
//! audio loop. 'A' = one block of audio (128 stereo frames, i16 LE, L R), 'T' = a text line,
//! 'F' = a piece of the screen (u32 LE byte offset, then bytes). `F` asks for one screen, sent
//! in pieces between audio blocks (the next one only when the PC asks again).
//!
//! `L` switches to link mode (docs/LINK_PROTOCOL.md): mux output plus 'L' screen spans and 'D' LEDs for the FM-1
//! terminal, packet input ('I' / 'R' from the FM-1, 'C' command lines), audio paced by the FM-1's clock.
#![no_std]
#![no_main]

#[cfg(fw_rec)]
mod audio_in;
#[cfg(all(fw_lab, not(fw_rec)))]
#[path = "audio_in_stub.rs"]
mod audio_in;
mod flashw;
#[cfg(fw_lab)]
mod hooks;
#[cfg(fw_lab)]
mod lab;
#[cfg(fw_lab)]
mod states;
#[cfg(not(fw_lab))]
#[path = "lab_stub.rs"]
mod lab;
#[cfg(not(fw_menu))]
mod nor;
#[cfg(fw_menu)]
mod roms;
#[cfg(not(fw_hook_sloop))] // (SLOOP's slot 3 is full)
mod psram;
mod sd;
mod usbh;

use core::fmt::Write as _;
use core::panic::PanicInfo;

use bsp::{board, usbd};
use teensy4_bsp as bsp;
use usb_device::prelude::*;
use usbd_serial::SerialPort;

unsafe extern "C" {
    fn web_nor_erase();
    fn web_boot();
    fn web_render(n: u32);
    fn web_buttons(labels: u32);
    fn web_keys(mask: u32);
    fn web_enc(role: u32, n: i32);
    fn web_master(v: u32);
    fn web_out_l() -> *const f32;
    fn web_out_r() -> *const f32;
    fn teensy_fb() -> *const u16;
    fn teensy_draws() -> u32;
    fn teensy_pixels() -> u32;
    fn teensy_ui_frames() -> u32;
    fn teensy_redraw();
    fn teensy_spans(out: *mut u8, cap: u32) -> u32;
    fn teensy_raw_input(notes: u32, buttons: u32);
    fn teensy_raw_enc(hw: u32, n: i32);
    fn teensy_raw_adc(master: u32, batt: u32);
    fn teensy_leds(out: *mut u8) -> u32;
    #[cfg(fw_menu)]
    fn teensy_menu(
        title: *const core::ffi::c_char,
        names: *const *const core::ffi::c_char,
        details: *const *const core::ffi::c_char,
        n: u32,
        sel: u32,
        hint: *const core::ffi::c_char,
    );
    #[cfg(fw_menu)]
    fn teensy_loading(
        title: *const core::ffi::c_char,
        name: *const core::ffi::c_char,
        status: *const core::ffi::c_char,
        pct: u32,
        hint: *const core::ffi::c_char,
    );
    static mut __suninit: u8;
    static mut __euninit: u8;
}

const FS: u32 = 44_100;
const BLOCK: u32 = 128;

/// The built-in firmware (slot 0) for the menu: name and detail. The other slots hold ROMs from the card (roms.rs).
#[cfg(fw_menu)]
const BUILTIN: (&[u8], &[u8]) = (b"FELUCCA", b"1.8.1\nBUILT IN");
/// Where the slot to start goes across the reset: three places, as none is documented for this: the SNVS low-power
/// general purpose register 0, SRC_GPR5 (Teensyduino's crash report uses it across a reset), and the last words of
/// OCRAM2 (past everything the firmware uses; the boot ROM leaves them). The boot line ('I') shows what each held.
const FLAG_AT: [usize; 2] = [0x400F_8030, 0x400D_4090]; // SRC_GPR5 (tested: survives a reset), SNVS (did not keep a write)
const BOOT_MAGIC: u32 = 0x7F31_0000; // | slot: slot 0 starts that slot, once, at the next reset
const BOOT_LAB: u32 = 0x100; // ... | this: and that slot opens the Sample Lab first (the menu's SAMPLE LAB)
const LAB_MAGIC: u32 = 0x7F32_0000; // left for the slot by slot 0: open the Lab at start
static mut BOOT_SEEN: [u32; 2] = [0; 2]; // what the three held at start
static mut FLASH_CR0: u32 = 0; // the flash size the boot header set (KB): the 'I' line

fn set_flag(v: u32) {
    unsafe { (FLAG_AT[0] as *mut u32).write_volatile(v) }; // SNVS only (as in the build that ran)
    cortex_m::asm::dsb();
}

/// Reset the Teensy into a slot (0: the menu).
fn reboot_to(slot: u32) -> ! {
    set_flag(if slot == 0 { 0 } else { BOOT_MAGIC | slot });
    cortex_m::peripheral::SCB::sys_reset();
}
/// Reset into a slot with its Sample Lab open (the menu's SAMPLE LAB).
#[cfg(fw_menu)]
fn reboot_to_lab(slot: u32) -> ! {
    set_flag(BOOT_MAGIC | BOOT_LAB | slot);
    cortex_m::peripheral::SCB::sys_reset();
}

/// The flash size the boot ROM set from the boot header (FlexSPI1 FLSHA1CR0, KB): build.sh patches the header from the
/// Rust default (teensy4-fcb: the Teensy 4.0's 2 MB, so a read of slot 1 at 4 MB faulted) to 8 MB. Read only:
/// changing it while running crashed the Teensy.
fn flash_window() {
    unsafe { FLASH_CR0 = (0x402A_8060 as *const u32).read_volatile() };
}

/// Slot 0, first thing in main (clocks, caches, USB as the boot ROM left them): a slot picked before the reset
/// starts now. Its image's boot header (IVT) points at its vector table: the stack, then its Reset, which sets up
/// the RAM for itself as at power-on.
#[cfg(fw_menu)]
unsafe fn start_picked() {
    unsafe {
        let v = (FLAG_AT[0] as *const u32).read_volatile();
        BOOT_SEEN[0] = v;
        (FLAG_AT[0] as *mut u32).write_volatile(0);
        let slot = (v & 0xFF) as usize;
        if v & 0xFFFF_0000 != BOOT_MAGIC || slot == 0 || slot >= roms::NSLOT {
            return;
        }
        if (0x402A_8060 as *const u32).read_volatile() < 8192 {
            return; // the boot header says less than 8 MB: slot 1 cannot be read
        }
        let ivt = roms::BASE[slot] + 0x1000;
        if (ivt as *const u32).read_volatile() != 0x4020_00D1 || ((ivt + 0x14) as *const u32).read_volatile() != ivt {
            return;
        }
        let vt = ((ivt + 4) as *const u32).read_volatile();
        let sp = (vt as *const u32).read_volatile();
        let reset = ((vt + 4) as *const u32).read_volatile();
        if v & BOOT_LAB != 0 {
            (FLAG_AT[0] as *mut u32).write_volatile(LAB_MAGIC); // (the slot reads it, and clears it)
        }
        core::arch::asm!("msr msp, {sp}", "bx {reset}", sp = in(reg) sp, reset = in(reg) reset, options(noreturn));
    }
}

/// A crash record in SRC_GPR6..8 (kept through a reset; not in OCRAM: before board init it is not clocked, and
/// touching it first thing crashed the Teensy): GPR6 = 0xC4 << 24 | crashes in a row << 16 | start-up stage,
/// GPR7 = the fault pc, GPR8 = CFSR.
const CR_INFO: *mut u32 = 0x400F_8034 as *mut u32;
const CR_PC: *mut u32 = 0x400F_8038 as *mut u32;
const CR_CFSR: *mut u32 = 0x400F_803C as *mut u32;
static mut LAST_CRASH: [u32; 3] = [0; 3];

fn stage(n: u32) {
    unsafe {
        let v = CR_INFO.read_volatile();
        let keep = if v >> 24 == 0xC4 { v & 0xFFFF_0000 } else { 0xC400_0000 };
        CR_INFO.write_volatile(keep | (n & 0xFFFF));
    }
}


/// The profiler (builds whose code runs from flash, so ITCM is free): SysTick samples the interrupted pc at
/// 5 kHz into ITCM; `H1` starts, `H0` stops, `H` sends "PROF n\n" then n u32 pcs (plain mode).
#[cfg(fw_text_flash)]
mod prof {
    pub const BUF: *mut u32 = 0x0001_0000 as *mut u32; // 64 KB of ITCM
    pub const CAP: usize = 16384;
    pub static mut N: usize = 0;
    core::arch::global_asm!(
        ".section .text.SysTick,\"ax\"",
        ".global SysTick",
        ".type SysTick,%function",
        ".thumb_func",
        "SysTick:",
        "mrs r0, msp",
        "b prof_sample",
    );
    #[unsafe(no_mangle)]
    extern "C" fn prof_sample(frame: *const u32) {
        unsafe {
            if N < CAP {
                BUF.add(N).write_volatile(frame.add(6).read_volatile());
                N += 1;
            }
        }
    }
    pub fn run(on: bool) {
        unsafe {
            let mut syst = cortex_m::Peripherals::steal().SYST;
            if on {
                N = 0;
                syst.set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
                syst.set_reload(600_000_000 / 5000 - 1);
                syst.clear_current();
                syst.enable_interrupt();
                syst.enable_counter();
            } else {
                syst.disable_interrupt();
                syst.disable_counter();
            }
        }
    }
}

/// The PC's serial port for roms::upload (it keeps USB serviced while it waits).
#[cfg(fw_menu)]
struct Pc<'a, 'b> {
    dev: &'a mut UsbDevice<'b, usbd::BusAdapter>,
    serial: &'a mut SerialPort<'b, usbd::BusAdapter>,
}
#[cfg(fw_menu)]
impl roms::Io for Pc<'_, '_> {
    fn read(&mut self, buf: &mut [u8]) -> usize {
        self.dev.poll(&mut [&mut *self.serial]);
        self.serial.read(buf).unwrap_or(0)
    }
    fn write(&mut self, b: &[u8]) {
        let mut rest = b;
        let t = cycles();
        while !rest.is_empty() && cycles().wrapping_sub(t) < 600_000_000 {
            self.dev.poll(&mut [&mut *self.serial]);
            if let Ok(n) = self.serial.write(rest) {
                rest = &rest[n..];
            }
        }
    }
}

/// The Sample Lab (lab.rs; a stub in mods without a sample hook).
static mut LAB: lab::Lab = lab::Lab::new();

static EP_MEMORY: usbd::EndpointMemory<2048> = usbd::EndpointMemory::new();
static EP_STATE: usbd::EndpointState = usbd::EndpointState::max_endpoints();

/// A fixed-size text buffer for status lines.
struct Line {
    buf: [u8; 256],
    len: usize,
}
impl Line {
    const fn new() -> Self {
        Self { buf: [0; 256], len: 0 }
    }
    fn as_bytes(&self) -> &[u8] {
        &self.buf[..self.len]
    }
}
impl core::fmt::Write for Line {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let b = s.as_bytes();
        let n = b.len().min(self.buf.len() - self.len);
        self.buf[self.len..self.len + n].copy_from_slice(&b[..n]);
        self.len += n;
        Ok(())
    }
}

/// A longer text buffer (the SD report).
struct Text {
    buf: [u8; 3000],
    len: usize,
}
impl Text {
    const fn new() -> Self {
        Self { buf: [0; 3000], len: 0 }
    }
    fn as_bytes(&self) -> &[u8] {
        &self.buf[..self.len]
    }
}
impl core::fmt::Write for Text {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let b = s.as_bytes();
        let n = b.len().min(self.buf.len() - self.len);
        self.buf[self.len..self.len + n].copy_from_slice(&b[..n]);
        self.len += n;
        Ok(())
    }
}

/// Bytes waiting to go out over USB (mux mode): written whole or not at all, drained as the host reads.
const TX: usize = 16 * 1024;
struct Ring {
    buf: [u8; TX],
    head: usize, // next write
    tail: usize, // next read
}
impl Ring {
    fn used(&self) -> usize {
        (self.head + TX - self.tail) % TX
    }
    fn free(&self) -> usize {
        TX - 1 - self.used()
    }
    fn put(&mut self, b: &[u8]) {
        for &x in b {
            self.buf[self.head] = x;
            self.head = (self.head + 1) % TX;
        }
    }
    /// One packet, or nothing if it does not fit.
    fn packet(&mut self, ty: u8, parts: &[&[u8]]) -> bool {
        let len: usize = parts.iter().map(|p| p.len()).sum();
        if len + 5 > self.free() {
            return false;
        }
        self.put(&[0xA5, 0x5A, ty, len as u8, (len >> 8) as u8]);
        for p in parts {
            self.put(p);
        }
        true
    }
    /// The longest run that can be handed to USB in one write.
    fn readable(&self) -> &[u8] {
        let end = if self.head >= self.tail { self.head } else { TX };
        &self.buf[self.tail..end]
    }
    fn consume(&mut self, n: usize) {
        self.tail = (self.tail + n) % TX;
    }
}
static mut RING: Ring = Ring { buf: [0; TX], head: 0, tail: 0 };

fn cycles() -> u32 {
    cortex_m::peripheral::DWT::cycle_count()
}

/// 64-bit cycle clock from the 32-bit DWT counter (call at least every 7 s).
struct Clock {
    last: u32,
    hi: u64,
}
impl Clock {
    fn now(&mut self) -> u64 {
        let c = cycles();
        if c < self.last {
            self.hi += 1 << 32;
        }
        self.last = c;
        self.hi | c as u64
    }
}

fn parse_hex(s: &[u8]) -> u32 {
    s.iter().fold(0u32, |v, &c| {
        let d = match c {
            b'0'..=b'9' => c - b'0',
            b'a'..=b'f' => c - b'a' + 10,
            b'A'..=b'F' => c - b'A' + 10,
            _ => return v,
        };
        v << 4 | d as u32
    })
}

fn parse_int(s: &[u8]) -> i32 {
    let (neg, s) = match s.first() {
        Some(b'-') => (true, &s[1..]),
        Some(b'+') => (false, &s[1..]),
        _ => (false, s),
    };
    let v = s.iter().take_while(|c| c.is_ascii_digit()).fold(0i32, |v, &c| v * 10 + (c - b'0') as i32);
    if neg { -v } else { v }
}

#[bsp::rt::entry]
fn main() -> ! {
    let safe = false;
    #[cfg(fw_menu)]
    unsafe {
        start_picked()
    };
    let board::Resources { pins, mut gpio2, usb, .. } = board::t41(board::instances());
    let led = board::led(&mut gpio2, pins.p13);

    let mut cp = cortex_m::Peripherals::take().unwrap();
    cp.SCB.enable_icache();
    cp.SCB.enable_dcache(&mut cp.CPUID);
    cp.DCB.enable_trace();
    cp.DWT.enable_cycle_counter();
    let hz = board::ARM_FREQUENCY as u64;

    // Felucca's big buffers sit in .uninit (OCRAM, build.rs): zero them as .bss would be.
    unsafe {
        let s = &raw mut __suninit;
        let e = &raw mut __euninit;
        core::ptr::write_bytes(s, 0, e as usize - s as usize);
    }

    let bus = usbd::BusAdapter::with_speed(usb, &EP_MEMORY, &EP_STATE, usbd::Speed::LowFull);
    bus.set_interrupts(false);
    let bus = usb_device::bus::UsbBusAllocator::new(bus);
    let mut serial = SerialPort::new(&bus);
    let mut dev = UsbDeviceBuilder::new(&bus, UsbVidPid(0x1209, 0x000A))
        .strings(&[StringDescriptors::default()
            .manufacturer("teensy-fm1")
            .product(concat!("BBL Mod Swapper (", env!("TFM1_NAME"), ")"))
            .serial_number("0001")])
        .unwrap()
        .device_class(usbd_serial::USB_CLASS_CDC)
        .max_packet_size_0(64)
        .unwrap()
        .build();
    let mut configured = false;
    let mut clock = Clock { last: cycles(), hi: 0 };
    // the FM-1 straight into the Teensy's USB host port (no PC): the link runs there (usbh.rs)
    let mut host = usbh::Host::new();
    let mut direct = false;

    // keep USB alive and handle the 134-baud reboot; returns true when a terminal is listening
    macro_rules! service {
        () => {{
            dev.poll(&mut [&mut serial]);
            let now = dev.state() == UsbDeviceState::Configured;
            if now && !configured {
                dev.bus().configure();
            }
            configured = now;
            if serial.line_coding().data_rate() == 134 {
                cortex_m::asm::delay(60_000_000);
                unsafe { core::arch::asm!("bkpt #251") };
            }
            let listening: bool = configured && serial.dtr();
            listening
        }};
    }
    // write everything (polling USB while the host drains it); gives up after ~1 s without progress
    macro_rules! send {
        ($bytes:expr) => {{
            let mut rest: &[u8] = $bytes;
            let mut stall = clock.now();
            while !rest.is_empty() {
                if !service!() {
                    break;
                }
                match serial.write(rest) {
                    Ok(n) if n > 0 => {
                        rest = &rest[n..];
                        stall = clock.now();
                    }
                    _ => {
                        if clock.now() - stall > hz {
                            break;
                        }
                    }
                }
            }
        }};
    }

    // ---- start: the menu (slot 0), or this slot's firmware ----
    let mut menu = cfg!(fw_menu); // the firmware menu is on the screen: no firmware running yet
    let mut sel: u32 = 0; // the menu entry shown
    let mut menu_in: Option<(u32, u32)> = None; // the FM-1's keys and buttons at its last report (menu)
    let mut bench = Line::new(); // what is running ('I')
    #[cfg(not(fw_menu))]
    let mut mirror = nor::Mirror::new(); // the save memory's copy on the card
    #[cfg(fw_hook_jambox)]
    let mut smp_move = hooks::jambox::Move::new();
    #[cfg(not(fw_menu))]
    let mut mirror_at: u64 = 0;
    // the Sample Lab: open over a paused mod (FX + HOME + GLO + REC); the gate keeps the combo from the mod
    let lab = unsafe { &mut *(&raw mut LAB) };
    let mut gate = lab::Gate::new();
    let mut menu_hold: Option<u32> = None; // FX + HOME went down at (ms): held MENU_HOLD_MS, back to the menu
    macro_rules! ms {
        () => {
            (clock.now() * 1000 / hz) as u32
        };
    }
    #[cfg(fw_lab)]
    macro_rules! card {
        () => {
            lab::fs(mirror.fs())
        };
    }
    #[cfg(not(fw_lab))]
    macro_rules! card {
        () => {
            ()
        };
    }
    macro_rules! lab_open {
        () => {{
            lab.open(card!(), ms!(), gate.raw());
        }};
    }
    // save states (FX + SAVE): the same overlay, its SAVES page
    macro_rules! lab_saves {
        () => {{
            lab.open_saves(card!(), ms!(), gate.raw());
        }};
    }
    macro_rules! lab_close {
        () => {{
            lab.close(card!());
            gate.closed(&mut |n, b| unsafe { teensy_raw_input(n, b) });
            unsafe { teensy_redraw() };
        }};
    }
    // the FM-1's keys and buttons: to the Lab when it is open, else through the gate to the mod
    macro_rules! raw_input {
        ($n:expr, $b:expr) => {{
            let (n, b) = ($n, $b);
            if lab.open {
                if lab.raw(card!(), n, b, ms!()) {
                    lab_close!();
                }
            } else {
                match gate.input(n, b, ms!(), &mut |n, b| unsafe { teensy_raw_input(n, b) }) {
                    1 => lab_open!(),
                    2 => lab_saves!(),
                    3 => menu_hold = Some(ms!()),
                    _ => {}
                }
            }
        }};
    }
    macro_rules! boot {
        () => {{
            led.set();
            let t = clock.now();
            #[cfg(not(fw_menu))]
            mirror.open();
            unsafe {
                web_nor_erase();
                web_boot();
                teensy_redraw();
            }
            bench = Line::new();
            let _ = write!(
                bench,
                "BOOT {} {} in {} ms\r\n",
                env!("TFM1_NAME"),
                env!("TFM1_VERSION"),
                (clock.now() - t) * 1000 / hz
            );
            led.clear();
        }};
    }
    // the ROM library (menu slot): the card, the menu's entries, a copy in progress, the hint line
    #[cfg(fw_menu)]
    let mut lib: Option<roms::Lib> = if menu { roms::Lib::open() } else { None };
    #[cfg(fw_menu)]
    let mut cat = roms::Catalog::new();
    #[cfg(fw_menu)]
    if menu {
        cat.build(BUILTIN, lib.as_ref());
    }
    #[cfg(fw_menu)]
    let mut loading: Option<(roms::Load, usize, u32)> = None; // the copy, its entry, the percent last drawn
    #[cfg(fw_menu)]
    let mut hint = [0u8; 48];
    #[cfg(fw_menu)]
    macro_rules! set_hint {
        ($($t:expr),*) => {{
            hint = [0u8; 48];
            let mut n = 0;
            $( for &c in AsRef::<[u8]>::as_ref($t) { if n < 47 { hint[n] = c; n += 1; } } )*
        }};
    }
    #[cfg(fw_menu)]
    set_hint!(if lib.is_some() { "SELECT\x04\x05\tPLAY\x01" } else { "NO CARD\tPLAY\x01" });
    #[cfg(fw_menu)]
    macro_rules! draw_menu {
        () => {{
            let mut names = [core::ptr::null::<core::ffi::c_char>(); roms::MAX];
            let mut details = [core::ptr::null::<core::ffi::c_char>(); roms::MAX];
            for i in 0..cat.n {
                names[i] = cat.e[i].name.as_ptr() as *const core::ffi::c_char;
                details[i] = cat.e[i].detail.as_ptr() as *const core::ffi::c_char;
            }
            unsafe { teensy_menu(c"PICK A MOD".as_ptr(), names.as_ptr(), details.as_ptr(), cat.n as u32, sel, hint.as_ptr() as *const core::ffi::c_char) };
        }};
    }
    #[cfg(fw_menu)]
    macro_rules! draw_loading {
        ($i:expr, $pct:expr) => {{
            let mut d = [0u8; 24];
            let p = $pct as u32;
            let txt: [u8; 3] = [if p >= 100 { b'1' } else { b' ' }, b'0' + ((p / 10) % 10) as u8, b'0' + (p % 10) as u8];
            let mut n = 0;
            for &c in b"COPYING ".iter().chain(txt.iter()).chain(b"%".iter()) { d[n] = c; n += 1; }
            unsafe {
                teensy_loading(c"LOADING".as_ptr(), cat.e[$i].name.as_ptr() as *const core::ffi::c_char,
                               d.as_ptr() as *const core::ffi::c_char, p, c"KEEP THE POWER ON".as_ptr())
            };
        }};
    }
    #[cfg(not(fw_menu))]
    macro_rules! draw_menu {
        () => {{}};
    }
    // the menu: move by steps, or start entry k
    #[cfg(fw_menu)]
    macro_rules! menu_move {
        ($steps:expr) => {{
            if loading.is_none() {
                let n = cat.n as i32;
                sel = (sel as i32 + ($steps as i32).signum()).rem_euclid(n) as u32;
                draw_menu!();
            }
        }};
    }
    #[cfg(not(fw_menu))]
    macro_rules! menu_move {
        ($steps:expr) => {{
            let _ = $steps;
        }};
    }
    #[cfg(fw_menu)]
    let mut lab_after = false; // the ROM being copied opens its Sample Lab when it starts
    #[cfg(fw_menu)]
    macro_rules! pick {
        ($k:expr) => {{
            let mut k = $k as usize;
            // SAMPLE LAB: the Jam Box, its Lab open
            lab_after = k < cat.n && cat.e[k].stem == roms::LAB_STEM;
            if lab_after {
                k = cat.e[..cat.n].iter().position(|e| e.stem == roms::LAB_HOST).unwrap_or(usize::MAX);
            }
            if menu && loading.is_none() && k < cat.n {
                if k == 0 {
                    menu = false;
                    boot!();
                } else if let Some(s) = cat.e[k].ready() {
                    roms::stamp(s);
                    if lab_after { reboot_to_lab(s as u32) } else { reboot_to(s as u32) };
                } else if let (Some(l), Some(s)) = (lib.as_ref(), cat.target(k)) {
                    match roms::Load::start(l, &cat.e[k].stem, s) {
                        Ok(ld) => {
                            loading = Some((ld, k, 0));
                            draw_loading!(k, 0);
                        }
                        Err(e) => {
                            set_hint!("CARD ERROR ", e);
                            draw_menu!();
                        }
                    }
                } else {
                    set_hint!("NOT IN FLASH - PUT THE CARD IN");
                    draw_menu!();
                }
            }
        }};
    }
    #[cfg(not(fw_menu))]
    macro_rules! pick {
        ($k:expr) => {{
            let _ = $k;
        }};
    }
    if menu {
        draw_menu!();
        let seen = unsafe { BOOT_SEEN };
        let _ = write!(
            bench,
            "MENU flags at start {:08x} {:08x}; flash window was {} KB; slot 1 header {:08x}\r\n",
            seen[0],
            seen[1],
            unsafe { (0x402A_8060 as *const u32).read_volatile() },
            if unsafe { (0x402A_8060 as *const u32).read_volatile() } >= 8192 {
                unsafe { (0x6020_1000 as *const u32).read_volatile() }
            } else {
                0
            }
        );
    } else {
        boot!();
        // started from the menu's SAMPLE LAB: the Lab first
        #[cfg(fw_rec)]
        if unsafe { (FLAG_AT[0] as *const u32).read_volatile() } == LAB_MAGIC {
            set_flag(0);
            lab_open!();
        }
    }
    {
        let c = unsafe { LAST_CRASH };
        if safe {
            let _ = write!(
                bench,
                "CRASHED x{} at start-up stage {}: pc={:08x} cfsr={:08x}{}\r\n",
                (c[0] >> 16) & 0xFF, c[0] & 0xFFFF, c[1], c[2],
                if safe { " (safe mode: no slot switch)" } else { "" }
            );
        }
    }

    // ---- real time: one block every BLOCK/FS seconds (or as the FM-1 plays them, in link mode) ----
    let period = hz * BLOCK as u64 / FS as u64;
    let mut next = clock.now() + period;
    let (mut busy, mut worst, mut blocks, mut late) = (0u64, 0u64, 0u32, 0u32);
    let mut report_at = clock.now() + hz;
    let mut listening = false;
    let mut cmd = [0u8; 64];
    let mut cmd_len = 0usize;
    let mut secs = 0u32;
    let (mut ui_last, mut px_last) = (0u32, 0u32);
    let ring = unsafe { &mut *(&raw mut RING) };
    let mut mux = false;
    let mut frame_at: Option<usize> = None; // next screen byte to send (mux mode)
    let (mut dropped, mut audio_sent) = (0u32, 0u32);
    const CHUNK: usize = 2048;
    const FB_BYTES: usize = 240 * 240 * 2;
    const AUDIO_MARGIN: usize = 4 * 520; // ring space kept for audio before screen data goes in
    const SPAN_BACKLOG: usize = 3 * 1024; // at most this much queued to USB before more spans go in
    // link mode (docs/LINK_PROTOCOL.md)
    let mut link = false;
    let mut pk = [0u8; 300]; // the packet being received
    let mut pk_len = 0usize;
    let mut sent_frames: u32 = 0; // audio frames sent since link mode began
    let mut played: u32 = 0; // ... and played by the FM-1 (its 'I' report)
    let mut link_gen: u8 = 0; // the sync generation ('Z'): the FM-1's counts are from our last 'Z'
    let mut synced = false; // a report of the current generation has come back
    let mut z_at: u64 = 0; // when the last 'Z' went out
    let mut last_in = (u32::MAX, u32::MAX, u32::MAX); // the FM-1's last notes, buttons, ADCs: applied on change
    let mut last_report: u64 = 0;
    let mut last_any: u64 = 0; // the FM-1's last report, of any generation
    let (mut fm1_seen, mut fm1_gone) = (false, false); // reports came in this link; they stopped for 2 s since
    let (mut fm1_under, mut fm1_fifo, mut reports) = (0u32, 0u32, 0u32);
    let mut target: u32 = 2048; // frames in flight (relay + FM-1 FIFO): 46 ms; 'Q n' sets it (the latency)
    let mut led_at = clock.now();
    let mut span_buf = [0u8; CHUNK];
    let mut led_buf = [0u8; 45];

    // one command line (serial, or a 'C' packet in link mode)
    macro_rules! command {
        ($line:expr) => {{
            let line: &[u8] = $line;
            match line.first() {
                Some(b'S') if !mux || direct => {
                    send!(b"SCREEN 240 240\n");
                    if lab.open {
                        let mut row = [0u8; 480];
                        for y in 0..240 {
                            lab.row_bytes(y, &mut row);
                            send!(&row);
                        }
                    } else {
                        let fb = unsafe { core::slice::from_raw_parts(teensy_fb() as *const u8, FB_BYTES) };
                        send!(fb);
                    }
                    next = clock.now() + period;
                }
                // the Sample Lab: J opens / closes it (as FX + HOME + GLO + REC / HOME), J? tells what it shows
                Some(b'J') if !menu => {
                    if line.get(1) != Some(&b'?') {
                        if lab.open { lab_close!(); } else { lab_open!(); }
                    }
                    let mut l = Text::new();
                    lab.status(&mut l);
                    if mux && !direct { ring.packet(b'T', &[l.as_bytes()]); } else { send!(l.as_bytes()); }
                }
                Some(b'B') if lab.open => {
                    if lab.buttons(card!(), parse_hex(&line[1..]), ms!()) {
                        lab_close!();
                    }
                }
                Some(b'B') if !menu && parse_hex(&line[1..]) & lab::COMBO == lab::COMBO && cfg!(fw_lab) => lab_open!(),
                Some(b'B') if !menu && parse_hex(&line[1..]) & lab::SAVES == lab::SAVES && cfg!(fw_lab) => lab_saves!(),
                Some(b'K') if lab.open => lab.keys_in(card!(), parse_hex(&line[1..]), ms!()),
                Some(b'E') if lab.open && line.len() > 2 => {
                    lab.enc(card!(), (line[1] - b'0') as u32, parse_int(&line[2..].trim_ascii()), ms!())
                }
                Some(b'B' | b'K') if menu => {
                    if parse_hex(&line[1..]) != 0 {
                        pick!(sel);
                    }
                }
                Some(b'E') if menu && line.len() > 2 => {
                    let st = parse_int(&line[2..].trim_ascii());
                    if st != 0 {
                        menu_move!(st);
                    }
                }
                Some(b'P') => pick!(parse_int(line[1..].trim_ascii()).max(0)),
                Some(b'X') => reboot_to(0),
                // PSRAM test (Teensy 4.1 underside chips): IDs, then write/read every word of each chip found
                #[cfg(not(fw_hook_sloop))]
                Some(b'O') => {
                    unsafe {
                        core::ptr::write_volatile(0xE000_EDFC as *mut u32, core::ptr::read_volatile(0xE000_EDFC as *const u32) | (1 << 24));
                        core::ptr::write_volatile(0xE000_1000 as *mut u32, core::ptr::read_volatile(0xE000_1000 as *const u32) | 1);
                    }
                    let (a, b) = psram::init();
                    let mut l = Line::new();
                    let _ = write!(l, "PSRAM chip1 id {:04X} chip2 id {:04X} (5D0D = APS6404 OK)\n", a, b);
                    for (k, id) in [a, b].into_iter().enumerate() {
                        if id == 0x5D0D {
                            let (bad, first, wr, rd) = psram::test_chip(k, 600);
                            if bad == 0 {
                                let _ = write!(l, "PSRAM chip{} 8 MB PASS (write {} MB/s, read {} MB/s)\n", k + 1, wr, rd);
                            } else {
                                let _ = write!(l, "PSRAM chip{} FAIL {} bad words, first at {:08X}\n", k + 1, bad, first);
                            }
                        } else {
                            let _ = write!(l, "PSRAM chip{} not found\n", k + 1);
                        }
                    }
                    if mux && !direct { ring.packet(b'T', &[l.as_bytes()]); } else { send!(l.as_bytes()); }
                }
                #[cfg(fw_text_flash)]
                Some(b'H') if !mux => match line.get(1) {
                    Some(b'1') => prof::run(true),
                    Some(b'0') => prof::run(false),
                    _ => {
                        prof::run(false);
                        let n = unsafe { prof::N };
                        let mut l = Line::new();
                        let _ = write!(l, "PROF {}\n", n);
                        send!(l.as_bytes());
                        send!(unsafe { core::slice::from_raw_parts(prof::BUF as *const u8, n * 4) });
                        next = clock.now() + period;
                    }
                },
                // the SD card: start it, its size and speed, the files in its root (a few lines, while audio waits)
                Some(b'D') => {
                    let mut out = Text::new();
                    #[cfg(fw_menu)] // card setup only in the menu slot (JIANT's ITCM is full)
                    if line.get(1) == Some(&b'1') { sd::setup(&mut out); } else { sd::report(&mut out); }
                    #[cfg(not(fw_menu))]
                    sd::report(&mut out);
                    if mux && !direct { ring.packet(b'T', &[out.as_bytes()]); } else { send!(out.as_bytes()); }
                    next = clock.now() + period;
                }
                // a file from the PC onto the card (menu slot; roms::upload has the protocol), then the menu again
                #[cfg(fw_menu)]
                Some(b'U') if (!mux || direct) && menu && loading.is_none() => {
                    if let Some(l) = lib.as_ref() {
                        let mut io = Pc { dev: &mut dev, serial: &mut serial };
                        roms::upload(l, line[1..].trim_ascii(), &mut io);
                        cat.build(BUILTIN, Some(l));
                        sel = sel.min(cat.n as u32 - 1);
                        draw_menu!();
                    } else {
                        send!(b"U ERR no card\r\n");
                    }
                    next = clock.now() + period;
                }
                // forget a flash slot (Z1..Z3) or delete a card file (Z DIR/NAME), then the menu again
                #[cfg(fw_menu)]
                Some(b'Z') if menu && loading.is_none() => {
                    let r = roms::remove(lib.as_ref(), &line[1..]);
                    cat.build(BUILTIN, lib.as_ref());
                    sel = sel.min(cat.n as u32 - 1);
                    draw_menu!();
                    let mut l = Line::new();
                    let _ = write!(l, "{}\r\n", r);
                    send!(l.as_bytes());
                }
                // the card again (it was put in, or changed): reopen it and rebuild the menu
                #[cfg(fw_menu)]
                Some(b'Y') if menu && loading.is_none() => {
                    lib = roms::Lib::open();
                    cat.build(BUILTIN, lib.as_ref());
                    sel = sel.min(cat.n as u32 - 1);
                    set_hint!(if lib.is_some() { "SELECT\x04\x05\tPLAY\x01" } else { "NO CARD\tPLAY\x01" });
                    draw_menu!();
                    let mut l = Text::new();
                    let _ = write!(l, "MENU {} entries, card {}\r\n", cat.n, if lib.is_some() { "yes" } else { "no" });
                    for i in 0..cat.n {
                        let e = &cat.e[i];
                        let z = |b: &[u8]| b.iter().position(|&c| c == 0).unwrap_or(b.len());
                        let _ = write!(l, "  {} {} [{}] slot {:?} card {:03b}\r\n", i,
                            core::str::from_utf8(&e.name[..z(&e.name)]).unwrap_or("?"),
                            core::str::from_utf8(&e.detail[..z(&e.detail)]).unwrap_or("?"), e.slot, e.card >> 1);
                    }
                    send!(l.as_bytes());
                }
                // flash write self-test in slot 3's first block (menu slot, plain mode only)
                #[cfg(fw_menu)]
                Some(b'W') if (!mux || direct) && menu => {
                    let (te, tw, bad, ne) = flashw::self_test();
                    let mut l = Line::new();
                    let _ = write!(l, "FLASH erase 64 KB {} us (not erased after: {}), write 64 KB {} us, wrong {}\r\n", te, ne, tw, bad);
                    send!(l.as_bytes());
                    next = clock.now() + period;
                }
                // probes, one at a time at run time: G0 read registers, G1 write SRC_GPR5 test value
                Some(b'G') => {
                    let mut l = Line::new();
                    let r = |a: usize| unsafe { (a as *const u32).read_volatile() };
                    match line.get(1) {
                        Some(b'1') => unsafe { (0x400F_8030 as *mut u32).write_volatile(0x5A5A_0001) },
                        Some(b'2') => unsafe { (0x400D_4090 as *mut u32).write_volatile(0x5A5A_0002) },
                        _ => {}
                    }
                    let _ = write!(l, "PROBE snvs90={:08x} srcgpr5={:08x} flexspi_cr0={:08x}\r\n", r(0x400D_4090), r(0x400F_8030), r(0x402A_8060));
                    if mux && !direct { ring.packet(b'T', &[l.as_bytes()]); } else { send!(l.as_bytes()); }
                }
                Some(b'B') => unsafe { web_buttons(parse_hex(&line[1..])) },
                Some(b'K') => unsafe { web_keys(parse_hex(&line[1..])) },
                Some(b'E') if line.len() > 2 => unsafe {
                    web_enc((line[1] - b'0') as u32, parse_int(&line[2..].trim_ascii()))
                },
                // the Jam Box's track recording: its state (trec.c), for testing
                #[cfg(fw_rec)]
                Some(b'R') if !menu => {
                    unsafe extern "C" {
                        fn teensy_trec_info(o: *mut u8, cap: u32) -> u32;
                    }
                    let mut t = Text::new();
                    let n = unsafe { teensy_trec_info(t.buf.as_mut_ptr(), t.buf.len() as u32) } as usize;
                    t.len = n.min(t.buf.len());
                    if mux && !direct { ring.packet(b'T', &[t.as_bytes()]); } else { send!(t.as_bytes()); }
                    let (fx, rs, lag) = audio_in::fixes();
                    let (lo, hi) = audio_in::lag_span();
                    let mut l = Line::new();
                    let _ = write!(l, "LINEIN ppm={} resyncs={} lag={} lag_min={} lag_max={} in_lat={} out_lat={} target={} fm1_fifo={} fm1_underruns={}\r\n",
                                   fx, rs, lag, lo, hi, audio_in::teensy_in_latency(),
                                   audio_in::OUT_LAT.load(core::sync::atomic::Ordering::Relaxed), target, fm1_fifo, fm1_under);
                    if mux && !direct { ring.packet(b'T', &[l.as_bytes()]); } else { send!(l.as_bytes()); }
                    let (w, ov, rst, cs, cs_stuck) = audio_in::health();
                    let mut h = Line::new();
                    let _ = write!(h, "LINEIN written={} overruns={} restarts={} rcsr={:08x} rcsr_stuck={:08x} nor_erases={} last_erase={:x}\r\n",
                                   w, ov, rst, cs, cs_stuck, nor::ERASES.load(core::sync::atomic::Ordering::Relaxed),
                                   nor::LAST_ERASE.load(core::sync::atomic::Ordering::Relaxed));
                    if mux && !direct { ring.packet(b'T', &[h.as_bytes()]); } else { send!(h.as_bytes()); }
                }
                Some(b'I') if mux && !direct => {
                    ring.packet(b'T', &[bench.as_bytes()]);
                }
                Some(b'I') => send!(bench.as_bytes()),
                // the save memory's copy on the card (slot ROMs with one): its state, sectors waiting, sectors written
                #[cfg(not(fw_menu))]
                Some(b'N') => {
                    let mut l = Line::new();
                    let _ = write!(l, "NOR {} at {:#x} {} KB: card {}, waiting {}, written {}\r\n", nor::STEM, nor::NOR_ADDR,
                        nor::NOR_SIZE / 1024, mirror.state, nor::teensy_nor_written(), mirror.sectors);
                    if mux && !direct { ring.packet(b'T', &[l.as_bytes()]); } else { send!(l.as_bytes()); }
                }
                Some(b'M') => {
                    mux = true;
                    frame_at = None;
                }
                Some(b'L') => {
                    mux = true;
                    link = true;
                    frame_at = None;
                    pk_len = 0;
                    sent_frames = 0;
                    played = 0;
                    synced = false;
                    z_at = 0; // 'Z' goes out from the loop below
                    fm1_seen = false;
                    fm1_gone = false;
                    unsafe { teensy_redraw() };
                }
                Some(b'm') => mux = false,
                Some(b'Q') => target = parse_int(line[1..].trim_ascii()).clamp(512, 8192) as u32,
                Some(b'V') => {
                    let v = parse_int(line[1..].trim_ascii()).max(0) as u32;
                    lab.master(v);
                    unsafe { web_master(v) }
                }
                Some(b'F') if mux => {
                    if frame_at.is_none() {
                        frame_at = Some(0);
                    }
                }
                _ => {}
            }
        }};
    }

    loop {
        let on = service!();
        if on && !listening {
            send!(bench.as_bytes());
        }
        if host.poll() {
            direct = true;
            command!(b"L");
        }
        // a ROM copy in progress: one step per pass (an erase is ~0.2 s), the screen redrawn as the percent moves
        #[cfg(fw_menu)]
        if let Some((ld, k, shown)) = loading.as_mut() {
            let (k, shown) = (*k, *shown);
            let ph = match lib.as_ref() { Some(l) => ld.step(l), None => roms::Phase::Failed("NO CARD") };
            let pct = ld.percent();
            let slot = ld.slot;
            match ph {
                roms::Phase::Done => {
                    draw_loading!(k, 100);
                    roms::stamp(slot);
                    if lab_after { reboot_to_lab(slot as u32) } else { reboot_to(slot as u32) };
                }
                roms::Phase::Failed(why) => {
                    loading = None;
                    cat.build(BUILTIN, lib.as_ref());
                    set_hint!("COPY FAILED ", why);
                    draw_menu!();
                }
                _ => {
                    if pct != shown {
                        draw_loading!(k, pct);
                        if let Some(x) = loading.as_mut() { x.2 = pct; }
                    }
                }
            }
            next = clock.now() + period;
        }
        // the save memory's written sectors onto the card: one (~20 ms) every 100 ms at most, the audio queue covers it
        #[cfg(not(fw_menu))]
        if !menu && !lab.open && clock.now() >= mirror_at && mirror.step() {
            mirror_at = clock.now() + hz / 10;
        }
        // the Jam Box's samples from their old place to the new one (once; hooks/jambox.rs Move), a sector a pass
        #[cfg(fw_hook_jambox)]
        if !menu && !lab.open && clock.now() >= mirror_at && smp_move.step() {
            mirror_at = clock.now() + hz / 10;
        }
        // the Lab's turn (the card, a send's flash step, the page), or the gate's held presses going to the mod
        if !menu {
            if lab.open {
                lab.step(card!(), ms!());
                if lab.broken {
                    reboot_to(0); // a load cut off half way: the mod's memory is part old, part new
                }
                if lab.close_now {
                    lab_close!(); // a state loaded: back to the mod, where it was
                }
            } else {
                gate.tick(ms!(), &mut |n, b| unsafe { teensy_raw_input(n, b) });
                // FX + HOME held: back to the menu (as switching the FM-1 off and on); let go early: nothing
                if let Some(t) = menu_hold {
                    let m = lab::to_matrix(lab::MENU);
                    if gate.raw().1 & m != m {
                        menu_hold = None;
                    } else if ms!().wrapping_sub(t) >= lab::MENU_HOLD_MS {
                        // a reset in the middle of a packet leaves the FM-1 half way through it, out of step (stuck
                        // on its splash): first hand USB what is queued, up to the packet's end, then stay quiet past
                        // the FM-1's 1 s link timeout so it starts over clean, then the menu
                        if direct {
                            let t0 = clock.now();
                            while (ring.used() > 0 || !host.can_write()) && clock.now() - t0 < hz {
                                host.poll();
                                let n = host.write(ring.readable());
                                ring.consume(n);
                            }
                            let t1 = clock.now();
                            while clock.now() - t1 < hz * 3 / 2 {
                                host.poll();
                            }
                        }
                        reboot_to(0);
                    }
                }
            }
        }
        if direct && !host.ready() {
            // the FM-1 was unplugged or switched off: the menu, as with the PC relay
            direct = false;
            if !menu {
                reboot_to(0);
            }
            mux = false;
            link = false;
            frame_at = None;
            ring.head = 0;
            ring.tail = 0;
        }
        if !on && !direct {
            mux = false;
            link = false;
            frame_at = None;
            ring.head = 0;
            ring.tail = 0;
        }
        listening = on;

        // mux mode: hand USB whatever it will take, never waiting
        if mux {
            let r = ring.readable();
            if !r.is_empty() {
                if direct {
                    let n = host.write(r);
                    ring.consume(n);
                } else if let Ok(n) = serial.write(r) {
                    ring.consume(n);
                }
            }
            // a piece of the screen, only while audio keeps a safe margin
            if let Some(at) = frame_at {
                if ring.free() > CHUNK + 16 + AUDIO_MARGIN {
                    let n = CHUNK.min(FB_BYTES - at);
                    let fb = unsafe { core::slice::from_raw_parts(teensy_fb() as *const u8, FB_BYTES) };
                    ring.packet(b'F', &[&(at as u32).to_le_bytes(), &fb[at..at + n]]);
                    frame_at = if at + n >= FB_BYTES { None } else { Some(at + n) };
                }
            }
            // link: the sync, what changed on the screen, and the LEDs
            if link && !synced && (z_at == 0 || clock.now() - z_at > hz / 2) && ring.free() > 16 {
                link_gen = link_gen.wrapping_add(1).max(1);
                sent_frames = 0;
                played = 0;
                ring.packet(b'Z', &[&[link_gen]]);
                z_at = clock.now();
            }
            if link {
                // screen data only while little is queued ahead: sound behind a deep queue of spans arrives late
                // (unsent spans stay as dirty rows here and merge: a busy screen updates less often, sound never waits)
                if ring.free() > CHUNK + 16 + AUDIO_MARGIN && TX - ring.free() < SPAN_BACKLOG {
                    let n = if lab.open {
                        lab.spans(&mut span_buf)
                    } else {
                        unsafe { teensy_spans(span_buf.as_mut_ptr(), CHUNK as u32) as usize }
                    };
                    if n > 0 {
                        ring.packet(b'L', &[&span_buf[..n]]);
                    }
                }
                if clock.now() - led_at > hz / 100 && ring.free() > 64 + AUDIO_MARGIN {
                    led_at = clock.now();
                    if unsafe { teensy_leds(led_buf.as_mut_ptr()) } != 0 {
                        ring.packet(b'D', &[&led_buf]);
                    }
                }
            }
        }

        if link && fm1_seen && !fm1_gone && clock.now() - last_any > 2 * hz {
            fm1_gone = true;
            // switched off: back to the menu now, so it is waiting when the FM-1 comes back on
            if !menu {
                reboot_to(0);
            }
        }

        // the Jam Box's track recording asked for the line in (trec.c): started here, never inside the render
        #[cfg(fw_rec)]
        if !menu && !lab.open && audio_in::stuck() {
            audio_in::restart(); // (the receiver stopped: no new samples while on)
        }
        #[cfg(fw_rec)]
        if !menu && !lab.open && audio_in::need() {
            audio_in::live_start();
        }
        #[cfg(fw_rec)]
        audio_in::OUT_LAT.store(if link { target } else { 2 * BLOCK }, core::sync::atomic::Ordering::Relaxed);

        // audio: on the Teensy's clock, or (link, with the FM-1 reporting) as the FM-1 plays
        let now = clock.now();
        let paced = link && synced && now - last_report < hz * 3 / 10;
        let due = if paced { sent_frames.wrapping_sub(played) < target } else { now >= next };
        if due {
            let a = clock.now();
            if lab.open {
                lab.render();
            } else if !menu {
                unsafe { web_render(BLOCK) };
            }
            let d = clock.now() - a;
            if mux {
                let (lp, rp) = if lab.open { lab.out_ptrs() } else { unsafe { (web_out_l(), web_out_r()) } };
                let (l, r) = unsafe {
                    (core::slice::from_raw_parts(lp, BLOCK as usize), core::slice::from_raw_parts(rp, BLOCK as usize))
                };
                let mut pcm = [0u8; BLOCK as usize * 4];
                for k in 0..BLOCK as usize * !menu as usize {
                    let cv = |x: f32| ((x * 65536.0).clamp(-32768.0, 32767.0) as i16).to_le_bytes(); // Felucca's Q15 (its DAC word >> 7)
                    pcm[k * 4..k * 4 + 2].copy_from_slice(&cv(l[k]));
                    pcm[k * 4 + 2..k * 4 + 4].copy_from_slice(&cv(r[k]));
                }
                if ring.packet(b'A', &[&pcm]) {
                    audio_sent += 1;
                    sent_frames = sent_frames.wrapping_add(BLOCK);
                } else {
                    dropped += 1;
                }
            }
            busy += d;
            worst = worst.max(d);
            blocks += 1;
            if paced {
                next = clock.now() + period;
            } else {
                next += period;
                if clock.now() > next {
                    late += 1;
                    next = clock.now() + period;
                }
            }
        }

        if now >= report_at {
            report_at += hz;
            secs += 1;
            if secs % 2 == 0 {
                led.toggle();
            }
            let (ui_now, px_now) = unsafe { (teensy_ui_frames(), teensy_pixels()) };
            if listening {
                let mut l = Line::new();
                let load = busy * 1000 / (blocks.max(1) as u64 * period);
                let _ = write!(
                    l,
                    "STAT t={}s load={}.{}% worst_block={}us (budget {}us) blocks={} late={} draws={} ui_fps={} panel_kb_s={} audio_sent={} audio_dropped={}",
                    secs,
                    load / 10,
                    load % 10,
                    worst * 1_000_000 / hz,
                    period * 1_000_000 / hz,
                    blocks,
                    late,
                    unsafe { teensy_draws() },
                    ui_now.wrapping_sub(ui_last),
                    px_now.wrapping_sub(px_last) * 2 / 1024,
                    audio_sent,
                    dropped,
                );
                if link {
                    let _ = write!(
                        l,
                        " link={} fm1_reports={} fm1_fifo={} fm1_underruns={} in_flight={} target={}",
                        if paced { "fm1-clock" } else { "teensy-clock" },
                        reports,
                        fm1_fifo,
                        fm1_under,
                        sent_frames.wrapping_sub(played),
                        target,
                    );
                }
                #[cfg(fw_rec)]
                {
                    let (fx, rs, lag) = audio_in::fixes();
                    let _ = write!(l, " line_in_ppm={} resyncs={} lag={} in_lat={}", fx, rs, lag, audio_in::teensy_in_latency());
                }
                let _ = write!(l, "\r\n");
                if mux {
                    ring.packet(b'T', &[l.as_bytes()]);
                    if direct && listening {
                        // a PC on the Teensy's own USB as well: the numbers there too (never waiting)
                        let mut h = Line::new();
                        let _ = write!(h, "HOST {:?} {:04x}:{:04x} in={} out={} errors={}\r\n", host.state, host.vid, host.pid,
                                       host.in_bytes, host.out_bytes, host.errors);
                        let _ = serial.write(l.as_bytes());
                        let _ = serial.write(h.as_bytes());
                    }
                } else {
                    send!(l.as_bytes());
                }
            }
            ui_last = ui_now;
            px_last = px_now;
            busy = 0;
            worst = 0;
            blocks = 0;
            audio_sent = 0;
            dropped = 0;
            reports = 0;
        }

        // input: command lines, or (link mode) packets
        // (direct: the FM-1's bytes from the host port are packets; the PC's, if one is there, plain commands)
        let mut hbuf = [0u8; usbh::IN_LEN];
        let hn = if direct { host.read(&mut hbuf) } else { 0 };
        let mut buf = [0u8; 64];
        let sn = serial.read(&mut buf).unwrap_or(0);
        {
            for (i, &c) in hbuf[..hn].iter().chain(buf[..sn].iter()).enumerate() {
                let as_link = if i < hn { true } else { link && !direct };
                if as_link {
                    // A5 5A type len_lo len_hi payload
                    if (pk_len == 0 && c != 0xA5) || (pk_len == 1 && c != 0x5A) {
                        pk_len = if c == 0xA5 { 1 } else { 0 };
                        continue;
                    }
                    pk[pk_len] = c;
                    pk_len += 1;
                    if pk_len >= 5 {
                        let len = pk[3] as usize | (pk[4] as usize) << 8;
                        if len > pk.len() - 5 {
                            pk_len = 0; // nonsense: look for the next packet
                        } else if pk_len == 5 + len {
                            let body = &pk[5..5 + len];
                            match pk[2] {
                                b'I' if len >= 30 => {
                                    let u32at = |o: usize| u32::from_le_bytes([body[o], body[o + 1], body[o + 2], body[o + 3]]);
                                    let u16at = |o: usize| u16::from_le_bytes([body[o], body[o + 1]]) as u32;
                                    if fm1_gone {
                                        // the FM-1 was away for 2 s and is back (switched off and on): the menu, as
                                        // a shared power switch will do it
                                        fm1_gone = false;
                                        if menu {
                                            unsafe { teensy_redraw() };
                                        } else {
                                            reboot_to(0);
                                        }
                                    }
                                    fm1_seen = true;
                                    last_any = clock.now();
                                    if menu {
                                        // keys and buttons pressed since the last report start the entry shown; a
                                        // knob moves it (what is held when the menu first hears the FM-1 does not count)
                                        let (nk, nb) = (u32at(0) & ((1 << 27) - 1), u32at(4));
                                        if let Some((pk_, pb)) = menu_in {
                                            if nk & !pk_ != 0 || nb & !pb != 0 {
                                                pick!(sel);
                                            }
                                        }
                                        menu_in = Some((nk, nb));
                                        let st: i32 = (0..7).map(|e| body[8 + e] as i8 as i32).sum();
                                        if menu && st != 0 {
                                            menu_move!(st);
                                        }
                                        // held through the start: not a press for the firmware
                                        last_in.0 = u32at(0);
                                        last_in.1 = u32at(4);
                                    }
                                    // on change only, so the PC's commands ('C') are not overwritten every 4 ms
                                    if !menu && (u32at(0), u32at(4)) != (last_in.0, last_in.1) {
                                        raw_input!(u32at(0), u32at(4));
                                    }
                                    for e in 0..7 {
                                        let s = body[8 + e] as i8;
                                        if s != 0 && !menu {
                                            if lab.open {
                                                lab.raw_enc(card!(), e as u32, s as i32, ms!());
                                            } else {
                                                unsafe { teensy_raw_enc(e as u32, s as i32) };
                                            }
                                        }
                                    }
                                    unsafe {
                                        let adc = u16at(15) | u16at(17) << 16;
                                        if adc != last_in.2 {
                                            teensy_raw_adc(u16at(15), u16at(17));
                                            lab.master(u16at(15));
                                        }
                                        last_in = (u32at(0), u32at(4), adc);
                                    }
                                    fm1_under = u16at(23);
                                    fm1_fifo = u16at(25);
                                    if len >= 30 && body[29] == link_gen {
                                        synced = true;
                                        played = u32at(19);
                                        last_report = clock.now();
                                    }
                                    reports += 1;
                                }
                                b'R' => {
                                    unsafe { teensy_redraw() };
                                    lab.redraw();
                                }
                                b'C' => {
                                    let mut line = [0u8; 64];
                                    let k = len.min(64);
                                    line[..k].copy_from_slice(&body[..k]);
                                    command!(line[..k].trim_ascii());
                                }
                                _ => {}
                            }
                            pk_len = 0;
                        }
                    }
                } else if c == b'\n' || c == b'\r' {
                    let mut line = [0u8; 64];
                    line[..cmd_len].copy_from_slice(&cmd[..cmd_len]);
                    let k = cmd_len;
                    cmd_len = 0;
                    command!(&line[..k]);
                } else if cmd_len < cmd.len() {
                    cmd[cmd_len] = c;
                    cmd_len += 1;
                }
            }
        }
    }
}

/// A panic blinks the LED fast forever (USB is gone by then).
#[panic_handler]
fn panic(_: &PanicInfo) -> ! {
    const GPIO2_DR: *mut u32 = 0x401B_C000 as *mut u32;
    loop {
        unsafe {
            let v = core::ptr::read_volatile(GPIO2_DR);
            core::ptr::write_volatile(GPIO2_DR, v ^ (1 << 3));
        }
        cortex_m::asm::delay(30_000_000);
    }
}

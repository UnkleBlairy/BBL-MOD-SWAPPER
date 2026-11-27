//! USB host on the Teensy 4.1's host port (USB2, EHCI), for one device: the FM-1 running the Teensy terminal
//! (pid.codes 1209:000B, full speed, CDC ACM data on bulk EP3). With it the FM-1 plugs straight into the Teensy,
//! with no PC relaying.
//!
//! Polled from the main loop, no interrupts. Enumeration blocks for ~0.3 s once when the FM-1 is plugged in
//! (or switched on); after that `write` and `read` never wait: one bulk OUT and one bulk IN transfer in flight.
//!
//! EHCI structures and buffers (~5 KB) are statics, so they sit in DTCM (.bss, build.rs): not cached, and reachable by the
//! USB controller's DMA, so no cache maintenance is needed.
//! Register offsets: the i.MX RT1060 reference manual, USB chapter (the ChipIdea EHCI host).

use core::ptr::{addr_of, addr_of_mut, read_volatile as rd, write_volatile as wr};

const USB2: usize = 0x402E_0200;
const USBCMD: usize = USB2 + 0x140;
const USBSTS: usize = USB2 + 0x144;
const USBINTR: usize = USB2 + 0x148;
const ASYNCLISTADDR: usize = USB2 + 0x158;
const BURSTSIZE: usize = USB2 + 0x160;
const PORTSC1: usize = USB2 + 0x184;
const USBMODE: usize = USB2 + 0x1A8;
const USBPHY2_PWD: usize = 0x400D_A000;
const USBPHY2_CTRL_SET: usize = 0x400D_A034;
const USBPHY2_CTRL_CLR: usize = 0x400D_A038;
const PLL_USB2: usize = 0x400D_8020;
const PLL_USB2_SET: usize = 0x400D_8024;
const PLL_USB2_CLR: usize = 0x400D_8028;
const CCM_CCGR6: usize = 0x400F_C080;

// PORTSC1 bits
const CCS: u32 = 1 << 0; // a device is there
const CSC: u32 = 1 << 1; // connect change (write 1 to clear)
const PE: u32 = 1 << 2; // enabled
const PEC: u32 = 1 << 3;
const OCC: u32 = 1 << 5;
const PR: u32 = 1 << 8; // reset
const PP: u32 = 1 << 12; // power
const W1C: u32 = CSC | PEC | OCC | PE; // writing these back as 1 would clear them (PE: disables the port)

// qTD token
const ACTIVE: u32 = 0x80;
const HALTED: u32 = 0x40;
const ERRS: u32 = 0x7C; // halted, buffer, babble, transaction, missed micro-frame
const PID_OUT: u32 = 0 << 8;
const PID_IN: u32 = 1 << 8;
const PID_SETUP: u32 = 2 << 8;
const CERR3: u32 = 3 << 10;
const T: u32 = 1; // terminate

#[repr(C, align(64))]
struct Qh {
    next: u32,
    chars: u32,
    caps: u32,
    cur: u32,
    // overlay
    o_next: u32,
    o_alt: u32,
    o_token: u32,
    o_buf: [u32; 5],
    o_ext: [u32; 5],
}
#[repr(C, align(32))]
#[derive(Clone, Copy)]
struct Qtd {
    next: u32,
    alt: u32,
    token: u32,
    buf: [u32; 5],
}
const QTD0: Qtd = Qtd { next: T, alt: T, token: 0, buf: [0; 5] };
const QH0: Qh = Qh { next: T, chars: 0, caps: 0, cur: 0, o_next: T, o_alt: T, o_token: 0, o_buf: [0; 5], o_ext: [0; 5] };

#[repr(C, align(32))]
struct Buf<const N: usize>([u8; N]);

// 0: control (EP0), 1: bulk IN, 2: bulk OUT. A ring: 0 -> 1 -> 2 -> 0, 0 the head.
static mut QH: [Qh; 3] = [QH0, QH0, QH0];
static mut CTD: [Qtd; 3] = [QTD0; 3]; // setup, data, status
static mut ITD: Qtd = QTD0;
static mut OTD: Qtd = QTD0;
static mut CBUF: Buf<256> = Buf([0; 256]); // control data
static mut IBUF: Buf<IN_LEN> = Buf([0; IN_LEN]); // bulk IN (one transfer: up to IN_LEN)
static mut OBUF: Buf<OUT_LEN> = Buf([0; OUT_LEN]); // bulk OUT (one transfer: up to OUT_LEN)
static mut SETUP: [u8; 8] = [0; 8];

pub const IN_LEN: usize = 512;
pub const OUT_LEN: usize = 4096;

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum State {
    Off,       // not started
    NoDevice,  // waiting for a device
    Ready,     // the FM-1 terminal, configured, DTR on
    Failed(u8), // enumeration failed at this step (see `step`); retried after unplug or every 2 s
}

pub struct Host {
    pub state: State,
    in_busy: bool,
    out_busy: bool,
    in_ep: u8,
    out_ep: u8,
    maxp: u16,
    pub vid: u16,
    pub pid: u16,
    pub errors: u32, // transfer errors (bulk) since start
    pub in_bytes: u64,
    pub out_bytes: u64,
    retry_at: u64,
}

/// Milliseconds since start, from the cycle counter (main enables it); wraps handled while called every < 7 s.
fn ms() -> u64 {
    static mut LAST: u32 = 0;
    static mut HI: u64 = 0;
    unsafe {
        let c = rd(0xE000_1004 as *const u32); // DWT CYCCNT
        if c < LAST {
            HI += 1 << 32;
        }
        LAST = c;
        (HI + c as u64) / 600_000
    }
}

fn delay_ms(ms: u32) {
    cortex_m::asm::delay(600_000 * ms);
}
fn addr<T>(p: *const T) -> u32 {
    p as u32
}

impl Host {
    /// Power the host port and start the controller.
    pub fn new() -> Host {
        unsafe { init() };
        Host {
            state: State::NoDevice,
            in_busy: false,
            out_busy: false,
            in_ep: 0,
            out_ep: 0,
            maxp: 64,
            vid: 0,
            pid: 0,
            errors: 0,
            in_bytes: 0,
            out_bytes: 0,
            retry_at: 0,
        }
    }

    pub fn ready(&self) -> bool {
        self.state == State::Ready
    }

    /// Call often: notices plug and unplug, enumerates. Returns true when the device just became ready.
    pub fn poll(&mut self) -> bool {
        let p = unsafe { rd(PORTSC1 as *const u32) };
        if p & CSC != 0 {
            unsafe { wr(PORTSC1 as *mut u32, (p & !W1C) | CSC) };
            if p & CCS == 0 {
                self.gone();
                return false;
            }
        }
        if p & CCS == 0 {
            if self.state != State::NoDevice {
                self.gone();
            }
            return false;
        }
        match self.state {
            State::Ready => {
                if p & PE == 0 {
                    // the port was disabled (an error, or the device went away and came back)
                    self.gone();
                }
                false
            }
            State::Failed(_) if ms() < self.retry_at => false,
            _ => match unsafe { self.enumerate() } {
                Ok(()) => {
                    self.state = State::Ready;
                    true
                }
                Err(step) => {
                    self.state = State::Failed(step);
                    self.retry_at = ms() + 2000;
                    false
                }
            },
        }
    }

    fn gone(&mut self) {
        unsafe {
            async_off();
            QH[1].o_next = T;
            QH[1].o_token = 0;
            QH[2].o_next = T;
            QH[2].o_token = 0;
            ITD.token = 0;
            OTD.token = 0;
            async_on();
        }
        self.in_busy = false;
        self.out_busy = false;
        self.state = State::NoDevice;
    }

    /// Bytes from the FM-1 since the last call (0 if none yet). Keeps one IN transfer queued.
    pub fn read(&mut self, out: &mut [u8]) -> usize {
        if !self.ready() {
            return 0;
        }
        let mut got = 0;
        unsafe {
            if self.in_busy {
                let tok = rd(addr_of!(ITD.token));
                if tok & ACTIVE != 0 {
                    return 0;
                }
                self.in_busy = false;
                if tok & ERRS != 0 {
                    self.errors += 1;
                    unhalt(1);
                } else {
                    let left = ((tok >> 16) & 0x7FFF) as usize;
                    let n = (IN_LEN - left).min(out.len());
                    let src = addr_of!(IBUF.0) as *const u8;
                    core::ptr::copy_nonoverlapping(src, out.as_mut_ptr(), n);
                    got = n;
                    self.in_bytes += n as u64;
                }
            }
            if !self.in_busy {
                queue(1, addr_of_mut!(ITD), PID_IN, addr_of!(IBUF.0) as u32, IN_LEN);
                self.in_busy = true;
            }
        }
        got
    }

    /// True when a new `write` can start (the last OUT transfer is done).
    pub fn can_write(&mut self) -> bool {
        if !self.ready() {
            return false;
        }
        if self.out_busy {
            let tok = unsafe { rd(addr_of!(OTD.token)) };
            if tok & ACTIVE != 0 {
                return false;
            }
            self.out_busy = false;
            if tok & ERRS != 0 {
                self.errors += 1;
                unsafe { unhalt(2) };
            }
        }
        true
    }

    /// Starts sending up to OUT_LEN bytes; returns how many were taken (0: busy or no device).
    pub fn write(&mut self, data: &[u8]) -> usize {
        if !self.can_write() || data.is_empty() {
            return 0;
        }
        let n = data.len().min(OUT_LEN);
        unsafe {
            let dst = addr_of_mut!(OBUF.0) as *mut u8;
            core::ptr::copy_nonoverlapping(data.as_ptr(), dst, n);
            queue(2, addr_of_mut!(OTD), PID_OUT, dst as u32, n);
        }
        self.out_busy = true;
        self.out_bytes += n as u64;
        n
    }

    unsafe fn enumerate(&mut self) -> Result<(), u8> {
        self.in_busy = false;
        self.out_busy = false;
        delay_ms(100); // let the device settle after plugging in (USB 2.0: 100 ms)
        // port reset, 50 ms
        let p = rd(PORTSC1 as *const u32);
        wr(PORTSC1 as *mut u32, (p & !W1C) | PR);
        delay_ms(50);
        let p = rd(PORTSC1 as *const u32);
        wr(PORTSC1 as *mut u32, p & !W1C & !PR);
        let mut i = 0;
        while rd(PORTSC1 as *const u32) & PR != 0 {
            i += 1;
            if i > 1000 {
                return Err(1);
            }
            delay_ms(1);
        }
        delay_ms(20);
        let p = rd(PORTSC1 as *const u32);
        if p & PE == 0 {
            return Err(2);
        }
        let speed = (p >> 26) & 3; // 0 full, 1 low, 2 high
        let eps = match speed {
            0 => 0,
            1 => 1,
            _ => 2,
        };
        // EP0 at address 0, max packet 8 until the device says
        async_off();
        setup_qh(0, 0, 0, 8, eps, true);
        async_on();
        let mut d = [0u8; 18];
        self.ctrl([0x80, 6, 0, 1, 0, 0, 8, 0], Some(&mut d[..8]), 3)?;
        self.maxp = d[7].max(8) as u16;
        self.ctrl([0x00, 5, 1, 0, 0, 0, 0, 0], None, 4)?; // SET_ADDRESS 1
        delay_ms(10);
        async_off();
        setup_qh(0, 1, 0, self.maxp, eps, true);
        async_on();
        self.ctrl([0x80, 6, 0, 1, 0, 0, 18, 0], Some(&mut d), 5)?;
        self.vid = u16::from_le_bytes([d[8], d[9]]);
        self.pid = u16::from_le_bytes([d[10], d[11]]);
        let mut c = [0u8; 255];
        self.ctrl([0x80, 6, 0, 2, 0, 0, 9, 0], Some(&mut c[..9]), 6)?;
        let total = (u16::from_le_bytes([c[2], c[3]]) as usize).min(255);
        let cfg = c[5];
        self.ctrl([0x80, 6, 0, 2, 0, 0, total as u8, 0], Some(&mut c[..total]), 7)?;
        // the CDC communication interface (class 2) and the data interface's (class 0x0A) bulk endpoints
        let (mut comm, mut cur_class, mut i_ep, mut o_ep, mut mp) = (0xFFu8, 0u8, 0u8, 0u8, 64u16);
        let mut k = 0;
        while k + 2 <= total && c[k] >= 2 {
            let (len, ty) = (c[k] as usize, c[k + 1]);
            if k + len > total {
                break;
            }
            if ty == 4 {
                cur_class = c[k + 5];
                if cur_class == 2 && comm == 0xFF {
                    comm = c[k + 2];
                }
            } else if ty == 5 && cur_class == 0x0A && c[k + 3] & 3 == 2 {
                mp = u16::from_le_bytes([c[k + 4], c[k + 5]]) & 0x7FF;
                if c[k + 2] & 0x80 != 0 {
                    i_ep = c[k + 2] & 0x0F;
                } else {
                    o_ep = c[k + 2] & 0x0F;
                }
            }
            k += len;
        }
        if i_ep == 0 || o_ep == 0 {
            return Err(8);
        }
        self.in_ep = i_ep;
        self.out_ep = o_ep;
        self.ctrl([0x00, 9, cfg, 0, 0, 0, 0, 0], None, 9)?; // SET_CONFIGURATION
        if comm != 0xFF {
            // SET_LINE_CODING 115200 8N1, then SET_CONTROL_LINE_STATE: DTR + RTS (the terminal only writes with DTR)
            let mut lc = [0x00, 0xC2, 0x01, 0x00, 0, 0, 8];
            let _ = self.ctrl_out([0x21, 0x20, 0, 0, comm, 0, 7, 0], &mut lc, 10);
            self.ctrl([0x21, 0x22, 3, 0, comm, 0, 0, 0], None, 11)?;
        }
        async_off();
        setup_qh(1, 1, i_ep, mp, eps, false);
        setup_qh(2, 1, o_ep, mp, eps, false);
        async_on();
        Ok(())
    }

    /// A control transfer with an IN data stage (or none).
    unsafe fn ctrl(&mut self, s: [u8; 8], data: Option<&mut [u8]>, step: u8) -> Result<(), u8> {
        let n = data.as_ref().map_or(0, |d| d.len());
        self.ctrl_raw(s, n, true, step)?;
        if let Some(d) = data {
            let src = addr_of!(CBUF.0) as *const u8;
            core::ptr::copy_nonoverlapping(src, d.as_mut_ptr(), n);
        }
        Ok(())
    }
    /// A control transfer with an OUT data stage.
    unsafe fn ctrl_out(&mut self, s: [u8; 8], data: &mut [u8], step: u8) -> Result<(), u8> {
        let dst = addr_of_mut!(CBUF.0) as *mut u8;
        core::ptr::copy_nonoverlapping(data.as_ptr(), dst, data.len());
        self.ctrl_raw(s, data.len(), false, step)
    }
    unsafe fn ctrl_raw(&mut self, s: [u8; 8], n: usize, dir_in: bool, step: u8) -> Result<(), u8> {
        let sp = addr_of_mut!(SETUP) as *mut u8;
        core::ptr::copy_nonoverlapping(s.as_ptr(), sp, 8);
        let td = addr_of_mut!(CTD) as *mut Qtd;
        let (t0, t1, t2) = (td, td.add(1), td.add(2));
        let status_pid = if n > 0 && dir_in { PID_OUT } else { PID_IN };
        // status: toggle 1, the other direction, interrupt-on-complete not needed (polled)
        fill(t2, T, (1 << 31) | CERR3 | status_pid | ACTIVE, 0, 0);
        let after_setup = if n > 0 {
            fill(t1, addr(t2), (1 << 31) | ((n as u32) << 16) | CERR3 | if dir_in { PID_IN } else { PID_OUT } | ACTIVE,
                 addr_of!(CBUF.0) as u32, n);
            addr(t1)
        } else {
            addr(t2)
        };
        fill(t0, after_setup, (8 << 16) | CERR3 | PID_SETUP | ACTIVE, sp as u32, 8);
        let qh = addr_of_mut!(QH[0]);
        wr(addr_of_mut!((*qh).o_token), 0);
        wr(addr_of_mut!((*qh).o_next), addr(t0));
        // wait for the status stage (or an error anywhere), up to 500 ms
        let t_end = ms() + 500;
        loop {
            let (a, b, c) = (rd(addr_of!((*t0).token)), rd(addr_of!((*t1).token)), rd(addr_of!((*t2).token)));
            let bad = a & ERRS != 0 || (n > 0 && b & ERRS != 0) || c & ERRS != 0;
            if bad {
                unhalt(0);
                return Err(step);
            }
            if c & ACTIVE == 0 {
                return Ok(());
            }
            if ms() > t_end {
                return Err(step | 0x80);
            }
        }
    }
}

unsafe fn fill(td: *mut Qtd, next: u32, token: u32, buf: u32, _n: usize) {
    wr(addr_of_mut!((*td).next), next);
    wr(addr_of_mut!((*td).alt), T);
    let b = addr_of_mut!((*td).buf) as *mut u32;
    for i in 0..5 {
        // pointer 0: the start; 1..4: the following 4 KB pages
        let v = if i == 0 { buf } else { (buf & !0xFFF).wrapping_add(0x1000 * i as u32) };
        wr(b.add(i), v);
    }
    wr(addr_of_mut!((*td).token), token);
}

/// One bulk transfer on QH `q` (1 IN, 2 OUT), the toggle kept by the QH.
unsafe fn queue(q: usize, td: *mut Qtd, pid: u32, buf: u32, n: usize) {
    fill(td, T, ((n as u32) << 16) | CERR3 | pid | ACTIVE, buf, n);
    let qh = addr_of_mut!(QH[q]);
    // the QH is idle (its last qTD is done): point its overlay at the new one, the HC fetches it next visit
    wr(addr_of_mut!((*qh).o_next), addr(td));
}

/// After an error the QH halts: clear it so the next transfer runs (the toggle is kept).
unsafe fn unhalt(q: usize) {
    let qh = addr_of_mut!(QH[q]);
    let t = rd(addr_of!((*qh).o_token));
    wr(addr_of_mut!((*qh).o_token), t & !(HALTED | ACTIVE | ERRS) & !0x7FFF_0000);
    wr(addr_of_mut!((*qh).o_next), T);
}

/// QH `q`: device `dev`, endpoint `ep`, max packet `mp`, speed `eps` (0 full, 1 low, 2 high).
unsafe fn setup_qh(q: usize, dev: u8, ep: u8, mp: u16, eps: u32, control: bool) {
    let qh = addr_of_mut!(QH[q]);
    let head = if q == 0 { 1 << 15 } else { 0 };
    let ctl = if control && eps != 2 { 1 << 27 } else { 0 }; // control endpoint flag (full/low speed only)
    let dtc = if control { 1 << 14 } else { 0 }; // control: the toggle comes from each qTD
    let rl = 0; // NAK reload off: a NAKed IN is simply retried on the next pass
    wr(addr_of_mut!((*qh).chars), (rl << 28) | ctl | ((mp as u32) << 16) | head | dtc | (eps << 12) | ((ep as u32) << 8) | dev as u32);
    wr(addr_of_mut!((*qh).caps), 1 << 30); // one transaction per micro-frame; the RT's built-in TT: no hub/port
    wr(addr_of_mut!((*qh).cur), 0);
    wr(addr_of_mut!((*qh).o_next), T);
    wr(addr_of_mut!((*qh).o_alt), T);
    wr(addr_of_mut!((*qh).o_token), 0);
}

unsafe fn async_off() {
    let c = rd(USBCMD as *const u32);
    wr(USBCMD as *mut u32, c & !(1 << 5));
    let mut i = 0;
    while rd(USBSTS as *const u32) & (1 << 15) != 0 && i < 100_000 {
        i += 1;
    }
}
unsafe fn async_on() {
    let c = rd(USBCMD as *const u32);
    wr(USBCMD as *mut u32, c | (1 << 5));
}

unsafe fn init() {
    // clock gate (usboh3, shared with the device port)
    wr(CCM_CCGR6 as *mut u32, rd(CCM_CCGR6 as *const u32) | 3);
    // USB2 PLL (480 MHz), as PJRC's USBHost_t36 brings it up
    for _ in 0..1_000_000 {
        let n = rd(PLL_USB2 as *const u32);
        if n & (1 << 1) != 0 {
            wr(PLL_USB2_CLR as *mut u32, 0xC000);
            wr(PLL_USB2_SET as *mut u32, 1 << 16);
            wr(PLL_USB2_CLR as *mut u32, (1 << 12) | (1 << 1) | (1 << 13) | (1 << 6));
            continue;
        }
        if n & (1 << 13) == 0 {
            wr(PLL_USB2_SET as *mut u32, 1 << 13);
            continue;
        }
        if n & (1 << 12) == 0 {
            wr(PLL_USB2_SET as *mut u32, 1 << 12);
            continue;
        }
        if n & (1 << 31) == 0 {
            continue;
        }
        if n & (1 << 16) != 0 {
            wr(PLL_USB2_CLR as *mut u32, 1 << 16);
            continue;
        }
        if n & (1 << 6) == 0 {
            wr(PLL_USB2_SET as *mut u32, 1 << 6);
            continue;
        }
        break;
    }
    wr(BURSTSIZE as *mut u32, 0x0404);
    // the PHY on
    wr(USBPHY2_CTRL_CLR as *mut u32, (1 << 31) | (1 << 30));
    wr(USBPHY2_CTRL_SET as *mut u32, (1 << 14) | (1 << 15));
    wr(USBPHY2_PWD as *mut u32, 0);
    // the host port's 5 V: the Teensy 4.1 switches it with GPIO_EMC_40 (GPIO3/GPIO8 bit 26, whichever is muxed)
    wr(0x401F_80B4 as *mut u32, 5); // IOMUXC mux EMC_40: GPIO
    wr(0x401F_82A4 as *mut u32, 0x0008);
    for base in [0x401C_0000usize, 0x4200_8000] {
        wr((base + 4) as *mut u32, rd((base + 4) as *const u32) | 1 << 26); // GDIR
        wr(base as *mut u32, rd(base as *const u32) | 1 << 26); // DR
    }
    delay_ms(10);
    // reset the controller, host mode
    wr(USBCMD as *mut u32, 1 << 1);
    let mut i = 0;
    while rd(USBCMD as *const u32) & (1 << 1) != 0 && i < 1_000_000 {
        i += 1;
    }
    wr(USBMODE as *mut u32, 3);
    wr(USBINTR as *mut u32, 0);
    // the async ring: QH0 (head) -> QH1 -> QH2 -> QH0, all idle
    let q = addr_of_mut!(QH) as *mut Qh;
    for k in 0..3 {
        let next = q.add((k + 1) % 3);
        wr(addr_of_mut!((*q.add(k)).next), addr(next) | 2); // type QH
    }
    setup_qh(0, 0, 0, 8, 0, true);
    setup_qh(1, 0, 1, 64, 0, false);
    setup_qh(2, 0, 2, 64, 0, false);
    wr(ASYNCLISTADDR as *mut u32, addr(q));
    // run, async schedule on (no periodic schedule: bulk and control only), interrupt threshold 1 micro-frame
    wr(USBCMD as *mut u32, (1 << 16) | (1 << 5) | 1);
    wr(PORTSC1 as *mut u32, (rd(PORTSC1 as *const u32) & !W1C) | PP);
}

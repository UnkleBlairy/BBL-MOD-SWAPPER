//! A mod without a sample hook has no Sample Lab: this stands in for lab.rs (nothing opens; input passes straight
//! through to the mod, as before the Lab existed).
pub const COMBO: u32 = 1 | 1 << 5;
pub const SAVES: u32 = 1 | 1 << 7;
pub const MENU: u32 = 1 | 1 << 6; // FX + HOME, as lab.rs: held MENU_HOLD_MS, back to the firmware menu
pub const MENU_HOLD_MS: u32 = 1000;
const BTN_MATRIX: [u8; 14] = [2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 0, 1];
pub fn to_matrix(labels: u32) -> u32 {
    (0..14).filter(|b| labels >> b & 1 != 0).fold(0, |m, b| m | 1 << BTN_MATRIX[b])
}
pub struct Lab {
    pub open: bool,
    pub close_now: bool,
    pub broken: bool,
}
impl Lab {
    pub const fn new() -> Lab {
        Lab { open: false, close_now: false, broken: false }
    }
    pub fn open(&mut self, _: (), _: u32, _: (u32, u32)) {}
    pub fn master(&mut self, _: u32) {}
    pub fn redraw(&mut self) {}
    pub fn open_saves(&mut self, _: (), _: u32, _: (u32, u32)) {}
    pub fn close(&mut self, _: ()) {}
    pub fn step(&mut self, _: (), _: u32) {}
    pub fn render(&mut self) {}
    pub fn out_ptrs(&self) -> (*const f32, *const f32) {
        (core::ptr::null(), core::ptr::null())
    }
    pub fn spans(&mut self, _: &mut [u8]) -> usize {
        0
    }
    pub fn row_bytes(&self, _: u32, _: &mut [u8; 480]) {}
    pub fn status(&self, out: &mut impl core::fmt::Write) {
        let _ = out.write_str("LAB none: this mod has no sample hook\r\n");
    }
    pub fn raw(&mut self, _: (), _: u32, _: u32, _: u32) -> bool {
        false
    }
    pub fn raw_enc(&mut self, _: (), _: u32, _: i32, _: u32) {}
    pub fn buttons(&mut self, _: (), _: u32, _: u32) -> bool {
        false
    }
    pub fn keys_in(&mut self, _: (), _: u32, _: u32) {}
    pub fn enc(&mut self, _: (), _: u32, _: i32, _: u32) {}
}
/// Input passes straight through; it only notices FX + HOME going down (3), for the way back to the menu.
pub struct Gate {
    raw: (u32, u32),
}
impl Gate {
    pub const fn new() -> Gate {
        Gate { raw: (0, 0) }
    }
    pub fn input(&mut self, n: u32, b: u32, _: u32, out: &mut dyn FnMut(u32, u32)) -> u8 {
        let m = to_matrix(MENU);
        let was = self.raw.1;
        self.raw = (n, b);
        out(n, b);
        if b & m == m && was & m != m { 3 } else { 0 }
    }
    pub fn raw(&self) -> (u32, u32) {
        self.raw
    }
    pub fn tick(&mut self, _: u32, _: &mut dyn FnMut(u32, u32)) {}
    pub fn closed(&mut self, _: &mut dyn FnMut(u32, u32)) {}
}

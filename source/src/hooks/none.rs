//! No sample hook: the mod has no sample slots (the overlay has save states only, no Sample Lab page).
pub const MAX: usize = 0;
pub const RATE: u32 = 22050;

pub fn label(_slot: usize) -> ([u8; 4], usize) {
    (*b"----", 4)
}

#[derive(Clone, Copy)]
pub struct Table {
    pub n: usize,
}
impl Table {
    pub const EMPTY: Table = Table { n: 0 };
    pub fn read() -> Table {
        Table::EMPTY
    }
    pub fn full(&self, _slot: usize) -> bool {
        false
    }
    pub fn top(&self) -> usize {
        0
    }
    pub fn room(&self, _slot: usize) -> u32 {
        0
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Phase {
    Done,
    Failed(&'static str),
}

pub struct Put {
    pub n: u32,
}
impl Put {
    pub const fn idle() -> Put {
        Put { n: 0 }
    }
    pub fn plan(&mut self, _slot: usize, _name: &[u8], _rate: u32, _want: u32) -> Result<(), &'static str> {
        Err("NO SAMPLE SLOTS")
    }
    pub fn percent(&self) -> u32 {
        100
    }
    pub fn step(&mut self, _pcm: &mut dyn FnMut(&mut [i16]) -> usize) -> Phase {
        Phase::Failed("NO SAMPLE SLOTS")
    }
}

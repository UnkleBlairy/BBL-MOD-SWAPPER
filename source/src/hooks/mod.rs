//! Sample hooks: how the one Sample Lab (lab.rs, part of the Mod Swapper runtime) talks to the mod that is loaded.
//! A mod with sample slots has a hook here; build.rs picks it (cfg fw_hook_<name>) and switches the Lab on (fw_lab).
//! A mod without one has no Lab: FX + HOME + GLO + REC does nothing special there.
//!
//! What a hook gives the Lab (see jambox.rs):
//!   `MAX`          how many slots the mod has
//!   `RATE`         the highest sample rate it stores (the Lab converts down to it, never up)
//!   `label(slot)`  the slot's name on screen (up to 4 characters: S01, USR1)
//!   `Table::read()` the slots now: `n` (debug), `full(slot)`: it holds a sound; `top()`: the last slot ALGORITHM
//!                  can pick; `room(slot)`: the samples a sound sent to that slot can have
//!   `Put`          a sound into a slot: `plan(slot, name, rate, samples)` (nothing written yet; `n` is what will
//!                  fit), then `step(pcm)` until `Phase::Done` / `Phase::Failed`, one short flash step each (the main
//!                  loop keeps the FM-1 link alive between them); `pcm` hands it 16-bit mono samples at `rate` as it
//!                  asks for them. At Done the mod has already read its slots again.
#[cfg(fw_hook_jambox)]
pub mod jambox;
#[cfg(fw_hook_jambox)]
pub use jambox as active;
#[cfg(fw_hook_sloop)]
pub mod sloop;
#[cfg(fw_hook_sloop)]
pub use sloop as active;
#[cfg(fw_hook_none)]
pub mod none;
#[cfg(fw_hook_none)]
pub use none as active;

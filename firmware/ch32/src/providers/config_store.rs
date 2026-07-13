//! ConfigStore provider (protocol sec 9.4): the two CONFIG flash slots with
//! A/B alternation, plus the boot-time overlay. Both `save` and `wipe` are
//! blocking -- the CPU fetch-stalls for every erase/program while code runs
//! from flash -- which is exactly the protocol sec 9.4 contract (dispatch's
//! torque gate makes the stall safe, the post-completion ack makes it visible).

use core::cell::SyncUnsafeCell;

use osc_core::persist::{self, CONFIG_LEN, HEADER_LEN, IMAGE_LEN, PROFILE_LEN, Slot, StoreError};

use crate::hal::flash;
use crate::runtime::statics::SHARED;

/// Slot bases come from this crate's `osc-config.x` fragment (shipped into
/// the link search path by build.rs; the board binary passes
/// `-Tosc-config.x`), resolved against the board's CONFIG_A/B regions -- the
/// flash layout has exactly one home. The target gate is deterministic host
/// hygiene: an ungated extern symbol links on the host only while no pulled
/// codegen unit references it, which is CGU-partition luck; host builds
/// (unit tests) never call the store, so they get a stub instead of a
/// link-time coupling.
#[cfg(target_arch = "riscv32")]
fn slot_addr(slot: Slot) -> u32 {
    unsafe extern "C" {
        static _config_a: u8;
        static _config_b: u8;
    }
    match slot {
        Slot::A => (&raw const _config_a) as u32,
        Slot::B => (&raw const _config_b) as u32,
    }
}

#[cfg(not(target_arch = "riscv32"))]
fn slot_addr(_slot: Slot) -> u32 {
    unimplemented!("host builds never touch the flash store")
}

/// The slot's stored image bytes, memory-mapped.
fn slot_bytes(slot: Slot) -> &'static [u8] {
    // SAFETY: memory.x reserves both windows inside main flash, which is
    // always readable and never holds code.
    unsafe { core::slice::from_raw_parts(slot_addr(slot) as *const u8, IMAGE_LEN) }
}

struct State {
    next_slot: Slot,
    next_seq: u16,
}

pub struct ConfigStore {
    /// Written by `boot_load` (pre-IRQ), then only from HIGH dispatch (the
    /// SESSION exclusivity invariant, `runtime::isr`) -- never concurrent.
    state: SyncUnsafeCell<State>,
}

static CONFIG_STORE: ConfigStore = ConfigStore {
    state: SyncUnsafeCell::new(State {
        next_slot: Slot::A,
        next_seq: 1,
    }),
};

impl ConfigStore {
    /// Boot-time load: overlay the newest valid image onto the (already
    /// default-seeded) table, prime the A/B state, and seed the store into
    /// `SHARED`. Bringup-only, pre-IRQ; sole writer (the
    /// `seed_config_defaults` contract).
    pub fn boot_load() {
        let pick = persist::boot_overlay(&SHARED.table, slot_bytes(Slot::A), slot_bytes(Slot::B));
        // SAFETY: pre-IRQ sole writer, see fn doc.
        unsafe {
            *CONFIG_STORE.state.get() = State {
                next_slot: pick.next_slot,
                next_seq: pick.next_seq,
            }
        };
        SHARED.seed_store(&CONFIG_STORE);
        crate::log::debug!(
            "config store: loaded={} next_seq={}",
            pick.loaded,
            pick.next_seq,
        );
    }
}

impl osc_core::ConfigStore for ConfigStore {
    /// Erase the older slot, stream header + regions straight into the page
    /// buffer (no staging copy), then readback-verify -- the verify is what
    /// lets the ack mean durable.
    fn save(
        &self,
        config: &[u8; CONFIG_LEN],
        profile: &[u8; PROFILE_LEN],
    ) -> Result<(), StoreError> {
        // SAFETY: HIGH-dispatch exclusive after boot, see the field doc.
        let state = unsafe { &mut *self.state.get() };
        let addr = slot_addr(state.next_slot);
        let header = persist::header(state.next_seq, config, profile);
        flash::erase(addr);
        flash::write(addr, &[&header, config, profile]);

        let stored = slot_bytes(state.next_slot);
        let intact = stored[..HEADER_LEN] == header
            && stored[HEADER_LEN..HEADER_LEN + CONFIG_LEN] == config[..]
            && stored[HEADER_LEN + CONFIG_LEN..] == profile[..];
        if !intact {
            return Err(StoreError);
        }
        state.next_slot = state.next_slot.other();
        state.next_seq = state.next_seq.wrapping_add(1);
        Ok(())
    }

    fn wipe(&self) -> Result<(), StoreError> {
        flash::erase(slot_addr(Slot::A));
        flash::erase(slot_addr(Slot::B));
        for slot in [Slot::A, Slot::B] {
            if slot_bytes(slot).iter().any(|&b| b != 0xFF) {
                return Err(StoreError);
            }
        }
        // SAFETY: HIGH-dispatch exclusive after boot, see the field doc.
        unsafe {
            *self.state.get() = State {
                next_slot: Slot::A,
                next_seq: 1,
            }
        };
        Ok(())
    }
}

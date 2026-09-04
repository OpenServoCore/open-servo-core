//! ConfigStore provider (protocol sec 9.4): the two CONFIG flash slots with
//! A/B alternation, the two CALIB slots (own image, own seq), plus the
//! boot-time overlays. Both `save` and `wipe` are blocking -- the CPU
//! fetch-stalls for every erase/program while code runs from flash -- which
//! is exactly the protocol sec 9.4 contract (dispatch's torque gate makes
//! the stall safe, the post-completion ack makes it visible).

use core::cell::SyncUnsafeCell;

use osc_servo_core::persist::{
    self, CALIB_IMAGE_LEN, CALIB_LEN, CONFIG_LEN, HEADER_LEN, IMAGE_LEN, PROFILE_LEN, Slot,
    StoreError,
};

use crate::hal::flash::{self, PAGE_SIZE};
use crate::runtime::statics::SHARED;

/// Header + region bytes that fit the calib slot's first page; the tail
/// spills onto page two.
const CALIB_PAGE1_BODY: usize = PAGE_SIZE - HEADER_LEN;

/// Slot bases come from this crate's `osc-config.x` fragment (shipped into
/// the link search path by build.rs; the board binary passes
/// `-Tosc-config.x`), resolved against the board's CONFIG_A/B and CALIB
/// regions -- the flash layout has exactly one home. The target gate is
/// deterministic host hygiene: an ungated extern symbol links on the host
/// only while no pulled codegen unit references it, which is CGU-partition
/// luck; host builds (unit tests) never call the store, so they get a stub
/// instead of a link-time coupling.
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

#[cfg(target_arch = "riscv32")]
fn calib_slot_addr(slot: Slot) -> u32 {
    unsafe extern "C" {
        static _calib_a: u8;
        static _calib_b: u8;
    }
    match slot {
        Slot::A => (&raw const _calib_a) as u32,
        Slot::B => (&raw const _calib_b) as u32,
    }
}

#[cfg(not(target_arch = "riscv32"))]
fn slot_addr(_slot: Slot) -> u32 {
    unimplemented!("host builds never touch the flash store")
}

#[cfg(not(target_arch = "riscv32"))]
fn calib_slot_addr(_slot: Slot) -> u32 {
    unimplemented!("host builds never touch the flash store")
}

/// The slot's stored image bytes, memory-mapped.
fn slot_bytes(slot: Slot) -> &'static [u8] {
    // SAFETY: memory.x reserves both windows inside main flash, which is
    // always readable and never holds code.
    unsafe { core::slice::from_raw_parts(slot_addr(slot) as *const u8, IMAGE_LEN) }
}

/// The calib slot's stored image bytes (both pages), memory-mapped.
fn calib_slot_bytes(slot: Slot) -> &'static [u8] {
    // SAFETY: as `slot_bytes` -- the CALIB region is reserved data flash.
    unsafe { core::slice::from_raw_parts(calib_slot_addr(slot) as *const u8, CALIB_IMAGE_LEN) }
}

struct State {
    next_slot: Slot,
    next_seq: u16,
    calib_next_slot: Slot,
    calib_next_seq: u16,
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
        calib_next_slot: Slot::A,
        calib_next_seq: 1,
    }),
};

impl ConfigStore {
    /// Boot-time load: overlay the newest valid config and calib images onto
    /// the (already default-seeded) table, prime both A/B states, and seed
    /// the store into `SHARED`. Bringup-only, pre-IRQ; sole writer (the
    /// `seed_config_defaults` contract). The caller re-seeds RO calib sense
    /// facts AFTER this so board data always wins over a stale image.
    pub fn boot_load() {
        let pick = persist::boot_overlay(&SHARED.table, slot_bytes(Slot::A), slot_bytes(Slot::B));
        let calib_pick = persist::boot_overlay_calib(
            &SHARED.table,
            calib_slot_bytes(Slot::A),
            calib_slot_bytes(Slot::B),
        );
        // SAFETY: pre-IRQ sole writer, see fn doc.
        unsafe {
            *CONFIG_STORE.state.get() = State {
                next_slot: pick.next_slot,
                next_seq: pick.next_seq,
                calib_next_slot: calib_pick.next_slot,
                calib_next_seq: calib_pick.next_seq,
            }
        };
        SHARED.seed_store(&CONFIG_STORE);
        crate::log::debug!(
            "config store: loaded={} next_seq={} calib loaded={} next_seq={}",
            pick.loaded,
            pick.next_seq,
            calib_pick.loaded,
            calib_pick.next_seq,
        );
    }
}

impl osc_servo_core::ConfigStore for ConfigStore {
    /// Erase the older slot of each image, stream header + regions straight
    /// into the page buffer (no staging copy), then readback-verify -- the
    /// verify is what lets the ack mean durable. Each image advances its A/B
    /// state only on its own verify, so a calib failure leaves the config
    /// image durable and the ack honest (`hardware`).
    fn save(
        &self,
        config: &[u8; CONFIG_LEN],
        profile: &[u8; PROFILE_LEN],
        calib: &[u8; CALIB_LEN],
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

        // Calib image spans two pages: header + region front on page one,
        // the tail on page two (buffer words past it program as erased).
        let calib_addr = calib_slot_addr(state.calib_next_slot);
        let calib_header = persist::calib_header(state.calib_next_seq, calib);
        flash::erase(calib_addr);
        flash::erase(calib_addr + PAGE_SIZE as u32);
        flash::write(calib_addr, &[&calib_header, &calib[..CALIB_PAGE1_BODY]]);
        flash::write(calib_addr + PAGE_SIZE as u32, &[&calib[CALIB_PAGE1_BODY..]]);

        let stored = calib_slot_bytes(state.calib_next_slot);
        let intact = stored[..HEADER_LEN] == calib_header && stored[HEADER_LEN..] == calib[..];
        if !intact {
            return Err(StoreError);
        }
        state.calib_next_slot = state.calib_next_slot.other();
        state.calib_next_seq = state.calib_next_seq.wrapping_add(1);
        Ok(())
    }

    fn wipe(&self) -> Result<(), StoreError> {
        flash::erase(slot_addr(Slot::A));
        flash::erase(slot_addr(Slot::B));
        for slot in [Slot::A, Slot::B] {
            flash::erase(calib_slot_addr(slot));
            flash::erase(calib_slot_addr(slot) + PAGE_SIZE as u32);
        }
        for slot in [Slot::A, Slot::B] {
            if slot_bytes(slot).iter().any(|&b| b != 0xFF)
                || calib_slot_bytes(slot).iter().any(|&b| b != 0xFF)
            {
                return Err(StoreError);
            }
        }
        // SAFETY: HIGH-dispatch exclusive after boot, see the field doc.
        unsafe {
            *self.state.get() = State {
                next_slot: Slot::A,
                next_seq: 1,
                calib_next_slot: Slot::A,
                calib_next_seq: 1,
            }
        };
        Ok(())
    }
}

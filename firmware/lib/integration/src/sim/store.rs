//! RAM-backed [`ConfigStore`] fake: two page-sized slots holding real images
//! through the core codec, so a rebuilt servo's boot overlay exercises the
//! same parse/pick path the chip takes. `Shared::seed_store` wants
//! `&'static`, so tests obtain instances via [`RamStore::leak`]; the interior
//! `Mutex` keeps the same reference inspectable (and shareable across a
//! rebuilt `Sim`, modeling a reboot with flash intact).

use std::sync::Mutex;

use osc_servo_core::persist::{self, CONFIG_LEN, IMAGE_LEN, PROFILE_LEN, Slot, StoreError};
use osc_servo_core::{ConfigStore, ControlTableCell};

const ERASED: [u8; IMAGE_LEN] = [0xFF; IMAGE_LEN];

fn idx(slot: Slot) -> usize {
    match slot {
        Slot::A => 0,
        Slot::B => 1,
    }
}

#[derive(Default)]
struct Inner {
    slots: [Option<[u8; IMAGE_LEN]>; 2],
    next_slot: usize,
    next_seq: u16,
    fail: bool,
    saves: usize,
    wipes: usize,
}

pub struct RamStore {
    inner: Mutex<Inner>,
}

impl RamStore {
    pub fn leak() -> &'static RamStore {
        Box::leak(Box::new(RamStore {
            inner: Mutex::new(Inner {
                next_seq: 1,
                ..Inner::default()
            }),
        }))
    }

    /// Boot-time load, mirroring the chip provider: overlay the newest valid
    /// image and prime the A/B state. Called by `SimServo::build` before the
    /// bus reads the table's comms block.
    pub fn boot_load(&self, table: &ControlTableCell) {
        let mut g = self.inner.lock().unwrap();
        let a = g.slots[0].unwrap_or(ERASED);
        let b = g.slots[1].unwrap_or(ERASED);
        let pick = persist::boot_overlay(table, &a, &b);
        g.next_slot = idx(pick.next_slot);
        g.next_seq = pick.next_seq;
    }

    /// Arm every subsequent save/wipe to fail (the chip's readback-verify
    /// miss).
    pub fn set_fail(&self, fail: bool) {
        self.inner.lock().unwrap().fail = fail;
    }

    pub fn saves(&self) -> usize {
        self.inner.lock().unwrap().saves
    }

    pub fn wipes(&self) -> usize {
        self.inner.lock().unwrap().wipes
    }

    /// The slot's stored image, if any (a copy -- tests parse it at leisure).
    pub fn slot(&self, slot: Slot) -> Option<[u8; IMAGE_LEN]> {
        self.inner.lock().unwrap().slots[idx(slot)]
    }
}

impl ConfigStore for RamStore {
    fn save(
        &self,
        config: &[u8; CONFIG_LEN],
        profile: &[u8; PROFILE_LEN],
    ) -> Result<(), StoreError> {
        let mut g = self.inner.lock().unwrap();
        if g.fail {
            return Err(StoreError);
        }
        let mut img = [0u8; IMAGE_LEN];
        persist::assemble(&mut img, g.next_seq, config, profile);
        let slot = g.next_slot;
        g.slots[slot] = Some(img);
        g.next_slot ^= 1;
        g.next_seq = g.next_seq.wrapping_add(1);
        g.saves += 1;
        Ok(())
    }

    fn wipe(&self) -> Result<(), StoreError> {
        let mut g = self.inner.lock().unwrap();
        if g.fail {
            return Err(StoreError);
        }
        g.slots = [None, None];
        g.next_slot = 0;
        g.next_seq = 1;
        g.wipes += 1;
        Ok(())
    }
}

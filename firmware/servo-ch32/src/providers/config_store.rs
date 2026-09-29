//! ConfigStore provider (protocol sec 9.4): the two CONFIG flash slots with
//! A/B alternation, the two CALIB slots (calibration plus its tables, own
//! image, own seq), plus the boot-time overlays. Both `save` and `wipe`
//! are blocking - the CPU fetch-stalls for every erase/program while code
//! runs from flash - which is exactly the protocol sec 9.4 contract
//! (dispatch's torque gate makes the stall safe, the post-completion ack
//! makes it visible).

use core::cell::SyncUnsafeCell;

use osc_servo_core::RegionStorage;
use osc_servo_core::persist::{
    self, BootPick, CALIB_IMAGE_LEN, CALIB_LEN, CONFIG_LEN, IMAGE_LEN, LUT_LEN, PROFILE_LEN, Slot,
    StoreError,
};
use osc_servo_core::pos_lut::INTERVALS;

use crate::hal::flash::{self, PAGE_SIZE};
use crate::runtime::statics::SHARED;

/// Slot bases come from this crate's `osc-config.x` fragment (shipped into
/// the link search path by build.rs; the board binary passes
/// `-Tosc-config.x`), resolved against the board's CONFIG_A/B and CALIB
/// regions - the flash layout has exactly one home. The target gate is
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

/// A stored image's bytes, memory-mapped.
fn stored(addr: u32, len: usize) -> &'static [u8] {
    // SAFETY: memory.x reserves every slot window inside main flash, which
    // is always readable and never holds code.
    unsafe { core::slice::from_raw_parts(addr as *const u8, len) }
}

/// The position table array as the LE byte stream the image body is: the
/// in-memory i16 layout on this little-endian target.
fn lut_bytes(lut: &[i16; INTERVALS]) -> &[u8; LUT_LEN] {
    // SAFETY: i16 has no padding, the array is exactly LUT_LEN bytes, and
    // u8 alignment is 1.
    unsafe { &*(lut as *const [i16; INTERVALS] as *const [u8; LUT_LEN]) }
}

/// Most segments one image streams (header, region, tables).
const SEGS_MAX: usize = 3;

/// Erase the slot's pages, stream the image's segments (header first)
/// across them with no staging copy - each page takes the sub-slices that
/// fall in it, buffer words past the image's last byte program as erased -
/// then readback-verify: the verify is what lets the ack mean durable.
fn program(addr: u32, segs: &[&[u8]]) -> Result<(), StoreError> {
    let total: usize = segs.iter().map(|s| s.len()).sum();
    for p in 0..total.div_ceil(PAGE_SIZE) {
        let (lo, hi) = (p * PAGE_SIZE, (p + 1) * PAGE_SIZE);
        flash::erase(addr + lo as u32);
        let mut parts: [&[u8]; SEGS_MAX] = [&[]; SEGS_MAX];
        let mut n = 0;
        let mut off = 0;
        for seg in segs {
            let (s, e) = (off, off + seg.len());
            off = e;
            let (a, b) = (s.max(lo), e.min(hi));
            if a < b
                && let Some(part) = seg.get(a - s..b - s)
                && let Some(slot) = parts.get_mut(n)
            {
                *slot = part;
                n += 1;
            }
        }
        flash::write(addr + lo as u32, &parts[..n]);
    }
    let s = stored(addr, total);
    let mut off = 0;
    for seg in segs {
        if s.get(off..off + seg.len()) != Some(*seg) {
            return Err(StoreError);
        }
        off += seg.len();
    }
    Ok(())
}

/// One image's A/B position: the slot the next SAVE programs and the seq
/// it carries.
struct Cursor {
    next_slot: Slot,
    next_seq: u16,
}

impl Cursor {
    const FRESH: Cursor = Cursor {
        next_slot: Slot::A,
        next_seq: 1,
    };

    const fn from_pick(pick: &BootPick) -> Cursor {
        Cursor {
            next_slot: pick.next_slot,
            next_seq: pick.next_seq,
        }
    }

    fn advance(&mut self) {
        self.next_slot = self.next_slot.other();
        self.next_seq = self.next_seq.wrapping_add(1);
    }
}

struct State {
    config: Cursor,
    calib: Cursor,
}

impl State {
    const FRESH: State = State {
        config: Cursor::FRESH,
        calib: Cursor::FRESH,
    };
}

pub struct ConfigStore {
    /// Written by `boot_load` (pre-IRQ), then only from HIGH dispatch (the
    /// SESSION exclusivity invariant, `runtime::isr`) - never concurrent.
    state: SyncUnsafeCell<State>,
}

static CONFIG_STORE: ConfigStore = ConfigStore {
    state: SyncUnsafeCell::new(State::FRESH),
};

impl ConfigStore {
    /// Boot-time load: overlay the newest valid config and calib images onto
    /// the (already default-seeded) table, the calib image's tables into
    /// `SHARED` beside it, prime both A/B states, publish the data state the
    /// verdicts make, and seed the store into `SHARED`. Bringup-only,
    /// pre-IRQ; sole writer (the `seed_config_defaults` contract). The
    /// caller re-seeds RO calib sense facts AFTER this so board data always
    /// wins over a stale image.
    pub fn boot_load() {
        let pick = persist::boot_overlay(
            &SHARED.table,
            stored(slot_addr(Slot::A), IMAGE_LEN),
            stored(slot_addr(Slot::B), IMAGE_LEN),
        );
        let calib_pick = persist::boot_overlay_calib(
            &SHARED,
            stored(calib_slot_addr(Slot::A), CALIB_IMAGE_LEN),
            stored(calib_slot_addr(Slot::B), CALIB_IMAGE_LEN),
        );
        SHARED.publish_data_state(pick.state, calib_pick.state);
        // SAFETY: pre-IRQ sole writer, see fn doc.
        unsafe {
            *CONFIG_STORE.state.get() = State {
                config: Cursor::from_pick(&pick),
                calib: Cursor::from_pick(&calib_pick),
            }
        };
        SHARED.seed_store(&CONFIG_STORE);
        // image state: 0 loaded, 1 virgin, 2 corrupt, 3 stale (data_state)
        crate::log::debug!(
            "config store: state={} next_seq={} calib state={} next_seq={} pos_lut_state={}",
            pick.state as u8,
            pick.next_seq,
            calib_pick.state as u8,
            calib_pick.next_seq,
            SHARED.table.with(|t| t.control.pos_lut.pos_lut_state),
        );
    }
}

impl osc_servo_core::ConfigStore for ConfigStore {
    /// Program each image's older slot in turn (config, then calib with its
    /// tables), each readback-verified. An image advances its A/B state
    /// only on its own verify, so a later failure leaves the earlier image
    /// durable and the ack honest (`hardware`).
    fn save(
        &self,
        config: &[u8; CONFIG_LEN],
        profile: &[u8; PROFILE_LEN],
        calib: &[u8; CALIB_LEN],
        lut: &[i16; INTERVALS],
    ) -> Result<(), StoreError> {
        // SAFETY: HIGH-dispatch exclusive after boot, see the field doc.
        let state = unsafe { &mut *self.state.get() };
        let addr = slot_addr(state.config.next_slot);
        let header = persist::header(state.config.next_seq, config, profile);
        program(addr, &[&header, config, profile])?;
        state.config.advance();

        let addr = calib_slot_addr(state.calib.next_slot);
        let header = persist::calib_header(state.calib.next_seq, calib, lut);
        program(addr, &[&header, calib, lut_bytes(lut)])?;
        state.calib.advance();
        Ok(())
    }

    fn wipe(&self) -> Result<(), StoreError> {
        type SlotAddr = fn(Slot) -> u32;
        const IMAGES: [(SlotAddr, usize); 2] =
            [(slot_addr, IMAGE_LEN), (calib_slot_addr, CALIB_IMAGE_LEN)];
        for (base, len) in IMAGES {
            for slot in [Slot::A, Slot::B] {
                for p in 0..len.div_ceil(PAGE_SIZE) {
                    flash::erase(base(slot) + (p * PAGE_SIZE) as u32);
                }
            }
        }
        for (base, len) in IMAGES {
            for slot in [Slot::A, Slot::B] {
                if stored(base(slot), len).iter().any(|&b| b != 0xFF) {
                    return Err(StoreError);
                }
            }
        }
        // SAFETY: HIGH-dispatch exclusive after boot, see the field doc.
        unsafe { *self.state.get() = State::FRESH };
        Ok(())
    }
}

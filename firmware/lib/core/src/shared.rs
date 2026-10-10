use core::cell::SyncUnsafeCell;

use core::sync::atomic::compiler_fence;
use osc_protocol::wire::UID_LEN;

use portable_atomic::{AtomicU8, AtomicU16, Ordering};

use crate::persist::ConfigStore;
use crate::pos_lut::POINTS;
use crate::regions::control::addr::lifecycle::STALL_PERMIT;
use crate::{ControlTableCell, RegionStorage};

#[repr(C)]
pub struct Shared {
    pub table: ControlTableCell,
    /// Work a commit left for the main loop (`data_state::job` bits). Bus
    /// dispatch posts and cancels; the main loop's publish clears what it
    /// ran, under a generation check.
    data_job: AtomicU8,
    /// Bumped by dispatch on every write the job hashes or validates (covered,
    /// stamp, LUT array, torque). A job whose generation moved discards
    /// its result. Dispatch is the sole writer, so plain load/store suffice.
    data_gen: AtomicU16,
    /// Bumped by dispatch on every committed grant of the stall permit; the
    /// kernel renews its lease when it moves (`permit_after_commit`).
    /// Dispatch is the sole writer.
    permit_gen: AtomicU8,
    /// Bumped by dispatch on every committed write into CONFIG or CALIB (and
    /// ASSIGN's id); the kernel rebuilds its configuration from the table
    /// when it moves. Dispatch is the sole writer.
    config_gen: AtomicU8,
    /// The factory UID, silicon ID zero-padded to the 16-byte wire field
    /// (osc-native sec 9.2) -- internal identity, not a table register; MGMT ENUM
    /// is its only wire reader.
    uid: SyncUnsafeCell<[u8; UID_LEN]>,
    /// The sec 9.4 persistence store; MGMT SAVE/FACTORY are its only callers
    /// (cold path -- `dyn` costs nothing that matters here).
    store: SyncUnsafeCell<Option<&'static dyn ConfigStore>>,
    /// The position table (`pos_lut` module), all-zero = identity; the CONTROL
    /// window loads it a page at a time. Boot, then the bus dispatcher
    /// alone, write it - the table's single-writer contract.
    pos_lut: SyncUnsafeCell<[i16; POINTS]>,
}

#[allow(clippy::new_without_default)]
impl Shared {
    pub const fn new() -> Self {
        Self {
            table: ControlTableCell::new(),
            data_job: AtomicU8::new(0),
            data_gen: AtomicU16::new(0),
            permit_gen: AtomicU8::new(0),
            config_gen: AtomicU8::new(0),
            uid: SyncUnsafeCell::new([0; UID_LEN]),
            store: SyncUnsafeCell::new(None),
            pos_lut: SyncUnsafeCell::new([0; POINTS]),
        }
    }

    /// Bus dispatch: a write the job hashes or validates landed; `post` what it
    /// leaves for the main loop, `cancel` what it retires.
    pub(crate) fn data_touch(&self, post: u8, cancel: u8) {
        self.data_gen.store(
            self.data_gen.load(Ordering::Relaxed).wrapping_add(1),
            Ordering::Relaxed,
        );
        let job = self.data_job.load(Ordering::Relaxed);
        self.data_job
            .store((job | post) & !cancel, Ordering::Relaxed);
    }

    /// A committed write `[addr, addr + len)` covering `stall_permit` that
    /// leaves it true with torque on is a grant. The table byte stays the
    /// host's request: the kernel only reads CONTROL, so the lease it grants
    /// lives in the kernel and a torque-off request never becomes one. Bus
    /// dispatch only; one copy behind both commit sites, O(1).
    #[inline(never)]
    pub fn permit_after_commit(&self, addr: u16, len: u16) {
        if addr > STALL_PERMIT || addr.saturating_add(len) <= STALL_PERMIT {
            return;
        }
        let (permit, torque) = self.table.with(|t| {
            let l = &t.control.lifecycle;
            (l.stall_permit, l.torque_enable)
        });
        if permit && torque {
            self.permit_gen.store(
                self.permit_gen.load(Ordering::Relaxed).wrapping_add(1),
                Ordering::Relaxed,
            );
        }
    }

    pub(crate) fn permit_gen(&self) -> u8 {
        self.permit_gen.load(Ordering::Relaxed)
    }

    /// Bus dispatch: CONFIG or CALIB changed under the kernel.
    pub fn config_touch(&self) {
        // The kernel preempts dispatch: the committed stores land before the
        // generation it snapshots them under (`Kernel::refresh` holds the
        // Acquire side).
        compiler_fence(Ordering::Release);
        self.config_gen.store(
            self.config_gen.load(Ordering::Relaxed).wrapping_add(1),
            Ordering::Relaxed,
        );
    }

    pub(crate) fn config_gen(&self) -> u8 {
        self.config_gen.load(Ordering::Relaxed)
    }

    pub(crate) fn data_gen(&self) -> u16 {
        self.data_gen.load(Ordering::Relaxed)
    }

    pub(crate) fn data_job(&self) -> u8 {
        self.data_job.load(Ordering::Relaxed)
    }

    /// Main loop, ISRs masked: retire the bits a published run serviced.
    pub(crate) fn data_job_done(&self, done: u8) {
        let job = self.data_job.load(Ordering::Relaxed);
        self.data_job.store(job & !done, Ordering::Relaxed);
    }

    /// Borrow the position table; the caller upholds the single-writer
    /// contract.
    pub fn with_pos_lut<T>(&self, f: impl FnOnce(&[i16; POINTS]) -> T) -> T {
        // SAFETY: see fn doc.
        f(unsafe { &*self.pos_lut.get() })
    }

    /// Mutably borrow the position table; bus dispatch (and pre-IRQ boot)
    /// only.
    pub fn with_pos_lut_mut<T>(&self, f: impl FnOnce(&mut [i16; POINTS]) -> T) -> T {
        // SAFETY: see fn doc.
        f(unsafe { &mut *self.pos_lut.get() })
    }

    /// The kernel's volatile-read handle (`Shared::pos_lut_q4`): a read may
    /// land inside a write, so the fast tick never forms a `&`.
    pub(crate) fn pos_lut_ptr(&self) -> *const [i16; POINTS] {
        self.pos_lut.get()
    }

    /// Seed the ESIG-derived UID. Bringup-only, pre-IRQ; sole writer (the
    /// `seed_config_defaults` contract).
    pub fn seed_uid(&self, uid: [u8; UID_LEN]) {
        // SAFETY: see fn doc -- no reader exists before IRQs enable.
        unsafe { *self.uid.get() = uid };
    }

    /// Stable-address borrow: ENUM replies stream straight from it (sec 4.2).
    pub fn uid(&self) -> &[u8; UID_LEN] {
        // SAFETY: written only by `seed_uid` pre-IRQ; read-only afterward.
        unsafe { &*self.uid.get() }
    }

    /// Seed the persistence store. Bringup-only, pre-IRQ; sole writer (the
    /// `seed_config_defaults` contract).
    pub fn seed_store(&self, store: &'static dyn ConfigStore) {
        // SAFETY: see fn doc -- no reader exists before IRQs enable.
        unsafe { *self.store.get() = Some(store) };
    }

    /// The sec 9.4 store; `None` until seeded (dispatch answers `hardware`).
    pub fn store(&self) -> Option<&'static dyn ConfigStore> {
        // SAFETY: written only by `seed_store` pre-IRQ; read-only afterward.
        unsafe { *self.store.get() }
    }
}

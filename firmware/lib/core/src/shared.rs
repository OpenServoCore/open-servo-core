use core::cell::SyncUnsafeCell;

use osc_protocol::wire::UID_LEN;
use portable_atomic::{AtomicU8, AtomicU16, Ordering};

use crate::ControlTableCell;
use crate::persist::ConfigStore;
use crate::pot_lut::KNOTS;

#[repr(C)]
pub struct Shared {
    pub table: ControlTableCell,
    /// Work a commit left for the main loop (`data_state::job` bits). HIGH
    /// dispatch posts and cancels; the main loop's publish clears what it
    /// ran, under a generation check.
    data_job: AtomicU8,
    /// Bumped by HIGH on every write the job hashes or validates (covered,
    /// stamp, LUT array, torque). A job whose generation moved discards
    /// its result. HIGH is the sole writer, so plain load/store suffice.
    data_gen: AtomicU16,
    /// The factory UID, silicon ID zero-padded to the 16-byte wire field
    /// (osc-native sec 9.2) -- internal identity, not a table register; MGMT ENUM
    /// is its only wire reader.
    uid: SyncUnsafeCell<[u8; UID_LEN]>,
    /// The sec 9.4 persistence store; MGMT SAVE/FACTORY are its only callers
    /// (cold path -- `dyn` costs nothing that matters here).
    store: SyncUnsafeCell<Option<&'static dyn ConfigStore>>,
    /// The pot LUT (`pot_lut` module), all-zero = identity; the CONTROL
    /// window loads it a page at a time. Boot, then the HIGH dispatcher
    /// alone, write it - the table's single-writer contract.
    pot_lut: SyncUnsafeCell<[i16; KNOTS]>,
}

#[allow(clippy::new_without_default)]
impl Shared {
    pub const fn new() -> Self {
        Self {
            table: ControlTableCell::new(),
            data_job: AtomicU8::new(0),
            data_gen: AtomicU16::new(0),
            uid: SyncUnsafeCell::new([0; UID_LEN]),
            store: SyncUnsafeCell::new(None),
            pot_lut: SyncUnsafeCell::new([0; KNOTS]),
        }
    }

    /// HIGH: a write the job hashes or validates landed; `post` what it
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

    /// Borrow the pot LUT; the caller upholds the single-writer contract.
    pub fn with_pot_lut<T>(&self, f: impl FnOnce(&[i16; KNOTS]) -> T) -> T {
        // SAFETY: see fn doc.
        f(unsafe { &*self.pot_lut.get() })
    }

    /// Mutably borrow the pot LUT; HIGH dispatch (and pre-IRQ boot) only.
    pub fn with_pot_lut_mut<T>(&self, f: impl FnOnce(&mut [i16; KNOTS]) -> T) -> T {
        // SAFETY: see fn doc.
        f(unsafe { &mut *self.pot_lut.get() })
    }

    /// The kernel's volatile-read handle (`Shared::pot_lut_q4`): the writer
    /// may preempt a read, so the fast tick never forms a `&`.
    pub(crate) fn pot_lut_ptr(&self) -> *const [i16; KNOTS] {
        self.pot_lut.get()
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

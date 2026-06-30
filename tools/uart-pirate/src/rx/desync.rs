//! Sticky-fatal "desynced" flag. Set once on either of two conditions:
//! IC ring overrun (`ic_overrun`) or stamp ring overflow from host
//! backpressure (`stamp_overflow`). Once set, `walk()` is a no-op and
//! host commands return `ERR desync <cause>` until a `RESET` (or `BAUD`,
//! which implicitly resets) clears the flag.

use portable_atomic::{AtomicBool, AtomicU8, Ordering};

/// Sticky-fatal cause for the desync flag. Stored as `u8` so it can sit
/// in an `AtomicU8`; first trip wins via `compare_exchange` so the cause
/// reflects the originating failure even if a later condition would also
/// trip.
#[repr(u8)]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum DesyncCause {
    IcOverrun = 2,
    StampOverflow = 3,
}

impl DesyncCause {
    fn from_u8(v: u8) -> Option<Self> {
        match v {
            2 => Some(Self::IcOverrun),
            3 => Some(Self::StampOverflow),
            _ => None,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::IcOverrun => "ic_overrun",
            Self::StampOverflow => "stamp_overflow",
        }
    }
}

static DESYNCED: AtomicBool = AtomicBool::new(false);
/// `0` = not desynced; non-zero encodes a `DesyncCause`. First trip wins
/// via `compare_exchange`.
static DESYNC_CAUSE: AtomicU8 = AtomicU8::new(0);

#[inline]
pub(super) fn is_desynced() -> bool {
    DESYNCED.load(Ordering::Relaxed)
}

#[inline]
pub(super) fn set(cause: DesyncCause) {
    let _ = DESYNC_CAUSE.compare_exchange(0, cause as u8, Ordering::AcqRel, Ordering::Relaxed);
    DESYNCED.store(true, Ordering::Release);
}

/// Returns `Some(cause)` once any of the trip conditions has fired,
/// `None` while READY. Recovery from desync is `RESET`.
pub fn desync_cause() -> Option<DesyncCause> {
    let raw = DESYNC_CAUSE.load(Ordering::Acquire);
    DesyncCause::from_u8(raw)
}

pub(super) fn clear() {
    DESYNC_CAUSE.store(0, Ordering::Release);
    DESYNCED.store(false, Ordering::Release);
}

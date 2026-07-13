//! Sticky-fatal "desynced" flag: the host let more than a ring of bytes
//! accumulate undrained, so DMA lapped unread data (`stamp_overflow` --
//! a bench-script bug, not a wire condition). Once set, drains error
//! out until a `RESET` (or `BAUD`, which implicitly resets) clears it.

use portable_atomic::{AtomicU8, Ordering};

#[repr(u8)]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum DesyncCause {
    StampOverflow = 3,
}

impl DesyncCause {
    fn from_u8(v: u8) -> Option<Self> {
        match v {
            3 => Some(Self::StampOverflow),
            _ => None,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::StampOverflow => "stamp_overflow",
        }
    }
}

/// `0` = not desynced; non-zero encodes a `DesyncCause`. First trip wins.
static DESYNC_CAUSE: AtomicU8 = AtomicU8::new(0);

#[inline]
pub(super) fn set(cause: DesyncCause) {
    let _ = DESYNC_CAUSE.compare_exchange(0, cause as u8, Ordering::AcqRel, Ordering::Relaxed);
}

/// Returns `Some(cause)` once tripped, `None` while READY. Recovery is
/// `RESET`.
pub fn desync_cause() -> Option<DesyncCause> {
    DesyncCause::from_u8(DESYNC_CAUSE.load(Ordering::Acquire))
}

pub(super) fn clear() {
    DESYNC_CAUSE.store(0, Ordering::Release);
}

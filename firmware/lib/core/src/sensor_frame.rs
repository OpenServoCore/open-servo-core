use core::marker::PhantomData;

use crate::regions::telemetry::TelemetryTick;

/// One acquisition frame in device counts, as the tests and the sim land it
/// in the table's tick block ([`Self::land`]) where the chip's ADC DMA and
/// tick ISR land a scan. Conversion to engineering units is the host's job.
#[derive(Copy, Clone, Debug, Default)]
pub struct SensorFrame {
    pub pos: u16,
    pub current_trough: u16,
    /// The shunt at the counter peak (`TelemetryTick::current_raw`).
    pub current: u16,
    pub vmotor_a: u16,
    pub vmotor_b: u16,
    pub vbus_raw: u16,
    pub ntc_raw: u16,
    pub vcal: u16,
    pub vmotor_a_trough: u16,
    pub vmotor_b_trough: u16,
}

impl SensorFrame {
    /// Write the frame's fields into `tick`, leaving the kernel's alone.
    ///
    /// # Safety
    /// `tick` is valid for writes, and nothing reads the fields meanwhile
    /// but this context.
    pub unsafe fn land(&self, tick: *mut TelemetryTick) {
        // SAFETY: fn contract; volatile per-field stores.
        unsafe {
            (&raw mut (*tick).pos).write_volatile(self.pos);
            (&raw mut (*tick).current_trough).write_volatile(self.current_trough);
            (&raw mut (*tick).current_raw).write_volatile(self.current);
            (&raw mut (*tick).vmotor_a).write_volatile(self.vmotor_a);
            (&raw mut (*tick).vmotor_b).write_volatile(self.vmotor_b);
            (&raw mut (*tick).vbus_raw).write_volatile(self.vbus_raw);
            (&raw mut (*tick).ntc_raw).write_volatile(self.ntc_raw);
            (&raw mut (*tick).vcal).write_volatile(self.vcal);
            (&raw mut (*tick).vmotor_a_trough).write_volatile(self.vmotor_a_trough);
            (&raw mut (*tick).vmotor_b_trough).write_volatile(self.vmotor_b_trough);
        }
    }
}

/// The frame as a tick block holds it, read in place: on the chip the ADC
/// DMA rewrites the block under the tick, so every read is volatile and no
/// reference to it is formed.
#[derive(Copy, Clone)]
pub struct Frame<'a> {
    tick: *const TelemetryTick,
    _block: PhantomData<&'a TelemetryTick>,
}

macro_rules! read {
    ($($(#[$m:meta])* $name:ident => $field:ident),* $(,)?) => {$(
        $(#[$m])*
        #[inline(always)]
        pub fn $name(self) -> u16 {
            // SAFETY: `tick` is valid for reads for 'a (constructors).
            unsafe { (&raw const (*self.tick).$field).read_volatile() }
        }
    )*};
}

impl<'a> Frame<'a> {
    /// # Safety
    /// `tick` is valid for reads for `'a`.
    #[inline(always)]
    pub const unsafe fn new(tick: *const TelemetryTick) -> Self {
        Self {
            tick,
            _block: PhantomData,
        }
    }

    pub fn of(tick: &'a TelemetryTick) -> Self {
        // SAFETY: a borrow is valid for reads for its lifetime.
        unsafe { Self::new(tick) }
    }

    read! {
        pos => pos,
        current_trough => current_trough,
        /// The shunt at the counter peak.
        current => current_raw,
        vmotor_a => vmotor_a,
        vmotor_b => vmotor_b,
        vbus_raw => vbus_raw,
        ntc_raw => ntc_raw,
        vmotor_a_trough => vmotor_a_trough,
        vmotor_b_trough => vmotor_b_trough,
    }
}

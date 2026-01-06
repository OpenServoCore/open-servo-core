//! Embassy async primitive wrappers.
//!
//! Implements [`SignalReader`], [`SignalWriter`], and [`AsyncTimer`] traits
//! from `open_servo_hw::v2` using embassy types.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};

use open_servo_hw::v2::{AsyncTimer, ResetLevel, SignalReader, SignalWriter};
use open_servo_units::{MicroSecond, TimeStampUs};

/// Newtype wrapper for embassy Signal<()> implementing SignalReader/SignalWriter.
#[derive(Clone, Copy)]
pub struct EmbassySignal(pub &'static Signal<CriticalSectionRawMutex, ()>);

impl SignalReader<()> for EmbassySignal {
    async fn wait(&self) {
        self.0.wait().await;
    }
}

impl SignalWriter<()> for EmbassySignal {
    fn signal(&self, _value: ()) {
        self.0.signal(());
    }
}

/// Newtype wrapper for ResetLevel-carrying signals.
#[derive(Clone, Copy)]
pub struct ResetSignalWrapper(pub &'static Signal<CriticalSectionRawMutex, ResetLevel>);

impl SignalReader<ResetLevel> for ResetSignalWrapper {
    async fn wait(&self) -> ResetLevel {
        self.0.wait().await
    }
}

impl SignalWriter<ResetLevel> for ResetSignalWrapper {
    fn signal(&self, level: ResetLevel) {
        self.0.signal(level);
    }
}

/// AsyncTimer implementation using embassy_time.
#[derive(Clone, Copy, Default)]
pub struct EmbassyTimer;

impl AsyncTimer for EmbassyTimer {
    fn now(&self) -> TimeStampUs {
        let now = Instant::now();
        TimeStampUs(now.as_micros() as u32)
    }

    async fn delay(&self, duration: MicroSecond) {
        Timer::after(Duration::from_micros(duration.0 as u64)).await;
    }
}

//! Host side of the firmware high-rate shunt burst: arm, poll, walk the
//! eight readback pages, release. One capture is 960 raw shunt codes at one
//! ADC conversion period each - a current step sampled ~46x per PWM period
//! instead of the once per period ordinary telemetry gives.
//!
//! Sans-io like the rest of the crate: [`BurstIo`] is the four wire moves
//! and the sleep the handshake needs, the driver supplies them. The page
//! walk is the only part with a protocol subtlety - the servo copies a page
//! into the readback window from its main loop with `page_echo` as the
//! guard (0xFF while copying, the page number after a fence), so a reply
//! whose echo does not match the requested page is a plain retry.
//!
//! The capture carries no rail voltage: the burst window is ADC time the
//! scan does not run, so vbus and the current-sense bias come from a
//! telemetry read taken just before the arm (see [`Pre`]).

use core::fmt;

use crate::regs::{burst as reg, control};

/// Samples one capture holds; 8 pages of 120.
pub const SAMPLES: usize = 960;
pub const PAGE_SAMPLES: usize = 120;
pub const PAGES: u8 = 8;

/// One READ of the burst section returns the selected page plus the whole
/// header - `MAX_PAYLOAD` on the wire, and exactly the section's live span.
pub const READ_LEN: u16 = 252;

/// `page_echo` while the servo is mid-copy.
pub const PAGE_BUSY: u8 = 0xFF;

pub const STATE_IDLE: u8 = 0;
pub const STATE_ARMED: u8 = 1;
pub const STATE_CAPTURING: u8 = 2;
pub const STATE_DONE: u8 = 3;
pub const STATE_REJECTED: u8 = 4;

/// HCLK cycles per conversion: TCONV = 26 ADCCLK and ADCCLK = HCLK/2.
pub const SAMPLE_HCLK: f64 = 52.0;
pub const HCLK_MHZ: f64 = 48.0;

/// Sample period, microseconds. One free-running conversion of one channel;
/// this is the ruler for the sample clock, while the trace's own ON-edge
/// spacing is the ruler for the PWM period.
pub const SAMPLE_US: f64 = SAMPLE_HCLK / HCLK_MHZ;

/// Samples per PWM period the sample clock predicts. Center-aligned, so one
/// period is 2 x ARR of HCLK. The measured cadence is gated against this.
pub fn nominal_cadence(pwm_arr: u16) -> f64 {
    2.0 * pwm_arr as f64 / SAMPLE_HCLK
}

/// What the host knew about the servo just before the arm. The burst
/// suspends the scan, so neither number is measurable inside the window.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
pub struct Pre {
    /// Duty in force over the pre-step half of the capture, q15.
    pub pre_q15: i16,
    pub vbus_raw: u16,
    /// Current-sense zero as the servo's trough tracker reports it.
    pub bias: u16,
}

/// The burst header as one READ returns it, minus the page payload.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
pub struct Meta {
    pub pre_q15: i16,
    pub step_q15: i16,
    pub step_index: u16,
    pub start_cnt: u16,
    pub pwm_arr: u16,
    pub start_dir: u8,
    pub restore_dir: u8,
    pub vbus_raw: u16,
    pub bias: u16,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Capture {
    pub samples: Vec<u16>,
    pub meta: Meta,
}

/// Poll and retry budgets. Defaults: the capture itself is ~2 ms
/// (BURST_SETTLE_TICKS + 960 conversions), so 200 polls at 2 ms is a 400 ms
/// ceiling - long enough that only a wedged servo hits it.
#[derive(Copy, Clone, Debug)]
pub struct CaptureCfg {
    pub poll_ms: u32,
    pub max_polls: u32,
    /// Attempts per page before the capture is abandoned.
    pub page_tries: u32,
}

impl Default for CaptureCfg {
    fn default() -> Self {
        Self {
            poll_ms: 2,
            max_polls: 200,
            page_tries: 5,
        }
    }
}

/// The wire moves a capture needs. `arm` must apply duty and arm in the
/// same instant (HOLD both, one COMMIT): a servo that sees arm before the
/// duty captures the old level.
pub trait BurstIo {
    type Error;

    fn arm(&mut self, duty_q15: i16) -> Result<(), Self::Error>;
    fn select_page(&mut self, page: u8) -> Result<(), Self::Error>;
    fn release(&mut self) -> Result<(), Self::Error>;
    /// One READ of the burst section, `len` bytes from `addr`.
    fn read_burst(&mut self, addr: u16, len: u16) -> Result<Vec<u8>, Self::Error>;
    fn pause_ms(&mut self, ms: u32);
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Error<E> {
    Io(E),
    /// The servo refused the arm: torque off, wrong mode, a latched fault,
    /// or TEL still streaming.
    Rejected,
    /// State never reached Done.
    Timeout {
        state: u8,
        polls: u32,
    },
    /// A page never echoed its own number.
    Page {
        page: u8,
        tries: u32,
    },
    /// A reply too short to hold the header.
    Short {
        got: usize,
        want: usize,
    },
    /// The servo published a sample count this client cannot page.
    Len {
        got: u16,
    },
}

impl<E: fmt::Display> fmt::Display for Error<E> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Io(e) => write!(f, "{e}"),
            Error::Rejected => write!(
                f,
                "servo rejected the arm (needs torque on, OpenLoop, no fault, TEL idle)"
            ),
            Error::Timeout { state, polls } => {
                write!(f, "burst stuck in state {state} after {polls} polls")
            }
            Error::Page { page, tries } => {
                write!(f, "page {page} never echoed in {tries} tries")
            }
            Error::Short { got, want } => write!(f, "burst read returned {got} B, need {want}"),
            Error::Len { got } => {
                write!(f, "servo published samples_len {got}, expected {SAMPLES}")
            }
        }
    }
}

/// Byte offsets inside a `READ(PAGE_ECHO, READ_LEN)` reply.
const fn off(reg: crate::regs::Reg) -> usize {
    (reg.addr - self::reg::PAGE_ECHO.addr) as usize
}

fn u16_at(raw: &[u8], at: usize) -> u16 {
    u16::from_le_bytes([raw[at], raw[at + 1]])
}

/// Header of one readback reply. Only `page_echo`/`state` are meaningful
/// before the capture is Done.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Header {
    pub page_echo: u8,
    pub state: u8,
    pub samples_len: u16,
    pub step_index: u16,
    pub start_cnt: u16,
    pub pwm_arr: u16,
    pub start_dir: u8,
    pub restore_dir: u8,
}

/// None when the slice is shorter than the header span.
pub fn parse_header(raw: &[u8]) -> Option<Header> {
    if raw.len() < READ_LEN as usize {
        return None;
    }
    Some(Header {
        page_echo: raw[off(reg::PAGE_ECHO)],
        state: raw[off(reg::STATE)],
        samples_len: u16_at(raw, off(reg::SAMPLES_LEN)),
        step_index: u16_at(raw, off(reg::STEP_INDEX)),
        start_cnt: u16_at(raw, off(reg::START_CNT)),
        pwm_arr: u16_at(raw, off(reg::PWM_ARR)),
        start_dir: raw[off(reg::START_DIR)],
        restore_dir: raw[off(reg::RESTORE_DIR)],
    })
}

/// The 120 LE codes of the page a reply carries.
pub fn parse_page(raw: &[u8]) -> impl Iterator<Item = u16> + '_ {
    let base = off(reg::SAMPLES);
    (0..PAGE_SAMPLES).map(move |k| u16_at(raw, base + 2 * k))
}

/// One capture end to end: arm at `duty_q15`, wait for Done, walk the
/// pages, release. `pre` is the host's pre-arm telemetry read; it is copied
/// into the capture's meta unchanged.
pub fn capture<IO: BurstIo>(
    io: &mut IO,
    duty_q15: i16,
    pre: Pre,
    cfg: &CaptureCfg,
) -> Result<Capture, Error<IO::Error>> {
    io.arm(duty_q15).map_err(Error::Io)?;
    let head = poll_done(io, cfg);
    // Release whatever the poll found: a rejected or wedged arm must not be
    // left holding the servo's burst FSM.
    let head = match head {
        Ok(h) => h,
        Err(e) => {
            let _ = io.release();
            return Err(e);
        }
    };
    let walk = walk_pages(io, cfg);
    io.release().map_err(Error::Io)?;
    let samples = walk?;
    Ok(Capture {
        samples,
        meta: Meta {
            pre_q15: pre.pre_q15,
            step_q15: duty_q15,
            step_index: head.step_index,
            start_cnt: head.start_cnt,
            pwm_arr: head.pwm_arr,
            start_dir: head.start_dir,
            restore_dir: head.restore_dir,
            vbus_raw: pre.vbus_raw,
            bias: pre.bias,
        },
    })
}

fn poll_done<IO: BurstIo>(io: &mut IO, cfg: &CaptureCfg) -> Result<Header, Error<IO::Error>> {
    let mut last = STATE_IDLE;
    for _ in 0..cfg.max_polls {
        let raw = io
            .read_burst(reg::PAGE_ECHO.addr, READ_LEN)
            .map_err(Error::Io)?;
        let h = parse_header(&raw).ok_or(Error::Short {
            got: raw.len(),
            want: READ_LEN as usize,
        })?;
        last = h.state;
        match h.state {
            STATE_DONE => {
                if h.samples_len as usize != SAMPLES {
                    return Err(Error::Len { got: h.samples_len });
                }
                return Ok(h);
            }
            STATE_REJECTED => return Err(Error::Rejected),
            _ => io.pause_ms(cfg.poll_ms),
        }
    }
    Err(Error::Timeout {
        state: last,
        polls: cfg.max_polls,
    })
}

fn walk_pages<IO: BurstIo>(io: &mut IO, cfg: &CaptureCfg) -> Result<Vec<u16>, Error<IO::Error>> {
    let mut out = Vec::with_capacity(SAMPLES);
    for page in 0..PAGES {
        let mut got = false;
        for _ in 0..cfg.page_tries {
            io.select_page(page).map_err(Error::Io)?;
            let raw = io
                .read_burst(reg::PAGE_ECHO.addr, READ_LEN)
                .map_err(Error::Io)?;
            let h = parse_header(&raw).ok_or(Error::Short {
                got: raw.len(),
                want: READ_LEN as usize,
            })?;
            if h.page_echo == page {
                out.extend(parse_page(&raw));
                got = true;
                break;
            }
        }
        if !got {
            return Err(Error::Page {
                page,
                tries: cfg.page_tries,
            });
        }
    }
    Ok(out)
}

// --- csv --------------------------------------------------------------------

/// Column header of a `burst-N.csv`. The meta columns carry a value on the
/// first data row only - they are one capture's constants, not a series.
pub const CSV_HEADER: &str = "k,current_raw,pre_q15,step_q15,step_index,start_cnt,pwm_arr,\
                              start_dir,restore_dir,vbus_raw,bias";

pub fn to_csv(cap: &Capture) -> String {
    let m = &cap.meta;
    let mut s = String::with_capacity(CSV_HEADER.len() + cap.samples.len() * 10);
    s.push_str(CSV_HEADER);
    s.push('\n');
    for (k, v) in cap.samples.iter().enumerate() {
        if k == 0 {
            let _ = fmt::Write::write_fmt(
                &mut s,
                format_args!(
                    "0,{v},{},{},{},{},{},{},{},{},{}\n",
                    m.pre_q15,
                    m.step_q15,
                    m.step_index,
                    m.start_cnt,
                    m.pwm_arr,
                    m.start_dir,
                    m.restore_dir,
                    m.vbus_raw,
                    m.bias
                ),
            );
        } else {
            let _ = fmt::Write::write_fmt(&mut s, format_args!("{k},{v}\n"));
        }
    }
    s
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CsvError {
    pub line: usize,
    pub what: &'static str,
}

impl fmt::Display for CsvError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "burst csv line {}: {}", self.line, self.what)
    }
}

/// Read a `burst-N.csv` back. Rows after the first carry only k and the
/// code; a row that repeats the meta is accepted and ignored.
pub fn from_csv(text: &str) -> Result<Capture, CsvError> {
    let mut samples = Vec::new();
    let mut meta = Meta::default();
    for (n, line) in text.lines().enumerate() {
        let line = line.trim();
        if line.is_empty() || n == 0 {
            continue;
        }
        let c: Vec<&str> = line.split(',').collect();
        let err = |what| CsvError { line: n + 1, what };
        if c.len() < 2 {
            return Err(err("needs at least k and current_raw"));
        }
        let raw: u16 = c[1].parse().map_err(|_| err("current_raw not a u16"))?;
        if samples.is_empty() {
            if c.len() < 11 {
                return Err(err("first row must carry the meta columns"));
            }
            let i16at = |i: usize| c[i].parse::<i16>().map_err(|_| err("meta not an i16"));
            let u16at = |i: usize| c[i].parse::<u16>().map_err(|_| err("meta not a u16"));
            let u8at = |i: usize| c[i].parse::<u8>().map_err(|_| err("meta not a u8"));
            meta = Meta {
                pre_q15: i16at(2)?,
                step_q15: i16at(3)?,
                step_index: u16at(4)?,
                start_cnt: u16at(5)?,
                pwm_arr: u16at(6)?,
                start_dir: u8at(7)?,
                restore_dir: u8at(8)?,
                vbus_raw: u16at(9)?,
                bias: u16at(10)?,
            };
        }
        samples.push(raw);
    }
    if samples.is_empty() {
        return Err(CsvError {
            line: 0,
            what: "no sample rows",
        });
    }
    Ok(Capture { samples, meta })
}

/// The registers a driver writes to arm and page, re-exported so a
/// [`BurstIo`] implementation has one import.
pub mod wire {
    use crate::regs::Reg;

    pub const DUTY_Q15: Reg = super::control::BURST_DUTY_Q15;
    pub const ARM: Reg = super::control::BURST_ARM;
    pub const PAGE: Reg = super::control::BURST_PAGE;
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A servo that answers the handshake, with knobs for the failure
    /// modes the client has to survive.
    struct Fake {
        state: u8,
        polls: u32,
        /// Polls to spend in Capturing before Done.
        busy: u32,
        page: u8,
        /// Pages that answer PAGE_BUSY once before echoing.
        stutter: bool,
        stuttered: Vec<u8>,
        armed: Option<i16>,
        released: bool,
        reads: u32,
    }

    impl Fake {
        fn new() -> Self {
            Self {
                state: STATE_IDLE,
                polls: 0,
                busy: 2,
                page: 0,
                stutter: false,
                stuttered: Vec::new(),
                armed: None,
                released: false,
                reads: 0,
            }
        }

        fn reply(&mut self, echo: u8) -> Vec<u8> {
            let mut raw = vec![0u8; READ_LEN as usize];
            raw[off(reg::PAGE_ECHO)] = echo;
            raw[off(reg::STATE)] = self.state;
            if self.state == STATE_DONE {
                for k in 0..PAGE_SAMPLES {
                    let v = (self.page as u16) * PAGE_SAMPLES as u16 + k as u16;
                    let at = off(reg::SAMPLES) + 2 * k;
                    raw[at..at + 2].copy_from_slice(&v.to_le_bytes());
                }
                raw[off(reg::SAMPLES_LEN)..][..2].copy_from_slice(&(SAMPLES as u16).to_le_bytes());
                raw[off(reg::STEP_INDEX)..][..2].copy_from_slice(&485u16.to_le_bytes());
                raw[off(reg::START_CNT)..][..2].copy_from_slice(&1094u16.to_le_bytes());
                raw[off(reg::PWM_ARR)..][..2].copy_from_slice(&1200u16.to_le_bytes());
                raw[off(reg::START_DIR)] = 1;
                raw[off(reg::RESTORE_DIR)] = 0;
            }
            raw
        }
    }

    impl BurstIo for Fake {
        type Error = &'static str;

        fn arm(&mut self, duty_q15: i16) -> Result<(), &'static str> {
            self.armed = Some(duty_q15);
            self.state = STATE_CAPTURING;
            self.polls = 0;
            Ok(())
        }

        fn select_page(&mut self, page: u8) -> Result<(), &'static str> {
            self.page = page;
            Ok(())
        }

        fn release(&mut self) -> Result<(), &'static str> {
            self.released = true;
            self.state = STATE_IDLE;
            Ok(())
        }

        fn read_burst(&mut self, _addr: u16, _len: u16) -> Result<Vec<u8>, &'static str> {
            self.reads += 1;
            if self.state == STATE_CAPTURING {
                self.polls += 1;
                if self.polls > self.busy {
                    self.state = STATE_DONE;
                }
                return Ok(self.reply(PAGE_BUSY));
            }
            let page = self.page;
            let echo = if self.stutter && !self.stuttered.contains(&page) {
                self.stuttered.push(page);
                PAGE_BUSY
            } else {
                page
            };
            Ok(self.reply(echo))
        }

        fn pause_ms(&mut self, _ms: u32) {}
    }

    #[test]
    fn capture_walks_every_page_in_order() {
        let mut f = Fake::new();
        let pre = Pre {
            pre_q15: 0,
            vbus_raw: 2169,
            bias: 118,
        };
        let cap = capture(&mut f, 13107, pre, &CaptureCfg::default()).expect("capture");
        assert_eq!(cap.samples.len(), SAMPLES);
        // the fake numbers every code with its own index
        assert!(
            cap.samples
                .iter()
                .enumerate()
                .all(|(k, v)| *v as usize == k)
        );
        assert_eq!(cap.meta.step_q15, 13107);
        assert_eq!(cap.meta.step_index, 485);
        assert_eq!(cap.meta.vbus_raw, 2169);
        assert_eq!(cap.meta.bias, 118);
        assert_eq!(f.armed, Some(13107));
        assert!(f.released, "arm must be dropped after the walk");
    }

    #[test]
    fn a_stuttering_page_is_retried_not_spliced() {
        let mut f = Fake::new();
        f.stutter = true;
        let cap = capture(&mut f, 8520, Pre::default(), &CaptureCfg::default()).expect("capture");
        assert!(
            cap.samples
                .iter()
                .enumerate()
                .all(|(k, v)| *v as usize == k)
        );
    }

    #[test]
    fn a_page_that_never_echoes_fails_the_capture() {
        struct Mute(Fake);
        impl BurstIo for Mute {
            type Error = &'static str;
            fn arm(&mut self, d: i16) -> Result<(), &'static str> {
                self.0.arm(d)
            }
            fn select_page(&mut self, p: u8) -> Result<(), &'static str> {
                self.0.select_page(p)
            }
            fn release(&mut self) -> Result<(), &'static str> {
                self.0.release()
            }
            fn read_burst(&mut self, a: u16, l: u16) -> Result<Vec<u8>, &'static str> {
                let mut raw = self.0.read_burst(a, l)?;
                if self.0.state != STATE_CAPTURING {
                    raw[off(reg::PAGE_ECHO)] = PAGE_BUSY;
                }
                Ok(raw)
            }
            fn pause_ms(&mut self, ms: u32) {
                self.0.pause_ms(ms)
            }
        }
        let mut m = Mute(Fake::new());
        let e = capture(&mut m, 8520, Pre::default(), &CaptureCfg::default()).unwrap_err();
        assert_eq!(
            e,
            Error::Page {
                page: 0,
                tries: CaptureCfg::default().page_tries
            }
        );
        assert!(m.0.released, "a failed walk still releases");
    }

    #[test]
    fn a_rejected_arm_reports_and_releases() {
        let mut f = Fake::new();
        f.busy = 0;
        // arm() moves to Capturing; force the servo's refusal on the first poll
        struct Reject(Fake);
        impl BurstIo for Reject {
            type Error = &'static str;
            fn arm(&mut self, d: i16) -> Result<(), &'static str> {
                self.0.arm(d)?;
                self.0.state = STATE_REJECTED;
                Ok(())
            }
            fn select_page(&mut self, p: u8) -> Result<(), &'static str> {
                self.0.select_page(p)
            }
            fn release(&mut self) -> Result<(), &'static str> {
                self.0.release()
            }
            fn read_burst(&mut self, a: u16, l: u16) -> Result<Vec<u8>, &'static str> {
                self.0.read_burst(a, l)
            }
            fn pause_ms(&mut self, ms: u32) {
                self.0.pause_ms(ms)
            }
        }
        let mut r = Reject(f);
        let e = capture(&mut r, 8520, Pre::default(), &CaptureCfg::default()).unwrap_err();
        assert_eq!(e, Error::Rejected);
        assert!(r.0.released);
    }

    #[test]
    fn a_wedged_state_times_out_within_the_budget() {
        struct Wedged;
        impl BurstIo for Wedged {
            type Error = &'static str;
            fn arm(&mut self, _: i16) -> Result<(), &'static str> {
                Ok(())
            }
            fn select_page(&mut self, _: u8) -> Result<(), &'static str> {
                Ok(())
            }
            fn release(&mut self) -> Result<(), &'static str> {
                Ok(())
            }
            fn read_burst(&mut self, _: u16, _: u16) -> Result<Vec<u8>, &'static str> {
                let mut raw = vec![0u8; READ_LEN as usize];
                raw[off(reg::STATE)] = STATE_CAPTURING;
                Ok(raw)
            }
            fn pause_ms(&mut self, _: u32) {}
        }
        let cfg = CaptureCfg {
            max_polls: 7,
            ..CaptureCfg::default()
        };
        let e = capture(&mut Wedged, 0, Pre::default(), &cfg).unwrap_err();
        assert_eq!(
            e,
            Error::Timeout {
                state: STATE_CAPTURING,
                polls: 7
            }
        );
    }

    #[test]
    fn header_offsets_match_the_register_map() {
        assert_eq!(off(reg::PAGE_ECHO), 0);
        assert_eq!(off(reg::STATE), 1);
        assert_eq!(off(reg::SAMPLES), 2);
        assert_eq!(off(reg::SAMPLES_LEN), 2 + 2 * PAGE_SAMPLES);
        // the header's last byte is the last byte one READ can carry
        assert_eq!(off(reg::RESTORE_DIR), READ_LEN as usize - 1);
        assert_eq!(PAGES as usize * PAGE_SAMPLES, SAMPLES);
    }

    #[test]
    fn csv_round_trips() {
        let cap = Capture {
            samples: (0..SAMPLES).map(|k| 100 + (k % 300) as u16).collect(),
            meta: Meta {
                pre_q15: 3276,
                step_q15: -8520,
                step_index: 485,
                start_cnt: 1055,
                pwm_arr: 1200,
                start_dir: 1,
                restore_dir: 0,
                vbus_raw: 2154,
                bias: 113,
            },
        };
        let back = from_csv(&to_csv(&cap)).expect("parse");
        assert_eq!(back, cap);
    }

    #[test]
    fn csv_rejects_a_first_row_without_meta() {
        let e = from_csv("k,current_raw\n0,113\n1,112\n").unwrap_err();
        assert_eq!(e.line, 2);
    }

    #[test]
    fn a_bench_fixture_parses_with_its_meta() {
        let text = include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/testdata/burst/rest-to-40.csv"
        ));
        let cap = from_csv(text).expect("fixture parses");
        assert_eq!(cap.samples.len(), SAMPLES);
        assert_eq!(cap.meta.step_q15, 13107);
        assert_eq!(cap.meta.pre_q15, 0);
        assert_eq!(cap.meta.step_index, 485);
        assert_eq!(cap.meta.pwm_arr, 1200);
    }
}

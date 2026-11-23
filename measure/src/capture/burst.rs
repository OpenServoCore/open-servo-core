//! The shunt burst (protocol sec 5.8): arm, poll, walk the eight readback
//! pages, release. One capture is 960 raw ADC codes at one conversion
//! period each, a current step sampled about 46 times a PWM period, and
//! lands as osc-lab's `burst-K.csv`.
//!
//! `chans` interleaves voltage channels behind the shunt: every frame is
//! the shunt then the selected extras in bit order. [`CHAN_INTERLEAVE`]
//! puts another shunt ahead of every extra after the first, so the shunt
//! keeps every second conversion.
//!
//! The servo copies a page into the readback window from its main loop
//! with `page_echo` as the guard (0xFF while copying, the page after a
//! fence), so a reply whose echo is not the page asked for is a plain
//! retry. The burst window is ADC time the scan does not run, so the rail,
//! the current zero and the terminal bias come from reads taken just
//! before the arm ([`Pre`]).

use core::fmt;
use std::time::Duration;

use osc_client::descriptor::Descriptor;
use osc_client::{Client, Error as BusError, Id, pipe::Pipe};

use super::rows::Reg;
use crate::state::field;

pub const SAMPLES: usize = 960;
pub const PAGE_SAMPLES: usize = 120;
pub const PAGES: u8 = 8;
/// One READ of the readback section: the page and the whole header.
pub const READ_LEN: u16 = 252;

pub const STATE_IDLE: u8 = 0;
pub const STATE_DONE: u8 = 3;
pub const STATE_REJECTED: u8 = 4;

pub const CHAN_VMOTOR_A: u8 = 1 << 0;
pub const CHAN_VMOTOR_B: u8 = 1 << 1;
pub const CHAN_VBUS: u8 = 1 << 2;
/// Layout bit: a shunt slot ahead of every extra after the first.
pub const CHAN_INTERLEAVE: u8 = 1 << 3;
pub const CHAN_EXTRAS: u8 = CHAN_VMOTOR_A | CHAN_VMOTOR_B | CHAN_VBUS;
pub const CHANS_MAX: u8 = CHAN_EXTRAS | CHAN_INTERLEAVE;
/// Both terminals at half rate each, the shunt at frame_len 2's rate.
pub const CHANS_DIFF: u8 = CHAN_VMOTOR_A | CHAN_VMOTOR_B | CHAN_INTERLEAVE;

/// The applied volts, duty times the rail, the firmware arms a burst at.
pub const MAX_MV: f64 = 3200.0;
/// `limit_flags` bit 3: the stall permit's grant is live.
pub const LIMIT_PERMIT_LIVE: u8 = 1 << 3;

/// The readback section, mirrored from the table; the test holds it to
/// the descriptor.
pub mod readback {
    use super::Reg;

    pub const PAGE_ECHO: Reg = Reg {
        name: "page_echo",
        addr: 0x2c0,
        width: 1,
    };
    pub const STATE: Reg = Reg {
        name: "state",
        addr: 0x2c1,
        width: 1,
    };
    pub const SAMPLES: Reg = Reg {
        name: "samples",
        addr: 0x2c2,
        width: 240,
    };
    pub const SAMPLES_LEN: Reg = Reg {
        name: "samples_len",
        addr: 0x3b2,
        width: 2,
    };
    pub const STEP_INDEX: Reg = Reg {
        name: "step_index",
        addr: 0x3b4,
        width: 2,
    };
    pub const START_CNT: Reg = Reg {
        name: "start_cnt",
        addr: 0x3b6,
        width: 2,
    };
    pub const PWM_ARR: Reg = Reg {
        name: "pwm_arr",
        addr: 0x3b8,
        width: 2,
    };
    pub const START_DIR: Reg = Reg {
        name: "start_dir",
        addr: 0x3ba,
        width: 1,
    };
    pub const RESTORE_DIR: Reg = Reg {
        name: "restore_dir",
        addr: 0x3bb,
        width: 1,
    };
    pub const CHANS_ECHO: Reg = Reg {
        name: "chans_echo",
        addr: 0x3bc,
        width: 1,
    };
    pub const FRAME_LEN: Reg = Reg {
        name: "frame_len",
        addr: 0x3bd,
        width: 1,
    };

    pub const ALL: [Reg; 11] = [
        PAGE_ECHO,
        STATE,
        SAMPLES,
        SAMPLES_LEN,
        STEP_INDEX,
        START_CNT,
        PWM_ARR,
        START_DIR,
        RESTORE_DIR,
        CHANS_ECHO,
        FRAME_LEN,
    ];
}

use readback as reg;

const fn shunts(chans: u8) -> usize {
    let n = (chans & CHAN_EXTRAS).count_ones() as usize;
    if chans & CHAN_INTERLEAVE != 0 && n > 0 {
        n
    } else {
        1
    }
}

/// Codes per frame for a mask: the shunts plus one per selected extra.
pub const fn frame_len(chans: u8) -> usize {
    shunts(chans) + (chans & CHAN_EXTRAS).count_ones() as usize
}

/// What the host read just before the arm: the burst suspends the scan.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Pre {
    /// Duty in force over the pre-step half of the capture, q15.
    pub pre_q15: i16,
    pub vbus_raw: u16,
    /// The current zero as the servo's trough tracker reports it.
    pub bias: u16,
    /// The terminal divider bias measured at boot, raw.
    pub vmotor_bias: u16,
    pub pos: u16,
    /// The rotor is held against a mechanical stop: no back-EMF.
    pub seated: bool,
    pub drive_polarity: bool,
}

impl Default for Pre {
    fn default() -> Self {
        Self {
            pre_q15: 0,
            vbus_raw: 0,
            bias: 0,
            vmotor_bias: 0,
            pos: 0,
            seated: false,
            drive_polarity: true,
        }
    }
}

/// The burst header as one READ returns it, and the pre-arm reads.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Meta {
    pub pre_q15: i16,
    pub step_q15: i16,
    /// Raw sample index, not frame index.
    pub step_index: u16,
    pub start_cnt: u16,
    pub pwm_arr: u16,
    pub start_dir: u8,
    pub restore_dir: u8,
    pub vbus_raw: u16,
    pub bias: u16,
    pub chans: u8,
    pub frame_len: u8,
    pub vmotor_bias: u16,
    pub pos: u16,
    pub seated: bool,
    pub drive_polarity: bool,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Capture {
    /// Raw codes in conversion order, frames interleaved.
    pub samples: Vec<u16>,
    pub meta: Meta,
}

/// Poll and retry budgets: the capture is about 2 ms, so 200 polls at 2 ms
/// is a 400 ms ceiling only a wedged servo meets.
#[derive(Copy, Clone, Debug)]
pub struct CaptureCfg {
    pub poll_ms: u32,
    pub max_polls: u32,
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

/// The wire moves a capture needs. `arm` applies duty, mask and arm in the
/// same instant (HOLD all three, one COMMIT): a servo that sees the arm
/// before the duty captures the old level.
pub(crate) trait BurstIo {
    type Error;

    async fn arm(&mut self, duty_q15: i16, chans: u8) -> Result<(), Self::Error>;
    async fn select_page(&mut self, page: u8) -> Result<(), Self::Error>;
    async fn release(&mut self) -> Result<(), Self::Error>;
    async fn read_burst(&mut self, addr: u16, len: u16) -> Result<Vec<u8>, Self::Error>;
    async fn pause_ms(&mut self, ms: u32);
}

const EVERY_RULE: &str = "it arms only with torque on, in OpenLoop, with no fault latched, with \
                          no telemetry capture running, at most 3.2 V applied (duty times the \
                          rail), 100 ms or more after the burst before, and with the shaft \
                          inside the soft limits unless the stall permit is held";

/// What the host reads once an arm is refused, to name the rule.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ArmSeen {
    pub vbus_counts: u16,
    /// Millivolts a vcount of the terminal taps.
    pub mv_per_count: f64,
    pub pos: u16,
    pub soft: (i32, i32),
    pub limit_flags: u8,
    pub fault_flags: u8,
}

/// Why the servo refused an arm at `duty_q15`: the rules `seen` shows
/// broken, else every rule, the ones the host cannot see included.
pub fn rejected(duty_q15: i16, seen: &ArmSeen) -> String {
    let mut why = Vec::new();
    if seen.fault_flags != 0 {
        why.push(format!(
            "it has a fault latched (flags {:#04x})",
            seen.fault_flags
        ));
    }
    let cap = (MAX_MV / seen.mv_per_count).round() as u32;
    let volts = (duty_q15.unsigned_abs() as u32 * seen.vbus_counts as u32) >> 15;
    if seen.mv_per_count > 0.0 && volts > cap {
        let rail = seen.vbus_counts as f64 * seen.mv_per_count / 1000.0;
        why.push(format!(
            "{:.1}% of its {rail:.2} V rail applies {:.3} V, over the 3.2 V a burst may",
            duty_q15.unsigned_abs() as f64 / 32767.0 * 100.0,
            volts as f64 * seen.mv_per_count / 1000.0
        ));
    }
    let (lo, hi) = seen.soft;
    let inside = (seen.pos as i32) > lo && (seen.pos as i32) < hi;
    if !inside && seen.limit_flags & LIMIT_PERMIT_LIVE == 0 {
        why.push(format!(
            "the shaft at {} is outside the soft limits {lo}..{hi} and the stall permit is not held",
            seen.pos
        ));
    }
    if why.is_empty() {
        return format!("the servo refused the burst: {EVERY_RULE}");
    }
    format!("the servo refused the burst: {}", why.join(", and "))
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Error<E> {
    Io(E),
    Rejected,
    Timeout { state: u8, polls: u32 },
    Page { page: u8, tries: u32 },
    Short { got: usize, want: usize },
    Len { got: u16 },
    Chans { want: u8, got: u8 },
    FrameLen { chans: u8, got: u8 },
}

impl<E: fmt::Display> fmt::Display for Error<E> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Io(e) => write!(f, "{e}"),
            Error::Rejected => write!(f, "the servo refused the burst: {EVERY_RULE}"),
            Error::Timeout { state, polls } => {
                write!(f, "burst stuck in state {state} after {polls} polls")
            }
            Error::Page { page, tries } => write!(f, "page {page} never echoed in {tries} tries"),
            Error::Short { got, want } => write!(f, "burst read returned {got} B, need {want}"),
            Error::Len { got } => {
                write!(f, "servo published samples_len {got}, expected {SAMPLES}")
            }
            Error::Chans { want, got } => write!(f, "armed chans {want}, servo echoed {got}"),
            Error::FrameLen { chans, got } => write!(
                f,
                "chans {chans} needs frame_len {}, servo published {got}",
                frame_len(*chans)
            ),
        }
    }
}

const fn off(r: Reg) -> usize {
    (r.addr - reg::PAGE_ECHO.addr) as usize
}

fn u16_at(raw: &[u8], at: usize) -> u16 {
    u16::from_le_bytes([raw[at], raw[at + 1]])
}

/// The header of one readback reply. Only `page_echo` and `state` mean
/// anything before the capture is done.
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

/// The 120 codes of the page a reply carries.
pub fn parse_page(raw: &[u8]) -> impl Iterator<Item = u16> + '_ {
    let base = off(reg::SAMPLES);
    (0..PAGE_SAMPLES).map(move |k| u16_at(raw, base + 2 * k))
}

/// The mask echo and frame length sit one past the page READ.
const TAIL_LEN: u16 = reg::FRAME_LEN.addr + 1 - reg::CHANS_ECHO.addr;

/// One capture end to end: arm at `duty_q15` sampling `chans`, wait for
/// done, check the echoed mask, walk the pages, release whatever happened.
pub(crate) async fn capture<IO: BurstIo>(
    io: &mut IO,
    duty_q15: i16,
    chans: u8,
    pre: Pre,
    cfg: &CaptureCfg,
) -> Result<Capture, Error<IO::Error>> {
    io.arm(duty_q15, chans).await.map_err(Error::Io)?;
    let head = match poll_done(io, cfg).await {
        Ok(h) => tail(io, chans).await.map(|t| (h, t)),
        Err(e) => Err(e),
    };
    // a rejected or wedged arm must not be left holding the burst machine
    let (head, (echo, flen)) = match head {
        Ok(h) => h,
        Err(e) => {
            let _ = io.release().await;
            return Err(e);
        }
    };
    let walk = walk_pages(io, cfg).await;
    io.release().await.map_err(Error::Io)?;
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
            chans: echo,
            frame_len: flen,
            vmotor_bias: pre.vmotor_bias,
            pos: pre.pos,
            seated: pre.seated,
            drive_polarity: pre.drive_polarity,
        },
    })
}

async fn tail<IO: BurstIo>(io: &mut IO, chans: u8) -> Result<(u8, u8), Error<IO::Error>> {
    let raw = io
        .read_burst(reg::CHANS_ECHO.addr, TAIL_LEN)
        .await
        .map_err(Error::Io)?;
    let (Some(&echo), Some(&flen)) = (raw.first(), raw.get(1)) else {
        return Err(Error::Short {
            got: raw.len(),
            want: TAIL_LEN as usize,
        });
    };
    if echo != chans {
        return Err(Error::Chans {
            want: chans,
            got: echo,
        });
    }
    if flen as usize != frame_len(chans) {
        return Err(Error::FrameLen { chans, got: flen });
    }
    Ok((echo, flen))
}

async fn poll_done<IO: BurstIo>(io: &mut IO, cfg: &CaptureCfg) -> Result<Header, Error<IO::Error>> {
    let mut last = STATE_IDLE;
    for _ in 0..cfg.max_polls {
        let raw = io
            .read_burst(reg::PAGE_ECHO.addr, READ_LEN)
            .await
            .map_err(Error::Io)?;
        let h = parse_header(&raw).ok_or(Error::Short {
            got: raw.len(),
            want: READ_LEN as usize,
        })?;
        last = h.state;
        match h.state {
            STATE_DONE if h.samples_len as usize != SAMPLES => {
                return Err(Error::Len { got: h.samples_len });
            }
            STATE_DONE => return Ok(h),
            STATE_REJECTED => return Err(Error::Rejected),
            _ => io.pause_ms(cfg.poll_ms).await,
        }
    }
    Err(Error::Timeout {
        state: last,
        polls: cfg.max_polls,
    })
}

async fn walk_pages<IO: BurstIo>(
    io: &mut IO,
    cfg: &CaptureCfg,
) -> Result<Vec<u16>, Error<IO::Error>> {
    let mut out = Vec::with_capacity(SAMPLES);
    for page in 0..PAGES {
        let mut got = false;
        for _ in 0..cfg.page_tries {
            io.select_page(page).await.map_err(Error::Io)?;
            let raw = io
                .read_burst(reg::PAGE_ECHO.addr, READ_LEN)
                .await
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

/// The handshake over the client. The chans byte is its own one-byte
/// write: the byte after it is a reserved alignment byte that refuses
/// writes. Before the arm `tel_count` goes to 0: the servo refuses a shunt
/// arm while a TEL count is armed.
struct Wire<'a, P: Pipe> {
    c: &'a mut Client<P>,
    id: Id,
    addrs: [u16; 5],
}

impl<P: Pipe> BurstIo for Wire<'_, P> {
    type Error = BusError;

    async fn arm(&mut self, duty_q15: i16, chans: u8) -> Result<(), BusError> {
        let [tel_count, duty, ch, arm, _] = self.addrs;
        self.c.write(self.id, tel_count, &[0, 0]).await?;
        self.c
            .write_hold(self.id, duty, &duty_q15.to_le_bytes())
            .await?;
        self.c.write_hold(self.id, ch, &[chans]).await?;
        self.c.write_hold(self.id, arm, &[1]).await?;
        self.c.commit().await
    }

    async fn select_page(&mut self, page: u8) -> Result<(), BusError> {
        self.c.write(self.id, self.addrs[4], &[page]).await
    }

    async fn release(&mut self) -> Result<(), BusError> {
        self.c.write(self.id, self.addrs[3], &[0]).await
    }

    async fn read_burst(&mut self, addr: u16, len: u16) -> Result<Vec<u8>, BusError> {
        self.c.read(self.id, addr, len).await
    }

    async fn pause_ms(&mut self, ms: u32) {
        self.c.pause(Duration::from_millis(ms as u64)).await
    }
}

/// One shunt burst on the servo at `duty_q15`, `pre` read before it.
pub async fn shunt<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    duty_q15: i16,
    chans: u8,
    pre: Pre,
    cfg: &CaptureCfg,
) -> Result<Capture, Error<BusError>> {
    let a = |n: &str| field(d, n).map(|f| f.addr).map_err(Error::Io);
    let addrs = [
        a("tel_count")?,
        a("duty_q15")?,
        a("chans")?,
        a("arm")?,
        a("page")?,
    ];
    capture(&mut Wire { c, id, addrs }, duty_q15, chans, pre, cfg).await
}

/// The column header of a `burst-K.csv`: `code` is the raw buffer in
/// conversion order; the meta columns carry a value on the first row only.
pub const CSV_HEADER: &str = "k,code,pre_q15,step_q15,step_index,start_cnt,pwm_arr,\
                              start_dir,restore_dir,vbus_raw,bias,chans,frame_len,vmotor_bias,\
                              pos,seated,drive_polarity";

pub fn to_csv(cap: &Capture) -> String {
    use fmt::Write as _;
    let m = &cap.meta;
    let mut s = String::with_capacity(CSV_HEADER.len() + cap.samples.len() * 10);
    s.push_str(CSV_HEADER);
    s.push('\n');
    for (k, v) in cap.samples.iter().enumerate() {
        if k == 0 {
            let _ = writeln!(
                s,
                "0,{v},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
                m.pre_q15,
                m.step_q15,
                m.step_index,
                m.start_cnt,
                m.pwm_arr,
                m.start_dir,
                m.restore_dir,
                m.vbus_raw,
                m.bias,
                m.chans,
                m.frame_len,
                m.vmotor_bias,
                m.pos,
                m.seated as u8,
                m.drive_polarity as u8
            );
        } else {
            let _ = writeln!(s, "{k},{v}");
        }
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;
    use futures_lite::future::block_on;

    const STATE_CAPTURING: u8 = 2;
    const PAGE_BUSY: u8 = 0xFF;

    /// A servo that answers the handshake, with knobs for the failures a
    /// host must survive.
    struct Fake {
        state: u8,
        polls: u32,
        busy: u32,
        page: u8,
        stutter: bool,
        stuttered: Vec<u8>,
        armed: Option<i16>,
        chans: u8,
        echo: Option<u8>,
        released: bool,
        reject: bool,
        mute: bool,
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
                chans: 0,
                echo: None,
                released: false,
                reject: false,
                mute: false,
            }
        }

        fn reply(&self, echo: u8) -> Vec<u8> {
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
            }
            raw
        }
    }

    impl BurstIo for Fake {
        type Error = &'static str;

        async fn arm(&mut self, duty_q15: i16, chans: u8) -> Result<(), &'static str> {
            self.armed = Some(duty_q15);
            self.chans = chans;
            self.state = if self.reject {
                STATE_REJECTED
            } else {
                STATE_CAPTURING
            };
            self.polls = 0;
            Ok(())
        }

        async fn select_page(&mut self, page: u8) -> Result<(), &'static str> {
            self.page = page;
            Ok(())
        }

        async fn release(&mut self) -> Result<(), &'static str> {
            self.released = true;
            self.state = STATE_IDLE;
            Ok(())
        }

        async fn read_burst(&mut self, addr: u16, len: u16) -> Result<Vec<u8>, &'static str> {
            if addr == reg::CHANS_ECHO.addr {
                let echo = self.echo.unwrap_or(self.chans);
                return Ok(vec![echo, frame_len(echo) as u8][..len as usize].to_vec());
            }
            if self.state == STATE_CAPTURING {
                self.polls += 1;
                if self.polls > self.busy {
                    self.state = STATE_DONE;
                }
                return Ok(self.reply(PAGE_BUSY));
            }
            let page = self.page;
            let echo = if self.mute {
                PAGE_BUSY
            } else if self.stutter && !self.stuttered.contains(&page) {
                self.stuttered.push(page);
                PAGE_BUSY
            } else {
                page
            };
            Ok(self.reply(echo))
        }

        async fn pause_ms(&mut self, _ms: u32) {}
    }

    fn run(f: &mut Fake, duty: i16, chans: u8, pre: Pre) -> Result<Capture, Error<&'static str>> {
        block_on(capture(f, duty, chans, pre, &CaptureCfg::default()))
    }

    #[test]
    fn the_readback_is_the_descriptors() {
        let p = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../descriptors/osc-servo/0.1.json"
        );
        let d: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(p).expect("read")).expect("json");
        for r in reg::ALL {
            let f = d["fields"]
                .as_array()
                .and_then(|a| a.iter().find(|f| f["name"] == r.name))
                .expect(r.name);
            assert_eq!(
                (f["addr"].as_u64(), f["width"].as_u64()),
                (Some(r.addr as u64), Some(r.width as u64)),
                "{}",
                r.name
            );
        }
        assert_eq!(off(reg::RESTORE_DIR), READ_LEN as usize - 1);
        assert_eq!(TAIL_LEN, 2);
    }

    #[test]
    fn capture_walks_every_page_in_order() {
        let mut f = Fake::new();
        let pre = Pre {
            vbus_raw: 2169,
            bias: 118,
            vmotor_bias: 779,
            pos: 122,
            seated: true,
            drive_polarity: false,
            pre_q15: 0,
        };
        let cap = run(&mut f, 13107, CHAN_VMOTOR_A | CHAN_VBUS, pre).expect("capture");
        assert!(
            cap.samples
                .iter()
                .enumerate()
                .all(|(k, v)| *v as usize == k)
        );
        assert_eq!(
            (
                cap.meta.step_q15,
                cap.meta.step_index,
                cap.meta.chans,
                cap.meta.frame_len
            ),
            (13107, 485, 5, 3)
        );
        assert!(f.released);
    }

    #[test]
    fn a_stuttering_page_is_retried_not_spliced() {
        let mut f = Fake::new();
        f.stutter = true;
        let cap = run(&mut f, 8520, 0, Pre::default()).expect("capture");
        assert!(
            cap.samples
                .iter()
                .enumerate()
                .all(|(k, v)| *v as usize == k)
        );
    }

    #[test]
    fn every_failure_releases_the_burst() {
        let mut f = Fake::new();
        f.echo = Some(0);
        assert_eq!(
            run(&mut f, 8520, CHAN_VMOTOR_A, Pre::default()),
            Err(Error::Chans { want: 1, got: 0 })
        );
        assert!(f.released);
        let mut f = Fake::new();
        f.mute = true;
        assert_eq!(
            run(&mut f, 8520, 0, Pre::default()),
            Err(Error::Page { page: 0, tries: 5 })
        );
        assert!(f.released);
        let mut f = Fake::new();
        f.reject = true;
        assert_eq!(run(&mut f, 8520, 0, Pre::default()), Err(Error::Rejected));
        assert!(f.released);
        let mut f = Fake::new();
        f.busy = u32::MAX;
        let e = block_on(capture(
            &mut f,
            0,
            0,
            Pre::default(),
            &CaptureCfg {
                max_polls: 7,
                ..CaptureCfg::default()
            },
        ));
        assert_eq!(
            e,
            Err(Error::Timeout {
                state: STATE_CAPTURING,
                polls: 7
            })
        );
    }

    #[test]
    fn a_rejected_arm_names_its_reason() {
        let seen = ArmSeen {
            vbus_counts: 3204,
            mv_per_count: 3.3 / 4096.0 * 10_100.0 / 3_300.0 * 1000.0,
            pos: 2029,
            soft: (432, 3626),
            limit_flags: 0,
            fault_flags: 0,
        };
        assert_eq!(rejected(13106, &seen), Error::<&str>::Rejected.to_string());
        let risen = ArmSeen {
            vbus_counts: 3248,
            ..seen
        };
        assert_eq!(
            rejected(13106, &risen),
            "the servo refused the burst: 40.0% of its 8.01 V rail applies 3.203 V, over the 3.2 V a burst may"
        );
        let outside = ArmSeen { pos: 3626, ..seen };
        assert_eq!(
            rejected(8192, &outside),
            "the servo refused the burst: the shaft at 3626 is outside the soft limits 432..3626 and the stall permit is not held"
        );
    }

    #[test]
    fn frame_lengths_follow_the_mask() {
        assert_eq!(frame_len(0), 1);
        assert_eq!(frame_len(CHAN_VMOTOR_A | CHAN_VBUS), 3);
        assert_eq!(frame_len(CHANS_DIFF), 4);
        assert_eq!(frame_len(CHANS_MAX), 6);
        for m in 0..=CHANS_MAX {
            assert_eq!(SAMPLES % frame_len(m), 0, "chans {m}");
        }
    }

    /// A committed capture's `burst-0.csv` parsed back into a capture and
    /// written again is the same bytes.
    #[test]
    fn a_burst_csv_is_byte_compatible_with_a_committed_capture() {
        let p = concat!(env!("CARGO_MANIFEST_DIR"), "/testdata/captures/burst-0.csv");
        let text = std::fs::read_to_string(p).expect("fixture");
        let mut lines = text.lines();
        assert_eq!(lines.next(), Some(CSV_HEADER));
        let first: Vec<i64> = lines
            .next()
            .expect("row 0")
            .split(',')
            .map(|v| v.parse().expect("int"))
            .collect();
        assert_eq!(first.len(), 17);
        let mut samples = vec![first[1] as u16];
        for (k, line) in lines.enumerate() {
            let (i, v) = line.split_once(',').expect("k,code");
            assert_eq!(i.parse::<usize>().expect("k"), k + 1);
            samples.push(v.parse().expect("code"));
        }
        let m = |i: usize| first[i];
        let cap = Capture {
            samples,
            meta: Meta {
                pre_q15: m(2) as i16,
                step_q15: m(3) as i16,
                step_index: m(4) as u16,
                start_cnt: m(5) as u16,
                pwm_arr: m(6) as u16,
                start_dir: m(7) as u8,
                restore_dir: m(8) as u8,
                vbus_raw: m(9) as u16,
                bias: m(10) as u16,
                chans: m(11) as u8,
                frame_len: m(12) as u8,
                vmotor_bias: m(13) as u16,
                pos: m(14) as u16,
                seated: m(15) != 0,
                drive_polarity: m(16) != 0,
            },
        };
        assert_eq!(cap.samples.len(), SAMPLES);
        assert!(to_csv(&cap) == text, "burst-K.csv drifted");
    }
}

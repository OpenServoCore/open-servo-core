//! tinyboot commands over TB_EXCHANGE (link sec 7), typed both ways. The
//! bus must already be in tinyboot mode ([`Client::set_bus`]). `select`
//! addresses the fleet; every other command reaches only the device the
//! last `select` left selected.

use std::time::Duration;

use osc_host::engine::{Outcome, TbExpect};
use tinyboot_protocol::{Cmd, EraseData, InfoData, VerifyData};

pub use tinyboot_protocol::{BaudRate, ResetFlags, Status, WriteFlags};

use crate::client::{Client, TbFrame, TbReply, desync_msg};
use crate::error::{Error, LinkError, RejectReason};
use crate::pipe::Pipe;

/// The devices a `select` drew: each clean reply's id, as the device sent
/// it (least significant byte first), and the garble the exchange heard.
/// Any garble means more than one device answered.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Selected {
    pub ids: Vec<Vec<u8>>,
    pub garble: u16,
}

/// A device's Info reply: [`InfoData`]'s fields, which is packed and
/// derives neither `Debug` nor `PartialEq`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BootInfo {
    /// App region capacity, bytes.
    pub capacity: u32,
    pub erase_size: u16,
    /// Packed 5.5.6 (`tinyboot_protocol::unpack_version`), `0xFFFF` none.
    pub boot_version: u16,
    pub app_version: u16,
    /// 0 bootloader, 1 app.
    pub mode: u8,
    /// Stored [`BaudRate`] index, `0xFF` the build default.
    pub boot_baud: u8,
    pub compat: u32,
    /// Largest request payload the device accepts.
    pub max_payload: u16,
}

/// Select every device whose id starts with the first `bits` bits of
/// `prefix` (bit `k` is `prefix[k / 8] >> (k % 8) & 1`) and deselect the
/// rest. Each selected device answers with its id after a slot drawn from
/// `seed`. A `prefix` shorter than `bits` is refused unsent.
pub async fn select<P: Pipe>(
    c: &mut Client<P>,
    seed: u8,
    prefix: &[u8],
    bits: u8,
) -> Result<Selected, Error> {
    let Some(prefix) = prefix.get(..bits.div_ceil(8) as usize) else {
        return Err(Error::Link(LinkError::Rejected(RejectReason::BadPayload)));
    };
    let mut data = vec![seed, bits];
    data.extend_from_slice(prefix);
    let window = select_window(c.info().bps, prefix.len());
    let r = c
        .tb_exchange(TbExpect::Collect, window, Cmd::Select as u8, 0, 0, &data)
        .await?;
    selected(r)
}

/// Every device's id, sorted bytewise as sent: a depth-first walk of
/// `select` prefixes that splits wherever more than one device answers. On
/// a clean bus it costs `3S + 2 - N` probes for `N` devices, `S` the
/// prefixes two or more ids share. The walk leaves the selection to its
/// last probe: select by full id before any other command.
pub async fn discover<P: Pipe>(c: &mut Client<P>) -> Result<Vec<Vec<u8>>, Error> {
    const FULL: u8 = (MAX_UID * 8) as u8;
    const FULL_DEPTH_RETRIES: usize = 3;
    let mut seed = 0;
    let mut found = Vec::new();
    let mut stack = vec![(0u8, [0u8; MAX_UID as usize])];
    while let Some((bits, prefix)) = stack.pop() {
        let mut p = probe(c, &mut seed, &prefix, bits).await?;
        if p == Probe::Empty {
            p = probe(c, &mut seed, &prefix, bits).await?;
        }
        if bits == FULL {
            // Ids are unique, so at most one device matches a full-depth
            // prefix: a collision here is noise. Retry, then skip the leaf.
            for _ in 0..FULL_DEPTH_RETRIES {
                if p != Probe::Many {
                    break;
                }
                p = probe(c, &mut seed, &prefix, bits).await?;
            }
        }
        match p {
            Probe::Empty => {}
            Probe::One(id) => found.push(id),
            Probe::Many if bits < FULL => {
                let mut one = prefix;
                one[bits as usize / 8] |= 1 << (bits % 8);
                stack.push((bits + 1, one));
                stack.push((bits + 1, prefix));
            }
            Probe::Many => {}
        }
    }
    found.sort();
    Ok(found)
}

#[derive(PartialEq)]
enum Probe {
    Empty,
    /// One clean reply and no garble: its CRC checked, the id as sent.
    One(Vec<u8>),
    /// More than one reply, or any garble.
    Many,
}

/// A `select` on its own seed, so slot draws re-roll probe to probe.
async fn probe<P: Pipe>(
    c: &mut Client<P>,
    seed: &mut u8,
    prefix: &[u8],
    bits: u8,
) -> Result<Probe, Error> {
    let s = select(c, *seed, prefix, bits).await?;
    *seed = seed.wrapping_add(1);
    Ok(match (s.ids.as_slice(), s.garble) {
        ([], 0) => Probe::Empty,
        ([id], 0) => Probe::One(id.clone()),
        _ => Probe::Many,
    })
}

pub async fn info<P: Pipe>(c: &mut Client<P>) -> Result<BootInfo, Error> {
    let window = info_window(c.info().bps);
    let f = one(c, window, Cmd::Info, 0, 0, &[]).await?;
    info_from(check(&f, Cmd::Info, 0)?)
}

/// Erase `byte_count` bytes from app offset `addr`, both whole erase pages
/// ([`BootInfo::erase_size`]). The first erase starts an update.
pub async fn erase<P: Pipe>(c: &mut Client<P>, addr: u32, byte_count: u16) -> Result<(), Error> {
    let window = erase_window(c.info().bps, byte_count);
    let f = one(c, window, Cmd::Erase, addr, 0, &byte_count.to_le_bytes()).await?;
    check(&f, Cmd::Erase, addr).map(drop)
}

// The Erase request is its `EraseData`: one u16, little-endian.
const _: () = assert!(size_of::<EraseData>() == 2);

/// Write `data` at app offset `addr`: a region's first write page-aligned,
/// each next one where the last ended. [`WriteFlags::FLUSH`] on a region's
/// last write commits its partial page.
pub async fn write<P: Pipe>(
    c: &mut Client<P>,
    addr: u32,
    data: &[u8],
    flags: WriteFlags,
) -> Result<(), Error> {
    let window = write_window(c.info().bps, data.len(), flags);
    let f = one(c, window, Cmd::Write, addr, flags.bits(), data).await?;
    check(&f, Cmd::Write, addr).map(drop)
}

/// The CRC-16 of the app region's first `app_size` bytes; the device
/// stores it with the size and moves to validating.
pub async fn verify<P: Pipe>(c: &mut Client<P>, app_size: u32) -> Result<u16, Error> {
    let window = verify_window(c.info().bps, app_size);
    let f = one(c, window, Cmd::Verify, app_size, 0, &[]).await?;
    crc_from(check(&f, Cmd::Verify, app_size)?)
}

/// Reset to the bootloader under [`ResetFlags::BOOTLOADER`], else to the
/// app (refused mid-update). `Some(baud)` is stored as the bootloader's
/// rate first; the reply still leaves at the current one.
pub async fn reset<P: Pipe>(
    c: &mut Client<P>,
    flags: ResetFlags,
    baud: Option<BaudRate>,
) -> Result<(), Error> {
    let (flags, addr) = match baud {
        Some(b) => (flags | ResetFlags::BAUD, b as u32),
        None => (flags - ResetFlags::BAUD, 0),
    };
    let window = reset_window(c.info().bps, flags);
    let f = one(c, window, Cmd::Reset, addr, flags.bits(), &[]).await?;
    check(&f, Cmd::Reset, addr).map(drop)
}

async fn one<P: Pipe>(
    c: &mut Client<P>,
    window: Duration,
    cmd: Cmd,
    addr: u32,
    flags: u8,
    data: &[u8],
) -> Result<TbFrame, Error> {
    only(
        c.tb_exchange(TbExpect::One, window, cmd as u8, addr, flags, data)
            .await?,
    )
}

fn only(r: TbReply) -> Result<TbFrame, Error> {
    let outcome = r.outcome;
    if let Outcome::Timeout { slot } = outcome {
        return Err(Error::Timeout { slot });
    }
    r.frames
        .into_iter()
        .next()
        .ok_or_else(|| desync_msg(format!("tinyboot exchange ended {outcome:?} with no reply")))
}

fn selected(r: TbReply) -> Result<Selected, Error> {
    let ids = r
        .frames
        .iter()
        .map(|f| check(f, Cmd::Select, 0).map(<[u8]>::to_vec))
        .collect::<Result<_, _>>()?;
    Ok(Selected {
        ids,
        garble: r.garble,
    })
}

const MALFORMED: Error = Error::Link(LinkError::Malformed);

/// A reply's data, once it echoes the request and answers `Ok`.
fn check(f: &TbFrame, cmd: Cmd, addr: u32) -> Result<&[u8], Error> {
    if f.cmd != cmd as u8 || f.addr != addr {
        return Err(MALFORMED);
    }
    match status(f.status) {
        Some(Status::Ok) => Ok(&f.data),
        Some(s) => Err(Error::Boot(s)),
        None => Err(MALFORMED),
    }
}

fn status(b: u8) -> Option<Status> {
    Some(match b {
        0x00 => Status::Request,
        0x01 => Status::Ok,
        0x02 => Status::WriteError,
        0x03 => Status::CrcMismatch,
        0x04 => Status::AddrOutOfBounds,
        0x05 => Status::Unsupported,
        0x06 => Status::PayloadOverflow,
        0x07 => Status::Incompatible,
        _ => return None,
    })
}

fn info_from(data: &[u8]) -> Result<BootInfo, Error> {
    let i = payload::<InfoData>(data).ok_or(MALFORMED)?;
    Ok(BootInfo {
        capacity: i.capacity,
        erase_size: i.erase_size,
        boot_version: i.boot_version,
        app_version: i.app_version,
        mode: i.mode,
        boot_baud: i.boot_baud,
        compat: i.compat,
        max_payload: i.max_payload,
    })
}

fn crc_from(data: &[u8]) -> Result<u16, Error> {
    payload::<VerifyData>(data).map(|v| v.crc).ok_or(MALFORMED)
}

/// A reply payload as its protocol struct, `None` when shorter. A longer
/// one decodes, so a release that appends fields still reads.
#[allow(unsafe_code)]
fn payload<T: Copy>(data: &[u8]) -> Option<T> {
    if data.len() < size_of::<T>() {
        return None;
    }
    // SAFETY: `data` covers `T` and `read_unaligned` takes any alignment.
    // `T` is only ever a tinyboot-protocol payload struct: padding-free,
    // every bit pattern valid, its memory bytes the wire bytes
    // (tinyboot-protocol refuses big-endian targets).
    Some(unsafe { data.as_ptr().cast::<T>().read_unaligned() })
}

// Reply windows: the reply's wire time at the bus rate plus MARGIN times
// the device's worst case from the request's last byte to the reply's
// first. The device model is the osc-dev-v006 boot build: 48 MHz HCLK,
// code fetched per the V006 fetch model (docs/v006-performance.md sec 1).
// Every received frame has its CRC checked, every reply has one computed
// before its first byte leaves.

const FRAME_BYTES: u64 = 12;
/// The device model is a fetch-model estimate on an untrimmed HSI, the
/// flash times datasheet maxima; a short window loses the reply, a long
/// one costs only time.
const MARGIN: u64 = 2;
/// tinyboot-protocol `crc16`, bitwise: 8 passes of a 4-word loop at 15
/// HCLK plus ~20 HCLK per byte, ~140 HCLK.
const CRC_NS_PER_BYTE: u64 = 3_000;
/// Write's copy into the page buffer: two taken branches per byte, ~30
/// HCLK.
const COPY_NS_PER_BYTE: u64 = 700;
/// V006 page erase and program maxima (protocol sec 9.4).
const ERASE_NS: u64 = 3_000_000;
const PROGRAM_NS: u64 = 2_000_000;
/// A boot META update is an erase and a program (tinyboot-ch32
/// `BootMetaStore::write`); Erase pays it from idle or validating, Verify
/// always, Reset to store a rate.
const META_NS: u64 = ERASE_NS + PROGRAM_NS;
/// V006 erase and write page.
const PAGE: u64 = 256;
/// tinyboot-core `MAX_UID_LEN`: bounds a Select reply.
const MAX_UID: u64 = 16;
/// Select slots run 0..=15 units of `wait_bytes` (tinyboot-ch32
/// hal/usart/usart_common.rs:37): 16 counts per unit, each BRR passes of a
/// one-word loop at 6 HCLK, so 96 bit-times, plus <= 12 HCLK of outer loop
/// per count, 4 us per unit.
const SLOT_MAX: u64 = 15;
const SLOT_BITS: u64 = 96;
const SLOT_PASS_NS: u64 = 4_000;

fn wire_ns(bytes: u64, bps: u32) -> u64 {
    (bytes * 10 * 1_000_000_000).div_ceil(bps.into())
}

fn crc_ns(bytes: u64) -> u64 {
    bytes * CRC_NS_PER_BYTE
}

/// The CRCs of a request carrying `req` data bytes and a reply carrying
/// `rep`, each over its 10-byte header too.
fn frames_ns(req: u64, rep: u64) -> u64 {
    crc_ns(10 + req) + crc_ns(10 + rep)
}

fn window(bps: u32, reply_data: u64, device_ns: u64) -> Duration {
    let ns = wire_ns(FRAME_BYTES + reply_data, bps) + MARGIN * device_ns;
    Duration::from_micros(ns.div_ceil(1_000))
}

fn select_window(bps: u32, prefix_bytes: usize) -> Duration {
    let slot = SLOT_MAX * ((SLOT_BITS * 1_000_000_000).div_ceil(bps.into()) + SLOT_PASS_NS);
    // The slot is drawn from a CRC over the id.
    let device = frames_ns(2 + prefix_bytes as u64, MAX_UID) + crc_ns(MAX_UID) + slot;
    window(bps, MAX_UID, device)
}

fn info_window(bps: u32) -> Duration {
    let rep = size_of::<InfoData>() as u64;
    window(bps, rep, frames_ns(0, rep))
}

fn erase_window(bps: u32, byte_count: u16) -> Duration {
    let pages = u64::from(byte_count).div_ceil(PAGE);
    window(bps, 0, frames_ns(2, 0) + META_NS + pages * ERASE_NS)
}

fn write_window(bps: u32, len: usize, flags: WriteFlags) -> Duration {
    let len = len as u64;
    // A write fills at most one page per PAGE bytes, plus the flush.
    let programs = len / PAGE + 1 + u64::from(flags.contains(WriteFlags::FLUSH));
    let device = frames_ns(len, 0) + len * COPY_NS_PER_BYTE + programs * PROGRAM_NS;
    window(bps, 0, device)
}

fn verify_window(bps: u32, app_size: u32) -> Duration {
    let rep = size_of::<VerifyData>() as u64;
    window(
        bps,
        rep,
        frames_ns(0, rep) + crc_ns(app_size.into()) + META_NS,
    )
}

fn reset_window(bps: u32, flags: ResetFlags) -> Duration {
    let meta = if flags.contains(ResetFlags::BAUD) {
        META_NS
    } else {
        0
    };
    window(bps, 0, frames_ns(0, 0) + meta)
}

#[cfg(test)]
mod tests {
    use super::*;
    use tinyboot_protocol::frame::{DEFAULT_PAYLOAD, Frame};

    /// A reply as a device sends it, reduced to the link body (link sec
    /// 7.1): sync, length and CRC stripped.
    fn device_reply(cmd: Cmd, status: Status, addr: u32, fill: impl FnOnce(&mut Frame)) -> TbFrame {
        let mut f = Frame::<DEFAULT_PAYLOAD>::default();
        f.cmd = cmd;
        f.status = status;
        f.set_addr(addr);
        fill(&mut f);
        let mut buf = [0u8; 128];
        let mut w: &mut [u8] = &mut buf;
        f.send(&mut w).expect("fits");
        let n = 128 - w.len();
        let mut body = buf[2..8].to_vec();
        body.extend_from_slice(&buf[10..n - 2]);
        TbFrame::parse(0, &body).expect("link body")
    }

    fn raw(f: &mut Frame, data: &[u8]) {
        let mut raw = [0u8; DEFAULT_PAYLOAD];
        raw[..data.len()].copy_from_slice(data);
        f.data.raw = raw;
        f.len = data.len() as u16;
    }

    fn reply(frames: Vec<TbFrame>, outcome: Outcome, garble: u16) -> TbReply {
        TbReply {
            frames,
            outcome,
            tick: 0,
            garble,
            trailing: false,
            laps: 0,
        }
    }

    const INFO: InfoData = InfoData {
        capacity: 58_112,
        erase_size: 256,
        boot_version: 0x0A41,
        app_version: 0xFFFF,
        mode: 0,
        boot_baud: 0xFF,
        compat: 0x1234_5678,
        max_payload: 240,
    };

    #[test]
    fn info_decodes_the_device_struct() {
        let f = device_reply(Cmd::Info, Status::Ok, 0, |f| {
            f.data.info = INFO;
            f.len = InfoData::SIZE;
        });
        assert_eq!(
            check(&f, Cmd::Info, 0).and_then(info_from),
            Ok(BootInfo {
                capacity: 58_112,
                erase_size: 256,
                boot_version: 0x0A41,
                app_version: 0xFFFF,
                mode: 0,
                boot_baud: 0xFF,
                compat: 0x1234_5678,
                max_payload: 240,
            })
        );
    }

    #[test]
    fn info_short_payload_is_malformed() {
        let f = device_reply(Cmd::Info, Status::Ok, 0, |f| {
            f.data.info = INFO;
            f.len = InfoData::SIZE - 1;
        });
        assert_eq!(check(&f, Cmd::Info, 0).and_then(info_from), Err(MALFORMED));
    }

    #[test]
    fn verify_decodes_the_crc() {
        let f = device_reply(Cmd::Verify, Status::Ok, 4096, |f| {
            f.data.verify = VerifyData { crc: 0xBEEF };
            f.len = 2;
        });
        assert_eq!(check(&f, Cmd::Verify, 4096).and_then(crc_from), Ok(0xBEEF));
    }

    #[test]
    fn verify_short_payload_is_malformed() {
        let f = device_reply(Cmd::Verify, Status::Ok, 4096, |f| raw(f, &[0xEF]));
        assert_eq!(
            check(&f, Cmd::Verify, 4096).and_then(crc_from),
            Err(MALFORMED)
        );
    }

    #[test]
    fn empty_replies_check_ok() {
        for (cmd, addr) in [(Cmd::Erase, 0x400), (Cmd::Write, 0x500), (Cmd::Reset, 13)] {
            let f = device_reply(cmd, Status::Ok, addr, |_| {});
            assert_eq!(check(&f, cmd, addr), Ok(&[][..]), "{cmd:?}");
        }
    }

    #[test]
    fn non_ok_status_is_a_boot_error() {
        for s in [
            Status::WriteError,
            Status::CrcMismatch,
            Status::AddrOutOfBounds,
            Status::Unsupported,
            Status::PayloadOverflow,
            Status::Incompatible,
        ] {
            let f = device_reply(Cmd::Erase, s, 0x400, |_| {});
            assert_eq!(check(&f, Cmd::Erase, 0x400), Err(Error::Boot(s)));
        }
    }

    #[test]
    fn unknown_status_byte_is_malformed() {
        let mut f = device_reply(Cmd::Erase, Status::Ok, 0x400, |_| {});
        f.status = 0x08;
        assert_eq!(check(&f, Cmd::Erase, 0x400), Err(MALFORMED));
    }

    #[test]
    fn wrong_echo_is_malformed() {
        let f = device_reply(Cmd::Write, Status::Ok, 0x500, |_| {});
        assert_eq!(check(&f, Cmd::Erase, 0x500), Err(MALFORMED));
        assert_eq!(check(&f, Cmd::Write, 0x600), Err(MALFORMED));
    }

    #[test]
    fn expect_one_timeout_is_error_timeout() {
        let r = reply(vec![], Outcome::Timeout { slot: 0 }, 3);
        assert_eq!(only(r), Err(Error::Timeout { slot: 0 }));
    }

    #[test]
    fn select_returns_every_id_and_the_garble() {
        let a = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12];
        let b = [0xC0; 12];
        let frames = [a, b]
            .map(|id| device_reply(Cmd::Select, Status::Ok, 0, |f| raw(f, &id)))
            .to_vec();
        assert_eq!(
            selected(reply(frames, Outcome::Complete, 7)),
            Ok(Selected {
                ids: vec![a.to_vec(), b.to_vec()],
                garble: 7,
            })
        );
    }

    #[test]
    fn select_reply_with_wrong_echo_is_malformed() {
        let f = device_reply(Cmd::Info, Status::Ok, 0, |f| raw(f, &[1; 12]));
        assert_eq!(
            selected(reply(vec![f], Outcome::Complete, 0)),
            Err(MALFORMED)
        );
    }

    const K500: u32 = 500_000;
    const M3: u32 = 3_000_000;

    fn us(us: u64) -> Duration {
        Duration::from_micros(us)
    }

    // Device time by hand: CRCs at 3 us per byte over header + data each
    // way, then the command's own work; doubled; plus 10 bits per reply
    // byte, 20 us at 500k and 3.33 us at 3M.

    #[test]
    fn info_window_by_hand() {
        // CRC (10 + 0) + (10 + 18) = 114 us; reply 30 B.
        assert_eq!(info_window(K500), us(228 + 600));
        assert_eq!(info_window(M3), us(228 + 100));
    }

    #[test]
    fn select_window_by_hand() {
        // A full 12-byte prefix: CRC (10 + 14) + (10 + 16) + id hash 16 =
        // 198 us; 15 slot units of 96 bit-times + 4 us; reply 28 B.
        // 500k: 198 + 15 x 196 = 3138 us.
        assert_eq!(select_window(K500, 12), us(2 * 3138 + 560));
        // 3M: 198 + 15 x 36 = 738 us; reply 93.33 us, rounded up.
        assert_eq!(select_window(M3, 12), us(2 * 738 + 94));
    }

    #[test]
    fn erase_window_by_hand() {
        // One page: CRC (10 + 2) + 10 = 66 us, META 5 ms, page erase 3 ms.
        assert_eq!(erase_window(K500, 256), us(2 * 8066 + 240));
        assert_eq!(erase_window(M3, 256), us(2 * 8066 + 40));
    }

    #[test]
    fn write_window_by_hand() {
        // 240 B: CRC (10 + 240) + 10 = 780 us, copy 168 us, then one
        // program, or two with the flush.
        let flush = 780 + 168 + 4000;
        let plain = 780 + 168 + 2000;
        assert_eq!(
            write_window(K500, 240, WriteFlags::FLUSH),
            us(2 * flush + 240)
        );
        assert_eq!(write_window(M3, 240, WriteFlags::FLUSH), us(2 * flush + 40));
        assert_eq!(
            write_window(K500, 240, WriteFlags::empty()),
            us(2 * plain + 240)
        );
        assert_eq!(
            write_window(M3, 240, WriteFlags::empty()),
            us(2 * plain + 40)
        );
    }

    #[test]
    fn verify_window_by_hand() {
        // 4096 B: CRC (10 + 0) + (10 + 2) = 66 us, app CRC 12288 us,
        // META 5 ms; reply 14 B, 46.67 us at 3M rounded up.
        let device = 66 + 12_288 + 5000;
        assert_eq!(verify_window(K500, 4096), us(2 * device + 280));
        assert_eq!(verify_window(M3, 4096), us(2 * device + 47));
    }

    #[test]
    fn reset_window_by_hand() {
        // CRC 10 + 10 = 60 us, plus META 5 ms to store a rate.
        assert_eq!(reset_window(K500, ResetFlags::BOOTLOADER), us(2 * 60 + 240));
        assert_eq!(reset_window(M3, ResetFlags::BOOTLOADER), us(2 * 60 + 40));
        let baud = ResetFlags::BOOTLOADER | ResetFlags::BAUD;
        assert_eq!(reset_window(K500, baud), us(2 * 5060 + 240));
        assert_eq!(reset_window(M3, baud), us(2 * 5060 + 40));
    }
}

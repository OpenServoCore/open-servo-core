//! Request-level spec tests for the single-shot [`Dispatcher`]. Each case
//! drives one decoded [`Request`] with a recording [`FakeReply`] and asserts on
//! the decoded reply shape (result / alert / data) plus any staged side effect.

use heapless::Vec;

use osc_protocol::FrameBytes;
use osc_protocol::table::STATUS_FLAG_CONFIG_DIRTY;
use osc_protocol::wire::{MgmtOp, ResultCode};

use crate::regions::CONTROL_BASE_ADDR;
use crate::regions::config::BaudRate;
use crate::services::bus::Dispatcher;
use crate::traits::{Dispatch, Dispatched, Reply, Request, RequestCtx, SendError, Status};
use crate::{BootMode, RegionStorage, Shared, StagedWrites};

/// One recorded `send_status` call.
struct Sent {
    result: ResultCode,
    alert: bool,
    data: Vec<u8, 256>,
}

struct FakeReply {
    sends: Vec<Sent, 8>,
    /// Span count of the last `send_status_gather` (0 = none seen).
    gather_spans: usize,
    staged_id: Option<u8>,
    /// `set_id` value, plus the send count at that instant -- ASSIGN must
    /// apply the id BEFORE its ack so the ack leaves from the new id (sec 9.2).
    set_id: Option<(u8, usize)>,
    staged_baud: Option<BaudRate>,
    response_deadline: Option<u16>,
    reboot: Option<BootMode>,
    clock_cal: Option<(u16, u8)>,
}

impl FakeReply {
    fn new() -> Self {
        Self {
            sends: Vec::new(),
            gather_spans: 0,
            staged_id: None,
            set_id: None,
            staged_baud: None,
            response_deadline: None,
            reboot: None,
            clock_cal: None,
        }
    }

    fn count(&self) -> usize {
        self.sends.len()
    }

    fn last(&self) -> &Sent {
        self.sends.last().expect("a status was sent")
    }
}

impl Reply for FakeReply {
    fn send_status(&mut self, status: Status<'_>) -> Result<(), SendError> {
        let mut data = Vec::new();
        data.extend_from_slice(status.data)
            .map_err(|_| SendError::Overflow)?;
        self.sends
            .push(Sent {
                result: status.result,
                alert: status.alert,
                data,
            })
            .map_err(|_| SendError::Overflow)?;
        Ok(())
    }

    fn send_status_gather(
        &mut self,
        result: ResultCode,
        alert: bool,
        spans: &[&[u8]],
    ) -> Result<(), SendError> {
        let mut data = Vec::new();
        for s in spans {
            data.extend_from_slice(s).map_err(|_| SendError::Overflow)?;
        }
        self.gather_spans = spans.len();
        self.sends
            .push(Sent {
                result,
                alert,
                data,
            })
            .map_err(|_| SendError::Overflow)?;
        Ok(())
    }

    fn stage_id(&mut self, id: u8) {
        self.staged_id = Some(id);
    }

    fn set_id(&mut self, id: u8) {
        self.set_id = Some((id, self.sends.len()));
    }

    fn stage_baud(&mut self, baud: BaudRate) {
        self.staged_baud = Some(baud);
    }

    fn set_response_deadline(&mut self, us: u16) {
        self.response_deadline = Some(us);
    }

    fn stage_reboot(&mut self, mode: BootMode) {
        self.reboot = Some(mode);
    }

    fn begin_clock_cal(&mut self, gap_us: u16, gaps: u8) {
        self.clock_cal = Some((gap_us, gaps));
    }
}

/// Dispatch one request and deliver a PASS verdict -- the common whole-exchange
/// shape (dispatch stages effects; the verdict commit promotes a staged table
/// effect, exactly as the bus does after a clean CRC).
fn go(shared: &Shared, staged: &mut StagedWrites, req: Request<'_>, may_reply: bool) -> FakeReply {
    let mut reply = FakeReply::new();
    let mut pending = None;
    let mut d = Dispatcher::new(shared, staged, &mut pending);
    if matches!(
        d.dispatch(req, RequestCtx { may_reply }, &mut reply),
        Dispatched::Pending
    ) {
        d.commit(&mut reply);
    }
    reply
}

fn enable_torque(shared: &Shared) {
    shared
        .table
        .with_mut(|t| t.control.lifecycle.torque_enable = true);
}

// --- Ping -----------------------------------------------------------------

#[test]
fn ping_replies_model_and_firmware() {
    let shared = Shared::new();
    shared.table.with_mut(|t| {
        t.config.common.model_number = 0x1234;
        t.config.common.firmware_version = 0x56;
    });
    let mut staged = StagedWrites::new();
    let reply = go(&shared, &mut staged, Request::Ping, true);
    assert_eq!(reply.count(), 1);
    assert_eq!(reply.last().result, ResultCode::Ok);
    assert_eq!(&reply.last().data[..], &[0x34, 0x12, 0x56]);
}

#[test]
fn ping_silent_when_may_reply_false() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let reply = go(&shared, &mut staged, Request::Ping, false);
    assert_eq!(reply.count(), 0);
}

// --- Read -----------------------------------------------------------------

#[test]
fn read_returns_table_slice() {
    let shared = Shared::new();
    shared
        .table
        .with_mut(|t| t.config.common.model_number = 0xABCD);
    let mut staged = StagedWrites::new();
    let reply = go(
        &shared,
        &mut staged,
        Request::Read { addr: 0, count: 2 },
        true,
    );
    assert_eq!(reply.last().result, ResultCode::Ok);
    assert_eq!(&reply.last().data[..], &[0xCD, 0xAB]);
}

#[test]
fn read_count_zero_rejects_range() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let reply = go(
        &shared,
        &mut staged,
        Request::Read { addr: 0, count: 0 },
        true,
    );
    assert_eq!(reply.last().result, ResultCode::Range);
    assert!(reply.last().data.is_empty());
}

#[test]
fn read_odd_addr_serves_bytes() {
    // sec 5: any read address is legal -- odd-addressed spans stage through the
    // chip CRC provider's copy path (sec 3.2); dispatch is parity-blind.
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let reply = go(
        &shared,
        &mut staged,
        Request::Read { addr: 1, count: 2 },
        true,
    );
    assert_eq!(reply.last().result, ResultCode::Ok);
    assert_eq!(reply.last().data.len(), 2);
}

#[test]
fn read_over_ceiling_rejects_limit() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let reply = go(
        &shared,
        &mut staged,
        Request::Read {
            addr: 0,
            count: 253,
        },
        true,
    );
    assert_eq!(reply.last().result, ResultCode::Limit);
}

#[test]
fn read_out_of_bounds_rejects_range() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let reply = go(
        &shared,
        &mut staged,
        Request::Read {
            addr: 0xFFFE,
            count: 1,
        },
        true,
    );
    assert_eq!(reply.last().result, ResultCode::Range);
}

#[test]
fn read_silent_when_may_reply_false() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let reply = go(
        &shared,
        &mut staged,
        Request::Read { addr: 0, count: 2 },
        false,
    );
    assert_eq!(reply.count(), 0);
}

// --- Profile read (sec 5.2) ---------------------------------------------------

use crate::regions::profile::span_word;

/// Slot 0 -> two spans: model_number (2 B at 0x000) and comms id (1 B).
fn seed_profile_slot0(shared: &Shared) {
    use crate::regions::config::addr::common::ID;
    shared.table.with_mut(|t| {
        t.config.common.model_number = 0xABCD;
        t.config.common.id = 7;
        t.profile.slots.words[0] = span_word(0, 2);
        t.profile.slots.words[1] = span_word(ID, 1);
    });
}

#[test]
fn profile_read_gathers_spans_in_order() {
    let shared = Shared::new();
    seed_profile_slot0(&shared);
    let mut staged = StagedWrites::new();
    let reply = go(&shared, &mut staged, Request::ReadProfile { slot: 0 }, true);
    assert_eq!(reply.last().result, ResultCode::Ok);
    assert_eq!(&reply.last().data[..], &[0xCD, 0xAB, 7]);
    assert_eq!(reply.gather_spans, 2);
}

#[test]
fn profile_read_skips_disabled_words() {
    let shared = Shared::new();
    shared.table.with_mut(|t| {
        t.config.common.model_number = 0xABCD;
        // Hole at word 0 and word 2: disabled words are skipped, not
        // terminators (sec 5.2).
        t.profile.slots.words[1] = span_word(0, 1);
        t.profile.slots.words[3] = span_word(1, 1);
    });
    let mut staged = StagedWrites::new();
    let reply = go(&shared, &mut staged, Request::ReadProfile { slot: 0 }, true);
    assert_eq!(reply.last().result, ResultCode::Ok);
    assert_eq!(&reply.last().data[..], &[0xCD, 0xAB]);
    assert_eq!(reply.gather_spans, 2);
}

#[test]
fn profile_read_bad_slot_rejects_range() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let reply = go(&shared, &mut staged, Request::ReadProfile { slot: 4 }, true);
    assert_eq!(reply.last().result, ResultCode::Range);
    assert!(reply.last().data.is_empty());
}

#[test]
fn profile_read_empty_slot_rejects_range() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let reply = go(&shared, &mut staged, Request::ReadProfile { slot: 1 }, true);
    assert_eq!(reply.last().result, ResultCode::Range);
}

#[test]
fn profile_read_oob_span_rejects_range() {
    let shared = Shared::new();
    // addr 1020 + count 8 overruns the 1024 B table.
    shared
        .table
        .with_mut(|t| t.profile.slots.words[0] = span_word(1020, 8));
    let mut staged = StagedWrites::new();
    let reply = go(&shared, &mut staged, Request::ReadProfile { slot: 0 }, true);
    assert_eq!(reply.last().result, ResultCode::Range);
}

#[test]
fn profile_read_over_ceiling_rejects_limit() {
    let shared = Shared::new();
    // 5 x 63 B = 315 B > MAX_PAYLOAD.
    shared.table.with_mut(|t| {
        for w in 0..5 {
            t.profile.slots.words[w] = span_word(0, 63);
        }
    });
    let mut staged = StagedWrites::new();
    let reply = go(&shared, &mut staged, Request::ReadProfile { slot: 0 }, true);
    assert_eq!(reply.last().result, ResultCode::Limit);
}

#[test]
fn profile_read_silent_when_may_reply_false() {
    let shared = Shared::new();
    seed_profile_slot0(&shared);
    let mut staged = StagedWrites::new();
    let reply = go(
        &shared,
        &mut staged,
        Request::ReadProfile { slot: 0 },
        false,
    );
    assert_eq!(reply.count(), 0);
}

// --- Write ----------------------------------------------------------------

#[test]
fn write_acks_ok_and_mutates() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let reply = go(
        &shared,
        &mut staged,
        Request::Write {
            addr: CONTROL_BASE_ADDR,
            data: FrameBytes::from(&[1][..]),
            hold: false,
        },
        true,
    );
    assert_eq!(reply.last().result, ResultCode::Ok);
    assert!(reply.last().data.is_empty());
    assert!(shared.table.with(|t| t.control.lifecycle.torque_enable));
}

#[test]
fn write_validation_reject_maps_to_validation() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    // 2 is outside a bool field's allowed {0, 1} -- a field-rule rejection.
    let reply = go(
        &shared,
        &mut staged,
        Request::Write {
            addr: CONTROL_BASE_ADDR,
            data: FrameBytes::from(&[2][..]),
            hold: false,
        },
        true,
    );
    assert_eq!(reply.last().result, ResultCode::Validation);
}

/// osc separates write from persistence (sec 9.4): config writes are never
/// torque-gated -- only MGMT SAVE is.
#[test]
fn write_config_allowed_with_torque_on() {
    use crate::regions::config::addr::common::ID;
    let shared = Shared::new();
    enable_torque(&shared);
    let mut staged = StagedWrites::new();
    let reply = go(
        &shared,
        &mut staged,
        Request::Write {
            addr: ID,
            data: FrameBytes::from(&[5][..]),
            hold: false,
        },
        true,
    );
    assert_eq!(reply.last().result, ResultCode::Ok);
    assert_eq!(shared.table.with(|t| t.config.common.id), 5);
    assert_eq!(reply.staged_id, Some(5));
}

#[test]
fn write_applies_even_when_may_reply_false() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let reply = go(
        &shared,
        &mut staged,
        Request::Write {
            addr: CONTROL_BASE_ADDR,
            data: FrameBytes::from(&[1][..]),
            hold: false,
        },
        false,
    );
    assert_eq!(reply.count(), 0);
    assert!(shared.table.with(|t| t.control.lifecycle.torque_enable));
}

#[test]
fn write_id_stages_via_reply() {
    use crate::regions::config::addr::common::ID;
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let reply = go(
        &shared,
        &mut staged,
        Request::Write {
            addr: ID,
            data: FrameBytes::from(&[42][..]),
            hold: false,
        },
        true,
    );
    assert_eq!(reply.last().result, ResultCode::Ok);
    assert_eq!(reply.staged_id, Some(42));
}

#[test]
fn write_baud_stages_via_reply() {
    use crate::regions::config::addr::common::BAUD_RATE_IDX;
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let reply = go(
        &shared,
        &mut staged,
        Request::Write {
            addr: BAUD_RATE_IDX,
            data: FrameBytes::from(&[BaudRate::B2000000 as u8][..]),
            hold: false,
        },
        true,
    );
    assert_eq!(reply.last().result, ResultCode::Ok);
    assert_eq!(reply.staged_baud, Some(BaudRate::B2000000));
}

#[test]
fn write_response_deadline_sets_via_reply() {
    use crate::regions::config::addr::common::RESPONSE_DEADLINE_US;
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let reply = go(
        &shared,
        &mut staged,
        Request::Write {
            addr: RESPONSE_DEADLINE_US,
            data: FrameBytes::from(&200u16.to_le_bytes()[..]),
            hold: false,
        },
        true,
    );
    assert_eq!(reply.last().result, ResultCode::Ok);
    assert_eq!(reply.response_deadline, Some(200));
}

// --- Hold write + commit --------------------------------------------------

#[test]
fn hold_write_stages_without_touching_live_table() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let reply = go(
        &shared,
        &mut staged,
        Request::Write {
            addr: CONTROL_BASE_ADDR,
            data: FrameBytes::from(&[1][..]),
            hold: true,
        },
        true,
    );
    assert_eq!(reply.last().result, ResultCode::Ok);
    assert!(!shared.table.with(|t| t.control.lifecycle.torque_enable));

    let reply = go(&shared, &mut staged, Request::Commit, true);
    assert_eq!(reply.last().result, ResultCode::Ok);
    assert!(shared.table.with(|t| t.control.lifecycle.torque_enable));
}

#[test]
fn commit_silent_when_may_reply_false() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    go(
        &shared,
        &mut staged,
        Request::Write {
            addr: CONTROL_BASE_ADDR,
            data: FrameBytes::from(&[1][..]),
            hold: true,
        },
        false,
    );
    let reply = go(&shared, &mut staged, Request::Commit, false);
    assert_eq!(reply.count(), 0);
    assert!(shared.table.with(|t| t.control.lifecycle.torque_enable));
}

// --- Mgmt -----------------------------------------------------------------

#[test]
fn mgmt_reboot_acks_and_stages_boot_mode() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let reply = go(
        &shared,
        &mut staged,
        Request::Mgmt {
            op: MgmtOp::Reboot,
            args: FrameBytes::from(&[][..]),
        },
        true,
    );
    assert_eq!(reply.last().result, ResultCode::Ok);
    assert!(reply.last().data.is_empty());
    assert_eq!(reply.reboot, Some(BootMode::App));
}

#[test]
fn mgmt_reboot_stages_without_ack_when_silent() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let reply = go(
        &shared,
        &mut staged,
        Request::Mgmt {
            op: MgmtOp::Reboot,
            args: FrameBytes::from(&[][..]),
        },
        false,
    );
    assert_eq!(reply.count(), 0);
    assert_eq!(reply.reboot, Some(BootMode::App));
}

#[test]
fn mgmt_wrapped_enum_assign_reply_instruction() {
    // ENUM/ASSIGN decode to dedicated Request variants; the Mgmt-wrapped
    // form is a bus bug, answered like any unknown op.
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    for op in [MgmtOp::Enum, MgmtOp::Assign] {
        let reply = go(
            &shared,
            &mut staged,
            Request::Mgmt {
                op,
                args: FrameBytes::from(&[][..]),
            },
            true,
        );
        assert_eq!(reply.last().result, ResultCode::Instruction);
    }
}

// --- Save / Factory (sec 9.4/sec 9.5) ---------------------------------------------

use core::sync::atomic::{AtomicBool, AtomicU32, AtomicUsize, Ordering};

use crate::persist::{CONFIG_LEN, ConfigStore, PROFILE_LEN, StoreError};

/// Atomics-only recording store (these tests are no_std): counts calls,
/// fingerprints the saved bytes by CRC, arms failure via `fail`.
struct FakeStore {
    saves: AtomicUsize,
    wipes: AtomicUsize,
    fail: AtomicBool,
    saved_crc: AtomicU32,
}

impl FakeStore {
    const fn new() -> Self {
        Self {
            saves: AtomicUsize::new(0),
            wipes: AtomicUsize::new(0),
            fail: AtomicBool::new(false),
            saved_crc: AtomicU32::new(0),
        }
    }
}

impl ConfigStore for FakeStore {
    fn save(
        &self,
        config: &[u8; CONFIG_LEN],
        profile: &[u8; PROFILE_LEN],
    ) -> Result<(), StoreError> {
        if self.fail.load(Ordering::Relaxed) {
            return Err(StoreError);
        }
        let crc = osc_protocol::crc::osc_crc_continue(osc_protocol::crc::osc_crc(config), profile);
        self.saved_crc.store(crc as u32, Ordering::Relaxed);
        self.saves.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }

    fn wipe(&self) -> Result<(), StoreError> {
        if self.fail.load(Ordering::Relaxed) {
            return Err(StoreError);
        }
        self.wipes.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }
}

fn mgmt_req(op: MgmtOp) -> Request<'static> {
    Request::Mgmt {
        op,
        args: FrameBytes::from(&[][..]),
    }
}

fn dirty(shared: &Shared) -> u8 {
    shared
        .table
        .with(|t| t.telemetry.common.status_flags & STATUS_FLAG_CONFIG_DIRTY)
}

/// CRC fingerprint of the live persisted regions, mirroring FakeStore::save.
fn table_crc(shared: &Shared) -> u32 {
    use crate::regions::{
        CONFIG_BASE_ADDR, CONFIG_REGION_SIZE, PROFILE_BASE_ADDR, PROFILE_REGION_SIZE,
    };
    use control_table::RegisterFile;
    let config = RegisterFile::read(&shared.table, CONFIG_BASE_ADDR, CONFIG_REGION_SIZE).unwrap();
    let profile =
        RegisterFile::read(&shared.table, PROFILE_BASE_ADDR, PROFILE_REGION_SIZE).unwrap();
    osc_protocol::crc::osc_crc_continue(osc_protocol::crc::osc_crc(config), profile) as u32
}

fn write_at(shared: &Shared, staged: &mut StagedWrites, addr: u16, data: &[u8]) {
    let reply = go(
        shared,
        staged,
        Request::Write {
            addr,
            data: FrameBytes::from(data),
            hold: false,
        },
        true,
    );
    assert_eq!(reply.last().result, ResultCode::Ok);
}

#[test]
fn save_streams_the_table_and_acks_after() {
    use crate::regions::config::addr::common::RESPONSE_DEADLINE_US;
    static STORE: FakeStore = FakeStore::new();
    let shared = Shared::new();
    shared.seed_store(&STORE);
    let mut staged = StagedWrites::new();
    write_at(
        &shared,
        &mut staged,
        RESPONSE_DEADLINE_US,
        &200u16.to_le_bytes(),
    );
    assert_eq!(dirty(&shared), 1, "config write marks modified-since-save");

    let reply = go(&shared, &mut staged, mgmt_req(MgmtOp::Save), true);
    assert_eq!(reply.last().result, ResultCode::Ok);
    assert!(reply.last().data.is_empty());
    assert_eq!(STORE.saves.load(Ordering::Relaxed), 1);
    assert_eq!(
        STORE.saved_crc.load(Ordering::Relaxed),
        table_crc(&shared),
        "the saved bytes are the live table's persisted regions"
    );
    assert_eq!(dirty(&shared), 0, "a successful SAVE clears the dirty bit");
}

#[test]
fn save_with_torque_on_nacks_access() {
    static STORE: FakeStore = FakeStore::new();
    let shared = Shared::new();
    shared.seed_store(&STORE);
    enable_torque(&shared);
    let mut staged = StagedWrites::new();
    let reply = go(&shared, &mut staged, mgmt_req(MgmtOp::Save), true);
    assert_eq!(reply.last().result, ResultCode::Access);
    assert_eq!(STORE.saves.load(Ordering::Relaxed), 0);
}

#[test]
fn save_store_failure_nacks_hardware_and_stays_dirty() {
    use crate::regions::config::addr::common::RESPONSE_DEADLINE_US;
    static STORE: FakeStore = FakeStore::new();
    STORE.fail.store(true, Ordering::Relaxed);
    let shared = Shared::new();
    shared.seed_store(&STORE);
    let mut staged = StagedWrites::new();
    write_at(
        &shared,
        &mut staged,
        RESPONSE_DEADLINE_US,
        &200u16.to_le_bytes(),
    );
    let reply = go(&shared, &mut staged, mgmt_req(MgmtOp::Save), true);
    assert_eq!(reply.last().result, ResultCode::Hardware);
    assert_eq!(dirty(&shared), 1);
}

#[test]
fn save_without_store_nacks_hardware() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let reply = go(&shared, &mut staged, mgmt_req(MgmtOp::Save), true);
    assert_eq!(reply.last().result, ResultCode::Hardware);
}

#[test]
fn noreply_save_still_saves() {
    static STORE: FakeStore = FakeStore::new();
    let shared = Shared::new();
    shared.seed_store(&STORE);
    let mut staged = StagedWrites::new();
    let reply = go(&shared, &mut staged, mgmt_req(MgmtOp::Save), false);
    assert_eq!(reply.count(), 0);
    assert_eq!(STORE.saves.load(Ordering::Relaxed), 1);
}

#[test]
fn factory_wipes_acks_and_stages_reboot() {
    static STORE: FakeStore = FakeStore::new();
    let shared = Shared::new();
    shared.seed_store(&STORE);
    let mut staged = StagedWrites::new();
    let reply = go(&shared, &mut staged, mgmt_req(MgmtOp::Factory), true);
    assert_eq!(reply.last().result, ResultCode::Ok);
    assert_eq!(STORE.wipes.load(Ordering::Relaxed), 1);
    assert_eq!(reply.reboot, Some(BootMode::App));
}

#[test]
fn factory_with_torque_on_nacks_access() {
    static STORE: FakeStore = FakeStore::new();
    let shared = Shared::new();
    shared.seed_store(&STORE);
    enable_torque(&shared);
    let mut staged = StagedWrites::new();
    let reply = go(&shared, &mut staged, mgmt_req(MgmtOp::Factory), true);
    assert_eq!(reply.last().result, ResultCode::Access);
    assert_eq!(STORE.wipes.load(Ordering::Relaxed), 0);
    assert_eq!(reply.reboot, None);
}

#[test]
fn factory_wipe_failure_nacks_hardware_without_reboot() {
    static STORE: FakeStore = FakeStore::new();
    STORE.fail.store(true, Ordering::Relaxed);
    let shared = Shared::new();
    shared.seed_store(&STORE);
    let mut staged = StagedWrites::new();
    let reply = go(&shared, &mut staged, mgmt_req(MgmtOp::Factory), true);
    assert_eq!(reply.last().result, ResultCode::Hardware);
    assert_eq!(reply.reboot, None);
}

#[test]
fn dirty_bit_tracks_persistent_regions_only() {
    use crate::regions::PROFILE_BASE_ADDR;
    use crate::regions::config::addr::common::RESPONSE_DEADLINE_US;
    static STORE: FakeStore = FakeStore::new();
    let shared = Shared::new();
    shared.seed_store(&STORE);
    let mut staged = StagedWrites::new();
    assert_eq!(dirty(&shared), 0, "boot state is clean");

    write_at(&shared, &mut staged, CONTROL_BASE_ADDR, &[1]);
    assert_eq!(dirty(&shared), 0, "volatile CONTROL writes don't mark");
    shared
        .table
        .with_mut(|t| t.control.lifecycle.torque_enable = false);

    write_at(
        &shared,
        &mut staged,
        RESPONSE_DEADLINE_US,
        &200u16.to_le_bytes(),
    );
    assert_eq!(dirty(&shared), 1);
    go(&shared, &mut staged, mgmt_req(MgmtOp::Save), true);
    assert_eq!(dirty(&shared), 0);

    write_at(&shared, &mut staged, PROFILE_BASE_ADDR, &[0x41, 0x00]);
    assert_eq!(dirty(&shared), 1, "PROFILE writes mark too");
}

#[test]
fn held_write_marks_dirty_at_commit_not_before() {
    use crate::regions::config::addr::common::RESPONSE_DEADLINE_US;
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let reply = go(
        &shared,
        &mut staged,
        Request::Write {
            addr: RESPONSE_DEADLINE_US,
            data: FrameBytes::from(&200u16.to_le_bytes()[..]),
            hold: true,
        },
        true,
    );
    assert_eq!(reply.last().result, ResultCode::Ok);
    assert_eq!(dirty(&shared), 0, "held entries aren't applied yet");
    go(&shared, &mut staged, Request::Commit, true);
    assert_eq!(dirty(&shared), 1);
}

// --- Enumerate / Assign (sec 9.2) ----------------------------------------------

const UID: [u8; 16] = [0xA5, 0x5A, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 0xF0];

fn shared_with_uid() -> Shared {
    let shared = Shared::new();
    shared.seed_uid(UID);
    shared
}

fn prefix_of(bits: u8) -> [u8; 16] {
    // A prefix that matches UID for any length: UID itself, masked by parse
    // semantics (the dispatcher only compares the first `bits` bits).
    let mut p = UID;
    if bits < 128 {
        let full = (bits / 8) as usize;
        for b in p.iter_mut().skip(full + 1) {
            *b = 0;
        }
        p[full] &= (1u8 << (bits % 8)).wrapping_sub(1);
    }
    p
}

#[test]
fn enumerate_matching_prefix_replies_full_uid() {
    let shared = shared_with_uid();
    let mut staged = StagedWrites::new();
    for bits in [0u8, 1, 8, 127, 128] {
        let reply = go(
            &shared,
            &mut staged,
            Request::Enumerate {
                prefix_len: bits,
                prefix: prefix_of(bits),
            },
            true,
        );
        assert_eq!(reply.last().result, ResultCode::Ok, "prefix_len {bits}");
        assert_eq!(reply.last().data, UID, "prefix_len {bits}");
    }
}

#[test]
fn enumerate_mismatch_is_silent() {
    let shared = shared_with_uid();
    let mut staged = StagedWrites::new();
    let mut prefix = prefix_of(1);
    prefix[0] ^= 1; // flip the queried bit
    let reply = go(
        &shared,
        &mut staged,
        Request::Enumerate {
            prefix_len: 1,
            prefix,
        },
        true,
    );
    assert_eq!(reply.count(), 0, "a mismatch draws no reply, not a nack");
}

#[test]
fn assign_matching_uid_sets_id_before_ack() {
    let shared = shared_with_uid();
    let mut staged = StagedWrites::new();
    let reply = go(
        &shared,
        &mut staged,
        Request::Assign {
            uid: UID,
            new_id: 42,
        },
        true,
    );
    assert_eq!(reply.last().result, ResultCode::Ok);
    // The id applies immediately and BEFORE the ack stages, so the ack
    // already leaves from the new id (sec 9.2).
    assert_eq!(reply.set_id, Some((42, 0)));
    assert_eq!(reply.staged_id, None, "immediate, not deferred");
    // Mirrored into the config table so a later SAVE persists it -- which
    // also marks modified-since-save (sec 9.4).
    assert_eq!(shared.table.with(|t| t.config.common.id), 42);
    assert_eq!(dirty(&shared), 1);
}

#[test]
fn assign_foreign_uid_is_silent_and_inert() {
    let shared = shared_with_uid();
    let mut staged = StagedWrites::new();
    let before = shared.table.with(|t| t.config.common.id);
    let mut uid = UID;
    uid[15] ^= 0x80;
    let reply = go(
        &shared,
        &mut staged,
        Request::Assign { uid, new_id: 42 },
        true,
    );
    assert_eq!(reply.count(), 0);
    assert_eq!(reply.set_id, None);
    assert_eq!(shared.table.with(|t| t.config.common.id), before);
}

#[test]
fn assign_invalid_id_nacks_validation() {
    let shared = shared_with_uid();
    let mut staged = StagedWrites::new();
    for bad in [0u8, 250, 254, 255] {
        let reply = go(
            &shared,
            &mut staged,
            Request::Assign {
                uid: UID,
                new_id: bad,
            },
            true,
        );
        assert_eq!(reply.last().result, ResultCode::Validation, "id {bad}");
        assert_eq!(reply.set_id, None, "id {bad}");
    }
}

#[test]
fn assign_noreply_still_applies() {
    let shared = shared_with_uid();
    let mut staged = StagedWrites::new();
    let reply = go(
        &shared,
        &mut staged,
        Request::Assign {
            uid: UID,
            new_id: 9,
        },
        false,
    );
    assert_eq!(reply.count(), 0);
    assert_eq!(reply.set_id, Some((9, 0)));
    assert_eq!(shared.table.with(|t| t.config.common.id), 9);
}

// --- Unsupported ----------------------------------------------------------

#[test]
fn unsupported_replies_instruction() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let reply = go(&shared, &mut staged, Request::Unsupported, true);
    assert_eq!(reply.last().result, ResultCode::Instruction);
}

#[test]
fn unsupported_silent_when_may_reply_false() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let reply = go(&shared, &mut staged, Request::Unsupported, false);
    assert_eq!(reply.count(), 0);
}

// --- Alert bit ------------------------------------------------------------

#[test]
fn alert_bit_set_when_fault_flags_nonzero() {
    let shared = Shared::new();
    shared
        .table
        .with_mut(|t| t.telemetry.common.fault_flags = 0x04);
    let mut staged = StagedWrites::new();
    let reply = go(&shared, &mut staged, Request::Ping, true);
    assert!(reply.last().alert);
}

#[test]
fn alert_bit_clear_when_no_fault() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let reply = go(&shared, &mut staged, Request::Ping, true);
    assert!(!reply.last().alert);
}

// --- The verdict gate (dispatch stages; commit/revert resolve) -------------

use crate::regions::control::addr::lifecycle::GOAL_VELOCITY;

fn write(addr: u16, data: &[u8], hold: bool) -> Request<'_> {
    Request::Write {
        addr,
        data: FrameBytes::from(data),
        hold,
    }
}

/// A fresh dispatcher over the same session-owned `staged` + `pending` --
/// mirrors the bus rebuilding one per wake.
fn disp<'a>(
    shared: &'a Shared,
    staged: &'a mut StagedWrites,
    pending: &'a mut Option<crate::services::bus::PendingWrite>,
) -> Dispatcher<'a> {
    Dispatcher::new(shared, staged, pending)
}

#[test]
fn write_stages_then_commit_applies() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let mut pending = None;
    let mut reply = FakeReply::new();

    let out = disp(&shared, &mut staged, &mut pending).dispatch(
        write(CONTROL_BASE_ADDR, &[1], false),
        RequestCtx { may_reply: true },
        &mut reply,
    );
    assert!(matches!(out, Dispatched::Pending));
    assert_eq!(reply.last().result, ResultCode::Ok);
    // Staged only -- the live table is untouched until the verdict commits.
    assert!(!shared.table.with(|t| t.control.lifecycle.torque_enable));

    disp(&shared, &mut staged, &mut pending).commit(&mut reply);
    assert!(shared.table.with(|t| t.control.lifecycle.torque_enable));
    assert!(pending.is_none());
    assert!(
        staged.is_empty(),
        "a plain write clears its staging on commit"
    );
}

#[test]
fn write_reverts_on_fail_verdict() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let mut pending = None;
    let mut reply = FakeReply::new();

    let out = disp(&shared, &mut staged, &mut pending).dispatch(
        write(CONTROL_BASE_ADDR, &[1], false),
        RequestCtx { may_reply: true },
        &mut reply,
    );
    assert!(matches!(out, Dispatched::Pending));

    // CRC failed: revert -- the table and staging are byte-identical to before.
    disp(&shared, &mut staged, &mut pending).revert();
    assert!(pending.is_none());
    assert!(staged.is_empty());
    assert!(!shared.table.with(|t| t.control.lifecycle.torque_enable));
}

#[test]
fn write_validation_reject_nacks_nothing_staged() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let mut pending = None;
    let mut reply = FakeReply::new();

    // 2 is outside a bool field's allowed {0, 1}: validation reject.
    let out = disp(&shared, &mut staged, &mut pending).dispatch(
        write(CONTROL_BASE_ADDR, &[2], false),
        RequestCtx { may_reply: true },
        &mut reply,
    );
    assert!(matches!(out, Dispatched::Done), "a reject owes no verdict");
    assert_eq!(reply.last().result, ResultCode::Validation);
    assert!(staged.is_empty());
    assert!(pending.is_none());
}

#[test]
fn write_staging_full_nacks_busy_nothing_staged() {
    use control_table::STAGE_DATA_CAP;
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    // Fill the staging data buffer to capacity (direct push bypasses validate).
    staged
        .push(CONTROL_BASE_ADDR, &[0u8; STAGE_DATA_CAP])
        .unwrap();

    let mut pending = None;
    let mut reply = FakeReply::new();
    let out = disp(&shared, &mut staged, &mut pending).dispatch(
        write(CONTROL_BASE_ADDR, &[1], false),
        RequestCtx { may_reply: true },
        &mut reply,
    );
    assert!(matches!(out, Dispatched::Done));
    // Held entries filled the buffer -- a COMMIT drains it, so Busy is honest.
    assert_eq!(reply.last().result, ResultCode::Busy);
    assert!(pending.is_none());
    // Nothing staged on top: only the pre-existing filler entry remains.
    assert_eq!(staged.iter_all().count(), 1);
}

#[test]
fn dangling_pending_write_auto_reverts_on_next_dispatch() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let mut pending = None;
    let mut reply = FakeReply::new();

    // Stage a write but never commit/revert (frame died with no verdict).
    disp(&shared, &mut staged, &mut pending).dispatch(
        write(CONTROL_BASE_ADDR, &[1], false),
        RequestCtx { may_reply: true },
        &mut reply,
    );
    assert!(pending.is_some());
    assert!(!staged.is_empty());

    // A following dispatch drops the dangling write before running.
    let mut reply2 = FakeReply::new();
    disp(&shared, &mut staged, &mut pending).dispatch(
        Request::Ping,
        RequestCtx { may_reply: true },
        &mut reply2,
    );
    assert!(pending.is_none());
    assert!(staged.is_empty());
    assert!(!shared.table.with(|t| t.control.lifecycle.torque_enable));
}

#[test]
fn real_commit_after_dangling_write_applies_only_held_entries() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let mut pending = None;
    let mut reply = FakeReply::new();

    // A HOLD write stages torque_enable=1 and gets its PASS verdict (the held
    // entry persists across frames, awaiting COMMIT).
    disp(&shared, &mut staged, &mut pending).dispatch(
        write(CONTROL_BASE_ADDR, &[1], true),
        RequestCtx { may_reply: true },
        &mut reply,
    );
    disp(&shared, &mut staged, &mut pending).commit(&mut reply);
    // A write on top stages goal_velocity, then dies (dangling -- no verdict).
    let gv = 5i32.to_le_bytes();
    disp(&shared, &mut staged, &mut pending).dispatch(
        write(GOAL_VELOCITY, &gv, false),
        RequestCtx { may_reply: true },
        &mut reply,
    );
    assert!(pending.is_some());

    // Real COMMIT auto-reverts the dangling write, then applies only the held
    // torque_enable -- the phantom goal_velocity is gone.
    disp(&shared, &mut staged, &mut pending).dispatch(
        Request::Commit,
        RequestCtx { may_reply: true },
        &mut reply,
    );
    assert!(shared.table.with(|t| t.control.lifecycle.torque_enable));
    assert_eq!(shared.table.with(|t| t.control.lifecycle.goal_velocity), 0);
}

#[test]
fn held_write_keeps_entries_through_its_verdict() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let mut pending = None;
    let mut reply = FakeReply::new();

    let out = disp(&shared, &mut staged, &mut pending).dispatch(
        write(CONTROL_BASE_ADDR, &[1], true),
        RequestCtx { may_reply: true },
        &mut reply,
    );
    assert!(matches!(out, Dispatched::Pending));
    assert_eq!(reply.last().result, ResultCode::Ok);

    // CRC passed: a held write keeps its entry staged (it applies on COMMIT).
    disp(&shared, &mut staged, &mut pending).commit(&mut reply);
    assert!(pending.is_none());
    assert!(
        !staged.is_empty(),
        "the held entry survives its own verdict"
    );
    assert!(!shared.table.with(|t| t.control.lifecycle.torque_enable));

    // A real COMMIT lands it.
    disp(&shared, &mut staged, &mut pending).dispatch(
        Request::Commit,
        RequestCtx { may_reply: true },
        &mut reply,
    );
    assert!(shared.table.with(|t| t.control.lifecycle.torque_enable));
}

#[test]
fn hooks_fire_on_write_commit() {
    use crate::regions::config::addr::common::ID;
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let mut pending = None;
    let mut reply = FakeReply::new();

    disp(&shared, &mut staged, &mut pending).dispatch(
        write(ID, &[42], false),
        RequestCtx { may_reply: true },
        &mut reply,
    );
    // Hooks are deferred to the verdict -- nothing staged through the reply yet.
    assert_eq!(reply.staged_id, None);

    disp(&shared, &mut staged, &mut pending).commit(&mut reply);
    assert_eq!(reply.staged_id, Some(42));
}

#[test]
fn hooks_fire_on_real_commit() {
    // A real COMMIT of a held hooked field fires its hook.
    use crate::regions::config::addr::common::ID;
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let mut pending = None;
    let mut reply = FakeReply::new();

    disp(&shared, &mut staged, &mut pending).dispatch(
        write(ID, &[42], true),
        RequestCtx { may_reply: true },
        &mut reply,
    );
    assert_eq!(reply.staged_id, None, "HOLD stages, no hook yet");
    disp(&shared, &mut staged, &mut pending).commit(&mut reply);
    assert_eq!(reply.staged_id, None, "held verdict defers to COMMIT");

    disp(&shared, &mut staged, &mut pending).dispatch(
        Request::Commit,
        RequestCtx { may_reply: true },
        &mut reply,
    );
    assert_eq!(reply.staged_id, Some(42), "COMMIT fires the deferred hook");
}

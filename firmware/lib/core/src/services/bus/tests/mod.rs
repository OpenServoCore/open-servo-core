//! Request-level spec tests for the single-shot [`Dispatcher`]. Each case
//! drives one decoded [`Request`] with a recording [`FakeReply`] and asserts on
//! the decoded reply shape (result / alert / data) plus any staged side effect.

use heapless::Vec;

use osc_protocol::FrameBytes;
use osc_protocol::wire::{MgmtOp, ResultCode};

use crate::regions::CONTROL_BASE_ADDR;
use crate::regions::config::BaudRate;
use crate::services::bus::Dispatcher;
use crate::traits::{Dispatch, Reply, Request, RequestCtx, SendError, Speculated, Status};
use crate::{BootMode, RegionStorage, Shared, StagedWrites};

/// One recorded `send_status` call.
struct Sent {
    result: ResultCode,
    alert: bool,
    data: Vec<u8, 256>,
}

struct FakeReply {
    sends: Vec<Sent, 8>,
    staged_id: Option<u8>,
    staged_baud: Option<BaudRate>,
    response_deadline: Option<u16>,
    reboot: Option<BootMode>,
}

impl FakeReply {
    fn new() -> Self {
        Self {
            sends: Vec::new(),
            staged_id: None,
            staged_baud: None,
            response_deadline: None,
            reboot: None,
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

    fn stage_id(&mut self, id: u8) {
        self.staged_id = Some(id);
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
}

fn go(shared: &Shared, staged: &mut StagedWrites, req: Request<'_>, may_reply: bool) -> FakeReply {
    let mut reply = FakeReply::new();
    let mut spec = None;
    Dispatcher::new(shared, staged, &mut spec).dispatch(req, RequestCtx { may_reply }, &mut reply);
    reply
}

/// Torque-on locks the config section; used by the access-error case.
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
        t.config.identity.model_number = 0x1234;
        t.config.identity.firmware_version = 0x56;
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
        .with_mut(|t| t.config.identity.model_number = 0xABCD);
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
    // §5: any read address is legal — odd-addressed spans stage through the
    // chip CRC provider's copy path (§3.2); dispatch is parity-blind.
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
    // 2 is outside a bool field's allowed {0, 1} — a field-rule rejection.
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

#[test]
fn write_torque_locked_config_maps_to_access() {
    use crate::regions::config::addr::comms::ID;
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
    assert_eq!(reply.last().result, ResultCode::Access);
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
    use crate::regions::config::addr::comms::ID;
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
    use crate::regions::config::addr::comms::BAUD_RATE_IDX;
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
    use crate::regions::config::addr::comms::RESPONSE_DEADLINE_US;
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
fn mgmt_non_reboot_ops_reply_instruction() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    for op in [MgmtOp::Save, MgmtOp::Factory, MgmtOp::Enum, MgmtOp::Assign] {
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
        .with_mut(|t| t.telemetry.fault.fault_flags = 0x04);
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

// --- Speculative dispatch (covered-complete, CRC not yet verified) ---------

use crate::regions::control::addr::lifecycle::GOAL_VELOCITY;

fn write(addr: u16, data: &[u8], hold: bool) -> Request<'_> {
    Request::Write {
        addr,
        data: FrameBytes::from(data),
        hold,
    }
}

/// A fresh dispatcher over the same session-owned `staged` + `spec` — mirrors
/// the bus rebuilding one per wake.
fn disp<'a>(
    shared: &'a Shared,
    staged: &'a mut StagedWrites,
    spec: &'a mut Option<crate::services::bus::SpecWrite>,
) -> Dispatcher<'a> {
    Dispatcher::new(shared, staged, spec)
}

#[test]
fn speculative_write_commits_and_acks() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let mut spec = None;
    let mut reply = FakeReply::new();

    let out = disp(&shared, &mut staged, &mut spec).dispatch_speculative(
        write(CONTROL_BASE_ADDR, &[1], false),
        RequestCtx { may_reply: true },
        &mut reply,
    );
    assert!(matches!(out, Speculated::Pending));
    assert_eq!(reply.last().result, ResultCode::Ok);
    // Staged only — the live table is untouched until commit.
    assert!(!shared.table.with(|t| t.control.lifecycle.torque_enable));

    disp(&shared, &mut staged, &mut spec).commit_speculation(&mut reply);
    assert!(shared.table.with(|t| t.control.lifecycle.torque_enable));
    assert!(spec.is_none());
    assert!(
        staged.is_empty(),
        "a plain write clears its staging on commit"
    );
}

#[test]
fn speculative_write_reject_nacks_nothing_staged() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let mut spec = None;
    let mut reply = FakeReply::new();

    // 2 is outside a bool field's allowed {0, 1}: validation reject.
    let out = disp(&shared, &mut staged, &mut spec).dispatch_speculative(
        write(CONTROL_BASE_ADDR, &[2], false),
        RequestCtx { may_reply: true },
        &mut reply,
    );
    assert!(matches!(out, Speculated::Done), "a reject needs no commit");
    assert_eq!(reply.last().result, ResultCode::Validation);
    assert!(staged.is_empty());
    assert!(spec.is_none());
}

#[test]
fn speculative_commit_op_is_refused_silently() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let mut spec = None;
    let mut reply = FakeReply::new();

    let out = disp(&shared, &mut staged, &mut spec).dispatch_speculative(
        Request::Commit,
        RequestCtx { may_reply: true },
        &mut reply,
    );
    assert!(matches!(out, Speculated::Refused));
    assert_eq!(reply.count(), 0, "Refused sends nothing");
    assert!(spec.is_none());
}

#[test]
fn speculative_write_staging_full_is_refused_nothing_staged() {
    use control_table::STAGE_DATA_CAP;
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    // Fill the staging data buffer to capacity (direct push bypasses validate).
    staged
        .push(CONTROL_BASE_ADDR, &[0u8; STAGE_DATA_CAP])
        .unwrap();

    let mut spec = None;
    let mut reply = FakeReply::new();
    let out = disp(&shared, &mut staged, &mut spec).dispatch_speculative(
        write(CONTROL_BASE_ADDR, &[1], false),
        RequestCtx { may_reply: true },
        &mut reply,
    );
    assert!(matches!(out, Speculated::Refused));
    assert_eq!(reply.count(), 0, "Refused sends nothing");
    assert!(spec.is_none());
    // Nothing staged on top: only the pre-existing filler entry remains.
    assert_eq!(staged.iter_all().count(), 1);
}

#[test]
fn dangling_speculative_write_auto_reverts_on_next_dispatch() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let mut spec = None;
    let mut reply = FakeReply::new();

    // Speculate a write but never commit/revert (frame died with no verdict).
    disp(&shared, &mut staged, &mut spec).dispatch_speculative(
        write(CONTROL_BASE_ADDR, &[1], false),
        RequestCtx { may_reply: true },
        &mut reply,
    );
    assert!(spec.is_some());
    assert!(!staged.is_empty());

    // A following plain dispatch drops the dangling spec before running.
    let mut reply2 = FakeReply::new();
    disp(&shared, &mut staged, &mut spec).dispatch(
        Request::Ping,
        RequestCtx { may_reply: true },
        &mut reply2,
    );
    assert!(spec.is_none());
    assert!(staged.is_empty());
    assert!(!shared.table.with(|t| t.control.lifecycle.torque_enable));
}

#[test]
fn real_commit_after_dangling_spec_applies_only_held_entries() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let mut spec = None;
    let mut reply = FakeReply::new();

    // A real HOLD write stages torque_enable=1 (persists across frames).
    disp(&shared, &mut staged, &mut spec).dispatch(
        write(CONTROL_BASE_ADDR, &[1], true),
        RequestCtx { may_reply: true },
        &mut reply,
    );
    // A speculative write on top stages goal_velocity, then dies (dangling).
    let gv = 5i32.to_le_bytes();
    disp(&shared, &mut staged, &mut spec).dispatch_speculative(
        write(GOAL_VELOCITY, &gv, false),
        RequestCtx { may_reply: true },
        &mut reply,
    );
    assert!(spec.is_some());

    // Real COMMIT auto-reverts the dangling spec, then applies only the held
    // torque_enable — the phantom goal_velocity is gone.
    disp(&shared, &mut staged, &mut spec).dispatch(
        Request::Commit,
        RequestCtx { may_reply: true },
        &mut reply,
    );
    assert!(shared.table.with(|t| t.control.lifecycle.torque_enable));
    assert_eq!(shared.table.with(|t| t.control.lifecycle.goal_velocity), 0);
}

#[test]
fn speculative_hold_keeps_entries_until_commit() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let mut spec = None;
    let mut reply = FakeReply::new();

    let out = disp(&shared, &mut staged, &mut spec).dispatch_speculative(
        write(CONTROL_BASE_ADDR, &[1], true),
        RequestCtx { may_reply: true },
        &mut reply,
    );
    assert!(matches!(out, Speculated::Pending));
    assert_eq!(reply.last().result, ResultCode::Ok);

    // CRC passed: a held write keeps its entry staged (it applies on COMMIT).
    disp(&shared, &mut staged, &mut spec).commit_speculation(&mut reply);
    assert!(spec.is_none());
    assert!(
        !staged.is_empty(),
        "the held entry survives the spec commit"
    );
    assert!(!shared.table.with(|t| t.control.lifecycle.torque_enable));

    // A real COMMIT lands it.
    disp(&shared, &mut staged, &mut spec).dispatch(
        Request::Commit,
        RequestCtx { may_reply: true },
        &mut reply,
    );
    assert!(shared.table.with(|t| t.control.lifecycle.torque_enable));
}

#[test]
fn hooks_fire_on_speculative_commit() {
    use crate::regions::config::addr::comms::ID;
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let mut spec = None;
    let mut reply = FakeReply::new();

    disp(&shared, &mut staged, &mut spec).dispatch_speculative(
        write(ID, &[42], false),
        RequestCtx { may_reply: true },
        &mut reply,
    );
    // Hooks are deferred to commit — nothing staged through the reply yet.
    assert_eq!(reply.staged_id, None);

    disp(&shared, &mut staged, &mut spec).commit_speculation(&mut reply);
    assert_eq!(reply.staged_id, Some(42));
}

#[test]
fn hooks_fire_on_real_commit() {
    // The gap fix: a real COMMIT of a held hooked field now fires its hook.
    use crate::regions::config::addr::comms::ID;
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let mut spec = None;
    let mut reply = FakeReply::new();

    disp(&shared, &mut staged, &mut spec).dispatch(
        write(ID, &[42], true),
        RequestCtx { may_reply: true },
        &mut reply,
    );
    assert_eq!(reply.staged_id, None, "HOLD stages, no hook yet");

    disp(&shared, &mut staged, &mut spec).dispatch(
        Request::Commit,
        RequestCtx { may_reply: true },
        &mut reply,
    );
    assert_eq!(reply.staged_id, Some(42), "COMMIT fires the deferred hook");
}

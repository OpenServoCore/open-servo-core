//! Request-level spec tests for the single-shot [`Dispatcher`]. Each case
//! drives one decoded [`Request`] with a recording [`FakeReply`] and asserts on
//! the decoded reply shape (result / alert / data) plus any staged side effect.

use heapless::Vec;

use osc_protocol::wire::{MgmtOp, ResultCode};

use crate::regions::CONTROL_BASE_ADDR;
use crate::regions::config::BaudRate;
use crate::services::bus::Dispatcher;
use crate::traits::{Dispatch, Reply, Request, RequestCtx, SendError, Status};
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
    Dispatcher::new(shared, staged).dispatch(req, RequestCtx { may_reply }, &mut reply);
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
            data: &[1],
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
            data: &[2],
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
            data: &[5],
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
            data: &[1],
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
            data: &[42],
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
            data: &[BaudRate::B2000000 as u8],
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
            data: &200u16.to_le_bytes(),
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
            data: &[1],
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
            data: &[1],
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
            args: &[],
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
            args: &[],
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
        let reply = go(&shared, &mut staged, Request::Mgmt { op, args: &[] }, true);
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

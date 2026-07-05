//! Request-level tests for the single-shot [`Dispatch`] path. These drive the
//! new dispatcher directly with decoded [`DxlRequest`] values and a recording
//! [`FakeReply`], mirroring the spec cases the streaming `dispatcher` tests
//! cover — reshaped to the addressing-resolved request surface (the bus, not
//! core, walks chain slots, so there are no Sync/Bulk reassembly cases here).

use super::*;
use crate::Dispatch;
use crate::StagedWrites;
use crate::services::dxl::limits::MAX_CONTROL_RW;
use crate::traits::{DxlDispatch, DxlRequest, DxlRequestCtx};
use dxl_protocol::Bytes;

/// Direct single-target instruction: replies per SRL.
fn direct() -> DxlRequestCtx {
    DxlRequestCtx {
        broadcast: false,
        may_reply: true,
        slot_reply: false,
    }
}

/// Broadcast instruction that still owes a reply (Ping).
fn broadcast_reply() -> DxlRequestCtx {
    DxlRequestCtx {
        broadcast: true,
        may_reply: true,
        slot_reply: false,
    }
}

/// Broadcast / coordinated instruction that must stay silent (Write/RegWrite/
/// Action via broadcast, SyncWrite/BulkWrite slots).
fn silent() -> DxlRequestCtx {
    DxlRequestCtx {
        broadcast: true,
        may_reply: false,
        slot_reply: false,
    }
}

/// Coordinated plain Status slot (Sync/Bulk Read).
fn plain_slot() -> DxlRequestCtx {
    DxlRequestCtx {
        broadcast: true,
        may_reply: true,
        slot_reply: false,
    }
}

/// FAST chain slot (Fast Sync/Bulk Read).
fn fast_slot() -> DxlRequestCtx {
    DxlRequestCtx {
        broadcast: true,
        may_reply: true,
        slot_reply: true,
    }
}

fn go(
    shared: &Shared,
    staged: &mut StagedWrites,
    req: DxlRequest<'_>,
    ctx: DxlRequestCtx,
) -> FakeReply {
    let mut reply = FakeReply::new();
    Dispatch::new(shared, staged).dispatch(req, ctx, &mut reply);
    reply
}

// --- Ping -----------------------------------------------------------------

#[test]
fn ping_direct_replies_with_identity() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let reply = go(&shared, &mut staged, DxlRequest::Ping, direct());
    assert_eq!(reply.send_count, 1);
    let (id, err, params) = parse_status(&reply.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
    assert_eq!(&params[..], &[0, 0, 0]);
    assert_eq!(reply.last_kind, Some(ReplyKind::Plain));
}

#[test]
fn ping_broadcast_replies() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let reply = go(&shared, &mut staged, DxlRequest::Ping, broadcast_reply());
    assert_eq!(reply.send_count, 1);
    let (_, err, _) = parse_status(&reply.tx);
    assert_eq!(err, 0);
}

#[test]
fn ping_replies_even_when_srl_none() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut staged = StagedWrites::new();
    let reply = go(&shared, &mut staged, DxlRequest::Ping, direct());
    assert_eq!(reply.send_count, 1);
}

// --- Read -----------------------------------------------------------------

#[test]
fn read_returns_register_bytes() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let reply = go(
        &shared,
        &mut staged,
        DxlRequest::Read {
            address: 0,
            length: 2,
        },
        direct(),
    );
    let (id, err, params) = parse_status(&reply.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
    assert_eq!(&params[..], &[0, 0]);
}

#[test]
fn read_zero_length_rejects_with_data_range() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let reply = go(
        &shared,
        &mut staged,
        DxlRequest::Read {
            address: 0,
            length: 0,
        },
        direct(),
    );
    let (_, err, _) = parse_status(&reply.tx);
    assert_eq!(err, StatusError::code(ErrorCode::DataRange).as_byte());
}

#[test]
fn read_srl_none_silences_reply() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut staged = StagedWrites::new();
    let reply = go(
        &shared,
        &mut staged,
        DxlRequest::Read {
            address: 0,
            length: 2,
        },
        direct(),
    );
    assert_eq!(reply.send_count, 0);
    assert!(reply.tx.is_empty());
}

#[test]
fn read_unmapped_returns_zero_bytes() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::Read);
    let mut staged = StagedWrites::new();
    let reply = go(
        &shared,
        &mut staged,
        DxlRequest::Read {
            address: 0xFFFE,
            length: 1,
        },
        direct(),
    );
    let (_, err, params) = parse_status(&reply.tx);
    assert_eq!(err, 0);
    assert_eq!(&params[..], &[0]);
}

#[test]
fn plain_slot_read_emits_plain_status() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let reply = go(
        &shared,
        &mut staged,
        DxlRequest::Read {
            address: 0,
            length: 2,
        },
        plain_slot(),
    );
    assert_eq!(reply.send_count, 1);
    assert_eq!(reply.last_kind, Some(ReplyKind::Plain));
}

#[test]
fn fast_slot_read_emits_slot_reply() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let reply = go(
        &shared,
        &mut staged,
        DxlRequest::Read {
            address: 0,
            length: 2,
        },
        fast_slot(),
    );
    assert_eq!(reply.send_count, 1);
    assert_eq!(reply.last_kind, Some(ReplyKind::Slot));
}

#[test]
fn fast_slot_read_srl_none_silent() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut staged = StagedWrites::new();
    let reply = go(
        &shared,
        &mut staged,
        DxlRequest::Read {
            address: 0,
            length: 2,
        },
        fast_slot(),
    );
    assert_eq!(reply.send_count, 0);
}

#[test]
fn fast_slot_read_invalid_length_silent() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let reply = go(
        &shared,
        &mut staged,
        DxlRequest::Read {
            address: 0,
            length: 0,
        },
        fast_slot(),
    );
    assert_eq!(reply.send_count, 0);
    assert!(reply.tx.is_empty());
}

// --- Write ----------------------------------------------------------------

#[test]
fn write_to_rw_address_succeeds_and_mutates() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let reply = go(
        &shared,
        &mut staged,
        DxlRequest::Write {
            address: CONTROL_BASE_ADDR,
            data: Bytes::raw(&[1]),
        },
        direct(),
    );
    let (id, err, params) = parse_status(&reply.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
    assert!(params.is_empty());
    assert!(shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn write_to_ro_address_replies_access_error() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let reply = go(
        &shared,
        &mut staged,
        DxlRequest::Write {
            address: 0,
            data: Bytes::raw(&[0xAA, 0xBB]),
        },
        direct(),
    );
    let (_, err, _) = parse_status(&reply.tx);
    assert_eq!(err, StatusError::code(ErrorCode::Access).as_byte());
    assert_eq!(shared.table.config.with(|c| c.identity.model_number), 0);
}

#[test]
fn write_to_unmapped_address_replies_data_range() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let reply = go(
        &shared,
        &mut staged,
        DxlRequest::Write {
            address: 0xFFFE,
            data: Bytes::raw(&[1]),
        },
        direct(),
    );
    let (_, err, _) = parse_status(&reply.tx);
    assert_eq!(err, StatusError::code(ErrorCode::DataRange).as_byte());
}

#[test]
fn write_overlong_data_rejects_with_data_range() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let big = [0u8; MAX_CONTROL_RW + 1];
    let reply = go(
        &shared,
        &mut staged,
        DxlRequest::Write {
            address: CONTROL_BASE_ADDR,
            data: Bytes::raw(&big),
        },
        direct(),
    );
    let (_, err, _) = parse_status(&reply.tx);
    assert_eq!(err, StatusError::code(ErrorCode::DataRange).as_byte());
}

#[test]
fn broadcast_write_applies_but_silent() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let reply = go(
        &shared,
        &mut staged,
        DxlRequest::Write {
            address: CONTROL_BASE_ADDR,
            data: Bytes::raw(&[1]),
        },
        silent(),
    );
    assert_eq!(reply.send_count, 0);
    assert!(shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn write_srl_none_silences_ack_but_applies() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut staged = StagedWrites::new();
    let reply = go(
        &shared,
        &mut staged,
        DxlRequest::Write {
            address: CONTROL_BASE_ADDR,
            data: Bytes::raw(&[1]),
        },
        direct(),
    );
    assert_eq!(reply.send_count, 0);
    assert!(shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn write_srl_read_silences_write_ack() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::Read);
    let mut staged = StagedWrites::new();
    let reply = go(
        &shared,
        &mut staged,
        DxlRequest::Write {
            address: CONTROL_BASE_ADDR,
            data: Bytes::raw(&[1]),
        },
        direct(),
    );
    assert_eq!(reply.send_count, 0);
}

#[test]
fn write_srl_none_silences_error_reply() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut staged = StagedWrites::new();
    let reply = go(
        &shared,
        &mut staged,
        DxlRequest::Write {
            address: 0,
            data: Bytes::raw(&[0xAA, 0xBB]),
        },
        direct(),
    );
    assert_eq!(reply.send_count, 0);
    assert!(reply.tx.is_empty());
}

#[test]
fn baud_write_stages_via_reply_handle() {
    use crate::regions::config::addr::comms::BAUD_RATE_IDX;
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let reply = go(
        &shared,
        &mut staged,
        DxlRequest::Write {
            address: BAUD_RATE_IDX,
            data: Bytes::raw(&[BaudRate::B1000000 as u8]),
        },
        direct(),
    );
    assert_eq!(reply.last_baud_staged, Some(BaudRate::B1000000));
}

#[test]
fn id_write_stages_via_reply_handle() {
    use crate::regions::config::addr::comms::ID;
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let reply = go(
        &shared,
        &mut staged,
        DxlRequest::Write {
            address: ID,
            data: Bytes::raw(&[42]),
        },
        direct(),
    );
    assert_eq!(reply.last_id_staged, Some(42));
}

#[test]
fn rdt_write_stages_via_reply_handle_in_us() {
    use crate::regions::config::addr::comms::RETURN_DELAY_2US;
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let reply = go(
        &shared,
        &mut staged,
        DxlRequest::Write {
            address: RETURN_DELAY_2US,
            data: Bytes::raw(&[100]),
        },
        direct(),
    );
    assert_eq!(reply.last_rdt_staged, Some(200));
}

// --- RegWrite + Action ----------------------------------------------------

#[test]
fn reg_write_then_action_commits_to_live_table() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut staged = StagedWrites::new();

    let reply = go(
        &shared,
        &mut staged,
        DxlRequest::RegWrite {
            address: CONTROL_BASE_ADDR,
            data: Bytes::raw(&[1]),
        },
        direct(),
    );
    let (_, err, _) = parse_status(&reply.tx);
    assert_eq!(err, 0);
    assert!(!shared.table.control.with(|c| c.lifecycle.torque_enable));

    let reply = go(&shared, &mut staged, DxlRequest::Action, direct());
    let (_, err, _) = parse_status(&reply.tx);
    assert_eq!(err, 0);
    assert!(shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn reg_write_to_ro_address_replies_access_error() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let reply = go(
        &shared,
        &mut staged,
        DxlRequest::RegWrite {
            address: 0,
            data: Bytes::raw(&[0xAA, 0xBB]),
        },
        direct(),
    );
    let (_, err, _) = parse_status(&reply.tx);
    assert_eq!(err, StatusError::code(ErrorCode::Access).as_byte());
}

#[test]
fn reg_write_invalid_value_rejected_at_stage_time() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut staged = StagedWrites::new();

    let reply = go(
        &shared,
        &mut staged,
        DxlRequest::RegWrite {
            address: CONTROL_BASE_ADDR,
            data: Bytes::raw(&[2]),
        },
        direct(),
    );
    let (_, err, _) = parse_status(&reply.tx);
    assert_eq!(err, StatusError::code(ErrorCode::DataRange).as_byte());

    let reply = go(&shared, &mut staged, DxlRequest::Action, direct());
    let (_, err, _) = parse_status(&reply.tx);
    assert_eq!(err, 0);
    assert!(!shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn write_preserves_pending_reg_write_chain() {
    use crate::regions::CONTROL_BASE_ADDR;
    use crate::regions::control::Mode;
    let shared = Shared::new();
    let mut staged = StagedWrites::new();

    go(
        &shared,
        &mut staged,
        DxlRequest::RegWrite {
            address: CONTROL_BASE_ADDR,
            data: Bytes::raw(&[1]),
        },
        direct(),
    );
    go(
        &shared,
        &mut staged,
        DxlRequest::Write {
            address: CONTROL_BASE_ADDR + 1,
            data: Bytes::raw(&[1]),
        },
        direct(),
    );
    assert_eq!(
        shared.table.control.with(|c| c.lifecycle.mode),
        Mode::PositionPid,
    );
    assert!(!shared.table.control.with(|c| c.lifecycle.torque_enable));

    let reply = go(&shared, &mut staged, DxlRequest::Action, direct());
    let (_, err, _) = parse_status(&reply.tx);
    assert_eq!(err, 0);
    assert!(shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn broadcast_reg_write_and_action_silent_but_commits() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut staged = StagedWrites::new();

    let reply = go(
        &shared,
        &mut staged,
        DxlRequest::RegWrite {
            address: CONTROL_BASE_ADDR,
            data: Bytes::raw(&[1]),
        },
        silent(),
    );
    assert_eq!(reply.send_count, 0);

    let reply = go(&shared, &mut staged, DxlRequest::Action, silent());
    assert_eq!(reply.send_count, 0);
    assert!(shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn action_with_empty_staging_replies_ok() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let reply = go(&shared, &mut staged, DxlRequest::Action, direct());
    let (_, err, _) = parse_status(&reply.tx);
    assert_eq!(err, 0);
}

#[test]
fn action_srl_none_silences_ack_but_commits() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut staged = StagedWrites::new();

    go(
        &shared,
        &mut staged,
        DxlRequest::RegWrite {
            address: CONTROL_BASE_ADDR,
            data: Bytes::raw(&[1]),
        },
        direct(),
    );
    let reply = go(&shared, &mut staged, DxlRequest::Action, direct());
    assert_eq!(reply.send_count, 0);
    assert!(shared.table.control.with(|c| c.lifecycle.torque_enable));
}

// --- Reboot ---------------------------------------------------------------

#[test]
fn reboot_direct_acks_and_stages() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let reply = go(&shared, &mut staged, DxlRequest::Reboot, direct());
    let (id, err, params) = parse_status(&reply.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
    assert!(params.is_empty());
    assert_eq!(reply.reboot_count, 1);
    assert_eq!(reply.last_reboot_mode, Some(BootMode::App));
}

#[test]
fn reboot_broadcast_stages_without_ack() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let reply = go(&shared, &mut staged, DxlRequest::Reboot, silent());
    assert_eq!(reply.send_count, 0);
    assert!(reply.tx.is_empty());
    assert_eq!(reply.reboot_count, 1);
    assert_eq!(reply.last_reboot_mode, Some(BootMode::App));
}

#[test]
fn reboot_honors_staged_boot_mode() {
    use crate::regions::control::addr::system::BOOT_MODE;
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    go(
        &shared,
        &mut staged,
        DxlRequest::Write {
            address: BOOT_MODE,
            data: Bytes::raw(&[BootMode::Bootloader as u8]),
        },
        direct(),
    );
    let reply = go(&shared, &mut staged, DxlRequest::Reboot, direct());
    assert_eq!(reply.reboot_count, 1);
    assert_eq!(reply.last_reboot_mode, Some(BootMode::Bootloader));
}

#[test]
fn reboot_srl_none_silences_ack_but_fires() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut staged = StagedWrites::new();
    let reply = go(&shared, &mut staged, DxlRequest::Reboot, direct());
    assert_eq!(reply.send_count, 0);
    assert!(reply.tx.is_empty());
    assert_eq!(reply.reboot_count, 1);
    assert_eq!(reply.last_reboot_mode, Some(BootMode::App));
}

// --- Unsupported (FactoryReset / Clear / ControlTableBackup / Ext) --------

#[test]
fn factory_reset_direct_replies_instruction_error() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let reply = go(
        &shared,
        &mut staged,
        DxlRequest::FactoryReset { mode: 0xFF },
        direct(),
    );
    let (_, err, _) = parse_status(&reply.tx);
    assert_eq!(err, StatusError::code(ErrorCode::Instruction).as_byte());
}

#[test]
fn factory_reset_srl_none_silent() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut staged = StagedWrites::new();
    let reply = go(
        &shared,
        &mut staged,
        DxlRequest::FactoryReset { mode: 0xFF },
        direct(),
    );
    assert_eq!(reply.send_count, 0);
    assert!(reply.tx.is_empty());
}

#[test]
fn clear_and_backup_and_ext_reply_instruction_error() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    for req in [
        DxlRequest::Clear {
            body: Bytes::raw(&[]),
        },
        DxlRequest::ControlTableBackup {
            body: Bytes::raw(&[]),
        },
        DxlRequest::Ext {
            instruction: 0xC3,
            params: Bytes::raw(&[]),
        },
    ] {
        let reply = go(&shared, &mut staged, req, direct());
        let (_, err, _) = parse_status(&reply.tx);
        assert_eq!(err, StatusError::code(ErrorCode::Instruction).as_byte());
    }
}

#[test]
fn unsupported_broadcast_silent() {
    let shared = Shared::new();
    let mut staged = StagedWrites::new();
    let reply = go(
        &shared,
        &mut staged,
        DxlRequest::FactoryReset { mode: 0xFF },
        silent(),
    );
    assert_eq!(reply.send_count, 0);
    assert!(reply.tx.is_empty());
}

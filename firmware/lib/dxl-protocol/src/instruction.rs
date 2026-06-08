#![allow(dead_code)]

pub const INSTR_PING: u8 = 0x01;
pub const INSTR_READ: u8 = 0x02;
pub const INSTR_WRITE: u8 = 0x03;
pub const INSTR_REG_WRITE: u8 = 0x04;
pub const INSTR_ACTION: u8 = 0x05;
pub const INSTR_FACTORY_RESET: u8 = 0x06;
pub const INSTR_REBOOT: u8 = 0x08;
pub const INSTR_STATUS: u8 = 0x55;
pub const INSTR_SYNC_READ: u8 = 0x82;
pub const INSTR_SYNC_WRITE: u8 = 0x83;
pub const INSTR_FAST_SYNC_READ: u8 = 0x8A;
pub const INSTR_BULK_READ: u8 = 0x92;
pub const INSTR_BULK_WRITE: u8 = 0x93;
pub const INSTR_FAST_BULK_READ: u8 = 0x9A;

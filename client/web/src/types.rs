//! Everything that crosses the JS boundary by value: serde mirrors of the
//! osc-client types with tsify declarations. Bytes travel as Uint8Array,
//! every other scalar as number.

use osc_client::BaudRate as WireBaud;
use osc_client::data_state;
use osc_client::descriptor as desc;
use serde::{Deserialize, Serialize};
use tsify::Tsify;

/// `[major, minor, patch]` of a packed firmware version.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Tsify)]
pub struct Version(pub u8, pub u8, pub u8);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Tsify)]
#[serde(rename_all = "camelCase")]
pub enum BaudRate {
    B500000,
    B1000000,
    B2000000,
    B3000000,
}

impl From<WireBaud> for BaudRate {
    fn from(r: WireBaud) -> Self {
        match r {
            WireBaud::B500000 => BaudRate::B500000,
            WireBaud::B1000000 => BaudRate::B1000000,
            WireBaud::B2000000 => BaudRate::B2000000,
            WireBaud::B3000000 => BaudRate::B3000000,
        }
    }
}

impl From<BaudRate> for WireBaud {
    fn from(r: BaudRate) -> Self {
        match r {
            BaudRate::B500000 => WireBaud::B500000,
            BaudRate::B1000000 => WireBaud::B1000000,
            BaudRate::B2000000 => WireBaud::B2000000,
            BaudRate::B3000000 => WireBaud::B3000000,
        }
    }
}

/// Adapter diagnostics tail of the INFO record.
#[derive(Debug, Clone, Copy, Serialize, Tsify)]
#[serde(rename_all = "camelCase")]
pub struct Diag {
    pub reset: u8,
    pub phase: u8,
    pub crash_seq: u32,
    pub mcause: u32,
    pub mepc: u32,
    pub mtval: u32,
    pub hse_fail: u32,
    pub resets: u32,
    pub uptime_ms: u32,
}

#[derive(Debug, Clone, Copy, Serialize, Tsify)]
#[serde(rename_all = "camelCase")]
pub struct LinkInfo {
    pub version: u8,
    pub ticks_per_us: u32,
    /// Absent from a pre-v3 adapter.
    #[tsify(optional)]
    #[serde(skip_serializing_if = "Option::is_none")]
    pub diag: Option<Diag>,
}

impl From<osc_client::session::LinkInfo> for LinkInfo {
    fn from(i: osc_client::session::LinkInfo) -> Self {
        LinkInfo {
            version: i.version,
            ticks_per_us: i.ticks_per_us,
            diag: i.diag.map(|d| Diag {
                reset: d.reset,
                phase: d.phase,
                crash_seq: d.crash_seq,
                mcause: d.mcause,
                mepc: d.mepc,
                mtval: d.mtval,
                hse_fail: d.hse_fail,
                resets: d.resets,
                uptime_ms: d.uptime_ms,
            }),
        }
    }
}

#[derive(Debug, Clone, Copy, Serialize, Tsify)]
#[serde(rename_all = "camelCase")]
pub struct Rails {
    pub v3v3: bool,
    pub v5: bool,
}

impl From<(bool, bool)> for Rails {
    fn from((v3v3, v5): (bool, bool)) -> Self {
        Rails { v3v3, v5 }
    }
}

/// One discovered node: the id it answered from and its UID as 32 hex
/// chars, most significant byte first.
#[derive(Debug, Clone, Serialize, Tsify)]
#[serde(rename_all = "camelCase")]
pub struct Found {
    pub id: u8,
    pub uid: String,
}

impl From<&osc_client::mgmt::Found> for Found {
    fn from(f: &osc_client::mgmt::Found) -> Self {
        Found {
            id: f.id.as_byte(),
            uid: crate::uid::to_hex(&f.uid),
        }
    }
}

/// One ping-sweep verdict (the roster `setBaud` reports).
#[derive(Debug, Clone, Copy, Serialize, Tsify)]
#[serde(rename_all = "camelCase")]
pub struct Alive {
    pub id: u8,
    pub alive: bool,
}

#[derive(Debug, Clone, Copy, Serialize, Tsify)]
#[serde(rename_all = "camelCase")]
pub struct Ping {
    pub model: u16,
    pub fw: u16,
    pub alert: bool,
}

impl From<osc_client::Ping> for Ping {
    fn from(p: osc_client::Ping) -> Self {
        Ping {
            model: p.model,
            fw: p.fw,
            alert: p.alert,
        }
    }
}

#[derive(Debug, Clone, Copy, Serialize, Tsify)]
#[serde(rename_all = "camelCase")]
pub struct Identity {
    pub model: u16,
    pub fw: u16,
    pub hw: u8,
    pub capabilities: u32,
    /// Samples a full TEL frame carries; absent on firmware that predates
    /// the register.
    pub tel_frame_samples: Option<u8>,
}

impl From<osc_client::common::Identity> for Identity {
    fn from(i: osc_client::common::Identity) -> Self {
        Identity {
            model: i.model,
            fw: i.fw,
            hw: i.hw,
            capabilities: i.capabilities,
            tel_frame_samples: i.tel_frame_samples,
        }
    }
}

#[derive(Debug, Clone, Copy, Serialize, Tsify)]
#[serde(rename_all = "camelCase")]
pub struct Health {
    pub fault_flags: u8,
    pub config_dirty: bool,
    pub trim_steps: i8,
    pub crc_fail_count: u32,
    pub framing_drop_count: u32,
}

impl From<osc_client::common::Health> for Health {
    fn from(h: osc_client::common::Health) -> Self {
        Health {
            fault_flags: h.fault_flags,
            config_dirty: h.config_dirty,
            trim_steps: h.trim_steps,
            crc_fail_count: h.crc_fail_count,
            framing_drop_count: h.framing_drop_count,
        }
    }
}

/// One `data_flags` reason: its firmware name and what to do about it.
#[derive(Debug, Clone, Serialize, Tsify)]
#[serde(rename_all = "camelCase")]
pub struct DataReason {
    pub name: String,
    pub text: String,
}

impl From<data_state::Reason> for DataReason {
    fn from(r: data_state::Reason) -> Self {
        DataReason {
            name: r.name().into(),
            text: r.text().into(),
        }
    }
}

/// Whether the persisted images and the identified set are the servo's
/// own (`data_flags`), with the kernel's latched fault beside it.
#[derive(Debug, Clone, Serialize, Tsify)]
#[serde(rename_all = "camelCase")]
pub struct DataState {
    pub flags: u8,
    pub fault_code: u8,
    /// `faultCode` by name; absent for a code this build does not know.
    #[tsify(optional)]
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fault: Option<String>,
    /// Most urgent first.
    pub reasons: Vec<DataReason>,
    /// The operator line; absent when nothing is wrong.
    #[tsify(optional)]
    #[serde(skip_serializing_if = "Option::is_none")]
    pub message: Option<String>,
    /// OpenLoop and Current may be enabled.
    pub open_loop: bool,
    /// Velocity and Position may be enabled.
    pub closed_loop: bool,
}

impl From<data_state::DataState> for DataState {
    fn from(s: data_state::DataState) -> Self {
        DataState {
            flags: s.flags,
            fault_code: s.fault_code,
            fault: data_state::fault::name(s.fault_code).map(String::from),
            reasons: s.reasons().into_iter().map(DataReason::from).collect(),
            message: s.message(),
            open_loop: data_state::allows(s.flags, false),
            closed_loop: data_state::allows(s.flags, true),
        }
    }
}

/// The stamp the servo holds beside the one its live set computes to.
#[derive(Debug, Clone, Copy, Serialize, Tsify)]
#[serde(rename_all = "camelCase")]
pub struct StampVerdict {
    pub stored: u16,
    pub computed: u16,
    pub matches: bool,
}

impl From<osc_client::stamp::Verdict> for StampVerdict {
    fn from(v: osc_client::stamp::Verdict) -> Self {
        StampVerdict {
            stored: v.stored,
            computed: v.computed,
            matches: v.matches(),
        }
    }
}

/// The position table as the servo holds it: `pos_lut_state` by value and
/// name, and the 256 points behind the CONTROL window (i16 corrections
/// against the identity ramp, point k at raw k * 16). The kernel applies them
/// only while `live`.
#[derive(Debug, Clone, Serialize, Tsify)]
#[serde(rename_all = "camelCase")]
pub struct PosLut {
    pub state: u8,
    /// `state` by name; absent for a value this build does not know.
    #[tsify(optional)]
    #[serde(skip_serializing_if = "Option::is_none")]
    pub state_name: Option<String>,
    pub live: bool,
    #[tsify(type = "number[]")]
    pub points: Vec<i16>,
}

impl From<osc_client::pos_lut::PosLut> for PosLut {
    fn from(l: osc_client::pos_lut::PosLut) -> Self {
        PosLut {
            state: l.state,
            state_name: l.state_name().map(String::from),
            live: l.live(),
            points: l.points.to_vec(),
        }
    }
}

/// One collected TEL burst (protocol sec 5.6): the CRC-clean stream frame
/// payloads in arrival order (byte 0 is the stream_seq; a dropped frame
/// is a seq hole of `frameSamples` ticks plus `garble`), `complete` false
/// when the window expired before the LAST-flagged frame.
#[derive(Debug, Clone, Serialize, Tsify)]
#[serde(rename_all = "camelCase")]
pub struct TelBurst {
    #[tsify(type = "Uint8Array[]")]
    pub frames: Vec<serde_bytes::ByteBuf>,
    /// The servo's batch: samples per full frame, so frame k's sample i is
    /// tick `k * frameSamples + i`.
    pub frame_samples: u8,
    pub complete: bool,
    /// Engine tick at the terminal (`LinkInfo.ticksPerUs` converts).
    pub tick: u32,
    /// Engine-counted clean statuses (ack + frames).
    pub statuses: u16,
    pub garble: u16,
    pub trailing: bool,
}

/// A recorded fast-tick track for the simulated adapter: one column per
/// `TelSample` field, all the same length, in device counts as captured
/// (`windowValid` is 0/1). Plain `number[]` columns are accepted too.
#[cfg(feature = "fake")]
#[derive(Debug, Clone, Deserialize, Tsify)]
#[serde(rename_all = "camelCase")]
pub struct Track {
    #[tsify(type = "Uint16Array")]
    pub pos: Vec<u16>,
    #[tsify(type = "Int16Array")]
    pub current: Vec<i16>,
    #[tsify(type = "Uint16Array")]
    pub current_trough: Vec<u16>,
    #[tsify(type = "Int16Array")]
    pub duty_q15: Vec<i16>,
    #[tsify(type = "Int16Array")]
    pub vdiff: Vec<i16>,
    #[tsify(type = "Uint16Array")]
    pub vbus: Vec<u16>,
    #[tsify(type = "Uint16Array")]
    pub current_raw: Vec<u16>,
    #[tsify(type = "Uint16Array")]
    pub vmotor_a: Vec<u16>,
    #[tsify(type = "Uint16Array")]
    pub vmotor_b: Vec<u16>,
    #[tsify(type = "Uint16Array")]
    pub vbus_raw: Vec<u16>,
    #[tsify(type = "Uint16Array")]
    pub ntc_raw: Vec<u16>,
    #[tsify(type = "Uint8Array")]
    pub window_valid: Vec<u8>,
}

/// One `fakeWithTracks` roster entry: a servo id, optionally playing a
/// track back, optionally factory-fresh (never calibrated, identified or
/// saved: every `data_flags` reason set) instead of stamped, optionally
/// with its winding thermometer off or pinned in the derate band.
#[cfg(feature = "fake")]
#[derive(Debug, Clone, Deserialize, Tsify)]
#[serde(rename_all = "camelCase")]
pub struct FakeServo {
    pub id: u8,
    #[tsify(optional)]
    pub track: Option<Track>,
    #[tsify(optional)]
    #[serde(default)]
    pub virgin: bool,
    /// The winding thermometer of a calibrated servo (default "on").
    #[tsify(optional, type = "\"on\" | \"off\" | \"derating\"")]
    #[serde(default)]
    pub therm: osc_client::fake::Therm,
}

#[derive(Debug, Clone, Copy, Serialize, Tsify)]
#[serde(rename_all = "lowercase")]
pub enum Access {
    Ro,
    Rw,
}

#[derive(Debug, Clone, Copy, Serialize, Tsify)]
#[serde(rename_all = "lowercase")]
pub enum Kind {
    Uint,
    Int,
    Bool,
    Enum,
    Bytes,
}

#[derive(Debug, Clone, Serialize, Tsify)]
#[serde(rename_all = "camelCase")]
pub struct Variant {
    pub name: String,
    pub value: u8,
}

#[derive(Debug, Clone, Serialize, Tsify)]
#[serde(rename_all = "camelCase")]
pub struct Field {
    pub name: String,
    pub addr: u16,
    pub width: u16,
    pub access: Access,
    pub kind: Kind,
    #[tsify(optional)]
    #[serde(skip_serializing_if = "Option::is_none")]
    pub min: Option<i64>,
    #[tsify(optional)]
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max: Option<i64>,
    pub variants: Vec<Variant>,
}

impl From<&desc::Field> for Field {
    fn from(f: &desc::Field) -> Self {
        Field {
            name: f.name.clone(),
            addr: f.addr,
            width: f.width,
            access: match f.access {
                desc::Access::Ro => Access::Ro,
                desc::Access::Rw => Access::Rw,
            },
            kind: match f.kind {
                desc::Kind::Uint => Kind::Uint,
                desc::Kind::Int => Kind::Int,
                desc::Kind::Bool => Kind::Bool,
                desc::Kind::Enum => Kind::Enum,
                desc::Kind::Bytes => Kind::Bytes,
            },
            min: f.min,
            max: f.max,
            variants: f
                .variants
                .iter()
                .map(|v| Variant {
                    name: v.name.clone(),
                    value: v.value,
                })
                .collect(),
        }
    }
}

/// A decoded register value; the variant follows the field's kind.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Tsify)]
#[serde(tag = "kind", content = "value", rename_all = "camelCase")]
pub enum Value {
    Uint(u64),
    Int(i64),
    Bool(bool),
    Enum(u8),
    Bytes(
        #[serde(with = "serde_bytes")]
        #[tsify(type = "Uint8Array")]
        Vec<u8>,
    ),
}

impl From<desc::Value> for Value {
    fn from(v: desc::Value) -> Self {
        match v {
            desc::Value::Uint(v) => Value::Uint(v),
            desc::Value::Int(v) => Value::Int(v),
            desc::Value::Bool(v) => Value::Bool(v),
            desc::Value::Enum(v) => Value::Enum(v),
            desc::Value::Bytes(v) => Value::Bytes(v),
        }
    }
}

impl From<Value> for desc::Value {
    fn from(v: Value) -> Self {
        match v {
            Value::Uint(v) => desc::Value::Uint(v),
            Value::Int(v) => desc::Value::Int(v),
            Value::Bool(v) => desc::Value::Bool(v),
            Value::Enum(v) => desc::Value::Enum(v),
            Value::Bytes(v) => desc::Value::Bytes(v),
        }
    }
}

/// `Registry.select`'s verdict; `index` feeds `Registry.at`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Tsify)]
#[serde(tag = "kind", rename_all = "camelCase")]
pub enum Selection {
    Exact {
        index: usize,
    },
    /// Same major, older minor: a valid subset of the servo's table.
    OlderMinor {
        index: usize,
    },
    /// Model known, no layout at or below the servo's major.minor;
    /// `newest` is the highest `[major, minor]` known for the model.
    Incompatible {
        newest: (u8, u8),
    },
    UnknownModel,
}

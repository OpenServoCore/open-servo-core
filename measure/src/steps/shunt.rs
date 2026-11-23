//! One shunt burst as the steps arm it: the rail, the current zero, the
//! terminal bias and the pot read just before the arm, the burst at one
//! duty from the duty in force, then the current read back after it and
//! the fault flags. A refused arm names the rule the host can see broken.

use osc_client::pipe::Pipe;

use super::{Halt, Rig, mean_u16, read_u8, read_u16};
use crate::capture::burst::{self, ArmSeen, CHANS_DIFF, Capture, CaptureCfg, Pre};

/// Reads of the current after a burst its level back is the mean of.
pub const AFTER_READS: u32 = 4;

/// What a burst is armed and named by.
#[derive(Clone, Debug, PartialEq)]
pub struct Arm {
    /// The `drive_polarity` in force, for the capture's meta.
    pub polarity: bool,
    /// The rotor rests against a stop.
    pub seated: bool,
    /// Millivolts a vcount of the rail, to name a refused arm.
    pub mv_per_count: f64,
    /// The soft limits in force, as the table holds them.
    pub soft: [Vec<u8>; 2],
}

/// A burst taken: its capture, the current read after it, counts, and the
/// host's clock around it.
#[derive(Clone, Debug, PartialEq)]
pub struct Shunted {
    pub cap: Capture,
    pub after_counts: f64,
    pub times: (f64, f64),
}

/// One shunt burst at `duty_q15` from zero duty, named `label` in what it
/// says: a refused arm refuses the step, a fault latched in it aborts.
pub async fn shot<P: Pipe>(
    rig: &mut Rig<'_, P>,
    arm: &Arm,
    duty_q15: i16,
    label: &str,
) -> Result<Shunted, Halt> {
    let pre = Pre {
        pre_q15: 0,
        vbus_raw: read_u16(rig, "vbus_raw").await?,
        bias: read_u16(rig, "current_bias_counts").await?,
        vmotor_bias: read_u16(rig, "vmotor_bias_counts").await?,
        pos: read_u16(rig, "pos").await?,
        seated: arm.seated,
        drive_polarity: arm.polarity,
    };
    let t0 = (rig.clock)();
    let cap = match burst::shunt(
        rig.c,
        rig.id,
        rig.d,
        duty_q15,
        CHANS_DIFF,
        pre,
        &CaptureCfg::default(),
    )
    .await
    {
        Ok(c) => c,
        Err(burst::Error::Rejected) => {
            let word = |v: &[u8]| i32::from_le_bytes(v[..4].try_into().unwrap_or([0; 4]));
            let seen = ArmSeen {
                vbus_counts: read_u16(rig, "vbus_counts").await?,
                mv_per_count: arm.mv_per_count,
                pos: pre.pos,
                soft: (word(&arm.soft[0]), word(&arm.soft[1])),
                limit_flags: read_u8(rig, "limit_flags").await?,
                fault_flags: read_u8(rig, "fault_flags").await?,
            };
            return Err(Halt::Refused(format!(
                "{label}: {}",
                burst::rejected(duty_q15, &seen)
            )));
        }
        Err(burst::Error::Io(e)) => return Err(Halt::from(e)),
        Err(e) => return Err(Halt::Error(format!("shunt {label}: {e}"))),
    };
    let t1 = (rig.clock)();
    let after_counts = mean_u16(rig, "current_raw", AFTER_READS).await?;
    let faults = read_u8(rig, "fault_flags").await?;
    if faults != 0 {
        let code = read_u8(rig, "fault_code").await.unwrap_or(0);
        return Err(Halt::Error(format!(
            "a fault latched during {label}: fault_flags {faults:#04x}, fault_code {code}"
        )));
    }
    Ok(Shunted {
        cap,
        after_counts,
        times: (t0, t1),
    })
}

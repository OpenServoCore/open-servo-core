//! `plan` and `step` on the file system: [`osc_measure::run::steps`], the
//! run folders under the paths given.

use std::path::{Path, PathBuf};
use std::sync::atomic::AtomicBool;

use anyhow::Result;
use osc_client::descriptor::Descriptor;
use osc_client::pipe::Pipe;
use osc_client::{Client, Id};
use osc_measure::run::steps as of;

pub use osc_measure::run::steps::{StepInputs, clear_unsaved, plan};

use crate::files::Fs;

/// [`StepInputs`] from the files at their paths, each line said about them
/// printed to stderr.
pub trait Load: Sized {
    fn load(nameplate: Option<&Path>, policy: Option<&Path>) -> Result<Self>;
}

impl Load for StepInputs {
    fn load(nameplate: Option<&Path>, policy: Option<&Path>) -> Result<Self> {
        let (i, notes) = StepInputs::read(&mut Fs, nameplate, policy)?;
        for n in &notes {
            eprintln!("{n}");
        }
        Ok(i)
    }
}

/// The lines a step command prints, whether it was accepted, and its run
/// folder.
type Stepped = Result<(Vec<String>, bool, PathBuf)>;

pub async fn step_rest<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    i: &StepInputs,
    out: &Path,
    servo: &str,
    stop: &AtomicBool,
) -> Stepped {
    of::step_rest(c, id, d, &mut crate::host(&mut Fs, stop), i, out, servo).await
}

pub async fn step_probe<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    i: &StepInputs,
    out: &Path,
    servo: &str,
    stop: &AtomicBool,
) -> Stepped {
    of::step_probe(c, id, d, &mut crate::host(&mut Fs, stop), i, out, servo).await
}

pub async fn step_verify<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    i: &StepInputs,
    out: &Path,
    servo: &str,
    stop: &AtomicBool,
) -> Stepped {
    of::step_verify(c, id, d, &mut crate::host(&mut Fs, stop), i, out, servo).await
}

#[allow(clippy::too_many_arguments)]
pub async fn step_cruise<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    i: &StepInputs,
    out: &Path,
    servo: &str,
    stop: &AtomicBool,
    run: Option<&Path>,
) -> Stepped {
    of::step_cruise(
        c,
        id,
        d,
        &mut crate::host(&mut Fs, stop),
        i,
        out,
        servo,
        run,
    )
    .await
}

pub async fn step_stops<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    i: &StepInputs,
    out: &Path,
    servo: &str,
    stop: &AtomicBool,
) -> Stepped {
    of::step_stops(c, id, d, &mut crate::host(&mut Fs, stop), i, out, servo).await
}

pub async fn step_table<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    i: &StepInputs,
    out: &Path,
    servo: &str,
    stop: &AtomicBool,
) -> Stepped {
    of::step_table(c, id, d, &mut crate::host(&mut Fs, stop), i, out, servo).await
}

pub async fn step_window<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    i: &StepInputs,
    out: &Path,
    servo: &str,
    stop: &AtomicBool,
) -> Stepped {
    of::step_window(c, id, d, &mut crate::host(&mut Fs, stop), i, out, servo).await
}

pub async fn step_spins<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    i: &StepInputs,
    out: &Path,
    servo: &str,
    stop: &AtomicBool,
) -> Stepped {
    of::step_spins(c, id, d, &mut crate::host(&mut Fs, stop), i, out, servo).await
}

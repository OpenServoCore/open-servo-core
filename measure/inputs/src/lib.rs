//! The identification's user inputs, sans io: the servo profile (what a
//! servo model is) and the tuning policy (how to run it), their versioned
//! file formats, the migration of the files written before the schemas,
//! their validation, and [`resolve`], which turns the two into the absolute
//! values the rules and the steps read. The CLI reads the files through
//! this crate, and the web validates and resolves through the same code.

pub mod migrate;
pub mod nameplate;
pub mod policy;
pub mod profile;
pub mod read;
pub mod resolve;
pub mod tuning;
mod why;

pub use migrate::{PolicyIn, ProfileIn, Upgraded, upgrade};
pub use nameplate::Nameplate;
pub use policy::Policy;
pub use profile::ServoProfile;
pub use resolve::{BoardFacts, Report, Resolved, resolve};
pub use tuning::TuningPolicy;

/// A file's schema version: none is version 0, the files written before
/// the schemas. A file of another kind, or newer than this tool reads, is
/// refused naming its schema.
pub fn version(schema: Option<&str>, current: &str) -> Result<u32, String> {
    let Some(schema) = schema else {
        return Ok(0);
    };
    let (kind, own) = current
        .split_once('/')
        .expect("a schema is <kind>/<version>");
    let own: u32 = own.parse().expect("a schema's version is a number");
    match schema.split_once('/') {
        Some((k, v)) if k == kind => match v.parse::<u32>() {
            Ok(n) if n <= own => Ok(n),
            Ok(n) => Err(format!(
                "The file is {kind} version {n}, newer than this tool reads (up to {own}): update the tool to read it"
            )),
            Err(_) => Err(format!(
                "The file's schema {schema:?} names no version: {kind} files are {current}"
            )),
        },
        _ => Err(format!(
            "The file is not a {kind} file: its schema is {schema:?}, a {kind} file's is {current}"
        )),
    }
}

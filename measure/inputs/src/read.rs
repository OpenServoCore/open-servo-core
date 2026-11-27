//! A profile or a policy file to what [`crate::upgrade`] takes, at
//! whatever version it was written: from its text with `json`, or from
//! values already parsed (the web's) through [`profile_with`] and
//! [`policy_with`].

use serde::Deserialize;

use crate::migrate::{PolicyIn, ProfileIn, Source};
use crate::nameplate::Nameplate;
use crate::policy::PolicyFile;
use crate::profile::ServoProfile;
use crate::tuning::TuningPolicy;
use crate::{profile, tuning, version};

/// What a file's version is read from: its schema, and a version 0 file's
/// `source` block.
#[derive(Deserialize)]
pub struct Probe {
    #[serde(default)]
    pub schema: Option<String>,
    #[serde(default)]
    pub source: Option<Source>,
}

/// The file `probe` was read from, parsed as its version's type.
pub fn profile_with(
    probe: Probe,
    v0: impl FnOnce() -> Result<Nameplate, String>,
    v1: impl FnOnce() -> Result<ServoProfile, String>,
) -> Result<ProfileIn, String> {
    match version(probe.schema.as_deref(), profile::SCHEMA)? {
        0 => Ok(ProfileIn::V0 {
            nameplate: v0()?,
            source: probe.source.unwrap_or_default(),
        }),
        _ => Ok(ProfileIn::V1(v1()?)),
    }
}

/// `key` names a version 0 policy, which has no key of its own.
pub fn policy_with(
    probe: Probe,
    key: &str,
    v0: impl FnOnce() -> Result<PolicyFile, String>,
    v1: impl FnOnce() -> Result<TuningPolicy, String>,
) -> Result<PolicyIn, String> {
    match version(probe.schema.as_deref(), tuning::SCHEMA)? {
        0 => Ok(PolicyIn::V0 {
            policy: Box::new(v0()?.policy()?),
            source: probe.source.unwrap_or_default(),
            key: key.into(),
        }),
        _ => Ok(PolicyIn::V1(Box::new(v1()?))),
    }
}

#[cfg(feature = "json")]
fn parse<T: serde::de::DeserializeOwned>(text: &str) -> Result<T, String> {
    serde_json::from_str(text).map_err(|e| e.to_string())
}

#[cfg(feature = "json")]
pub fn profile(text: &str) -> Result<ProfileIn, String> {
    profile_with(parse(text)?, || parse(text), || parse(text))
}

/// `key` names a version 0 policy, which has no key of its own.
#[cfg(feature = "json")]
pub fn policy(text: &str, key: &str) -> Result<PolicyIn, String> {
    policy_with(parse(text)?, key, || parse(text), || parse(text))
}

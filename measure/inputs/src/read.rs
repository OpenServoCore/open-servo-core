//! A profile or a policy file's text to what [`crate::upgrade`] takes, at
//! whatever version it was written.

use serde::Deserialize;

use crate::migrate::{PolicyIn, ProfileIn, Source};
use crate::{profile, tuning, version};

#[derive(Deserialize)]
struct Probe {
    #[serde(default)]
    schema: Option<String>,
    #[serde(default)]
    source: Option<Source>,
}

fn probe(text: &str) -> Result<Probe, String> {
    serde_json::from_str(text).map_err(|e| e.to_string())
}

pub fn profile(text: &str) -> Result<ProfileIn, String> {
    let p = probe(text)?;
    match version(p.schema.as_deref(), profile::SCHEMA)? {
        0 => Ok(ProfileIn::V0 {
            nameplate: serde_json::from_str(text).map_err(|e| e.to_string())?,
            source: p.source.unwrap_or_default(),
        }),
        _ => Ok(ProfileIn::V1(
            serde_json::from_str(text).map_err(|e| e.to_string())?,
        )),
    }
}

/// `key` names a version 0 policy, which has no key of its own.
pub fn policy(text: &str, key: &str) -> Result<PolicyIn, String> {
    let p = probe(text)?;
    match version(p.schema.as_deref(), tuning::SCHEMA)? {
        0 => Ok(PolicyIn::V0 {
            policy: Box::new(serde_json::from_str(text).map_err(|e| e.to_string())?),
            source: p.source.unwrap_or_default(),
            key: key.into(),
        }),
        _ => Ok(PolicyIn::V1(Box::new(
            serde_json::from_str(text).map_err(|e| e.to_string())?,
        ))),
    }
}

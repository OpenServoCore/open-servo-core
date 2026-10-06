//! Embeds this checkout's HEAD as `OSC_HOST_GIT_SHA`, so run metadata names
//! the commit the tool was built from, not whatever repo the bench runs in.

use std::path::{Path, PathBuf};
use std::process::Command;

fn git(dir: &Path, args: &[&str]) -> Option<String> {
    let out = Command::new("git")
        .current_dir(dir)
        .args(args)
        .output()
        .ok()
        .filter(|o| o.status.success())?;
    String::from_utf8(out.stdout)
        .ok()
        .map(|s| s.trim().to_owned())
}

fn watch(dir: &Path, git_path: &str) -> Option<PathBuf> {
    git(dir, &["rev-parse", "--git-path", git_path]).map(|p| dir.join(p))
}

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    let dir = PathBuf::from(std::env::var_os("CARGO_MANIFEST_DIR").unwrap_or_default());
    let sha = git(&dir, &["rev-parse", "HEAD"]).unwrap_or_else(|| "unknown".into());
    println!("cargo:rustc-env=OSC_HOST_GIT_SHA={sha}");

    if let Some(head) = watch(&dir, "HEAD") {
        println!("cargo:rerun-if-changed={}", head.display());
    }
    let Some(branch) = git(&dir, &["symbolic-ref", "-q", "HEAD"]) else {
        return;
    };
    // A packed ref turns loose on the next commit: until the loose file
    // exists, watch the nearest directory that does.
    if let Some(r) = watch(&dir, &branch)
        && let Some(target) = r.ancestors().find(|p| p.exists())
    {
        println!("cargo:rerun-if-changed={}", target.display());
    }
    if let Some(packed) = watch(&dir, "packed-refs").filter(|p| p.exists()) {
        println!("cargo:rerun-if-changed={}", packed.display());
    }
}

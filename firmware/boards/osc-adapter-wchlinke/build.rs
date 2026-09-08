use std::path::PathBuf;
use std::{env, fs};

fn main() {
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());
    fs::copy("memory.x", out.join("memory.x")).unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");
    // Crash-record home (`_crash` in the CRASH region); the fragment ships
    // from osc-host-ch32's build.rs, which also adds its search path.
    println!("cargo:rustc-link-arg=-Tosc-crash.x");
}

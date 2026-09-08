use std::path::PathBuf;
use std::{env, fs};

fn main() {
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());
    // Ship the crash-record linker fragment (a board passes
    // `-Tosc-crash.x`); the link-search path propagates to the board link.
    fs::copy("osc-crash.x", out.join("osc-crash.x")).unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=osc-crash.x");
    println!("cargo:rerun-if-changed=build.rs");
}

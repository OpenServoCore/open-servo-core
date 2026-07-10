fn main() {
    let manifest_dir = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    let out_dir = std::env::var("OUT_DIR").unwrap();

    std::fs::copy(
        format!("{manifest_dir}/memory.x"),
        format!("{out_dir}/memory.x"),
    )
    .expect("copy memory.x");

    println!("cargo:rustc-link-search={out_dir}");
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rustc-link-arg=-Ttb-app.x");
    println!("cargo:rustc-link-arg=-Ttb-run-mode.x");
    // Saved-config slot bases (osc-native §9.4); the fragment ships from
    // osc-ch32's build.rs, which also adds its search path.
    println!("cargo:rustc-link-arg=-Tosc-config.x");

    if std::env::var("CARGO_FEATURE_DEFMT").is_ok() {
        println!("cargo:rustc-link-arg=-Tdefmt.x");
    }
}

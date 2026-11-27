fn main() {
    let manifest_dir = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    let out_dir = std::env::var("OUT_DIR").unwrap();

    std::fs::copy(
        format!("{manifest_dir}/memory.x"),
        format!("{out_dir}/memory.x"),
    )
    .expect("copy memory.x");

    // The board's facts (driver-pattern sec 2.6): `run!` takes the literal,
    // `app_version!` the compatibility key.
    let board = std::path::Path::new(&manifest_dir).join("../board.toml");
    println!("cargo:rerun-if-changed=../board.toml");
    let generated = osc_board_file::emit_board_config(&board).unwrap_or_else(|e| {
        eprintln!("{e}");
        std::process::exit(1)
    });
    std::fs::write(format!("{out_dir}/board_config.rs"), generated.config)
        .expect("write board_config.rs");
    std::fs::write(format!("{out_dir}/board_compat.rs"), generated.compat)
        .expect("write board_compat.rs");

    println!("cargo:rustc-link-search={out_dir}");
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rustc-link-arg=-Ttb-app.x");
    println!("cargo:rustc-link-arg=-Ttb-run-mode.x");
    // Saved-config slot bases (protocol sec 9.4); the fragment ships from
    // osc-servo-ch32's build.rs, which also adds its search path.
    println!("cargo:rustc-link-arg=-Tosc-config.x");

    if std::env::var("CARGO_FEATURE_DEFMT").is_ok() {
        println!("cargo:rustc-link-arg=-Tdefmt.x");
    }
}

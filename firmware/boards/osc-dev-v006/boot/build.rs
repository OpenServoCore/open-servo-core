fn main() {
    let manifest_dir = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    let out_dir = std::env::var("OUT_DIR").unwrap();

    std::fs::copy(
        format!("{manifest_dir}/memory.x"),
        format!("{out_dir}/memory.x"),
    )
    .expect("copy memory.x");

    // The compatibility key comes from the board file the app is built
    // from, so the bootloader accepts only that board's images.
    let board = std::path::Path::new(&manifest_dir).join("../board.toml");
    println!("cargo:rerun-if-changed=../board.toml");
    let generated = osc_board_file::emit_board_config(&board).unwrap_or_else(|e| {
        eprintln!("{e}");
        std::process::exit(1)
    });
    std::fs::write(format!("{out_dir}/board_compat.rs"), generated.compat)
        .expect("write board_compat.rs");

    println!("cargo:rustc-link-search={out_dir}");
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rustc-link-arg=-Ttb-boot.x");
    println!("cargo:rustc-link-arg=-Ttb-run-mode.x");
}

fn main() {
    let manifest_dir = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    let out_dir = std::env::var("OUT_DIR").unwrap();

    // The sim's dev-v006 board is the file that board's image is built from.
    let board = std::path::Path::new(&manifest_dir).join("../../boards/osc-dev-v006/board.toml");
    println!("cargo:rerun-if-changed=../../boards/osc-dev-v006/board.toml");
    let mirror = osc_board_file::emit_sim_mirror(&board, "DEV_V006").unwrap_or_else(|e| {
        eprintln!("{e}");
        std::process::exit(1)
    });
    std::fs::write(format!("{out_dir}/dev_v006_board.rs"), mirror)
        .expect("write dev_v006_board.rs");
}

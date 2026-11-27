use std::fmt::Write as _;
use std::path::Path;

/// The sim's boards are the files their images are built from: every
/// `firmware/boards/<board>/board.toml`, mirrored with the prefix its
/// directory names without `osc-` (`osc-dev-v006` -> `DEV_V006`), and listed
/// in `BOARD_FILES`.
fn main() {
    let manifest_dir = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    let out_dir = std::env::var("OUT_DIR").unwrap();

    let boards = Path::new(&manifest_dir).join("../../boards");
    let mut names: Vec<String> = std::fs::read_dir(&boards)
        .expect("read firmware/boards")
        .filter_map(|e| e.ok())
        .filter(|e| e.path().join("board.toml").is_file())
        .map(|e| e.file_name().to_string_lossy().into_owned())
        .collect();
    names.sort();

    let mut out = String::new();
    let mut list = String::from(
        "/// Every servo board file under `firmware/boards`, as its mirror seeds it.\n\
         pub const BOARD_FILES: &[BoardFile] = &[\n",
    );
    for name in &names {
        println!("cargo:rerun-if-changed=../../boards/{name}/board.toml");
        let prefix = name
            .strip_prefix("osc-")
            .unwrap_or(name)
            .to_uppercase()
            .replace('-', "_");
        let file = boards.join(name).join("board.toml");
        let mirror = osc_board_file::emit_sim_mirror(&file, &prefix).unwrap_or_else(|e| {
            eprintln!("{name}/{e}");
            std::process::exit(1)
        });
        out.push_str(&mirror);
        out.push('\n');
        let _ = writeln!(
            list,
            "    BoardFile {{ name: {name:?}, sense: {prefix}_SENSE, defaults: {prefix}_DEFAULTS, \
             protection: {prefix}_PROTECTION }},"
        );
    }
    list.push_str("];\n");
    out.push_str(&list);
    std::fs::write(format!("{out_dir}/board_files.rs"), out).expect("write board_files.rs");
}

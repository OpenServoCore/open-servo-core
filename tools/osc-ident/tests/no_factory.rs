//! The CLI has no path that wipes a servo's stored data: no source calls
//! the client's wipe or names its op. The library's own test says the same
//! of its sources and fails on one on the simulated bus.

#[test]
fn the_cli_has_no_factory_path() {
    let src = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
    let mut n = 0;
    for e in std::fs::read_dir(&src).expect("src") {
        let p = e.expect("entry").path();
        if p.extension().is_none_or(|x| x != "rs") {
            continue;
        }
        let s = std::fs::read_to_string(&p)
            .expect("read")
            .to_ascii_lowercase();
        for call in ["factory(", "mgmtop::factory", "factory as u8"] {
            assert!(!s.contains(call), "{} names {call}", p.display());
        }
        n += 1;
    }
    assert_eq!(n, 5);
}

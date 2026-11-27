//! Every servo board file seeds a CONFIG the table's own field rules take:
//! the boot seed is stamped past the rules, so a value they refuse would
//! boot and then refuse its own write-back.

use control_table::{Error, FieldDesc, RegisterFile};
use osc_integration::sim::{BOARD_FILES, BoardFile};
use osc_servo_core::regions::{CONFIG_BASE_ADDR, CONFIG_REGION_SIZE};
use osc_servo_core::{ControlTable, ControlTableCell, ProtectionSeed};

/// Writes each run of adjacent writable CONFIG fields back as seeded, one
/// write per run; the first refusal names its fields.
fn write_back(board: &BoardFile) -> Result<(), (Vec<&'static str>, Error)> {
    let t = ControlTableCell::new();
    t.seed_config_defaults(
        &board.defaults,
        &ProtectionSeed::from_sense(&board.protection, &board.sense),
    );
    let end = CONFIG_BASE_ADDR + CONFIG_REGION_SIZE;
    let mut fields: Vec<_> = ControlTable::FIELDS
        .iter()
        .filter(|d| d.writable && (CONFIG_BASE_ADDR..end).contains(&d.addr))
        .collect();
    fields.sort_by_key(|d| d.addr);
    let mut runs: Vec<Vec<&FieldDesc>> = Vec::new();
    for d in fields {
        match runs.last_mut() {
            Some(run) if run.last().is_some_and(|p| p.addr + p.width == d.addr) => run.push(d),
            _ => runs.push(vec![d]),
        }
    }
    for run in runs {
        let addr = run[0].addr;
        let len: u16 = run.iter().map(|d| d.width).sum();
        let bytes = t.read(addr, len).expect("in the table").to_vec();
        t.write(addr, &bytes)
            .map_err(|e| (run.iter().map(|d| d.name).collect(), e))?;
    }
    Ok(())
}

#[test]
fn every_board_file_seeds_a_table_the_rules_accept() {
    assert!(
        BOARD_FILES.iter().any(|b| b.name == "osc-dev-v006"),
        "the dev board's file is among them"
    );
    for board in BOARD_FILES {
        if let Err((fields, e)) = write_back(board) {
            panic!("{}: CONFIG {fields:?} refuses its seed: {e:?}", board.name);
        }
    }
}

#[test]
fn a_seed_the_rules_refuse_fails_its_write_back() {
    let dev = *BOARD_FILES
        .iter()
        .find(|b| b.name == "osc-dev-v006")
        .unwrap();
    for (defaults, field) in [
        (
            osc_servo_core::ConfigDefaults {
                id: 0,
                ..dev.defaults
            },
            "id",
        ),
        (
            osc_servo_core::ConfigDefaults {
                pos_min_phys_counts: 4095,
                ..dev.defaults
            },
            "pos_min_phys_counts",
        ),
    ] {
        let (fields, _) = write_back(&BoardFile { defaults, ..dev }).expect_err(field);
        assert!(fields.contains(&field), "{field}: {fields:?}");
    }
}

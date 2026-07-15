//! Walks `ControlTable::FIELDS` and prints a device-description JSON to
//! stdout. Checked-in output lives at `descriptors/osc-servo-v006.json`;
//! CI regenerates and diffs it to catch drift from the control table.

use control_table::descriptor::FieldKind;
use osc_servo_core::regions::ControlTable;
use serde::Serialize;

#[derive(Serialize)]
struct Variant {
    name: &'static str,
    value: u8,
}

#[derive(Serialize)]
struct Field {
    name: &'static str,
    addr: u16,
    width: u16,
    access: &'static str,
    kind: &'static str,
    #[serde(skip_serializing_if = "Option::is_none")]
    min: Option<i32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    max: Option<i32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    variants: Option<Vec<Variant>>,
}

#[derive(Serialize)]
struct Descriptor {
    format: u32,
    model: &'static str,
    model_number: u16,
    table_size: usize,
    generator: &'static str,
    fields: Vec<Field>,
}

fn build_descriptor() -> Descriptor {
    let fields = ControlTable::FIELDS
        .iter()
        .map(|d| {
            let (kind, variants) = match d.kind {
                FieldKind::UInt => ("uint", None),
                FieldKind::Int => ("int", None),
                FieldKind::Bool => ("bool", None),
                FieldKind::Bytes => ("bytes", None),
                FieldKind::Enum(vs) => (
                    "enum",
                    Some(
                        vs.iter()
                            .map(|v| Variant {
                                name: v.name,
                                value: v.value,
                            })
                            .collect(),
                    ),
                ),
            };
            Field {
                name: d.name,
                addr: d.addr,
                width: d.width,
                access: if d.writable { "rw" } else { "ro" },
                kind,
                min: d.min,
                max: d.max,
                variants,
            }
        })
        .collect();

    // Identity block is unseeded at this point in bringup, so model_number
    // reads 0 here just as it does on the wire from a fresh servo.
    let model_number = ControlTable::new().config.common.model_number;

    Descriptor {
        format: 1,
        model: "osc-servo-v006",
        model_number,
        table_size: core::mem::size_of::<ControlTable>(),
        generator: "cargo run -p table-export (firmware/lib)",
        fields,
    }
}

fn main() {
    let json = serde_json::to_string_pretty(&build_descriptor()).unwrap();
    println!("{json}");
}

#[cfg(test)]
mod tests {
    use osc_servo_core::regions::ControlTable;
    use serde_json::Value;

    #[test]
    fn descriptor_covers_all_fields_and_pins_id_bounds() {
        let json = serde_json::to_string(&super::build_descriptor()).unwrap();
        let value: Value = serde_json::from_str(&json).unwrap();

        let fields = value["fields"].as_array().unwrap();
        assert_eq!(fields.len(), ControlTable::FIELDS.len());

        let id = fields.iter().find(|f| f["name"] == "id").unwrap();
        assert_eq!(id["min"], 1);
        assert_eq!(id["max"], 249);
    }
}

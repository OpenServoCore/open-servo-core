//! Walks `ControlTable::FIELDS` and writes a device-description JSON to
//! `<root>/<model>/<major>.<minor>.json`: a descriptor names one table
//! layout, and patch never changes the table. Checked-in output lives under
//! `descriptors/`; CI regenerates and diffs it to catch drift from the
//! control table.

use control_table::descriptor::FieldKind;
use osc_protocol::models::{MODEL_OSC_SERVO, class_name, model_class};
use osc_protocol::version::unpack_version;
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
    class: &'static str,
    model_number: u16,
    firmware_major: u8,
    firmware_minor: u8,
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

    let (firmware_major, firmware_minor, _) = unpack_version(osc_servo_core::FIRMWARE_VERSION);
    Descriptor {
        format: 2,
        model: "osc-servo",
        class: class_name(model_class(MODEL_OSC_SERVO)),
        model_number: MODEL_OSC_SERVO,
        firmware_major,
        firmware_minor,
        table_size: core::mem::size_of::<ControlTable>(),
        generator: "cargo run -p table-export -- ../../descriptors (firmware/lib)",
        fields,
    }
}

fn main() {
    let root = std::env::args()
        .nth(1)
        .expect("usage: table-export <descriptors-root>");
    let d = build_descriptor();
    let dir = std::path::Path::new(&root).join(d.model);
    std::fs::create_dir_all(&dir).expect("create model dir");
    let mut json = serde_json::to_string_pretty(&d).unwrap();
    json.push('\n');
    let path = dir.join(format!("{}.{}.json", d.firmware_major, d.firmware_minor));
    std::fs::write(&path, json).expect("write descriptor");
}

#[cfg(test)]
mod tests {
    use osc_protocol::version::unpack_version;
    use osc_servo_core::regions::ControlTable;
    use serde_json::Value;

    #[test]
    fn descriptor_covers_all_fields_and_pins_id_bounds() {
        let json = serde_json::to_string(&super::build_descriptor()).unwrap();
        let value: Value = serde_json::from_str(&json).unwrap();

        assert_eq!(value["format"], 2);
        assert_eq!(value["model_number"], 0x0101);
        let (major, minor, _) = unpack_version(osc_servo_core::FIRMWARE_VERSION);
        assert_eq!(value["firmware_major"], major);
        assert_eq!(value["firmware_minor"], minor);

        let fields = value["fields"].as_array().unwrap();
        assert_eq!(fields.len(), ControlTable::FIELDS.len());

        let id = fields.iter().find(|f| f["name"] == "id").unwrap();
        assert_eq!(id["min"], 1);
        assert_eq!(id["max"], 249);
    }
}

use control_table::Enum;

#[derive(Enum)]
#[repr(u8)]
struct NotAnEnum {
    x: u8,
}

fn main() {}

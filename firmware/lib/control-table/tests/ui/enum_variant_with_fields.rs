use control_table::Enum;

#[derive(Enum)]
#[repr(u8)]
enum HasPayload {
    A,
    B(u8),
}

fn main() {}

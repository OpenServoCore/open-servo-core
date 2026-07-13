use control_table::Enum;

#[derive(Enum)]
#[repr(u16)]
enum ReprU16 {
    A = 0,
    B = 1,
}

fn main() {}

use control_table::Enum;

#[derive(Enum)]
#[repr(i8)]
enum ReprI8 {
    A = 0,
    B = 1,
}

fn main() {}

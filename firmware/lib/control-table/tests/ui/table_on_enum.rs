use control_table::Table;

#[repr(C)]
#[derive(Table)]
#[ct_table(max_sram = 1024)]
enum NotAStruct {
    A,
    B,
}

fn main() {}

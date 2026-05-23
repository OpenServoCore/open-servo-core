use control_table::Table;

#[repr(C)]
#[derive(Table)]
#[ct_table(max_sram = 1024)]
struct WrongFieldTy {
    #[ct_region]
    pub r: u32,
}

fn main() {}

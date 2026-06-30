use mockall::mock;

use crate::traits::dxl::EdgeDma;

mock! {
    pub EdgeDma {}
    impl EdgeDma for EdgeDma {
        fn remaining(&self) -> u16;
    }
}

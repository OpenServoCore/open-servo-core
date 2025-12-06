use core::cell::UnsafeCell;

const N: usize = 5;

#[repr(transparent)]
pub struct AdcDmaBuf(UnsafeCell<[u16; N]>);

unsafe impl Sync for AdcDmaBuf {}

static BUF: AdcDmaBuf = AdcDmaBuf(UnsafeCell::new([0; N]));

impl AdcDmaBuf {
    pub fn ptr(&self) -> *mut u16 {
        self.0.get().cast()
    }

    pub fn snapshot(&self) -> [u16; N] {
        let mut out = [0u16; N];
        for i in 0..N {
            unsafe {
                out[i] = core::ptr::read_volatile(self.0.get().cast::<u16>().add(i));
            }
        }
        out
    }
}

pub fn dma_target_ptr() -> *mut u16 {
    BUF.ptr()
}

pub fn read_block() -> [u16; N] {
    BUF.snapshot()
}
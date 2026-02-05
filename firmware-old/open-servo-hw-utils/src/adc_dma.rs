use core::cell::UnsafeCell;

/// Const-generic ADC DMA buffer.
/// Board crate instantiates with the appropriate channel count.
#[repr(transparent)]
pub struct AdcDmaBuf<const N: usize>(UnsafeCell<[u16; N]>);

unsafe impl<const N: usize> Sync for AdcDmaBuf<N> {}

impl<const N: usize> AdcDmaBuf<N> {
    pub const fn new() -> Self {
        Self(UnsafeCell::new([0; N]))
    }

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

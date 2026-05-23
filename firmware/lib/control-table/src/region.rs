pub trait Region {}

/// Impls may rely on the caller holding the region's single-writer guarantee.
pub trait RegionStorage<R: Region> {
    fn with<T>(&self, f: impl FnOnce(&R) -> T) -> T;
    fn with_mut<T>(&self, f: impl FnOnce(&mut R) -> T) -> T;
}

#[cfg(feature = "sync-unsafe-cell")]
impl<R: Region> RegionStorage<R> for core::cell::SyncUnsafeCell<R> {
    fn with<T>(&self, f: impl FnOnce(&R) -> T) -> T {
        // SAFETY: caller upholds RegionStorage's single-writer contract.
        unsafe { f(&*self.get()) }
    }
    fn with_mut<T>(&self, f: impl FnOnce(&mut R) -> T) -> T {
        // SAFETY: caller upholds RegionStorage's single-writer contract.
        unsafe { f(&mut *self.get()) }
    }
}

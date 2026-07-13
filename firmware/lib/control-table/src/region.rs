pub trait Region {}

/// Impls may rely on the caller holding the region's single-writer guarantee.
pub trait RegionStorage<R: Region> {
    fn with<T>(&self, f: impl FnOnce(&R) -> T) -> T;
    fn with_mut<T>(&self, f: impl FnOnce(&mut R) -> T) -> T;
}

/// Raw-pointer access for Router's memcpy dispatch path.
///
/// # Safety
/// Impls must return a valid, non-null, aligned pointer to `R` that remains
/// valid for the lifetime of `&self`. Lock-based wrappers must not impl this
/// trait -- the pointer would outlive any lock scope.
pub unsafe trait RegionStorageRaw<R: Region>: RegionStorage<R> {
    fn region_ptr(&self) -> *mut R;
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

// SAFETY: `SyncUnsafeCell::get` returns a valid aligned pointer to its inner
// `R`, valid for the cell's lifetime.
#[cfg(feature = "sync-unsafe-cell")]
unsafe impl<R: Region> RegionStorageRaw<R> for core::cell::SyncUnsafeCell<R> {
    fn region_ptr(&self) -> *mut R {
        self.get()
    }
}

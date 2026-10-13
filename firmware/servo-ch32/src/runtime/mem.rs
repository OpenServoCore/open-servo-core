//! The `memcpy` and `memset` symbols, in place of compiler_builtins' word
//! routines: LLVM lowers the aggregate moves that build the statics to
//! these calls, and only bringup may make them (`.github/check-no-mem.sh`
//! holds every other path to none). Byte loops are the smallest form;
//! volatile keeps each a loop, which would otherwise compile back into a
//! call to itself.

use core::ffi::c_void;

/// # Safety
/// The C contract: `dst` and `src` valid for `n` bytes and not overlapping.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn memcpy(dst: *mut c_void, src: *const c_void, n: usize) -> *mut c_void {
    let (d, s) = (dst.cast::<u8>(), src.cast::<u8>());
    for i in 0..n {
        // SAFETY: the C contract.
        unsafe { d.add(i).write_volatile(s.add(i).read()) };
    }
    dst
}

/// # Safety
/// The C contract: `dst` valid for `n` bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn memset(dst: *mut c_void, c: i32, n: usize) -> *mut c_void {
    let d = dst.cast::<u8>();
    for i in 0..n {
        // SAFETY: the C contract.
        unsafe { d.add(i).write_volatile(c as u8) };
    }
    dst
}

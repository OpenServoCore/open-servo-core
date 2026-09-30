//! Stack high-water mark. `paint` fills the free stack with a pattern before
//! bringup runs; the main loop walks the painted span a few words per pass
//! up from the bottom, and the first word that lost the pattern marks the
//! deepest the stack has reached.

const PAINT: u32 = 0xA5C3_5A3C;
const WORDS_PER_PASS: usize = 4;

pub struct Scan {
    bottom: *const u32,
    at: usize,
    /// Free words under the deepest stack seen: no walk needs to pass it.
    free: usize,
}

impl Scan {
    /// # Safety
    /// `words` words from `bottom` stay readable for the scan's life.
    pub unsafe fn new(bottom: *const u32, words: usize) -> Self {
        Self {
            bottom,
            at: 0,
            free: words,
        }
    }

    /// Checks the next few words. A walk that reaches a word without the
    /// pattern, or the smallest free span already seen, returns the free
    /// bytes and restarts from the bottom.
    pub fn step(&mut self) -> Option<u16> {
        for _ in 0..WORDS_PER_PASS {
            // SAFETY: `at < free <= words`, readable per `new`.
            if self.at == self.free || unsafe { self.bottom.add(self.at).read_volatile() } != PAINT
            {
                self.free = self.at;
                self.at = 0;
                return Some((self.free * 4) as u16);
            }
            self.at += 1;
        }
        None
    }
}

/// Paints every word from the end of `.bss` up to `MARGIN` bytes under the
/// stack pointer and returns the scan over them. Runs first in `__run`,
/// before any interrupt is live.
#[cfg(target_arch = "riscv32")]
#[inline(always)]
pub fn paint() -> Scan {
    /// Room for the painting frame itself.
    const MARGIN: usize = 64;
    unsafe extern "C" {
        static _ebss: u8;
    }
    let bottom = (&raw const _ebss as usize).next_multiple_of(4);
    let sp: usize;
    // SAFETY: reads sp, nothing else.
    unsafe { core::arch::asm!("mv {}, sp", out(reg) sp) };
    let words = (sp - MARGIN).saturating_sub(bottom) / 4;
    let p = bottom as *mut u32;
    for i in 0..words {
        // SAFETY: between the end of `.bss` and the live stack: RAM nothing
        // owns until the stack grows into it.
        unsafe { p.add(i).write_volatile(PAINT) };
    }
    // SAFETY: the span is RAM for the program's life.
    unsafe { Scan::new(p, words) }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use std::vec;
    use std::vec::Vec;

    use super::*;

    /// Walks until the scan completes `n` times; the reported free bytes.
    fn walks(words: &[u32], n: usize) -> Vec<u16> {
        let mut scan = unsafe { Scan::new(words.as_ptr(), words.len()) };
        core::iter::from_fn(|| Some(scan.step()))
            .take(10_000)
            .flatten()
            .take(n)
            .collect()
    }

    #[test]
    fn untouched_stack_is_all_free() {
        assert_eq!(walks(&[PAINT; 37], 2), [148, 148]);
    }

    #[test]
    fn partly_touched_stack_stops_at_the_first_touched_word() {
        let mut words = vec![PAINT; 40];
        words[30] = 0;
        words[35] = 0;
        assert_eq!(walks(&words, 2), [120, 120]);
    }

    #[test]
    fn stack_touched_at_the_bottom_has_nothing_free() {
        let mut words = vec![PAINT; 40];
        words[0] = 0;
        assert_eq!(walks(&words, 1), [0]);
    }

    #[test]
    fn free_only_shrinks() {
        let mut words = vec![PAINT; 40];
        let p = words.as_mut_ptr();
        let mut scan = unsafe { Scan::new(p, words.len()) };
        let mut next = || core::iter::from_fn(|| Some(scan.step())).flatten().next();
        unsafe { p.add(20).write_volatile(0) };
        assert_eq!(next(), Some(80));
        unsafe { p.add(20).write_volatile(PAINT) };
        assert_eq!(next(), Some(80));
        unsafe { p.add(10).write_volatile(0) };
        assert_eq!(next(), Some(40));
    }
}

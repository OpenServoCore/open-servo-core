//! Value type shared by the frame emitters: the write-failure code.

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum WriteError {
    Overflow,
    Invalid,
}

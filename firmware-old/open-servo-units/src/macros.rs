//! Internal macros for implementing unit type operations.

/// Implements basic saturating arithmetic operations for unit types.
macro_rules! impl_unit_int_ops {
    ($name:ident) => {
        impl ::core::ops::Add for $name {
            type Output = Self;
            #[inline]
            fn add(self, rhs: Self) -> Self {
                Self(self.0.saturating_add(rhs.0))
            }
        }

        impl ::core::ops::Sub for $name {
            type Output = Self;
            #[inline]
            fn sub(self, rhs: Self) -> Self {
                Self(self.0.saturating_sub(rhs.0))
            }
        }

        impl ::core::ops::Neg for $name {
            type Output = Self;
            #[inline]
            fn neg(self) -> Self {
                Self(self.0.saturating_neg())
            }
        }

        impl ::core::ops::Mul<i16> for $name {
            type Output = Self;
            #[inline]
            fn mul(self, rhs: i16) -> Self {
                Self(self.0.saturating_mul(rhs))
            }
        }

        impl ::core::ops::Div<i16> for $name {
            type Output = Self;
            #[inline]
            fn div(self, rhs: i16) -> Self {
                Self(self.0 / rhs)
            }
        }
    };
}

pub(crate) use impl_unit_int_ops;

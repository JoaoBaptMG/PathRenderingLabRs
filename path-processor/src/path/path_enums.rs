//------------------------------------------------------------------------------
// path_enums.rs
//------------------------------------------------------------------------------
// Provides the enums to configure all the process of path splitting, stroke
// generation and compiling
//------------------------------------------------------------------------------

use crate::derive_more::*;

#[derive(Clone, Copy, Display, Debug)]
pub enum FillRule { EvenOdd, NonZero }

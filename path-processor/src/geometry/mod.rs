//--------------------------------------------------------------------
// geometry.rs
//--------------------------------------------------------------------
// Provides the geometric constructs used in the project
//--------------------------------------------------------------------

mod coord_utils;
mod rect;
mod vec2;
mod vec4;
mod polygon;

pub type Coord = f64;
pub use std::f64 as CoordM;

pub use coord_utils::*;
pub use rect::*;
pub use vec2::*;
pub use vec4::*;
pub use polygon::*;

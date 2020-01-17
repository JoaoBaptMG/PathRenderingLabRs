//--------------------------------------------------------------------
// lib.rs
//--------------------------------------------------------------------
// Declaration of all modules occurs here
//--------------------------------------------------------------------

extern crate approx;
extern crate derive_more;
extern crate ordered_float;
extern crate roots;
extern crate arrayvec;

mod geometry;
mod path;
mod curve;
mod union_find;
mod pause;
mod vec_utils;
mod merge;

pub use geometry::{Coord, Vec2};
pub use path::*;
pub use curve::*;

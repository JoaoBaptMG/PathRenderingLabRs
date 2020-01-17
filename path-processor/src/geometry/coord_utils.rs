//--------------------------------------------------------------------
// rect.rs
//--------------------------------------------------------------------
// Provides utilities to work with the coordinate class
//--------------------------------------------------------------------

use super::*;
use CoordM::consts::PI;

pub const TWO_PI: Coord = 2.0 * PI;
pub const EPSILON: Coord = 1.0 / 32768.0;
pub const EPSILON2: Coord = EPSILON * EPSILON;

// Some utility functions for Coord
// trait used only for implementation
pub trait Geometry where Self: Sized {
    fn wrap_angle(self) -> Self;
    fn wrap_angle_360(self, ccw: bool) -> Self;

    fn wrap_angle_360_ccw(self) -> Self;
    fn wrap_angle_360_cw(self) -> Self;

    fn roughly_zero(self) -> bool;
    fn roughly_zero_squared(self) -> bool;
    fn roughly_equals(self, other: Self) -> bool;
    fn roughly_equals_squared(self, other: Self) -> bool;
}

impl Geometry for Coord {
    fn wrap_angle(self) -> Coord { self + TWO_PI * (-self / TWO_PI).round() }
    fn wrap_angle_360(self, ccw: bool) -> Coord {
        if ccw { self.wrap_angle_360_ccw() }
        else { self.wrap_angle_360_cw() }
    }

    fn wrap_angle_360_ccw(self) -> Coord { self - TWO_PI * (self / TWO_PI).ceil() }
    fn wrap_angle_360_cw(self) -> Coord { self - TWO_PI * (self / TWO_PI).floor() }

    fn roughly_zero(self) -> bool { self > -EPSILON && self < EPSILON }
    fn roughly_zero_squared(self) -> bool { self > -EPSILON2 && self < EPSILON2 }
    fn roughly_equals(self, other: Self) -> bool { (self - other).roughly_zero() }
    fn roughly_equals_squared(self, other: Self) -> bool { (self - other).roughly_zero_squared() }
}

pub fn inside01(t: Coord) -> bool { t >= 0.0 && t <= 1.0 }

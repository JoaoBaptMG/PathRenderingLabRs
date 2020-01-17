//--------------------------------------------------------------------
// vec2.rs
//--------------------------------------------------------------------
// Provides a two-element vector class
//--------------------------------------------------------------------

use crate::derive_more::*;

use super::*;
use std::cmp::Ordering;

// Vec2
#[derive(Copy, Clone, Add, Sub, Mul, Div, AddAssign, SubAssign, Neg, PartialEq,
    MulAssign, DivAssign, From, Into, Display, Constructor)]
#[display(fmt = "({},{})", x, y)]
pub struct Vec2 { pub x: Coord, pub y: Coord }

impl Vec2 {
    pub fn zero() -> Vec2 { Vec2 { x: 0.0, y: 0.0 } }
    pub fn from_angle(angle: Coord) -> Vec2 { Vec2::new(angle.cos(), angle.sin()) }

    pub fn dot(&self, other: Vec2) -> Coord { self.x * other.x + self.y * other.y }
    pub fn cross(&self, other: Vec2) -> Coord { self.x * other.y - self.y * other.x }

    pub fn length_sq(&self) -> Coord { self.dot(*self) }
    pub fn length(&self) -> Coord { self.length_sq().sqrt() }
    pub fn normalized(&self) -> Vec2 { *self / self.length() }

    pub fn rot_scale(&self, other: Vec2) -> Vec2 {
        let x = self.x * other.x - self.y * other.y;
        let y = self.x * other.y + self.y * other.x;
        Vec2 { x, y }
    }

    pub fn rotate(&self, other: Vec2) -> Vec2 { self.rot_scale(other.normalized()) }
    pub fn rotate_by_angle(&self, angle: Coord) -> Vec2 { self.rot_scale(Vec2::from_angle(angle)) }
    pub fn ccw_perpendicular(&self) -> Vec2 { Vec2 { x: -self.y, y: self.x } }
    pub fn cw_perpendicular(&self) -> Vec2 { -self.ccw_perpendicular() }

    pub fn angle(&self) -> Coord { self.y.atan2(self.x) }
    pub fn angle_facing(&self, other: Vec2) -> Coord { (other - *self).angle() }
    pub fn angle_between(&self, other: Vec2) -> Coord { self.cross(other).atan2(self.dot(other)) }

    pub fn roughly_zero(&self) -> bool { self.length_sq().roughly_zero_squared() }
    pub fn roughly_equals(&self, other: Vec2) -> bool { (*self - other).length_sq().roughly_zero_squared() } 
}

// Implement scalar * mul as required
impl core::ops::Mul<Vec2> for Coord {
    type Output = Vec2;
    fn mul(self, rhs: Vec2) -> Vec2 { Vec2::new(self * rhs.x, self * rhs.y) }
}

pub fn canonical(a: &Vec2, b: &Vec2) -> Ordering {
    if a.y == b.y { b.x.partial_cmp(&a.x).unwrap() }
    else { a.y.partial_cmp(&b.y).unwrap() }
}

impl std::fmt::Debug for Vec2 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self)
    }
}


//--------------------------------------------------------------------
// vec4.rs
//--------------------------------------------------------------------
// Provides a four-element vector class
//--------------------------------------------------------------------

use crate::derive_more::*;

use super::*;

// Vec4
#[derive(Copy, Clone, Add, Sub, Mul, Div, AddAssign, SubAssign, Neg, PartialEq,
    MulAssign, DivAssign, From, Into, Display, Constructor)]
#[display(fmt = "({},{},{},{})", x, y, z, w)]
pub struct Vec4 { pub x: Coord, pub y: Coord, pub z: Coord, pub w: Coord }

impl Vec4 {
    pub fn zero() -> Vec4 { Vec4 { x: 0.0, y: 0.0, z: 0.0, w: 0.0 } }
    pub fn dot(&self, other: Vec4) -> Coord {
        self.x * other.x + self.y * other.y + self.z * other.z + self.w * other.w
    }
    pub fn length_sq(&self) -> Coord { self.dot(*self) }
    pub fn length(&self) -> Coord { self.length_sq().sqrt() }
    pub fn normalized(&self) -> Vec4 { *self / self.length() }
}

// Implement scalar * mul as required
impl core::ops::Mul<Vec4> for Coord {
    type Output = Vec4;
    fn mul(self, rhs: Vec4) -> Vec4 { Vec4::new(self * rhs.x, self * rhs.y, self * rhs.z, self * rhs.w) }
}

impl std::fmt::Debug for Vec4 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self)
    }
}

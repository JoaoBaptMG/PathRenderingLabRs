//--------------------------------------------------------------------
// line.rs
//--------------------------------------------------------------------
// Represents a line segment
//--------------------------------------------------------------------

use crate::derive_more::*;
use crate::geometry::*;
use crate::arrayvec::*;
use super::*;

// The line structure
#[derive(Display)]
#[display(fmt = "Line({},{})", a, b)]
pub struct Line { pub a: Vec2, pub b: Vec2 }

// The myriad of different functions present here
impl Line {
    pub fn at(&self, t: Coord) -> Vec2 { (1.0-t) * self.a + t * self.b }

    pub fn derivative(&self) -> Line { Line { a: self.b - self.a, b: self.b - self.a } }

    pub fn subcurve(&self, l: Coord, r: Coord) -> Line {
        Line { a: self.at(l), b: self.at(r) }
    }

    pub fn reverse(&self) -> Line { Line {a: self.b, b: self.a } }

    pub fn winding(&self) -> Coord { self.a.cross(self.b) }

    pub fn angle_key(&self) -> AngleKey { AngleKey::new(self.a.angle_facing(self.b), 0.0, 0.0) }

    pub fn intersection_x(&self, x: Coord) -> roots::Roots<Coord> {
        roots::find_roots_linear(self.b.x - self.a.x, self.a.x - x)
    }

    pub fn intersection_y(&self, y: Coord) -> roots::Roots<Coord> {
        roots::find_roots_linear(self.b.y - self.a.y, self.a.y - y)
    }

    pub fn intersection_seg(&self, v1: Vec2, v2: Vec2) -> roots::Roots<Coord> {
        let dv = v2 - v1;
        roots::find_roots_linear(dv.cross(self.b - self.a), dv.cross(self.a - v1))
    }

    pub fn entry_tangent(&self) -> Vec2 { (self.b-self.a).normalized() }
    
    pub fn exit_tangent(&self) -> Vec2 { (self.b-self.a).normalized() }

    pub fn enclosing_polygon(&self) -> ArrayVec<[Vec2; MAX_POLYGON_VERTICES]> {
        [self.a, self.b].into_iter().copied().collect()
    }

    pub fn critical_points(&self) -> ArrayVec<[Coord; MAX_CRITICAL_POINTS]> { 
        [0.0, 1.0].iter().copied().collect()
    }
}

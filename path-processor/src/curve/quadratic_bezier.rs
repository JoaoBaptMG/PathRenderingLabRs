//--------------------------------------------------------------------
// quadratic-bezier.rs
//--------------------------------------------------------------------
// Represents a quadratic Bézier curve
//--------------------------------------------------------------------

use crate::derive_more::*;

use crate::geometry::*;
use super::line::Line;
use crate::arrayvec::*;
use super::*;
use crate::vec_utils::*;

// The quadratic bezier structure
#[derive(Display)]
#[display(fmt = "QuadraticBezier({},{},{})", a, b, c)]
pub struct QuadraticBezier { pub a: Vec2, pub b: Vec2, pub c: Vec2 }

// The myriad of different functions present here
impl QuadraticBezier {
    pub fn at(&self, t: Coord) -> Vec2 { 
        let ct = 1.0 - t;
        ct * ct * self.a + 2.0 * ct * t * self.b + t * t * self.c
    }

    pub fn derivative(&self) -> Line { Line { a: 2.0 * (self.b - self.a), b: 2.0 * (self.c - self.b) } }

    pub fn subcurve(&self, l: Coord, r: Coord) -> QuadraticBezier {
        // The endpoints
        let a = self.at(l);
        let c = self.at(r);

        // The control point
        let cl = 1.0 - l;
        let cr = 1.0 - r;
        let b = cl*cr * self.a + (l*cr + r*cl) * self.b + l*r * self.c;

        QuadraticBezier { a, b, c }
    }

    pub fn reverse(&self) -> QuadraticBezier { QuadraticBezier { a: self.c, b: self.b, c: self.a } }

    pub fn winding(&self) -> Coord { 
        (2.0 * self.a.cross(self.b) + 2.0 * self.b.cross(self.c) + self.a.cross(self.c)) / 3.0
    }

    pub fn angle_key(&self) -> AngleKey {
        let dv1 = self.b - self.a;
        let dv2 = self.c - self.b;

        // If dv1 is zero the following angles will fall apart, so we take the limit
        if dv1.roughly_zero() { self.derivative().angle_key() }
        else {
            let dt = dv1.cross(dv2 - dv1) / dv1.length_sq();
            let ddt = -2.0 * dv1.dot(dv2 - dv1) * dt / dv1.length_sq();
            AngleKey::new(dv1.angle(), dt, ddt)
        }
    }

    pub fn intersection_x(&self, x: Coord) -> roots::Roots<Coord> {
        roots::find_roots_quadratic(self.a.x - 2.0*self.b.x + self.c.x, 2.0 * (self.b.x - self.a.x), self.a.x - x)
    }

    pub fn intersection_y(&self, y: Coord) -> roots::Roots<Coord> {
        roots::find_roots_quadratic(self.a.y - 2.0*self.b.y + self.c.y, 2.0 * (self.b.y - self.a.y), self.a.y - y)
    }

    pub fn intersection_seg(&self, v1: Vec2, v2: Vec2) -> roots::Roots<Coord> {
        let dv = v1 - v2;
        roots::find_roots_quadratic(dv.cross(self.a - 2.0*self.b + self.c),
            2.0 * dv.cross(self.b - self.a), dv.cross(self.a - v1))
    }

    pub fn entry_tangent(&self) -> Vec2 { (self.b-self.a).normalized() }
    
    pub fn exit_tangent(&self) -> Vec2 { (self.c-self.b).normalized() }

    pub fn enclosing_polygon(&self) -> ArrayVec<[Vec2; MAX_POLYGON_VERTICES]> {
        [self.a, self.b, self.c].into_iter().copied().collect()
    }

    pub fn critical_points(&self) -> ArrayVec<[Coord; MAX_CRITICAL_POINTS]> {
        let dd = self.derivative();
        let tx = dd.a.x / (dd.a.x - dd.b.x);
        let ty = dd.a.y / (dd.a.y - dd.b.y);

        let mut v = ArrayVec::new();
        v.push(0.0);
        v.push(tx);
        v.push(ty);
        v.push(1.0);
        v.retain(|&mut t| inside01(t));
        v.sort_unstable_by(|a: &Coord, b: &Coord| a.partial_cmp(b).unwrap());
        arrayvec_dedup(&mut v);
        v
    }
}

//--------------------------------------------------------------------
// quadratic-bezier.rs
//--------------------------------------------------------------------
// Represents a quadratic Bézier curve
//--------------------------------------------------------------------

use crate::derive_more::*;

use crate::roots;
use crate::geometry::*;
use super::quadratic_bezier::QuadraticBezier;
use crate::arrayvec::*;
use crate::vec_utils::*;
use super::*;

// The cubic bezier structure
#[derive(Display)]
#[display(fmt = "CubicBezier({},{},{},{})", a, b, c, d)]
pub struct CubicBezier { pub a: Vec2, pub b: Vec2, pub c: Vec2, pub d: Vec2 }

// The myriad of different functions present here
impl CubicBezier {
    pub fn at(&self, t: Coord) -> Vec2 { 
        let ct = 1.0 - t;
        ct * ct * ct * self.a + 3.0 * ct * ct * t * self.b
            + 3.0 * ct * t * t * self.c + t * t * t * self.d
    }

    pub fn derivative(&self) -> QuadraticBezier {
        let a = 3.0 * (self.b - self.a);
        let b = 3.0 * (self.c - self.b);
        let c = 3.0 * (self.d - self.c);
        QuadraticBezier { a, b, c }
    }

    pub fn subcurve(&self, l: Coord, r: Coord) -> CubicBezier {
        // The endpoints
        let a = self.at(l);
        let d = self.at(r);

        // The control points
        let dd = self.derivative();
        let d1 = dd.at(l) * (r-l);
        let d2 = dd.at(r) * (r-l);

        let b = d1 / 3.0 + a;
        let c = d - d2 / 3.0;

        CubicBezier { a, b, c, d }
    }

    pub fn reverse(&self) -> CubicBezier {
        CubicBezier { a: self.d, b: self.c, c: self.b, d: self.a }
    }

    pub fn winding(&self) -> Coord { 
        (6.0 * self.a.cross(self.b) + 3.0 * self.a.cross(self.c) + self.a.cross(self.d) +
            3.0 * self.b.cross(self.c) + 3.0 * self.b.cross(self.d) + 6.0 * self.c.cross(self.d)) / 10.0
    }

    pub fn angle_key(&self) -> AngleKey {
        let dv1 = self.b - self.a;
        let dv2 = self.c - self.b;
        let dv3 = self.d - self.c;

        // If dv1 is zero, the following angles will fall apart, so we take the limit
        if dv1.roughly_zero() { self.derivative().angle_key() }
        else {
            let dt = 2.0 * dv1.cross(dv2 - dv1) / dv1.length_sq();
            let ddt = (2.0 * dv1.cross(dv3 - 2.0 * dv2 + dv1) - 8.0 * dv1.dot(dv2 - dv1) * dt) / dv1.length_sq();
            AngleKey::new(dv1.angle(), dt, ddt)
        }
    }

    pub fn intersection_x(&self, x: Coord) -> roots::Roots<Coord> {
        roots::find_roots_cubic(-self.a.x + 3.0 * self.b.x - 3.0 * self.c.x + self.d.x,
            3.0 * (self.a.x - 2.0 * self.b.x + self.c.x), 3.0 * (self.b.x - self.a.x), self.a.x - x)
    }

    pub fn intersection_y(&self, y: Coord) -> roots::Roots<Coord> {
        roots::find_roots_cubic(-self.a.y + 3.0 * self.b.y - 3.0 * self.c.y + self.d.y,
            3.0 * (self.a.y - 2.0 * self.b.y + self.c.y), 3.0 * (self.b.y - self.a.y), self.a.y - y)
    }

    pub fn intersection_seg(&self, v1: Vec2, v2: Vec2) -> roots::Roots<Coord> {
        let dv = v1 - v2;
        roots::find_roots_cubic(dv.cross(-self.a + 3.0 * self.b - 3.0 * self.c + self.d),
            3.0 * dv.cross(self.a - 2.0 * self.b + self.c),
            3.0 * dv.cross(self.b - self.a), dv.cross(self.a - v1))
    }
    
    pub fn entry_tangent(&self) -> Vec2 {
        if self.b.roughly_equals(self.a) { self.derivative().entry_tangent() }
        else { (self.b-self.a).normalized() }
    }
    
    pub fn exit_tangent(&self) -> Vec2 {
        if self.d.roughly_equals(self.c) { self.derivative().exit_tangent() }
        else { (self.d-self.c).normalized() }
    }

    pub fn enclosing_polygon(&self) -> ArrayVec<[Vec2; MAX_POLYGON_VERTICES]> {
        [self.a, self.b, self.c, self.d].into_iter().copied().collect()
    }

    pub fn critical_points(&self) -> ArrayVec<[Coord; MAX_CRITICAL_POINTS]> {
        let dd = self.derivative();
        let tx = roots::find_roots_quadratic(dd.a.x - 2.0 * dd.b.x + dd.c.x, 2.0 * (dd.b.x - dd.a.x), dd.a.x);
        let ty = roots::find_roots_quadratic(dd.a.y - 2.0 * dd.b.y + dd.c.y, 2.0 * (dd.b.y - dd.a.y), dd.a.y);

        let mut v = ArrayVec::new();
        v.push(0.0);
        v.extend(tx.as_ref().iter().copied());
        v.extend(ty.as_ref().iter().copied());
        v.push(1.0);
        v.retain(|&mut t| inside01(t));
        v.sort_unstable_by(|a: &Coord, b: &Coord| a.partial_cmp(b).unwrap());
        arrayvec_dedup(&mut v);
        v
    }
}

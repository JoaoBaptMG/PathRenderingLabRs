//--------------------------------------------------------------------
// quadratic-bezier.rs
//--------------------------------------------------------------------
// Represents an elliptic arc
//--------------------------------------------------------------------

use crate::derive_more::*;

use crate::geometry::*;
use CoordM::{INFINITY, consts::*};
use crate::arrayvec::*;
use super::*;
use crate::vec_utils::*;

// The elliptic arc structure
#[derive(Copy, Clone, Display)]
#[display(fmt = "EllipticArc(center = {}, radii = {}, rotation = {}, t1 = {}, dt = {})",
    center, radii, "crot.angle().to_degrees()", "t1.to_degrees()", "dt.to_degrees()")]
pub struct EllipticArc { pub center: Vec2, pub radii: Vec2, pub crot: Vec2, pub t1: Coord, pub dt: Coord }

impl EllipticArc {
    pub fn local_to_global(&self, p: Vec2) -> Vec2 { self.center + self.crot.rot_scale(p) }
    fn delta_at(&self, t: Coord) -> Vec2 {
        let th = self.t1 * t + self.dt;
        Vec2::new(self.radii.x * th.cos(), self.radii.y * th.sin())
    }

    pub fn lesser_angle(&self) -> Coord { self.t1.min(self.t1 + self.dt) }
    pub fn greater_angle(&self) -> Coord { self.t1.max(self.t1 + self.dt) }

    pub fn angle_to_param(&self, theta: Coord) -> Coord {
        let theta = theta.wrap_angle();

        // Test the angle and up to two double turns before and after
        let mut i = -2.0;
        while i <= 2.0 {
            let cand = theta + i * 2.0 * PI;
            if self.lesser_angle() <= cand && cand <= self.greater_angle() {
                return (cand - self.t1) / self.dt;
            }
            i += 1.0;
        }

        INFINITY
    }

    pub fn at(&self, t: Coord) -> Vec2 { self.local_to_global(self.delta_at(t)) }

    pub fn derivative(&self) -> EllipticArc {
        let center = Vec2::zero();
        let radii = self.dt.abs() * self.radii;
        let crot = self.crot;
        let t1 = self.t1 + FRAC_PI_2.copysign(self.dt);
        let dt = self.dt;
        EllipticArc { center, radii, crot, t1, dt }
    }

    pub fn subcurve(&self, l: Coord, r: Coord) -> EllipticArc {
        EllipticArc { t1: self.t1 + l * self.dt, dt: (r - l) * self.dt, ..*self }
    }

    pub fn reverse(&self) -> EllipticArc {
        EllipticArc { t1: self.t1 + self.dt, dt: -self.dt, ..*self }
    }

    pub fn critical_points(&self) -> ArrayVec<[Coord; MAX_CRITICAL_POINTS]> {
        let ax = (-self.radii.y * self.crot.y).atan2(self.radii.x * self.crot.x);
        let ay = (self.radii.y * self.crot.x).atan2(self.radii.x * self.crot.y);

        let mut v = ArrayVec::new();
        v.push(0.0);
        v.push(self.angle_to_param(ax));
        v.push(self.angle_to_param(ax + PI));
        v.push(self.angle_to_param(ay));
        v.push(self.angle_to_param(ay + PI));
        v.push(1.0);
        v.retain(|&mut t| inside01(t));
        v.sort_unstable_by(|a: &Coord, b: &Coord| a.partial_cmp(b).unwrap());
        arrayvec_dedup(&mut v);
        v
    }

    pub fn winding(&self) -> Coord {
        let p0 = self.delta_at(0.0);
        let p1 = self.delta_at(1.0);
        self.dt * self.radii.x * self.radii.y + self.center.cross(self.crot.rot_scale(p1 - p0))
    }

    pub fn angle_key(&self) -> AngleKey {
        let pr = self.delta_at(0.0);
        let r = self.radii;

        let dts = (1.0 as Coord).copysign(self.dt); // basically 1 * the sign of dt
        let t = (self.crot.angle() + (dts * pr.y).atan2(-dts * pr.x)).wrap_angle();
        let dt = -self.dt * r.x * r.y / pr.length_sq();
        let ddt = 2.0 * dt * (2.0 * self.t1).sin() * (r.x * r.x - r.y * r.y);
        AngleKey::new(t, dt, ddt)
    }

    pub fn intersection_x(&self, x: Coord) -> roots::Roots<Coord> {
        // Get the compensation vector and the difference
        let cp = Vec2::new(self.radii.x * self.crot.x, self.radii.y * self.crot.y);
        let diff = x - self.center.x;
        // If the difference is two large, bail out
        if diff.abs() > cp.length() { roots::Roots::No([]) }
        else {
            let acos = (diff / cp.length()).acos();
            roots::Roots::Two([self.angle_to_param(acos + cp.angle()), self.angle_to_param(-acos + cp.angle())])
        }
    }

    pub fn intersection_y(&self, y: Coord) -> roots::Roots<Coord> {
        // Get the compensation vector and the difference
        let cp = Vec2::new(self.radii.x * self.crot.y, self.radii.y * self.crot.x);
        let diff = y - self.center.y;
        // If the difference is two large, bail out
        if diff.abs() > cp.length() { roots::Roots::No([]) }
        else {
            let acos = (diff / cp.length()).acos();
            roots::Roots::Two([self.angle_to_param(acos + cp.angle()), self.angle_to_param(-acos + cp.angle())])
        }
    }

    pub fn intersection_seg(&self, v1: Vec2, v2: Vec2) -> roots::Roots<Coord> {
        // Get the compensation vector and the difference
        let dv = v2 - v1;
        let cp = Vec2::new(self.radii.x * self.crot.cross(dv), self.radii.y * self.crot.dot(dv));
        let diff = (self.center - v1).cross(dv);
        // If the difference is two large, bail out
        if diff.abs() > cp.length() { roots::Roots::No([]) }
        else {
            let acos = (diff / cp.length()).acos();
            roots::Roots::Two([self.angle_to_param(acos + cp.angle()), self.angle_to_param(-acos + cp.angle())])
        }
    }

    pub fn entry_tangent(&self) -> Vec2 { self.derivative().at(0.0).normalized() }

    pub fn exit_tangent(&self) -> Vec2 { self.derivative().at(1.0).normalized() }

    pub fn enclosing_polygon_local_space(&self) -> ArrayVec<[Vec2; MAX_POLYGON_VERTICES]> {
        // Get the possible "problematic" derivative points
        let mut plist = [0.0, 1.0, 0.0, 0.0, 0.0, 0.0];
        let mut i = 2;

        let mut first = (self.lesser_angle() / FRAC_PI_2).ceil();
        let second = (self.greater_angle() / FRAC_PI_2).floor();
        while first <= second {
            plist[i] = self.angle_to_param(first * FRAC_PI_2);
            first += 1.0;
            i += 1;
        }

        plist[0..i].sort_unstable_by(|a,b| a.partial_cmp(b).unwrap());
        let mut k = 0;
        for j in 0..i {
            if j < i-1 && plist[j].roughly_equals(plist[j+1]) { k += 1 }
            else if k > 0 { plist[j-k] = plist[j]; }
        }
        i -= k;

        let mut point_list = ArrayVec::new();
        point_list.push(self.delta_at(plist[0]));
        let d = self.derivative();

        // Add the points at ray intersections
        for j in 1..i {
            let c0 = self.delta_at(plist[j-1]);
            let d0 = d.delta_at(plist[j-1]);
            let c1 = self.delta_at(plist[j]);
            let d1 = d.delta_at(plist[j]);

            // Intersections
            let k = d0.cross(d1); // k is guaranteed to be nonzero
            let p0 = c0 + d0 * (c1 - c0).cross(d1) / k;
            let p1 = c1 + d1 * (c1 - c0).cross(d0) / k;

            // Average best estimates
            point_list.push((p0 + p1) / 2.0);
        }

        point_list.push(self.delta_at(plist[i-1]));
        point_list
    }

    pub fn enclosing_polygon(&self) -> ArrayVec<[Vec2; MAX_POLYGON_VERTICES]> {
        self.enclosing_polygon_local_space().into_iter().map(|p| self.local_to_global(p)).collect()
    }
}
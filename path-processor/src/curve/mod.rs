//--------------------------------------------------------------------
// curve.rs
//--------------------------------------------------------------------
// Provides the proxy enumeration Curve, which acts as a dispatcher
// between the known types of curves
//--------------------------------------------------------------------

use crate::derive_more::*;

mod line;
mod quadratic_bezier;
mod cubic_bezier;
mod elliptic_arc;
mod elliptic_arc_gen;
mod intersection;
mod simplification;
mod angle_key;

pub use intersection::*;
pub use simplification::*;
pub use angle_key::*;

use crate::geometry::*;
use crate::arrayvec::ArrayVec;

#[derive(Display)]
pub enum Curve {
    Line(line::Line),
    QuadraticBezier(quadratic_bezier::QuadraticBezier),
    CubicBezier(cubic_bezier::CubicBezier),
    EllipticArc(elliptic_arc::EllipticArc)
}

// Use a simplifier macro to implement the "normal" functions
// Macro idea by https://github.com/u32i64
macro_rules! forward_to_curves {
    ($($i:ident ( $($arg:ident : $arg_ty:ty),* ) -> $result:ty );*) => {
        $(
            pub fn $i(&self, $($arg : $arg_ty,)*) -> $result {
                match self {
                    Curve::Line(l) => l.$i($($arg,)*),
                    Curve::QuadraticBezier(q) => q.$i($($arg,)*),
                    Curve::CubicBezier(c) => c.$i($($arg,)*),
                    Curve::EllipticArc(a) => a.$i($($arg,)*)
                }
            }
        )*
    }
}

pub const MAX_POLYGON_VERTICES: usize = 6;
pub const MAX_CRITICAL_POINTS: usize = 6;

pub type PolygonVertices = ArrayVec<[Vec2; MAX_POLYGON_VERTICES]>;
pub type CriticalPoints = ArrayVec<[Coord; MAX_CRITICAL_POINTS]>;

impl Curve {
    // Forward the implementations that have similar signatures
    forward_to_curves! {
        at(t: Coord) -> Vec2;
        winding() -> Coord;
        angle_key() -> AngleKey;

        intersection_x(x: Coord) -> roots::Roots<Coord>;
        intersection_y(y: Coord) -> roots::Roots<Coord>;
        intersection_seg(v1: Vec2, v2: Vec2) -> roots::Roots<Coord>;

        entry_tangent() -> Vec2;
        exit_tangent() -> Vec2;

        enclosing_polygon() -> PolygonVertices;
        critical_points() -> CriticalPoints
    }

    // Derivative and subcurve are pathological cases, just forward them manually
    pub fn derivative(&self) -> Curve {
        match self {
            Curve::Line(l) => Curve::Line(l.derivative()),
            Curve::QuadraticBezier(q) => Curve::Line(q.derivative()),
            Curve::CubicBezier(c) => Curve::QuadraticBezier(c.derivative()),
            Curve::EllipticArc(a) => Curve::EllipticArc(a.derivative())
        }
    }

    pub fn subcurve(&self, l: Coord, r: Coord) -> Curve {
        match self {
            Curve::Line(ln) => Curve::Line(ln.subcurve(l, r)),
            Curve::QuadraticBezier(q) => Curve::QuadraticBezier(q.subcurve(l, r)),
            Curve::CubicBezier(c) => Curve::CubicBezier(c.subcurve(l, r)),
            Curve::EllipticArc(a) => Curve::EllipticArc(a.subcurve(l, r))
        }
    }

    pub fn reverse(&self) -> Curve {
        match self {
            Curve::Line(l) => Curve::Line(l.reverse()),
            Curve::QuadraticBezier(q) => Curve::QuadraticBezier(q.reverse()),
            Curve::CubicBezier(c) => Curve::CubicBezier(c.reverse()),
            Curve::EllipticArc(a) => Curve::EllipticArc(a.reverse())
        }
    }

    pub fn bbox(&self) -> Rect {
        Rect::enclosing_rect(self.critical_points().iter().map(|&t| self.at(t))).unwrap()
    }

    // Create curves of specific types
    pub fn line(a: Vec2, b: Vec2) -> Curve { Curve::Line(line::Line { a, b }) }
    pub fn quadratic_bezier(a: Vec2, b: Vec2, c: Vec2) -> Curve {
        Curve::QuadraticBezier(quadratic_bezier::QuadraticBezier { a, b, c })
    }
    pub fn cubic_bezier(a: Vec2, b: Vec2, c: Vec2, d: Vec2) -> Curve {
        Curve::CubicBezier(cubic_bezier::CubicBezier { a, b, c, d })
    }
    pub fn elliptic_arc(cur: Vec2, radii: Vec2, rot: Coord, large_arc: bool, sweep: bool, target: Vec2) -> Curve {
        Curve::EllipticArc(elliptic_arc_gen::from_path_params(cur, radii, rot, large_arc, sweep, target))
    }
    pub fn circle(center: Vec2, radius: Coord, v1: Vec2, v2: Vec2, ccw: bool) -> Curve {
        Curve::EllipticArc(elliptic_arc_gen::circle(center, radius, v1, v2, ccw))
    }

    // No curve
    pub fn none() -> Curve { Curve::line(Vec2::zero(), Vec2::zero()) }

    // Some utility functions
    pub fn winding_relative_to(&self, v: Vec2) -> Coord {
        self.winding() - v.cross(self.at(1.0) - self.at(0.0))
    }

    pub fn winding_at_midpoint(&self) -> Coord {
        self.winding_relative_to((self.at(0.0) + self.at(1.0)) / 2.0)
    }

    pub fn is_convex(&self) -> bool { self.winding_at_midpoint() > 0.0 }

    pub fn is_line(&self) -> bool {
        match self {
            Curve::Line(_) => true,
            _ => false
        }
    }

    #[allow(dead_code)]
    pub fn path_command(&self) -> String {
        match self {
            Curve::Line(l) => format!("M{},{} L{},{}", l.a.x, l.a.y, l.b.x, l.b.y),
            Curve::QuadraticBezier(q) => format!("M{},{} Q{},{} {},{}", q.a.x, q.a.y, q.b.x, q.b.x, q.c.x, q.c.y),
            Curve::CubicBezier(c) => format!("M{},{} C{},{} {},{} {},{}", c.a.x, c.a.y, c.b.x, c.b.y, c.c.x, c.c.y, c.d.x, c.d.y),
            _ => String::new()
        }
    }
}

impl std::clone::Clone for Curve {
    fn clone(&self) -> Self {
        match self {
            Curve::Line(l) => Curve::Line(line::Line { ..*l }),
            Curve::QuadraticBezier(q) => Curve::QuadraticBezier(quadratic_bezier::QuadraticBezier { ..*q }),
            Curve::CubicBezier(c) => Curve::CubicBezier(cubic_bezier::CubicBezier { ..*c }),
            Curve::EllipticArc(a) => Curve::EllipticArc(elliptic_arc::EllipticArc { ..*a })
        }
    }
}

impl std::fmt::Debug for Curve {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self)
    }
}

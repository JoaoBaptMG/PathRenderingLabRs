//------------------------------------------------------------------------------
// mod.rs
//------------------------------------------------------------------------------
// Provides the Path data structure, which is just a container for PathCommands
//------------------------------------------------------------------------------

mod splitting;
mod path_enums;
mod dcel;
mod fill_face;
mod subdivision_structs;
mod subdivision;
mod compiled_drawing;
mod curve_vertices;
mod triangulation;

pub use splitting::*;
pub use path_enums::*;
pub use fill_face::*;
pub use compiled_drawing::*;
pub use subdivision_structs::*;

use std::fmt::*;

use crate::geometry::*;
use crate::Curve;

// The PathCommand enum
#[derive(Clone, Copy)]
pub enum PathCommand {
    MoveTo(Vec2),
    LineTo(Vec2),
    QuadraticBezierTo(Vec2, Vec2),
    CubicBezierTo(Vec2, Vec2, Vec2),
    EllipticArcTo(Vec2, Coord, bool, bool, Vec2),
    ClosePath
}

// Implementing the display
impl Display for PathCommand {
    fn fmt(&self, f: &mut Formatter) -> Result {
        match self {
            PathCommand::MoveTo(target) => write!(f, "MoveTo({})", target),
            PathCommand::LineTo(target) => write!(f, "LineTo({})", target),
            PathCommand::QuadraticBezierTo(c, t) => write!(f, "QuadraticBezierTo({}, {})", c, t),
            PathCommand::CubicBezierTo(c1, c2, t) => write!(f, "CubicBezierTo({}, {}, {})", c1, c2, t),
            PathCommand::EllipticArcTo(radii, angle, large_arc, sweep, target)
                => write!(f, "EllipticArcTo({}, {}, {}, {}, {})", radii, angle,
                    if *large_arc { "large" } else { "small" },
                    if *sweep { "positive" } else { "negative" }, target),
            PathCommand::ClosePath => write!(f, "ClosePath()"),
        }
    }
}

// The Path is just a vector of path commands
pub type Path = Vec<PathCommand>;

// Split a path into curves and their components
pub struct CurveComp {
    pub curves: Vec<Curve>,
    pub closed: bool
}

pub fn path_to_curves(path: &Path) -> PathToCurvesIterator<'_> {
    PathToCurvesIterator { first_vec: Vec2::zero(), prev_vec: Vec2::zero(), path: path.iter() }
}

pub struct PathToCurvesIterator<'a> {
    first_vec: Vec2, prev_vec: Vec2,
    path: std::slice::Iter<'a, PathCommand>
}

impl<'a> Iterator for PathToCurvesIterator<'a> {
    type Item = CurveComp;

    fn next(&mut self) -> Option<Self::Item> {
        let mut curves = Vec::new();

        while let Some(cmd) = self.path.next() {
            match cmd {
                PathCommand::MoveTo(target) => {
                    self.first_vec = *target;
                    self.prev_vec = *target;

                    if !curves.is_empty() {
                        return Some(CurveComp { curves, closed: false });
                    }
                }
                PathCommand::LineTo(target) => {
                    curves.push(Curve::line(self.prev_vec, *target));
                    self.prev_vec = *target;
                }
                PathCommand::QuadraticBezierTo(ctl, target) => {
                    curves.push(Curve::quadratic_bezier(self.prev_vec, *ctl, *target));
                    self.prev_vec = *target;
                }
                PathCommand::CubicBezierTo(ctl1, ctl2, target) => {
                    curves.push(Curve::cubic_bezier(self.prev_vec, *ctl1, *ctl2, *target));
                    self.prev_vec = *target;
                }
                PathCommand::EllipticArcTo(radii, angle, large_arc, sweep, target) => {
                    curves.push(Curve::elliptic_arc(self.prev_vec, *radii,
                        *angle, *large_arc, *sweep, *target));
                    self.prev_vec = *target;
                }
                PathCommand::ClosePath => {
                    if self.prev_vec != self.first_vec {
                        curves.push(Curve::line(self.prev_vec, self.first_vec));
                    }

                    self.prev_vec = self.first_vec;
                    if !curves.is_empty() { 
                        return Some(CurveComp { curves, closed: true });
                    }
                }
            }
        }
    
        if !curves.is_empty() { Some(CurveComp { curves, closed: false }) }
        else { None }
    }
}
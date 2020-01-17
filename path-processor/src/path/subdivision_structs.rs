//------------------------------------------------------------------------------
// subdivision_structs.rs
//------------------------------------------------------------------------------
// Provides the structures to subdivide the generated simple faces in triangles,
// curve triangles and double curve triangles
//------------------------------------------------------------------------------

use crate::geometry::*;
use crate::derive_more::*;

#[derive(Copy, Clone, Debug, Display)]
#[display(fmt = "Triangle({},{},{})", a, b, c)]
pub struct Triangle {
    pub a: Vec2, pub b: Vec2, pub c: Vec2
}

impl Triangle {
    pub fn new(a: Vec2, mut b: Vec2, mut c: Vec2) -> Triangle {
        if (b-a).cross(c-a) < 0.0 { std::mem::swap(&mut b, &mut c) }
        Triangle { a, b, c }
    }

    pub fn is_degenerate(&self) -> bool { (self.b-self.a).cross(self.c-self.a).roughly_zero() }
}

#[derive(Copy, Clone, Debug, Constructor, Display)]
#[display(fmt = "({},{})", pos, tex)]
pub struct CurveVertex {
    pub pos: Vec2, pub tex: Vec4
}

impl CurveVertex {
    pub fn make_triangle_fan(vertices: &[CurveVertex]) -> impl Iterator<Item = CurveTriangle> + '_ {
        let len = vertices.len();
        (2..len).into_iter().map(move |i| CurveTriangle::new(vertices[0], vertices[i-1], vertices[i]))
    }
}

#[derive(Copy, Clone, Debug, Display)]
#[display(fmt = "CurveTriangle({},{},{})", a, b, c)]
pub struct CurveTriangle {
    pub a: CurveVertex, pub b: CurveVertex, pub c: CurveVertex
}

impl CurveTriangle {
    pub fn new(a: CurveVertex, mut b: CurveVertex, mut c: CurveVertex) -> CurveTriangle {
        if (b.pos-a.pos).cross(c.pos-a.pos) < 0.0 { std::mem::swap(&mut b, &mut c) }
        CurveTriangle { a, b, c }
    }

    pub fn is_degenerate(&self) -> bool { (self.b.pos-self.a.pos).cross(self.c.pos-self.a.pos).roughly_zero() }
}

#[derive(Copy, Clone, Debug, Constructor, Display)]
#[display(fmt = "({},{},{})", pos, tex0, tex1)]
pub struct DoubleCurveVertex {
    pub pos: Vec2, pub tex0: Vec4, pub tex1: Vec4, disjoint_union: bool
}

impl DoubleCurveVertex {
    pub fn make_triangle_fan(vertices: &[DoubleCurveVertex]) -> impl Iterator<Item = DoubleCurveTriangle> + '_ {
        let len = vertices.len();
        (2..len).into_iter().map(move |i| DoubleCurveTriangle::new(vertices[0], vertices[i-1], vertices[i]))
    }
}

#[derive(Copy, Clone, Debug, Display)]
#[display(fmt = "DoubleCurveTriangle({},{},{})", a, b, c)]
pub struct DoubleCurveTriangle {
    pub a: DoubleCurveVertex, pub b: DoubleCurveVertex, pub c: DoubleCurveVertex
}

impl DoubleCurveTriangle {
    pub fn new(a: DoubleCurveVertex, mut b: DoubleCurveVertex, mut c: DoubleCurveVertex) -> DoubleCurveTriangle {
        if (b.pos-a.pos).cross(c.pos-a.pos) < 0.0 { std::mem::swap(&mut b, &mut c) }
        DoubleCurveTriangle { a, b, c }
    }

    pub fn is_degenerate(&self) -> bool { (self.b.pos-self.a.pos).cross(self.c.pos-self.a.pos).roughly_zero() }
}

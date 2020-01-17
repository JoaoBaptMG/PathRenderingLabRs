//------------------------------------------------------------------------------
// curve_vertices.rs
//------------------------------------------------------------------------------
// Provides how to generate the curve vertices from a specific curve
//------------------------------------------------------------------------------

use crate::geometry::*;
use crate::curve::*;
use crate::arrayvec::*;
use super::subdivision_structs::*;

// Apply the Loop-Blinn method to get the texture coordinates
// Available on https://www.microsoft.com/en-us/research/wp-content/uploads/2005/01/p1000-loop.pdf
pub fn curve_vertices(curve: &Curve) -> ArrayVec<[CurveVertex; MAX_POLYGON_VERTICES]> {
    let sign = if curve.is_convex() { 1.0 } else { -1.0 };
    match curve {
        Curve::Line(_) => unreachable!(),
        Curve::QuadraticBezier(q) => {
            // The simplest case
            [
                CurveVertex::new(q.a, Vec4::new(0.0, 0.0, 1.0, sign)),
                CurveVertex::new(q.b, Vec4::new(0.0, 0.5, 1.0, sign)),
                CurveVertex::new(q.c, Vec4::new(1.0, 1.0, 1.0, sign))
            ].into_iter().copied().collect()
        },
        Curve::CubicBezier(c) => {
            // Use the computations in Chapter 4 of the Loop-Blinn paper
            // The canonical form of the curve
            let c3 = -c.a + 3.0 * c.b - 3.0 * c.c + c.d;
            let c2 = 3.0 * c.a - 6.0 * c.b + 3.0 * c.c;
            let c1 = -3.0 * c.a + 3.0 * c.b;

            // Again the inflection point polynomials, this time the homogeneous form
            let d3 = c1.cross(c2);
            let d2 = c3.cross(c1);
            let d1 = c2.cross(c3);

            // The texture coordinates in canonical form
            let mut f0;
            let mut f1;
            let mut f2;
            let mut f3;

            // Check for the "true" cubics first
            if d1 != 0.0 {
                // Discriminant
                let d = 3.0 * d2 * d2 - 4.0 * d3 * d1;

                // Serpentine or cusp
                if d >= 0.0 {
                    // Find the roots of the equation
                    let x1;
                    let x2;

                    let dv = (d/3.0).sqrt();
                    let dv = if d2 >= 0.0 { dv } else { -dv };
                    let q = 0.5 * (d2 + dv);

                    // Special case if q == 0
                    if q != 0.0 {
                        x1 = q / d1;
                        x2 = (d3 / 3.0) / q;
                    } else {
                        x1 = 0.0;
                        x2 = 0.0;
                    }

                    assert!(!x1.is_nan() && !x2.is_nan());

                    let l = x1.min(x2);
                    let m = x1.max(x2);

                    f0 = Vec4::new(l * m, l * l * l, m * m * m, 0.0);
                    f1 = Vec4::new(-l - m, -3.0 * l * l, -3.0 * m * m, 0.0);
                    f2 = Vec4::new(1.0, 3.0 * l, 3.0 * m, 0.0);
                    f3 = Vec4::new(0.0, -1.0, -1.0, 0.0);

                    // Guarantee that the signs are correct
                    if d1 < 0.0 {
                        f0 = -f0;
                        f1 = -f1;
                        f2 = -f2;
                        f3 = -f3;
                    }
                } else {
                    // Loop
                    // Find the roots of the equation
                    let dv = (-d).sqrt();
                    let dv = if d2 >= 0.0 { dv } else { -dv };
                    let q = 0.5 * (d2 + dv);

                    let x1 = q / d1;
                    let x2 = (d2 * d2 / d1 - d3) / q;

                    let d = x1.min(x2);
                    let e = x1.max(x2);

                    f0 = Vec4::new(d * e, d * d * e, d * e * e, 0.0);
                    f1 = Vec4::new(-d - e, -d * d - 2.0 * e * d, -e * e - 2.0 * d * e, 0.0);
                    f2 = Vec4::new(1.0, e + 2.0 * d, d + 2.0 * e, 0.0);
                    f3 = Vec4::new(0.0, -1.0, -1.0, 0.0);

                    // Guarantee that the signs are correct
                    let h1 = d3 * d1 - d2 * d2;
                    let h2 = d3 * d1 - d2 * d2 + d1 * d2 - d1 * d1;
                    let h = if h1.abs() > h2.abs() { h1 } else { h2 };
                    let h12 = d3 * d1 - d2 * d2 + d1 * d2 / 2.0 - d1 * d1 / 4.0;
                    let h = if h12.abs() > h.abs() { h12 } else { h };

                    if d1 * h > 0.0 {
                        f0 = -f0;
                        f1 = -f1;
                        f2 = -f2;
                        f3 = -f3;
                    }
                }
            } else if d2 != 0.0 {
                // Other kind of cusp
                // A single root
                let l = d3 / (3.0 * d2);

                f0 = Vec4::new(l, l * l * l, 1.0, 0.0);
                f1 = Vec4::new(-1.0, -3.0 * l * l, 0.0, 0.0);
                f2 = Vec4::new(0.0, -3.0 * l, 0.0, 0.0);
                f3 = Vec4::new(0.0, -1.0, 0.0, 0.0);
            } else if d3 != 0.0 {
                // Degenerate forms for the cubic - a quadratic
                f0 = Vec4::new(0.0, 0.0, 1.0, 0.0);
                f1 = Vec4::new(1.0, 0.0, 1.0, 0.0);
                f2 = Vec4::new(0.0, 1.0, 0.0, 0.0);
                f3 = Vec4::new(0.0, 0.0, 0.0, 0.0);
            } else { return ArrayVec::new(); }

            // Put the texture coordinates back into "normal" form
            [
                CurveVertex::new(c.a, f0),
                CurveVertex::new(c.b, f0 + f1 / 3.0),
                CurveVertex::new(c.c, f0 + (2.0 * f1 + f2) / 3.0),
                CurveVertex::new(c.d, f0 + f1 + f2 + f3)
            ].into_iter().copied().collect()
        },
        Curve::EllipticArc(a) => {
            // This case is not described in the paper, but it is easily derived
            // The texture coordinates are actually, x/a, 1 - y/b, 1 + y/b in local space
            let local_points = a.enclosing_polygon_local_space();
            let mut vertices = ArrayVec::new();
            for p in local_points {
                let kx = p.x / a.radii.x;
                let ky = p.y / a.radii.y;
                vertices.push(CurveVertex::new(a.local_to_global(p),
                    Vec4::new(kx, 1.0 - ky, 1.0 + ky, sign)));
            }
            vertices
        }
    }
}

pub fn fuse_curve_vertices(c1: &Curve, c2: &Curve) -> ArrayVec<[DoubleCurveVertex; 2 * MAX_POLYGON_VERTICES]> {
    // Check whether the two curves form a disjoint union or an intersection
    let disjoint_union = combined_windings(c1, c2) < 0.0;

    // Extract the curve vertices
    let t1 = curve_vertices(c1);
    let t2 = curve_vertices(c2);

    // Now, we are going to pick the convex hull of the polygon
    let hull = convex_hull(t1.iter().chain(t2.iter()).map(|&v| v.pos).collect());

    // Finally, calculate the new texture coordinates
    let vertices = hull.into_iter().map(|p| DoubleCurveVertex::new(p,
        coord_extrapolator(&t1, p), coord_extrapolator(&t2, p), disjoint_union));

    vertices.collect()
}

pub fn combined_windings(c1: &Curve, c2: &Curve) -> Coord {
    c1.winding() + c1.at(1.0).cross(c2.at(0.0)) + c2.winding() + c2.at(1.0).cross(c1.at(0.0))
}

fn coord_extrapolator(vertices: &[CurveVertex], x: Vec2) -> Vec4 {
    // Check for the length
    let len = vertices.len();
    match len {
        0 => Vec4::zero(),
        1 => vertices[0].tex,
        
        // Extrapolate along the line
        2 => {
            let va = vertices[0];
            let vb = vertices[1];
            let dx = vb.pos - va.pos;

            let t = (x - va.pos).dot(dx) / dx.length_sq();
            va.tex + t * (vb.tex - va.tex)
        },

        // Extrapolate along a triangle
        _ => {
            // Choose a non-zero area triangle
            let mut i = 0;
            let mut ik = 1;
            let mut ik2 = 2;

            while i < vertices.len() {
                ik = (i+1) % vertices.len();
                ik2 = (i+2) % vertices.len();

                let winding = vertices[i].pos.cross(vertices[ik].pos) +
                    vertices[ik].pos.cross(vertices[ik2].pos) +
                    vertices[ik2].pos.cross(vertices[i].pos);

                if winding.roughly_zero_squared() { break; }
                i += 1;
            }

            // If there is no nonzero-area triangle, choose the most distant vertices and pick their extrapolator
            if i == vertices.len() {
                let mut imin = 0;
                let mut imax = 0;

                for i in 1..vertices.len() {
                    if vertices[imin].pos.x > vertices[i].pos.x { imin = i; }
                    if vertices[imax].pos.x < vertices[i].pos.x { imax = i; }
                }

                coord_extrapolator(&[vertices[imin], vertices[imax]], x)
            } else {
                let a = vertices[i].pos;
                let dv1 = vertices[ik].pos;
                let dv2 = vertices[ik2].pos;
                let k = dv1.cross(dv2);

                let ta = vertices[i].tex;
                let tb = vertices[ik].tex;
                let tc = vertices[ik2].tex;

                let u = (x - a).cross(dv2) / k;
                let v = -(x - a).cross(dv1) / k;
                ta + u * (tb - ta) + v * (tc - ta)
            }
        }
    }
}

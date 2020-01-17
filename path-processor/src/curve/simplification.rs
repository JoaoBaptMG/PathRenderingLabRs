//------------------------------------------------------------------------------
// simplification.rs
//------------------------------------------------------------------------------
// Provides simplification of the curves
//------------------------------------------------------------------------------

use super::*;
use crate::geometry::CoordM::consts::FRAC_PI_2;

// Simplification of the curves
pub fn simplify_curves(curves: Vec<Curve>) -> Vec<Curve> {
    let mut new_curves = Vec::new();
    for curve in curves {
        if !is_curve_degenerate(&curve) {
            simplify_curve(&mut new_curves, &curve);
        }
    }
    new_curves.retain(|c| !is_curve_degenerate(c));
    new_curves
}

pub fn is_curve_degenerate(curve: &Curve) -> bool {
    match curve {
        Curve::Line(l) => l.a.roughly_equals(l.b),
        Curve::QuadraticBezier(q) => q.a.roughly_equals(q.b)
            && q.b.roughly_equals(q.c),
        Curve::CubicBezier(c) => c.a.roughly_equals(c.b)
            && c.b.roughly_equals(c.c)
            && c.c.roughly_equals(c.d),
        Curve::EllipticArc(a) => a.radii.roughly_zero()
    }
}

fn simplify_curve(out: &mut Vec<Curve>, curve: &Curve) {
    match curve {
        Curve::Line(l) => { out.push(Curve::line(l.a, l.b)); }
        Curve::QuadraticBezier(q) => { simplify_quadratic_bezier(out, q); }
        Curve::CubicBezier(c) => { simplify_cubic_bezier(out, c); }
        Curve::EllipticArc(a) => { simplify_elliptic_arc(out, a); }
    }
}

fn simplify_quadratic_bezier(out: &mut Vec<Curve>, q: &quadratic_bezier::QuadraticBezier) {
    // If the quadratic Bézier is roughly a line, treat it as a line
    if q.a.roughly_equals(q.b) {
        out.push(Curve::line((q.a + q.b) / 2.0, q.c));
    } else if q.b.roughly_equals(q.c) {
        out.push(Curve::line(q.a, (q.b + q.c) / 2.0));
    }
    else if (q.c - q.b).normalized().cross((q.b - q.a).normalized()).roughly_zero() {
        // Find the maximum point
        let d = q.derivative();
        let tm = d.a.x / (d.a.x - d.b.x);
        // If both are equal, we will have +inf or -inf, both will be discarded

        // If the maximum point is outside the range
        if tm < 0.0 || tm > 1.0 { out.push(Curve::line(q.a, q.c)); }
        else {
            let mi = q.at(tm);
            out.push(Curve::line(q.a, mi));
            out.push(Curve::line(mi, q.c));
        }
    } else { // Not a degenerate curve
        out.push(Curve::quadratic_bezier(q.a, q.b, q.c));
    }
}

fn simplify_cubic_bezier(out: &mut Vec<Curve>, c: &cubic_bezier::CubicBezier) {
    // If the Bézier is lines
    if c.a.roughly_equals(c.b) && c.c.roughly_equals(c.d) {
        out.push(Curve::line((c.a + c.b) / 2.0, (c.c + c.d) / 2.0));
    } else if (c.a.roughly_equals(c.b) || (c.b - c.a).normalized().cross((c.c - c.b).normalized()).roughly_zero())
        && (c.a.roughly_equals(c.b) || (c.d - c.c).normalized().cross((c.c - c.b).normalized()).roughly_zero()) {
        let roots = c.derivative().intersection_x(0.0);
        let tmp = roots.as_ref().iter().chain([0.0, 1.0].iter());
        let mut vec: Vec<_> = tmp.filter(|&&t| inside01(t)).cloned().collect();
        vec.sort_by(|a, b| a.partial_cmp(b).unwrap());
        for i in 1..vec.len() { out.push(Curve::line(c.at(vec[i-1]), c.at(vec[i]))) }
    } else if (c.a - 3.0 * c.b + 3.0 * c.c - c.d).roughly_zero() {
        // The Bézier should be a quadratic instead
        let b1 = 3.0 * c.b - c.a;
        let b2 = 3.0 * c.c - c.d;

        // Subdivide as if it were a quadratic
        out.push(Curve::quadratic_bezier(c.a, (b1 + b2) / 4.0, c.c));
    } else {
        // Detect loops, cusps and inflection points
        let mut roots = vec![0.0, 1.0];

        // Here we use the method based on https://stackoverflow.com/a/38644407
        let mut da = c.b - c.a;
        let mut db = c.c - c.b;
        let mut dc = c.d - c.c;

        // Check for no loops on cross product
        let ab = da.cross(db);
        let ac = da.cross(dc);
        let bc = db.cross(dc);
        if ac * ac <= 4.0 * ab * bc {
            // The coefficients of the canonical form
            let mut c3 = da + dc - 2.0 * db;

            // Rotate if beforehand if necessary
            if !c3.y.roughly_zero() {
                c3 = Vec2::new(c3.x, -c3.y);
                da = da.rot_scale(c3);
                db = db.rot_scale(c3);
                dc = dc.rot_scale(c3);
                c3 = da + dc - 2.0 * db;
            }

            let c2 = 3.0 * (db - da);
            let c1 = 3.0 * da;

            // Calculate the coefficients of the loop polynomial
            let bb = -c1.y / c2.y;
            let s1 = c1.x / c3.x;
            let s2 = c2.x / c3.x;

            // Find the roots (that happen to be the loop points)
            roots.extend_from_slice(roots::find_roots_quadratic(1.0, -bb, bb * (bb + s2) + s1).as_ref());
        }

        // The inflection point polynom
        let axby = c.a.x * c.b.y; let axcy = c.a.x * c.c.y; let axdy = c.a.x * c.d.y;
        let bxay = c.b.x * c.a.y; let bxcy = c.b.x * c.c.y; let bxdy = c.b.x * c.d.y;
        let cxay = c.c.x * c.a.y; let cxby = c.c.x * c.b.y; let cxdy = c.c.x * c.d.y;
        let dxay = c.d.x * c.a.y; let dxby = c.d.x * c.b.y; let dxcy = c.d.x * c.c.y;

        let k2 = axby - 2.0 * axcy + axdy - bxay + 3.0 * bxcy - 2.0 * bxdy + 2.0 * cxay - 3.0 * cxby
            + cxdy - dxay + 2.0 * dxby - dxcy;
        let k1 = -2.0 * axby + 3.0 * axcy - axdy + 2.0 * bxay - 3.0 * bxcy + bxdy - 3.0 * cxay + 3.0 * cxby + dxay - dxby;
        let k0 = axby - axcy - bxay + bxcy + cxay - cxby;

        roots.extend_from_slice(roots::find_roots_quadratic(k2, k1, k0).as_ref());

        // Sort and keep only the roots that are in the interval
        roots.retain(|&t| inside01(t));
        roots.sort_by(|a, b| a.partial_cmp(b).unwrap());
        for i in 1..roots.len() { out.push(Curve::CubicBezier(c.subcurve(roots[i-1], roots[i]))); }
    }
}

fn simplify_elliptic_arc(out: &mut Vec<Curve>, a: &elliptic_arc::EllipticArc) {
    // If the elliptic arc is a (rotated) line
    if a.radii.x.roughly_zero() || a.radii.y.roughly_zero() {
        // Find the maximum points
        let mut tests = vec![0.0, 1.0];

        let mut k = (a.lesser_angle() / FRAC_PI_2).ceil();
        let kn = (a.greater_angle() / FRAC_PI_2).floor();
        while k <= kn {
            let t = a.angle_to_param(k * FRAC_PI_2);
            if inside01(t) { tests.push(t); }
            k += 1.0;
        }

        tests.sort_by(|a, b| a.partial_cmp(b).unwrap());
        for i in 1..tests.len() { out.push(Curve::line(a.at(tests[i-1]), a.at(tests[i]))); }
    }
    else { out.push(Curve::EllipticArc(elliptic_arc::EllipticArc { ..*a })); }
}

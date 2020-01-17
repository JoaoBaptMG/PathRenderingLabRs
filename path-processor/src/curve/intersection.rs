//--------------------------------------------------------------------
// intersection.rs
//--------------------------------------------------------------------
// Provides all the intersection routines between curves
//--------------------------------------------------------------------

use crate::geometry::*;
use super::*;

#[derive(Debug)]
pub struct IntersectionPair(pub Coord, pub Coord);

pub fn intersection(curve1: &Curve, curve2: &Curve, cp1: &CriticalPoints, cp2: &CriticalPoints)
    -> Vec<IntersectionPair> {
    let mut intersections = Vec::new();

    // Check all special cases
    match (curve1, curve2) {
        (Curve::Line(line1), Curve::Line(line2)) => {
            intersection_line_line(&mut intersections, line1, line2);
        }
        (Curve::Line(line1), _) => {
            let ints = curve2.intersection_seg(line1.a, line1.b);
            for root in ints.as_ref().iter().filter(|&&t| inside01(t)) {
                let df = line1.b - line1.a;
                let pos = df.dot(curve2.at(*root) - line1.a) / df.length_sq();
                intersections.push(IntersectionPair(pos, *root));
            }
        }
        (_, Curve::Line(line2)) => {
            let ints = curve1.intersection_seg(line2.a, line2.b);
            for root in ints.as_ref().iter().filter(|&&t| inside01(t)) {
                let df = line2.b - line2.a;
                let pos = df.dot(curve1.at(*root) - line2.a) / df.length_sq();
                intersections.push(IntersectionPair(*root, pos));
            }
        }
        (_, _) => {
            intersection_generic(&mut intersections, &curve1, &curve2, &cp1, &cp2);
        }
    };

    // return it
    intersections
}

fn intersection_line_line(out: &mut Vec<IntersectionPair>, l1: &line::Line, l2: &line::Line) {
    // Check if both lines are subdividing
    let p = l1.a;
    let q = l2.a;
    let r = l1.b - l1.a;
    let s = l2.b - l2.a;

    let rr = r.normalized();
    let ss = s.normalized();

    // Intersection measure
    let k = r.cross(s);
    let kk = rr.cross(ss);

    // If the lines have the same direction
    if kk.roughly_zero_squared() {
        let rs = if rr.dot(ss) > 0.0 { (rr + ss).normalized() } else { (rr - ss).normalized() };
        
        // Check if they are collinear
        if (q - p).cross(rs).roughly_zero_squared() {
            let tab0 = (q - p).dot(r) / r.length_sq();
            let tab1 = tab0 + s.dot(r) / r.length_sq();

            let tba0 = (p - q).dot(s) / s.length_sq();
            let tba1 = tba0 + r.dot(s) / s.length_sq();

            // If there is intersection
            if 1.0 > tab0.min(tab1) && 0.0 < tab0.max(tab1) {
                // Assemble the lines accordingly (tedious cases...)
                if 0.0 <= tab0 && tab0 <= 1.0 { // l1.a -- l2.a -- l1.b, with l2.b elsewhere
                    if tab1 > 1.0 { // l2.b to the right of l1
                        out.append(&mut vec![IntersectionPair(tab0, 0.0), IntersectionPair(1.0, tba1)]);
                    } else if tab1 < 0.0 { // l2.b to the left of l1
                        out.append(&mut vec![IntersectionPair(tab0, 0.0), IntersectionPair(0.0, tba0)]);
                    } else { // l2 inside l1
                        out.append(&mut vec![IntersectionPair(tab0, 0.0), IntersectionPair(tab1, 1.0)]);
                    }
                } else if 0.0 < tab1 && tab1 <= 1.0 { // l1.a -- l2.b -- l1.b with l2.a elsewhere
                    if tab0 < 0.0 { // l2.a to the left of l1
                        out.append(&mut vec![IntersectionPair(0.0, tba0), IntersectionPair(tab1, 1.0)]);
                    } else { // l2.a to the right of l1
                        out.append(&mut vec![IntersectionPair(1.0, tba1), IntersectionPair(tab1, 1.0)]);
                    }
                } else { //l1 inside l2
                    out.append(&mut vec![IntersectionPair(0.0, tba0), IntersectionPair(1.0, tba1)]);
                }
            }
        }
    } else {
        // If they don't, calculate t and u
        let t = (q - p).cross(s) / k;
        let u = (q - p).cross(r) / k;
        out.push(IntersectionPair(t, u));
    }
}

fn intersection_generic(out: &mut Vec<IntersectionPair>, c1: &Curve, c2: &Curve,
    cp1: &CriticalPoints, cp2: &CriticalPoints) {
    for i1 in cp1.windows(2) {
        for i2 in cp2.windows(2) {
            intersection_generic_monotonous(out, c1, c2, i1[0], i1[1], i2[0], i2[1]);
        }
    }
}

fn intersection_generic_monotonous(out: &mut Vec<IntersectionPair>, c1: &Curve, c2: &Curve,
    t1l: Coord, t1r: Coord, t2l: Coord, t2r: Coord) {

    // Treat endpoints
    if c1.at(t1l) == c2.at(t2l) { out.push(IntersectionPair(t1l, t2l)); }
    if c1.at(t1l) == c2.at(t2r) { out.push(IntersectionPair(t1l, t2r)); }
    if c1.at(t1r) == c2.at(t2l) { out.push(IntersectionPair(t1r, t2l)); }
    if c1.at(t1r) == c2.at(t2r) { out.push(IntersectionPair(t1r, t2r)); }

    // Take the subcurves for the current iteration (use the fact that they are monotonous here)
    let bb1s = Rect::enclosing_rect_of_two_points(c1.at(t1l), c1.at(t1r));
    let bb2s = Rect::enclosing_rect_of_two_points(c2.at(t2l), c2.at(t2r));

    // If the bounding boxes don't intersect, neither do the curves
    if let Some(bb) = bb1s.strict_intersection(bb2s) {
        // The midpoints
        let t1m = (t1l + t1r) / 2.0;
        let t2m = (t2l + t2r) / 2.0;

        // Pick the right curves based on whether each rectangle is negligible
        fn is_rectangle_negligible(r: Rect) -> bool { 
            (r.width * 2.0).roughly_zero() && (r.height * 2.0).roughly_zero() 
        }

        let r1 = is_rectangle_negligible(bb1s);
        let r2 = is_rectangle_negligible(bb2s);

        if !r1 && !r2 {
            intersection_generic_monotonous(out, c1, c2, t1l, t1m, t2l, t2m);
            intersection_generic_monotonous(out, c1, c2, t1l, t1m, t2m, t2r);
            intersection_generic_monotonous(out, c1, c2, t1m, t1r, t2l, t2m);
            intersection_generic_monotonous(out, c1, c2, t1m, t1r, t2m, t2r);
        } else if r1 && !r2 {
            intersection_generic_monotonous(out, c1, c2, t1l, t1r, t2l, t2m);
            intersection_generic_monotonous(out, c1, c2, t1l, t1r, t2m, t2r);
        } else if !r1 && r2 {
            intersection_generic_monotonous(out, c1, c2, t1l, t1m, t2l, t2r);
            intersection_generic_monotonous(out, c1, c2, t1m, t1r, t2l, t2r);
        } else { 
            // Pick the correct root points
            // Check for the endpoints
            if bb.contains_point(c1.at(t1l)) && bb.contains_point(c2.at(t2l)) {
                out.push(IntersectionPair(t1l, t2l))
            }
            if bb.contains_point(c1.at(t1l)) && bb.contains_point(c2.at(t2r)) {
                out.push(IntersectionPair(t1l, t2r))
            }
            if bb.contains_point(c1.at(t1r)) && bb.contains_point(c2.at(t2l)) {
                out.push(IntersectionPair(t1r, t2l))
            }
            if bb.contains_point(c1.at(t1r)) && bb.contains_point(c2.at(t2r)) {
                out.push(IntersectionPair(t1r, t2r))
            }

            let curve_roots = |curve: &Curve, tl, tr| {
                let roots = [curve.intersection_x(bb.x),
                    curve.intersection_x(bb.x + bb.width),
                    curve.intersection_y(bb.y),
                    curve.intersection_y(bb.y + bb.height)];

                // Total compile tries before getting right: 20+
                let temp = roots.into_iter().flat_map(|v| v.as_ref().into_iter());
                let vec: Vec<_> = temp.filter(|&&t| t >= tl && t <= tr).copied().collect();
                if vec.is_empty() { None }
                else { 
                    let len = vec.len() as Coord;
                    Some(vec.into_iter().sum::<Coord>() / len)
                }
            };

            if let Some(r1) = curve_roots(c1, t1l, t1r) {
                if let Some(r2) = curve_roots(c2, t2l, t2r) {
                    out.push(IntersectionPair(r1, r2));
                }
            }
        }
    }
}

//------------------------------------------------------------------------------
// subdivision.rs
//------------------------------------------------------------------------------
// Provides the structures to subdivide the generated simple faces in triangles,
// curve triangles and double curve triangles
//------------------------------------------------------------------------------

use crate::geometry::*;
use crate::curve::*;
use super::FillFace;

// Check if two curves are eligible for double curve promotion
pub fn are_curves_fusable(c1: &Curve, c2: &Curve) -> bool {
    // Bail out if one of them is a line
    if c1.is_line() || c2.is_line() { return false; }

    fn eligible(c1: &Curve, c2: &Curve) -> bool {
        // 1) The curves must have a common endpoint
        if !c1.at(1.0).roughly_equals(c2.at(0.0)) { false }
        // 2) The tangents on that endpoint must be similar
        else if c1.exit_tangent().dot(c2.entry_tangent()) >= -0.99 { false }
        // 3) Both must not have the same convexity
        else if c1.is_convex() == c2.is_convex() { false }
        else { true }
    }

    // If any combination is eligible, return true
    eligible(c1, c2) || eligible(c2, c1)
}

fn avg(a: usize, b: usize) -> usize { a/2 + b/2 + (a&b&1) }

pub fn subdivide_overlapping(face: FillFace) -> FillFace {
    // An empty face subdivided is an empty face
    if face.contours.is_empty() { return face; }

    // Get the vector to store the curves
    let len = face.contours.len();
    let curves_it = face.contours.into_iter().enumerate();
    let mut curves: Vec<_> = curves_it.flat_map(|(j, contour)| {
        let radix = contour.len().leading_zeros();
        contour.into_iter().enumerate().map(move |(i, c)| (j, i << radix, (i+1) << radix, c))
    }).collect();

    // Utility function for curve subdivision
    fn subdivide_curve_in(t: Coord, curves: &mut Vec<(usize, usize, usize, Curve)>, node: usize) {
        println!("Subdivision happening!");
        let (j, begin, end, _) = curves[node];
        let mid = avg(begin, end);
        let curve = std::mem::replace(&mut curves[node].3, Curve::none());

        curves[node] = (j, begin, mid, curve.subcurve(0.0, t));
        curves.push((j, mid, end, curve.subcurve(t, 1.0)));
    }

    let mut old_len = 0;
    // Iterate for each convex-concave pair until no subdivisions are needed anymore
    while old_len != curves.len() {
        old_len = curves.len();

        for n1 in 0..old_len {
            // Skip degenerate curves
            if is_curve_degenerate(&curves[n1].3) { continue; }

            for n2 in n1+1..old_len {
                // Skip degenerate curves
                if is_curve_degenerate(&curves[n2].3) { continue; }

                // Get the curve intersection info for the pair
                // l1: Polygon 1 has points from polygon 2 in its interior
                // l2: Polygon 2 has points from polygon 1 in its interior
                // k1: Curve 1 intersects polygon 2
                // k2: Curve 2 intersects polygon 1
                if let Some((l1,l2,k1,k2)) = get_intersection_info_from_curves(&curves[n1].3, &curves[n2].3) {
                    if (!l1 && !l2 && !k1 && !k2) || (l1 && l2) || (k1 && k2) {
                        subdivide_curve_in(0.5, &mut curves, n1);
                        subdivide_curve_in(0.5, &mut curves, n2);
                    }
                    else if l1 && !k1 { subdivide_curve_in(0.5, &mut curves, n1); }
                    else if l2 && !k2 { subdivide_curve_in(0.5, &mut curves, n2); }
                    else if k1 { subdivide_curve_in(0.5, &mut curves, n2);}
                    else if k2 { subdivide_curve_in(0.5, &mut curves, n1); }
                    else { unreachable!(); }
                }
            }
        }
    }

    // Sort the curves
    curves.sort_unstable_by_key(|(j,i,_,_)| (*j,*i));

    // Check for fusable triples
    let mut i = 0;
    for j in 0..len {
        // If the current contour has less than 3 curves, continue
        if i+1 < curves.len() && curves[i].0 != curves[i+1].0 { i += 1; continue; }
        else if i+2 < curves.len() && curves[i+1].0 != curves[i+2].0 { i += 2; continue; }

        // Just a sanity check here
        assert_eq!(curves[i].0, j);
        let old_i = i;
        while i < curves.len() && curves[i].0 == j {
            // Get the previous and next node
            let ik = if i+1 < curves.len() && curves[i+1].0 == j { i+1 } else { old_i };
            let ikk = if i+2 < curves.len() && curves[i+2].0 == j { i+2 } else { old_i+1 };

            // If the triple is an eligible triple for fusion, subdivide the middle curve
            if are_curves_fusable(&curves[i].3, &curves[ik].3) && are_curves_fusable(&curves[ik].3, &curves[ikk].3) {
                subdivide_curve_in(0.5, &mut curves, ik);
                // Make sure the *second* half of the curve is there
                let len1 = curves.len()-1;
                curves.swap(ik, len1);
            }

            i += 1;
        }
    }

    // Sort again
    curves.sort_unstable_by_key(|(j,i,_,_)| (*j,*i));

    // We are also going to check whether there are eligible curves with really disproportionate windings
    // because they might cause overflow errors
    i = 0;
    for j in 0..len {
        // If the current contour has less than 2 curves, continue
        if i+1 < curves.len() && curves[i].0 != curves[i+1].0 { i += 1; continue; }

        // Just a sanity check here
        assert_eq!(curves[i].0, j);
        let old_i = i;
        while i < curves.len() && curves[i].0 == j {
            // Get the next node
            let ik = if i+1 < curves.len() && curves[i+1].0 == j { i+1 } else { old_i };

            // Only do this for fusable curves
            if are_curves_fusable(&curves[i].3, &curves[ik].3) {
                // Get the windings
                let mut winding1;
                let mut winding2 = curves[ik].3.winding_at_midpoint().abs();

                // Subdivide if one curve is much bigger than the other
                let mut t = 2.0;

                // Subdivide until the curve comes to a reasonable size
                loop {
                    t /= 2.0;
                    winding1 = curves[i].3.subcurve(1.0 - t, 1.0).winding_at_midpoint().abs();
                    if winding1 <= 32.0 * winding2 { break; }
                }

                // Bail out if the curve was subdivided
                if t < 1.0 {
                    subdivide_curve_in(1.0 - t, &mut curves, i);
                } else {
                    // Otherwise, try to subdivide the other curve
                    t = 2.0;
                    loop {
                        t /= 2.0;
                        winding2 = curves[ik].3.subcurve(0.0, t).winding_at_midpoint().abs();
                        if winding2 <= 32.0 * winding1 { break; }
                    }

                    // Subdivide the curve if necessary
                    if t < 1.0 {
                        subdivide_curve_in(t, &mut curves, ik);
                    }
                }
            }

            i += 1;
        }
    }

    // Sort a third time and extract the contours
    curves.sort_unstable_by_key(|(j,i,_,_)| (*j,*i));

    let mut contours = vec![Vec::new(); len];
    for (j,_,_,c) in curves.into_iter() {
        contours[j].push(c);
    }
    FillFace { contours }
}

fn strictly_inside_convex_polygon(poly: &[Vec2], pt: Vec2) -> bool {
    for i in 0..poly.len() {
        let ik = (i+1) % poly.len();
        if poly[i].roughly_equals(poly[ik]) { continue; }
        if ((poly[ik] - poly[i]).cross(pt - poly[i])) <= 0.0 { return false; }
    }
    true
}

fn curve_intersects(poly: &[Vec2], curve: &Curve) -> bool {
    for i in 0..poly.len() {
        let ik = (i+1) % poly.len();
        if poly[i].roughly_equals(poly[ik]) { continue; }
        let iter = curve.intersection_seg(poly[i], poly[ik]);
        if iter.as_ref().iter().any(|&t| inside01(t)) { return true; }
    }
    false
}

fn get_intersection_info_from_curves(c1: &Curve, c2: &Curve) -> Option<(bool, bool, bool, bool)> {
    // Get the curves' enclosing polygons
    let mut p1 = c1.enclosing_polygon();
    let mut p2 = c2.enclosing_polygon();

    if polygon_winding(&p1) < 0.0 { p1.reverse(); }
    if polygon_winding(&p2) < 0.0 { p2.reverse(); }

    // If there is no intersection, no info
    if !polygons_overlap(&p1, &p2, true) { None }
    else {
        let l1 = p2.iter().any(|&p| strictly_inside_convex_polygon(&p1, p));
        let l2 = p1.iter().any(|&p| strictly_inside_convex_polygon(&p2, p));
        let k1 = curve_intersects(&p2, c1);
        let k2 = curve_intersects(&p1, c2);
        Some((l1,l2,k1,k2))
    }
}

//--------------------------------------------------------------------
// polygon.rs
//--------------------------------------------------------------------
// Provides various utilitiy functions for polygons 
//--------------------------------------------------------------------

use crate::geometry::*;

fn inside_segment_collinear(x0: Vec2, x1: Vec2, y: Vec2, strict: bool) -> bool {
    let d = (x1 - x0).dot(y - x0);
    if strict { d > 0.0 && d < (x1 - x0).length_sq() }
    else { d >= 0.0 && d <= (x1 - x0).length_sq() }
}

fn segments_intersect(p0: Vec2, p1: Vec2, q0: Vec2, q1: Vec2, strict: bool) -> bool {
    // The cross products
    let crossq0 = (p1 - p0).cross(q0 - p0);
    let crossq1 = (p1 - p0).cross(q1 - p0);
    let crossp0 = (q1 - q0).cross(p0 - q0);
    let crossp1 = (q1 - q0).cross(p1 - q0);

    // If two points are equal, we have only containment (not considered in strict case)
    if p0.roughly_equals(p1) {
        return !strict && crossp0.roughly_zero_squared() && inside_segment_collinear(q0, q1, p0, strict);
    }
    if q0.roughly_equals(q1) {
        return !strict && crossq0.roughly_zero_squared() && inside_segment_collinear(p0, p1, q0, strict);
    }

    // Point coincidence is considered a false result on strict mode
    if strict {
        if p0.roughly_equals(q0) || p0.roughly_equals(q1)
            || p1.roughly_equals(q0) || p1.roughly_equals(q1) {
            return false;
        }
    }

    // Containment (not considered on strict mode)
    if crossq0.roughly_zero_squared() { return !strict && inside_segment_collinear(p0, p1, q0, strict); }
    if crossq1.roughly_zero_squared() { return !strict && inside_segment_collinear(p0, p1, q1, strict); }
    if crossp0.roughly_zero_squared() { return !strict && inside_segment_collinear(q0, q1, p0, strict); }
    if crossp1.roughly_zero_squared() { return !strict && inside_segment_collinear(q0, q1, p1, strict); }

    // Check everything is on one side
    if crossq0 < 0.0 && crossq1 < 0.0 { return false; }
    if crossq0 > 0.0 && crossq1 > 0.0 { return false; }
    if crossp0 < 0.0 && crossp1 < 0.0 { return false; }
    if crossp0 > 0.0 && crossp1 > 0.0 { return false; }

    // Otherwise...
    true
}

pub fn polygon_winding(poly: &[Vec2]) -> Coord {
    let mut winding = 0.0;

    for i in 0..poly.len() {
        let ik = (i+1) % poly.len();
        winding += poly[i].cross(poly[ik]);
    }

    winding
}

pub fn segment_equivalent(poly: &[Vec2]) -> Option<(Vec2, Vec2)> {
    // If the polygon is already a segment or its winding is non-negligible, return it or no equivalent
    if poly.len() == 2 { Some((poly[0], poly[1])) }
    else if !polygon_winding(poly.as_ref()).roughly_zero_squared() { None }
    else {
        // Else, build the segment
        let mut imin = 0;
        let mut imax = 0;

        for i in 1..poly.len() {
            if poly[imin].x > poly[i].x { imin = i; }
            if poly[imax].x < poly[i].x { imax = i; }
        }

        Some((poly[imin], poly[imax]))
    }
}

fn polygon_contains_point(poly: &[Vec2], p: Vec2, strict: bool) -> bool {
    let mut contains = false;

    for i in 0..poly.len() {
        let mut p0 = poly[i];
        let mut p1 = poly[if i == 0 { poly.len()-1 } else { i-1 }];

        // If the two points are equal, skip
        if p0.roughly_equals(p1) { continue; }

        // For strictness, if the line is "inside" the polygon, we have a problem
        if strict && (p1-p0).cross(p-p0).roughly_zero_squared()
            && inside_segment_collinear(p0, p1, p, false) { return false; }

        if p0.x < p.x && p1.x < p.x { continue; }
        if p0.x < p.x { p0 = p1 + (p.x - p1.x) / (p0.x - p1.x) * (p0 - p1); }
        if p1.x < p.x { p1 = p0 + (p.x - p0.x) / (p1.x - p0.x) * (p1 - p0); }
        if (p0.y >= p.y) != (p1.y >= p.y) { contains = !contains; }
    }

    contains
}

fn polygon_segment_intersect(poly: &[Vec2], a: Vec2, b: Vec2, strict: bool) -> bool {
    // Check for segments intersection
    for i in 0..poly.len() {
        let p0 = poly[i];
        let p1 = poly[if i == 0 { poly.len()-1 } else { i-1 }];
        if segments_intersect(p0, p1, a, b, strict) { return true; }
    }

    // Check for overlapping of the segment point
    if polygon_contains_point(poly, a, strict) || polygon_contains_point(poly, b, strict) {
        return true;
    }

    // Otherwise...
    false
}

pub fn polygons_overlap(poly0: &[Vec2], poly1: &[Vec2], strict: bool) -> bool {
    // Check first for segment polygons
    let s0 = segment_equivalent(poly0);
    let s1 = segment_equivalent(poly1);

    if let (Some((p0,p1)), Some((q0,q1))) = (s0, s1) {
        return segments_intersect(p0, p1, q0, q1, strict);
    } else if let Some((p0,p1)) = s0 {
        return polygon_segment_intersect(poly1, p0, p1, strict);
    } else if let Some((q0,q1)) = s1 {
        return polygon_segment_intersect(poly0, q0, q1, strict);
    }

    // Check for segments intersection
    for j in 0..poly0.len() {
        for i in 0..poly1.len() {
            let p0 = poly0[j];
            let p1 = poly0[if j == 0 { poly0.len()-1 } else { j-1 }];

            let q0 = poly1[i];
            let q1 = poly1[if i == 0 { poly1.len()-1 } else { i-1 }];

            if segments_intersect(p0, p1, q0, q1, strict) { return true; }
        }
    }

    // Check for overlapping of any of the points
    if poly0.iter().any(|&p| polygon_contains_point(poly1, p, strict)) { return true; }
    if poly1.iter().any(|&p| polygon_contains_point(poly0, p, strict)) { return true; }

    // Otherwise...
    false
}

pub fn convex_hull(mut points: Vec<Vec2>) -> Vec<Vec2> {
    // Sort the points using the canonical comparer
    points.sort_by(canonical);
    points.dedup();

    let mut hull = Vec::with_capacity(points.len() + 1);
    // Work with the points array forwards and backwards
    for _ in 0..2 {
        let old_len = hull.len();
        // Add the first two points
        hull.push(points[0]);
        hull.push(points[1]);

        // Run through the array
        for i in 2..points.len() {
            // Rollback the possible vertex
            while hull.len() > old_len+1 &&
                (hull[hull.len()-1] - hull[hull.len()-2]).cross(points[i] - hull[hull.len()-1]) >= 0.0 {
                hull.pop();
            }

            // Add the vertex
            hull.push(points[i]);
        }

        // Remove the last vertex
        hull.pop();
        points.reverse();
    }

    hull.reverse();
    hull
}

pub fn simplify_polygon(poly: &[Vec2]) -> Vec<Vec2> {
    // Quickly discard degenerate polygons
    if poly.len() < 3 { return poly.iter().copied().collect(); }

    // Auxiliary function to follow the same direction
    fn same_direction(u: Vec2, v: Vec2) -> bool {
        u.cross(v).roughly_zero_squared() && u.dot(v) <= 0.0
    }

    // Find a non-collinear starting vertex
    let len = poly.len();
    let mut istart = 0;
    while istart < len {
        let ik = (istart + 1) % len;
        let ip = (istart + len - 1) % len;

        if !same_direction(poly[ik] - poly[istart], poly[ip] - poly[istart]) { break; }

        istart += 1;
    }

    // If there are no non-collinear vertices, just return a line
    if istart == len {
        let mut imin = 0;
        let mut imax = 0;

        for i in 1..poly.len() {
            if poly[imin].x > poly[i].x { imin = i; }
            if poly[imax].x < poly[i].x { imax = i; }
        }

        vec![poly[imin], poly[imax]]
    } else {
        // Start with a single point
        let mut pts = vec![poly[istart]];

        // Only add the point if it doesn't form a parallel line with the next point on the line
        for i in (istart+1..len).chain(0..istart) {
            if !same_direction(poly[(i+1)%len] - poly[i], pts[pts.len()-1] - poly[i]) {
                pts.push(poly[i]);
            }
        }

        pts
    }
}

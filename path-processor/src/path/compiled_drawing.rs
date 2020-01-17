//------------------------------------------------------------------------------
// compiled_drawing.rs
//------------------------------------------------------------------------------
// A set of triangles, curve triangles and double curve triangles generated
// from a path
//------------------------------------------------------------------------------

use super::*;
use subdivision::*;
use subdivision_structs::*;
use curve_vertices::*;
use triangulation::*;

#[derive(Debug)]
pub struct CompiledDrawing {
    pub triangles: Vec<Triangle>,
    pub curve_triangles: Vec<CurveTriangle>,
    pub double_curve_triangles: Vec<DoubleCurveTriangle>
}

impl CompiledDrawing {
    pub fn concat_many(drawings: impl Iterator<Item = CompiledDrawing>) -> CompiledDrawing {
        let mut triangles = Vec::new();
        let mut curve_triangles = Vec::new();
        let mut double_curve_triangles = Vec::new();

        for mut drawing in drawings {
            triangles.append(&mut drawing.triangles);
            curve_triangles.append(&mut drawing.curve_triangles);
            double_curve_triangles.append(&mut drawing.double_curve_triangles);
        }

        CompiledDrawing { triangles, curve_triangles, double_curve_triangles }
    }

    pub fn empty() -> CompiledDrawing {
        CompiledDrawing {
            triangles: Vec::new(),
            curve_triangles: Vec::new(),
            double_curve_triangles: Vec::new()
        }
    }

    pub fn from_face(face: FillFace) -> CompiledDrawing {
        // Simplify the face by subdividing overlapping curves
        //let then = std::time::Instant::now();
        let face = subdivide_overlapping(face);
        //let now = std::time::Instant::now();
        //println!("Time spent in the subdivision: {:?}", now.duration_since(then));

        // Build the fill polygons and triangulate them
        let mut curve_triangles = Vec::new();
        let mut double_curve_triangles = Vec::new();

        let polygons = face.contours.into_iter().map(|c| {
            build_polygon_and_curves(c.as_ref(), &mut curve_triangles, &mut double_curve_triangles)
        });

        //let then = std::time::Instant::now();
        let triangles = triangulate(polygons);
        //let now = std::time::Instant::now();
        //println!("Time spent in the triangulation: {:?}", now.duration_since(then));

        CompiledDrawing { triangles, curve_triangles, double_curve_triangles }
    }
}

fn build_polygon_and_curves(contour: &[Curve], curve_triangles: &mut Vec<CurveTriangle>,
    double_curve_triangles: &mut Vec<DoubleCurveTriangle>) -> Vec<Vec2> {
    if contour.is_empty() { return Vec::new(); }
    // "Guess" a capacity for the list
    let mut list = Vec::with_capacity((1.4 * contour.len() as f64) as usize);

    // Check first if the last and first curve aren't joinable
    let last_first_join = are_curves_fusable(&contour[contour.len()-1], &contour[0]);
    if last_first_join
    { 
        list.push(contour[0].at(1.0));
        double_curve_triangles.extend(DoubleCurveVertex::make_triangle_fan(
            &fuse_curve_vertices(&contour[contour.len()-1], &contour[0])));
    } else if !contour[0].is_line() {
        curve_triangles.extend(CurveVertex::make_triangle_fan(&curve_vertices(&contour[0])));
    }

    // Now, scan the curve list for pairs of scannable curves
    let k = if last_first_join { 1 } else { 0 };
    let mut i = k;
    while i < contour.len()-k {
        if i < contour.len()-1 && are_curves_fusable(&contour[i], &contour[i+1]) {
            // Add the curve triangles
            double_curve_triangles.extend(DoubleCurveVertex::make_triangle_fan(
                &fuse_curve_vertices(&contour[i], &contour[i+1])));

            // If they describe a positive winding on the plane, add only their endpoints
            let endp0 = contour[i].at(0.0);
            let endp1 = contour[i+1].at(1.0);

            if combined_windings(&contour[i], &contour[i+1]) > 0.0 { list.push(endp1); }
            else {
                // Else, compute the convex hull and add the correct point sequence
                let iter1 = contour[i].enclosing_polygon().into_iter();
                let iter2 = contour[i+1].enclosing_polygon().into_iter();
                let hull = convex_hull(iter1.chain(iter2).collect());

                // We have to go through the hull clockwise. Find the first point
                let iv = (0..hull.len()).find(|&i| hull[i] == endp0).unwrap();

                // And run through it in reverse (iter is essentially a cycle reverse range)
                let iter = (iv+1..hull.len()).chain(0..iv+1);
                list.extend(iter.map(|i| hull[i]).rev());
            }

            i += 2;
        } else {
            // Add the single triangles
            if !contour[i].is_line() { 
                curve_triangles.extend(CurveVertex::make_triangle_fan(&curve_vertices(&contour[i])));
            }
            if contour[i].is_line() || contour[i].is_convex() { list.push(contour[i].at(1.0)); }
            else { list.extend_from_slice(&(contour[i].enclosing_polygon())[1..]) }
            i += 1;
        }
    }

    list
}


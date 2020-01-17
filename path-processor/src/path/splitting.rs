//------------------------------------------------------------------------------
// splitting.rs
//------------------------------------------------------------------------------
// Provides the function to split a generic collection of curves into their
// simple, non-connected components
//------------------------------------------------------------------------------

use crate::ordered_float::OrderedFloat;

use crate::geometry::*;
use crate::curve::*;
use super::*;
use crate::union_find::UnionFind;
use std::collections::*;

// Function to detect all the possible intersections
// TODO: add an accelerating data structure here (possibly O(n log n) in the average case?)
fn for_all_intersections<F>(curves: &[Curve], mut f: F)
    where F: FnMut(usize, usize) -> () {
    let len = curves.len();
    for i in 0..len-1 {
        for j in i+1..len {
            f(i, j);
        }
    }
}

pub fn split_comps(curves: Vec<Curve>, fill_rule: FillRule) -> Vec<FillFace> {
    // Cache the curve's critical points
    let critical_points: Vec<_> = curves.iter().map(|c| c.critical_points()).collect();

    let then = std::time::Instant::now();
    // First, get all intersection points in the curve
    let mut intersections = vec![BTreeMap::<OrderedFloat<_>, _>::new(); curves.len()];
    for_all_intersections(curves.as_slice(), |i1, i2| {
        let ints = intersection(&curves[i1], &curves[i2], &critical_points[i1], &critical_points[i2]);
        for int in ints {
            assert!(!int.0.is_nan());
            assert!(!int.1.is_nan());
            if inside01(int.0) && inside01(int.1) {
                intersections[i1].insert(int.0.into(), curves[i1].at(int.0));
                intersections[i2].insert(int.1.into(), curves[i2].at(int.1));
            }
        }
    });

    for i in 0..intersections.len() {
        // Remove all intersections outside of the range [0, 1) and add the [0,1) points
        intersections[i].insert(0.0.into(), curves[i].at(0.0));
        intersections[i].insert(1.0.into(), curves[i].at(1.0));
    }

    // Cluster the intersections and add them to the dcel
    let (clusters, num_pts) = derive_clusters(&intersections);
    let now = std::time::Instant::now();
    println!("Time spent in the intersection: {:?}", now.duration_since(then));

    //let then = std::time::Instant::now();
    let mut dcel = super::dcel::Dcel::new(num_pts);
    for (curve, cluster) in curves.into_iter().zip(clusters.into_iter()) {
        let cluster: Vec<_> = cluster.into_iter().collect();
        if cluster.len() == 2 {
            if !is_curve_degenerate(&curve) {
                dcel.add_curve(cluster[0].1, cluster[1].1, curve);
            }
        } else {
            for i in 1..cluster.len() {
                // Skip degenerate curves
                let curve = curve.subcurve(*cluster[i-1].0, *cluster[i].0);
                if !is_curve_degenerate(&curve) {
                    dcel.add_curve(cluster[i-1].1, cluster[i].1, curve);
                }
            }
        }
    }

    // Do the DCEL simplification
    dcel.remove_wedges();
    dcel.assign_face_fill_numbers();
    dcel.simplify_faces(fill_rule);
    //let now = std::time::Instant::now();
    //println!("Time spent in the DCEL: {:?}", now.duration_since(then));

    // Return the visible faces
    dcel.get_face_contours(fill_rule)
}

fn derive_clusters(intersections: &Vec<BTreeMap<OrderedFloat<Coord>, Vec2>>)
    -> (Vec<BTreeMap<OrderedFloat<Coord>, usize>>, usize) {
    // First, gather all points and create the union find
    let all_points: Vec<_> = intersections.iter().flat_map(|map| map.values()).collect();
    let mut uf = UnionFind::new(all_points.len());

    // Now, reunite the clusters
    for i in 0..all_points.len()-1 {
        for j in (i+1)..all_points.len() {
            if all_points[i].roughly_equals(*all_points[j]) {
                uf.union(i, j);
            }
        }
    }

    // "Flatten" the cluster parents
    let mut flat = HashMap::new();
    let mut max = 0;
    for i in 0..all_points.len() {
        if flat.get(&uf.find(i)).is_none() {
            flat.insert(uf.find(i), max);
            max += 1;
        }
    }

    // Finally, attribute the (flattened) clusters to the original curves
    let mut clusters = vec![BTreeMap::new(); intersections.len()];
    let mut k = 0;

    for i in 0..intersections.len() {
        for (t, _) in &intersections[i] {
            clusters[i].insert(*t, *flat.get(&uf.find(k)).unwrap());
            k += 1;
        }
    }

    (clusters, max)
}

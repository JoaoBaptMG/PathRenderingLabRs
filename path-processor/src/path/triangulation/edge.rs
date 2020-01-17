//------------------------------------------------------------------------------
// vertex.rs
//------------------------------------------------------------------------------
// The edge of the DCEL used for triangulation
//------------------------------------------------------------------------------

use crate::geometry::*;
use std::cmp::Ordering;

#[derive(Copy, Clone)]
pub struct EdgeKey {
    pub a: Vec2, pub b: Vec2
}

pub struct Edge {
    pub key: EdgeKey,
    pub helper_vertex: usize,
    pub prev: usize, pub next: usize
}

impl Edge {
    pub fn new(a: Vec2, b: Vec2) -> Edge {
        Edge { key: EdgeKey { a, b }, helper_vertex: 0, prev: 0, next: 0 }
    }
}

impl PartialEq for EdgeKey {
    fn eq(&self, other: &Self) -> bool {
        (self.a == other.a && self.b == other.b) || (self.a == other.b && self.b == other.a)
    }
}

impl Eq for EdgeKey {}

impl Ord for EdgeKey {
    fn cmp(&self, other: &Self) -> Ordering {
        // First, check for equal edges
        if self.eq(&other) { return Ordering::Equal; }

        // Build an upwards-facing edge
        let (min, max) = match canonical(&self.a, &self.b) {
            // If the edge is degenerate (i.e. if it is a vertex)
            Ordering::Equal => return other.cmp(&self).reverse(),

            Ordering::Less => (self.a, self.b),
            Ordering::Greater => (self.b, self.a)
        };

        // The edge is to the left if the points of the other edge have a
        // positive cross product with the other
        // If both points disagree, use the other comparison
        let cmp1 = (max - min).cross(other.a - min).partial_cmp(&0.0).unwrap();
        let cmp2 = (max - min).cross(other.b - min).partial_cmp(&0.0).unwrap();
        if cmp1 == Ordering::Equal { cmp2 }
        else if cmp2 == Ordering::Equal { cmp1 }
        else if cmp1 != cmp2 { other.cmp(&self).reverse() }
        else { cmp1 }
    }
}

impl PartialOrd for EdgeKey {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
//------------------------------------------------------------------------------
// vertex.rs
//------------------------------------------------------------------------------
// The vertex of the DCEL used for triangulation
//------------------------------------------------------------------------------

use crate::geometry::*;
use std::collections::BTreeMap;
use std::cmp::Ordering;
use std::ops::Bound::*;
use super::edge::EdgeKey;

#[derive(Copy, Clone, Ord, PartialOrd, Eq, PartialEq, Debug)]
pub enum VertexType { End, Start, Split, RegularLeft, RegularRight, Merge }

pub struct DcelVertex {
    pub type_: VertexType, pub cur: Vec2,
    pub outgoing: BTreeMap<EdgeKey, usize>, pub incoming: BTreeMap<EdgeKey, usize>,
    pub next_edge: usize, pub prev_edge: usize
}

#[derive(Copy, Clone)]
pub struct ChainVertex {
    pub pos: Vec2, pub type_: VertexType
}

impl DcelVertex {
    pub fn new(prev: Vec2, cur: Vec2, next: Vec2) -> DcelVertex {
        // Comparison variables
        let cp = canonical(&cur, &prev);
        let cn = canonical(&cur, &next);
        let reflex = (prev - cur).angle_between(next - cur) > 0.0;

        // Compute the type of the vertex
        let type_ = match (cp, cn) {
            // Both lie below
            (Ordering::Greater, Ordering::Greater) => if reflex { VertexType::Split } else { VertexType::Start },
            // Both lie above
            (Ordering::Less, Ordering::Less) => if reflex { VertexType::Merge } else { VertexType::End },
            // We have to take account if the vertex is on the outer or the inner contour
            (_, Ordering::Greater) => VertexType::RegularLeft,
            (_, _) => VertexType::RegularRight
        };

        DcelVertex { type_, cur, outgoing: BTreeMap::new(), incoming: BTreeMap::new(), next_edge: 0, prev_edge: 0 }
    }

    pub fn search_outgoing(&self, key: EdgeKey) -> Option<(usize, usize)> {
        if self.outgoing.is_empty() || self.outgoing.contains_key(&key) { None }
        else {
            // Try to mimic a cyclical edge list
            let mut before = self.outgoing.range((Unbounded, Excluded(key)));
            let mut after = self.outgoing.range((Excluded(key), Unbounded));

            let before = before.next_back().or(self.outgoing.iter().next_back()).unwrap();
            let after = after.next().or(self.outgoing.iter().next()).unwrap();
            Some((*before.1, *after.1))
        }
    }

    pub fn search_incoming(&self, key: EdgeKey) -> Option<(usize, usize)> {
        if self.incoming.is_empty() || self.incoming.contains_key(&key) { None }
        else {
            // Try to mimic a cyclical edge list
            let mut before = self.incoming.range((Unbounded, Excluded(key)));
            let mut after = self.incoming.range((Excluded(key), Unbounded));

            let before = before.next_back().or(self.incoming.iter().next_back()).unwrap();
            let after = after.next().or(self.incoming.iter().next()).unwrap();
            Some((*before.1, *after.1))
        }
    }
}

impl PartialEq for DcelVertex {
    fn eq(&self, other: &DcelVertex) -> bool {
        self.cur == other.cur && self.type_ == other.type_
    }
}

impl Eq for DcelVertex {}

impl Ord for DcelVertex {
    fn cmp(&self, other: &Self) -> Ordering {
        let c = canonical(&self.cur, &other.cur);
        if c == Ordering::Equal { self.type_.cmp(&other.type_) }
        else { c }
    }
}

impl PartialOrd for DcelVertex {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl PartialEq for ChainVertex {
    fn eq(&self, other: &Self) -> bool {
        self.pos == other.pos
    }
}

impl Eq for ChainVertex {}

impl Ord for ChainVertex {
    fn cmp(&self, other: &Self) -> Ordering {
        canonical(&self.pos, &other.pos)
    }
}

impl PartialOrd for ChainVertex {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
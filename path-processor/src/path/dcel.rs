//------------------------------------------------------------------------------
// dcel.rs
//------------------------------------------------------------------------------
// Provides the Doubly-Connected Edge List data structure, to build the
// simple faces from the path structure
//------------------------------------------------------------------------------

use std::collections::*;
use crate::curve::*;
use crate::geometry::*;
use std::ops::Bound::*;
use std::convert::TryInto;
use super::{FillRule, FillFace};
use crate::vec_utils::*;

#[derive(Debug)]
struct Vertex {
    out_edges: BTreeMap<AngleKey, usize>
}

impl Vertex {
    fn new() -> Vertex { Vertex { out_edges: BTreeMap::new() } }

    fn search_outgoing(&self, key: AngleKey) -> Option<(usize, usize)> {
        if self.out_edges.is_empty() || self.out_edges.contains_key(&key) { None }
        else {
            // Try to mimic a cyclical edge list
            let mut before = self.out_edges.range((Unbounded, Excluded(key)));
            let mut after = self.out_edges.range((Excluded(key), Unbounded));

            let before = before.next_back().or(self.out_edges.iter().next_back()).unwrap();
            let after = after.next().or(self.out_edges.iter().next()).unwrap();
            Some((*before.1, *after.1))
        }
    }

    fn search(&self, key: AngleKey) -> Option<usize> {
        self.out_edges.get(&key).copied()
    }
}

#[derive(Debug)]
struct Edge {
    curve: Curve,

    // Cached edge IDs
    twin: usize, next: usize, prev: usize,

    // Canonicity of the edge (how many times the edge appears on the original path)
    canonicity: isize,

    // The face the edge is in
    face: usize
}

impl Edge {
    fn new(curve: Curve, twin: usize) -> Edge {
        Edge { curve, twin, next: 0, prev: 0, canonicity: 0, face: 0 }
    }
}

#[derive(Debug)]
struct Face {
    contours: Vec<usize>,
    fill_number: isize,
    is_outer: bool
}

impl Face {
    fn new() -> Face { Face { contours: Vec::new(), fill_number: 0, is_outer: false } }
    fn outer() -> Face { Face { contours: Vec::new(), fill_number: 0, is_outer: true } }
}

#[derive(Debug)]
pub struct Dcel {
    vertices: Box<[Vertex]>,
    edges: Vec<Edge>,
    faces: Vec<Face>
}

impl Dcel {
    // Printing function
    #[cfg(feature = "debug_dcel")]
    fn print(&self) {
        println!("{:#?}", self);
        crate::pause::pause();
    }

    #[cfg(not(feature = "debug_dcel"))]
    fn print(&self) {}

    // Problematic cycles
    #[cfg(debug_assertions)]
    fn check_problematic_cycles(&self, edge: usize) {
        let mut all_edges = empty_bool_vec(self.edges.len());

        for e in self.edge_loop_iter(edge) {
            if all_edges[e] {
                panic!("Problematic edge cycle detected!");
            }
            all_edges[e] = true;
        }
    }

    #[cfg(not(debug_assertions))]
    fn check_problematic_cycles(&self, edge: usize) {}

    /// Create a new DCEL with the specified number of points
    pub fn new(num_pts: usize) -> Dcel {
        let mut verts = Vec::with_capacity(num_pts);
        for _ in 0..num_pts { verts.push(Vertex::new()); }

        Dcel {
            vertices: verts.into_boxed_slice(),
            edges: Vec::new(), faces: vec![Face::outer()]
        }
    }

    fn pair_of_edges(&mut self, curve: Curve) -> (usize, usize) {
        let len = self.edges.len();
        let rev = curve.reverse();
        self.edges.push(Edge::new(curve, len+1));
        self.edges.push(Edge::new(rev, len));
        (len, len+1)
    }

    pub fn add_curve(&mut self, v1: usize, v2: usize, curve: Curve) {
        self.add_curve_canonicity(v1, v2, curve, 1);
    }

    // This is for drain_filter down there
    pub fn add_curve_canonicity(&mut self, v1: usize, v2: usize, curve: Curve, canonicity_change: isize) {
        // Check if the vertices already have ongoing edges
        let found1 = !self.vertices[v1].out_edges.is_empty();
        let found2 = !self.vertices[v2].out_edges.is_empty();

        // Helpers
        let ak1 = curve.angle_key();
        let ak2 = curve.reverse().angle_key();
        let p0 = curve.at(0.5);

        // There are four main cases:
        // 1) The vertices are both new: we find which face they pertain and add them to the contour list
        // 2) The vertices are both existing and they connect two different contours of a face: we join those contours
        // 3) The vertices are both existing and they connect a contour of a face to itself: here, we close the face
        // 4) One of the vertices is new: here, there is not a lot of preprocessing to do

        // Cases 1) and 3) need to be treated differently if both vertices happen to be the same vertex
        // 1a) The loop will form another face, which will need to be added to the vertex's contour
        // 3a) If the loop is formed along, the edge link is set differently, and it needs to be accounted for

        // If none are found, add them individually and add a contour to the face
        if !found1 && !found2 {
            let face = self.get_face_from_point(p0);

            // The edge indices
            let (e1, e2) = self.pair_of_edges(curve);
            self.edges[e1].canonicity += canonicity_change;

            // If the vertices are different, wire them on a loop
            if v1 != v2 {
                self.edges[e1].next = e2;
                self.edges[e1].prev = e2;
                self.edges[e2].next = e1;
                self.edges[e2].prev = e1;
                self.edges[e1].face = face;
                self.edges[e2].face = face;

                // Add the edge to the contours
                self.faces[face].contours.push(e1);
            } else {
                // If they are equal, they need to be wired differently
                self.edges[e1].next = e1;
                self.edges[e1].prev = e1;
                self.edges[e2].next = e2;
                self.edges[e2].prev = e2;

                // Create a new face
                let new_face = self.faces.len();
                self.faces.push(Face::new());

                // Select the convex edge, and add it to the new face
                let (edge, twin) = if self.edges[e1].curve.winding() > 0.0 { (e1, e2) } else { (e2, e1) };
                self.faces[new_face].contours.push(edge);
                self.edges[edge].face = new_face;

                // Extract all the contours that should pertain to the new face
                let (old_contours, mut new_contours): (Vec<_>, Vec<_>) 
                    = self.faces[face].contours.iter().partition(|&&e| {
                    self.face_contains_vertex(face, self.edges[e].curve.at(0.5))
                });
                self.faces[face].contours = old_contours;

                // Add them to the new face
                for c in &new_contours { self.assign_face(new_face, *c); }
                self.faces[new_face].contours.append(&mut new_contours);

                // Add the concave edge to the outer face
                self.faces[face].contours.push(twin);
                self.edges[twin].face = face;
            }

            // Add the edges to the vertices
            self.vertices[v1].out_edges.insert(ak1, e1);
            self.vertices[v2].out_edges.insert(ak2, e2);

            self.check_problematic_cycles(e1);
            self.check_problematic_cycles(e2);
        } else if found1 && found2 {
            // If both of them are found, we create the edge and find out which shapes they are
            if let Some((e1lo, e1ro)) = self.vertices[v1].search_outgoing(ak1) {
                let (e1, e2) = self.pair_of_edges(curve);
                self.edges[e1].canonicity += canonicity_change;

                // The other matching edge is guaranteeded not to be found
                let (e2lo, e2ro) = self.vertices[v2].search_outgoing(ak2).unwrap();
                let t1ro = self.edges[e1ro].twin;
                let t2ro = self.edges[e2ro].twin;

                // Check whether the new edge will connect to different contours
                let diff_contours = self.edge_loop_iter(e1lo).find(|&e| e == e2lo).is_none();

                // WARNING: the operation above CANNOT be commuted with this below - leave the upper variable there
                // There is a special case that needs to be handled for the same vertex
                if v1 == v2 && e1lo == e2lo && e1ro == e2ro {
                    // Find the convex edge
                    let (edge, twin) = if self.edges[e1].curve.winding() > 0.0 { (e1, e2) } else { (e2, e1) };
                    self.edges[edge].next = edge;
                    self.edges[edge].prev = edge;
                    self.edges[t1ro].next = twin;
                    self.edges[e1lo].prev = twin;
                    self.edges[twin].next = e1lo;
                    self.edges[twin].prev = t1ro;
                } else {
                    // Correctly create the edge links
                    self.edges[t1ro].next = e1;
                    self.edges[e2lo].prev = e1;
                    self.edges[t2ro].next = e2;
                    self.edges[e1lo].prev = e2;
                    self.edges[e1].next = e2lo;
                    self.edges[e1].prev = t1ro;
                    self.edges[e2].next = e1lo;
                    self.edges[e2].prev = t2ro;
                }

                self.check_problematic_cycles(e1);
                self.check_problematic_cycles(e2);

                // Add the edges to the vertices
                self.vertices[v1].out_edges.insert(ak1, e1);
                self.vertices[v2].out_edges.insert(ak2, e2);

                // If the new edges were connected to different contours, fuse the contours
                if diff_contours {
                    let face = self.edges[e1lo].face;
                    self.edges[e1].face = face;
                    self.edges[e2].face = face;

                    // Create a hashset to get things faster
                    let edges = bool_vec(self.edges.len(), self.edge_loop_iter(e1));

                    // Remove the previous edge links and add a reference edge
                    self.faces[face].contours.retain(|&e| { !edges[e] });
                    self.faces[face].contours.push(e1);
                } else {
                    // The case where the edges connected the same contour is trickier
                    // First, create a face
                    let new_face = self.faces.len();
                    self.faces.push(Face::new());
                    let old_face = self.edges[e1lo].face;

                    // Remove the contours that pertained to the old edges
                    let edges = bool_vec(self.edges.len(), self.edge_loop_iter(e1).chain(self.edge_loop_iter(e2)));
                    self.faces[old_face].contours.retain(|&e| { !edges[e] });

                    // Pick the edge that forms a counterclockwise face
                    let winding: Coord = self.edge_loop_iter(e1).map(|e| self.edges[e].curve.winding()).sum();
                    let (edge, twin) = if winding > 0.0 { (e1,e2) } else { (e2, e1) };

                    // And add it to the new face
                    self.assign_face(new_face, edge);
                    self.faces[new_face].contours.push(edge);

                    // Now, pluck all the old contours that should pertain to the new face
                    let (old_contours, mut new_contours): (Vec<_>, Vec<_>) 
                        = self.faces[old_face].contours.iter().partition(|&&e| {
                        self.face_contains_vertex(old_face, self.edges[e].curve.at(0.5))
                    });
                    self.faces[old_face].contours = old_contours;

                    // Add them to the new face
                    for c in &new_contours { self.assign_face(new_face, *c); }
                    self.faces[new_face].contours.append(&mut new_contours);

                    // Put the counterclockwise edge's twin in the old face
                    self.assign_face(old_face, twin);
                    self.faces[old_face].contours.push(twin);
                }
            } else {
                // If a matching edge is found, we're done here, just up the canonicity of the edge
                let e1 = self.vertices[v1].search(ak1).unwrap();
                self.edges[e1].canonicity += canonicity_change;
            }
        } else {
            // If only one of them is found, the case is very simple
            // Create the new pair of edges and set the right canonicity
            let (e1, e2) = self.pair_of_edges(curve);
            self.edges[e1].canonicity += canonicity_change;

            // Let the new pair of edges according to the old vertex
            let (e1, e2) = if found1 { (e1, e2) } else { (e2, e1) };
            let (v1, v2) = if found1 { (v1, v2) } else { (v2, v1) };
            let (ak1, ak2) = if found1 { (ak1, ak2) } else { (ak2, ak1) };

            // Search for the adjacent edges of the new vertex
            let (e1lo, e1ro) = self.vertices[v1].search_outgoing(self.edges[e1].curve.angle_key()).unwrap();
            let t1ro = self.edges[e1ro].twin;

            // Set the vertices correctly
            self.edges[e1].prev = t1ro;
            self.edges[e1].next = e2;
            self.edges[e2].prev = e1;
            self.edges[e2].next = e1lo;
            self.edges[t1ro].next = e1;
            self.edges[e1lo].prev = e2;

            // The face is the outmost face
            self.edges[e1].face = self.edges[e1lo].face;
            self.edges[e2].face = self.edges[e1lo].face;

            // Add the edges to the vertices
            self.vertices[v1].out_edges.insert(ak1, e1);
            self.vertices[v2].out_edges.insert(ak2, e2);

            self.check_problematic_cycles(e1);
            self.check_problematic_cycles(e2);
        }

        self.print();
    }

    // The test to see if an edge is (part of) a wedge
    // Its happen if the edge's twin is on the same face and all the edges in the sequence between
    // those two also happen to have the same feature
    fn is_wedge(&self, edge: usize) -> bool {
        self.edge_loop_iter(edge).take_while(|&e| e != self.edges[e].twin)
            .all(|e| self.edges[e].face == self.edges[self.edges[e].twin].face)
    }

    pub fn remove_wedges(&mut self) {
        // Go through all the faces and all the contours for this (and damn, borrow checker!)
        for j in 0..self.faces.len() {
            // An array of indices of contours to purge, if necessary
            let mut indices = Vec::new();

            // Go through each contour in order
            'outer: for i in 0..self.faces[j].contours.len() {
                let mut set = empty_bool_vec(self.edges.len());

                // Guarantee that we will not break the cycle
                let mut e = self.faces[j].contours[i];
                while !set[e] {
                    set[e] = true;
                    // If the edge is a wedge
                    if self.is_wedge(e) {
                        // Try to find the start of the wedge
                        while self.edges[e].face == self.edges[self.edges[e].twin].face {
                            // If if the previous edge is also the twin edge, we
                            // find that the entire contour is a wedge, so we remove it
                            if self.edges[e].prev == self.edges[e].twin {
                                indices.push(e);
                                break 'outer;
                            }

                            e = self.edges[e].prev;
                        }

                        // Fix the links
                        let en = self.edges[e].next;
                        let en = self.edges[en].twin;
                        let en = self.edges[en].next;

                        self.edges[e].next = en;
                        self.edges[en].prev = e;

                        // Set the new contour beginning
                        self.faces[j].contours[i] = e;
                    }
                }
            }

            // Pluck the contours which are only edges
            self.faces[j].contours.remove_indices(indices);
        }

        self.print();
    }

    pub fn assign_face_fill_numbers(&mut self) {
        // Create the iteration queue
        let mut already_assigned_faces = empty_bool_vec(self.faces.len());
        let mut iteration_queue = VecDeque::new();

        // Add the outer face first
        let outer_face = 0;
        iteration_queue.push_back(outer_face);
        already_assigned_faces[outer_face] = true;

        // Assign fill numbers to every face
        while !iteration_queue.is_empty() {
            let face = iteration_queue.pop_front().unwrap();

            // Pass through every face and assign fill numbers (and here I am fighting the borrow checker again)
            for i in 0..self.faces[face].contours.len() {
                for e in edge_loop_iter(self.edges.as_ref(), self.faces[face].contours[i]) {
                    let t = self.edges[e].twin;
                    let twin_face = self.edges[t].face;

                    // Ignore faces already assigned
                    if already_assigned_faces[twin_face] { continue; }

                    // Assign the fill number to the face
                    let fill_number = self.faces[face].fill_number
                        - self.edges[e].canonicity + self.edges[t].canonicity;
                    self.faces[twin_face].fill_number = fill_number;
                    iteration_queue.push_back(twin_face);
                    already_assigned_faces[twin_face] = true;
                }
            }
        }

        self.print();
    }

    pub fn simplify_faces(&mut self, fill_rule: FillRule) {
        // Curiously, this code unmodified works with single edges

        // First, we are going to pass through all the edges to check which can be removed
        let mut edges_to_remove = empty_bool_vec(self.edges.len());

        for e in 0..self.edges.len() {
            // If the edge's twin is already inserted, ignore it
            let t = self.edges[e].twin;
            if edges_to_remove[t] { continue; }
            // Add it if the faces have the same predicate
            if self.face_visible(self.edges[e].face, fill_rule) ==
                self.face_visible(self.edges[t].face, fill_rule) {
                edges_to_remove[e] = false;
            }
        }

        // Now, proceed to remove the edges
        for e in (0..self.edges.len()).filter(|&i| edges_to_remove[i]) {
            // Fix the links
            let t = self.edges[e].twin;
            let ep = self.edges[e].prev;
            let en = self.edges[e].next;
            let tp = self.edges[t].prev;
            let tn = self.edges[t].next;

            if ep != t {
                self.edges[ep].next = tn;
                self.edges[tn].prev = ep;
            }

            if en != t {
                self.edges[en].prev = tp;
                self.edges[tp].next = en;
            }

            // Now, fix if necessary the contour lists
            // If they are on same face, necessarily they separate a single contour in two
            if self.edges[e].face == self.edges[t].face {
                let mut edge_set = empty_bool_vec(self.edges.len());
                if en != t { for e in self.edge_loop_iter(en) { edge_set[e] = true; } }
                if ep != t { for e in self.edge_loop_iter(ep) { edge_set[e] = true; } }

                // Remove the original contour
                let contours = &mut self.faces[self.edges[e].face].contours;
                contours.retain(|&c| !edge_set[c]);

                // Add the new contours, accounting for "wedge" edges
                if ep != t { contours.push(ep); }
                if en != t { contours.push(en); }
            } else {
                // If they are on different faces, we need to join both faces' contours somewhat
                let et = if ep != t { ep } else { en };

                // There is a single contour now
                let mut edge_set = bool_vec(self.edges.len(), self.edge_loop_iter(et));
                edge_set[e] = true;
                edge_set[t] = true;

                let mut keep_face = self.edges[e].face;
                let mut remove_face = self.edges[t].face;
                // Just make sure we don't trash the outer face by accident
                if self.faces[remove_face].is_outer { std::mem::swap(&mut keep_face, &mut remove_face); }

                // Remove the references to the possible new contour
                self.faces[keep_face].contours.retain(|&c| !edge_set[c]);
                self.faces[remove_face].contours.retain(|&c| !edge_set[c]);

                // Now, join the contours, reassigning the face, and add the new joined one
                for i in 0..self.faces[remove_face].contours.len() {
                    self.assign_face(keep_face, self.faces[remove_face].contours[i]);
                }

                // Yes, fight the borrow checker
                let mut old_contours = std::mem::replace(&mut self.faces[remove_face].contours, Vec::new());
                self.faces[keep_face].contours.append(&mut old_contours);
                self.faces[keep_face].contours.push(et);
                self.assign_face(keep_face, et);
            }
        }

        self.print();
    }

    // Get the face contours as a Vec of FillFaces
    // I should return an enumerator, but I had enough fights with the compiler
    // to give up this route
    pub fn get_face_contours(self, fill_rule: FillRule) -> Vec<FillFace> {
        let faces = (0..self.faces.len()).filter(|&fr| self.face_visible(fr, fill_rule));
        faces.map(|fr| {
            let iters = self.faces[fr].contours.iter();
            let iters = iters.map(|&c| self.edge_loop_iter(c).map(|e| &self.edges[e].curve).cloned());
            FillFace::new(iters)
        }).collect()
    }

    fn face_visible(&self, face: usize, fill_rule: FillRule) -> bool {
        match fill_rule {
            FillRule::EvenOdd => self.faces[face].fill_number % 2 != 0,
            FillRule::NonZero => self.faces[face].fill_number != 0
        }
    }

    fn get_face_from_point(&self, v: Vec2) -> usize {
        for i in 0..self.faces.len() {
            if self.face_contains_vertex(i, v) { return i; }
        }
        unreachable!()
    }

    fn edge_loop_iter(&self, edge: usize) -> EdgeLoopIterator<'_> { 
        edge_loop_iter(self.edges.as_ref(), edge)
    }

    // Cannot alter self.edges[_].next!!!!
    unsafe fn edge_loop_iter_unsafe(&self, edge: usize) -> UnsafeEdgeLoopIterator { 
        UnsafeEdgeLoopIterator { edges: self.edges.as_ptr(), length: self.edges.len(), first: edge, cur: edge }
    }

    fn face_contains_vertex(&self, face: usize, v: Vec2) -> bool {
        let face = &self.faces[face];
        let mut contains = face.is_outer;

        for contour in &face.contours {
            for e in self.edge_loop_iter(*contour) {
                // Ignore edges which interface on "blank" contours
                if self.edges[e].face == self.edges[self.edges[e].twin].face { continue; }

                let roots = self.edges[e].curve.intersection_y(v.y);
                for t in roots.as_ref() {
                    if *t >= 0.0 && *t < 1.0 && self.edges[e].curve.at(*t).x >= v.x {
                        contains = !contains;
                    }
                }
            }
        }

        return contains;
    }

    fn assign_face(&mut self, face: usize, edge: usize) {
        // Safety: this function won't alter self.edges[_].next, that is needed for the iterator
        unsafe {
            for e in self.edge_loop_iter_unsafe(edge) { self.edges[e].face = face; }
        }
    }
}

fn edge_loop_iter(edges: &[Edge], edge: usize) -> EdgeLoopIterator<'_> { 
    EdgeLoopIterator { edges: Some(edges), first: edge, cur: edge }
}

struct EdgeLoopIterator<'a> {
    edges: Option<&'a [Edge]>,
    first: usize,
    cur: usize
}

impl<'a> Iterator for EdgeLoopIterator<'a> {
    type Item = usize;

    fn next(&mut self) -> Option<usize> {
        if let Some(edges_) = self.edges {
            let cur = self.cur;
            self.cur = edges_[cur].next;
            if self.cur == self.first { self.edges = None }
            Some(cur)
        } else { None }
    }
}

struct UnsafeEdgeLoopIterator {
    edges: *const Edge,
    length: usize,
    first: usize,
    cur: usize
}

impl Iterator for UnsafeEdgeLoopIterator {
    type Item = usize;

    fn next(&mut self) -> Option<usize> {
        if !self.edges.is_null() {
            let cur = self.cur;
            if cur > self.length { panic!("Trying to access edge {} from a DCEL that has {} edges!", cur, self.length); }
            self.cur = unsafe { (*self.edges.offset(cur.try_into().unwrap())).next };
            if self.cur == self.first { self.edges = std::ptr::null(); }
            Some(cur)
        } else { None }
    }
}

fn bool_vec(sz: usize, values: impl Iterator<Item = usize>) -> Vec<bool> {
    let mut vec = empty_bool_vec(sz);
    for i in values { vec[i] = true; }
    vec
}

fn empty_bool_vec(sz: usize) -> Vec<bool> {
    vec![false; sz]
}

// Testing (couldn't that be done in another file?)
#[cfg(test)]
mod test {
    use super::*;

    fn test_edge_iter(dcel: &Dcel, edge: usize, expected: &[usize]) {
        let lp: Vec<_> = dcel.edge_loop_iter(edge).collect();
        assert!(lp.iter().eq(expected.iter()),
            "Edge {}: expected loop {:?}, but got loop {:?}", edge, expected, lp.as_slice());
    }

    fn test_faces(dcel: &Dcel, expected: usize) {
        assert!(dcel.faces.len() == expected, "Expected {} faces, but got {} faces", dcel.faces.len(), expected);
    }

    fn test_vertices(dcel: &Dcel, expected: &[usize]) {
        assert_eq!(dcel.vertices.len(), expected.len());
        for i in 0..expected.len() {
            assert!(dcel.vertices[i].out_edges.len() == expected[i],
                "Vertex {} should have {} outgoing edges, but have {} outgoing edges",
                i, expected[i], dcel.vertices[i].out_edges.len());
        }
    }

    #[test]
    fn test_simple_face() {
        // Create the DCEL
        let mut dcel = Dcel::new(3);

        // Add our first curve
        dcel.add_curve(0, 1, Curve::line(Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0)));
        test_edge_iter(&dcel, 0, &[0, 1]);
        test_faces(&dcel, 1);
        test_vertices(&dcel, &[1, 1, 0]);

        // Add our second curve
        dcel.add_curve(1, 2, Curve::line(Vec2::new(1.0, 0.0), Vec2::new(0.0, 1.0)));
        test_edge_iter(&dcel, 0, &[0, 2, 3, 1]);
        test_faces(&dcel, 1);
        test_vertices(&dcel, &[1, 2, 1]);

        // Add our third curve
        dcel.add_curve(2, 0, Curve::line(Vec2::new(0.0, 1.0), Vec2::new(0.0, 0.0)));
        test_edge_iter(&dcel, 0, &[0, 2, 4]);
        test_edge_iter(&dcel, 1, &[1, 5, 3]);
        test_faces(&dcel, 2);
        test_vertices(&dcel, &[2, 2, 2]);
    }

    #[test]
    fn test_double_face() {
        // Create the DCEL
        let mut dcel = Dcel::new(4);

        // Add our first curve
        dcel.add_curve(0, 1, Curve::line(Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0)));
        test_edge_iter(&dcel, 0, &[0, 1]);
        test_faces(&dcel, 1);
        test_vertices(&dcel, &[1, 1, 0, 0]);

        // Add our second curve
        dcel.add_curve(1, 2, Curve::line(Vec2::new(1.0, 0.0), Vec2::new(0.0, 1.0)));
        test_edge_iter(&dcel, 0, &[0, 2, 3, 1]);
        test_faces(&dcel, 1);
        test_vertices(&dcel, &[1, 2, 1, 0]);

        // Add our third curve
        dcel.add_curve(2, 0, Curve::line(Vec2::new(0.0, 1.0), Vec2::new(0.0, 0.0)));
        test_edge_iter(&dcel, 0, &[0, 2, 4]);
        test_edge_iter(&dcel, 1, &[1, 5, 3]);
        test_faces(&dcel, 2);
        test_vertices(&dcel, &[2, 2, 2, 0]);

        // Fourth curve
        dcel.add_curve(1, 3, Curve::line(Vec2::new(1.0, 0.0), Vec2::new(1.0, 1.0)));
        test_edge_iter(&dcel, 1, &[1, 5, 3, 6, 7]);
        test_faces(&dcel, 2);
        test_vertices(&dcel, &[2, 3, 2, 1]);

        // Fifth curve
        dcel.add_curve(3, 2, Curve::line(Vec2::new(1.0, 1.0), Vec2::new(0.0, 1.0)));
        test_edge_iter(&dcel, 1, &[1, 5, 9, 7]);
        test_edge_iter(&dcel, 3, &[3, 6, 8]);
        test_faces(&dcel, 3);
        test_vertices(&dcel, &[2, 3, 3, 2]);
    }

    // TODO: add tests for each of the three "simplification" routines
}
